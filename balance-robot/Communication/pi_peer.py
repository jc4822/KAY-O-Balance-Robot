#!/usr/bin/env python3
import asyncio
import json
import subprocess
import traceback
import serial               # pyserial for ESP32 communication
import av                   # PyAV for H.264 decoding
import time
from aiohttp import web
import adafruit_vl53l0x
import board
import busio

from aiortc import (
    RTCPeerConnection,
    RTCSessionDescription,
    RTCConfiguration,
    RTCIceServer,
    VideoStreamTrack,
)
from av import VideoFrame   # import from PyAV

control_channel = None

i2c = busio.I2C(3, 2)
print("I2C initialised")
sensor = adafruit_vl53l0x.VL53L0X(i2c)
print("sensor name: ", sensor)

ICE_SERVERS = [
    RTCIceServer(urls=["stun:stun.l.google.com:19302"])
]
SIGNALING_PORT = 8000

# ESP32 serial settings
ESP32_SERIAL_PORT = "/dev/ttyUSB0"
ESP32_BAUDRATE    = 115200

pcs = set()                      # track active PeerConnections
esp32_serial = None              # will be set to serial.Serial()

class LibcameraH264Track(VideoStreamTrack):
    """
    A VideoStreamTrack that runs `libcamera-vid` as a subprocess in H.264 mode,
    decodes via PyAV, and yields frames to aiortc.
    """

    def __init__(self, width=640, height=480, fps=15):
        super().__init__()  # initialize base class
        self.frame_id = 0
        cmd = [
            "libcamera-vid",
            "-t", "0",
            "--inline",
            "--width", str(width),
            "--height", str(height),
            "--framerate", str(fps),
            "-o", "-"  # send raw H.264 to stdout
        ]

        # Spawn subprocess; stdout is a PIPE containing H.264 bitstream
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)

        # Open a PyAV container on the subprocess's stdout, telling PyAV it's raw H.264
        self.container = av.open(self.proc.stdout, format="h264", mode="r")

        # The H.264 stream is at index 0
        self.video_stream = self.container.streams.video[0]

        # Create a generator to decode frames
        self.frame_iterator = self.container.decode(video=0)

    async def recv(self):
        self.frame_id += 1
        capture_ts = int(time.time() * 1000)
        if control_channel and control_channel.readyState == "open":
            control_channel.send(json.dumps({
                "frame_id":   self.frame_id,
                "capture_ts": capture_ts
            }))

        pts, time_base = await self.next_timestamp()

        av_frame = next(self.frame_iterator, None)
        if av_frame is None:
            # If the subprocess ended, send a black frame
            w = self.video_stream.width
            h = self.video_stream.height
            import numpy as np
            black = np.zeros((h, w, 3), dtype="uint8")
            video_frame = VideoFrame.from_ndarray(black, format="rgb24")
            video_frame.pts = pts
            video_frame.time_base = time_base
            return video_frame

        # Convert av.VideoFrame (YUV) → RGB numpy array
        img = av_frame.to_ndarray(format="rgb24")
        video_frame = VideoFrame.from_ndarray(img, format="rgb24")
        video_frame.pts = pts
        video_frame.time_base = time_base
        return video_frame

    def stop(self):
        try:
            if self.proc:
                self.proc.kill()
        except Exception:
            pass
        try:
            if self.container:
                self.container.close()
        except Exception:
            pass
        super().stop()

# CORS middleware (allow UI on :8080 to POST to :8000)
@web.middleware
async def cors_middleware(request, handler):
    if request.method == "OPTIONS":
        resp = web.Response(status=200)
    else:
        resp = await handler(request)

    resp.headers["Access-Control-Allow-Origin"]  = "*"
    resp.headers["Access-Control-Allow-Methods"] = "POST, OPTIONS"
    resp.headers["Access-Control-Allow-Headers"] = "Content-Type"
    return resp

past_pos_x = 0.0
past_pos_y = 0.0
past_pos_y = 0.0

serial_lock = asyncio.Lock()

# POST /offer (no‐trickle ICE)
async def offer(request):
    try:
        PiToWeb_task = None
        params     = await request.json()
        peer_id    = params["peer_id"]
        offer_sdp  = params["sdp"]
        offer_type = params["type"]

        # 1) Create a new RTCPeerConnection
        pc = RTCPeerConnection(configuration=RTCConfiguration(iceServers=ICE_SERVERS))
        pcs.add(pc)
        pc.peer_id = peer_id

        # 2) DataChannel handler (in case the browser’s offer already included it)
        @pc.on("datachannel")
        def on_datachannel(channel):
            print(f"[*] DataChannel created for peer_id={peer_id}: label={channel.label}")
            global control_channel
            control_channel = channel

            async def PiToWeb():
                while channel.readyState == "open":
                    d = sensor.range
                    if esp32_serial.in_waiting:
                        try:

                            rawdata = esp32_serial.read(esp32_serial.in_waiting)

                            try:
                                sensor_data = json.loads(rawdata.decode('utf-8').strip())
                                if not isinstance(sensor_data, dict):
                                    raise ValueError("Parsed JSON is not a dictionary")
                            except (json.JSONDecodeError, ValueError, UnicodeDecodeError) as e:
                                sensor_data = {"x": past_pos_x, "y": past_pos_y, "a": past_yaw}


                            past_pos_x = sensor_data.get("x", 0)
                            past_pos_y = sensor_data.get("y", 0)
                            past_yaw = sensor_data.get("a", 0)
                            
                            telemetry = {
                                "battery": 100,
                                "pidP": 0.5,
                                "pidI": 0.1,
                                "pidD": 0.2,
                                "pos_x": sensor_data.get("x", 0),
                                "pos_y": sensor_data.get("y", 0),
                                "yaw": sensor_data.get("a", 0),
                                "depth": d,
                                "depthTs": time.time() * 1000
                            }
                            channel.send(json.dumps(telemetry))
                        except Exception as e:
                            print(f"[!] data error: {e}")
                    
                    await asyncio.sleep(0.05)
                    
            PiToWeb_task = asyncio.create_task(PiToWeb())

            @channel.on("message")
            def on_message(message):
                msg = json.loads(message)
                if msg.get("type") == "ping":
                    channel.send(json.dumps({
                        "type": "pong",
                        "ts": msg["ts"]
                    }))
                    return
                # ── clock sync request ──
                if msg.get("type") == "sync_req":
                    channel.send(json.dumps({
                        "type":      "sync_res",
                        "ts_client": msg["ts_client"],
                        "ts_server": int(time.time() * 1000)
                    }))
                    return
                
                try:
                    cmd = json.loads(message)
                except Exception as e:
                    print("[!] Invalid JSON from Browser:", e)
                    return

                direction = cmd.get("direction", "")
                if   direction == "forward":  out_char = "w"
                elif direction == "backward": out_char = "s"
                elif direction == "left":     out_char = "a"
                elif direction == "right":    out_char = "d"
                elif direction == "stop":     out_char = "x"
                else:
                    print(f"[!] Unknown direction from browser: {direction}")
                    return

                serial_line = out_char + "\n"

                async def send_serial_command():
                    async with serial_lock:
                        if not esp32_serial:
                            print("[!] Serial port not open")
                            return
                            
                        try:
                            esp32_serial.write(f"{out_char}\n".encode())
                            print(f"[>] Sent to ESP32: {out_char}")
                        except Exception as e:
                            print(f"[!] Serial write failed: {e}")

                asyncio.create_task(send_serial_command())

            @channel.on("close")
            def on_close():
                print(f"[*] DataChannel for {peer_id} closed")
                if PiToWeb_task:
                    PiToWeb_task.cancel()

        @pc.on("connectionstatechange")
        async def on_connectionstatechange():
            print(f"Connection state is {pc.connectionState}")
            if pc.connectionState == "failed" or pc.connectionState == "disconnected" or pc.connectionState == "closed":
                await pc.close()
                pcs.discard(pc)

        # 3) Stub out ICE‐candidate callback (no‐trickle ICE)
        @pc.on("icecandidate")
        async def on_icecandidate(event):
            pass

        # 4) Set the browser’s SDP as our remote description
        offer_desc = RTCSessionDescription(sdp=offer_sdp, type=offer_type)
        await pc.setRemoteDescription(offer_desc)

        # 5) Add the PiCamera track (libcamera-vid → H.264 → PyAV → aiortc)
        camera_track = LibcameraH264Track(width=640, height=480, fps=15)
        pc.addTrack(camera_track)
        print("[*] Added LibcameraH264Track (libcamera-vid → H.264 → PyAV)")

        # 6) Create & set our answer SDP (it will include m=application + m=video sendonly)
        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)

        # 7) Return the answer SDP (with ICE candidates + video) back to the browser
        return web.json_response({
            "sdp": pc.localDescription.sdp,
            "type": pc.localDescription.type
        })

    except Exception as e:
        traceback.print_exc()
        return web.Response(
            status=500,
            text="Server exception in /offer: " + str(e)
        )

async def index(request):
    return web.Response(
        text="<h1>WebRTC PI + ESP32 + libcamera-vid (addTrack after recvonly)</h1>",
        content_type="text/html"
    )

# Main: open ESP32 serial, then start aiohttp server
async def main():

    global esp32_serial
    try:
        esp32_serial = serial.Serial(
            port=ESP32_SERIAL_PORT,
            baudrate=ESP32_BAUDRATE,
            timeout=0.1
        )
        print(f"[*] Opened ESP32 serial on {ESP32_SERIAL_PORT} @ {ESP32_BAUDRATE} baud")
    except Exception as e:
        print(f"[!] ERROR: could not open ESP32 serial on {ESP32_SERIAL_PORT}:", e)
        esp32_serial = None

    app = web.Application(middlewares=[cors_middleware])
    app.router.add_route("*", "/", index)
    app.router.add_route("*", "/offer", offer)

    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, "0.0.0.0", SIGNALING_PORT)
    await site.start()
    print(f"[*] Signaling server listening on http://0.0.0.0:{SIGNALING_PORT}")
    await asyncio.Event().wait()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
