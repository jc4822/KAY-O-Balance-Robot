#!/usr/bin/env python3
import asyncio
import json
import subprocess
import traceback
import serial               # pyserial for ESP32 communication
import av                   # PyAV for H.264 decoding
from aiohttp import web

from aiortc import (
    RTCPeerConnection,
    RTCSessionDescription,
    RTCConfiguration,
    RTCIceServer,
    VideoStreamTrack,
)
from av import VideoFrame   # import from PyAV

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

# POST /offer
async def offer(request):
    try:
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

            @channel.on("message")
            def on_message(message):
                # Expect JSON like: {"direction":"forward"}
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
                try:
                    if esp32_serial:
                        esp32_serial.write(serial_line.encode("utf-8"))
                        print(f"[*] Sent to ESP32: '{out_char}'")
                    else:
                        print("[!] ESP32 serial not open; cannot send command")
                except Exception as e:
                    print("[!] Failed to write to ESP32 serial:", e)

            @channel.on("close")
            def on_close():
                print(f"[*] DataChannel for {peer_id} closed")

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

# Main: open ESP32 serial
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
