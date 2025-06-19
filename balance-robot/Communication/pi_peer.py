#!/usr/bin/env python3
import asyncio
import json
import subprocess
import traceback
import serial
import av
import time
import threading
from queue import Queue
from aiohttp import web
import adafruit_vl53l0x
import board
import busio
import numpy as np

from aiortc import (
    RTCPeerConnection,
    RTCSessionDescription,
    RTCConfiguration,
    RTCIceServer,
    VideoStreamTrack,
)
from av import VideoFrame

# 全局变量初始化
control_channel = None
i2c = busio.I2C(3, 2)
sensor = adafruit_vl53l0x.VL53L0X(i2c)
print(f"Sensor initialized: {sensor}")

# 配置参数
ICE_SERVERS = [RTCIceServer(urls=["stun:stun.l.google.com:19302"])]
SIGNALING_PORT = 8000
ESP32_SERIAL_PORT = "/dev/ttyUSB0"
ESP32_BAUDRATE = 115200

# 全局状态
pcs = set()
esp32_serial = None
past_pos_x = 0.0
past_pos_y = 0.0
past_yaw = 0.0
serial_lock = asyncio.Lock()

class LibcameraH264Track(VideoStreamTrack):

    def __init__(self, width=640, height=480, fps=15):
        super().__init__()
        self.frame_id = 0
        self.width = width
        self.height = height
        self.fps = fps
        
        # 启动libcamera-vid子进程
        cmd = [
            "libcamera-vid",
            "-t", "0",
            "--inline",
            "--width", str(width),
            "--height", str(height),
            "--framerate", str(fps),
            "-o", "-"
        ]
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)

        self.container = av.open(self.proc.stdout, format="h264", mode="r")
        self.video_stream = self.container.streams.video[0]

        self.frame_queue = Queue(maxsize=10)  # 限制队列大小防止内存溢出
        self.decoder_thread = threading.Thread(target=self._decode_frames, daemon=True)
        self.decoder_thread.start()
        self.running = True

    def _decode_frames(self):
        try:
            for frame in self.container.decode(video=0):
                if not self.running:
                    break
                # 转换为RGB24格式
                rgb_frame = frame.to_ndarray(format="rgb24")
                self.frame_queue.put(rgb_frame)
        except Exception as e:
            print(f"Video decoder error: {e}")
        finally:
            self.container.close()
            self.proc.kill()

    async def recv(self):
        self.frame_id += 1
        capture_ts = int(time.time() * 1000)
        if control_channel and control_channel.readyState == "open":
            control_channel.send(json.dumps({
                "frame_id": self.frame_id,
                "capture_ts": capture_ts
            }))

        pts, time_base = await self.next_timestamp()

        try:
            rgb_array = await asyncio.get_event_loop().run_in_executor(
                None, 
                self.frame_queue.get, 
                True,
                2.0
            )
        except Exception as e:
            print(f"Frame queue error: {e}")
            rgb_array = np.zeros((self.height, self.width, 3), dtype="uint8")


        video_frame = VideoFrame.from_ndarray(rgb_array, format="rgb24")
        video_frame.pts = pts
        video_frame.time_base = time_base
        return video_frame

    def stop(self):
        self.running = False
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

# CORS中间件
@web.middleware
async def cors_middleware(request, handler):
    if request.method == "OPTIONS":
        resp = web.Response(status=200)
    else:
        resp = await handler(request)

    resp.headers["Access-Control-Allow-Origin"] = "*"
    resp.headers["Access-Control-Allow-Methods"] = "POST, OPTIONS"
    resp.headers["Access-Control-Allow-Headers"] = "Content-Type"
    return resp

async def read_serial_data():

    global past_pos_x, past_pos_y, past_yaw
    
    if not esp32_serial or not esp32_serial.in_waiting:
        return past_pos_x, past_pos_y, past_yaw
    
    try:
        # 异步读取串口数据
        rawdata = await asyncio.get_event_loop().run_in_executor(
            None,
            esp32_serial.read,
            esp32_serial.in_waiting
        )
        
        try:
            sensor_data = json.loads(rawdata.decode('utf-8').strip())
            if isinstance(sensor_data, dict):
                past_pos_x = sensor_data.get("x", past_pos_x)
                past_pos_y = sensor_data.get("y", past_pos_y)
                past_yaw = sensor_data.get("a", past_yaw)
        except (json.JSONDecodeError, ValueError, UnicodeDecodeError):
            pass
            
    except Exception as e:
        print(f"Serial read error: {e}")
    
    return past_pos_x, past_pos_y, past_yaw

async def read_sensor_data():
    try:
        return await asyncio.get_event_loop().run_in_executor(
            None, 
            lambda: sensor.range
        )
    except Exception as e:
        print(f"Sensor read error: {e}")
        return 0

async def telemetry_task(channel):

    #print("[*] Starting telemetry task")
    while channel.readyState == "open":
        start_time = time.monotonic()
        
        try:
            depth_future = read_sensor_data()
            pos_future = read_serial_data()
            
            depth = await depth_future
            pos_x, pos_y, yaw = await pos_future

            telemetry = {
                "battery": 100,
                "pidP": 0.5,
                "pidI": 0.1,
                "pidD": 0.2,
                "pos_x": pos_x,
                "pos_y": pos_y,
                "yaw": yaw,
                "depth": depth,
                "depthTs": int(time.time() * 1000)
            }
            
            if channel.readyState == "open":
                channel.send(json.dumps(telemetry))
        except Exception as e:
            print(f"Telemetry error: {e}")
        
        elapsed = time.monotonic() - start_time
        await asyncio.sleep(max(0.05 - elapsed, 0))
    
    print("[*] Telemetry task stopped")

async def handle_control_message(message, channel):
    try:
        msg = json.loads(message)

        if msg.get("type") == "ping":
            channel.send(json.dumps({"type": "pong", "ts": msg["ts"]}))
            return

        if msg.get("type") == "sync_req":
            channel.send(json.dumps({
                "type": "sync_res",
                "ts_client": msg["ts_client"],
                "ts_server": int(time.time() * 1000)
            }))
            return
        
        direction = msg.get("direction", "")
        if direction in ["forward", "backward", "left", "right", "stop"]:
            char_map = {
                "forward": "w",
                "backward": "s",
                "left": "a",
                "right": "d",
                "stop": "x"
            }
            await send_serial_command(char_map[direction])
    except Exception as e:
        print(f"Control message error: {e}")

async def send_serial_command(command_char):

    async with serial_lock:
        if not esp32_serial:
            print("[!] Serial port not open")
            return
            
        try:
            esp32_serial.write(f"{command_char}\n".encode())
            print(f"[>] Sent to ESP32: {command_char}")
        except Exception as e:
            print(f"[!] Serial write failed: {e}")

async def offer(request):
    try:
        params = await request.json()
        peer_id = params["peer_id"]
        offer_sdp = params["sdp"]
        offer_type = params["type"]

        pc = RTCPeerConnection(configuration=RTCConfiguration(iceServers=ICE_SERVERS))
        pcs.add(pc)
        pc.peer_id = peer_id

        telemetry_future = None

        @pc.on("datachannel")
        def on_datachannel(channel):
            nonlocal telemetry_future
            print(f"[*] DataChannel created: {channel.label}")
            global control_channel
            control_channel = channel

            @channel.on("message")
            def on_message(message):
                asyncio.create_task(handle_control_message(message, channel))

            @channel.on("close")
            def on_close():
                print(f"[*] DataChannel closed for {peer_id}")
                if telemetry_future and not telemetry_future.done():
                    telemetry_future.cancel()
                if control_channel == channel:
                    control_channel = None

            telemetry_future = asyncio.create_task(telemetry_task(channel))

        @pc.on("connectionstatechange")
        async def on_connectionstatechange():
            state = pc.connectionState
            print(f"[*] Connection state changed to '{state}'")
            if state in ["failed", "disconnected", "closed"]:
                await pc.close()
                pcs.discard(pc)

        @pc.on("icecandidate")
        async def on_icecandidate(event):
            pass

        offer_desc = RTCSessionDescription(sdp=offer_sdp, type=offer_type)
        await pc.setRemoteDescription(offer_desc)

        camera_track = LibcameraH264Track(width=640, height=480, fps=15)
        pc.addTrack(camera_track)
        print("[*] Added LibcameraH264Track")

        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)

        return web.json_response({
            "sdp": pc.localDescription.sdp,
            "type": pc.localDescription.type
        })

    except Exception as e:
        traceback.print_exc()
        return web.Response(
            status=500,
            text=f"Server exception: {str(e)}"
        )

async def index(request):
    return web.Response(
        text="<h1>WebRTC Robot Controller</h1>"
             "<p>Video streaming and telemetry service</p>",
        content_type="text/html"
    )

async def main():
    global esp32_serial

    try:
        esp32_serial = serial.Serial(
            port=ESP32_SERIAL_PORT,
            baudrate=ESP32_BAUDRATE,
            timeout=0.1
        )
        print(f"[*] ESP32 serial connected: {ESP32_SERIAL_PORT} @ {ESP32_BAUDRATE} baud")
    except Exception as e:
        print(f"[!] Failed to open serial port: {e}")
        esp32_serial = None

    app = web.Application(middlewares=[cors_middleware])
    app.router.add_route("*", "/", index)
    app.router.add_route("POST", "/offer", offer)

    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, "0.0.0.0", SIGNALING_PORT)
    await site.start()
    print(f"[*] Signaling server running on http://0.0.0.0:{SIGNALING_PORT}")

    await asyncio.Event().wait()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[*] Server shutdown")