#!/usr/bin/env python3
import asyncio
import json
import traceback
import serial                            # pyserial
from aiohttp import web
from aiortc import (
    RTCPeerConnection,
    RTCSessionDescription,
    RTCConfiguration,
    RTCIceServer
)

# ─── Configuration ──────────────────────────────────────────────────────────────
ICE_SERVERS = [
    RTCIceServer(urls=["stun:stun.l.google.com:19302"])
]
SIGNALING_PORT = 8000

# ESP32 serial settings:
ESP32_SERIAL_PORT = "/dev/ttyUSB0"
ESP32_BAUDRATE    = 115200
# ────────────────────────────────────────────────────────────────────────────────

# Globals
pcs = set()                      # track active RTCPeerConnection instances
esp32_serial = None              # will be set to a serial.Serial() object

# CORS middleware: add Access-Control-Allow-* to every response (incl. OPTIONS)
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

# POST /offer  (no trickle ICE)
async def offer(request):
    try:
        params = await request.json()
        peer_id   = params["peer_id"]
        offer_sdp = params["sdp"]
        offer_type= params["type"]

        # 1) Create a new RTCPeerConnection with proper STUN servers
        pc = RTCPeerConnection(
            configuration=RTCConfiguration(iceServers=ICE_SERVERS)
        )
        pcs.add(pc)
        pc.peer_id = peer_id

        # 2) When the browser opens DataChannel “control”:
        @pc.on("datachannel")
        def on_datachannel(channel):
            print(f"[*] DataChannel created for peer_id={peer_id}: label={channel.label}")

            @channel.on("message")
            def on_message(message):
                """
                Expecting message JSON, e.g.:
                  {"robotId":"1","action":"move","direction":"forward","timestamp":"..."}
                Map direction → single char:
                  forward  → 'w'
                  backward → 's'
                  left     → 'a'
                  right    → 'd'
                  stop     → 'x'
                Send that char + newline to ESP32 over serial.
                """
                try:
                    cmd = json.loads(message)
                except Exception as e:
                    print("[!] Invalid JSON from Browser:", e)
                    return

                direction = cmd.get("direction", "")
                if direction == "forward":
                    out_char = "w"
                elif direction == "backward":
                    out_char = "s"
                elif direction == "left":
                    out_char = "a"
                elif direction == "right":
                    out_char = "d"
                elif direction == "stop":
                    out_char = "x"
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

        # 3) We do not trickle ICE. The browser’s SDP already contains all candidates.
        @pc.on("icecandidate")
        async def on_icecandidate(event):
            pass

        # 4) Set remote description (browser’s “full‐ICE” offer)
        offer_desc = RTCSessionDescription(sdp=offer_sdp, type=offer_type)
        await pc.setRemoteDescription(offer_desc)

        # 5) Create answer (this answer SDP will include all of Pi’s ICE candidates)
        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)

        # 6) Send the answer SDP (with candidates) back to the browser
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

# Optional: GET /health-check
async def index(request):
    return web.Response(
        text="<h1>WebRTC PI Signaling Server + ESP32 Serial Forwarder</h1>",
        content_type="text/html"
    )

# Main: open serial port, then start aiohttp server
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
