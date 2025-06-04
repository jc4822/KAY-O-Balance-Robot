#!/usr/bin/env python3
import asyncio
import json
import traceback
from aiohttp import web
from aiortc import (
    RTCPeerConnection,
    RTCSessionDescription,
    RTCConfiguration,
    RTCIceServer
)

#
# Configuration
#
# Use actual RTCIceServer objects:
ICE_SERVERS = [
    RTCIceServer(urls=["stun:stun.l.google.com:19302"])
]
SIGNALING_PORT = 8000

#
# Globals
#
pcs = set()  # keep track of active RTCPeerConnections

#
# CORS middleware (same as before)
#
@web.middleware
async def cors_middleware(request, handler):
    if request.method == "OPTIONS":
        resp = web.Response(status=200)
    else:
        resp = await handler(request)

    # Always allow our UI’s origin to talk to us
    resp.headers["Access-Control-Allow-Origin"]  = "*"
    resp.headers["Access-Control-Allow-Methods"] = "POST, OPTIONS"
    resp.headers["Access-Control-Allow-Headers"] = "Content-Type"
    return resp

#
# POST /offer  (no separate /candidate anymore)
#
async def offer(request):
    try:
        params = await request.json()
        peer_id   = params["peer_id"]
        offer_sdp = params["sdp"]
        offer_type= params["type"]

        # 1) Create a new RTCPeerConnection with a proper RTCConfiguration
        pc = RTCPeerConnection(
            configuration=RTCConfiguration(iceServers=ICE_SERVERS)
        )
        pcs.add(pc)
        pc.peer_id = peer_id

        # 2) When Browser opens a DataChannel (“control”), hook it
        @pc.on("datachannel")
        def on_datachannel(channel):
            print(f"[*] DataChannel created for peer_id={peer_id}: label={channel.label}")

            @channel.on("message")
            def on_message(message):
                try:
                    cmd = json.loads(message)
                except Exception as e:
                    print("[!] Invalid JSON from Browser:", e)
                    return
                print(f"[*] Received control from browser ({peer_id}): {cmd}")
                # (In the future you could forward to ESP32 here)

            @channel.on("close")
            def on_close():
                print(f"[*] DataChannel for {peer_id} closed")

        # 3) We do NOT do any `@pc.on("icecandidate")` trickling here.
        #    Instead, we expect the SDP (in offer_desc) to already contain all ICE candidates.

        # 4) Set remote description (the “full‐ICE” offer from the browser)
        offer_desc = RTCSessionDescription(sdp=offer_sdp, type=offer_type)
        await pc.setRemoteDescription(offer_desc)

        # 5) Create our answer (which will likewise bundle all ICE candidates)
        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)

        # 6) Send the answer SDP (with candidates) back to the browser in one shot
        return web.json_response({
            "sdp": pc.localDescription.sdp,
            "type": pc.localDescription.type
        })

    except Exception as e:
        # Print a full traceback so you can see what went wrong
        traceback.print_exc()
        return web.Response(
            status=500,
            text="Server exception in /offer: " + str(e)
        )

#
# GET / is just a simple health‐check (optional)
#
async def index(request):
    return web.Response(
        text="<h1>WebRTC PI Signaling Server is running</h1>",
        content_type="text/html"
    )

#
# Main: set up the aiohttp app with CORS and routes
#
async def main():
    app = web.Application(middlewares=[cors_middleware])
    app.router.add_route("*", "/", index)
    app.router.add_route("*", "/offer", offer)

    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, "0.0.0.0", SIGNALING_PORT)
    await site.start()

    print(f"[*] Signaling server listening on http://0.0.0.0:{SIGNALING_PORT}")
    await asyncio.Event().wait()  # run forever until Ctrl‐C

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
