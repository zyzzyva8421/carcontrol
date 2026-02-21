
# pi_status_ws.py
"""
WebSocket status server for Pi car.
* Listeners (GUI) connect and receive status updates.
* Publishers (auto_avoid.py) connect and send status updates.
All status messages received from any client are broadcast to all listeners.
"""
import asyncio
import websockets
import json

LISTENERS = set()

async def status_server(websocket, path):
    # By default, treat all clients as listeners
    LISTENERS.add(websocket)
    try:
        async for message in websocket:
            # Any message received is treated as a status update to broadcast
            try:
                json.loads(message)  # Validate JSON
                await broadcast_to_listeners(message)
            except Exception:
                pass
    finally:
        LISTENERS.discard(websocket)

async def broadcast_to_listeners(msg):
    for ws in list(LISTENERS):
        try:
            await ws.send(msg)
        except Exception:
            LISTENERS.discard(ws)

if __name__ == "__main__":
    start_server = websockets.serve(status_server, "0.0.0.0", 9000)
    asyncio.get_event_loop().run_until_complete(start_server)
    print("[pi_status_ws] WebSocket status server running on port 9000")
    asyncio.get_event_loop().run_forever()
