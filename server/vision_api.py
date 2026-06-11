from fastapi import APIRouter
from fastapi.responses import JSONResponse, FileResponse
import asyncio
import json
import zmq
import zmq.asyncio
from sse_starlette.sse import EventSourceResponse
import socket

router = APIRouter()

VISION_HOST = "127.0.0.1"
VISION_PORT = 9011

VISION_CMD_MAP = {
    "ping"   : "PING",
    "start"  : "START",
    "stop"   : "STOP",
    "status" : "STATUS",
}

def send_vision(cmd: str, timeout=5.0) -> str:
    with socket.create_connection((VISION_HOST, VISION_PORT), timeout=timeout) as s:
        s.sendall((cmd + "\n").encode())
        data = s.recv(4096)
    return data.decode(errors="ignore").strip()

@router.get("/vision_test")
async def vision_test_page():
    return FileResponse("/opt/projects/robotour/server/static/vision_test.html")

@router.get("/vision_stream")
async def vision_stream():
    async def event_generator():
        context = zmq.asyncio.Context.instance()
        sub = context.socket(zmq.SUB)
        sub.setsockopt(zmq.CONFLATE, 1)
        sub.connect("ipc:///tmp/robot-vision")
        sub.setsockopt_string(zmq.SUBSCRIBE, "")
        
        try:
            print("🟢 [Vision API] Připojeno k ZMQ, čekám na zprávy...")
            while True:
                # Čekáme na data z kamery
                msg = await sub.recv_string()
                print(f"📥 [Vision API] Přijata zpráva (délka: {len(msg)})")
                
                try:
                    topic, json_str = msg.split("/", 1)
                    # json_str obsahuje: {"time": ..., "side": ..., "frame": ..., "pose": [...]}
                    # Rovnou přepošleme klientovi přes SSE
                    yield f"data: {json_str}\n\n"
                except Exception as e:
                    print(f"⚠️ [Vision API] Chyba parsování JSONu: {e}")
                    
        except asyncio.CancelledError:
            print("🛑 [Vision API] SSE klient odpojen")
        finally:
            sub.close()
            
    return EventSourceResponse(event_generator())

@router.get("/vision/{action}")
async def vision_action(action: str):
    action = action.lower()
    if action not in VISION_CMD_MAP:
        return JSONResponse(status_code=400, content={"error":"bad action"})
    try:
        resp = await asyncio.to_thread(send_vision, VISION_CMD_MAP[action])
        return {"action":action, "response":resp}
    except Exception as e:
        return JSONResponse(status_code=500, content={"error":str(e)})

