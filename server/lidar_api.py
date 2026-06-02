from fastapi import APIRouter
from fastapi.responses import JSONResponse, FileResponse
import asyncio
import socket
import json
import io
import base64
from PIL import Image
from sse_starlette.sse import EventSourceResponse

router = APIRouter()

LIDAR_HOST = "127.0.0.1"
LIDAR_PORT = 9002

LIDAR_CMD_MAP = {
    "status"      : "PING",
    "start"       : "START",
    "stop"        : "STOP",
    "distance"    : "DISTANCE",
    "reflectivity": "REFLECTIVITY",
}

def send_lidar(cmd: str, timeout=150) -> str:
    with socket.create_connection((LIDAR_HOST, LIDAR_PORT), timeout=timeout) as s:
        s.sendall((cmd + "\n").encode())
        data = s.recv(4096)
    return data.decode(errors="ignore").strip()
def get_base64_reflectivity(resp: str) -> str:
    if not resp.startswith("OK IMAGE "):
        return ""
    try:
        path = resp.split("OK IMAGE ", 1)[1].strip()
        with Image.open(path) as img:
            buffered = io.BytesIO()
            img.save(buffered, format="PNG")
            return base64.b64encode(buffered.getvalue()).decode()
    except Exception as e:
        print(f"⚠️ Chyba při konverzi PGM na PNG: {e}")
        return ""
@router.get("/lidar_test")
async def lidar_test_page():
    return FileResponse("/opt/projects/robotour/server/static/lidar_test.html")

@router.get("/lidar/{action}")
async def lidar_action(action: str):
    action = action.lower()
    if action not in LIDAR_CMD_MAP:
        return JSONResponse(status_code=400, content={"error":"bad action"})
    try:
        resp = await asyncio.to_thread(send_lidar, LIDAR_CMD_MAP[action])
        
        # Speciální podpora pro reflectivity, abychom vrátili i base64 obrázek přímo
        image_base64 = ""
        if action == "reflectivity" and resp.startswith("OK IMAGE "):
            image_base64 = await asyncio.to_thread(get_base64_reflectivity, resp)
            
        return {"action":action, "response":resp, "image": image_base64}
    except Exception as e:
        return JSONResponse(status_code=500, content={"error":str(e)})

@router.get("/lidar_stream")
async def lidar_stream():
    async def event_generator():
        try:
            while True:
                # Voláme oba příkazy na pozadí, abychom neblokovali event loop
                dist_resp = await asyncio.to_thread(send_lidar, "DISTANCE")
                reflect_resp = await asyncio.to_thread(send_lidar, "REFLECTIVITY")
                
                img_base64 = ""
                if reflect_resp.startswith("OK IMAGE "):
                    img_base64 = await asyncio.to_thread(get_base64_reflectivity, reflect_resp)
                
                payload = {
                    "distance": dist_resp,
                    "image": img_base64
                }
                yield f"data: {json.dumps(payload)}\n\n"
                await asyncio.sleep(0.5)
        except asyncio.CancelledError:
            print("🛑 SSE klient odpojen")
            return
    return EventSourceResponse(event_generator())
