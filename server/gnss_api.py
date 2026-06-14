from fastapi import APIRouter
from fastapi.responses import JSONResponse, FileResponse
import asyncio
import socket

router = APIRouter()

SERVICES = {
    "gnss": 9006,
    "pointperfect": 9007,
    "heading": 9010
}

GNSS_CMD_MAP = {
    "ping": "PING",
    "status": "STATUS",
    "start" : "START",
    "stop"  : "STOP",
    "data"  : "DATA",
    "heading": "HEADING",
}

def send_to_service(host: str, port: int, cmd: str, timeout=2.0) -> str:
    try:
        with socket.create_connection((host, port), timeout=timeout) as s:
            s.sendall((cmd+"\n").encode())
            data = s.recv(4096)
        return data.decode(errors="ignore").strip()
    except Exception as e:
        return f"ERROR: {str(e)}"

@router.get("/gnss_test")
async def gnss_test_page():
    return FileResponse("/opt/projects/robotour/server/static/gnss_test.html")

@router.get("/gnss/{action}")
async def gnss_action(action: str):
    action = action.lower()
    if action not in GNSS_CMD_MAP:
        return JSONResponse(status_code=400, content={"error": "bad action"})
    
    cmd = GNSS_CMD_MAP[action]
    
    if action == "data":
        resp = await asyncio.to_thread(send_to_service, "127.0.0.1", SERVICES["gnss"], cmd)
        return {"action": action, "results": {"gnss": resp}}
    
    if action == "heading":
        resp = await asyncio.to_thread(send_to_service, "127.0.0.1", SERVICES["heading"], cmd)
        return {"action": action, "results": {"heading": resp}}
    
    async def call_service(name, port):
        resp = await asyncio.to_thread(send_to_service, "127.0.0.1", port, cmd)
        return name, resp
    
    tasks = [call_service(name, port) for name, port in SERVICES.items()]
    results = await asyncio.gather(*tasks)
    
    return {"action": action, "results": dict(results)}
