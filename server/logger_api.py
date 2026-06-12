from fastapi import APIRouter
from fastapi.responses import JSONResponse, FileResponse
import asyncio
import socket

router = APIRouter()

LOGGER_HOST = "127.0.0.1"
LOGGER_PORT = 9012

LOGGER_CMD_MAP = {
    "ping": "PING",
    "status": "STATUS",
    "start" : "START",
    "stop"  : "STOP",
}

def send_logger(cmd: str, timeout=2.0) -> str:
    with socket.create_connection((LOGGER_HOST, LOGGER_PORT), timeout=timeout) as s:
        s.sendall((cmd+"\n").encode())
        data = s.recv(4096)
    return data.decode(errors="ignore").strip()

@router.get("/logger_test")
async def logger_test_page():
    return FileResponse("/opt/projects/robotour/server/static/logger_test.html")

@router.get("/logger/{action}")
async def logger_action(action: str):
    action = action.lower()
    if action not in LOGGER_CMD_MAP:
        return JSONResponse(status_code=400, content={"error":"bad action"})
    try:
        resp = await asyncio.to_thread(send_logger, LOGGER_CMD_MAP[action])
        return {"action":action, "response":resp}
    except Exception as e:
        return JSONResponse(status_code=500, content={"error":str(e)})
