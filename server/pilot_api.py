from fastapi import APIRouter
from fastapi.responses import JSONResponse, FileResponse
import asyncio
import socket
import subprocess
from sse_starlette.sse import EventSourceResponse

router = APIRouter()

PILOT_HOST = "127.0.0.1"
PILOT_PORT = 9102
LOG_FILE = "/data/logs/pilot_vision/pilot_vision.log"

PILOT_CMD_MAP = {
    "ping": "PING",
    "status": "STATUS",
    "start": "START TEST_TOKEN",
    "stop": "STOP",
    "pause": "PAUSE",
    "resume": "RESUME"
}

def send_pilot(cmd: str, timeout=2.0) -> str:
    with socket.create_connection((PILOT_HOST, PILOT_PORT), timeout=timeout) as s:
        s.sendall((cmd + "\n").encode())
        data = s.recv(4096)
    return data.decode(errors="ignore").strip()

@router.get("/pilot_test")
async def pilot_test_page():
    return FileResponse("/opt/projects/robotour/server/static/pilot_test.html")

@router.get("/pilot/cmd/{action}")
async def pilot_action(action: str):
    action = action.lower()
    if action not in PILOT_CMD_MAP:
        return JSONResponse(status_code=400, content={"error": "bad action"})
    try:
        resp = await asyncio.to_thread(send_pilot, PILOT_CMD_MAP[action])
        return {"action": action, "response": resp}
    except Exception as e:
        return JSONResponse(status_code=500, content={"error": str(e)})

async def log_generator():
    process = await asyncio.create_subprocess_exec(
        "tail", "-F", "-n", "30", LOG_FILE,
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.PIPE
    )
    try:
        while True:
            line = await process.stdout.readline()
            if not line:
                break
            yield line.decode("utf-8").rstrip()
    except asyncio.CancelledError:
        process.terminate()
        raise

@router.get("/pilot/log")
async def pilot_log_stream():
    return EventSourceResponse(log_generator())
