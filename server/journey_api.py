from fastapi import APIRouter
from fastapi.responses import JSONResponse, FileResponse
import asyncio
import socket
import os
import glob
import shutil
from datetime import datetime
from sse_starlette.sse import EventSourceResponse

router = APIRouter()

JOURNEY_HOST = "127.0.0.1"
JOURNEY_PORT = 9004

JOURNEY_CMD_MAP = {
    "ping"    : "PING",
    "status"    : "STATE",
    "demo"      : "DEMO",
    "manual"    : "MANUAL",
    "point-set" : "SET-POINT",
    "point-goto": "TO-POINT",
    "point-back": "POINT-AND-BACK",
    "auto"      : "AUTO",
    "stop"      : "STOP",
}

def send_journey(cmd: str, timeout=20) -> str:
    with socket.create_connection((JOURNEY_HOST, JOURNEY_PORT), timeout=timeout) as s:
        s.sendall((cmd + "\n").encode())
        data = s.recv(4096)
    return data.decode(errors="ignore").strip()

@router.get("/journey_test")
async def journey_test_page():
    return FileResponse("/opt/projects/robotour/server/static/journey_test.html")

@router.get("/journey/set-latest-waypoints")
async def set_latest_waypoints():
    try:
        journey_dir = "/data/robot/journey"
        route_file = "/opt/projects/robotour/journey/waypoints/_route.json"
        
        search_pattern = os.path.join(journey_dir, "waypoints-*.json")
        files = glob.glob(search_pattern)
        
        if not files:
            return JSONResponse(status_code=404, content={"error": "Nebyly nalezeny žádné soubory s waypointy."})
            
        # Nalezení nejmladšího souboru podle času poslední modifikace
        latest_file = max(files, key=os.path.getmtime)
        
        # Pokud existuje aktuální _route.json, vytvoříme zálohu s přesností na sekundy
        if os.path.exists(route_file):
            ts = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
            backup_file = route_file.replace(".json", f"_{ts}.json")
            shutil.copy2(route_file, backup_file)
            
        # Přepsání aktuálního _route.json nejnovějšími daty
        os.makedirs(os.path.dirname(route_file), exist_ok=True)
        shutil.copy2(latest_file, route_file)
        
        return {"action": "set-latest-waypoints", "status": "ok", "source": latest_file, "destination": route_file}
    except Exception as e:
        return JSONResponse(status_code=500, content={"error": str(e)})

@router.get("/journey/load-route/{route_id}")
async def load_route(route_id: str):
    try:
        waypoints_dir = "/opt/projects/robotour/journey/waypoints"
        route_file = os.path.join(waypoints_dir, "_route.json")
        
        # Bezpečnostní kontrola proti path traversal
        if ".." in route_id or "/" in route_id or "\\" in route_id:
            return JSONResponse(status_code=400, content={"error": "Neplatný formát parametru route_id."})
            
        source_file = os.path.join(waypoints_dir, f"_route-{route_id}.json")
        
        if not os.path.exists(source_file):
            return JSONResponse(status_code=404, content={"error": f"Zdrojový soubor trasy nebyl nalezen: _route-{route_id}.json"})
            
        # Pokud existuje aktuální _route.json, vytvoříme zálohu s přesností na sekundy
        if os.path.exists(route_file):
            ts = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
            backup_file = route_file.replace(".json", f"_{ts}.json")
            shutil.copy2(route_file, backup_file)
            
        # Přepsání aktuálního _route.json vybranými daty
        os.makedirs(os.path.dirname(route_file), exist_ok=True)
        shutil.copy2(source_file, route_file)
        
        return {"action": "load-route", "status": "ok", "route_id": route_id, "source": source_file, "destination": route_file}
    except Exception as e:
        return JSONResponse(status_code=500, content={"error": str(e)})

@router.get("/journey/{action}")
async def journey_action(action: str):
    action = action.lower()
    if action not in JOURNEY_CMD_MAP:
        return JSONResponse(status_code=400, content={"error":"bad action"})
    try:
        resp = await asyncio.to_thread(send_journey, JOURNEY_CMD_MAP[action])
        return {"action":action, "response":resp}
    except Exception as e:
        return JSONResponse(status_code=500, content={"error":str(e)})
