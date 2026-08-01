import asyncio
import time
import zmq
import zmq.asyncio
from camera_client import CameraClient
from fusion_client import FusionClient
from lidar_client import LidarClient
from pilot_waypoints_client import PilotWaypointsClient
from drive_client import DriveClient
from rtk_client import RtkClient

class OfficerWatchdog:
    def __init__(self, logger, zmq_address="ipc:///tmp/robot-oow", fallback_address="tcp://127.0.0.1:5555"):
        self.logger = logger
        self.last_heartbeat = 0.0
        self.is_running = False
        self.ble_server = None
        self.camera_client = CameraClient()
        self.fusion_client = FusionClient()
        self.lidar_client = LidarClient()
        self.pilot_waypoints_client = PilotWaypointsClient()
        self.drive_client = DriveClient()
        self.rtk_client = RtkClient()
        
        self.zmq_address = zmq_address
        self.fallback_address = fallback_address
        
        self.zmq_context = zmq.asyncio.Context()
        self.zmq_pub = self.zmq_context.socket(zmq.PUB)
        
        # Pokus o bind na IPC, pokud selže (např. nepodporované na Windows), fallback na TCP
        try:
            self.zmq_pub.bind(self.zmq_address)
            print(f"[Watchdog][INFO] ZeroMQ Publisher bound to {self.zmq_address}")
        except Exception as e:
            print(f"[Watchdog][WARNING] Failed to bind to IPC {self.zmq_address}: {e}. Falling back to {self.fallback_address}")
            self.zmq_pub.bind(self.fallback_address)
            print(f"[Watchdog][INFO] ZeroMQ Publisher bound to {self.fallback_address}")

    async def emit_off(self):
        await self._send_zmq("OFF")

    def update_heartbeat(self, client_id):
        if not self.is_running:
            return
        self.last_heartbeat = time.time()
        self.logger.log_event(client_id, "HEARTBEAT")

    def handle_command(self, client_id, command):
        """Zpracování explicitního příkazu."""
        if not self.is_running:
            return
            
        self.logger.log_event(client_id, "COMMAND", command)
        
        if command == "STOP":
            print(f"[Watchdog][WARNING] Client {client_id} sent STOP command!")
            asyncio.create_task(self._send_zmq("STOP"))
            return
        elif command == "PAUSE":
            asyncio.create_task(self._send_zmq("PAUSE"))
            return
        elif command == "RESUME":
            asyncio.create_task(self._send_zmq("RESUME"))
            return
        elif command == "POWEROFF":
            print(f"[Watchdog][WARNING] Client {client_id} sent POWEROFF command!")
            asyncio.create_task(self._poweroff())
            return
            
        clients = [
            self.camera_client,
            self.fusion_client,
            self.lidar_client,
            self.pilot_waypoints_client,
            self.drive_client,
            self.rtk_client
        ]
        for client in clients:
            asyncio.create_task(self._try_client_command(client, command, client_id))

    async def _try_client_command(self, client, command, client_id):
        response = await client.handle_command(command)
        if response is not None:
            print(f"[Watchdog][INFO] Client {client_id} command '{command}' response: {response}")
            if self.ble_server and hasattr(self.ble_server, 'send_response'):
                self.ble_server.send_response(response)

    async def _poweroff(self):
        print("[Watchdog][WARNING] Executing sudo poweroff...")
        try:
            proc = await asyncio.create_subprocess_exec("sudo", "poweroff")
            await proc.wait()
        except Exception as e:
            print(f"[Watchdog][ERROR] Failed to execute poweroff: {e}")

    def get_status(self):
        if self.is_running and (time.time() - self.last_heartbeat) < 1.0:
            return "ON"
        return "OFF"

    async def _send_zmq(self, message):
        print(f"[Watchdog][DEBUG] ZMQ PUB -> {message}")
        await self.zmq_pub.send_string(message)

    async def run_loop(self):
        """Watchdog smyčka, která kontroluje ztrátu spojení a posílá varování."""
        print("[Watchdog][INFO] Watchdog loop started.")
        self.is_running = True
        
        await self.emit_off()
        last_state = "OFF"
        
        while self.is_running:
            current_state = self.get_status()
            
            if current_state != last_state:
                if current_state == "ON":
                    print("[Watchdog][INFO] OOW ON: Active heartbeat detected.")
                    await self._send_zmq("ON")
                else:
                    print("[Watchdog][WARNING] OOW OFF: Heartbeat lost (timeout).")
                    await self._send_zmq("OFF")
                last_state = current_state
                
            await asyncio.sleep(0.1)  # Kontrola každých 100 ms
