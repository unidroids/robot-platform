import asyncio
import json
import socket
import subprocess
import psutil
import platform
import os
import sys
from datetime import datetime

from bless import (
    BlessServer,
    BlessGATTCharacteristic,
    GATTCharacteristicProperties,
    GATTAttributePermissions,
)

# UUIDs pro naší OOW Službu a charakteristiky
SERVICE_UUID = "87654321-4321-4321-4321-abcdef987654"
CHAR_COMMAND_UUID = "87654321-4321-4321-4321-abcdef987655"
CHAR_HEARTBEAT_UUID = "87654321-4321-4321-4321-abcdef987656"
CHAR_TELEMETRY_UUID = "87654321-4321-4321-4321-abcdef987657"

class OowBleServer:
    def __init__(self, loop, watchdog):
        self.loop = loop
        self.watchdog = watchdog
        self.watchdog.ble_server = self
        self.server = None

    def send_response(self, response: str):
        """Odešle odpověď klienta přes BLE jako notifikaci na telemetrické charakteristice."""
        if not self.server:
            return
        try:
            char = self.server.get_characteristic(CHAR_TELEMETRY_UUID)
            if char:
                char.value = bytearray(response.encode("utf-8"))
                self.server.update_value(SERVICE_UUID, CHAR_TELEMETRY_UUID)
                print(f"[BLE_Service][DEBUG] Sent BLE response: {response}")
            else:
                print("[BLE_Service][WARNING] Telemetry characteristic not found for response")
        except Exception as e:
            print(f"[BLE_Service][ERROR] Failed to send BLE response: {e}")

    def read_request_handler(self, characteristic: BlessGATTCharacteristic, **kwargs) -> bytearray:
        """Callback při pokusu klienta o čtení charakteristiky."""
        if characteristic.uuid.lower() == CHAR_TELEMETRY_UUID.lower():
            print("[BLE_Service][DEBUG] Read request for Telemetry")
            return self.get_telemetry_payload()
        return bytearray()

    def write_request_handler(self, characteristic: BlessGATTCharacteristic, value: bytearray, **kwargs):
        """Callback při pokusu klienta o zápis do charakteristiky."""
        try:
            val_str = value.decode("utf-8").strip()
            
            if characteristic.uuid.lower() == CHAR_COMMAND_UUID.lower():
                print(f"[BLE_Service][DEBUG] Received command: {val_str}")
                # Formát: "MAC_ADRESA:COMMAND" (např. "00:11:22:33:44:55:ON")
                parts = val_str.split(":", 1)
                if len(parts) == 2:
                    client_id, command = parts
                    self.watchdog.handle_command(client_id.strip(), command.strip().upper())
                else:
                    print(f"[BLE_Service][WARNING] Invalid command format: {val_str}")
                    
            elif characteristic.uuid.lower() == CHAR_HEARTBEAT_UUID.lower():
                # Formát: "MAC_ADRESA"
                client_id = val_str
                self.watchdog.update_heartbeat(client_id.strip())
                
        except Exception as e:
            print(f"[BLE_Service][ERROR] Error handling write request: {e}")

    def get_telemetry_payload(self) -> bytearray:
        """Sestavení telemetrických informací dle specifikace."""
        hostname = socket.gethostname()
        route = ""
        web = ""
        try:
            # Příklad výstupu: 8.8.8.8 via 192.168.188.1 dev wlP1p1s0 src 192.168.188.223 uid 1000
            route_output = subprocess.check_output(["ip", "route", "get", "8.8.8.8"]).decode("utf-8").strip()
            # Může to vrátit více řádků, zajímá nás většinou ten první
            route = route_output.split("\n")[0]
            
            parts = route.split()
            if "src" in parts:
                src_index = parts.index("src")
                if src_index + 1 < len(parts):
                    ip_addr = parts[src_index + 1]
                    web = f"http://{ip_addr}:8080"
        except Exception as e:
            print(f"[BLE_Service][WARNING] nepodařilo se získat ip route: {e}")

        info = {
            "date": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "hostname": hostname,
            "route": route,
            "web": web,
            "system": platform.system(),
            "release": platform.release(),
            "cpu_count": os.cpu_count(),
            "oow_status": self.watchdog.get_status()
        }
        return bytearray(json.dumps(info).encode("utf-8"))

    async def start(self):
        """Spustí a inicializuje BLE server."""
        self.server = BlessServer(name="Tříkolka-OOW", loop=self.loop)
        self.server.read_request_func = self.read_request_handler
        self.server.write_request_func = self.write_request_handler

        # Přidání nové služby
        await self.server.add_new_service(SERVICE_UUID)

        # Command - povolíme oba režimy pro flexibilitu
        await self.server.add_new_characteristic(
            SERVICE_UUID,
            CHAR_COMMAND_UUID,
            GATTCharacteristicProperties.write | GATTCharacteristicProperties.write_without_response,
            b"",
            GATTAttributePermissions.writeable
        )

        # Heartbeat - striktně bez potvrzení pro rychlost
        await self.server.add_new_characteristic(
            SERVICE_UUID,
            CHAR_HEARTBEAT_UUID,
            GATTCharacteristicProperties.write_without_response,
            b"",
            GATTAttributePermissions.writeable
        )

        # Telemetry - přidáme NOTIFY, aby robot mohl data sám odesílat při změně
        await self.server.add_new_characteristic(
            SERVICE_UUID,
            CHAR_TELEMETRY_UUID,
            GATTCharacteristicProperties.read | GATTCharacteristicProperties.notify,
            self.get_telemetry_payload(),
            GATTAttributePermissions.readable
        )

        await self.server.start()
        print(f"[BLE_Service][INFO] BLE Server started. Service UUID: {SERVICE_UUID}")

    async def stop(self):
        """Zastaví BLE server."""
        if self.server:
            await self.server.stop()
            print("[BLE_Service][INFO] BLE Server stopped.")
