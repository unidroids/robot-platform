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
        self.server = None

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
        try:
            ip = socket.gethostbyname(hostname)
        except Exception:
            ip = "unknown"
        
        ips = []
        interfaces = {}
        try:
            for iface, addrs in psutil.net_if_addrs().items():
                iface_ips = []
                for addr in addrs:
                    if addr.family == socket.AF_INET:
                        iface_ips.append(addr.address)
                        ips.append(addr.address)
                if iface_ips:
                    interfaces[iface] = iface_ips
        except Exception:
            try:
                ips = subprocess.check_output(["hostname", "-I"]).decode().strip().split()
            except Exception:
                ips = []
            interfaces = {"default": ips}

        info = {
            "date": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "hostname": hostname,
            "ip": ip,
            "ips": ips,
            "interfaces": interfaces,
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
