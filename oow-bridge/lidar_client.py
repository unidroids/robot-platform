import asyncio
import json

class LidarClient:
    def __init__(self, host="127.0.0.1", port=9002, collisions_port=9222, traversability_port=9223):
        self.host = host
        self.port = port
        self.services = {
            "LIDAR": {"port": port, "pong": "PONG LIDAR"},
            "LIDAR-COLLISIONS": {"port": collisions_port, "pong": "PONG LIDAR-COLLISIONS"},
            "LIDAR-TRAVERSABILITY": {"port": traversability_port, "pong": "PONG LIDAR-TRAVERSABILITY"}
        }

    async def _send_to_service(self, port: int, cmd: str, timeout: float = 3.0) -> str:
        """Odešle TCP příkaz dané službě lidaru a vrátí odpověď s timeoutem."""
        try:
            async def _interact():
                reader, writer = await asyncio.open_connection(self.host, port)
                try:
                    writer.write(f"{cmd.strip()}\n".encode("utf-8"))
                    await writer.drain()
                    data = await reader.readline()
                    return data.decode("utf-8").strip()
                finally:
                    try:
                        writer.close()
                        await writer.wait_closed()
                    except Exception:
                        pass

            response = await asyncio.wait_for(_interact(), timeout=timeout)
            return response if response else "ERR: No response"
        except asyncio.TimeoutError:
            print(f"[LidarClient][WARNING] Timeout waiting for response on port {port} (cmd: {cmd})")
            return "ERR: Timeout"
        except Exception as e:
            print(f"[LidarClient][ERROR] Error communicating with port {port}: {e}")
            return f"ERR: {e}"

    async def send_command(self, cmd: str, timeout: float = 3.0) -> str:
        """Odešle příkaz primární službě LIDAR (port 9002)."""
        return await self._send_to_service(self.port, cmd, timeout=timeout)

    async def lidar_on(self, timeout: float = 3.0) -> str:
        """Spustí postupně všechny lidar služby: LIDAR, LIDAR-COLLISIONS a LIDAR-TRAVERSABILITY."""
        results = {}
        # 1. Spustit hardware a rotaci LIDARu
        results["LIDAR"] = await self._send_to_service(self.services["LIDAR"]["port"], "START", timeout=timeout)
        # 2. Spustit vyhodnocování kolizí
        results["LIDAR-COLLISIONS"] = await self._send_to_service(self.services["LIDAR-COLLISIONS"]["port"], "START", timeout=timeout)
        # 3. Spustit vyhodnocování sjízdnosti terénu
        results["LIDAR-TRAVERSABILITY"] = await self._send_to_service(self.services["LIDAR-TRAVERSABILITY"]["port"], "START", timeout=timeout)

        # Stav pro aplikaci (ON pokud se LIDAR nastartoval)
        lidar_ok = "OK" in results["LIDAR"].upper() or "STARTED" in results["LIDAR"].upper()
        results["lidar_status"] = "ON" if lidar_ok else "OFF"

        return json.dumps(results, ensure_ascii=False)

    async def lidar_off(self, timeout: float = 3.0) -> str:
        """Zastaví postupně v opačném pořadí: LIDAR-COLLISIONS, LIDAR-TRAVERSABILITY, LIDAR."""
        results = {}
        # 1. Zastavit vyhodnocování kolizí
        results["LIDAR-COLLISIONS"] = await self._send_to_service(self.services["LIDAR-COLLISIONS"]["port"], "STOP", timeout=timeout)
        # 2. Zastavit vyhodnocování sjízdnosti terénu
        results["LIDAR-TRAVERSABILITY"] = await self._send_to_service(self.services["LIDAR-TRAVERSABILITY"]["port"], "STOP", timeout=timeout)
        # 3. Zastavit samotný LIDAR
        results["LIDAR"] = await self._send_to_service(self.services["LIDAR"]["port"], "STOP", timeout=timeout)

        results["lidar_status"] = "OFF"
        return json.dumps(results, ensure_ascii=False)

    async def lidar_status(self, timeout: float = 3.0) -> str:
        """Získá stav všech tří lidarových služeb."""
        raw_lidar = await self._send_to_service(self.services["LIDAR"]["port"], "STATUS", timeout=timeout)
        raw_col = await self._send_to_service(self.services["LIDAR-COLLISIONS"]["port"], "STATUS", timeout=timeout)
        raw_trav = await self._send_to_service(self.services["LIDAR-TRAVERSABILITY"]["port"], "STATUS", timeout=timeout)

        def _parse_or_raw(val: str):
            try:
                return json.loads(val)
            except Exception:
                return val

        results = {
            "LIDAR": raw_lidar,
            "LIDAR-COLLISIONS": _parse_or_raw(raw_col),
            "LIDAR-TRAVERSABILITY": _parse_or_raw(raw_trav),
        }
        is_running = "RUNNING" in raw_lidar.upper()
        results["lidar_status"] = "ON" if is_running else "OFF"

        return json.dumps(results, ensure_ascii=False)

    async def handle_command(self, cmd: str) -> str | None:
        if cmd == "LIDAR_ON": return await self.lidar_on()
        if cmd == "LIDAR_OFF": return await self.lidar_off()
        if cmd == "LIDAR_STATUS": return await self.lidar_status()
        return None
