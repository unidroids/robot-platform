import asyncio
import json

class FusionClient:
    def __init__(self, host="127.0.0.1"):
        self.host = host
        # Služby a jejich očekávané PONG odpovědi
        self.services = {
            "LOGGER": {"port": 9012, "pong": "PONG LOGGER"},
            "GPS": {"port": 9006, "pong": "PONG GPS"},
            "DRIVE": {"port": 9003, "pong": "PONG DRIVE"},
            "HEADING": {"port": 9010, "pong": "PONG HEADING"},
            "COMPASS": {"port": 9014, "pong": "PONG COMPASS"}
        }
        self.fusion_service = {"port": 9009, "pong": "PONG FUSION"}

    async def _send_command(self, port: int, expected_pong: str, cmd: str, timeout: float = 3.0) -> str:
        """Odešle PING a po ověření pošle samotný příkaz."""
        try:
            async def _interact():
                reader, writer = await asyncio.open_connection(self.host, port)
                try:
                    # Krok 1: PING
                    writer.write(b"PING\n")
                    await writer.drain()
                    pong_data = await reader.readline()
                    pong_resp = pong_data.decode("utf-8").strip()
                    
                    if pong_resp != expected_pong:
                        return f"ERR: Expected '{expected_pong}', got '{pong_resp}'"

                    # Krok 2: Samotný příkaz
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
            return "ERR: Timeout"
        except Exception as e:
            return f"ERR: {e}"

    async def _control_services(self, cmd: str, reverse_order: bool = False, timeout: float = 3.0) -> str:
        """Postupně zavolá příkaz na všech službách krmících fusion."""
        results = {}
        items = list(self.services.items())
        if reverse_order:
            items.reverse()
            
        for name, config in items:
            res = await self._send_command(config["port"], config["pong"], cmd, timeout)
            results[name] = res
            
        return json.dumps(results, ensure_ascii=False)

    async def fusion_on(self, timeout: float = 3.0) -> str:
        return await self._control_services("START", reverse_order=False, timeout=timeout)

    async def fusion_off(self, timeout: float = 3.0) -> str:
        return await self._control_services("STOP", reverse_order=True, timeout=timeout)

    async def fusion_status(self, timeout: float = 5.0) -> str:
        return await self._send_command(self.fusion_service["port"], self.fusion_service["pong"], "STATUS", timeout)

    async def handle_command(self, cmd: str) -> str | None:
        if cmd == "FUSION_ON": return await self.fusion_on()
        if cmd == "FUSION_OFF": return await self.fusion_off()
        if cmd == "FUSION_STATUS": return await self.fusion_status()
        return None
