import asyncio

class PilotManualClient:
    def __init__(self, host="127.0.0.1", port=9103):
        self.host = host
        self.port = port

    async def send_command(self, cmd: str, timeout: float = 3.0) -> str:
        """Odešle příkaz TCP službě pilot_manual na zadaném portu a vrátí odpověď s timeoutem."""
        try:
            async def _interact():
                reader, writer = await asyncio.open_connection(self.host, self.port)
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
            return response if response else "ERR: No response from pilot_manual service"
        except asyncio.TimeoutError:
            print(f"[PilotManualClient][WARNING] Timeout waiting for pilot_manual response (cmd: {cmd})")
            return "ERR: Timeout (3s)"
        except Exception as e:
            print(f"[PilotManualClient][ERROR] Error communicating with pilot_manual service: {e}")
            return f"ERR: {e}"

    async def start(self, timeout: float = 3.0) -> str:
        return await self.send_command("START", timeout=timeout)

    async def stop(self, timeout: float = 3.0) -> str:
        return await self.send_command("STOP", timeout=timeout)

    async def status(self, timeout: float = 3.0) -> str:
        return await self.send_command("STATUS", timeout=timeout)

    async def handle_command(self, cmd: str) -> str | None:
        if cmd == "PILOT_MANUAL_START": return await self.start()
        if cmd == "PILOT_MANUAL_STOP": return await self.stop()
        if cmd == "PILOT_MANUAL_STATUS": return await self.status()
        return None
