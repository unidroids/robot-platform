import asyncio

class DriveClient:
    def __init__(self, host="127.0.0.1", port=9003):
        self.host = host
        self.port = port

    async def send_command(self, cmd: str, timeout: float = 3.0) -> str:
        """Odešle příkaz TCP službě drive na zadaném portu a vrátí odpověď s timeoutem."""
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
            return response if response else "ERR: No response from drive service"
        except asyncio.TimeoutError:
            print(f"[DriveClient][WARNING] Timeout waiting for drive response (cmd: {cmd})")
            return "ERR: Timeout (3s)"
        except Exception as e:
            print(f"[DriveClient][ERROR] Error communicating with drive service: {e}")
            return f"ERR: {e}"

    async def drive_on(self, timeout: float = 3.0) -> str:
        return await self.send_command("ON", timeout=timeout)

    async def drive_off(self, timeout: float = 3.0) -> str:
        return await self.send_command("OFF", timeout=timeout)

    async def status(self, timeout: float = 3.0) -> str:
        return await self.send_command("STATUS", timeout=timeout)

    async def handle_command(self, cmd: str) -> str | None:
        if cmd == "DRIVE_ON": return await self.drive_on()
        if cmd == "DRIVE_OFF": return await self.drive_off()
        if cmd == "DRIVE_STATUS": return await self.status()
        return None
