import asyncio

class RtkClient:
    def __init__(self, host="127.0.0.1", port=9015):
        self.host = host
        self.port = port

    async def send_command(self, cmd: str, timeout: float = 3.0) -> str:
        """Odešle příkaz TCP službě RTK na zadaném portu a vrátí odpověď s timeoutem."""
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
            return response if response else "ERR: No response from rtk service"
        except asyncio.TimeoutError:
            print(f"[RtkClient][WARNING] Timeout waiting for rtk response (cmd: {cmd})")
            return "ERR: Timeout (3s)"
        except Exception as e:
            print(f"[RtkClient][ERROR] Error communicating with rtk service: {e}")
            return f"ERR: {e}"

    async def rtk_on(self, timeout: float = 3.0) -> str:
        return await self.send_command("START", timeout=timeout)

    async def rtk_off(self, timeout: float = 3.0) -> str:
        return await self.send_command("STOP", timeout=timeout)

    async def rtk_status(self, timeout: float = 3.0) -> str:
        return await self.send_command("STATUS", timeout=timeout)

    async def handle_command(self, cmd: str) -> str | None:
        if cmd == "RTK_ON": return await self.rtk_on()
        if cmd == "RTK_OFF": return await self.rtk_off()
        if cmd == "RTK_STATUS": return await self.rtk_status()
        return None
