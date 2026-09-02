import asyncio

class LoggerClient:
    def __init__(self, host="127.0.0.1", port=9012):
        self.host = host
        self.port = port

    async def send_command(self, cmd: str, timeout: float = 3.0) -> str:
        """Odešle příkaz TCP službě logger na zadaném portu a vrátí odpověď s timeoutem."""
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
            return response if response else "ERR: No response from logger service"
        except asyncio.TimeoutError:
            print(f"[LoggerClient][WARNING] Timeout waiting for logger response (cmd: {cmd})")
            return "ERR: Timeout (3s)"
        except Exception as e:
            print(f"[LoggerClient][ERROR] Error communicating with logger service: {e}")
            return f"ERR: {e}"

    async def logger_on(self, timeout: float = 3.0) -> str:
        return await self.send_command("START", timeout=timeout)

    async def logger_off(self, timeout: float = 3.0) -> str:
        return await self.send_command("STOP", timeout=timeout)

    async def logger_status(self, timeout: float = 3.0) -> str:
        return await self.send_command("STATUS", timeout=timeout)

    async def handle_command(self, cmd: str) -> str | None:
        if cmd == "LOGGER_ON": return await self.logger_on()
        if cmd == "LOGGER_OFF": return await self.logger_off()
        if cmd == "LOGGER_STATUS": return await self.logger_status()
        return None
