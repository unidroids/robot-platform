import asyncio

class GamepadClient:
    def __init__(self, host="127.0.0.1", port=9005):
        self.host = host
        self.port = port

    async def send_command(self, cmd: str, timeout: float = 3.0) -> str:
        """Odešle příkaz TCP službě gamepadu na zadaném portu a vrátí odpověď s timeoutem."""
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
            return response if response else "ERR: No response from gamepad service"
        except asyncio.TimeoutError:
            print(f"[GamepadClient][WARNING] Timeout waiting for gamepad response (cmd: {cmd})")
            return "ERR: Timeout (3s)"
        except Exception as e:
            print(f"[GamepadClient][ERROR] Error communicating with gamepad service: {e}")
            return f"ERR: {e}"

    async def gamepad_on(self, timeout: float = 3.0) -> str:
        return await self.send_command("START", timeout=timeout)

    async def gamepad_off(self, timeout: float = 3.0) -> str:
        return await self.send_command("STOP", timeout=timeout)

    async def gamepad_status(self, timeout: float = 3.0) -> str:
        return await self.send_command("STATUS", timeout=timeout)

    async def handle_command(self, cmd: str) -> str | None:
        if cmd == "GAMEPAD_ON": return await self.gamepad_on()
        if cmd == "GAMEPAD_OFF": return await self.gamepad_off()
        if cmd == "GAMEPAD_STATUS": return await self.gamepad_status()
        return None
