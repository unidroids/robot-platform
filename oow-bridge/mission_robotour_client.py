import asyncio

class MissionRobotourClient:
    def __init__(self, host="127.0.0.1", port=9031):
        self.host = host
        self.port = port

    async def send_command(self, cmd: str, timeout: float = 3.0) -> str:
        """Odešle příkaz TCP službě mission_robotour na zadaném portu a vrátí odpověď s timeoutem."""
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
            return response if response else "ERR: No response from mission_robotour service"
        except asyncio.TimeoutError:
            print(f"[MissionRobotourClient][WARNING] Timeout waiting for mission_robotour response (cmd: {cmd})")
            return "ERR: Timeout (3s)"
        except Exception as e:
            print(f"[MissionRobotourClient][ERROR] Error communicating with mission_robotour service: {e}")
            return f"ERR: {e}"

    async def start(self, timeout: float = 3.0) -> str:
        return await self.send_command("START", timeout=timeout)

    async def stop(self, timeout: float = 3.0) -> str:
        return await self.send_command("STOP", timeout=timeout)

    async def status(self, timeout: float = 3.0) -> str:
        return await self.send_command("STATUS", timeout=timeout)

    async def handle_command(self, cmd: str) -> str | None:
        if cmd == "MISSION_ROBOTOUR_ON": return await self.start()
        if cmd == "MISSION_ROBOTOUR_OFF": return await self.stop()
        if cmd == "MISSION_ROBOTOUR_STATUS": return await self.status()
        return None
