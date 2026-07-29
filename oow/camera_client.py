import asyncio

class CameraClient:
    def __init__(self, host="127.0.0.1", port=9001):
        self.host = host
        self.port = port

    async def send_command(self, cmd: str, timeout: float = 3.0) -> str:
        """Odešle příkaz TCP službě kamery na zadaném portu a vrátí odpověď s timeoutem 3s."""
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
            return response if response else "ERR: No response from camera service"
        except asyncio.TimeoutError:
            print(f"[CameraClient][WARNING] Timeout waiting for camera response (cmd: {cmd})")
            return "ERR: Timeout (3s)"
        except Exception as e:
            print(f"[CameraClient][ERROR] Error communicating with camera service: {e}")
            return f"ERR: {e}"

    async def camera_on(self, timeout: float = 3.0) -> str:
        return await self.send_command("START", timeout=timeout)

    async def camera_off(self, timeout: float = 3.0) -> str:
        return await self.send_command("STOP", timeout=timeout)

    async def camera_status(self, timeout: float = 3.0) -> str:
        return await self.send_command("STATUS", timeout=timeout)
