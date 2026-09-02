import zmq
import zmq.asyncio
import json

class GamepadPublisher:
    """
    Spravuje ZMQ IPC komunikaci na 'ipc:///tmp/robot-gamepad'.
    Publikuje témata:
      - 'AXES'     : normalizované osy a páčky
      - 'BUTTONS'  : události tlačítek (down, up, hold)
      - 'STATUS'   : stav připojení gamepadu (ON / OFF)
    """
    def __init__(self, endpoint="ipc:///tmp/robot-gamepad"):
        self.endpoint = endpoint
        self.ctx = zmq.asyncio.Context()
        self.socket = None
        self.is_bound = False

    def start(self):
        if self.is_bound:
            return
        try:
            self.socket = self.ctx.socket(zmq.PUB)
            self.socket.bind(self.endpoint)
            self.is_bound = True
            print(f"[Publisher][INFO] ZMQ Publisher bound to {self.endpoint}")
        except Exception as e:
            print(f"[Publisher][ERROR] Failed to bind to {self.endpoint}: {e}")
            self.is_bound = False

    def stop(self):
        if self.is_bound and self.socket:
            try:
                self.socket.close(linger=0)
            except Exception:
                pass
            self.socket = None
            self.is_bound = False
            print("[Publisher][INFO] ZMQ Publisher stopped")

    async def publish_axes(self, axes_dict: dict):
        if not self.is_bound or not self.socket:
            return
        try:
            payload = json.dumps(axes_dict).encode("utf-8")
            await self.socket.send_multipart([b"AXES", payload])
        except Exception as e:
            print(f"[Publisher][ERROR] publish_axes error: {e}")

    async def publish_buttons(self, button_events: list):
        """
        Odesílá události tlačítek jako multipart zprávu:
        [b"BUTTONS", b'{"button":"A","state":"down"}', ...]
        """
        if not self.is_bound or not self.socket or not button_events:
            return
        try:
            frames = [b"BUTTONS"]
            for ev in button_events:
                frames.append(json.dumps(ev).encode("utf-8"))
            await self.socket.send_multipart(frames)
        except Exception as e:
            print(f"[Publisher][ERROR] publish_buttons error: {e}")

    async def publish_status(self, status: str):
        if not self.is_bound or not self.socket:
            return
        try:
            await self.socket.send_multipart([b"STATUS", status.encode("utf-8")])
        except Exception as e:
            print(f"[Publisher][ERROR] publish_status error: {e}")
