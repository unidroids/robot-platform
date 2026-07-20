# dummy_handler.py
import zmq

class DummyHandler:
    def __init__(self):
        self.cnt = 0
        pass

    def handle(self, message_bytes: bytes):
        # Catch-all for unparsed message types.
        # Can be useful for debugging.
        # msg_type = message_bytes[1]
        # print(f"[DummyHandler] Received unhandled message type: 0x{msg_type:02X}")
        self.cnt += 1
        if self.cnt % 10 == 0:
            print(f"[DummyHandler] Received unhandled message type: {self.cnt}")
