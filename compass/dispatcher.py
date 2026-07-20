# dispatcher.py
import threading
from typing import Dict
from imu_serial import ImuSerialIO

class MessageDispatcher:
    """Single-thread dispatcher for parsed IMU messages."""

    def __init__(self, imu_serial: ImuSerialIO):
        self.imu_serial = imu_serial
        self._stop_event = threading.Event()
        self._thread = None 
        self._handlers: Dict[int, object] = {}  
        self._messages_handled = 0
        self._messages_unknown = 0
        self._messages_errors = 0

    def register_handler(self, msg_type: int, handler_obj: object):
        if not hasattr(handler_obj, "handle") or not callable(getattr(handler_obj, "handle")):
            raise TypeError("Handler must implement handle(message_bytes)")
        self._handlers[msg_type] = handler_obj

    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        t = self._thread
        if t and t.is_alive():
            t.join(timeout=0.5)
        self._thread = None        

    def stats(self):
        return (self._messages_handled,
                self._messages_unknown,
                self._messages_errors)

    def _run(self):
        while not self._stop_event.is_set():
            try:
                message = self.imu_serial.get_message(timeout=1.0)
                if not message:
                    continue
                
                # Message is guaranteed to be 11 bytes, 0x55 is index 0
                msg_type = message[1]
                handler = self._handlers.get(msg_type)
                
                if handler:
                    handler.handle(message)
                    self._messages_handled += 1
                else:
                    self._messages_unknown += 1
                    
            except Exception as e:
                print(f"[MessageDispatcher] Error: {e}")
                self._messages_errors += 1
