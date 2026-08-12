import threading
import zmq

class RtkPoller:
    def __init__(self, gnss_serial):
        self.gnss_serial = gnss_serial
        self.running = False
        self._thread = None
        self._stop_event = threading.Event()

    def start(self):
        if self.running:
            return
        self._stop_event.clear()
        self.running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        print("[RTK Poller] Nastartován, naslouchám na ipc:///tmp/robot-rtk")

    def stop(self):
        if not self.running:
            return
        self._stop_event.set()
        self.running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        print("[RTK Poller] Zastaven.")

    def _run(self):
        ctx = zmq.Context.instance()
        zmq_sub = ctx.socket(zmq.SUB)
        zmq_sub.connect("ipc:///tmp/robot-rtk")
        zmq_sub.setsockopt_string(zmq.SUBSCRIBE, "RTCM ")
        zmq_sub.setsockopt(zmq.RCVTIMEO, 1000)

        while not self._stop_event.is_set():
            try:
                # Očekáváme binární data: b"RTCM " + data
                msg = zmq_sub.recv()
                if msg.startswith(b"RTCM "):
                    data = msg[5:]
                    if self.gnss_serial:
                        self.gnss_serial.send_data(data)
                        print(f"[RTK Poller] Odesláno {len(data)} bytů RTCM do gnss_serial")
            except zmq.error.Again:
                pass
            except Exception as e:
                print(f"[RTK Poller] Chyba ve smyčce polleru: {e}")

        zmq_sub.close()
