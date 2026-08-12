import threading
import zmq
import json
import time

class DataReceiver:
    """
    Asynchronní smyčka poslouchající zprávy ze všech senzorů přes ZMQ.
    Při přijetí dat aktualizuje sdílený RobotState.
    """
    def __init__(self, state):
        self.state = state
        self.is_running = False
        self.thread = None
        self.shutdown_event = threading.Event()

    def start(self):
        if not self.is_running:
            self.shutdown_event.clear()
            self.thread = threading.Thread(target=self._receiver_loop, daemon=True)
            self.thread.start()
            self.is_running = True

    def stop(self):
        if self.is_running:
            self.shutdown_event.set()
            if self.thread:
                self.thread.join(timeout=3.0)
            self.is_running = False

    def _receiver_loop(self):
        context = zmq.Context.instance()
        
        # Připojení k Vision
        sub_vision = context.socket(zmq.SUB)
        sub_vision.setsockopt(zmq.CONFLATE, 1)
        sub_vision.connect("ipc:///tmp/robot-vision")
        sub_vision.setsockopt_string(zmq.SUBSCRIBE, "")

        # Připojení k Lidar
        sub_lidar = context.socket(zmq.SUB)
        sub_lidar.setsockopt(zmq.CONFLATE, 1)
        sub_lidar.connect("ipc:///tmp/robot-lidar")
        sub_lidar.setsockopt_string(zmq.SUBSCRIBE, "")

        # Připojení k Drive (Odometrie)
        sub_odom = context.socket(zmq.SUB)
        sub_odom.setsockopt(zmq.CONFLATE, 1)
        sub_odom.connect("ipc:///tmp/robot-drive")
        sub_odom.setsockopt_string(zmq.SUBSCRIBE, "ODM")

        poller = zmq.Poller()
        poller.register(sub_vision, zmq.POLLIN)
        poller.register(sub_lidar, zmq.POLLIN)
        poller.register(sub_odom, zmq.POLLIN)
        
        print("📥 [DataReceiver] Vlákno spuštěno, naslouchám senzorům.")

        try:
            while not self.shutdown_event.is_set():
                socks = dict(poller.poll(200))

                if sub_vision in socks:
                    parts = sub_vision.recv_multipart()
                    if len(parts) == 2:
                        if parts[0].decode('utf-8') == "DETECTIONS":
                            try:
                                data = json.loads(parts[1].decode('utf-8'))
                                self.state.update_vision(data.get("pose", []))
                            except Exception as e:
                                print(f"❌ [DataReceiver] Chyba JSON parsování (Vision): {e}")
                        else:
                            print(f"⚠️ [DataReceiver] Neznámý topic z Vision: {parts[0].decode('utf-8', errors='ignore')}")
                    else:
                        first_frame = parts[0].decode('utf-8', errors='ignore') if len(parts) > 0 else "EMPTY"
                        print(f"⚠️ [DataReceiver] Neplatný formát zprávy z Vision, očekávány 2 části, přijato {len(parts)}, první frame: '{first_frame}'")

                if sub_lidar in socks:
                    parts = sub_lidar.recv_multipart()
                    if len(parts) == 2:
                        if parts[0].decode('utf-8') == "DISTANCE":
                            try:
                                data = json.loads(parts[1].decode('utf-8'))
                                self.state.update_lidar(data.get("distance", -1.0))
                            except Exception as e:
                                print(f"❌ [DataReceiver] Chyba JSON parsování (Lidar): {e}")
                        else:
                            print(f"⚠️ [DataReceiver] Neznámý topic z Lidar: {parts[0].decode('utf-8', errors='ignore')}")
                    else:
                        first_frame = parts[0].decode('utf-8', errors='ignore') if len(parts) > 0 else "EMPTY"
                        print(f"⚠️ [DataReceiver] Neplatný formát zprávy z Lidar, očekávány 2 části, přijato {len(parts)}, první frame: '{first_frame}'")

                if sub_odom in socks:
                    parts = sub_odom.recv_multipart()
                    if len(parts) == 2:
                        if parts[0].decode('utf-8') == "ODM":
                            try:
                                data = json.loads(parts[1].decode('utf-8'))
                                self.state.update_odom(data.get("left_speed", 0), data.get("right_speed", 0))
                            except Exception as e:
                                print(f"❌ [DataReceiver] Chyba JSON parsování (Odom): {e}")
                        else:
                            print(f"⚠️ [DataReceiver] Neznámý topic z Odom: {parts[0].decode('utf-8', errors='ignore')}")
                    else:
                        first_frame = parts[0].decode('utf-8', errors='ignore') if len(parts) > 0 else "EMPTY"
                        print(f"⚠️ [DataReceiver] Neplatný formát zprávy z Odom, očekávány 2 části, přijato {len(parts)}, první frame: '{first_frame}'")

        except Exception as e:
            print(f"❌ [DataReceiver] Chyba: {e}")
        finally:
            sub_vision.close()
            sub_lidar.close()
            sub_odom.close()
            print("🛑 [DataReceiver] Vlákno ukončeno.")
