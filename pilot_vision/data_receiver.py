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

        # Připojení k Odometry
        sub_odom = context.socket(zmq.SUB)
        sub_odom.setsockopt(zmq.CONFLATE, 1)
        sub_odom.connect("ipc:///tmp/robot-odometry")
        sub_odom.setsockopt_string(zmq.SUBSCRIBE, "")

        poller = zmq.Poller()
        poller.register(sub_vision, zmq.POLLIN)
        poller.register(sub_lidar, zmq.POLLIN)
        poller.register(sub_odom, zmq.POLLIN)
        
        print("📥 [DataReceiver] Vlákno spuštěno, naslouchám senzorům.")

        try:
            while not self.shutdown_event.is_set():
                socks = dict(poller.poll(200))

                if sub_vision in socks:
                    msg = sub_vision.recv_string()
                    try:
                        topic, json_str = msg.split("/", 1)
                        data = json.loads(json_str)
                        self.state.update_vision(data.get("pose", []))
                    except Exception:
                        pass

                if sub_lidar in socks:
                    msg = sub_lidar.recv_string()
                    try:
                        topic, json_str = msg.split("/", 1)
                        data = json.loads(json_str)
                        self.state.update_lidar(data.get("distance", -1.0))
                    except Exception:
                        pass

                if sub_odom in socks:
                    msg = sub_odom.recv_string()
                    try:
                        topic, json_str = msg.split("/", 1)
                        data = json.loads(json_str)
                        self.state.update_odom(data.get("left", 0), data.get("right", 0))
                    except Exception:
                        pass

        except Exception as e:
            print(f"❌ [DataReceiver] Chyba: {e}")
        finally:
            sub_vision.close()
            sub_lidar.close()
            sub_odom.close()
            print("🛑 [DataReceiver] Vlákno ukončeno.")
