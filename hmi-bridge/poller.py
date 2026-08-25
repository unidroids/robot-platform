import zmq
import time

class ZMQPoller:
    def __init__(self, port):
        self.port = port
        self.context = zmq.Context()
        self.sub_socket = self.context.socket(zmq.SUB)
        # We bind to all interfaces so adb reverse (from device) can reach us
        self.sub_socket.bind(f"tcp://127.0.0.1:{self.port}")
        self.sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")
        
        self.pub_sockets = {}
        self.running = False

    def get_pub_socket(self, channel_name):
        if channel_name not in self.pub_sockets:
            pub = self.context.socket(zmq.PUB)
            endpoint = f"ipc:///tmp/{channel_name}"
            try:
                pub.bind(endpoint)
                print(f"[POLLER-{self.port}] Bound PUB socket to {endpoint}")
            except Exception as e:
                print(f"[POLLER-{self.port}] Failed to bind {endpoint}: {e}")
            self.pub_sockets[channel_name] = pub
        return self.pub_sockets[channel_name]

    def run(self):
        self.running = True
        print(f"[POLLER-{self.port}] Started listening on tcp://127.0.0.1:{self.port}")
        
        poller = zmq.Poller()
        poller.register(self.sub_socket, zmq.POLLIN)
        
        while self.running:
            try:
                events = dict(poller.poll(timeout=1000))
                if self.sub_socket in events:
                    frames = self.sub_socket.recv_multipart()
                    if len(frames) == 3:
                        channel_frame = frames[0].decode('utf-8')
                        payload_frames = frames[1:]
                        
                        pub_socket = self.get_pub_socket(channel_frame)
                        pub_socket.send_multipart(payload_frames)
                        # print(f"[POLLER-{self.port}] Forwarded msg to {channel_frame}")
                    else:
                        print(f"[POLLER-{self.port}] Received unexpected msg with {len(frames)} frames")
            except Exception as e:
                if self.running:
                    print(f"[POLLER-{self.port}] Exception: {e}")
                
    def stop(self):
        self.running = False
        self.sub_socket.close()
        for pub in self.pub_sockets.values():
            pub.close()
        self.context.term()
