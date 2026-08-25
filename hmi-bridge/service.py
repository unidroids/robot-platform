import socket
import threading
import subprocess
import os
import glob
import time

class HMIService:
    def __init__(self, device_id, port, sound_path):
        self.device_id = device_id
        self.port = port
        self.sound_path = sound_path
        self.server_socket = None
        self.running = False
        
    def setup_bridge(self):
        print("[SERVICE] Setting up ADB bridge mappings...")
        
        # reverse: device -> host
        # forward: host -> device
        commands = [
            ["adb", "-s", self.device_id, "reverse", "tcp:9000", "tcp:9020"],
            ["adb", "-s", self.device_id, "reverse", "tcp:8001", "tcp:8001"],
            ["adb", "-s", self.device_id, "reverse", "tcp:8002", "tcp:8002"],
            ["adb", "-s", self.device_id, "forward", "tcp:9021", "tcp:9001"],
            ["adb", "-s", self.device_id, "forward", "tcp:9022", "tcp:9002"]
        ]
        
        for cmd in commands:
            print(f"[SERVICE] Executing: {' '.join(cmd)}")
            try:
                res = subprocess.run(cmd, capture_output=True, text=True)
                if res.stdout.strip():
                    print(f"  {res.stdout.strip()}")
                if res.stderr.strip():
                    print(f"  ERROR: {res.stderr.strip()}")
            except Exception as e:
                print(f"  Exception: {e}")

    def monitor_device(self):
        print(f"[SERVICE] Starting device monitor for {self.device_id}...")
        device_was_connected = False
        while self.running:
            try:
                res = subprocess.run(["adb", "devices"], capture_output=True, text=True)
                lines = res.stdout.strip().split("\n")
                
                is_connected = False
                for line in lines[1:]:
                    if self.device_id in line and "device" in line:
                        is_connected = True
                        break
                        
                if is_connected and not device_was_connected:
                    print(f"[SERVICE] Device {self.device_id} CONNECTED. Setting up bridges.")
                    self.setup_bridge()
                    device_was_connected = True
                elif not is_connected and device_was_connected:
                    print(f"[SERVICE] Device {self.device_id} DISCONNECTED.")
                    device_was_connected = False
                    
            except Exception as e:
                pass
            
            for _ in range(6):
                if not self.running:
                    break
                time.sleep(0.5)

    def run(self):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind(("127.0.0.1", self.port))
        self.server_socket.listen(5)
        self.server_socket.settimeout(1.0)
        self.running = True
        
        threading.Thread(target=self.monitor_device, daemon=True).start()
        
        print(f"[SERVICE] Listening for commands on TCP 127.0.0.1:{self.port}...")
        
        while self.running:
            try:
                client, addr = self.server_socket.accept()
                print(f"[SERVICE] Accepted connection from {addr}")
                threading.Thread(target=self.handle_client, args=(client,), daemon=True).start()
            except socket.timeout:
                continue
            except OSError:
                break # socket closed
                
    def teardown_bridge(self):
        print("[SERVICE] Tearing down ADB bridge mappings...")
        commands = [
            ["adb", "-s", self.device_id, "reverse", "--remove", "tcp:9000"],
            ["adb", "-s", self.device_id, "reverse", "--remove", "tcp:8001"],
            ["adb", "-s", self.device_id, "reverse", "--remove", "tcp:8002"],
            ["adb", "-s", self.device_id, "forward", "--remove", "tcp:9021"],
            ["adb", "-s", self.device_id, "forward", "--remove", "tcp:9022"]
        ]
        
        for cmd in commands:
            print(f"[SERVICE] Executing: {' '.join(cmd)}")
            try:
                subprocess.run(cmd, capture_output=True, text=True)
            except Exception as e:
                print(f"  Exception: {e}")
                
    def stop(self):
        self.running = False
        if self.server_socket:
            try:
                self.server_socket.close()
            except:
                pass
        self.teardown_bridge()
                
    def handle_client(self, client):
        client.settimeout(60.0)
        with client:
            while self.running:
                try:
                    data = client.recv(1024)
                    if not data:
                        break
                        
                    cmd = data.decode('utf-8').strip()
                    if not cmd:
                        continue
                        
                    print(f"[SERVICE] Received command: {cmd}")
                    
                    if cmd == "PING":
                        client.sendall(b"PONG HMI BRIDGE\n")
                    elif cmd == "SHUTDOWN":
                        client.sendall(b"OK\n")
                        print("[SERVICE] Received SHUTDOWN command. Exiting...")
                        self.stop()
                        os._exit(0)
                    elif cmd == "EXIT":
                        client.sendall(b"OK\n")
                        print("[SERVICE] Client requested EXIT. Closing connection.")
                        break
                    elif cmd == "SYNC":
                        response = self.sync_sounds()
                        client.sendall(response.encode('utf-8') + b"\n")
                    elif cmd == "STATUS":
                        response = self.get_status()
                        client.sendall(response.encode('utf-8') + b"\n")
                    else:
                        client.sendall(b"ERROR Unknown command\n")
                except socket.timeout:
                    continue
                except Exception as e:
                    print(f"[SERVICE] Client error: {e}")
                    break

    def sync_sounds(self):
        remote_dir = "/sdcard/Android/data/com.unidroids.robot_hmi/files/Sounds"
        print(f"[SERVICE] Syncing sounds from {self.sound_path} to {remote_dir}")
        
        if not os.path.exists(self.sound_path):
            msg = f"ERROR Sound path {self.sound_path} does not exist locally."
            print(f"[SERVICE] {msg}")
            return msg
            
        try:
            mkdir_cmd = ["adb", "-s", self.device_id, "shell", "mkdir", "-p", remote_dir]
            subprocess.run(mkdir_cmd)
            
            files = glob.glob(os.path.join(self.sound_path, "*"))
            if not files:
                print("[SERVICE] No sounds to sync.")
                return "OK No files to sync"
                
            for f in files:
                if os.path.isfile(f):
                    push_cmd = ["adb", "-s", self.device_id, "push", f, remote_dir + "/"]
                    print(f"[SERVICE] > {' '.join(push_cmd)}")
                    res = subprocess.run(push_cmd, capture_output=True, text=True)
                    if res.returncode != 0:
                        return f"ERROR ADB push failed: {res.stderr.strip()}"
                        
            print("[SERVICE] Sync complete.")
            return "OK"
        except Exception as e:
            msg = f"ERROR {e}"
            print(f"[SERVICE] {msg}")
            return msg

    def get_status(self):
        try:
            # Check device state
            res = subprocess.run(["adb", "devices"], capture_output=True, text=True)
            lines = res.stdout.strip().split("\n")
            device_connected = False
            state = "offline"
            
            for line in lines[1:]:
                if self.device_id in line:
                    device_connected = True
                    parts = line.split()
                    if len(parts) >= 2:
                        state = parts[1]
                        
            status_msg = f"STATUS Device: {self.device_id} | Connected: {device_connected} | State: {state}"
            print(f"[SERVICE] {status_msg}")
            return status_msg
        except Exception as e:
            return f"ERROR Failed to get status: {e}"
