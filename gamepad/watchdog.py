import asyncio
import time
import zmq
import zmq.asyncio
import dbus

TARGET_NAME = "GameSir-Nova Lite"

class GamepadWatchdog:
    def __init__(self, zmq_address="ipc:///tmp/robot-gamepad", fallback_address="tcp://127.0.0.1:5556"):
        self.is_running = False
        self.connected = False
        self.button_states = {}
        self.target_path = None
        self.bus = None
        
        self.zmq_address = zmq_address
        self.fallback_address = fallback_address
        
        self.zmq_context = zmq.asyncio.Context()
        self.zmq_pub = self.zmq_context.socket(zmq.PUB)
        
        try:
            self.zmq_pub.bind(self.zmq_address)
            print(f"[Watchdog][INFO] ZMQ Publisher bound to {self.zmq_address}")
        except Exception as e:
            print(f"[Watchdog][WARNING] Failed to bind to {self.zmq_address}: {e}. Falling back to {self.fallback_address}")
            self.zmq_pub.bind(self.fallback_address)

        try:
            self.bus = dbus.SystemBus()
        except Exception as e:
            print(f"[Watchdog][ERROR] DBus SystemBus failed: {e}")

    def find_device_by_name(self):
        if not self.bus:
            return None, None
        try:
            mngr = dbus.Interface(self.bus.get_object("org.bluez", "/"), "org.freedesktop.DBus.ObjectManager")
            objects = mngr.GetManagedObjects()
            for path, ifaces in objects.items():
                dev = ifaces.get("org.bluez.Device1")
                if not dev: continue
                
                name = dev.get("Name", "")
                alias = dev.get("Alias", "")
                
                if TARGET_NAME in (name, alias):
                    return path, dev
        except Exception as e:
            pass
        return None, None

    def check_connection(self, path):
        if not self.bus:
            return
        try:
            dev_obj = self.bus.get_object("org.bluez", path)
            props = dbus.Interface(dev_obj, "org.freedesktop.DBus.Properties")
            dev_if = dbus.Interface(dev_obj, "org.bluez.Device1")
            
            connected = props.Get("org.bluez.Device1", "Connected")
            services_resolved = props.Get("org.bluez.Device1", "ServicesResolved")
            
            if connected and not services_resolved:
                print("[Watchdog][INFO] Connected but services not resolved. Calling Connect().")
                try:
                    dev_if.Connect()
                except Exception as e:
                    print(f"[Watchdog][ERROR] Connect() failed: {e}")
            
            self.connected = bool(connected and services_resolved)
            
        except Exception as e:
            self.connected = False
        self.button_states = {}

    def update_buttons(self, states):
        self.button_states.update(states)

    def get_button_states(self):
        import json
        return json.dumps(self.button_states)

    def initial_check(self):
        path, dev = self.find_device_by_name()
        if path:
            self.target_path = path
            self.check_connection(path)
        else:
            self.connected = False
        self.button_states = {}

    async def publish(self, topic, data):
        if self.is_running:
            await self.zmq_pub.send_multipart([topic.encode('utf-8'), data.encode('utf-8')])

    def get_status(self):
        return "ON" if self.connected else "OFF"

    async def run_loop(self):
        print("[Watchdog][INFO] Watchdog loop started.")
        self.is_running = True
        self.initial_check()
        
        last_state = "OFF"
        
        while self.is_running:
            if self.target_path:
                self.check_connection(self.target_path)
            else:
                self.initial_check()
                
            current_state = self.get_status()
            if current_state != last_state:
                print(f"[Watchdog][INFO] Gamepad status changed to: {current_state}")
                last_state = current_state
                
            await asyncio.sleep(1.0)
