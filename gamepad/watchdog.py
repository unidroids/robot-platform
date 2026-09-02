import asyncio
import dbus

TARGET_NAME = "GameSir-Nova Lite"

class GamepadWatchdog:
    """
    Sleduje a spravuje Bluetooth LE vrstvu (BlueZ přes DBus) pro Gamepad.
    Spouští se až po příkazu START a ukončuje při STOP.
    """
    def __init__(self, publisher=None, target_name=TARGET_NAME):
        self.publisher = publisher
        self.target_name = target_name
        self.is_running = False
        self.ble_connected = False
        self.services_resolved = False
        self.target_path = None
        self.device_name = None
        self.device_address = None
        self.bus = None
        self._task = None

        self._init_dbus()

    def _init_dbus(self):
        try:
            self.bus = dbus.SystemBus()
        except Exception as e:
            print(f"[Watchdog][ERROR] DBus SystemBus inicializace selhala: {e}")
            self.bus = None

    @property
    def is_ready(self) -> bool:
        return bool(self.ble_connected and self.services_resolved)

    def get_status(self) -> str:
        return "ON" if self.is_ready else "OFF"

    def get_detailed_status(self) -> dict:
        return {
            "ble_connected": self.ble_connected,
            "services_resolved": self.services_resolved,
            "device_name": self.device_name,
            "device_address": self.device_address,
            "status": self.get_status()
        }

    def find_device_by_name(self):
        if not self.bus:
            self._init_dbus()
            if not self.bus:
                return None, None
        try:
            mngr = dbus.Interface(self.bus.get_object("org.bluez", "/"), "org.freedesktop.DBus.ObjectManager")
            objects = mngr.GetManagedObjects()
            for path, ifaces in objects.items():
                dev = ifaces.get("org.bluez.Device1")
                if not dev:
                    continue
                name = str(dev.get("Name", ""))
                alias = str(dev.get("Alias", ""))
                address = str(dev.get("Address", ""))
                if self.target_name in (name, alias):
                    self.device_name = alias or name
                    self.device_address = address
                    return path, dev
        except Exception as e:
            print(f"[Watchdog][DEBUG] find_device_by_name error: {e}")
        return None, None

    def check_connection(self, path):
        if not self.bus:
            self.ble_connected = False
            self.services_resolved = False
            return
        try:
            dev_obj = self.bus.get_object("org.bluez", path)
            props = dbus.Interface(dev_obj, "org.freedesktop.DBus.Properties")
            dev_if = dbus.Interface(dev_obj, "org.bluez.Device1")

            connected = bool(props.Get("org.bluez.Device1", "Connected"))
            services_resolved = bool(props.Get("org.bluez.Device1", "ServicesResolved"))

            if connected and not services_resolved:
                print("[Watchdog][INFO] Gamepad připojen na BLE, ale služby nejsou resolved. Volám Connect().")
                try:
                    dev_if.Connect()
                except Exception as e:
                    print(f"[Watchdog][WARNING] dev_if.Connect() selhalo: {e}")

            self.ble_connected = connected
            self.services_resolved = services_resolved

        except Exception as e:
            self.ble_connected = False
            self.services_resolved = False

    def initial_check(self):
        path, dev = self.find_device_by_name()
        if path:
            self.target_path = path
            self.check_connection(path)
        else:
            self.target_path = None
            self.ble_connected = False
            self.services_resolved = False

    async def run_loop(self):
        print("[Watchdog][INFO] Watchdog smyčka START")
        self.is_running = True
        self.initial_check()

        last_state = "OFF"

        try:
            while self.is_running:
                if self.target_path:
                    self.check_connection(self.target_path)
                else:
                    self.initial_check()

                current_state = self.get_status()
                if current_state != last_state:
                    print(f"[Watchdog][INFO] Stav BLE gamepadu změněn na: {current_state} (ble_connected={self.ble_connected}, resolved={self.services_resolved})")
                    if self.publisher:
                        await self.publisher.publish_status(current_state)
                    last_state = current_state

                await asyncio.sleep(1.0)
        except asyncio.CancelledError:
            pass
        except Exception as e:
            print(f"[Watchdog][ERROR] Chyba ve watchdog smyčce: {e}")
        finally:
            self.is_running = False
            self.ble_connected = False
            self.services_resolved = False
            print("[Watchdog][INFO] Watchdog smyčka STOP")

    def start(self):
        if self.is_running:
            return
        self.is_running = True
        self._task = asyncio.create_task(self.run_loop())

    async def stop(self):
        self.is_running = False
        if self._task:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
            self._task = None
        self.ble_connected = False
        self.services_resolved = False
