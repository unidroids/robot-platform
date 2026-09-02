import os
import time
import asyncio
import pygame
from datalogger import GamepadDataLogger
from devices.profile import GamepadProfile
from publisher import GamepadPublisher
from watchdog import GamepadWatchdog

class GamepadService:
    """
    Hlavní výkonná služba pro gamepad:
      - Zprostředkovává čtení vstupů z pygame.joystick (10 Hz).
      - Normalizuje osy a generuje události tlačítek (down, up, hold).
      - Publikuje data přes GamepadPublisher do ZMQ IPC.
      - Loguje surová data přes GamepadDataLogger.
      - Sleduje BLE stav přes GamepadWatchdog.
    """
    def __init__(self, watchdog: GamepadWatchdog, publisher: GamepadPublisher):
        self.watchdog = watchdog
        self.publisher = publisher
        self.poll_period_sec = 0.1
        self.hold_delay_sec = 0.5

        self.joystick = None
        self.is_running = False
        self.packet_count = 0

        self.button_states = GamepadProfile.get_default_button_states()
        self.button_state_tracker = {}
        self.logger_instance = GamepadDataLogger()
        self._task = None

    def init_gamepad(self) -> bool:
        os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
        try:
            pygame.init()
            pygame.joystick.quit()
            pygame.joystick.init()

            if pygame.joystick.get_count() > 0:
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
                print(f"[GAMEPAD][INFO] Používám joystick: {self.joystick.get_name()} (axes={self.joystick.get_numaxes()})")
                return True
            else:
                self.joystick = None
                return False
        except Exception as e:
            print(f"[GAMEPAD][ERROR] init_gamepad selhalo: {e}")
            self.joystick = None
            return False

    def get_raw_state(self):
        if self.joystick and self.watchdog.is_ready:
            try:
                pygame.event.pump()
                axes = [self.joystick.get_axis(i) for i in range(self.joystick.get_numaxes())]
                buttons = [self.joystick.get_button(i) for i in range(self.joystick.get_numbuttons())]
                hats = [self.joystick.get_hat(i) for i in range(self.joystick.get_numhats())]
                return axes, buttons, hats
            except Exception as e:
                print(f"[GAMEPAD][DEBUG] get_raw_state error: {e}")
                return [], [], []
        else:
            try:
                pygame.event.pump()
            except Exception:
                pass
            return [], [], []

    def process_buttons(self, raw_buttons) -> list:
        current_buttons = GamepadProfile.map_buttons(raw_buttons)
        now = time.time()
        events = []

        for btn, is_down in current_buttons.items():
            if btn not in self.button_state_tracker:
                self.button_state_tracker[btn] = {"is_down": False, "down_time": 0, "hold_emitted": False}

            tracker = self.button_state_tracker[btn]

            if is_down and not tracker["is_down"]:
                tracker["is_down"] = True
                tracker["down_time"] = now
                tracker["hold_emitted"] = False
                events.append({"button": btn, "state": "down"})
            elif not is_down and tracker["is_down"]:
                tracker["is_down"] = False
                events.append({"button": btn, "state": "up"})
            elif is_down and tracker["is_down"] and not tracker["hold_emitted"]:
                if now - tracker["down_time"] >= self.hold_delay_sec:
                    tracker["hold_emitted"] = True
                    events.append({"button": btn, "state": "hold"})

        # Aktualizace celkového stavu tlačítek (pro dotaz BUTTONS)
        for b, tracker in self.button_state_tracker.items():
            if tracker["hold_emitted"]:
                self.button_states[b] = "hold"
            elif tracker["is_down"]:
                self.button_states[b] = "down"
            else:
                self.button_states[b] = "up"

        return events

    def get_button_states(self) -> dict:
        return self.button_states.copy()

    def get_status_info(self) -> dict:
        joystick_name = None
        if self.joystick:
            try:
                joystick_name = self.joystick.get_name()
            except Exception:
                pass
        if not joystick_name:
            joystick_name = self.watchdog.device_name

        return {
            "service": "RUNNING" if self.is_running else "IDLE",
            "ble_connected": self.watchdog.ble_connected,
            "services_resolved": self.watchdog.services_resolved,
            "joystick_ready": self.joystick is not None,
            "device_name": joystick_name,
            "streaming": bool(self.is_running and self.watchdog.is_ready and self.joystick is not None),
            "packet_count": self.packet_count
        }

    async def compute_loop(self):
        print("[GAMEPAD][INFO] Výpočetní smyčka START")
        last_watchdog_ready = False

        try:
            while self.is_running:
                current_watchdog_ready = self.watchdog.is_ready

                # Detekce přechodu do stavu ON -> reinicializujeme pygame joystick
                if not last_watchdog_ready and current_watchdog_ready:
                    print("[GAMEPAD][INFO] Detekováno připojení BLE gamepadu, inicializuji joystick...")
                    await asyncio.sleep(1.0)
                    self.init_gamepad()
                elif last_watchdog_ready and not current_watchdog_ready:
                    print("[GAMEPAD][INFO] Gamepad odpojen na BLE vrstvě.")
                    self.joystick = None

                last_watchdog_ready = current_watchdog_ready

                if not current_watchdog_ready or self.joystick is None:
                    # Pokud je gamepad na BLE připojen, ale joystick ještě nebyl inicializován, zkusíme to
                    if current_watchdog_ready and self.joystick is None:
                        self.init_gamepad()
                    await asyncio.sleep(self.poll_period_sec)
                    continue

                raw_axes, raw_buttons, raw_hats = self.get_raw_state()

                if raw_axes or raw_buttons or raw_hats:
                    axes_state = GamepadProfile.normalize_axes(raw_axes, raw_hats)
                    self.logger_instance.log_raw_data(raw_axes, raw_buttons, raw_hats)

                    # Publikace os do ZMQ
                    await self.publisher.publish_axes(axes_state)

                    # Zpracování a publikace tlačítek do ZMQ
                    button_events = self.process_buttons(raw_buttons)
                    if button_events:
                        await self.publisher.publish_buttons(button_events)

                    self.packet_count += 1

                await asyncio.sleep(self.poll_period_sec)

        except asyncio.CancelledError:
            pass
        except Exception as e:
            print(f"[GAMEPAD][ERROR] Chyba ve výpočetní smyčce: {e}")
        finally:
            self.is_running = False
            self.joystick = None
            print("[GAMEPAD][INFO] Výpočetní smyčka STOP")

    def start(self) -> bool:
        if self.is_running:
            return False

        print("[GAMEPAD][INFO] Spouštím službu GamepadService...")
        self.publisher.start()
        self.watchdog.start()

        self.is_running = True
        self.packet_count = 0
        self.button_states = GamepadProfile.get_default_button_states()
        self.button_state_tracker = {}
        self.logger_instance.start()

        self.init_gamepad()
        self._task = asyncio.create_task(self.compute_loop())
        return True

    async def stop(self):
        if not self.is_running:
            return

        print("[GAMEPAD][INFO] Zastavuji službu GamepadService...")
        self.is_running = False

        if self._task:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
            self._task = None

        self.logger_instance.stop()
        await self.watchdog.stop()
        self.publisher.stop()

        self.button_states = GamepadProfile.get_default_button_states()
        self.button_state_tracker.clear()
        self.joystick = None
        try:
            pygame.joystick.quit()
        except Exception:
            pass
