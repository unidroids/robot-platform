#!/usr/bin/env python3
import os, time, asyncio, pygame, json
from datalogger import GamepadDataLogger
from devices import GamepadProfile

class GamepadService:
    def __init__(self, watchdog):
        self.watchdog = watchdog
        self.poll_period_sec = 0.1
        self.hold_delay_sec = 0.5
        
        self.joystick = None
        self.is_running = False
        
        self.button_state_tracker = {}
        self.logger_instance = GamepadDataLogger()
        self._task = None

    def init_gamepad(self):
        os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            print(f"[GAMEPAD] Používám: {self.joystick.get_name()} (axes={self.joystick.get_numaxes()})")
            return True
        else:
            self.joystick = None
            print("[GAMEPAD] Upozornění: Joystick nenalezen, poběží v nulových hodnotách.")
            return False

    def get_raw_state(self):
        if self.joystick:
            pygame.event.pump()
            axes = [self.joystick.get_axis(i) for i in range(self.joystick.get_numaxes())]
            buttons = [self.joystick.get_button(i) for i in range(self.joystick.get_numbuttons())]
            hats = [self.joystick.get_hat(i) for i in range(self.joystick.get_numhats())]
            return axes, buttons, hats
        else:
            pygame.event.pump()
            return [], [], []

    def process_buttons(self, raw_buttons):
        current_buttons = GamepadProfile.map_buttons(raw_buttons)
        now = time.time()
        events = {}
        
        for btn, is_down in current_buttons.items():
            if btn not in self.button_state_tracker:
                self.button_state_tracker[btn] = {"is_down": False, "down_time": 0, "hold_emitted": False}
                
            tracker = self.button_state_tracker[btn]
            
            if is_down and not tracker["is_down"]:
                tracker["is_down"] = True
                tracker["down_time"] = now
                tracker["hold_emitted"] = False
                events[btn] = "down"
            elif not is_down and tracker["is_down"]:
                tracker["is_down"] = False
                events[btn] = "up"
            elif is_down and tracker["is_down"] and not tracker["hold_emitted"]:
                if now - tracker["down_time"] >= self.hold_delay_sec:
                    tracker["hold_emitted"] = True
                    events[btn] = "hold"
                    
        if self.watchdog:
            full_state = {}
            for b, tracker in self.button_state_tracker.items():
                if tracker["hold_emitted"]:
                    full_state[b] = "hold"
                elif tracker["is_down"]:
                    full_state[b] = "down"
                else:
                    full_state[b] = "up"
            self.watchdog.update_buttons(full_state)

        if events:
            return events
            
        return None

    async def compute_loop(self):
        print("[GAMEPAD] Výpočetní smyčka START")
        try:
            while self.is_running:
                raw_axes, raw_buttons, raw_hats = self.get_raw_state()
                
                axes_state = GamepadProfile.normalize_axes(raw_axes, raw_hats)
                self.logger_instance.log_raw_data(raw_axes, raw_buttons, raw_hats)
                
                if self.watchdog:
                    await self.watchdog.publish("AXES", json.dumps(axes_state))
                                
                button_events = self.process_buttons(raw_buttons)
                if button_events and self.watchdog:
                    frames = []
                    for btn, ev in button_events.items():
                        data = json.dumps({"button": btn, "state": ev})
                        frames.append(data)
                    if frames:
                        await self.watchdog.publish("BUTTONS", *frames)

                await asyncio.sleep(self.poll_period_sec)
        except Exception as e:
            print("[GAMEPAD] Výpočetní smyčka Error:", e)            
        finally:
            self.is_running = False
            print("[GAMEPAD] Výpočetní smyčka STOP")

    def start(self):
        if self.is_running:
            return False
        if not self.init_gamepad():
            return False
        self.is_running = True
        self.button_state_tracker = {}
        self.logger_instance.start()
        self._task = asyncio.create_task(self.compute_loop())
        return True

    def stop(self):
        self.is_running = False
        self.logger_instance.stop()
