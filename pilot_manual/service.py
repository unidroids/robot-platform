#!/usr/bin/env python3
import asyncio
import json
import time
import zmq
import zmq.asyncio
from drive_client import DriveClient

class PilotManualService:
    def __init__(self):
        self.is_running = False
        
        # State
        self.max_speed = 1.0  # m/s
        self.current_base_speed = 0.0 # m/s (aktuální rychlost po aplikaci akcelerace)
        
        self.gas_input = 0.0
        self.steering_input = 0.0
        
        self.lidar_dist = -1.0
        self.last_lidar_time = 0.0
        
        self.gamepad_ok = False
        
        # Drive Client
        self.drive_client = DriveClient("127.0.0.1", 9003)
        
        # ZMQ
        self.ctx = zmq.asyncio.Context()
        self.pub = self.ctx.socket(zmq.PUB)
        self.pub.bind("ipc:///tmp/robot-pilot-manual")
        
        self.tasks = []

    async def start(self):
        if self.is_running:
            return True
            
        try:
            self.drive_client.connect()
            if not self.drive_client.ping():
                print("[PilotManual] Start zrušen: Drive service neodpověděl PONG DRIVE.")
                self.drive_client.disconnect()
                return False
        except Exception as e:
            print(f"[PilotManual] Start zrušen: Nelze se připojit k Drive service: {e}")
            return False
            
        try:
            reader, writer = await asyncio.open_connection("127.0.0.1", 9005)
            writer.write(b"PING\n")
            await writer.drain()
            data = await asyncio.wait_for(reader.readline(), timeout=0.5)
            writer.close()
            await writer.wait_closed()
            if b"PONG GAMEPAD" not in data:
                print(f"[PilotManual] Start zrušen: Gamepad service neodpověděl PONG GAMEPAD, ale {data}")
                self.drive_client.disconnect()
                return False
        except Exception as e:
            print(f"[PilotManual] Start zrušen: Nelze se připojit ke Gamepad service: {e}")
            self.drive_client.disconnect()
            return False
            
        self.is_running = True
                
        self.tasks.append(asyncio.create_task(self._zmq_gamepad_loop()))
        self.tasks.append(asyncio.create_task(self._zmq_lidar_loop()))
        self.tasks.append(asyncio.create_task(self._gamepad_tcp_check()))
        self.tasks.append(asyncio.create_task(self._control_loop()))
        print("[PilotManual] Service started.")
        return True

    async def stop(self):
        self.is_running = False
        for t in self.tasks:
            t.cancel()
        self.tasks.clear()
        
        try:
            self.drive_client.send_drive(0, 0, 0)
            self.drive_client.disconnect()
        except:
            pass
        print("[PilotManual] Service stopped.")

    def get_status(self):
        state = "RUNNING" if self.is_running else "IDLE"
        info = {
            "state": state,
            "max_speed": round(self.max_speed, 2),
            "gamepad_ok": self.gamepad_ok,
            "current_base_speed": round(self.current_base_speed, 2)
        }
        return f"{state} {json.dumps(info)}"

    async def _zmq_gamepad_loop(self):
        sub = self.ctx.socket(zmq.SUB)
        sub.connect("ipc:///tmp/robot-gamepad")
        sub.setsockopt_string(zmq.SUBSCRIBE, "AXES")
        sub.setsockopt_string(zmq.SUBSCRIBE, "BUTTONS")
        
        while self.is_running:
            try:
                parts = await sub.recv_multipart()
                topic = parts[0].decode("utf-8")
                
                if topic == "AXES":
                    data = json.loads(parts[1].decode("utf-8"))
                    self.gas_input = data.get("gas", 0.0)
                    self.steering_input = data.get("right_stick", [0.0, 0.0])[0]
                    
                elif topic == "BUTTONS":
                    for p in parts[1:]:
                        btn_event = json.loads(p.decode("utf-8"))
                        if btn_event.get("state") == "down":
                            btn = btn_event.get("button")
                            if btn == "RB":
                                self.max_speed = min(1.7, self.max_speed + 0.1)
                                print(f"[PilotManual] Zvýšena max rychlost na {self.max_speed:.1f} m/s")
                            elif btn == "LB":
                                self.max_speed = max(0.3, self.max_speed - 0.1)
                                print(f"[PilotManual] Snížena max rychlost na {self.max_speed:.1f} m/s")
            except asyncio.CancelledError:
                break
            except Exception as e:
                print(f"[PilotManual] ZMQ Gamepad error: {e}")

    async def _zmq_lidar_loop(self):
        sub = self.ctx.socket(zmq.SUB)
        sub.connect("ipc:///tmp/robot-lidar")
        sub.setsockopt_string(zmq.SUBSCRIBE, "")
        
        while self.is_running:
            try:
                parts = await sub.recv_multipart()
                data = json.loads(parts[-1].decode("utf-8"))
                self.lidar_dist = data.get("distance", -1.0)
                self.last_lidar_time = time.time()
            except asyncio.CancelledError:
                break
            except Exception as e:
                pass

    async def _gamepad_tcp_check(self):
        while self.is_running:
            try:
                reader, writer = await asyncio.open_connection("127.0.0.1", 9005)
                writer.write(b"GAMEPAD\n")
                await writer.drain()
                data = await asyncio.wait_for(reader.readline(), timeout=0.5)
                status = data.decode("utf-8").strip()
                self.gamepad_ok = (status == "ON")
                writer.close()
                await writer.wait_closed()
            except Exception as e:
                self.gamepad_ok = False
            await asyncio.sleep(1.0)

    async def _control_loop(self):
        # 10 Hz smyčka
        dt = 0.1
        accel_limit = 0.3 * dt # max zrychlení za tik (0.03 m/s)
        decel_limit = 1.0 * dt # max zpomalení za tik (0.1 m/s)
        
        tick_count = 0
        while self.is_running:
            try:
                # 1. Base target speed
                target_base = self.gas_input * self.max_speed
                
                # Pokud není gamepad připojen, natvrdo chceme zastavit
                if not self.gamepad_ok:
                    target_base = 0.0
                    
                # 2. Lidar antikolize
                lidar_active = (time.time() - self.last_lidar_time) < 2.0
                current_lidar = self.lidar_dist if lidar_active else -1.0
                
                lidar_factor = 1.0
                if 0 < current_lidar <= 50.0:
                    lidar_factor = 0.0
                elif 50.0 < current_lidar < 150.0:
                    lidar_factor = (current_lidar - 50.0) / 100.0
                    
                target_base *= lidar_factor
                
                # 3. Aplikace zrychlení pouze na base speed
                diff = target_base - self.current_base_speed
                if diff > accel_limit:
                    self.current_base_speed += accel_limit
                elif diff < -decel_limit:
                    self.current_base_speed -= decel_limit
                else:
                    self.current_base_speed = target_base
                    
                # 4. Steering (bez zrychlení, okamžitá odezva)
                # Kladný X = doprava -> levé kolo přidá, pravé ubere
                steering = self.steering_input * 0.4
                l_speed = self.current_base_speed + steering
                r_speed = self.current_base_speed - steering
                
                # Převod na cm/s
                l_speed_cm = int(round(l_speed * 100))
                r_speed_cm = int(round(r_speed * 100))
                
                # 5. Výpočet PWM
                # 1m/s = 100cm/s => 150 pwm => násobič 1.5
                max_speed_cm = max(abs(l_speed_cm), abs(r_speed_cm))
                pwm = int(max_speed_cm * 1.5)
                
                # 6. Odeslání do DriveClient a ZMQ
                self.drive_client.send_drive(pwm, l_speed_cm, r_speed_cm)
                
                payload = json.dumps({"pwm": pwm, "lspeed": l_speed_cm, "rspeed": r_speed_cm})
                await self.pub.send_multipart([b"DRIVE", payload.encode("utf-8")])
                
                if tick_count % 10 == 0:
                    print(f"[PilotManual] STATS | Gamepad OK: {self.gamepad_ok} | Gas: {self.gas_input:.2f} | Steering: {self.steering_input:.2f} | Lidar: {current_lidar:.1f} cm (factor: {lidar_factor:.2f}) | Max_speed: {self.max_speed:.1f} | Base_speed: {self.current_base_speed:.2f} | DRIVE sent: pwm={pwm}, L={l_speed_cm}, R={r_speed_cm}")
                tick_count += 1
                
            except asyncio.CancelledError:
                break
            except Exception as e:
                print(f"[PilotManual] Control loop error: {e}")
                
            await asyncio.sleep(dt)
