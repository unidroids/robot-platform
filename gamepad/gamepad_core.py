#!/usr/bin/env python3
import os, time, asyncio, pygame

POLL_PERIOD_SEC = 0.025
DEADZONE_WHEELS = 5
DEADZONE_SPEED  = 5
STEER_RANGE     = 40
SELECT_BUTTONS  = {10}

joystick = None
mode = "DRIVE"
is_running = False
watchdog = None

axes = {'axis_0': 0.0, 'axis_1': 0.0, 'axis_2': 0.0, 'axis_3': 0.0, 'axis_4': 0.0, 'axis_5': 0.0}
buttons = {'b_0': 0, 'b_1': 0, 'b_2': 0, 'b_3': 0, 'b_4': 0}
last_button = None

left_wheel = 0
right_wheel = 0

last_ts = 0.0
msg_index = 0

def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v

def apply_deadzone_int(v, dz):
    return 0 if -dz <= v <= dz else v

def scale_to_int(x, out_max):
    return int(round(x * out_max))

def init_gamepad():
    global joystick
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() > 0:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print(f"[GAMEPAD] Používám: {joystick.get_name()} (axes={joystick.get_numaxes()})")
        return True
    else:
        joystick = None
        print("[GAMEPAD] Upozornění: Joystick nenalezen, poběží v nulových hodnotách.")
        return False

def read_axes_and_buttons():
    global last_button, mode, axes
    if joystick:
        for e in pygame.event.get():
            if e.type == pygame.JOYBUTTONDOWN:
                last_button = f"b_{e.button}"
                buttons[last_button] = 1
                if e.button in SELECT_BUTTONS:
                    mode = "WHEEL" if mode == "DRIVE" else "DRIVE"
            if e.type == pygame.JOYBUTTONUP:
                last_button = f"b_{e.button}"
                buttons[last_button] = 0

        a0 = joystick.get_axis(0) if joystick.get_numaxes() > 0 else 0.0
        a1 = joystick.get_axis(1) if joystick.get_numaxes() > 1 else 0.0
        a2 = joystick.get_axis(2) if joystick.get_numaxes() > 2 else 0.0
        a3 = joystick.get_axis(3) if joystick.get_numaxes() > 3 else 0.0
        a4 = joystick.get_axis(4) if joystick.get_numaxes() > 4 else 0.0
        a5 = joystick.get_axis(5) if joystick.get_numaxes() > 5 else 0.0
    else:
        pygame.event.pump()
        a0 = a1 = a2 = a3 = a4 = a5 = 0.0

    axes['axis_0'] = a0
    axes['axis_1'] = a1
    axes['axis_2'] = a2
    axes['axis_3'] = a3
    axes['axis_4'] = a4
    axes['axis_5'] = a5

def compute_wheels_from_axes():
    global left_wheel, right_wheel
    ly = axes['axis_5']
    ry = axes['axis_4']
    l = apply_deadzone_int(scale_to_int(ly, 50)+50, DEADZONE_WHEELS)
    r = apply_deadzone_int(scale_to_int(ry, 50)+50, DEADZONE_WHEELS)
    left_wheel  = clamp(l, 0, 100)
    right_wheel = clamp(r, 0, 100)

def compute_drive_from_axes():
    global left_wheel, right_wheel
    speed = apply_deadzone_int(scale_to_int(axes['axis_4'], 60)+60, DEADZONE_SPEED)
    steer = scale_to_int(axes['axis_2'], STEER_RANGE)
    boost = 0
    left_wheel  = clamp(speed + steer + boost, -50, 375)
    right_wheel = clamp(speed - steer + boost, -50, 375)

def build_payload():
    return f"PWM {left_wheel} {right_wheel}#I:{msg_index} M:{mode} B:{last_button} T:{time.time()} A:{dict(axes)}"

async def compute_loop():
    global last_ts, msg_index, is_running
    print("[GAMEPAD] Vlákno výpočtů START")
    try:
        while is_running:
            read_axes_and_buttons()
            if mode == "WHEEL":
                compute_wheels_from_axes()
            else:
                compute_drive_from_axes()
            
            last_ts = time.time()
            msg_index += 1
            payload = build_payload()
            payload_buttons = f"{buttons['b_0']} {buttons['b_1']} {buttons['b_2']} {buttons['b_3']} {buttons['b_4']}"
            
            if watchdog:
                await watchdog.publish("AXES", payload)
                await watchdog.publish("BUTTONS", payload_buttons)
                
            await asyncio.sleep(POLL_PERIOD_SEC)
    except Exception as e:
        print("[GAMEPAD] Vlákno výpočtů Error:", e)            
    finally:
        is_running = False
        print("[GAMEPAD] Vlákno výpočtů STOP")

def start_compute_once(wd):
    global is_running, msg_index, watchdog
    if is_running:
        return False
    msg_index = 0
    is_running = True
    watchdog = wd
    asyncio.create_task(compute_loop())
    return True

def stop_all():
    global is_running
    is_running = False
