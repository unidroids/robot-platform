#!/usr/bin/env python3
import pygame
import time
import os

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

pygame.init()
pygame.joystick.init()

while pygame.joystick.get_count() == 0:
    print("Čekám na gamepad...")
    time.sleep(1)
    pygame.joystick.quit()
    pygame.joystick.init()

js = pygame.joystick.Joystick(0)
js.init()

print(f"Nalezen gamepad: {js.get_name()}")
print(f"Osy: {js.get_numaxes()}, Tlačítka: {js.get_numbuttons()}, Hats: {js.get_numhats()}")

def wait_for_axis(prompt, threshold=0.7):
    print(f"\n>>> {prompt}")
    print("Čekám na pohyb... (zmáčkni Ctrl+C pro přeskočení)")
    pygame.event.pump()
    baseline = [js.get_axis(i) for i in range(js.get_numaxes())]
    try:
        while True:
            pygame.event.pump()
            for i in range(js.get_numaxes()):
                val = js.get_axis(i)
                # Některé triggery začínají na -1, takže hledáme změnu vůči klidu
                if abs(val - baseline[i]) >= threshold:
                    print(f"Detekována osa {i} s novou hodnotou {val:.2f} (původní {baseline[i]:.2f})")
                    # Počkáme až se vrátí do klidu
                    while abs(js.get_axis(i) - baseline[i]) >= 0.2:
                        pygame.event.pump()
                        time.sleep(0.05)
                    time.sleep(0.2)
                    return i
            time.sleep(0.05)
    except KeyboardInterrupt:
        return None

def wait_for_button(prompt):
    print(f"\n>>> {prompt}")
    print("Čekám na stisk... (zmáčkni Ctrl+C pro přeskočení)")
    try:
        while True:
            pygame.event.pump()
            for i in range(js.get_numbuttons()):
                if js.get_button(i):
                    print(f"Detekováno tlačítko {i}")
                    while js.get_button(i):
                        pygame.event.pump()
                        time.sleep(0.05)
                    time.sleep(0.1)
                    return i
            time.sleep(0.05)
    except KeyboardInterrupt:
        return None

def wait_for_hat(prompt):
    if js.get_numhats() == 0:
        return None
    print(f"\n>>> {prompt}")
    print("Čekám na stisk D-Padu... (zmáčkni Ctrl+C pro přeskočení)")
    try:
        while True:
            pygame.event.pump()
            for i in range(js.get_numhats()):
                val = js.get_hat(i)
                if val != (0, 0):
                    print(f"Detekován hat {i} s hodnotou {val}")
                    while js.get_hat(i) != (0, 0):
                        pygame.event.pump()
                        time.sleep(0.05)
                    time.sleep(0.1)
                    return i
            time.sleep(0.05)
    except KeyboardInterrupt:
        return None

mapping = {}
print("\n--- MAPOVÁNÍ OS ---")
mapping["axis_left_x"] = wait_for_axis("Pohni LEVOU páčkou doprava na maximum")
mapping["axis_left_y"] = wait_for_axis("Pohni LEVOU páčkou dolů na maximum")
mapping["axis_right_x"] = wait_for_axis("Pohni PRAVOU páčkou doprava na maximum")
mapping["axis_right_y"] = wait_for_axis("Pohni PRAVOU páčkou dolů na maximum")
mapping["axis_brake"] = wait_for_axis("Zmáčkni BRZDU (LT/L2) na maximum")
mapping["axis_gas"] = wait_for_axis("Zmáčkni PLYN (RT/R2) na maximum")

print("\n--- MAPOVÁNÍ TLAČÍTEK ---")
btn_names = ["A", "B", "X", "Y", "LB", "RB", "VIEW", "MENU", "HOME", "LSB", "RSB", "EXTRA_BOTTOM", "EXTRA_L", "EXTRA_R"]
for name in btn_names:
    mapping[f"btn_{name}"] = wait_for_button(f"Stiskni tlačítko {name}")

print("\n--- MAPOVÁNÍ D-PAD (Hats) ---")
mapping["hat_dpad"] = wait_for_hat("Stiskni jakýkoliv směr na D-Padu (křížovém ovladači)")

print("\n\n=== VÝSLEDNÉ MAPOVÁNÍ PRO TŘÍDU ===")
print("class GamepadProfile:")
for k, v in mapping.items():
    if v is not None:
        print(f"    {k} = {v}")
