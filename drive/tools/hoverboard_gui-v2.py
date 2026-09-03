#!/usr/bin/env python3
# hoverboard_gui-v2.py
# Desktop GUI for hoverboard communication (Windows-friendly, also works on Linux/Mac)
# - Left pane: RX plain-text lines (entries separated by CRLF)
# - Right pane: controls + log (outgoing messages and errors)
# - TX encodes fixed-length frames: STX,CMD,P1,P2,P3,P4,MTX,CMD,P1,P2,P3,P4,ETX
# - Two threads: RX reader (byte-by-byte -> CRLF lines), TX sender (from FIFO queue)
#
# Commands:
#   cmd=0   HALT (rychlé nouzové zastavení, p1..p4 = 125)
#   cmd=1   STOP motorů / OFF (vypnutí buzení, p1..p4 = 125)
#   cmd=2   START motorů / ON (aktivace buzení, p1..p4 = 125)
#   cmd=3   POWER_OFF (vypnutí napájení, p1..p4 = 125)
#   cmd=4   DRIVE (p1/p2 max_pwm, p3 left_speed, p4 right_speed)
#   cmd=5   BREAK (zastavení / dobrzdění, p1..p4 = 125)
#   cmd=50  PROBE / ECHO (diagnostický dotaz, p1..p4 volitelné / timestamp)
#   cmd=101 PWM (p1/p2 left_pwm, p3/p4 right_pwm)
#   cmd=102 TUNE (kalibrační korekce Lcw, Lccw, Rcw, Rccw)
#   RAW     Přímý rámec (cmd, p1..p4 v rozsahu 0..250)
#
# Requires: pyserial
#   pip install pyserial
#
# Usage:
#   python hoverboard_gui-v2.py
#
from __future__ import annotations

import threading
import queue
import time
import sys
from datetime import datetime
from typing import Tuple
import tkinter as tk
from tkinter import ttk
from tkinter.scrolledtext import ScrolledText

try:
    import serial
except ImportError:
    serial = None

# --- Rámcové konstanty ---
STX = 251
MTX = 252
ETX = 253
P_NEUTRAL = 125
DEFAULT_BAUD = 921600
BASE = 251


# ---------------------- Encoding helpers ----------------------
def _assert_param(v: int) -> None:
    iv = int(v)
    if iv < 0 or iv > 250:
        raise ValueError(f"Param {iv} out of range 0..250")


def build_frame(cmd: int, p1: int, p2: int, p3: int, p4: int) -> bytes:
    """Sestaví 13bajtový binární rámec dle specifikace."""
    for v in (cmd, p1, p2, p3, p4):
        _assert_param(v)
    return bytes([STX, cmd, p1, p2, p3, p4, MTX, cmd, p1, p2, p3, p4, ETX])


def encode_pwm(d: int) -> Tuple[int, int]:
    """Kusová mapovací funkce PWM -> (p1,p2) v rozsahu 0..250.

    d ∈ [-125..375]
      d <= 0:     p1=0,   p2=d+125
      0 < d<=250: p1=d,   p2=125
      d > 250:    p1=250, p2=d-125
    """
    d = int(d)
    if d < -125 or d > 375:
        raise ValueError(f"PWM {d} out of range [-125, 375]")
    if d <= 0:
        p1, p2 = 0, d + 125
    elif d <= 250:
        p1, p2 = d, 125
    else:
        p1, p2 = 250, d - 125
    _assert_param(p1)
    _assert_param(p2)
    return p1, p2


def encode_speed(v: int) -> int:
    """Mapuje rychlost do p3/p4: v ∈ [-50, 200]  =>  p = v + 50 ∈ [0..250]."""
    v = int(v)
    p = v + 50
    _assert_param(p)
    return p


def encode_corr(c: int) -> int:
    """tune correction: c ∈ [-125, 125] -> param = c + 125 ∈ [0..250]."""
    c = int(c)
    p = c + 125
    _assert_param(p)
    return p


def base251_encode_u32(val: int) -> Tuple[int, int, int, int]:
    """base-251 kódování 32bit čísla do p1..p4 (0..250)."""
    v = val % (BASE ** 4)
    d0 = v % BASE; v //= BASE
    d1 = v % BASE; v //= BASE
    d2 = v % BASE; v //= BASE
    d3 = v % BASE
    return d0, d1, d2, d3  # low..high


# ---------------------- GUI App ----------------------
class HoverboardGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Hoverboard UART Tool")
        self.geometry("1100x720")
        self.minsize(980, 620)

        # Shared state
        self.ser = None
        self.stop_event = threading.Event()
        self.tx_queue: queue.Queue = queue.Queue(maxsize=200)
        self.rx_thread = None
        self.tx_thread = None

        # --- Layout: two columns ---
        self.grid_columnconfigure(0, weight=1)  # left pane grows
        self.grid_columnconfigure(1, weight=1)  # right pane grows
        self.grid_rowconfigure(0, weight=1)     # main row grows

        # Left: RX
        left_frame = ttk.Frame(self)
        left_frame.grid(row=0, column=0, sticky="nsew", padx=(8, 4), pady=8)
        left_frame.grid_rowconfigure(1, weight=1)
        left_frame.grid_columnconfigure(0, weight=1)

        ttk.Label(left_frame, text="RX (plain text; lines separated by CRLF)").grid(row=0, column=0, sticky="w")
        self.rx_text = ScrolledText(left_frame, wrap="none", height=10)
        self.rx_text.grid(row=1, column=0, sticky="nsew", pady=(4, 8))

        # Right: Controls + Log
        right_frame = ttk.Frame(self)
        right_frame.grid(row=0, column=1, sticky="nsew", padx=(4, 8), pady=8)
        right_frame.grid_columnconfigure(0, weight=1)
        right_frame.grid_rowconfigure(2, weight=1)

        # Port controls
        port_frame = ttk.Frame(right_frame)
        port_frame.grid(row=0, column=0, sticky="ew", pady=(0, 6))
        port_frame.grid_columnconfigure(6, weight=1)

        ttk.Label(port_frame, text="Port:").grid(row=0, column=0, sticky="w")
        self.port_var = tk.StringVar(value="COM7")
        self.port_entry = ttk.Entry(port_frame, textvariable=self.port_var, width=10)
        self.port_entry.grid(row=0, column=1, padx=(4, 8))

        ttk.Label(port_frame, text="Baud:").grid(row=0, column=2, sticky="w")
        self.baud_var = tk.StringVar(value=str(DEFAULT_BAUD))
        self.baud_combo = ttk.Combobox(
            port_frame,
            textvariable=self.baud_var,
            values=["921600", "115200", "57600", "38400"],
            width=9,
        )
        self.baud_combo.grid(row=0, column=3, padx=(4, 10))

        self.start_btn = ttk.Button(port_frame, text="Open Port", command=self.on_start)
        self.start_btn.grid(row=0, column=4, padx=(0, 6))
        self.stop_btn = ttk.Button(port_frame, text="Close Port", command=self.on_stop, state="disabled")
        self.stop_btn.grid(row=0, column=5, sticky="w")

        # Command panel
        cmds = ttk.LabelFrame(right_frame, text="Commands (Hoverboard Protocol)")
        cmds.grid(row=1, column=0, sticky="ew", pady=(0, 6))
        for i in range(8):
            cmds.grid_columnconfigure(i, weight=0)
        cmds.grid_columnconfigure(7, weight=1)

        # 1) State / Control Quick Buttons
        state_frame = ttk.LabelFrame(cmds, text="State / Control Commands (p1..p4 = 125)")
        state_frame.grid(row=0, column=0, columnspan=8, sticky="ew", padx=4, pady=(2, 6))

        self.btn_halt = ttk.Button(state_frame, text="HALT (cmd=0)", command=self.send_halt)
        self.btn_halt.grid(row=0, column=0, padx=3, pady=3)

        self.btn_break = ttk.Button(state_frame, text="BREAK (cmd=5)", command=self.send_brake)
        self.btn_break.grid(row=0, column=1, padx=3, pady=3)

        self.btn_stop = ttk.Button(state_frame, text="STOP / OFF (cmd=1)", command=self.send_stop)
        self.btn_stop.grid(row=0, column=2, padx=3, pady=3)

        self.btn_start = ttk.Button(state_frame, text="START / ON (cmd=2)", command=self.send_start)
        self.btn_start.grid(row=0, column=3, padx=3, pady=3)

        self.btn_power_off = ttk.Button(state_frame, text="POWER OFF (cmd=3)", command=self.send_power_off)
        self.btn_power_off.grid(row=0, column=4, padx=3, pady=3)

        # 2) drive (cmd=4) - PWM (-125..375), L Speed (-50..200), R Speed (-50..200)
        self.drive_pwm = tk.StringVar(value="0")
        self.drive_left_speed = tk.StringVar(value="0")
        self.drive_right_speed = tk.StringVar(value="0")
        ttk.Label(cmds, text="drive (cmd=4)").grid(row=1, column=0, sticky="w", padx=2, pady=2)
        ttk.Label(cmds, text="PWM").grid(row=1, column=1, sticky="e")
        ttk.Entry(cmds, textvariable=self.drive_pwm, width=6).grid(row=1, column=2, padx=2)
        ttk.Label(cmds, text="L Speed").grid(row=1, column=3, sticky="e")
        ttk.Entry(cmds, textvariable=self.drive_left_speed, width=6).grid(row=1, column=4, padx=2)
        ttk.Label(cmds, text="R Speed").grid(row=1, column=5, sticky="e")
        ttk.Entry(cmds, textvariable=self.drive_right_speed, width=6).grid(row=1, column=6, padx=2)
        ttk.Button(cmds, text="Send", command=self.send_drive).grid(row=1, column=7, padx=4, sticky="w")

        # 3) pwm (cmd=101) - left/right PWM (-125..375)
        self.pwm_left = tk.StringVar(value="0")
        self.pwm_right = tk.StringVar(value="0")
        ttk.Label(cmds, text="pwm (cmd=101)").grid(row=2, column=0, sticky="w", padx=2, pady=2)
        ttk.Label(cmds, text="LeftPWM").grid(row=2, column=1, sticky="e")
        ttk.Entry(cmds, textvariable=self.pwm_left, width=6).grid(row=2, column=2, padx=2)
        ttk.Label(cmds, text="RightPWM").grid(row=2, column=3, sticky="e")
        ttk.Entry(cmds, textvariable=self.pwm_right, width=6).grid(row=2, column=4, padx=2)
        ttk.Button(cmds, text="Send", command=self.send_pwm).grid(row=2, column=7, padx=4, sticky="w")

        # 4) tune (cmd=102) - 4x correction (-125..125)
        self.tune_vals = [tk.StringVar(value="0") for _ in range(4)]
        ttk.Label(cmds, text="tune (cmd=102) Lcw Lccw Rcw Rccw").grid(row=3, column=0, sticky="w", padx=2, pady=2)
        for i in range(4):
            ttk.Entry(cmds, textvariable=self.tune_vals[i], width=6).grid(row=3, column=1 + i, padx=2)
        ttk.Button(cmds, text="Send", command=self.send_tune).grid(row=3, column=7, padx=4, sticky="w")

        # 5) probe (cmd=50) - p1..p4 (0..250) or base251 timestamp
        self.probe_p = [tk.StringVar(value="0") for _ in range(4)]
        ttk.Label(cmds, text="probe (cmd=50) p1..p4").grid(row=4, column=0, sticky="w", padx=2, pady=2)
        for i in range(4):
            ttk.Entry(cmds, textvariable=self.probe_p[i], width=6).grid(row=4, column=1 + i, padx=2)
        probe_btn_frame = ttk.Frame(cmds)
        probe_btn_frame.grid(row=4, column=7, padx=4, sticky="w")
        ttk.Button(probe_btn_frame, text="Send", command=self.send_probe, width=6).pack(side="left")
        ttk.Button(probe_btn_frame, text="Fill TS", command=self.fill_probe_ts, width=6).pack(side="left", padx=(3, 0))

        # 6) raw - direct values (cmd, p1..p4 in 0..250)
        self.raw_vals = [tk.StringVar(value="0") for _ in range(5)]
        ttk.Label(cmds, text="raw (cmd p1 p2 p3 p4)").grid(row=5, column=0, sticky="w", padx=2, pady=2)
        for i in range(5):
            ttk.Entry(cmds, textvariable=self.raw_vals[i], width=5).grid(row=5, column=1 + i, padx=2)
        ttk.Button(cmds, text="Send", command=self.send_raw).grid(row=5, column=7, padx=4, sticky="w")

        # Log pane (right-bottom)
        ttk.Label(right_frame, text="Log (actions, errors, outgoing frames, ACK/NACK)").grid(
            row=2, column=0, sticky="w", pady=(6, 0)
        )
        self.log_text = ScrolledText(right_frame, wrap="word", height=10)
        self.log_text.grid(row=3, column=0, sticky="nsew", pady=(4, 0))
        right_frame.grid_rowconfigure(3, weight=1)

        # Footer status
        self.status_var = tk.StringVar(value="Stopped")
        status_bar = ttk.Label(self, textvariable=self.status_var, relief="sunken", anchor="w")
        status_bar.grid(row=1, column=0, columnspan=2, sticky="ew")

        # Close handling
        self.protocol("WM_DELETE_WINDOW", self.on_close)

        # Update UI loop
        self.after(500, self._tick_status)

    # ---------------------- UI logging helpers ----------------------
    def log_right(self, msg: str):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        def _append():
            self.log_text.insert("end", f"{ts} | {msg}\n")
            self.log_text.see("end")
        self.log_text.after(0, _append)

    def log_left(self, msg: str):
        def _append():
            self.rx_text.insert("end", msg + "\n")
            self.rx_text.see("end")
        self.rx_text.after(0, _append)

    def set_status(self, text: str):
        self.status_var.set(text)

    # ---------------------- Port control ----------------------
    def on_start(self):
        if serial is None:
            self.log_right("pyserial not installed. Run: pip install pyserial")
            return
        if self.ser is not None:
            self.log_right("Already running.")
            return
        port = self.port_var.get().strip()
        if not port:
            self.log_right("Please enter serial port (e.g., COM7)")
            return
        try:
            baud = int(self.baud_var.get().strip() or str(DEFAULT_BAUD))
        except ValueError:
            baud = DEFAULT_BAUD
        try:
            self.ser = serial.Serial(port, baud, timeout=0.05)
        except Exception as e:
            self.log_right(f"[ERR] Opening port '{port}' @{baud}: {e}")
            self.ser = None
            return
        self.log_right(f"[OK] Opened port {port} @{baud}")
        self.stop_event.clear()

        # Spawn RX thread
        self.rx_thread = threading.Thread(target=self._rx_loop, name="RX", daemon=True)
        self.rx_thread.start()

        # Spawn TX thread
        self.tx_thread = threading.Thread(target=self._tx_loop, name="TX", daemon=True)
        self.tx_thread.start()

        self.start_btn.configure(state="disabled")
        self.stop_btn.configure(state="normal")
        self.set_status(f"Running on {port} @{baud}")

    def on_stop(self):
        self.stop_event.set()
        time.sleep(0.1)
        try:
            if self.ser is not None:
                self.ser.close()
        except Exception:
            pass
        self.ser = None
        self.start_btn.configure(state="normal")
        self.stop_btn.configure(state="disabled")
        self.set_status("Stopped")
        self.log_right("[OK] Port closed, threads stopping...")

    def on_close(self):
        self.on_stop()
        time.sleep(0.2)
        self.destroy()

    # ---------------------- RX: read bytes, emit lines on CRLF ----------------------
    def _rx_loop(self):
        buf = bytearray()
        while not self.stop_event.is_set():
            try:
                b = self.ser.read(1) if self.ser else b""
            except Exception as e:
                self.log_right(f"[RX ERR] {e}")
                time.sleep(0.1)
                continue
            if not b:
                continue
            ch = b[0]
            if ch == 0x0D:  # CR
                # wait for LF
                continue
            if ch == 0x0A:  # LF
                line = buf.decode("utf-8", errors="replace")
                self.log_left(line)
                self._handle_rx_line(line)
                buf.clear()
                continue
            buf.append(ch)
            if len(buf) > 4096:
                line = buf.decode("utf-8", errors="replace")
                self.log_left(line)
                self._handle_rx_line(line)
                buf.clear()

    def _handle_rx_line(self, line: str):
        """Zpracuje příchozí řádku a zvýrazní potvrzení ACK/NACK v pravém logu."""
        s = line.strip()
        if not s:
            return
        if s.startswith("$IAM"):
            self.log_right(f"[RX ACK] {s}")
        elif s.startswith("$INM"):
            self.log_right(f"[RX NACK] {s}")

    # ---------------------- TX: take frames from queue and write ----------------------
    def _tx_loop(self):
        while not self.stop_event.is_set():
            try:
                item = self.tx_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            if item is None:
                continue
            try:
                frame_desc, frame_bytes = item
                if self.ser is None:
                    self.log_right("[TX ERR] Port not open. Discarding frame.")
                    continue
                self.ser.write(frame_bytes)
                self.log_right(f"[TX OK] {frame_desc} | hex: {' '.join(f'{b:02X}' for b in frame_bytes)}")
            except Exception as e:
                self.log_right(f"[TX ERR] {e}")

    # ---------------------- Command handlers (enqueue) ----------------------
    def send_halt(self):
        try:
            cmd = 0
            p1 = p2 = p3 = p4 = P_NEUTRAL
            desc = f"halt cmd={cmd} p1={p1} p2={p2} p3={p3} p4={p4} (HALT/nouzové zastavení)"
            self.log_right(desc)
            frame = build_frame(cmd, p1, p2, p3, p4)
            self.tx_queue.put((desc, frame))
        except Exception as e:
            self.log_right(f"[halt ERR] {e}")

    def send_brake(self):
        try:
            cmd = 5
            p1 = p2 = p3 = p4 = P_NEUTRAL
            desc = f"break cmd={cmd} p1={p1} p2={p2} p3={p3} p4={p4} (BREAK/zastavení)"
            self.log_right(desc)
            frame = build_frame(cmd, p1, p2, p3, p4)
            self.tx_queue.put((desc, frame))
        except Exception as e:
            self.log_right(f"[brake ERR] {e}")

    def send_stop(self):
        try:
            cmd = 1
            p1 = p2 = p3 = p4 = P_NEUTRAL
            desc = f"stop cmd={cmd} p1={p1} p2={p2} p3={p3} p4={p4} (STOP motorů / OFF)"
            self.log_right(desc)
            frame = build_frame(cmd, p1, p2, p3, p4)
            self.tx_queue.put((desc, frame))
        except Exception as e:
            self.log_right(f"[stop ERR] {e}")

    def send_start(self):
        try:
            cmd = 2
            p1 = p2 = p3 = p4 = P_NEUTRAL
            desc = f"start cmd={cmd} p1={p1} p2={p2} p3={p3} p4={p4} (START motorů / ON)"
            self.log_right(desc)
            frame = build_frame(cmd, p1, p2, p3, p4)
            self.tx_queue.put((desc, frame))
        except Exception as e:
            self.log_right(f"[start ERR] {e}")

    def send_power_off(self):
        try:
            cmd = 3
            p1 = p2 = p3 = p4 = P_NEUTRAL
            desc = f"power_off cmd={cmd} p1={p1} p2={p2} p3={p3} p4={p4} (POWER OFF/vypnutí napájení)"
            self.log_right(desc)
            frame = build_frame(cmd, p1, p2, p3, p4)
            self.tx_queue.put((desc, frame))
        except Exception as e:
            self.log_right(f"[power_off ERR] {e}")

    def send_drive(self):
        try:
            pwm = int(self.drive_pwm.get().strip() or "0")
            lspd = int(self.drive_left_speed.get().strip() or "0")
            rspd = int(self.drive_right_speed.get().strip() or "0")
            p1, p2 = encode_pwm(pwm)
            p3 = encode_speed(lspd)
            p4 = encode_speed(rspd)
            cmd = 4
            desc = f"drive cmd={cmd} pwm={pwm}->(p1={p1},p2={p2}) L_spd={lspd}->p3={p3} R_spd={rspd}->p4={p4}"
            self.log_right(desc)
            frame = build_frame(cmd, p1, p2, p3, p4)
            self.tx_queue.put((desc, frame))
        except Exception as e:
            self.log_right(f"[drive ERR] {e}")

    def send_pwm(self):
        try:
            lp = int(self.pwm_left.get().strip() or "0")
            rp = int(self.pwm_right.get().strip() or "0")
            lp1, lp2 = encode_pwm(lp)
            rp1, rp2 = encode_pwm(rp)
            cmd = 101
            desc = f"pwm cmd={cmd} left={lp}->(p1={lp1},p2={lp2}) right={rp}->(p3={rp1},p4={rp2})"
            self.log_right(desc)
            frame = build_frame(cmd, lp1, lp2, rp1, rp2)
            self.tx_queue.put((desc, frame))
        except Exception as e:
            self.log_right(f"[pwm ERR] {e}")

    def send_tune(self):
        try:
            vals = [int(v.get().strip() or "0") for v in self.tune_vals]
            p = [encode_corr(v) for v in vals]
            cmd = 102
            desc = f"tune cmd={cmd} Lcw={vals[0]} Lccw={vals[1]} Rcw={vals[2]} Rccw={vals[3]} -> (p1..p4)={p}"
            self.log_right(desc)
            frame = build_frame(cmd, p[0], p[1], p[2], p[3])
            self.tx_queue.put((desc, frame))
        except Exception as e:
            self.log_right(f"[tune ERR] {e}")

    def fill_probe_ts(self):
        """Vyplní parametry p1..p4 base251 časovou značkou (mikrosekundy)."""
        t_us = int(time.monotonic() * 1_000_000)
        t_mod = t_us % (BASE ** 4)
        p1, p2, p3, p4 = base251_encode_u32(t_mod)
        for i, val in enumerate((p1, p2, p3, p4)):
            self.probe_p[i].set(str(val))

    def send_probe(self):
        try:
            p = [int(v.get().strip() or "0") for v in self.probe_p]
            cmd = 50
            desc = f"probe cmd={cmd} p1={p[0]} p2={p[1]} p3={p[2]} p4={p[3]}"
            self.log_right(desc)
            frame = build_frame(cmd, p[0], p[1], p[2], p[3])
            self.tx_queue.put((desc, frame))
        except Exception as e:
            self.log_right(f"[probe ERR] {e}")

    def send_raw(self):
        try:
            cv = [int(v.get().strip() or "0") for v in self.raw_vals]
            cmd, p1, p2, p3, p4 = cv
            desc = f"raw cmd={cmd} p1={p1} p2={p2} p3={p3} p4={p4}"
            self.log_right(desc)
            frame = build_frame(cmd, p1, p2, p3, p4)
            self.tx_queue.put((desc, frame))
        except Exception as e:
            self.log_right(f"[raw ERR] {e}")

    # ---------------------- Status ticker ----------------------
    def _tick_status(self):
        qsz = self.tx_queue.qsize()
        ser_open = self.ser is not None
        self.set_status(f"{'Running' if ser_open else 'Stopped'} | TX queue: {qsz}")
        self.after(500, self._tick_status)


if __name__ == "__main__":
    app = HoverboardGUI()
    app.mainloop()
