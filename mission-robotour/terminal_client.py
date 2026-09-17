# mission-robotour/terminal_client.py
from __future__ import annotations
import json
from typing import List, Dict, Optional
try:
    from .microservices import send_tcp_command
except (ImportError, ValueError):
    from microservices import send_tcp_command


class TerminalClient:
    """
    Klient pro komunikaci s aplikací TERMINAL na HMI telefonu (výchozí TCP port 9022).
    Podporuje:
      - MESSAGE <json> : zobrazení dialogu s tlačítky
      - SOUND <name>   : přehrání zvuku (barking, game-over, meow, notification)
      - BLINK <color> <freq> <duration_ms> : vizuální signalizace
    """

    def __init__(self, host: str = "127.0.0.1", port: int = 9022, timeout: float = 2.0):
        self.host = host
        self.port = port
        self.timeout = timeout

    def show_message(self, header: str, text: str, buttons: Optional[List[Dict[str, str]]] = None) -> bool:
        """
        Odešle zprávu do TERMINALu.
        buttons: seznam slovníků tvaru [{"id": "...", "text": "..."}, ...]
        """
        msg_payload = {
            "header": header,
            "text": text,
            "buttons": buttons if buttons is not None else []
        }
        cmd = f"MESSAGE {json.dumps(msg_payload, ensure_ascii=False)}"
        short_text = (text[:40] + "...") if len(text) > 40 else text
        print(f"[TerminalClient] Odesílám MESSAGE '{header}': '{short_text}' (port {self.port})...")
        ok, resp = send_tcp_command(self.host, self.port, cmd, timeout=self.timeout)
        if not ok:
            print(f"[TerminalClient] Chyba odeslání MESSAGE: {resp}")
        else:
            print(f"[TerminalClient] MESSAGE odpověď: ok={ok}, resp='{resp}'")
        return ok

    def hide_message(self, timeout: Optional[float] = 1.0) -> bool:
        """
        Skryje aktivní dialog/zprávu na HMI telefonu (odešle MESSAGE CLEAR).
        """
        cmd = "MESSAGE CLEAR"
        to = timeout if timeout is not None else self.timeout
        print(f"[TerminalClient] Odesílám MESSAGE CLEAR (port {self.port})...")
        ok, resp = send_tcp_command(self.host, self.port, cmd, timeout=to)
        if not ok:
            print(f"[TerminalClient] Chyba odeslání MESSAGE CLEAR: {resp}")
        else:
            print(f"[TerminalClient] MESSAGE CLEAR odpověď: ok={ok}, resp='{resp}'")
        return ok

    def clear_message(self, timeout: Optional[float] = 1.0) -> bool:
        """Alias pro hide_message."""
        return self.hide_message(timeout=timeout)

    def sound(self, name: str) -> bool:
        """
        Přehraje zvuk na HMI telefonu (např. notification, barking, game-over, meow).
        """
        cmd = f"SOUND {name.strip()}"
        print(f"[TerminalClient] Odesílám SOUND '{name}' (port {self.port})...")
        ok, resp = send_tcp_command(self.host, self.port, cmd, timeout=self.timeout)
        if not ok:
            print(f"[TerminalClient] Chyba odeslání SOUND {name}: {resp}")
        else:
            print(f"[TerminalClient] SOUND odpověď: ok={ok}, resp='{resp}'")
        return ok

    def blink(self, color_hex: str, freq_hz: float, duration_ms: int) -> bool:
        """
        Spustí blikání displeje (např. #FFA500 2 3000).
        """
        cmd = f"BLINK {color_hex.strip()} {freq_hz} {duration_ms}"
        print(f"[TerminalClient] Odesílám BLINK {color_hex} (port {self.port})...")
        ok, resp = send_tcp_command(self.host, self.port, cmd, timeout=self.timeout)
        if not ok:
            print(f"[TerminalClient] Chyba odeslání BLINK: {resp}")
        else:
            print(f"[TerminalClient] BLINK odpověď: ok={ok}, resp='{resp}'")
        return ok
