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
        ok, resp = send_tcp_command(self.host, self.port, cmd, timeout=self.timeout)
        if not ok:
            print(f"[TerminalClient] Chyba odeslání MESSAGE: {resp}")
        return ok

    def sound(self, name: str) -> bool:
        """
        Přehraje zvuk na HMI telefonu (např. notification, barking, game-over, meow).
        """
        cmd = f"SOUND {name.strip()}"
        ok, resp = send_tcp_command(self.host, self.port, cmd, timeout=self.timeout)
        if not ok:
            print(f"[TerminalClient] Chyba odeslání SOUND {name}: {resp}")
        return ok

    def blink(self, color_hex: str, freq_hz: float, duration_ms: int) -> bool:
        """
        Spustí blikání displeje (např. #FFA500 2 3000).
        """
        cmd = f"BLINK {color_hex.strip()} {freq_hz} {duration_ms}"
        ok, resp = send_tcp_command(self.host, self.port, cmd, timeout=self.timeout)
        if not ok:
            print(f"[TerminalClient] Chyba odeslání BLINK: {resp}")
        return ok
