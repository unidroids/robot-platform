import os
import signal
import sys
from pathlib import Path
from dotenv import load_dotenv

from worker import RtkWorker
from server import RtkServer

def main():
    base_path = Path(__file__).parent
    env_path = base_path / ".env"
    load_dotenv(env_path)

    ini_path = base_path / "service.ini"
    
    settings = {}
    if ini_path.exists():
        with open(ini_path, "r") as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                if "=" in line:
                    k, v = line.split("=", 1)
                    settings[k.strip()] = v.strip()
                    
    ntrip_host = settings.get("ENDPOINT", "ppntrip.services.u-blox.com")
    # Ponechal jsem tvůj překlep z INI "MOUNTINPOINT", ale fallbackne na "MOUNTPOINT"
    ntrip_mount = settings.get("MOUNTINPOINT", settings.get("MOUNTPOINT", "NEAR-RTCM"))
    ntrip_port = int(settings.get("HTTPPORT", "2101"))
    
    tls = False # Dle zadání HTTP, takže TLS nepoužíváme
    
    ntrip_user = os.getenv("POINTPERFECT_USER", "")
    ntrip_pass = os.getenv("POINTPERFECT_PASS", "")

    worker = RtkWorker(ntrip_user, ntrip_pass, ntrip_host, ntrip_port, ntrip_mount, tls)
    server = RtkServer("127.0.0.1", 9015, worker)

    def sigint_handler(sig, frame):
        print("\n[MAIN] Přijat signál SIGINT, ukončuji služby...")
        server.stop() # Zastavení serveru uvolní hlavní vlákno ze smyčky

    signal.signal(signal.SIGINT, sigint_handler)

    try:
        server.start()
    finally:
        worker.stop()
        print("[MAIN] Mikroslužba RTK byla úspěšně ukončena.")

if __name__ == "__main__":
    main()
