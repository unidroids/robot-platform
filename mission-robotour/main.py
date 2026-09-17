# mission-robotour/main.py
"""
Vstupní bod síťové služby MISSION-ROBOTOUR na TCP portu 9031.
Spuštění:
  python mission-robotour/main.py [--port 9031] [--host 127.0.0.1]
"""
from __future__ import annotations
import argparse
import asyncio
import json
import signal
import sys

try:
    from .service import MissionRobotourService
except (ImportError, ValueError):
    from service import MissionRobotourService

DEFAULT_PORT = 9031


async def handle_client(reader: asyncio.StreamReader, writer: asyncio.StreamWriter, service: MissionRobotourService):
    addr = writer.get_extra_info("peername")
    print(f"[MISSION-TCP] Klient připojen: {addr}")

    try:
        while True:
            try:
                data = await asyncio.wait_for(reader.readline(), timeout=2.0)
                if not data:
                    break
                line = data.decode("utf-8").strip()
                if not line:
                    continue

                parts = line.split()
                cmd = parts[0].upper()
                print(f"[MISSION-TCP] Zpracovávám příkaz: {cmd}")

                if cmd == "PING":
                    writer.write(b"PONG MISSION_ROBOTOUR\n")

                elif cmd == "START":
                    ok, msg = service.start_mission()
                    if ok:
                        writer.write(b"OK\n")
                    else:
                        writer.write(f"ERROR {msg}\n".encode("utf-8"))

                elif cmd == "STOP":
                    ok, msg = service.stop_mission()
                    if ok:
                        writer.write(b"OK\n")
                    else:
                        writer.write(f"ERROR {msg}\n".encode("utf-8"))

                elif cmd == "STATUS":
                    st = service.get_status_dict()
                    writer.write(f"{json.dumps(st, ensure_ascii=False)}\n".encode("utf-8"))

                elif cmd == "EXIT":
                    writer.write(b"BYE\n")
                    await writer.drain()
                    break

                elif cmd == "SHUTDOWN":
                    writer.write(b"OK SHUTDOWN\n")
                    await writer.drain()
                    service.shutdown()
                    sys.exit(0)

                else:
                    writer.write(b"ERROR UNKNOWN_CMD\n")

                await writer.drain()

            except asyncio.TimeoutError:
                pass

    except Exception as e:
        print(f"[MISSION-TCP] Klient chyba: {e}")
    finally:
        print(f"[MISSION-TCP] Klient odpojen: {addr}")
        try:
            writer.close()
            await writer.wait_closed()
        except Exception:
            pass


async def amain(host: str, port: int):
    service = MissionRobotourService(host=host)

    server = await asyncio.start_server(
        lambda r, w: handle_client(r, w, service),
        host, port
    )

    print(f"[MISSION-ROBOTOUR] Služba naslouchá na {host}:{port}")

    loop = asyncio.get_running_loop()

    def handle_signal():
        print("\n[MISSION-ROBOTOUR] Zachycen signál ukončení, zastavuji...")
        service.shutdown()
        server.close()
        sys.exit(0)

    try:
        loop.add_signal_handler(signal.SIGINT, handle_signal)
        loop.add_signal_handler(signal.SIGTERM, handle_signal)
    except NotImplementedError:
        # Windows nepodporuje loop.add_signal_handler pro všechny signály
        signal.signal(signal.SIGINT, lambda s, f: handle_signal())
        signal.signal(signal.SIGTERM, lambda s, f: handle_signal())

    async with server:
        await server.serve_forever()


def main():
    parser = argparse.ArgumentParser(description="Unidroids Robot Platform - MISSION-ROBOTOUR Service")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help=f"TCP port (výchozí: {DEFAULT_PORT})")
    parser.add_argument("--host", type=str, default="127.0.0.1", help="Listen host (výchozí: 127.0.0.1)")
    args = parser.parse_args()

    try:
        asyncio.run(amain(args.host, args.port))
    except (KeyboardInterrupt, SystemExit):
        print("[MISSION-ROBOTOUR] Služba ukončena.")


if __name__ == "__main__":
    main()
