import asyncio
import sys
import json
import signal

from publisher import GamepadPublisher
from watchdog import GamepadWatchdog
from gamepad_service import GamepadService

class GamepadTcpServer:
    def __init__(self, host: str, port: int, service: GamepadService):
        self.host = host
        self.port = port
        self.service = service
        self.server = None
        self.is_running = False
        self.shutdown_event = asyncio.Event()

    async def start(self):
        self.server = await asyncio.start_server(self.handle_client, self.host, self.port)
        self.is_running = True
        print(f"[TCP_Server][INFO] Naslouchám na {self.host}:{self.port}")

    async def stop(self):
        if self.server:
            self.server.close()
            await self.server.wait_closed()
            self.server = None
        self.is_running = False

    async def handle_client(self, reader, writer):
        addr = writer.get_extra_info("peername")
        print(f"[TCP_Server][INFO] Klient připojen: {addr}")

        try:
            while not self.shutdown_event.is_set():
                try:
                    data = await asyncio.wait_for(reader.readline(), timeout=2.0)
                except asyncio.TimeoutError:
                    continue

                if not data:
                    break

                line = data.decode("utf-8").strip().upper()
                print(f"[TCP_Server][INFO] Příkaz od {addr}: {line}")

                parts = line.split()
                if not parts:
                    continue
                cmd = parts[0]

                if cmd == "PING":
                    writer.write(b"PONG GAMEPAD\n")

                elif cmd == "START":
                    if self.service.start():
                        writer.write(b"OK STARTED\n")
                    else:
                        writer.write(b"ALREADY RUNNING\n")

                elif cmd == "STOP":
                    await self.service.stop()
                    writer.write(b"OK STOPPED\n")

                elif cmd == "STATUS":
                    status_info = self.service.get_status_info()
                    status_word = status_info["service"]
                    writer.write(f"{status_word} {json.dumps(status_info)}\n".encode("utf-8"))

                elif cmd == "GAMEPAD":
                    status = self.service.watchdog.get_status()
                    writer.write(f"{status}\n".encode("utf-8"))

                elif cmd == "BUTTONS":
                    if self.service.is_running:
                        states = self.service.get_button_states()
                        writer.write(f"{json.dumps(states)}\n".encode("utf-8"))
                    else:
                        writer.write(b"{}\n")

                elif cmd == "EXIT":
                    writer.write(b"BYE GAMEPAD\n")
                    break

                elif cmd == "SHUTDOWN":
                    writer.write(b"SHUTTING DOWN GAMEPAD\n")
                    await writer.drain()
                    self.shutdown_event.set()
                    break

                else:
                    writer.write(b"ERR Unknown cmd\n")

                await writer.drain()

        except Exception as e:
            print(f"[TCP_Server][ERROR] Chyba klienta {addr}: {e}")
        finally:
            print(f"[TCP_Server][INFO] Odpojeno: {addr}")
            try:
                writer.close()
                await writer.wait_closed()
            except Exception:
                pass

async def main():
    loop = asyncio.get_running_loop()

    publisher = GamepadPublisher(endpoint="ipc:///tmp/robot-gamepad")
    watchdog = GamepadWatchdog(publisher=publisher)
    service = GamepadService(watchdog=watchdog, publisher=publisher)
    tcp_server = GamepadTcpServer(host="127.0.0.1", port=9005, service=service)

    def signal_handler(sig_name):
        print(f"[Main][INFO] Systémový signál {sig_name} zachycen, ukončuji...")
        tcp_server.shutdown_event.set()

    if sys.platform != "win32":
        for sig in (signal.SIGINT, signal.SIGTERM):
            loop.add_signal_handler(sig, signal_handler, sig.name)

    print("[Main][INFO] Spouštím Gamepad Service...")

    try:
        await tcp_server.start()
        print("[Main][INFO] Gamepad TCP Server běží v režimu IDLE. Čekám na příkaz START...")
        await tcp_server.shutdown_event.wait()

    except asyncio.CancelledError:
        print("[Main][INFO] Obdržen signál pro vypnutí.")
    except Exception as e:
        print(f"[Main][ERROR] Chyba v hlavní smyčce: {e}")
    finally:
        print("[Main][INFO] Ukončuji službu...")
        if sys.platform != "win32":
            for sig in (signal.SIGINT, signal.SIGTERM):
                loop.remove_signal_handler(sig)

        await service.stop()
        await tcp_server.stop()
        print("[Main][INFO] Gamepad Service úspěšně zastavena.")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("[Main][INFO] Program ukončen uživatelem.")
