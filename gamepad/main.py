import asyncio
import sys
import json
import signal

from watchdog import GamepadWatchdog
from gamepad_service import GamepadService

class GamepadTcpServer:
    def __init__(self, host, port, watchdog, service):
        self.host = host
        self.port = port
        self.watchdog = watchdog
        self.service = service
        self.server = None
        self.is_running = False
        self.shutdown_event = asyncio.Event()

    async def start(self):
        self.server = await asyncio.start_server(self.handle_client, self.host, self.port)
        print(f"[TCP_Server][INFO] Naslouchám na {self.host}:{self.port}")

    async def stop(self):
        if self.server:
            self.server.close()
            await self.server.wait_closed()

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
                    self.service.stop()
                    writer.write(b"OK STOPPED\n")

                elif cmd == "STATUS":
                    status = "RUNNING" if self.service.is_running else "IDLE"
                    hw_status = self.watchdog.get_status()
                    writer.write(f'{status} {{"gamepad":"{hw_status}"}}\n'.encode())
                    
                elif cmd == "GAMEPAD":
                    status = self.watchdog.get_status()
                    writer.write(f"{status}\n".encode())
                    
                elif cmd == "BUTTONS":
                    status = self.watchdog.get_status()
                    if status == "ON" and self.service.is_running:
                        states = self.watchdog.button_states.copy()
                        writer.write(f"{json.dumps(states)}\n".encode())
                    else:
                        writer.write(b'{}\n')
                    
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
            writer.close()
            await writer.wait_closed()

async def main():
    loop = asyncio.get_running_loop()
    
    watchdog = GamepadWatchdog()
    service = GamepadService(watchdog)
    tcp_server = GamepadTcpServer(host="127.0.0.1", port=9005, watchdog=watchdog, service=service)
    
    def signal_handler(sig_name):
        print(f"[Main][INFO] System signal {sig_name} received, shutting down...")
        tcp_server.shutdown_event.set()
        
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, signal_handler, sig.name)
    
    print("[Main][INFO] Starting Gamepad Service...")
    
    try:
        await tcp_server.start()
        tcp_server.is_running = True
        
        watchdog_task = asyncio.create_task(watchdog.run_loop())
        
        print("[Main][INFO] Gamepad TCP Server is running. Press Ctrl+C to stop.")
        
        await tcp_server.shutdown_event.wait()
        
    except asyncio.CancelledError:
        print("[Main][INFO] Shutdown signal received.")
    except Exception as e:
        print(f"[Main][ERROR] Error in main loop: {e}")
    finally:
        print("[Main][INFO] Shutting down...")
        for sig in (signal.SIGINT, signal.SIGTERM):
            loop.remove_signal_handler(sig)
            
        await tcp_server.stop()
        tcp_server.is_running = False
        watchdog.is_running = False
        service.stop()
        
        if "watchdog_task" in locals():
            try:
                watchdog_task.cancel()
                await watchdog_task
            except asyncio.CancelledError:
                pass

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("[Main][INFO] Program terminated by user.")
