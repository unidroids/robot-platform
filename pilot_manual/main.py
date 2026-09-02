#!/usr/bin/env python3
import asyncio
import signal
from service import PilotManualService

class PilotManualTcpServer:
    def __init__(self, host, port, service):
        self.host = host
        self.port = port
        self.service = service
        self.server = None
        self.shutdown_event = asyncio.Event()

    async def start(self):
        self.server = await asyncio.start_server(self.handle_client, self.host, self.port)
        print(f"[TCP_Server] Naslouchám na {self.host}:{self.port}")

    async def stop(self):
        if self.server:
            self.server.close()
            await self.server.wait_closed()

    async def handle_client(self, reader, writer):
        addr = writer.get_extra_info("peername")
        print(f"[TCP_Server] Klient připojen: {addr}")
        
        try:
            while not self.shutdown_event.is_set():
                try:
                    data = await asyncio.wait_for(reader.readline(), timeout=2.0)
                except asyncio.TimeoutError:
                    continue
                
                if not data:
                    break
                    
                line = data.decode("utf-8").strip().upper()
                print(f"[TCP_Server] Příkaz: {line}")
                
                parts = line.split()
                if not parts:
                    continue
                cmd = parts[0]
                
                if cmd == "PING":
                    writer.write(b"PONG PILOT_MANUAL\n")
                    
                elif cmd == "START":
                    success = await self.service.start()
                    if success:
                        writer.write(b"OK STARTED\n")
                    else:
                        writer.write(b"ERR INITIALIZATION FAILED (SERVICES NOT RESPONDING)\n")
                        
                elif cmd == "STOP":
                    await self.service.stop()
                    writer.write(b"OK STOPPED\n")

                elif cmd == "STATUS":
                    status = self.service.get_status()
                    writer.write(f"{status}\n".encode())
                    
                elif cmd == "EXIT":
                    writer.write(b"BYE PILOT_MANUAL\n")
                    break
                    
                elif cmd == "SHUTDOWN":
                    writer.write(b"SHUTTING DOWN PILOT_MANUAL\n")
                    await writer.drain()
                    self.shutdown_event.set()
                    break
                    
                else:
                    writer.write(b"ERR Unknown cmd\n")
                    
                await writer.drain()
                
        except Exception as e:
            print(f"[TCP_Server] Chyba klienta {addr}: {e}")
        finally:
            print(f"[TCP_Server] Odpojeno: {addr}")
            writer.close()
            await writer.wait_closed()


async def main():
    loop = asyncio.get_running_loop()
    
    service = PilotManualService()
    tcp_server = PilotManualTcpServer("127.0.0.1", 9103, service)
    
    def signal_handler(sig_name):
        print(f"[Main] System signal {sig_name} received, shutting down...")
        tcp_server.shutdown_event.set()
        
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, signal_handler, sig.name)
    
    try:
        await tcp_server.start()
        print("[Main] Pilot Manual TCP Server is running. Press Ctrl+C to stop.")
        await tcp_server.shutdown_event.wait()
    except asyncio.CancelledError:
        pass
    except Exception as e:
        print(f"[Main] Error: {e}")
    finally:
        for sig in (signal.SIGINT, signal.SIGTERM):
            loop.remove_signal_handler(sig)
            
        await tcp_server.stop()
        await service.stop()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("[Main] Terminated by user.")
