import asyncio
import sys

class OowTcpServer:
    def __init__(self, host, port, ble_server, watchdog, logger_comp, shutdown_event):
        self.host = host
        self.port = port
        self.ble_server = ble_server
        self.watchdog = watchdog
        self.logger_comp = logger_comp
        self.shutdown_event = shutdown_event
        self.server = None
        self.watchdog_task = None
        self.is_running = False

    async def start(self):
        self.server = await asyncio.start_server(self.handle_client, self.host, self.port)
        print(f"[TCP_Server][INFO] Naslouchám na {self.host}:{self.port}")

    async def stop(self):
        if self.server:
            self.server.close()
            await self.server.wait_closed()

    async def handle_client(self, reader, writer):
        addr = writer.get_extra_info('peername')
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
                    writer.write(b"PONG OOW\n")
                    
                elif cmd == "RESTART":
                    writer.write(b"RESTARTING PROCESS\n")
                    await writer.drain()
                    self.shutdown_event.set()
                    break
                        
                elif cmd == "STATUS":
                    status = "RUNNING" if self.is_running else "IDLE"
                    writer.write(f"{status}\n".encode())
                    
                elif cmd == "OOW":
                    status = self.watchdog.get_status()
                    writer.write(f"{status}\n".encode())
                    
                elif cmd == "EXIT":
                    writer.write(b"BYE\n")
                    break
                    
                elif cmd == "SHUTDOWN":
                    writer.write(b"SHUTTING DOWN\n")
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
