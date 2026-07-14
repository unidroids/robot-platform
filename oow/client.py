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
                    
                elif cmd == "START":
                    if not self.is_running:
                        try:
                            self.logger_comp.start()
                            await self.ble_server.start()
                            self.watchdog_task = asyncio.create_task(self.watchdog.run_loop())
                            self.is_running = True
                            writer.write(b"OK STARTED\n")
                        except Exception as e:
                            print(f"[TCP_Server][ERROR] Start failed: {e}")
                            self.logger_comp.stop()
                            try:
                                await self.ble_server.stop()
                            except Exception:
                                pass
                            self.is_running = False
                            writer.write(f"ERR {str(e)}\n".encode())
                    else:
                        writer.write(b"OK ALREADY RUNNING\n")
                        
                elif cmd == "STOP":
                    if self.is_running:
                        try:
                            self.is_running = False
                            self.watchdog.is_running = False
                            if self.watchdog_task:
                                await self.watchdog_task
                                self.watchdog_task = None
                            await self.ble_server.stop()
                            self.logger_comp.stop()
                            await self.watchdog.emit_off()
                            writer.write(b"OK STOPPED\n")
                        except Exception as e:
                            print(f"[TCP_Server][ERROR] Stop failed: {e}")
                            writer.write(f"ERR {str(e)}\n".encode())
                    else:
                        writer.write(b"OK ALREADY STOPPED\n")
                        
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
                    if self.is_running:
                        try:
                            self.is_running = False
                            self.watchdog.is_running = False
                            if self.watchdog_task:
                                await self.watchdog_task
                            await self.ble_server.stop()
                            self.logger_comp.stop()
                            await self.watchdog.emit_off()
                        except Exception as e:
                            print(f"[TCP_Server][ERROR] Error during shutdown: {e}")
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
