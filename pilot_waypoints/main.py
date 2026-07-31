import asyncio
import sys
from service import WaypointsPilotService

async def oow_poller(service):
    print("[OOW_Poller] Spouštím TCP poller OOW na portu 9013 (každou 1s)")
    while service.running:
        try:
            reader, writer = await asyncio.open_connection("127.0.0.1", 9013)
            writer.write(b"OOW\n")
            await writer.drain()
            data = await asyncio.wait_for(reader.readline(), timeout=0.5)
            status = data.decode("utf-8").strip()
            if status == "ON":
                service.set_oow_tcp_ok(True)
            else:
                service.set_oow_tcp_ok(False)
            writer.close()
            await writer.wait_closed()
        except Exception as e:
            service.set_oow_tcp_ok(False)
            
        await asyncio.sleep(1.0)

async def handle_client(reader, writer, service):
    addr = writer.get_extra_info('peername')
    print(f"[TCP_Server] Klient připojen: {addr}")
    
    try:
        while True:
            try:
                data = await asyncio.wait_for(reader.readline(), timeout=1.0)
                if not data:
                    break
                line = data.decode("utf-8").strip()
                if not line: continue
                
                parts = line.split()
                cmd = parts[0].upper()
                print(f"[TCP_Server] Zpracovávám TCP příkaz: {line}")
                
                if cmd == "PING":
                    writer.write(b"PONG PILOT_WAYPOINTS\n")
                elif cmd == "START":
                    speed = 100
                    pwm = 150
                    if len(parts) >= 2: speed = int(parts[1])
                    if len(parts) >= 3: pwm = int(parts[2])
                    service.start_service(max_speed=speed, max_pwm=pwm)
                    
                    if service.oow_task is None or service.oow_task.done():
                        service.oow_task = asyncio.create_task(oow_poller(service))
                    writer.write(b"OK\n")
                elif cmd == "STOP":
                    service.stop_service()
                    writer.write(b"OK\n")
                elif cmd == "PAUSE":
                    service.pause_service(source="USER", info="TCP Command")
                    writer.write(b"OK\n")
                elif cmd == "RESUME":
                    service.resume_service(source="USER", info="TCP Command")
                    writer.write(b"OK\n")
                elif cmd == "STATUS":
                    st = service.get_status()
                    writer.write(f"{st}\n".encode())
                elif cmd == "EXIT":
                    writer.write(b"BYE\n")
                    break
                elif cmd == "SHUTDOWN":
                    writer.write(b"SHUTTING DOWN\n")
                    await writer.drain()
                    service.shutdown()
                    sys.exit(0)
                else:
                    writer.write(b"ERR Unknown\n")
                await writer.drain()
            except asyncio.TimeoutError:
                pass
    except Exception as e:
        print(f"[TCP_Server] Klient chyba: {e}")
    finally:
        print(f"[TCP_Server] Klient odpojen: {addr}")
        try:
            writer.close()
        except:
            pass

async def main():
    service = WaypointsPilotService()
    server = await asyncio.start_server(
        lambda r, w: handle_client(r, w, service),
        '0.0.0.0', 9101
    )
    
    print("[TCP_Server] Služba pilot_waypoints naslouchá na TCP portu 9101.")
    async with server:
        await server.serve_forever()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("[Main] Přerušeno uživatelem.")
