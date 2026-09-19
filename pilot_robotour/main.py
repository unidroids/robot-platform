import asyncio
import json
import sys

try:
    from .service import RobotourPilotService
except (ImportError, ValueError):
    from service import RobotourPilotService

SERVICE_PORT = 9104

OOW_PORT = 9030

async def oow_poller(service):
    print(f"[OOW_Poller] Spouštím TCP poller OOW na portu {OOW_PORT} (každou 1s)")
    while service.running:
        try:
            reader, writer = await asyncio.open_connection("127.0.0.1", OOW_PORT)
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
        except Exception:
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
                if not line:
                    continue
                
                parts = line.split()
                cmd = parts[0].upper()
                print(f"[TCP_Server] Zpracovávám TCP příkaz: {line}")
                
                if cmd == "PING":
                    writer.write(b"PONG PILOT_ROBOTOUR\n")
                elif cmd == "START":
                    speed = 100
                    pwm = 150
                    route_input = None
                    rest = line[len(parts[0]):].strip()
                    
                    # Zkontrolujeme, zda zbytek řádku obsahuje JSON (např. trasa z MAPS)
                    json_start = -1
                    for idx, ch in enumerate(rest):
                        if ch in ('{', '['):
                            json_start = idx
                            break
                            
                    if json_start != -1:
                        prefix_args = rest[:json_start].strip().split()
                        if len(prefix_args) >= 1 and prefix_args[0].isdigit():
                            speed = int(prefix_args[0])
                        if len(prefix_args) >= 2 and prefix_args[1].isdigit():
                            pwm = int(prefix_args[1])
                        json_str = rest[json_start:].strip()
                        try:
                            route_input = json.loads(json_str)
                        except Exception as e:
                            writer.write(f"ERR: Neplatny JSON format trasy ({e})\n".encode("utf-8"))
                            await writer.drain()
                            continue
                    else:
                        writer.write(b"ERR: START vyzaduje JSON payload s trasou\n")
                        await writer.drain()
                        continue

                    ok, msg = service.start_service(max_speed=speed, max_pwm=pwm, route_input=route_input)
                    if ok:
                        if service.oow_task is None or service.oow_task.done():
                            service.oow_task = asyncio.create_task(oow_poller(service))
                        writer.write(b"OK\n")
                    else:
                        writer.write(f"{msg}\n".encode("utf-8"))
                elif cmd == "STOP":
                    service.stop_service()
                    writer.write(b"OK\n")
                elif cmd == "PAUSE":
                    service.pause_service(source="USER", info="Pozastaveno uživatelem (PAUSE)")
                    writer.write(b"OK\n")
                elif cmd == "RESUME":
                    service.resume_service(source="USER", info="Obnoveno uživatelem (RESUME)")
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
        except Exception:
            pass

async def main():
    service = RobotourPilotService()
    server = await asyncio.start_server(
        lambda r, w: handle_client(r, w, service),
        '0.0.0.0', SERVICE_PORT
    )
    
    print(f"[TCP_Server] Služba PILOT-ROBOTOUR naslouchá na TCP portu {SERVICE_PORT}.")
    async with server:
        await server.serve_forever()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("[Main] Přerušeno uživatelem.")
