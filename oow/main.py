import asyncio
import sys

import signal

from watchdog import OfficerWatchdog
from oow_service import OowBleServer
from oow_logger import OowLogger
from client import OowTcpServer

async def main():
    loop = asyncio.get_running_loop()
    
    # Inicializace logovací komponenty
    logger_comp = OowLogger()
    
    # Inicializace watchdogu
    watchdog = OfficerWatchdog(logger=logger_comp, zmq_address="ipc:///tmp/robot-oow")
    
    # Inicializace BLE serveru
    ble_server = OowBleServer(loop=loop, watchdog=watchdog)
    
    # Inicializace TCP serveru
    shutdown_event = asyncio.Event()
    tcp_server = OowTcpServer(
        host="127.0.0.1", 
        port=9013, 
        ble_server=ble_server, 
        watchdog=watchdog, 
        logger_comp=logger_comp,
        shutdown_event=shutdown_event
    )
    
    def signal_handler(sig_name):
        print(f"[Main][INFO] System signal {sig_name} received, shutting down...")
        shutdown_event.set()
        
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, signal_handler, sig.name)
    
    # Odeslat OFF hned po startu programu
    await watchdog.emit_off()
    
    try:
        await tcp_server.start()
        print("[Main][INFO] OOW TCP Server is running. Press Ctrl+C to stop.")
        
        # Čekáme na SHUTDOWN příkaz přes TCP (nebo Ctrl+C)
        await shutdown_event.wait()
        
    except asyncio.CancelledError:
        print("[Main][INFO] Shutdown signal received.")
    except Exception as e:
        print(f"[Main][ERROR] Error in main loop: {e}")
    finally:
        print("[Main][INFO] Shutting down...")
        for sig in (signal.SIGINT, signal.SIGTERM):
            loop.remove_signal_handler(sig)
            
        await tcp_server.stop()
        
        # Ujistíme se, že všechny služby jsou zastaveny bez ohledu na příznak is_running
        tcp_server.is_running = False
        watchdog.is_running = False
        
        if getattr(tcp_server, 'watchdog_task', None):
            try:
                tcp_server.watchdog_task.cancel()
                await tcp_server.watchdog_task
            except asyncio.CancelledError:
                pass
            except Exception as e:
                print(f"[Main][ERROR] Error during watchdog cleanup: {e}")
                
        try:
            await ble_server.stop()
        except Exception as e:
            print(f"[Main][ERROR] Error stopping BLE server: {e}")
            
        try:
            logger_comp.stop()
        except Exception:
            pass

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("[Main][INFO] Program terminated by user.")
