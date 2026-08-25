import time
import threading
import signal
import sys
from service import HMIService
from poller import ZMQPoller

DEVICE_ID = "120453749J000566"
SOUND_PATH = "/opt/projects/robotour/hmi-bridge/sounds"
TCP_PORT = 9020

ZMQ_PORTS = [8001, 8002]

def main():
    print(f"[MAIN] Starting HMI Bridge Service on port {TCP_PORT} for device {DEVICE_ID}")
    
    # Initialize ZMQ pollers
    pollers = []
    for port in ZMQ_PORTS:
        poller = ZMQPoller(port)
        pollers.append(poller)
        threading.Thread(target=poller.run, daemon=True).start()
        print(f"[MAIN] ZMQ Poller started for port {port}")
        
    # Initialize and run TCP service
    service = HMIService(DEVICE_ID, TCP_PORT, SOUND_PATH)

    def signal_handler(sig, frame):
        print("\n🧯 Detekován signál k ukončení, zahajuji čisté vypnutí...")
        service.stop()
        for poller in pollers:
            poller.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        service.run()
    except KeyboardInterrupt:
        pass
    finally:
        service.stop()
        for poller in pollers:
            poller.stop()

if __name__ == "__main__":
    main()
