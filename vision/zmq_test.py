import zmq
import json

def main():
    print("Testovací ZMQ klient pro vision stream...")
    context = zmq.Context()
    sub = context.socket(zmq.SUB)
    sub.connect("ipc:///tmp/robot-vision")
    sub.setsockopt_string(zmq.SUBSCRIBE, "")
    
    print("Připojeno. Čekám na první zprávu...")
    
    try:
        msg = sub.recv_string()
        print(f"PŘIJATO:\n{msg[:500]}...")
        
        topic, json_str = msg.split("/", 1)
        data = json.loads(json_str)
        print("\nRozkódováno úspěšně! Pose délka:", len(data.get("pose", [])))
    except Exception as e:
        print("Chyba:", e)
    finally:
        sub.close()

if __name__ == "__main__":
    main()
