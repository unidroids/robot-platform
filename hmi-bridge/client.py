import socket

class HMICLient:
    def __init__(self, host='127.0.0.1', port=9020):
        self.host = host
        self.port = port
        
    def _send_command(self, cmd):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(5.0)
                s.connect((self.host, self.port))
                s.sendall(cmd.encode('utf-8'))
                
                # Receive response
                response = s.recv(4096).decode('utf-8').strip()
                return response
        except Exception as e:
            return f"ERROR Connection failed: {e}"
            
    def ping(self):
        return self._send_command("PING")
        
    def sync(self):
        return self._send_command("SYNC")
        
    def status(self):
        return self._send_command("STATUS")
        
    def shutdown(self):
        return self._send_command("SHUTDOWN")

    def exit(self):
        return self._send_command("EXIT")
