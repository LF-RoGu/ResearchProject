import socket
import time

TCP_IP = "192.168.38.97"  # Replace with your Raspberry Pi's IP if running remotely
TCP_PORT = 8888
RETRY_INTERVAL = 1.0  # Seconds

def wait_for_connection(ip, port):
    while True:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((ip, port))
            print(f"[PYTHON] Connected to {ip}:{port}")
            return sock
        except socket.error:
            print(f"[PYTHON] Server not ready, retrying in {RETRY_INTERVAL} sec...")
            time.sleep(RETRY_INTERVAL)

def read_data(sock):
    try:
        buffer = ""
        while True:
            data = sock.recv(4096)
            if not data:
                print("[PYTHON] Connection closed by server.")
                break

            buffer += data.decode('utf-8')
            while '\n' in buffer:
                line, buffer = buffer.split('\n', 1)
                print("[RECEIVED]", line.strip())

    except Exception as e:
        print(f"[PYTHON ERROR] {e}")
    finally:
        sock.close()

if __name__ == "__main__":
    sock = wait_for_connection(TCP_IP, TCP_PORT)
    read_data(sock)
