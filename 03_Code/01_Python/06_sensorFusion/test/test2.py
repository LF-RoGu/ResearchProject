import socket
import time
import select

TCP_IP = "192.168.38.97"  # Use your Raspberry Pi's IP
TCP_PORT = 8888
RETRY_INTERVAL = 1.0  # Seconds
POLL_INTERVAL = 0.01  # Non-blocking poll delay (10ms)

def wait_for_connection(ip, port):
    while True:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((ip, port))
            sock.setblocking(False)  # 🔁 Make the socket non-blocking
            print(f"[PYTHON] Connected to {ip}:{port}")
            return sock
        except socket.error:
            print(f"[PYTHON] Server not ready, retrying in {RETRY_INTERVAL} sec...")
            time.sleep(RETRY_INTERVAL)

def read_data(sock):
    buffer = ""
    try:
        while True:
            # Use select to check for readability
            readable, _, _ = select.select([sock], [], [], 0)
            if readable:
                try:
                    data = sock.recv(4096)
                    if not data:
                        print("[PYTHON] Connection closed by server.")
                        break

                    buffer += data.decode('utf-8')
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        print("[RECEIVED]", line.strip())

                except BlockingIOError:
                    pass  # No data ready
                except Exception as e:
                    print(f"[PYTHON ERROR] recv: {e}")
                    break
            else:
                time.sleep(POLL_INTERVAL)  # Prevents 100% CPU usage

    except KeyboardInterrupt:
        print("[PYTHON] Interrupted by user.")
    finally:
        sock.close()
        print("[PYTHON] Socket closed.")

if __name__ == "__main__":
    sock = wait_for_connection(TCP_IP, TCP_PORT)
    read_data(sock)
