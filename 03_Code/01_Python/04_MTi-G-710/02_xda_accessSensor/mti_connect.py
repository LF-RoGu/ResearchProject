import serial
import serial.tools.list_ports

XSENS_VID = '2639'

def find_xsens_port():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if XSENS_VID in port.hwid:
            print(f"✅ Xsens device found on port: {port.device}")
            return port.device
    print("❌ No Xsens device found.")
    return None

def scan_for_xbus_sync(port, baudrate=115200, timeout=2):
    try:
        with serial.Serial(port, baudrate, timeout=timeout) as ser:
            print(f"🔌 Port {port} opened. Scanning for Xbus sync...")
            while True:
                byte = ser.read(1)
                if not byte:
                    print("⚠️ No data received.")
                    break
                if byte[0] == 0xFA:
                    next_byte = ser.read(1)
                    if next_byte and next_byte[0] == 0xFF:
                        print("✅ Xbus sync header detected: 0xFA 0xFF")
                        print("🎉 MTi-G-710 is connected and sending binary data.")
                        return
    except serial.SerialException as e:
        print(f"❌ Serial error: {e}")

if __name__ == "__main__":
    port = find_xsens_port()
    if port:
        scan_for_xbus_sync(port)
