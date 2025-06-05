import serial
import serial.tools.list_ports

XSENS_VID = '2639'
XBUS_SYNC_1 = 0xFA
XBUS_SYNC_2 = 0xFF
MTDATA2_MSG_ID = 0x36  # Most common Xbus data message

def find_xsens_port():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if XSENS_VID in port.hwid:
            print(f"✅ Xsens device found on port: {port.device}")
            return port.device
    print("❌ No Xsens device found.")
    return None

def calc_checksum(msg_bytes):
    # Checksum is (256 - (sum from msg ID to end of payload)) % 256
    return (256 - sum(msg_bytes)) % 256

def read_xbus_packet(ser):
    while True:
        byte1 = ser.read(1)
        if not byte1 or byte1[0] != XBUS_SYNC_1:
            continue
        byte2 = ser.read(1)
        if not byte2 or byte2[0] != XBUS_SYNC_2:
            continue

        msg_id = ser.read(1)[0]
        length = ser.read(1)[0]
        payload = ser.read(length)
        checksum = ser.read(1)[0]

        all_msg = [msg_id, length] + list(payload)
        if calc_checksum(all_msg) != checksum:
            print("❌ Checksum mismatch — skipping packet.")
            continue

        return msg_id, payload

def scan_and_read(port, baudrate=115200):
    try:
        with serial.Serial(port, baudrate, timeout=1) as ser:
            print(f"🔌 Port {port} opened. Waiting for data...")
            while True:
                msg_id, payload = read_xbus_packet(ser)
                if msg_id == MTDATA2_MSG_ID:
                    print(f"\n✅ MTData2 Packet Received ({len(payload)} bytes)")
                    print("Payload (hex):", payload.hex())
                else:
                    print(f"📦 Received unknown message ID: 0x{msg_id:02X}")
    except serial.SerialException as e:
        print(f"❌ Serial error: {e}")

if __name__ == "__main__":
    port = find_xsens_port()
    if port:
        scan_and_read(port)
