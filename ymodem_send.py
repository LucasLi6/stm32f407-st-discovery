"""
Send a file via Ymodem protocol over a serial port.
Usage: python ymodem_send.py <COM_PORT> <FILE>
  e.g. python ymodem_send.py COM3 rtthread.bin
"""
import serial, struct, sys, os, time

SOH, STX, EOT, ACK, NAK, CAN, CRC16 = 0x01, 0x02, 0x04, 0x06, 0x15, 0x18, ord('C')
PACKET_SIZE = 128
TIMEOUT = 1.0

def crc16(data):
    c = 0
    for b in data:
        c ^= (b << 8)
        for _ in range(8):
            c = (c << 1) ^ 0x1021 if c & 0x8000 else c << 1
            c &= 0xFFFF
    return c

def send_packet(ser, seq, data):
    """Send one Ymodem 128-byte packet."""
    pkt = bytearray()
    pkt.append(SOH)
    pkt.append(seq & 0xFF)
    pkt.append((~seq) & 0xFF)
    pkt.extend(data.ljust(128, b'\x00'))
    c = crc16(data.ljust(128, b'\x00'))
    pkt.append((c >> 8) & 0xFF)
    pkt.append(c & 0xFF)
    ser.write(pkt)

def wait_for(ser, expected, timeout=TIMEOUT):
    """Wait for a specific byte from serial."""
    start = time.time()
    while time.time() - start < timeout:
        if ser.in_waiting:
            c = ser.read(1)[0]
            if c in expected:
                return c
    return None

def main():
    if len(sys.argv) < 3:
        print(f"Usage: python {sys.argv[0]} <COM_PORT> <FILE>")
        print(f"  e.g. python {sys.argv[0]} COM3 rtthread.bin")
        sys.exit(1)

    port = sys.argv[1]
    path = sys.argv[2]

    if not os.path.exists(path):
        print(f"File not found: {path}")
        sys.exit(1)

    data = open(path, 'rb').read()
    total_packets = (len(data) + 127) // 128
    print(f"File: {path} ({len(data)} bytes, {total_packets} packets)")
    print(f"Opening {port} at 115200...")

    ser = serial.Serial(port, 115200, timeout=0.5)

    # Wait for 'C' from bootloader
    print("Waiting for bootloader to be ready ('C')...")
    retries = 0
    while retries < 120:
        c = wait_for(ser, [CRC16])
        if c == CRC16:
            break
        retries += 1
    if retries >= 120:
        print("Timeout waiting for bootloader. Did you type 'ota' in the terminal?")
        sys.exit(1)

    print("Bootloader ready. Sending...")

    offset = 0
    seq = 1
    while offset < len(data):
        chunk = data[offset:offset+128]
        send_packet(ser, seq, chunk)
        c = wait_for(ser, [ACK, NAK])
        if c == ACK:
            offset += 128
            seq = (seq % 255) + 1
            pct = min(100, offset * 100 // len(data))
            print(f"\rProgress: {pct}%", end='', flush=True)
        elif c == NAK:
            print(f"\nNAK received for packet {seq}, retrying...")
        else:
            print(f"\nNo response for packet {seq}, retrying...")
            send_packet(ser, seq, chunk)

    # End of transfer
    print("\nSending EOT...")
    for _ in range(3):
        ser.write(bytes([EOT]))
        if wait_for(ser, [NAK]):
            break
    for _ in range(3):
        ser.write(bytes([EOT]))
        if wait_for(ser, [ACK]):
            break

    # Send final empty packet (seq=0)
    ser.write(bytes([CRC16]))
    wait_for(ser, [SOH])
    send_packet(ser, 0, bytes(128))

    ser.close()
    print("Done!")

if __name__ == '__main__':
    main()
