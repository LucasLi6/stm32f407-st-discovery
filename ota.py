"""One-click OTA: auto-detects COM port, sends 'ota', transfers firmware via Ymodem."""
import serial, serial.tools.list_ports, sys, os, time

SOH, STX, EOT, ACK, NAK, CRC = 0x01, 0x02, 0x04, 0x06, 0x15, ord('C')
BIN = 'rtthread.bin'

def crc16(data):
    c = 0
    for b in data:
        c ^= (b << 8)
        for _ in range(8):
            c = (c << 1) ^ 0x1021 if c & 0x8000 else c << 1
            c &= 0xFFFF
    return c

def send_pkt(ser, seq, data):
    d = data.ljust(128, b'\x00')
    c = crc16(d)
    ser.write(bytes([SOH, seq & 0xFF, (~seq) & 0xFF]) + d + bytes([c >> 8, c & 0xFF]))

def wait_byte(ser, expect, timeout=1.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if ser.in_waiting and ser.read(1)[0] in expect:
            return True
    return False

def find_stm32():
    """Scan COM ports for an STM32 (look for '>' prompt)."""
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        return None
    # Try each port
    for p in ports:
        try:
            ser = serial.Serial(p.device, 115200, timeout=0.3)
            time.sleep(0.2); ser.reset_input_buffer()
            ser.write(b'\r\n')
            t0 = time.time()
            while time.time() - t0 < 1.0:
                if ser.in_waiting:
                    buf = ser.read(ser.in_waiting)
                    if b'>' in buf or b'STM32' in buf or b'CC' in buf:
                        ser.close()
                        return p.device
                time.sleep(0.05)
            ser.close()
        except:
            pass
    # If nothing responded, return first available port as fallback
    return ports[0].device if ports else None

def main():
    # Args: [COM_PORT] [BIN_FILE]
    port = None; bin_path = None
    for a in sys.argv[1:]:
        if a.endswith('.bin'):
            bin_path = a
        elif a.startswith('COM') or a.startswith('/dev/'):
            port = a
    if not port:
        port = find_stm32()

    # Find .bin files
    script_dir = os.path.dirname(os.path.abspath(__file__))
    if bin_path:
        if not os.path.exists(bin_path):
            print(f"ERROR: {bin_path} not found."); sys.exit(1)
    else:
        bins = sorted([f for f in os.listdir(script_dir) if f.endswith('.bin')])
        if not bins:
            print(f"ERROR: No .bin files found in {script_dir}"); sys.exit(1)
        if len(bins) == 1:
            bin_path = os.path.join(script_dir, bins[0])
        else:
            print("\nSelect firmware:")
            for i, name in enumerate(bins):
                size = os.path.getsize(os.path.join(script_dir, name))
                print(f"  {i+1}. {name} ({size/1024:.1f} KB)")
            while True:
                try:
                    n = input(f"  Choice (1-{len(bins)}): ").strip()
                    idx = int(n) - 1
                    if 0 <= idx < len(bins):
                        bin_path = os.path.join(script_dir, bins[idx])
                        break
                except:
                    pass
                print("  Invalid, try again.")

    data = open(bin_path, 'rb').read()
    APP_MAX = 208 * 1024  # must match FW_SIZE in partition.h
    if len(data) > APP_MAX:
        print(f"\nERROR: {os.path.basename(bin_path)} is {len(data)/1024:.1f} KB.")
        print(f"       APP partition is only {APP_MAX/1024:.0f} KB.")
        print(f"       Firmware too large — trim code or reduce features.")
        sys.exit(1)
    print(f"\nOTA: {os.path.basename(bin_path)} {len(data)/1024:.1f} KB -> {port}")
    try:
        ser = serial.Serial(port, 115200, timeout=0.3)
    except:
        print(f"ERROR: Cannot open {port} — close your terminal/serial monitor first!")
        sys.exit(1)
    time.sleep(0.3); ser.reset_input_buffer()

    # Send 'ota' if APP is running (look for prompt)
    print("Looking for STM32...")
    ser.write(b'\r\n')
    t0 = time.time(); prompt = False
    while time.time() - t0 < 2:
        if ser.in_waiting:
            buf = ser.read(ser.in_waiting)
            if b'>' in buf or b'STM32' in buf:
                prompt = True; break
        time.sleep(0.1)
    if prompt:
        print("Sending 'ota'...")
        ser.write(b'ota\r\n')
        time.sleep(0.5)

    # Wait for bootloader 'C'
    print("Waiting for bootloader...")
    ser.reset_input_buffer()
    for _ in range(120):
        if wait_byte(ser, [CRC], 1.0):
            ser.reset_input_buffer()
            print("Sending firmware...")
            break
    else:
        print("Timeout."); ser.close(); sys.exit(1)

    # Ymodem send
    offset, seq = 0, 1
    while offset < len(data):
        chunk = data[offset:offset+128]
        for _ in range(5):
            send_pkt(ser, seq, chunk)
            if wait_byte(ser, [ACK], 1.0):
                offset += 128; seq = (seq % 255) + 1
                print(f"\r  {min(100, offset*100//len(data))}%", end='', flush=True)
                break
        else:
            print(f"\nFailed pkt {seq}"); ser.close(); sys.exit(1)

    # End transfer
    print("\nFinishing...")
    ser.write(bytes([EOT])); wait_byte(ser, [NAK], 2.0)
    ser.write(bytes([EOT])); wait_byte(ser, [ACK], 2.0)
    ser.write(bytes([CRC])); wait_byte(ser, [SOH], 2.0)
    send_pkt(ser, 0, bytes(128))
    ser.close()
    print("Done!")

if __name__ == '__main__':
    main()
