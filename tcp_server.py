"""One-click WiFi OTA: finds ESP8266, sends 'ota wifi', serves firmware in paced chunks."""
import socket, sys, os, time, threading
from concurrent.futures import ThreadPoolExecutor, as_completed

PORT = 8888

def find_esp():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try: s.connect(('8.8.8.8', 80)); my_ip = s.getsockname()[0]
    except: my_ip = '192.168.1.100'
    s.close()
    parts = my_ip.split('.')
    subnet = f"{parts[0]}.{parts[1]}.{parts[2]}"
    print(f"Scanning local network for ESP8266...")

    def probe(i):
        try:
            t = socket.socket(); t.settimeout(0.05)
            t.connect((f"{subnet}.{i}", 23)); t.close()
            return f"{subnet}.{i}"
        except: return None

    with ThreadPoolExecutor(max_workers=100) as pool:
        futures = {pool.submit(probe, i): i for i in range(1, 255)}
        for f in as_completed(futures):
            r = f.result()
            if r: print(f"Found ESP8266 at {r}"); return r
    return None

def pick_bin():
    """List .bin files, let user pick by number."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    bins = sorted([f for f in os.listdir(script_dir) if f.endswith('.bin')])
    if not bins: return None
    if len(bins) == 1:
        path = os.path.join(script_dir, bins[0])
        print(f"Auto-selected: {bins[0]} ({os.path.getsize(path)/1024:.1f} KB)")
        return path
    print("\nSelect firmware:")
    for i, name in enumerate(bins):
        size = os.path.getsize(os.path.join(script_dir, name))
        print(f"  {i+1}. {name} ({size/1024:.1f} KB)")
    while True:
        try:
            n = int(input(f"  Choice (1-{len(bins)}): ").strip())
            if 1 <= n <= len(bins):
                return os.path.join(script_dir, bins[n-1])
        except: pass

def main():
    bin_path = pick_bin()
    if not bin_path: print("No .bin file found!"); sys.exit(1)
    data = open(bin_path, 'rb').read()
    print(f"Firmware: {os.path.basename(bin_path)} ({len(data)/1024:.1f} KB)")

    # Load last-used IP from file
    my_dir = os.path.dirname(os.path.abspath(__file__))
    ip_file = os.path.join(my_dir, 'ota_ip.txt')
    saved_ip = ''
    if os.path.exists(ip_file):
        try: saved_ip = open(ip_file).read().strip()
        except: pass

    prompt = f"ESP8266 IP" + (f" [{saved_ip}]" if saved_ip else "") + ": "
    esp_ip = input(prompt).strip()
    if not esp_ip: esp_ip = saved_ip
    if not esp_ip: print("No IP."); sys.exit(1)

    # Save IP for next time
    try: open(ip_file, 'w').write(esp_ip)
    except: pass

    # Start firmware server
    s = socket.socket(); s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('0.0.0.0', PORT)); s.listen(1)

    def serve():
        c, a = s.accept()
        c.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        print(f"Bootloader connected from {a[0]}. Sending {len(data)} bytes via Ymodem...")
        c.settimeout(10)

        def crc16(d):
            c = 0
            for b in d:
                c ^= (b << 8)
                for _ in range(8):
                    c = ((c << 1) ^ 0x1021) if (c & 0x8000) else (c << 1)
                    c &= 0xFFFF
            return c

        # Ymodem: wait for 'C', send 128-byte packets
        chunk = 128
        seq = 1
        off = 0

        # Wait for 'C' from bootloader
        while c.recv(1) != b'C':
            pass

        while off < len(data):
            payload = data[off:off+chunk]
            if len(payload) < chunk:
                payload = payload + b'\x1A' * (chunk - len(payload))  # pad with Ctrl-Z
            pkt = bytes([0x01, seq, 0xFF - seq]) + payload
            pkt += crc16(payload).to_bytes(2, 'big')
            while True:
                try:
                    c.sendall(pkt)
                    ack = c.recv(1)
                    if ack == b'\x06':  # ACK
                        off += chunk
                        pct = min(100, off * 100 // len(data))
                        sys.stdout.write(f'\r  {pct}%  {off/1024:.0f}/{len(data)/1024:.0f}KB\033[K')
                        sys.stdout.flush()
                        seq = seq % 255 + 1   # 1..255, never 0 (reserved for final packet)
                        break
                except socket.timeout:
                    pass  # retry
        # EOT
        while True:
            try:
                c.sendall(b'\x04')
                ack = c.recv(1)
                if ack == b'\x15':  # NAK
                    c.sendall(b'\x04')
                    ack = c.recv(1)
                    if ack == b'\x06':  # ACK
                        # send empty final packet
                        c.sendall(b'\x01\x00\xFF' + b'\x00'*128 + crc16(b'\x00'*128).to_bytes(2,'big'))
                        c.recv(1)  # consume final ACK
                        print(' EOT OK')
                        break
            except:
                pass
        time.sleep(1)
        try: c.close()
        except: pass
        print('\nDone!'); s.close()
        input('Press Enter...'); os._exit(0)

    threading.Thread(target=serve, daemon=True).start()
    time.sleep(0.3)

    # Telnet to ESP and send command
    print(f"Sending 'ota wifi' to {esp_ip}...")
    telnet = None
    try:
        telnet = socket.socket(); telnet.settimeout(5)
        telnet.connect((esp_ip, 23)); time.sleep(0.3)
        telnet.send(b'ota wifi\r\n')
        print("Command sent. Waiting for bootloader to connect back...")
        # Keep telnet alive — bootloader needs it for CIPSTATUS
    except Exception as e:
        print(f"Telnet failed: {e}")

    # Wait for bootloader (serve thread handles the transfer)
    try:
        while True: time.sleep(1)
    except KeyboardInterrupt: pass
    finally:
        if telnet:
            try: telnet.close()
            except: pass

if __name__ == '__main__':
    main()
