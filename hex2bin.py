"""Convert Intel HEX to raw binary for OTA."""
import sys, os

script_dir = os.path.dirname(os.path.abspath(__file__))
hex_file = os.path.join(script_dir, 'Objects', 'rt-thread.hex')
bin_file = os.path.join(script_dir, 'rtthread.bin')

if not os.path.exists(hex_file):
    print(f'{hex_file} not found. Build first.')
    sys.exit(1)

APP_BASE = 0x0800C000     # must match APP1_BASE in bootloader/partition.h
APP_SIZE = 208 * 1024     # must match FW_SIZE in bootloader/partition.h
data = bytearray(APP_SIZE)
ext_addr = 0
base_addr = None
pairs = []

with open(hex_file) as f:
    for line in f:
        line = line.strip()
        if not line or line[0] != ':': continue
        b = bytes.fromhex(line[1:])
        count, addr16, rectype = b[0], (b[1]<<8)|b[2], b[3]
        if rectype == 0x04:
            ext_addr = ((b[4]<<8)|b[5]) << 16
        elif rectype == 0x00:
            addr = ext_addr | addr16
            if base_addr is None: base_addr = addr
            for i in range(count):
                pairs.append((addr + i, b[4 + i]))
        elif rectype == 0x01:
            break

if base_addr is None:
    print('No data!')
    sys.exit(1)

if base_addr != APP_BASE:
    print(f'WARNING: hex base 0x{base_addr:08X} != APP_BASE 0x{APP_BASE:08X} '
          f'(bootloader/partition.h)')

max_off = 0
for addr, val in pairs:
    off = addr - base_addr
    if 0 <= off < len(data):
        data[off] = val
        if off > max_off: max_off = off

result = data[:max_off+1]
# Fix Keil's 0x0080xxxx addresses → proper 0x0800xxxx in vector table
for i in range(0, min(256, len(result)), 4):
    w = int.from_bytes(result[i:i+4], 'little')
    if (w & 0x00FF0000) == 0x00800000:
        w = (w & 0xFF00FFFF) | 0x08000000  # 0x0080xxxx -> 0x0800xxxx
        result[i:i+4] = w.to_bytes(4, 'little')
open(bin_file, 'wb').write(result)
print(f'hex2bin: {len(result)} bytes, base 0x{base_addr:08X} -> rtthread.bin')

