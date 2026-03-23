#!/usr/bin/env python3
"""
ViewPro camera serial diagnostic tool.
Sends commands and dumps raw response bytes from the camera.

Usage: python3 viewpro_diag.py /dev/ttyUSB0
"""
import serial
import sys
import time

PORT = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
BAUD = 115200

# ViewPro commands
HANDSHAKE_CMD = bytes([0x55, 0xAA, 0xDC, 0x04, 0x00, 0x00, 0x04])
HEARTBEAT_CMD = bytes([0x55, 0xAA, 0xDC, 0x04, 0x10, 0x00, 0x14])
# S2 TGCC 0x0A: Open periodic T1F1B1D1 to serial port, period=200ms (5Hz)
PERIODIC_T1F1B1D1_CMD = bytes([0x55, 0xAA, 0xDC, 0x08, 0x26, 0x0A, 0x00, 0x00, 0x00, 0xC8, 0xEC])

OUTPUT_TO_DEG = 360.0 / 65536.0

def calc_crc(data):
    res = 0
    for b in data:
        res ^= b
    return res & 0xFF

def parse_t1f1b1d1(payload):
    """Parse angles from T1F1B1D1 data payload (after frame_id)"""
    if len(payload) < 29:
        print(f"  T1F1B1D1 payload too short: {len(payload)} bytes")
        return
    # Roll: 12-bit from bytes 23-24 (offset from data start, which is payload[0])
    # In the packet, data starts after frame_id
    # The published offsets (23,24,25...) are relative to data_start in the buffer
    # In our payload array, these map directly since payload[0] = first data byte
    roll_raw = ((payload[23] & 0x0F) << 8) | payload[24]
    roll_deg = roll_raw * (180.0 / 4095.0) - 90.0

    yaw_raw = (payload[25] << 8) | payload[26]
    if yaw_raw > 32767:
        yaw_raw -= 65536
    yaw_deg = yaw_raw * OUTPUT_TO_DEG

    pitch_raw = (payload[27] << 8) | payload[28]
    if pitch_raw > 32767:
        pitch_raw -= 65536
    pitch_deg = -(pitch_raw * OUTPUT_TO_DEG)

    print(f"  ANGLES: yaw={yaw_deg:.2f}° pitch={pitch_deg:.2f}° roll={roll_deg:.2f}°")

def main():
    print(f"Opening {PORT} at {BAUD} baud...")
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.1)
    except Exception as e:
        print(f"Failed to open serial: {e}")
        sys.exit(1)

    # Flush any garbage
    ser.reset_input_buffer()
    time.sleep(0.1)

    # Send handshake first
    print(f"\n--- Sending HANDSHAKE: {HANDSHAKE_CMD.hex(' ')} ---")
    ser.write(HANDSHAKE_CMD)
    time.sleep(0.3)

    # Send heartbeat
    print(f"\n--- Sending HEARTBEAT: {HEARTBEAT_CMD.hex(' ')} ---")
    ser.write(HEARTBEAT_CMD)
    time.sleep(0.5)

    # Read and dump whatever came back
    raw = ser.read(1024)
    print(f"Received {len(raw)} bytes after heartbeat:")
    if raw:
        print(f"  HEX: {raw.hex(' ')}")

    # Send periodic T1F1B1D1 request
    print(f"\n--- Sending PERIODIC T1F1B1D1 request: {PERIODIC_T1F1B1D1_CMD.hex(' ')} ---")
    ser.write(PERIODIC_T1F1B1D1_CMD)

    print("\n--- Reading responses (Ctrl+C to stop) ---")
    buf = bytearray()
    pkt_count = 0

    try:
        while True:
            chunk = ser.read(256)
            if not chunk:
                continue
            buf.extend(chunk)

            # Try to parse packets from buffer
            while len(buf) >= 5:
                # Look for header
                idx = -1
                for j in range(len(buf) - 2):
                    if buf[j] == 0x55 and buf[j+1] == 0xAA and buf[j+2] == 0xDC:
                        idx = j
                        break

                if idx < 0:
                    # No header found, keep last 2 bytes (might be partial header)
                    if len(buf) > 2:
                        print(f"  DISCARD {len(buf)-2} bytes (no header): {buf[:min(20,len(buf)-2)].hex(' ')}")
                        buf = buf[-2:]
                    break

                # Discard bytes before header
                if idx > 0:
                    print(f"  SKIP {idx} bytes before header: {buf[:idx].hex(' ')}")
                    buf = buf[idx:]

                # Need at least header(3) + length(1)
                if len(buf) < 4:
                    break

                data_len = buf[3] & 0x3F
                total_pkt_len = 3 + data_len  # 3 header bytes + everything from length to CRC

                if len(buf) < total_pkt_len:
                    # Need more bytes
                    break

                # Extract full packet
                pkt = buf[:total_pkt_len]
                buf = buf[total_pkt_len:]

                # Verify CRC (XOR of bytes from length to second-to-last)
                crc_data = pkt[3:-1]  # from length byte to byte before CRC
                expected_crc = calc_crc(crc_data)
                actual_crc = pkt[-1]

                frame_id = pkt[4]
                pkt_count += 1

                status = "OK" if expected_crc == actual_crc else f"FAIL(exp=0x{expected_crc:02X})"
                print(f"\n[PKT #{pkt_count}] FrameID=0x{frame_id:02X} Len={data_len} CRC={status}")
                print(f"  RAW: {pkt.hex(' ')}")

                if expected_crc == actual_crc and frame_id == 0x40:
                    # T1F1B1D1 - extract angles
                    payload = pkt[5:-1]  # data after frame_id, before CRC
                    parse_t1f1b1d1(payload)

    except KeyboardInterrupt:
        print(f"\n\nTotal packets received: {pkt_count}")

    ser.close()

if __name__ == '__main__':
    main()
