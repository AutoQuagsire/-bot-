#!/usr/bin/env python3
"""
Minimal DebugLink PING/ACK test.

Usage:
    python ping_test.py COM3
    python ping_test.py /dev/ttyUSB0

Requires: pyserial (pip install pyserial)
"""

import sys
import struct
import time
import serial

SOF0 = 0x5A
SOF1 = 0xA5
VERSION = 0x01

MSG_PING_REQ = 0x01
MSG_ACK = 0x80


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_frame(msg_type: int, seq: int, payload: bytes) -> bytes:
    header = bytes([VERSION, msg_type, seq]) + struct.pack('<H', len(payload))
    body = header + payload
    crc = crc16_ccitt(body)
    return bytes([SOF0, SOF1]) + body + struct.pack('<H', crc)


def parse_frame(data: bytes):
    if len(data) < 9:
        return None
    if data[0] != SOF0 or data[1] != SOF1:
        return None

    msg_type = data[3]
    seq = data[4]
    payload_len = struct.unpack_from('<H', data, 5)[0]

    expected_len = 2 + 5 + payload_len + 2
    if len(data) < expected_len:
        return None

    body = data[2 : 2 + 5 + payload_len]
    crc_calc = crc16_ccitt(body)
    crc_recv = struct.unpack_from('<H', data, 2 + 5 + payload_len)[0]

    if crc_calc != crc_recv:
        print(f"CRC mismatch: calc=0x{crc_calc:04X} recv=0x{crc_recv:04X}")
        return None

    payload = data[7 : 7 + payload_len]
    return (msg_type, seq, payload)


def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <serial_port>")
        sys.exit(1)

    port = sys.argv[1]
    baud = 921600

    print(f"[LINK] open {port} {baud}")
    ser = serial.Serial(port, baud, timeout=1.0)

    host_time_ms = int(time.time() * 1000) & 0xFFFFFFFF
    payload = struct.pack('<I', host_time_ms)
    seq = 1

    ping_frame = build_frame(MSG_PING_REQ, seq, payload)

    print(f"[TX] PING_REQ seq={seq}")
    ser.write(ping_frame)
    ser.flush()

    print("Waiting for ACK...")
    response = ser.read(256)

    if not response:
        print("ERROR: No response (timeout)")
        ser.close()
        sys.exit(1)

    result = parse_frame(response)
    if result is None:
        print(f"ERROR: Failed to parse response: {response.hex(' ')}")
        ser.close()
        sys.exit(1)

    msg_type, resp_seq, resp_payload = result

    if msg_type == MSG_ACK:
        if len(resp_payload) >= 2:
            req_msg = resp_payload[0]
            status = resp_payload[1]
            print(f"[RX] ACK req=0x{req_msg:02X} status={status}")
            if req_msg == MSG_PING_REQ and status == 0:
                print("SUCCESS: PING/ACK round-trip complete!")
            else:
                print("WARNING: Unexpected ACK content")
        else:
            print(f"[RX] ACK payload: {resp_payload.hex(' ')}")
    else:
        print(f"[RX] Unexpected msg_type=0x{msg_type:02X}")

    ser.close()


if __name__ == '__main__':
    main()
