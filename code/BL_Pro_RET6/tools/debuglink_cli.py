#!/usr/bin/env python3
"""
DebugLink CLI tool.

Usage:
    python debuglink_cli.py --port COM33 ping
    python debuglink_cli.py --port COM33 info
    python debuglink_cli.py --port COM33 driver on
    python debuglink_cli.py --port COM33 balance on
    python debuglink_cli.py --port COM33 stream --rate 100

Requires: pyserial (pip install pyserial)
"""

import argparse
import signal
import struct
import sys
import time

import serial

from debuglink_parser import parse_fastcap_chunk, parse_fastcap_status, parse_status_stream

SOF0 = 0x5A
SOF1 = 0xA5
VERSION = 0x01

MSG_PING_REQ = 0x01
MSG_GET_DEVICE_INFO_REQ = 0x02
MSG_STREAM_CONTROL_REQ = 0x10
MSG_POWER_STAGE_REQ = 0x15
MSG_ATTITUDE_CONTROL_REQ = 0x16
MSG_ACK = 0x80
MSG_NACK = 0x81
MSG_DEVICE_INFO_RSP = 0x82
MSG_STATUS_STREAM = 0x90
MSG_FAST_CAPTURE_REQ = 0x14
MSG_FAST_CAPTURE_DATA = 0x91

OP_FASTCAP_ARM = 0x01
OP_FASTCAP_STATUS = 0x02
OP_FASTCAP_READ_CHUNK = 0x03
OP_FASTCAP_STOP = 0x04

FASTCAP_SOURCE_LEFT = 0
FASTCAP_SOURCE_RIGHT = 1
FASTCAP_CHUNK_MAX = 22  # (240 - 11) // 10

FOC_STATUS_FLAG_SPEED_FAULT_L = 1 << 0
FOC_STATUS_FLAG_SPEED_FAULT_R = 1 << 1
FOC_STATUS_FLAG_STACK_READY = 1 << 8
FOC_STATUS_FLAG_CONTROL_IT_ENABLED = 1 << 9
FOC_STATUS_FLAG_BUS_VALID = 1 << 10
FOC_STATUS_FLAG_CURRENT_LOOP_ACTIVE = 1 << 11
FOC_STATUS_FLAG_SPEED_LOOP_ENABLED = 1 << 12
FOC_STATUS_FLAG_CURRENT_LOOP_ENABLED = 1 << 13
FOC_STATUS_FLAG_POWER_STAGE_OFF = 1 << 14
FOC_STATUS_FLAG_ATTITUDE_CONTROL_ON = 1 << 15

DEFAULT_OPEN_RETRIES = 3
DEFAULT_CMD_RETRIES = 3
DEFAULT_SETTLE_MS = 80
DEFAULT_RETRY_DELAY_MS = 80


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
    header = bytes([VERSION, msg_type, seq]) + struct.pack("<H", len(payload))
    body = header + payload
    crc = crc16_ccitt(body)
    return bytes([SOF0, SOF1]) + body + struct.pack("<H", crc)


def sleep_ms(delay_ms: int):
    time.sleep(max(delay_ms, 0) / 1000.0)


def format_foc_status(flags: int) -> str:
    labels = []
    if flags & FOC_STATUS_FLAG_SPEED_FAULT_L:
        labels.append("spdfltL")
    if flags & FOC_STATUS_FLAG_SPEED_FAULT_R:
        labels.append("spdfltR")
    if flags & FOC_STATUS_FLAG_STACK_READY:
        labels.append("stack")
    if flags & FOC_STATUS_FLAG_CONTROL_IT_ENABLED:
        labels.append("it")
    if flags & FOC_STATUS_FLAG_BUS_VALID:
        labels.append("bus")
    if flags & FOC_STATUS_FLAG_CURRENT_LOOP_ACTIVE:
        labels.append("iloop")
    if flags & FOC_STATUS_FLAG_SPEED_LOOP_ENABLED:
        labels.append("sloop")
    if flags & FOC_STATUS_FLAG_CURRENT_LOOP_ENABLED:
        labels.append("cloop")
    if flags & FOC_STATUS_FLAG_POWER_STAGE_OFF:
        labels.append("drv_off")
    if flags & FOC_STATUS_FLAG_ATTITUDE_CONTROL_ON:
        labels.append("bal")
    return ",".join(labels) if labels else "-"


class FrameReader:
    def __init__(self):
        self.reset()

    def reset(self):
        self.state = 0
        self.header = bytearray()
        self.payload = bytearray()
        self.expected_len = 0
        self.crc_bytes = bytearray()

    def feed(self, byte: int):
        if self.state == 0:
            if byte == SOF0:
                self.state = 1
            return None

        if self.state == 1:
            if byte == SOF1:
                self.state = 2
                self.header = bytearray()
            else:
                self.reset()
            return None

        if self.state == 2:
            self.header.append(byte)
            if len(self.header) == 5:
                self.expected_len = struct.unpack_from("<H", self.header, 3)[0]
                if self.expected_len > 240:
                    self.reset()
                    return None
                if self.expected_len == 0:
                    self.state = 4
                    self.crc_bytes = bytearray()
                else:
                    self.state = 3
                    self.payload = bytearray()
            return None

        if self.state == 3:
            self.payload.append(byte)
            if len(self.payload) >= self.expected_len:
                self.state = 4
                self.crc_bytes = bytearray()
            return None

        if self.state == 4:
            self.crc_bytes.append(byte)
            if len(self.crc_bytes) == 2:
                self.state = 0
                return self._check_frame()
            return None

        return None

    def _check_frame(self):
        body = bytes(self.header) + bytes(self.payload)
        crc_calc = crc16_ccitt(body)
        crc_recv = struct.unpack_from("<H", self.crc_bytes)[0]
        if crc_calc != crc_recv:
            print(f"[WARN] CRC mismatch: calc=0x{crc_calc:04X} recv=0x{crc_recv:04X}")
            return None
        msg_type = self.header[1]
        seq = self.header[2]
        return (msg_type, seq, bytes(self.payload))


def try_reset_input_buffer(ser: serial.Serial):
    try:
        ser.reset_input_buffer()
    except Exception:
        pass


def try_reset_output_buffer(ser: serial.Serial):
    try:
        ser.reset_output_buffer()
    except Exception:
        pass


def recv_frame(ser: serial.Serial, reader: FrameReader, timeout: float = 2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        waiting = ser.in_waiting
        if waiting <= 0:
            time.sleep(0.001)
            continue

        data = ser.read(1)
        if not data:
            continue

        result = reader.feed(data[0])
        if result is not None:
            return result
    return None


def is_unsolicited_stream_frame(frame) -> bool:
    return frame is not None and frame[0] == MSG_STATUS_STREAM


def open_serial_port(port: str, baud: int, settle_ms: int, open_retries: int) -> serial.Serial:
    last_error = None

    for attempt in range(1, open_retries + 1):
        ser = None
        try:
            ser = serial.Serial(
                port=port,
                baudrate=baud,
                timeout=0.05,
                write_timeout=1.0,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
                exclusive=None,
            )

            print(f"[LINK] open {port} {baud} (attempt {attempt}/{open_retries})")

            try:
                ser.dtr = False
                ser.rts = False
            except Exception:
                pass

            sleep_ms(settle_ms)

            try_reset_input_buffer(ser)
            try_reset_output_buffer(ser)
            return ser
        except (serial.SerialException, OSError, PermissionError) as e:
            last_error = e
            if ser is not None:
                try:
                    ser.close()
                except Exception:
                    pass
            if attempt < open_retries:
                print(f"[WARN] open failed: {e}")
                sleep_ms(settle_ms)

    raise serial.SerialException(f"open {port} failed after {open_retries} attempts: {last_error}")


def send_frame_and_wait(
    ser: serial.Serial,
    reader: FrameReader,
    frame: bytes,
    label: str,
    timeout: float,
    retries: int,
    retry_delay_ms: int,
    accept_fn=None,
):
    last_error = None

    for attempt in range(1, retries + 1):
        try:
            if attempt == 1:
                try_reset_input_buffer(ser)
            ser.write(frame)
            ser.flush()

            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                remaining = deadline - time.monotonic()
                result = recv_frame(ser, reader, timeout=max(remaining, 0.01))
                if result is None:
                    break

                if is_unsolicited_stream_frame(result):
                    continue

                if accept_fn is None or accept_fn(result):
                    return result

                print(f"[WARN] ignored unexpected frame ({label}): msg=0x{result[0]:02X}")

            print(f"[RX] timeout ({label}, attempt {attempt}/{retries})")
        except (serial.SerialException, OSError, PermissionError) as e:
            last_error = e
            print(f"[WARN] serial error ({label}, attempt {attempt}/{retries}): {e}")

        if attempt < retries:
            sleep_ms(retry_delay_ms)

    if last_error is not None:
        raise serial.SerialException(last_error)

    return None


def cmd_ping(ser: serial.Serial, retries: int, retry_delay_ms: int):
    reader = FrameReader()
    seq = 1
    payload = struct.pack("<I", int(time.time() * 1000) & 0xFFFFFFFF)
    frame = build_frame(MSG_PING_REQ, seq, payload)

    print(f"[TX] PING_REQ seq={seq}")
    result = send_frame_and_wait(
        ser,
        reader,
        frame,
        "PING_REQ",
        2.0,
        retries,
        retry_delay_ms,
        accept_fn=lambda r: r[0] in (MSG_ACK, MSG_NACK),
    )
    if result is None:
        return False

    msg_type, _, resp_payload = result
    if msg_type == MSG_ACK and len(resp_payload) >= 2:
        req_msg = resp_payload[0]
        status = resp_payload[1]
        print(f"[RX] ACK req=0x{req_msg:02X} status={status}")
        return req_msg == MSG_PING_REQ and status == 0

    print(f"[RX] unexpected: msg=0x{msg_type:02X}")
    return False


def cmd_info(ser: serial.Serial, retries: int, retry_delay_ms: int):
    reader = FrameReader()
    seq = 2
    frame = build_frame(MSG_GET_DEVICE_INFO_REQ, seq, b"")

    print(f"[TX] GET_DEVICE_INFO_REQ seq={seq}")
    result = send_frame_and_wait(
        ser,
        reader,
        frame,
        "GET_DEVICE_INFO_REQ",
        2.0,
        retries,
        retry_delay_ms,
        accept_fn=lambda r: r[0] in (MSG_DEVICE_INFO_RSP, MSG_NACK),
    )
    if result is None:
        return False

    msg_type, _, payload = result

    if msg_type == MSG_NACK and len(payload) >= 2:
        print(f"[RX] NACK req=0x{payload[0]:02X} reason={payload[1]}")
        return False

    if msg_type != MSG_DEVICE_INFO_RSP or len(payload) != 9:
        print(f"[RX] unexpected: msg=0x{msg_type:02X} len={len(payload)}")
        return False

    device_type = payload[0]
    proto_ver = payload[1]
    fw_major = payload[2]
    fw_minor = payload[3]
    fw_patch = payload[4]
    cap_flags = struct.unpack_from("<H", payload, 5)[0]
    max_payload = struct.unpack_from("<H", payload, 7)[0]

    print("[RX] DEVICE_INFO_RSP")
    print(f"  device_type   = 0x{device_type:02X}")
    print(f"  proto_version = {proto_ver}")
    print(f"  fw            = {fw_major}.{fw_minor}.{fw_patch}")
    print(f"  cap_flags     = 0x{cap_flags:04X}")
    print(f"  max_payload   = {max_payload}")
    return True


def cmd_stream(ser: serial.Serial, rate: int, retries: int, retry_delay_ms: int):
    reader = FrameReader()

    start_payload = struct.pack("<BBHI", 1, 0, rate, 0xFFFFFFFF)
    start_frame = build_frame(MSG_STREAM_CONTROL_REQ, 3, start_payload)

    print(f"[TX] STREAM_CONTROL_REQ enable=1 rate={rate}Hz")
    result = send_frame_and_wait(
        ser,
        reader,
        start_frame,
        "STREAM_CONTROL_REQ start",
        2.0,
        retries,
        retry_delay_ms,
        accept_fn=lambda r: r[0] in (MSG_ACK, MSG_NACK),
    )
    if result is None:
        return False

    msg_type, _, resp_payload = result
    if msg_type == MSG_ACK and len(resp_payload) >= 2 and resp_payload[1] == 0:
        print(f"[STREAM] started rate={rate}Hz")
    elif msg_type == MSG_NACK and len(resp_payload) >= 2:
        print(f"[RX] NACK req=0x{resp_payload[0]:02X} reason={resp_payload[1]}")
        return False
    else:
        print(f"[RX] unexpected: msg=0x{msg_type:02X}")
        return False

    running = True

    def on_sigint(_sig, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, on_sigint)

    count = 0
    t_start = time.time()

    while running:
        result = recv_frame(ser, reader, timeout=1.0)
        if result is None:
            continue

        msg_type, _, payload = result
        if msg_type != MSG_STATUS_STREAM:
            continue

        try:
            host_rx_time_ms = int(time.time() * 1000)
            frame = parse_status_stream(payload, host_rx_time_ms)
        except ValueError as e:
            print(f"[WARN] stream parse error: {e}")
            continue

        tick_ms = frame.tick_ms
        pitch_target = frame.pitch_target_deg
        speed_p_term = frame.speed_p_term_deg
        speed_i_term = frame.speed_i_term_deg
        pitch_meas = frame.pitch_meas_deg
        pitch_rate = frame.pitch_rate_dps
        speed_target = frame.speed_target_radps
        speed_meas = frame.speed_meas_radps
        attitude_p_term = frame.attitude_p_iq_cmd_a
        attitude_d_term = frame.attitude_d_iq_cmd_a
        iq_cmd = frame.iq_cmd_a
        iq_cmd_clamped = frame.iq_cmd_clamped_a
        speed_output_limit = frame.speed_output_limit_deg
        attitude_output_limit = frame.attitude_output_limit_a
        iq_l = frame.iq_l_a
        iq_r = frame.iq_r_a
        uq_l = frame.uq_l_v
        uq_r = frame.uq_r_v
        bus_v = frame.bus_v
        fault = frame.fault_flags
        fault_text = frame.fault_labels

        print(
            f"t={tick_ms:>8} pitch_target={pitch_target:>+6.2f}deg "
            f"speed_p_term={speed_p_term:>+6.2f}deg "
            f"speed_i_term={speed_i_term:>+6.2f}deg "
            f"pitch_meas={pitch_meas:>+6.2f}deg pitch_rate={pitch_rate:>+7.2f}dps "
            f"speed_target={speed_target:>+6.3f} speed_meas={speed_meas:>+6.3f} "
            f"attitude_p_term={attitude_p_term:>+5.3f} "
            f"attitude_d_term={attitude_d_term:>+5.3f} "
            f"iq_cmd={iq_cmd:>+5.3f} iq_cmd_clamped={iq_cmd_clamped:>+5.3f} "
            f"speed_output_limit={speed_output_limit:>+6.2f}deg "
            f"attitude_output_limit={attitude_output_limit:>+5.3f} "
            f"iq_l={iq_l:>+5.3f} iq_r={iq_r:>+5.3f} "
            f"uq_l={uq_l:>5.2f}V uq_r={uq_r:>5.2f}V "
            f"bus={bus_v:>6.2f}V fault=0x{fault:04X} [{fault_text}]"
        )
        count += 1

    elapsed = time.time() - t_start
    print(
        f"\n[STREAM] stopped. {count} frames in {elapsed:.1f}s "
        f"({count / elapsed if elapsed > 0 else 0:.1f} Hz)"
    )

    stop_payload = struct.pack("<BBHI", 0, 0, rate, 0)
    stop_frame = build_frame(MSG_STREAM_CONTROL_REQ, 4, stop_payload)
    result = send_frame_and_wait(
        ser,
        reader,
        stop_frame,
        "STREAM_CONTROL_REQ stop",
        1.0,
        retries,
        retry_delay_ms,
        accept_fn=lambda r: r[0] in (MSG_ACK, MSG_NACK),
    )
    if result and result[0] == MSG_ACK:
        print("[TX] STREAM_CONTROL_REQ enable=0 -> ACK")
    else:
        print("[TX] STREAM_CONTROL_REQ enable=0 -> no ACK")

    return True


def cmd_driver(ser: serial.Serial, enable: bool, retries: int, retry_delay_ms: int):
    reader = FrameReader()
    seq = 5
    payload = struct.pack("<B", 1 if enable else 0)
    frame = build_frame(MSG_POWER_STAGE_REQ, seq, payload)
    action = "on" if enable else "off"

    print(f"[TX] POWER_STAGE_REQ state={action}")
    result = send_frame_and_wait(
        ser,
        reader,
        frame,
        "POWER_STAGE_REQ",
        2.0,
        retries,
        retry_delay_ms,
        accept_fn=lambda r: r[0] in (MSG_ACK, MSG_NACK),
    )
    if result is None:
        return False

    msg_type, _, resp_payload = result
    if msg_type == MSG_ACK and len(resp_payload) >= 2:
        req_msg = resp_payload[0]
        status = resp_payload[1]
        print(f"[RX] ACK req=0x{req_msg:02X} status={status}")
        return req_msg == MSG_POWER_STAGE_REQ and status == 0

    if msg_type == MSG_NACK and len(resp_payload) >= 2:
        print(f"[RX] NACK req=0x{resp_payload[0]:02X} reason={resp_payload[1]}")
        return False

    print(f"[RX] unexpected: msg=0x{msg_type:02X}")
    return False


def cmd_balance(ser: serial.Serial, enable: bool, retries: int, retry_delay_ms: int):
    reader = FrameReader()
    seq = 6
    payload = struct.pack("<B", 1 if enable else 0)
    frame = build_frame(MSG_ATTITUDE_CONTROL_REQ, seq, payload)
    action = "on" if enable else "off"

    print(f"[TX] ATTITUDE_CONTROL_REQ state={action}")
    result = send_frame_and_wait(
        ser,
        reader,
        frame,
        "ATTITUDE_CONTROL_REQ",
        2.0,
        retries,
        retry_delay_ms,
        accept_fn=lambda r: r[0] in (MSG_ACK, MSG_NACK),
    )
    if result is None:
        return False

    msg_type, _, resp_payload = result
    if msg_type == MSG_ACK and len(resp_payload) >= 2:
        req_msg = resp_payload[0]
        status = resp_payload[1]
        print(f"[RX] ACK req=0x{req_msg:02X} status={status}")
        return req_msg == MSG_ATTITUDE_CONTROL_REQ and status == 0

    if msg_type == MSG_NACK and len(resp_payload) >= 2:
        print(f"[RX] NACK req=0x{resp_payload[0]:02X} reason={resp_payload[1]}")
        return False

    print(f"[RX] unexpected: msg=0x{msg_type:02X}")
    return False


def cmd_fastcap_arm(ser: serial.Serial, side: str, retries: int, retry_delay_ms: int):
    reader = FrameReader()
    seq = 7
    source = FASTCAP_SOURCE_LEFT if side.upper() == "L" else FASTCAP_SOURCE_RIGHT
    payload = struct.pack("<BB", OP_FASTCAP_ARM, source)
    frame = build_frame(MSG_FAST_CAPTURE_REQ, seq, payload)

    print(f"[TX] FAST_CAPTURE_REQ ARM source={side.upper()}")
    result = send_frame_and_wait(
        ser, reader, frame, "FASTCAP_ARM",
        2.0, retries, retry_delay_ms,
        accept_fn=lambda r: r[0] in (MSG_ACK, MSG_NACK),
    )
    if result is None:
        return False

    msg_type, _, resp = result
    if msg_type == MSG_ACK and len(resp) >= 2 and resp[1] == 0:
        print("[RX] ACK -> FastLog armed")
        return True
    if msg_type == MSG_NACK and len(resp) >= 2:
        nack_reasons = {1: "BAD_LENGTH", 3: "BAD_PARAM_VALUE", 4: "BUSY", 5: "UNSUPPORTED"}
        reason = nack_reasons.get(resp[1], f"code={resp[1]}")
        print(f"[RX] NACK reason={reason}")
    return False


def cmd_fastcap_status(ser: serial.Serial, retries: int, retry_delay_ms: int):
    reader = FrameReader()
    seq = 8
    payload = struct.pack("<B", OP_FASTCAP_STATUS)
    frame = build_frame(MSG_FAST_CAPTURE_REQ, seq, payload)

    print("[TX] FAST_CAPTURE_REQ STATUS")
    result = send_frame_and_wait(
        ser, reader, frame, "FASTCAP_STATUS",
        2.0, retries, retry_delay_ms,
        accept_fn=lambda r: r[0] == MSG_FAST_CAPTURE_DATA,
    )
    if result is None:
        return None

    _, _, pld = result
    try:
        meta = parse_fastcap_status(pld)
    except ValueError as e:
        print(f"[WARN] fastcap status parse error: {e}")
        return None

    source      = meta.source
    capture_id  = meta.capture_id
    total_count = meta.total_count
    armed       = 1 if meta.armed else 0
    done        = 1 if meta.done else 0
    blocked     = 1 if meta.blocked else 0
    capacity    = meta.capacity

    side_label = "R" if source == FASTCAP_SOURCE_RIGHT else "L"

    print(f"[RX] FAST_CAPTURE_DATA STATUS")
    print(f"  source       = {side_label}")
    print(f"  capture_id   = {capture_id}")
    print(f"  total_count  = {total_count}")
    print(f"  armed        = {armed}")
    print(f"  done         = {done}")
    print(f"  blocked      = {blocked}")
    print(f"  capacity     = {capacity}")

    return {
        "source": source,
        "capture_id": capture_id,
        "total_count": total_count,
        "armed": armed,
        "done": done,
        "blocked": blocked,
        "capacity": capacity,
    }


def cmd_fastcap_stop(ser: serial.Serial, retries: int, retry_delay_ms: int):
    reader = FrameReader()
    seq = 9
    payload = struct.pack("<B", OP_FASTCAP_STOP)
    frame = build_frame(MSG_FAST_CAPTURE_REQ, seq, payload)

    print("[TX] FAST_CAPTURE_REQ STOP")
    result = send_frame_and_wait(
        ser, reader, frame, "FASTCAP_STOP",
        2.0, retries, retry_delay_ms,
        accept_fn=lambda r: r[0] in (MSG_ACK, MSG_NACK),
    )
    if result is None:
        return False

    msg_type, _, resp = result
    if msg_type == MSG_ACK and len(resp) >= 2 and resp[1] == 0:
        print("[RX] ACK -> FastLog stopped")
        return True
    if msg_type == MSG_NACK and len(resp) >= 2:
        print(f"[RX] NACK reason={resp[1]}")
    return False


def cmd_fastcap_dump(
    ser: serial.Serial,
    side: str,
    out_file: str,
    retries: int,
    retry_delay_ms: int,
    poll_ms: int = 50,
    timeout_s: float = 5.0,
):
    reader = FrameReader()
    seq_base = 20
    requested_source = FASTCAP_SOURCE_LEFT if side.upper() == "L" else FASTCAP_SOURCE_RIGHT
    status = cmd_fastcap_status(ser, retries, retry_delay_ms)

    if (
        status is not None
        and status["done"]
        and status["total_count"] > 0
        and status["source"] == requested_source
    ):
        print("[DUMP] reusing completed capture from matching side")
    else:
        if status is not None and status["done"] and status["total_count"] > 0:
            current_side = "R" if status["source"] == FASTCAP_SOURCE_RIGHT else "L"
            print(f"[DUMP] clearing completed capture from side {current_side} before re-arming")
            if not cmd_fastcap_stop(ser, retries, retry_delay_ms):
                print("[ERROR] STOP failed while clearing previous capture")
                return False

        if not cmd_fastcap_arm(ser, side, retries, retry_delay_ms):
            print("[ERROR] ARM failed")
            return False
        seq_base += 1

        deadline = time.monotonic() + timeout_s
        status = None
        while time.monotonic() < deadline:
            sleep_ms(poll_ms)
            status = cmd_fastcap_status(ser, retries, retry_delay_ms)
            if status is None:
                continue
            seq_base += 1
            if status["done"]:
                break

        if status is None or not status["done"]:
            print(f"[ERROR] capture did not complete within {timeout_s}s")
            if status:
                print(f"  armed={status['armed']} done={status['done']}")
            return False

    total_count = status["total_count"]
    capture_id = status["capture_id"]
    source = status["source"]
    side_label = "R" if source == FASTCAP_SOURCE_RIGHT else "L"

    print(f"[DUMP] capture_id={capture_id} motor={side_label} samples={total_count}")

    all_samples = []
    start_idx = 0
    while start_idx < total_count:
        remaining = total_count - start_idx
        max_req = min(FASTCAP_CHUNK_MAX, remaining)

        payload = struct.pack("<BHB", OP_FASTCAP_READ_CHUNK, start_idx, max_req)
        frame = build_frame(MSG_FAST_CAPTURE_REQ, seq_base & 0xFF, payload)
        seq_base += 1

        result = send_frame_and_wait(
            ser, reader, frame, f"FASTCAP_READ_CHUNK start={start_idx}",
            2.0, retries, retry_delay_ms,
            accept_fn=lambda r: r[0] == MSG_FAST_CAPTURE_DATA,
        )
        if result is None:
            print(f"[WARN] READ_CHUNK start={start_idx} timed out, retrying ...")
            continue

        _, _, pld = result
        try:
            chunk_meta, chunk_samples = parse_fastcap_chunk(pld)
        except ValueError as e:
            print(f"[WARN] fastcap chunk parse error: {e}")
            continue

        if chunk_meta.op_echo != OP_FASTCAP_READ_CHUNK:
            print(f"[WARN] unexpected op_echo=0x{chunk_meta.op_echo:02X}, skipping")
            continue
        if chunk_meta.capture_id != capture_id:
            print(f"[WARN] capture_id mismatch: got {chunk_meta.capture_id}, expected {capture_id}, skipping")
            continue
        if chunk_meta.source != source:
            print(f"[WARN] source mismatch: got {chunk_meta.source}, expected {source}, skipping")
            continue

        sample_count = len(chunk_samples)
        if sample_count == 0:
            break

        if chunk_samples[0].index != start_idx:
            print(f"[WARN] start_idx mismatch: got {chunk_samples[0].index}, expected {start_idx}, skipping")
            continue

        all_samples.extend(chunk_samples)

        start_idx += sample_count

    try:
        with open(out_file, "w", newline="") as f:
            f.write("idx,target_iq,iq_ref,filtered_iq,raw_iq,uq_final,source,capture_id\n")
            for idx, s in enumerate(all_samples):
                f.write(f"{idx},{s.target_iq_a:.6f},{s.iq_ref_a:.6f},{s.filtered_iq_a:.6f},{s.raw_iq_a:.6f},{s.uq_final_v:.6f},{side_label},{capture_id}\n")
        print(f"[DUMP] wrote {len(all_samples)} samples to {out_file}")
    except OSError as e:
        print(f"[ERROR] cannot write {out_file}: {e}")
        if not cmd_fastcap_stop(ser, retries, retry_delay_ms):
            print("[WARN] STOP after write failure did not ACK")
        return False

    cmd_fastcap_stop(ser, retries, retry_delay_ms)
    return True


def main():
    parser = argparse.ArgumentParser(description="DebugLink CLI")
    parser.add_argument("--port", required=True, help="Serial port (e.g. COM33)")
    parser.add_argument("--baud", type=int, default=921600, help="Baud rate")
    parser.add_argument(
        "--open-retries",
        type=int,
        default=DEFAULT_OPEN_RETRIES,
        help="Retry count when opening the serial port",
    )
    parser.add_argument(
        "--retries",
        type=int,
        default=DEFAULT_CMD_RETRIES,
        help="Retry count for request/response commands",
    )
    parser.add_argument(
        "--settle-ms",
        type=int,
        default=DEFAULT_SETTLE_MS,
        help="Delay after open before first TX",
    )
    parser.add_argument(
        "--retry-delay-ms",
        type=int,
        default=DEFAULT_RETRY_DELAY_MS,
        help="Delay between command retries",
    )
    sub = parser.add_subparsers(dest="command")

    sub.add_parser("ping", help="Send PING_REQ")
    sub.add_parser("info", help="Get device info")
    p_driver = sub.add_parser("driver", help="Enable or disable the power stage")
    p_driver.add_argument("state", choices=("on", "off"), help="Target driver state")
    p_balance = sub.add_parser("balance", help="Enable or disable the attitude/speed control loop")
    p_balance.add_argument("state", choices=("on", "off"), help="Target balance state")

    p_stream = sub.add_parser("stream", help="Start status stream")
    p_stream.add_argument("--rate", type=int, default=100, help="Stream rate Hz")

    p_fastcap = sub.add_parser("fastcap", help="FastLog capture commands (binary protocol)")
    fastcap_sub = p_fastcap.add_subparsers(dest="fastcap_op")

    p_fc_arm = fastcap_sub.add_parser("arm", help="Arm FastLog")
    p_fc_arm.add_argument("side", choices=("L", "R"), help="Motor side (L or R)")

    fastcap_sub.add_parser("status", help="Query FastLog status")
    fastcap_sub.add_parser("stop", help="Stop active FastLog capture")

    p_fc_dump = fastcap_sub.add_parser("dump", help="Arm, wait, read all, and write CSV")
    p_fc_dump.add_argument("--side", required=True, choices=("L", "R"), help="Motor side")
    p_fc_dump.add_argument("--out", required=True, help="Output CSV file path")
    p_fc_dump.add_argument("--timeout", type=float, default=5.0, help="Capture completion timeout seconds")
    p_fc_dump.add_argument("--poll-ms", type=int, default=50, help="STATUS poll interval ms")

    args = parser.parse_args()

    if args.command is None:
        parser.print_help()
        sys.exit(1)

    ser = None
    ok = False
    try:
        ser = open_serial_port(args.port, args.baud, args.settle_ms, args.open_retries)
        if args.command == "ping":
            ok = cmd_ping(ser, args.retries, args.retry_delay_ms)
        elif args.command == "info":
            ok = cmd_info(ser, args.retries, args.retry_delay_ms)
        elif args.command == "driver":
            ok = cmd_driver(ser, args.state == "on", args.retries, args.retry_delay_ms)
        elif args.command == "balance":
            ok = cmd_balance(ser, args.state == "on", args.retries, args.retry_delay_ms)
        elif args.command == "stream":
            ok = cmd_stream(ser, args.rate, args.retries, args.retry_delay_ms)
        elif args.command == "fastcap":
            if args.fastcap_op is None:
                p_fastcap.print_help()
                sys.exit(1)
            if args.fastcap_op == "arm":
                ok = cmd_fastcap_arm(ser, args.side, args.retries, args.retry_delay_ms)
            elif args.fastcap_op == "status":
                ok = cmd_fastcap_status(ser, args.retries, args.retry_delay_ms) is not None
            elif args.fastcap_op == "stop":
                ok = cmd_fastcap_stop(ser, args.retries, args.retry_delay_ms)
            elif args.fastcap_op == "dump":
                ok = cmd_fastcap_dump(
                    ser, args.side, args.out,
                    args.retries, args.retry_delay_ms,
                    args.poll_ms, args.timeout,
                )
    except serial.SerialException as e:
        print(f"[ERROR] {e}")
    finally:
        if ser is not None:
            ser.close()

    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
