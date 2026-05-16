"""
DebugLink protocol parser — wire bytes -> strongly-typed model objects.

All layouts match the firmware in MyCode/System/debug_link.c.
Little-endian throughout.

STATUS_STREAM (0x90) — 42 bytes  (DebugLink_StatusSnapshot_t)
----------------------------------------------------------------------------
Offset  Size  C field                       Python field              Scale
----------------------------------------------------------------------------
  0      4    tick_ms                       tick_ms                     1
  4      2    pitch_target_deg_x100         pitch_target_deg          /100.0
  6      2    speed_p_term_deg_x100         speed_p_term_deg          /100.0
  8      2    speed_i_term_deg_x100         speed_i_term_deg          /100.0
 10      2    pitch_meas_deg_x100           pitch_meas_deg            /100.0
 12      2    pitch_rate_dps_x100           pitch_rate_dps            /100.0
 14      2    speed_target_radps_x1000      speed_target_radps        /1000.0
 16      2    speed_meas_radps_x1000        speed_meas_radps          /1000.0
 18      2    attitude_p_term_ma            attitude_p_iq_cmd_a       /1000.0
 20      2    attitude_d_term_ma            attitude_d_iq_cmd_a       /1000.0
 22      2    iq_cmd_ma                     iq_cmd_a                  /1000.0
 24      2    iq_cmd_clamped_ma             iq_cmd_clamped_a          /1000.0
 26      2    speed_output_limit_deg_x100   speed_output_limit_deg    /100.0
 28      2    attitude_output_limit_ma      attitude_output_limit_a   /1000.0
 30      2    iq_l_x1000                    iq_l_a                    /1000.0
 32      2    iq_r_x1000                    iq_r_a                    /1000.0
 34      2    uq_l_mv                       uq_l_v                    /1000.0
 36      2    uq_r_mv                       uq_r_v                    /1000.0
 38      2    bus_mv                        bus_v                     /1000.0
 40      2    fault_flags                   fault_flags                 1
----------------------------------------------------------------------------

FASTCAP_STATUS  (0x91, op_echo = 0x02) — 13 bytes  (debug_link.c:319-327)
----------------------------------------------------------------------------
Offset  Size  Field            Notes
----------------------------------------------------------------------------
  0      1    op_echo          uint8, echoes DL_FASTCAP_OP_STATUS (0x02)
  1      1    source           uint8, 0=L 1=R
  2      4    capture_id       uint32 LE
  6      2    total_count      uint16 LE
  8      1    armed            uint8 → bool
  9      1    done             uint8 → bool
 10      1    blocked          uint8 → bool
 11      2    capacity         uint16 LE  (APP_FASTLOG_SIZE)
----------------------------------------------------------------------------

FASTCAP_CHUNK   (0x91, op_echo = 0x03) — header + samples  (debug_link.c:372-397)
----------------------------------------------------------------------------
Offset  Size  Field            Notes
----------------------------------------------------------------------------
  0      1    op_echo          uint8, echoes DL_FASTCAP_OP_READ_CHUNK (0x03)
  1      1    source           uint8, 0=L 1=R
  2      4    capture_id       uint32 LE
  6      2    total_count      uint16 LE
  8      2    start_idx        uint16 LE, first sample index in this chunk
 10      1    sample_count     uint8
 11     N*10  samples          N = sample_count
----------------------------------------------------------------------------

Each sample (10 bytes, 5 × int16 LE):
----------------------------------------------------------------------------
Offset  Size  C field           Python field        Scale
----------------------------------------------------------------------------
  0      2    target_iq  (i16)  target_iq_a         /1000.0  (mA → A)
  2      2    iq_ref     (i16)  iq_ref_a            /1000.0
  4      2    filtered_iq(i16)  filtered_iq_a       /1000.0
  6      2    raw_iq     (i16)  raw_iq_a            /1000.0
  8      2    uq_final   (i16)  uq_final_v          /1000.0  (mV → V)
----------------------------------------------------------------------------

capture_id, source, and index in FastCapSample are populated from the
chunk header context — they are not repeated per-sample on the wire.
"""

import struct

from debuglink_models import FastCapMeta, FastCapSample, LiveFrame

# ---------------------------------------------------------------------------
# Fault-flag decoding (mirrors debug_link.h / debuglink_cli.py)
# ---------------------------------------------------------------------------

_FOC_FLAG_SPEED_FAULT_L = 1 << 0
_FOC_FLAG_SPEED_FAULT_R = 1 << 1
_FOC_FLAG_STACK_READY = 1 << 8
_FOC_FLAG_CONTROL_IT_ENABLED = 1 << 9
_FOC_FLAG_BUS_VALID = 1 << 10
_FOC_FLAG_CURRENT_LOOP_ACTIVE = 1 << 11
_FOC_FLAG_SPEED_LOOP_ENABLED = 1 << 12
_FOC_FLAG_CURRENT_LOOP_ENABLED = 1 << 13
_FOC_FLAG_POWER_STAGE_OFF = 1 << 14
_FOC_FLAG_ATTITUDE_CONTROL_ON = 1 << 15

_FLAG_TABLE = [
    (_FOC_FLAG_SPEED_FAULT_L, "spdfltL"),
    (_FOC_FLAG_SPEED_FAULT_R, "spdfltR"),
    (_FOC_FLAG_STACK_READY, "stack"),
    (_FOC_FLAG_CONTROL_IT_ENABLED, "it"),
    (_FOC_FLAG_BUS_VALID, "bus"),
    (_FOC_FLAG_CURRENT_LOOP_ACTIVE, "iloop"),
    (_FOC_FLAG_SPEED_LOOP_ENABLED, "sloop"),
    (_FOC_FLAG_CURRENT_LOOP_ENABLED, "cloop"),
    (_FOC_FLAG_POWER_STAGE_OFF, "drv_off"),
    (_FOC_FLAG_ATTITUDE_CONTROL_ON, "bal"),
]

_STREAM_PAYLOAD_SIZE = 42
_FASTCAP_STATUS_SIZE = 13       # debug_link.c line 329: payload, 13U
_FASTCAP_CHUNK_HEADER_SIZE = 11  # op(1)+src(1)+cap_id(4)+total(2)+start(2)+count(1)
_FASTCAP_SAMPLE_SIZE = 10        # 5 × int16


def _decode_fault_labels(flags: int) -> str:
    labels = [label for mask, label in _FLAG_TABLE if flags & mask]
    return ",".join(labels) if labels else "-"


# ---------------------------------------------------------------------------
# source_to_label
# ---------------------------------------------------------------------------


def source_to_label(v: int) -> str:
    """Map source integer to human-readable label.  0→L, 1→R."""
    if v == 0:
        return "L"
    if v == 1:
        return "R"
    raise ValueError(f"invalid source value: {v} (expected 0 or 1)")


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------


def parse_status_stream(payload: bytes, host_rx_time_ms: int) -> LiveFrame:
    """Unpack a 42-byte STATUS_STREAM (0x90) payload into a LiveFrame."""
    if len(payload) < _STREAM_PAYLOAD_SIZE:
        raise ValueError(
            f"STATUS_STREAM payload too short: need {_STREAM_PAYLOAD_SIZE} bytes, got {len(payload)}"
        )

    tick_ms = struct.unpack_from("<I", payload, 0)[0]
    pitch_target_deg = struct.unpack_from("<h", payload, 4)[0] / 100.0
    speed_p_term_deg = struct.unpack_from("<h", payload, 6)[0] / 100.0
    speed_i_term_deg = struct.unpack_from("<h", payload, 8)[0] / 100.0
    pitch_meas_deg = struct.unpack_from("<h", payload, 10)[0] / 100.0
    pitch_rate_dps = struct.unpack_from("<h", payload, 12)[0] / 100.0
    speed_target_radps = struct.unpack_from("<h", payload, 14)[0] / 1000.0
    speed_meas_radps = struct.unpack_from("<h", payload, 16)[0] / 1000.0
    attitude_p_iq_cmd_a = struct.unpack_from("<h", payload, 18)[0] / 1000.0
    attitude_d_iq_cmd_a = struct.unpack_from("<h", payload, 20)[0] / 1000.0
    iq_cmd_a = struct.unpack_from("<h", payload, 22)[0] / 1000.0
    iq_cmd_clamped_a = struct.unpack_from("<h", payload, 24)[0] / 1000.0
    speed_output_limit_deg = struct.unpack_from("<h", payload, 26)[0] / 100.0
    attitude_output_limit_a = struct.unpack_from("<h", payload, 28)[0] / 1000.0
    iq_l_a = struct.unpack_from("<h", payload, 30)[0] / 1000.0
    iq_r_a = struct.unpack_from("<h", payload, 32)[0] / 1000.0
    uq_l_v = struct.unpack_from("<h", payload, 34)[0] / 1000.0
    uq_r_v = struct.unpack_from("<h", payload, 36)[0] / 1000.0
    bus_v = struct.unpack_from("<H", payload, 38)[0] / 1000.0
    fault_flags = struct.unpack_from("<H", payload, 40)[0]
    fault_labels = _decode_fault_labels(fault_flags)

    return LiveFrame(
        tick_ms=tick_ms,
        pitch_target_deg=pitch_target_deg,
        speed_p_term_deg=speed_p_term_deg,
        speed_i_term_deg=speed_i_term_deg,
        pitch_meas_deg=pitch_meas_deg,
        pitch_rate_dps=pitch_rate_dps,
        speed_target_radps=speed_target_radps,
        speed_meas_radps=speed_meas_radps,
        attitude_p_iq_cmd_a=attitude_p_iq_cmd_a,
        attitude_d_iq_cmd_a=attitude_d_iq_cmd_a,
        iq_cmd_a=iq_cmd_a,
        iq_cmd_clamped_a=iq_cmd_clamped_a,
        speed_output_limit_deg=speed_output_limit_deg,
        attitude_output_limit_a=attitude_output_limit_a,
        iq_l_a=iq_l_a,
        iq_r_a=iq_r_a,
        uq_l_v=uq_l_v,
        uq_r_v=uq_r_v,
        bus_v=bus_v,
        fault_flags=fault_flags,
        fault_labels=fault_labels,
        host_rx_time_ms=host_rx_time_ms,
    )


def parse_fastcap_status(payload: bytes) -> FastCapMeta:
    """Unpack a 0x91 STATUS-reply payload into FastCapMeta.

    Wire layout (13 bytes, debug_link.c:319-327):
        op_echo(1) source(1) capture_id(u32) total_count(u16)
        armed(1) done(1) blocked(1) capacity(u16)
    """
    if len(payload) < _FASTCAP_STATUS_SIZE:
        raise ValueError(
            f"FASTCAP_STATUS payload too short: need {_FASTCAP_STATUS_SIZE} bytes, got {len(payload)}"
        )

    op_echo = payload[0]
    source = payload[1]
    if source not in (0, 1):
        raise ValueError(f"invalid source in FASTCAP_STATUS: {source}")

    capture_id = struct.unpack_from("<I", payload, 2)[0]
    total_count = struct.unpack_from("<H", payload, 6)[0]
    armed = bool(payload[8])
    done = bool(payload[9])
    blocked = bool(payload[10])
    capacity = struct.unpack_from("<H", payload, 11)[0]

    return FastCapMeta(
        op_echo=op_echo,
        capture_id=capture_id,
        source=source,
        total_count=total_count,
        armed=armed,
        done=done,
        blocked=blocked,
        capacity=capacity,
    )


def parse_fastcap_chunk(payload: bytes) -> tuple[FastCapMeta, list[FastCapSample]]:
    """Unpack a 0x91 READ_CHUNK-reply payload.

    Wire layout (debug_link.c:372-397):
        op_echo(1) source(1) capture_id(u32) total_count(u16)
        start_idx(u16) sample_count(u8)  +  N × 10-byte samples

    Returns (FastCapMeta, [FastCapSample, ...]).
    FastCapMeta.armed / .done / .blocked / .capacity are set to False/0
    because the chunk header does not carry them.
    """
    if len(payload) < _FASTCAP_CHUNK_HEADER_SIZE:
        raise ValueError(
            f"FASTCAP_CHUNK payload too short: need at least {_FASTCAP_CHUNK_HEADER_SIZE} bytes, got {len(payload)}"
        )

    op_echo = payload[0]
    source = payload[1]
    if source not in (0, 1):
        raise ValueError(f"invalid source in FASTCAP_CHUNK: {source}")

    capture_id = struct.unpack_from("<I", payload, 2)[0]
    total_count = struct.unpack_from("<H", payload, 6)[0]
    start_idx = struct.unpack_from("<H", payload, 8)[0]
    sample_count = payload[10]

    expected_len = _FASTCAP_CHUNK_HEADER_SIZE + sample_count * _FASTCAP_SAMPLE_SIZE
    if len(payload) != expected_len:
        raise ValueError(
            f"FASTCAP_CHUNK payload length mismatch: expected {expected_len} bytes "
            f"for sample_count={sample_count}, got {len(payload)}"
        )

    samples = []
    offset = _FASTCAP_CHUNK_HEADER_SIZE
    for i in range(sample_count):
        target_iq_a = struct.unpack_from("<h", payload, offset)[0] / 1000.0
        iq_ref_a = struct.unpack_from("<h", payload, offset + 2)[0] / 1000.0
        filtered_iq_a = struct.unpack_from("<h", payload, offset + 4)[0] / 1000.0
        raw_iq_a = struct.unpack_from("<h", payload, offset + 6)[0] / 1000.0
        uq_final_v = struct.unpack_from("<h", payload, offset + 8)[0] / 1000.0

        samples.append(
            FastCapSample(
                capture_id=capture_id,
                source=source,
                index=start_idx + i,
                target_iq_a=target_iq_a,
                iq_ref_a=iq_ref_a,
                filtered_iq_a=filtered_iq_a,
                raw_iq_a=raw_iq_a,
                uq_final_v=uq_final_v,
            )
        )
        offset += _FASTCAP_SAMPLE_SIZE

    meta = FastCapMeta(
        op_echo=op_echo,
        capture_id=capture_id,
        source=source,
        total_count=total_count,
        armed=False,
        done=False,
        blocked=False,
        capacity=0,
    )

    return meta, samples


# ---------------------------------------------------------------------------
# Self-test  —  verifies parser against firmware-consistent wire samples
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import sys

    ok = 0
    fail = 0

    def check(desc, cond):
        global ok, fail
        if cond:
            ok += 1
        else:
            fail += 1
            print(f"  FAIL: {desc}", file=sys.stderr)

    # --- source_to_label ---
    check("source_to_label(0) == 'L'", source_to_label(0) == "L")
    check("source_to_label(1) == 'R'", source_to_label(1) == "R")
    try:
        source_to_label(2)
        check("source_to_label(2) raises", False)
    except ValueError:
        check("source_to_label(2) raises", True)

    # --- 42-byte STATUS_STREAM ---
    stream_payload = struct.pack(
        "<I"
        "hhhhhhhhhhhhhhhhh"
        "H"
        "H",
        10000,   # tick_ms
        100,     # pitch_target = 1.00 deg
        50,      # speed_p_term = 0.50 deg
        25,      # speed_i_term = 0.25 deg
        110,     # pitch_meas = 1.10 deg
        10,      # pitch_rate = 0.10 dps
        500,     # speed_target = 0.500 rad/s
        450,     # speed_meas = 0.450 rad/s
        200,     # attitude_p_iq = 0.200 A
        100,     # attitude_d_iq = 0.100 A
        300,     # iq_cmd = 0.300 A
        280,     # iq_cmd_clamped = 0.280 A
        2000,    # speed_output_limit = 20.00 deg
        500,     # attitude_output_limit = 0.500 A
        250,     # iq_l = 0.250 A
        260,     # iq_r = 0.260 A
        12000,   # uq_l = 12.000 V
        12000,   # uq_r = 12.000 V
        24000,   # bus = 24.000 V
        0xC703,  # fault: spdfltL,spdfltR,stack,it,bus,drv_off,bal
    )
    assert len(stream_payload) == 42, f"expected 42, got {len(stream_payload)}"

    frame = parse_status_stream(stream_payload, host_rx_time_ms=999)
    check("stream tick_ms", frame.tick_ms == 10000)
    check("stream pitch_target", frame.pitch_target_deg == 1.00)
    check("stream pitch_meas", frame.pitch_meas_deg == 1.10)
    check("stream pitch_rate", frame.pitch_rate_dps == 0.10)
    check("stream speed_target", frame.speed_target_radps == 0.500)
    check("stream speed_meas", frame.speed_meas_radps == 0.450)
    check("stream attitude_p_iq", frame.attitude_p_iq_cmd_a == 0.200)
    check("stream attitude_d_iq", frame.attitude_d_iq_cmd_a == 0.100)
    check("stream iq_cmd", frame.iq_cmd_a == 0.300)
    check("stream iq_cmd_clamped", frame.iq_cmd_clamped_a == 0.280)
    check("stream speed_output_limit", frame.speed_output_limit_deg == 20.00)
    check("stream attitude_output_limit", frame.attitude_output_limit_a == 0.500)
    check("stream iq_l", frame.iq_l_a == 0.250)
    check("stream iq_r", frame.iq_r_a == 0.260)
    check("stream uq_l", frame.uq_l_v == 12.000)
    check("stream uq_r", frame.uq_r_v == 12.000)
    check("stream bus", frame.bus_v == 24.000)
    check("stream fault_flags", frame.fault_flags == 0xC703)
    check("stream fault_labels", "spdfltL" in frame.fault_labels and "bal" in frame.fault_labels)
    check("stream host_rx_time_ms", frame.host_rx_time_ms == 999)

    # --- stream short payload raises ---
    try:
        parse_status_stream(b"\x00" * 20, 0)
        check("stream short raises", False)
    except ValueError as e:
        check("stream short raises", "too short" in str(e))

    # === FASTCAP_STATUS (13 bytes, firmware layout debug_link.c:319-327) ===

    status_payload = struct.pack(
        "<B B I H B B B H",
        0x02,      # op_echo = DL_FASTCAP_OP_STATUS
        0,         # source = L
        42,        # capture_id (u32)
        256,       # total_count
        1,         # armed
        0,         # done
        0,         # blocked
        512,       # capacity
    )
    assert len(status_payload) == 13, f"expected 13, got {len(status_payload)}"

    meta = parse_fastcap_status(status_payload)
    check("fc_status op_echo", meta.op_echo == 0x02)
    check("fc_status capture_id", meta.capture_id == 42)
    check("fc_status source", meta.source == 0)
    check("fc_status total_count", meta.total_count == 256)
    check("fc_status armed", meta.armed is True)
    check("fc_status done", meta.done is False)
    check("fc_status blocked", meta.blocked is False)
    check("fc_status capacity", meta.capacity == 512)

    # --- fastcap status short ---
    try:
        parse_fastcap_status(b"\x00" * 5)
        check("fc_status short raises", False)
    except ValueError as e:
        check("fc_status short raises", "too short" in str(e))

    # === FASTCAP_CHUNK (11-byte header + N×10-byte samples) ===

    sample_count = 2
    start_idx = 0

    # Build chunk header (11 bytes)
    chunk_header = struct.pack(
        "<B B I H H B",
        0x03,      # op_echo = DL_FASTCAP_OP_READ_CHUNK
        1,         # source = R
        42,        # capture_id (u32)
        256,       # total_count
        start_idx, # start_idx
        sample_count,
    )
    assert len(chunk_header) == 11

    # Build two 10-byte samples (5 × i16 each)
    s0 = struct.pack(
        "<5h",
        1500,   # target_iq = 1.500 A
        1480,   # iq_ref = 1.480 A
        1490,   # filtered_iq = 1.490 A
        1510,   # raw_iq = 1.510 A
        12000,  # uq_final = 12.000 V
    )
    s1 = struct.pack(
        "<5h",
        -500,   # target_iq = -0.500 A
        -480,   # iq_ref = -0.480 A
        -490,   # filtered_iq = -0.490 A
        -510,   # raw_iq = -0.510 A
        11500,  # uq_final = 11.500 V
    )
    assert len(s0) == 10
    assert len(s1) == 10

    chunk_payload = chunk_header + s0 + s1
    assert len(chunk_payload) == 11 + 2 * 10

    fc_meta, fc_samples = parse_fastcap_chunk(chunk_payload)
    check("fc_chunk meta.op_echo", fc_meta.op_echo == 0x03)
    check("fc_chunk meta.capture_id", fc_meta.capture_id == 42)
    check("fc_chunk meta.source", fc_meta.source == 1)
    check("fc_chunk meta.total_count", fc_meta.total_count == 256)
    check("fc_chunk sample_count", len(fc_samples) == 2)

    # sample 0
    check("fc_chunk s0.index", fc_samples[0].index == 0)
    check("fc_chunk s0.capture_id", fc_samples[0].capture_id == 42)
    check("fc_chunk s0.source", fc_samples[0].source == 1)
    check("fc_chunk s0.target_iq", fc_samples[0].target_iq_a == 1.500)
    check("fc_chunk s0.iq_ref", fc_samples[0].iq_ref_a == 1.480)
    check("fc_chunk s0.filtered_iq", fc_samples[0].filtered_iq_a == 1.490)
    check("fc_chunk s0.raw_iq", fc_samples[0].raw_iq_a == 1.510)
    check("fc_chunk s0.uq_final", fc_samples[0].uq_final_v == 12.000)

    # sample 1
    check("fc_chunk s1.index", fc_samples[1].index == 1)
    check("fc_chunk s1.capture_id", fc_samples[1].capture_id == 42)
    check("fc_chunk s1.source", fc_samples[1].source == 1)
    check("fc_chunk s1.target_iq", fc_samples[1].target_iq_a == -0.500)
    check("fc_chunk s1.iq_ref", fc_samples[1].iq_ref_a == -0.480)
    check("fc_chunk s1.filtered_iq", fc_samples[1].filtered_iq_a == -0.490)
    check("fc_chunk s1.raw_iq", fc_samples[1].raw_iq_a == -0.510)
    check("fc_chunk s1.uq_final", fc_samples[1].uq_final_v == 11.500)

    # --- chunk with non-zero start_idx ---
    chunk_header2 = struct.pack("<B B I H H B", 0x03, 0, 42, 256, 10, 1)
    cp2 = chunk_header2 + s0
    _, samples2 = parse_fastcap_chunk(cp2)
    check("fc_chunk start_idx=10 index", samples2[0].index == 10)

    # --- chunk length mismatch (claim 5 samples, provide 1) ---
    bad_header = struct.pack("<B B I H H B", 0x03, 0, 42, 256, 0, 5)
    bad_chunk = bad_header + s0  # only 1 sample, header says 5
    try:
        parse_fastcap_chunk(bad_chunk)
        check("fc_chunk mismatch raises", False)
    except ValueError as e:
        check("fc_chunk mismatch raises", "mismatch" in str(e))

    # --- chunk too short (no sample_count byte) ---
    try:
        parse_fastcap_chunk(b"\x00" * 5)
        check("fc_chunk short raises", False)
    except ValueError as e:
        check("fc_chunk short raises", "too short" in str(e))

    # --- summary ---
    total = ok + fail
    print(f"\n{ok}/{total} passed", file=sys.stderr)
    if fail:
        print(f"{fail} FAILURES", file=sys.stderr)
        sys.exit(1)
    else:
        print("All good.", file=sys.stderr)
        sys.exit(0)
