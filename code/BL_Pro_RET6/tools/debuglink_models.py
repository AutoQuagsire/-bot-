"""
DebugLink data models — strongly-typed dataclasses for stream and fastcap.

Wire-format scale: deg (x100), radps (x1000), A (x1000 or /1000), V (x1000).
All float fields are after scaling has been applied by the parser.
"""

from dataclasses import dataclass, field


@dataclass
class LiveFrame:
    """One decoded STATUS_STREAM (0x90) telemetry frame."""

    tick_ms: int
    pitch_target_deg: float
    speed_p_term_deg: float
    speed_i_term_deg: float
    pitch_meas_deg: float
    pitch_rate_dps: float
    speed_target_radps: float
    speed_meas_radps: float
    attitude_p_iq_cmd_a: float
    attitude_d_iq_cmd_a: float
    iq_cmd_a: float
    iq_cmd_clamped_a: float
    speed_output_limit_deg: float
    attitude_output_limit_a: float
    iq_l_a: float
    iq_r_a: float
    uq_l_v: float
    uq_r_v: float
    bus_v: float
    fault_flags: int
    fault_labels: str
    host_rx_time_ms: int


@dataclass
class FastCapMeta:
    """Metadata for one fast-capture session (0x91 STATUS / header of READ_CHUNK).

    Wire layout (13 bytes, debug_link.c:319):
        op_echo(u8) source(u8) capture_id(u32) total_count(u16)
        armed(u8) done(u8) blocked(u8) capacity(u16)
    """

    op_echo: int
    capture_id: int  # uint32 on the wire
    source: int
    total_count: int
    armed: bool
    done: bool
    blocked: bool
    capacity: int


@dataclass
class FastCapSample:
    """One sample inside a fast-capture chunk."""

    capture_id: int
    source: int
    index: int
    target_iq_a: float
    iq_ref_a: float
    filtered_iq_a: float
    raw_iq_a: float
    uq_final_v: float
