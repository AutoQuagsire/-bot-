"""
DebugLink GUI scaffold (PySide6).

By default this runs against real DebugLink serial stream. Use ``--mock`` for
local UI-only rendering.
"""

from __future__ import annotations

import math
import random
import struct
import sys
import time
from dataclasses import dataclass

from debuglink_models import FastCapMeta, FastCapSample, LiveFrame
from debuglink_parser import parse_status_stream

from debuglink_cli import (
    FrameReader,
    MSG_ACK,
    MSG_DEVICE_INFO_RSP,
    MSG_GET_DEVICE_INFO_REQ,
    MSG_NACK,
    MSG_PING_REQ,
    MSG_STATUS_STREAM,
    MSG_STREAM_CONTROL_REQ,
    build_frame,
    open_serial_port,
    recv_frame,
    send_frame_and_wait,
)

try:
    from PySide6.QtCore import QObject, QTimer, Signal
    from PySide6.QtWidgets import (
        QApplication,
        QFormLayout,
        QGridLayout,
        QGroupBox,
        QHBoxLayout,
        QLabel,
        QMainWindow,
        QPushButton,
        QTabWidget,
        QTextEdit,
        QVBoxLayout,
        QWidget,
    )
except ImportError as exc:
    print("PySide6 is required. Install with: pip install PySide6")
    print(f"Import error: {exc}")
    sys.exit(1)


@dataclass
class LiveViewState:
    last_frame: LiveFrame | None = None
    paused: bool = False
    history_limit: int = 2000


@dataclass
class FastCapViewState:
    meta: FastCapMeta | None = None
    sample_count: int = 0
    dumping: bool = False
    output_path: str | None = None


@dataclass
class ControlViewState:
    connected: bool = False
    port: str | None = None
    baud: int | None = None
    fw: str | None = None
    cap_flags: int | None = None
    last_message: str | None = None


class DebugLinkGateway(QObject):
    linkStateChanged = Signal(bool, object, object)  # connected, port, baud
    deviceInfoReady = Signal(dict)
    liveFrameReady = Signal(object)  # LiveFrame
    fastCapStatusReady = Signal(object)  # FastCapMeta
    fastCapChunkReady = Signal(object, list)  # FastCapMeta, list[FastCapSample]
    errorRaised = Signal(str)

    def on_link_state(self, connected: bool, port: str | None, baud: int | None) -> None:
        self.linkStateChanged.emit(connected, port, baud)

    def on_device_info(
        self,
        device_type: int,
        proto_version: int,
        fw: str,
        cap_flags: int,
        max_payload: int,
    ) -> None:
        self.deviceInfoReady.emit(
            {
                "device_type": device_type,
                "proto_version": proto_version,
                "fw": fw,
                "cap_flags": cap_flags,
                "max_payload": max_payload,
            }
        )

    def on_live_frame(self, frame: LiveFrame) -> None:
        self.liveFrameReady.emit(frame)

    def on_fastcap_status(self, meta: FastCapMeta) -> None:
        self.fastCapStatusReady.emit(meta)

    def on_fastcap_chunk(self, meta: FastCapMeta, samples: list[FastCapSample]) -> None:
        self.fastCapChunkReady.emit(meta, samples)

    def on_error(self, message: str) -> None:
        self.errorRaised.emit(message)


class RealLinkFeeder(QObject):
    def __init__(self, gateway: DebugLinkGateway, port: str, baud: int, rate_hz: int) -> None:
        super().__init__()
        self.gateway = gateway
        self.port = port
        self.baud = baud
        self.rate_hz = rate_hz
        self.reader = FrameReader()
        self.ser = open_serial_port(port, baud, settle_ms=80, open_retries=3)
        self.running = False
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._poll)

        self.gateway.on_link_state(True, port, baud)
        self._ping_once()
        self._read_info_once()
        self._start_stream()

    def _ping_once(self) -> None:
        payload = struct.pack("<I", int(time.time() * 1000) & 0xFFFFFFFF)
        frame = build_frame(MSG_PING_REQ, 1, payload)
        result = send_frame_and_wait(
            self.ser,
            self.reader,
            frame,
            "GUI_PING_REQ",
            1.0,
            2,
            40,
            accept_fn=lambda r: r[0] in (MSG_ACK, MSG_NACK),
        )
        if result is None or result[0] != MSG_ACK:
            raise RuntimeError("GUI ping failed")

    def _read_info_once(self) -> None:
        frame = build_frame(MSG_GET_DEVICE_INFO_REQ, 2, b"")
        result = send_frame_and_wait(
            self.ser,
            self.reader,
            frame,
            "GUI_GET_DEVICE_INFO_REQ",
            1.0,
            2,
            40,
            accept_fn=lambda r: r[0] in (MSG_DEVICE_INFO_RSP, MSG_NACK),
        )
        if result is None:
            raise RuntimeError("GUI info request timeout")
        msg_type, _, payload = result
        if msg_type != MSG_DEVICE_INFO_RSP or len(payload) != 9:
            raise RuntimeError("GUI info response invalid")
        self.gateway.on_device_info(
            device_type=payload[0],
            proto_version=payload[1],
            fw=f"{payload[2]}.{payload[3]}.{payload[4]}",
            cap_flags=struct.unpack_from("<H", payload, 5)[0],
            max_payload=struct.unpack_from("<H", payload, 7)[0],
        )

    def _start_stream(self) -> None:
        payload = struct.pack("<BBHI", 1, 0, self.rate_hz, 0xFFFFFFFF)
        frame = build_frame(MSG_STREAM_CONTROL_REQ, 3, payload)
        result = send_frame_and_wait(
            self.ser,
            self.reader,
            frame,
            "GUI_STREAM_CONTROL_REQ start",
            1.5,
            2,
            40,
            accept_fn=lambda r: r[0] in (MSG_ACK, MSG_NACK),
        )
        if result is None or result[0] != MSG_ACK:
            raise RuntimeError("GUI stream start rejected")
        self.running = True
        self.timer.start(5)

    def _stop_stream(self) -> None:
        payload = struct.pack("<BBHI", 0, 0, self.rate_hz, 0)
        frame = build_frame(MSG_STREAM_CONTROL_REQ, 4, payload)
        send_frame_and_wait(
            self.ser,
            self.reader,
            frame,
            "GUI_STREAM_CONTROL_REQ stop",
            0.8,
            2,
            40,
            accept_fn=lambda r: r[0] in (MSG_ACK, MSG_NACK),
        )

    def _poll(self) -> None:
        if not self.running:
            return
        try:
            # Drain all currently buffered bytes to minimize UI-lag.
            while True:
                result = recv_frame(self.ser, self.reader, timeout=0.001)
                if result is None:
                    break
                msg_type, _, payload = result
                if msg_type != MSG_STATUS_STREAM:
                    continue
                try:
                    frame = parse_status_stream(payload, int(time.time() * 1000))
                except ValueError as e:
                    self.gateway.on_error(f"stream parse error: {e}")
                    continue
                self.gateway.on_live_frame(frame)
        except Exception as e:
            self.gateway.on_error(f"stream polling error: {e}")
            self.stop()

    def stop(self) -> None:
        self.running = False
        try:
            self.timer.stop()
        except Exception:
            pass
        try:
            self._stop_stream()
        except Exception:
            pass
        try:
            self.ser.close()
        except Exception:
            pass
        self.gateway.on_link_state(False, self.port, self.baud)


class MainWindow(QMainWindow):
    def __init__(self, gateway: DebugLinkGateway) -> None:
        super().__init__()
        self.gateway = gateway
        self.live_state = LiveViewState()
        self.fastcap_state = FastCapViewState()
        self.control_state = ControlViewState()

        self.setWindowTitle("DebugLink GUI (Scaffold)")
        self.resize(1100, 760)
        self._build_ui()
        self._bind_signals()

    def _build_ui(self) -> None:
        tabs = QTabWidget()
        tabs.addTab(self._build_live_tab(), "Live")
        tabs.addTab(self._build_fastcap_tab(), "FastCap")
        tabs.addTab(self._build_control_tab(), "Control")
        self.setCentralWidget(tabs)

    def _build_live_tab(self) -> QWidget:
        root = QWidget()
        layout = QVBoxLayout(root)

        summary = QGroupBox("Summary")
        sform = QFormLayout(summary)
        self.live_tick = QLabel("-")
        self.live_bus = QLabel("-")
        self.live_fault = QLabel("-")
        sform.addRow("tick_ms", self.live_tick)
        sform.addRow("bus_v", self.live_bus)
        sform.addRow("fault", self.live_fault)
        layout.addWidget(summary)

        grid = QGridLayout()
        self.live_labels: dict[str, QLabel] = {}
        fields = [
            "pitch_target_deg",
            "pitch_meas_deg",
            "pitch_rate_dps",
            "speed_target_radps",
            "speed_meas_radps",
            "speed_p_term_deg",
            "speed_i_term_deg",
            "iq_cmd_a",
            "iq_cmd_clamped_a",
            "attitude_p_iq_cmd_a",
            "attitude_d_iq_cmd_a",
            "attitude_output_limit_a",
            "iq_l_a",
            "iq_r_a",
            "uq_l_v",
            "uq_r_v",
        ]
        for idx, name in enumerate(fields):
            lbl_name = QLabel(name)
            lbl_val = QLabel("-")
            self.live_labels[name] = lbl_val
            row = idx // 2
            col = (idx % 2) * 2
            grid.addWidget(lbl_name, row, col)
            grid.addWidget(lbl_val, row, col + 1)
        box = QGroupBox("Live Fields")
        box.setLayout(grid)
        layout.addWidget(box)
        layout.addStretch(1)
        return root

    def _build_fastcap_tab(self) -> QWidget:
        root = QWidget()
        layout = QVBoxLayout(root)

        form_box = QGroupBox("Capture Status")
        form = QFormLayout(form_box)
        self.fc_source = QLabel("-")
        self.fc_capture_id = QLabel("-")
        self.fc_total = QLabel("-")
        self.fc_armed = QLabel("-")
        self.fc_done = QLabel("-")
        self.fc_blocked = QLabel("-")
        self.fc_capacity = QLabel("-")
        self.fc_samples = QLabel("0")
        form.addRow("source", self.fc_source)
        form.addRow("capture_id", self.fc_capture_id)
        form.addRow("total_count", self.fc_total)
        form.addRow("armed", self.fc_armed)
        form.addRow("done", self.fc_done)
        form.addRow("blocked", self.fc_blocked)
        form.addRow("capacity", self.fc_capacity)
        form.addRow("samples_in_view", self.fc_samples)
        layout.addWidget(form_box)

        self.fc_log = QTextEdit()
        self.fc_log.setReadOnly(True)
        self.fc_log.setPlaceholderText("FastCap events and summary will appear here.")
        layout.addWidget(self.fc_log)
        return root

    def _build_control_tab(self) -> QWidget:
        root = QWidget()
        layout = QVBoxLayout(root)

        info_box = QGroupBox("Device")
        form = QFormLayout(info_box)
        self.ctrl_conn = QLabel("disconnected")
        self.ctrl_port = QLabel("-")
        self.ctrl_baud = QLabel("-")
        self.ctrl_fw = QLabel("-")
        self.ctrl_caps = QLabel("-")
        form.addRow("link", self.ctrl_conn)
        form.addRow("port", self.ctrl_port)
        form.addRow("baud", self.ctrl_baud)
        form.addRow("fw", self.ctrl_fw)
        form.addRow("cap_flags", self.ctrl_caps)
        layout.addWidget(info_box)

        btn_row = QHBoxLayout()
        for text in ("Ping", "Driver On", "Driver Off", "Balance On", "Balance Off"):
            btn = QPushButton(text)
            btn.setEnabled(False)
            btn_row.addWidget(btn)
        layout.addLayout(btn_row)

        self.ctrl_log = QTextEdit()
        self.ctrl_log.setReadOnly(True)
        self.ctrl_log.setPlaceholderText("Control events and errors.")
        layout.addWidget(self.ctrl_log)
        return root

    def _bind_signals(self) -> None:
        self.gateway.linkStateChanged.connect(self._on_link_state)
        self.gateway.deviceInfoReady.connect(self._on_device_info)
        self.gateway.liveFrameReady.connect(self._on_live_frame)
        self.gateway.fastCapStatusReady.connect(self._on_fastcap_status)
        self.gateway.fastCapChunkReady.connect(self._on_fastcap_chunk)
        self.gateway.errorRaised.connect(self._on_error)

    def _on_link_state(self, connected: bool, port: str | None, baud: int | None) -> None:
        self.control_state.connected = connected
        self.control_state.port = port
        self.control_state.baud = baud
        self.ctrl_conn.setText("connected" if connected else "disconnected")
        self.ctrl_port.setText(port or "-")
        self.ctrl_baud.setText(str(baud) if baud is not None else "-")

    def _on_device_info(self, info: dict) -> None:
        self.control_state.fw = info.get("fw")
        self.control_state.cap_flags = info.get("cap_flags")
        self.ctrl_fw.setText(str(info.get("fw", "-")))
        caps = info.get("cap_flags")
        self.ctrl_caps.setText(f"0x{caps:04X}" if isinstance(caps, int) else "-")

    def _on_live_frame(self, frame: LiveFrame) -> None:
        if self.live_state.paused:
            return
        self.live_state.last_frame = frame
        self.live_tick.setText(str(frame.tick_ms))
        self.live_bus.setText(f"{frame.bus_v:.2f} V")
        self.live_fault.setText(f"0x{frame.fault_flags:04X} [{frame.fault_labels}]")
        for key, lbl in self.live_labels.items():
            val = getattr(frame, key)
            if isinstance(val, float):
                lbl.setText(f"{val:+.3f}")
            else:
                lbl.setText(str(val))

    def _on_fastcap_status(self, meta: FastCapMeta) -> None:
        self.fastcap_state.meta = meta
        self.fc_source.setText("R" if meta.source == 1 else "L")
        self.fc_capture_id.setText(str(meta.capture_id))
        self.fc_total.setText(str(meta.total_count))
        self.fc_armed.setText(str(int(meta.armed)))
        self.fc_done.setText(str(int(meta.done)))
        self.fc_blocked.setText(str(int(meta.blocked)))
        self.fc_capacity.setText(str(meta.capacity))
        self.fc_log.append(
            f"STATUS id={meta.capture_id} src={'R' if meta.source == 1 else 'L'} "
            f"count={meta.total_count} armed={int(meta.armed)} done={int(meta.done)}"
        )

    def _on_fastcap_chunk(self, meta: FastCapMeta, samples: list[FastCapSample]) -> None:
        self.fastcap_state.meta = meta
        self.fastcap_state.sample_count += len(samples)
        self.fc_samples.setText(str(self.fastcap_state.sample_count))
        self.fc_log.append(
            f"CHUNK id={meta.capture_id} src={'R' if meta.source == 1 else 'L'} +{len(samples)} samples"
        )

    def _on_error(self, message: str) -> None:
        self.control_state.last_message = message
        self.ctrl_log.append(message)

    def closeEvent(self, event):  # type: ignore[override]
        real_feeder = getattr(self, "real_feeder", None)
        if real_feeder is not None:
            real_feeder.stop()
        super().closeEvent(event)


class MockFeeder(QObject):
    def __init__(self, gateway: DebugLinkGateway) -> None:
        super().__init__()
        self.gateway = gateway
        self.tick = 0
        self.phase = 0.0
        self.fastcap_sent = False
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._step)
        self.timer.start(50)
        self.gateway.on_link_state(True, "COM33", 921600)
        self.gateway.on_device_info(0x01, 1, "0.1.0", 0x0063, 240)

    def _step(self) -> None:
        self.tick += 50
        self.phase += 0.08
        frame = LiveFrame(
            tick_ms=self.tick,
            pitch_target_deg=0.5 * math.sin(self.phase),
            speed_p_term_deg=0.4 * math.sin(self.phase * 1.1),
            speed_i_term_deg=0.2 * math.sin(self.phase * 0.4),
            pitch_meas_deg=0.8 * math.sin(self.phase + 0.2),
            pitch_rate_dps=2.5 * math.cos(self.phase),
            speed_target_radps=0.0,
            speed_meas_radps=0.4 * math.sin(self.phase * 0.6),
            attitude_p_iq_cmd_a=0.2 * math.sin(self.phase * 1.3),
            attitude_d_iq_cmd_a=0.1 * math.cos(self.phase * 1.4),
            iq_cmd_a=0.25 * math.sin(self.phase * 1.2),
            iq_cmd_clamped_a=max(-0.2, min(0.2, 0.25 * math.sin(self.phase * 1.2))),
            speed_output_limit_deg=4.58,
            attitude_output_limit_a=1.2,
            iq_l_a=0.1 * math.sin(self.phase * 1.8) + random.uniform(-0.02, 0.02),
            iq_r_a=0.1 * math.cos(self.phase * 1.6) + random.uniform(-0.02, 0.02),
            uq_l_v=1.5 * math.sin(self.phase * 1.2),
            uq_r_v=1.5 * math.cos(self.phase * 1.1),
            bus_v=19.95 + random.uniform(-0.02, 0.02),
            fault_flags=0x3F00,
            fault_labels="stack,it,bus,iloop,sloop,cloop",
            host_rx_time_ms=int(time.time() * 1000),
        )
        self.gateway.on_live_frame(frame)

        if not self.fastcap_sent and self.tick > 1500:
            meta = FastCapMeta(
                op_echo=0x03,
                capture_id=12,
                source=0,
                total_count=512,
                armed=False,
                done=True,
                blocked=False,
                capacity=512,
            )
            self.gateway.on_fastcap_status(meta)
            chunk = []
            for i in range(64):
                idx = i
                chunk.append(
                    FastCapSample(
                        capture_id=12,
                        source=0,
                        index=idx,
                        target_iq_a=0.8 * math.sin(i * 0.08),
                        iq_ref_a=0.8 * math.sin(i * 0.08 + 0.02),
                        filtered_iq_a=0.75 * math.sin(i * 0.08 + 0.03),
                        raw_iq_a=0.75 * math.sin(i * 0.08 + 0.03) + random.uniform(-0.05, 0.05),
                        uq_final_v=2.0 * math.sin(i * 0.08),
                    )
                )
            self.gateway.on_fastcap_chunk(meta, chunk)
            self.fastcap_sent = True


def main() -> int:
    import argparse

    parser = argparse.ArgumentParser(description="DebugLink GUI")
    parser.add_argument("--port", default="COM33", help="serial port (default: COM33)")
    parser.add_argument("--baud", type=int, default=921600, help="baudrate (default: 921600)")
    parser.add_argument("--rate", type=int, default=100, help="stream rate Hz (default: 100)")
    parser.add_argument("--mock", action="store_true", help="use mock data instead of real serial")
    args = parser.parse_args()

    app = QApplication(sys.argv)
    gateway = DebugLinkGateway()
    window = MainWindow(gateway)
    window.show()
    if args.mock:
        # Keep feeder alive for the full app lifetime; otherwise Qt may stop timer updates.
        window.mock_feeder = MockFeeder(gateway)  # type: ignore[attr-defined]
        window.ctrl_log.append("Running in MOCK mode.")
    else:
        try:
            window.real_feeder = RealLinkFeeder(gateway, args.port, args.baud, args.rate)  # type: ignore[attr-defined]
            window.ctrl_log.append(
                f"Running in REAL mode: {args.port} {args.baud}, stream={args.rate}Hz"
            )
        except Exception as e:
            gateway.on_error(f"real mode init failed: {e}")
            gateway.on_link_state(False, args.port, args.baud)
            window.ctrl_log.append("Tip: run with --mock to test UI without hardware.")
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
