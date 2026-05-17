"""
DebugLink GUI scaffold (PySide6).

By default this runs against real DebugLink serial stream. Use ``--mock`` for
local UI-only rendering.
"""

from __future__ import annotations

import math
import random
import sys
import csv
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from debuglink_models import (
    FastRingMeta,
    FastRingSample,
    LiveFrame,
)
from debuglink_transport import DebugLinkTransport, TransportError

try:
    from PySide6.QtCore import QObject, QTimer, Signal
    from PySide6.QtWidgets import (
        QApplication,
        QCheckBox,
        QFormLayout,
        QGridLayout,
        QGroupBox,
        QHBoxLayout,
        QLabel,
        QMainWindow,
        QPushButton,
        QSpinBox,
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
class FastRingViewState:
    meta: FastRingMeta | None = None
    sample_count: int = 0
    dumping: bool = False
    busy: bool = False
    output_path: str | None = None


@dataclass
class ControlViewState:
    connected: bool = False
    streaming: bool = False
    connecting: bool = False
    mock_mode: bool = False
    port: str | None = None
    baud: int | None = None
    fw: str | None = None
    cap_flags: int | None = None
    last_message: str | None = None


class DebugLinkGateway(QObject):
    linkStateChanged = Signal(bool, object, object)  # connected, port, baud
    deviceInfoReady = Signal(dict)
    liveFrameReady = Signal(object)  # LiveFrame
    fastRingStatusReady = Signal(object)  # FastRingMeta
    fastRingChunkReady = Signal(object, list)  # FastRingMeta, list[FastRingSample]
    errorRaised = Signal(str)
    connectSucceeded = Signal(object, bool, object)  # transport, ping_ok, info
    connectFailed = Signal(str)
    fastRingFinished = Signal(str)

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

    def on_fastring_status(self, meta: FastRingMeta) -> None:
        self.fastRingStatusReady.emit(meta)

    def on_fastring_chunk(self, meta: FastRingMeta, samples: list[FastRingSample]) -> None:
        self.fastRingChunkReady.emit(meta, samples)

    def on_error(self, message: str) -> None:
        self.errorRaised.emit(message)

    def on_connect_succeeded(
        self, transport: object, ping_ok: bool, info: dict | None
    ) -> None:
        self.connectSucceeded.emit(transport, ping_ok, info)

    def on_connect_failed(self, message: str) -> None:
        self.connectFailed.emit(message)


class MainWindow(QMainWindow):
    def __init__(
        self,
        gateway: DebugLinkGateway,
        mock: bool = False,
        port: str = "COM33",
        baud: int = 921600,
        rate: int = 100,
    ) -> None:
        super().__init__()
        self.gateway = gateway
        self.live_state = LiveViewState()
        self.fastring_state = FastRingViewState()
        self.control_state = ControlViewState()

        self._transport: DebugLinkTransport | None = None
        self._closing = False
        self._repo_root = Path(__file__).resolve().parents[1]

        self.setWindowTitle("DebugLink GUI (Scaffold)")
        self.resize(1100, 760)
        self._build_ui()
        self._bind_signals()
        self.configure_session(port, baud, rate, mock)
        self._apply_ui_state()

    def _build_ui(self) -> None:
        tabs = QTabWidget()
        tabs.addTab(self._build_live_tab(), "Live")
        tabs.addTab(self._build_fastring_tab(), "FastRing")
        tabs.addTab(self._build_control_tab(), "Control")
        self.setCentralWidget(tabs)

    def _build_live_tab(self) -> QWidget:
        root = QWidget()
        layout = QVBoxLayout(root)

        session_box = QGroupBox("Live Session")
        session_form = QFormLayout(session_box)
        self.live_mode = QLabel("standby")
        self.live_stream = QLabel("stopped")
        self.live_rate = QLabel("-")
        session_form.addRow("mode", self.live_mode)
        session_form.addRow("stream", self.live_stream)
        session_form.addRow("rate_hz", self.live_rate)
        layout.addWidget(session_box)

        live_actions = QHBoxLayout()
        self.btn_connect = QPushButton("Connect")
        self.btn_disconnect = QPushButton("Disconnect")
        self.btn_stream_start = QPushButton("Start Stream")
        self.btn_stream_stop = QPushButton("Stop Stream")
        self.btn_pause_view = QPushButton("Pause View")
        for btn in (
            self.btn_connect,
            self.btn_disconnect,
            self.btn_stream_start,
            self.btn_stream_stop,
            self.btn_pause_view,
        ):
            live_actions.addWidget(btn)
        live_actions.addStretch(1)
        layout.addLayout(live_actions)

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

    def _build_fastring_tab(self) -> QWidget:
        root = QWidget()
        layout = QVBoxLayout(root)

        action_row = QHBoxLayout()
        self.btn_fastring_status = QPushButton("FastRing Status")
        self.btn_fastring_dump_left = QPushButton("Dump L")
        self.btn_fastring_dump_right = QPushButton("Dump R")
        self.btn_fastring_dump_both = QPushButton("Dump Both")
        self.btn_fastring_dump = QPushButton("Dump Dual")
        for btn in (
            self.btn_fastring_status,
            self.btn_fastring_dump_left,
            self.btn_fastring_dump_right,
            self.btn_fastring_dump_both,
            self.btn_fastring_dump,
        ):
            action_row.addWidget(btn)
        action_row.addStretch(1)
        layout.addLayout(action_row)

        form_box = QGroupBox("Ring Status")
        form = QFormLayout(form_box)
        self.fr_total = QLabel("-")
        self.fr_capacity = QLabel("-")
        self.fr_head = QLabel("-")
        self.fr_write_seq = QLabel("-")
        self.fr_samples = QLabel("0")
        form.addRow("total_count", self.fr_total)
        form.addRow("capacity", self.fr_capacity)
        form.addRow("head", self.fr_head)
        form.addRow("write_seq", self.fr_write_seq)
        form.addRow("samples_in_view", self.fr_samples)
        layout.addWidget(form_box)

        self.fr_log = QTextEdit()
        self.fr_log.setReadOnly(True)
        self.fr_log.setPlaceholderText("FastRing events and summary will appear here.")
        layout.addWidget(self.fr_log)
        return root

    def _build_control_tab(self) -> QWidget:
        root = QWidget()
        layout = QVBoxLayout(root)

        info_box = QGroupBox("Device")
        form = QFormLayout(info_box)
        self.ctrl_conn = QLabel("disconnected")
        self.ctrl_mode = QLabel("standby")
        self.ctrl_stream = QLabel("stopped")
        self.ctrl_port = QLabel("-")
        self.ctrl_baud = QLabel("-")
        self.ctrl_fw = QLabel("-")
        self.ctrl_caps = QLabel("-")
        form.addRow("link", self.ctrl_conn)
        form.addRow("mode", self.ctrl_mode)
        form.addRow("stream", self.ctrl_stream)
        form.addRow("port", self.ctrl_port)
        form.addRow("baud", self.ctrl_baud)
        form.addRow("fw", self.ctrl_fw)
        form.addRow("cap_flags", self.ctrl_caps)
        layout.addWidget(info_box)

        session_box = QGroupBox("Session Defaults")
        session_form = QFormLayout(session_box)
        self.ctrl_rate_spin = QSpinBox()
        self.ctrl_rate_spin.setRange(1, 500)
        self.ctrl_rate_spin.setValue(100)
        self.ctrl_mock_checkbox = QCheckBox("Mock mode")
        session_form.addRow("stream_rate_hz", self.ctrl_rate_spin)
        session_form.addRow("options", self.ctrl_mock_checkbox)
        layout.addWidget(session_box)

        btn_row = QHBoxLayout()
        self.btn_ping = QPushButton("Ping")
        self.btn_driver_on = QPushButton("Driver On")
        self.btn_driver_off = QPushButton("Driver Off")
        self.btn_balance_on = QPushButton("Balance On")
        self.btn_balance_off = QPushButton("Balance Off")
        for btn in (
            self.btn_ping,
            self.btn_driver_on,
            self.btn_driver_off,
            self.btn_balance_on,
            self.btn_balance_off,
        ):
            btn_row.addWidget(btn)
        layout.addLayout(btn_row)

        self.ctrl_log = QTextEdit()
        self.ctrl_log.setReadOnly(True)
        self.ctrl_log.setPlaceholderText("Control events and errors.")
        layout.addWidget(self.ctrl_log)
        return root

    def configure_session(self, port: str, baud: int, rate_hz: int, mock_mode: bool) -> None:
        self.control_state.port = port
        self.control_state.baud = baud
        self.control_state.mock_mode = mock_mode
        self.ctrl_port.setText(port)
        self.ctrl_baud.setText(str(baud))
        self.ctrl_rate_spin.setValue(rate_hz)
        self.ctrl_mock_checkbox.setChecked(mock_mode)
        self.live_rate.setText(str(rate_hz))
        self._apply_ui_state()

    def set_streaming_state(self, streaming: bool) -> None:
        self.control_state.streaming = streaming
        self._apply_ui_state()

    def _bind_signals(self) -> None:
        self.gateway.linkStateChanged.connect(self._on_link_state)
        self.gateway.deviceInfoReady.connect(self._on_device_info)
        self.gateway.liveFrameReady.connect(self._on_live_frame)
        self.gateway.fastRingStatusReady.connect(self._on_fastring_status)
        self.gateway.fastRingChunkReady.connect(self._on_fastring_chunk)
        self.gateway.errorRaised.connect(self._on_error)
        self.gateway.connectSucceeded.connect(self._on_connect_succeeded)
        self.gateway.connectFailed.connect(self._on_connect_failed)
        self.gateway.fastRingFinished.connect(self._on_fastring_finished)

        self.btn_pause_view.clicked.connect(self._toggle_live_pause)
        self.btn_connect.clicked.connect(self._on_connect_clicked)
        self.btn_disconnect.clicked.connect(self._on_disconnect_clicked)
        self.btn_stream_start.clicked.connect(self._on_start_stream_clicked)
        self.btn_stream_stop.clicked.connect(self._on_stop_stream_clicked)

        self.btn_ping.clicked.connect(self._on_ping_clicked)
        self.btn_driver_on.clicked.connect(
            lambda: self._append_control_log("Driver On action reserved")
        )
        self.btn_driver_off.clicked.connect(
            lambda: self._append_control_log("Driver Off action reserved")
        )
        self.btn_balance_on.clicked.connect(
            lambda: self._append_control_log("Balance On action reserved")
        )
        self.btn_balance_off.clicked.connect(
            lambda: self._append_control_log("Balance Off action reserved")
        )
        self.btn_fastring_status.clicked.connect(self._on_fastring_status_clicked)
        self.btn_fastring_dump_left.clicked.connect(
            lambda: self._start_fastring_side_dump(0, "left.csv")
        )
        self.btn_fastring_dump_right.clicked.connect(
            lambda: self._start_fastring_side_dump(1, "right.csv")
        )
        self.btn_fastring_dump_both.clicked.connect(self._start_fastring_split_dump)
        self.btn_fastring_dump.clicked.connect(self._on_fastring_dump_clicked)

    def _on_connect_clicked(self) -> None:
        if self._transport is not None or self.control_state.connecting:
            self._append_control_log("[WARN] connect already in progress or active")
            return

        port = self.control_state.port or "COM33"
        baud = self.control_state.baud or 921600

        self.control_state.connecting = True
        self._apply_ui_state()
        self._append_control_log(f"Connecting to {port} @ {baud}...")

        worker = threading.Thread(
            target=self._connect_worker,
            args=(port, baud),
            name="gui-connect",
            daemon=True,
        )
        worker.start()

    def _connect_worker(self, port: str, baud: int) -> None:
        transport = DebugLinkTransport()
        try:
            transport.connect(port, baud)
        except TransportError as e:
            self.gateway.on_connect_failed(f"connect failed: {e}")
            return

        ping_ok = transport.ping()
        info = None
        try:
            info = transport.get_info()
        except TransportError:
            info = None

        self.gateway.on_connect_succeeded(transport, ping_ok, info)

    def _on_connect_succeeded(
        self, transport: object, ping_ok: bool, info: dict | None
    ) -> None:
        if self._closing:
            try:
                transport.disconnect()  # type: ignore[union-attr]
            except Exception:
                pass
            return
        if not self.control_state.connecting:
            try:
                transport.disconnect()  # type: ignore[union-attr]
            except Exception:
                pass
            return

        self._transport = transport if isinstance(transport, DebugLinkTransport) else None
        self.control_state.connecting = False
        self.gateway.on_link_state(True, self.control_state.port, self.control_state.baud)
        self._append_control_log("Serial port opened.")
        self._append_control_log(f"Ping -> {'OK' if ping_ok else 'FAIL'}")
        if not ping_ok:
            self._append_control_log("[WARN] ping failed - device may not be responding")

        if info is not None:
            self.gateway.on_device_info(
                device_type=info["device_type"],
                proto_version=info["proto_version"],
                fw=f"{info['fw_major']}.{info['fw_minor']}.{info['fw_patch']}",
                cap_flags=info["cap_flags"],
                max_payload=info["max_payload"],
            )
            self._append_control_log(
                f"Device: type=0x{info['device_type']:02X} "
                f"fw={info['fw_major']}.{info['fw_minor']}.{info['fw_patch']} "
                f"caps=0x{info['cap_flags']:04X}"
            )
        else:
            self._append_control_log("[WARN] get_info failed")

        self._apply_ui_state()

    def _on_connect_failed(self, message: str) -> None:
        if self._closing:
            return
        self.control_state.connecting = False
        self._apply_ui_state()
        self._append_control_log(f"[ERROR] {message}")

    def _on_disconnect_clicked(self) -> None:
        self._append_control_log("Disconnecting...")
        self._stop_stream_internal()

        if self._transport is not None:
            try:
                self._transport.disconnect()
            except Exception as e:
                self._append_control_log(f"[WARN] disconnect error: {e}")
            self._transport = None

        mock_feeder = getattr(self, "mock_feeder", None)
        if mock_feeder is not None:
            try:
                mock_feeder.timer.stop()
            except Exception:
                pass

        self.gateway.on_link_state(False, self.control_state.port, self.control_state.baud)
        self._append_control_log("Disconnected.")

    def _on_start_stream_clicked(self) -> None:
        if self._transport is None:
            self._append_control_log("[WARN] not connected")
            return
        if self.control_state.streaming:
            self._append_control_log("[WARN] stream already running")
            return

        rate = self.ctrl_rate_spin.value()
        self._append_control_log(f"Starting stream @ {rate} Hz...")

        self._transport.set_stream_callback(self._on_stream_frame)
        ok = self._transport.stream_start(rate_hz=rate)
        if ok:
            self.control_state.streaming = True
            self._apply_ui_state()
            self._append_control_log("Stream started.")
        else:
            self._transport.set_stream_callback(None)
            self._append_control_log("[ERROR] stream start failed")

    def _on_stop_stream_clicked(self) -> None:
        self._append_control_log("Stopping stream...")
        self._stop_stream_internal()
        self._append_control_log("Stream stopped.")

    def _stop_stream_internal(self) -> None:
        if not self.control_state.streaming:
            return
        self.control_state.streaming = False
        self._apply_ui_state()

        if self._transport is not None:
            self._transport.set_stream_callback(None)
            try:
                self._transport.stream_stop()
            except Exception:
                pass

    def _on_ping_clicked(self) -> None:
        if self._transport is None:
            self._append_control_log("[WARN] not connected")
            return
        ok = self._transport.ping()
        self._append_control_log(f"Ping -> {'OK' if ok else 'FAIL'}")

    def _on_stream_frame(self, frame: LiveFrame) -> None:
        self.gateway.liveFrameReady.emit(frame)

    def _on_fastring_status_clicked(self) -> None:
        if self._transport is None:
            self._append_fastring_log("[WARN] not connected")
            return
        try:
            meta = self._transport.fastring_status()
        except TransportError as e:
            self._append_fastring_log(f"[ERROR] fastring_status: {e}")
            return
        self.gateway.on_fastring_status(meta)

    def _on_fastring_dump_clicked(self) -> None:
        if self._transport is None:
            self._append_fastring_log("[WARN] not connected")
            return
        if self.fastring_state.busy or self.fastring_state.dumping:
            self._append_fastring_log("[WARN] dump already in progress")
            return

        self.fastring_state.busy = True
        self.fastring_state.dumping = True
        self.fastring_state.sample_count = 0
        output_path = self._resolve_fastring_output_path("fastring_dual.csv")
        self.fastring_state.output_path = str(output_path)
        self._apply_ui_state()
        self._append_fastring_log(f"FastRing dump starting -> {output_path}")

        worker = threading.Thread(
            target=self._fastring_dump_worker,
            args=(output_path,),
            name="gui-fastring-dump",
            daemon=True,
        )
        worker.start()

    def _start_fastring_side_dump(self, target_source: int, filename: str) -> None:
        if self._transport is None:
            self._append_fastring_log("[WARN] not connected")
            return
        if self.fastring_state.dumping or self.fastring_state.busy:
            self._append_fastring_log("[WARN] dump already in progress")
            return

        self.fastring_state.busy = True
        self.fastring_state.dumping = True
        self.fastring_state.sample_count = 0
        output_path = self._resolve_fastring_output_path(filename)
        self.fastring_state.output_path = str(output_path)
        self._apply_ui_state()
        self._append_fastring_log(
            f"Dump {filename} starting (target source={target_source}) -> {output_path}"
        )

        worker = threading.Thread(
            target=self._fastring_side_dump_worker,
            args=(target_source, output_path),
            name="gui-fastring-side-dump",
            daemon=True,
        )
        worker.start()

    def _start_fastring_split_dump(self) -> None:
        if self._transport is None:
            self._append_fastring_log("[WARN] not connected")
            return
        if self.fastring_state.dumping or self.fastring_state.busy:
            self._append_fastring_log("[WARN] dump already in progress")
            return

        self.fastring_state.busy = True
        self.fastring_state.dumping = True
        self.fastring_state.sample_count = 0
        self.fastring_state.output_path = str(self._repo_root / "current_loop_data")
        self._apply_ui_state()
        self._append_fastring_log(
            f"Dump Both starting -> {self._resolve_fastring_output_path('left.csv')} , {self._resolve_fastring_output_path('right.csv')}"
        )

        worker = threading.Thread(
            target=self._fastring_split_dump_worker,
            name="gui-fastring-split-dump",
            daemon=True,
        )
        worker.start()

    def _fastring_side_dump_worker(self, target_source: int, output_path: Path) -> None:
        try:
            snapshot_meta, all_samples = self._snapshot_and_collect_fastring()
            self._write_fastring_side_csv(
                all_samples, target_source, snapshot_meta.write_seq, output_path
            )
            self.gateway.fastRingFinished.emit(f"Dump complete -> {output_path}")
        except TransportError as e:
            self.gateway.fastRingFinished.emit(f"[ERROR] dump transport: {e}")
        except OSError as e:
            self.gateway.fastRingFinished.emit(
                f"[ERROR] file write failed: {output_path} ({e})"
            )

    def _fastring_split_dump_worker(self) -> None:
        left_path = self._resolve_fastring_output_path("left.csv")
        right_path = self._resolve_fastring_output_path("right.csv")
        try:
            snapshot_meta, all_samples = self._snapshot_and_collect_fastring()
            self._write_fastring_side_csv(
                all_samples, 0, snapshot_meta.write_seq, left_path
            )
            self.gateway.fastRingFinished.emit(f"Dump L complete -> {left_path}")
            self.fastring_state.sample_count = 0
            self._write_fastring_side_csv(
                all_samples, 1, snapshot_meta.write_seq, right_path
            )
            self.gateway.fastRingFinished.emit(f"Dump R complete -> {right_path}")
            self.gateway.fastRingFinished.emit(
                f"Dump Both complete -> {left_path} , {right_path}"
            )
        except TransportError as e:
            self.gateway.fastRingFinished.emit(f"[ERROR] dump both transport: {e}")
        except OSError as e:
            self.gateway.fastRingFinished.emit(f"[ERROR] dump both file write failed: {e}")

    def _fastring_dump_worker(self, output_path: Path) -> None:
        try:
            _, all_samples = self._snapshot_and_collect_fastring()
            self._write_fastring_dual_csv(all_samples, output_path)

            self.gateway.fastRingFinished.emit(
                f"FastRing dump complete: {len(all_samples)} samples -> {output_path}"
            )
        except TransportError as e:
            self.gateway.fastRingFinished.emit(f"[ERROR] fastring dump transport: {e}")
        except OSError as e:
            self.gateway.fastRingFinished.emit(
                f"[ERROR] fastring file write failed: {output_path} ({e})"
            )

    def _snapshot_and_collect_fastring(self) -> tuple[FastRingMeta, list[FastRingSample]]:
        live_meta = self._transport.fastring_status()
        self.gateway.on_fastring_status(live_meta)
        if live_meta.total_count == 0:
            raise TransportError("FastRing is empty")

        snapshot_meta = self._transport.fastring_snapshot()
        self.gateway.on_fastring_status(snapshot_meta)
        self.gateway.fastRingFinished.emit(
            f"[INFO] Snapshot frozen: total_count={snapshot_meta.total_count} write_seq={snapshot_meta.write_seq}"
        )

        all_samples: list[FastRingSample] = []
        chunk_size = 8
        start_idx = 0
        while start_idx < snapshot_meta.total_count:
            remaining = snapshot_meta.total_count - start_idx
            req = min(chunk_size, remaining)
            chunk_meta, samples = self._transport.fastring_read_chunk(
                snapshot_meta.write_seq, start_idx, req
            )
            if chunk_meta.total_count != snapshot_meta.total_count:
                raise TransportError("fastring dump aborted: total_count changed during read")
            if chunk_meta.write_seq != snapshot_meta.write_seq:
                raise TransportError("fastring dump aborted: write_seq changed during read")
            if samples and samples[0].index != start_idx:
                raise TransportError("fastring dump aborted: start_idx mismatch during read")

            all_samples.extend(samples)
            self.gateway.on_fastring_chunk(chunk_meta, samples)
            start_idx += len(samples)
            if len(samples) == 0:
                break

        if len(all_samples) != snapshot_meta.total_count:
            raise TransportError(
                f"fastring dump incomplete: expected {snapshot_meta.total_count} samples, got {len(all_samples)}"
            )
        return snapshot_meta, all_samples

    def _write_fastring_dual_csv(
        self, all_samples: list[FastRingSample], output_path: Path
    ) -> None:
        with open(output_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    "idx",
                    "target_iq_l",
                    "iq_ref_l",
                    "filtered_iq_l",
                    "raw_iq_l",
                    "uq_final_l",
                    "target_iq_r",
                    "iq_ref_r",
                    "filtered_iq_r",
                    "raw_iq_r",
                    "uq_final_r",
                    "bus_v",
                    "sample_idx",
                    "status_flags",
                ]
            )
            for s in all_samples:
                writer.writerow(
                    [
                        s.index,
                        s.target_iq_l_a,
                        s.iq_ref_l_a,
                        s.filtered_iq_l_a,
                        s.raw_iq_l_a,
                        s.uq_final_l_v,
                        s.target_iq_r_a,
                        s.iq_ref_r_a,
                        s.filtered_iq_r_a,
                        s.raw_iq_r_a,
                        s.uq_final_r_v,
                        s.bus_v,
                        s.sample_idx,
                        s.status_flags,
                    ]
                )

    def _write_fastring_side_csv(
        self,
        all_samples: list[FastRingSample],
        target_source: int,
        snapshot_write_seq: int,
        output_path: Path,
    ) -> None:
        side_label = "R" if target_source == 1 else "L"
        with open(output_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    "idx",
                    "target_iq",
                    "iq_ref",
                    "filtered_iq",
                    "raw_iq",
                    "uq_final",
                    "source",
                    "capture_id",
                ]
            )
            for idx, s in enumerate(all_samples):
                if target_source == 1:
                    row = [
                        idx,
                        s.target_iq_r_a,
                        s.iq_ref_r_a,
                        s.filtered_iq_r_a,
                        s.raw_iq_r_a,
                        s.uq_final_r_v,
                        side_label,
                        snapshot_write_seq,
                    ]
                else:
                    row = [
                        idx,
                        s.target_iq_l_a,
                        s.iq_ref_l_a,
                        s.filtered_iq_l_a,
                        s.raw_iq_l_a,
                        s.uq_final_l_v,
                        side_label,
                        snapshot_write_seq,
                    ]
                writer.writerow(row)

    def _append_control_log(self, message: str) -> None:
        self.ctrl_log.append(message)

    def _append_fastring_log(self, message: str) -> None:
        self.fr_log.append(message)

    def _reset_live_view(self) -> None:
        self.live_tick.setText("-")
        self.live_bus.setText("-")
        self.live_fault.setText("-")
        for lbl in self.live_labels.values():
            lbl.setText("-")

    def _reset_fastring_view(self) -> None:
        self.fr_total.setText("-")
        self.fr_capacity.setText("-")
        self.fr_head.setText("-")
        self.fr_write_seq.setText("-")
        self.fr_samples.setText("0")
        self.fastring_state.meta = None
        self.fastring_state.sample_count = 0
        self.fastring_state.busy = False
        self.fastring_state.dumping = False
        self.fastring_state.output_path = None

    def _resolve_fastring_output_path(self, filename: str) -> Path:
        output_dir = self._repo_root / "current_loop_data"
        output_dir.mkdir(parents=True, exist_ok=True)
        return output_dir / filename

    def _toggle_live_pause(self) -> None:
        self.live_state.paused = not self.live_state.paused
        self.btn_pause_view.setText("Resume View" if self.live_state.paused else "Pause View")
        self._append_control_log(
            "Live view paused" if self.live_state.paused else "Live view resumed"
        )

    def _apply_ui_state(self) -> None:
        connected = self.control_state.connected
        streaming = self.control_state.streaming
        connecting = self.control_state.connecting
        fastring_busy = self.fastring_state.busy or self.fastring_state.dumping
        busy = connecting or fastring_busy

        self.ctrl_conn.setText(
            "connecting..." if connecting else ("connected" if connected else "disconnected")
        )
        self.ctrl_mode.setText("mock" if self.control_state.mock_mode else "real")
        self.ctrl_stream.setText("running" if streaming else "stopped")
        self.live_mode.setText("mock" if self.control_state.mock_mode else "real")
        self.live_stream.setText("running" if streaming else "stopped")
        self.live_rate.setText(str(self.ctrl_rate_spin.value()))

        self.btn_connect.setEnabled(not connected and not connecting)
        self.btn_disconnect.setEnabled(connected and not busy)
        self.btn_stream_start.setEnabled(connected and not streaming and not busy)
        self.btn_stream_stop.setEnabled(connected and streaming and not busy)
        self.btn_pause_view.setEnabled(connected or self.control_state.mock_mode)

        self.btn_ping.setEnabled(connected and not busy)
        self.btn_driver_on.setEnabled(connected and not busy)
        self.btn_driver_off.setEnabled(connected and not busy)
        self.btn_balance_on.setEnabled(connected and not busy)
        self.btn_balance_off.setEnabled(connected and not busy)

        self.btn_fastring_status.setEnabled(connected and not streaming and not busy)
        self.btn_fastring_dump_left.setEnabled(connected and not streaming and not busy)
        self.btn_fastring_dump_right.setEnabled(connected and not streaming and not busy)
        self.btn_fastring_dump_both.setEnabled(connected and not streaming and not busy)
        self.btn_fastring_dump.setEnabled(connected and not streaming and not busy)
        self.ctrl_rate_spin.setEnabled(not connected and not busy)
        self.ctrl_mock_checkbox.setEnabled(False)

    def _on_link_state(self, connected: bool, port: str | None, baud: int | None) -> None:
        self.control_state.connected = connected
        self.control_state.port = port
        self.control_state.baud = baud
        self.ctrl_port.setText(port or "-")
        self.ctrl_baud.setText(str(baud) if baud is not None else "-")
        if not connected:
            self.control_state.streaming = False
            self.live_state.last_frame = None
            self._reset_live_view()
            self._reset_fastring_view()
        self._apply_ui_state()

    def _on_device_info(self, info: dict) -> None:
        self.control_state.fw = info.get("fw")
        self.control_state.cap_flags = info.get("cap_flags")
        self.ctrl_fw.setText(str(info.get("fw", "-")))
        caps = info.get("cap_flags")
        self.ctrl_caps.setText(f"0x{caps:04X}" if isinstance(caps, int) else "-")
        self._apply_ui_state()

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

    def _on_fastring_status(self, meta: FastRingMeta) -> None:
        self.fastring_state.meta = meta
        self.fr_total.setText(str(meta.total_count))
        self.fr_capacity.setText(str(meta.capacity))
        self.fr_head.setText(str(meta.head))
        self.fr_write_seq.setText(str(meta.write_seq))
        self.fr_log.append(
            f"STATUS total={meta.total_count} capacity={meta.capacity} head={meta.head} write_seq={meta.write_seq}"
        )
        self._apply_ui_state()

    def _on_fastring_chunk(self, meta: FastRingMeta, samples: list[FastRingSample]) -> None:
        self.fastring_state.meta = meta
        self.fastring_state.sample_count += len(samples)
        self.fr_samples.setText(str(self.fastring_state.sample_count))
        self._apply_ui_state()

    def _on_error(self, message: str) -> None:
        self.control_state.last_message = message
        self._append_control_log(message)

    def _on_fastring_finished(self, message: str) -> None:
        self._append_fastring_log(message)
        self.fastring_state.busy = False
        self.fastring_state.dumping = False
        self._apply_ui_state()

    def closeEvent(self, event: Any) -> None:
        self._closing = True
        self._stop_stream_internal()
        if self._transport is not None:
            try:
                self._transport.disconnect()
            except Exception:
                pass
            self._transport = None
        mock_feeder = getattr(self, "mock_feeder", None)
        if mock_feeder is not None:
            try:
                mock_feeder.timer.stop()
            except Exception:
                pass
        super().closeEvent(event)


class MockFeeder(QObject):
    def __init__(self, gateway: DebugLinkGateway) -> None:
        super().__init__()
        self.gateway = gateway
        self.tick = 0
        self.phase = 0.0
        self.fastring_sent = False
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._step)
        self.timer.start(50)
        self.gateway.on_link_state(True, "COM33", 921600)
        self.gateway.on_device_info(0x01, 1, "0.1.0", 0x00E3, 240)

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

        if not self.fastring_sent and self.tick > 1500:
            meta = FastRingMeta(
                op_echo=0x03,
                total_count=128,
                capacity=512,
                head=128,
                write_seq=12,
            )
            self.gateway.on_fastring_status(meta)
            chunk = []
            for i in range(64):
                chunk.append(
                    FastRingSample(
                        index=i,
                        target_iq_l_a=0.8 * math.sin(i * 0.08),
                        iq_ref_l_a=0.8 * math.sin(i * 0.08 + 0.02),
                        filtered_iq_l_a=0.75 * math.sin(i * 0.08 + 0.03),
                        raw_iq_l_a=0.75 * math.sin(i * 0.08 + 0.03) + random.uniform(-0.05, 0.05),
                        uq_final_l_v=2.0 * math.sin(i * 0.08),
                        target_iq_r_a=-0.7 * math.sin(i * 0.08),
                        iq_ref_r_a=-0.7 * math.sin(i * 0.08 + 0.02),
                        filtered_iq_r_a=-0.65 * math.sin(i * 0.08 + 0.03),
                        raw_iq_r_a=-0.65 * math.sin(i * 0.08 + 0.03) + random.uniform(-0.05, 0.05),
                        uq_final_r_v=-1.8 * math.sin(i * 0.08),
                        bus_v=19.95 + random.uniform(-0.02, 0.02),
                        sample_idx=i,
                        status_flags=0x3F00,
                    )
                )
            self.gateway.on_fastring_chunk(meta, chunk)
            self.fastring_sent = True


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
    window = MainWindow(
        gateway,
        mock=args.mock,
        port=args.port,
        baud=args.baud,
        rate=args.rate,
    )
    window.show()

    if args.mock:
        window.mock_feeder = MockFeeder(gateway)  # type: ignore[attr-defined]
        window.set_streaming_state(True)
        window.ctrl_log.append("Running in MOCK mode.")
    else:
        window.ctrl_log.append(f"Ready. Click Connect to open {args.port} @ {args.baud}.")

    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
