"""
Helpers for rendering FastRing waveforms in the GUI.

This module is intentionally backend-agnostic: it only provides curve metadata,
sample buffering, and simple point decimation. The main GUI can later choose
any concrete drawing approach.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Iterable

from debuglink_models import FastRingSample


@dataclass(frozen=True)
class FastRingSeriesDef:
    key: str
    label: str
    color: str
    unit: str
    axis_group: str


FASTRING_SERIES = (
    FastRingSeriesDef("target_iq_l_a", "L target_iq", "#1f77b4", "A", "current_l"),
    FastRingSeriesDef("iq_ref_l_a", "L iq_ref", "#4f9fe6", "A", "current_l"),
    FastRingSeriesDef("filtered_iq_l_a", "L filtered_iq", "#0d3b66", "A", "current_l"),
    FastRingSeriesDef("raw_iq_l_a", "L raw_iq", "#7fb3ff", "A", "current_l"),
    FastRingSeriesDef("uq_final_l_v", "L uq_final", "#2ca02c", "V", "voltage_l"),
    FastRingSeriesDef("target_iq_r_a", "R target_iq", "#d62728", "A", "current_r"),
    FastRingSeriesDef("iq_ref_r_a", "R iq_ref", "#ff7f7f", "A", "current_r"),
    FastRingSeriesDef("filtered_iq_r_a", "R filtered_iq", "#8c1d18", "A", "current_r"),
    FastRingSeriesDef("raw_iq_r_a", "R raw_iq", "#ffb3b3", "A", "current_r"),
    FastRingSeriesDef("uq_final_r_v", "R uq_final", "#9467bd", "V", "voltage_r"),
    FastRingSeriesDef("bus_v", "bus_v", "#bcbd22", "V", "bus"),
)

FASTRING_SERIES_BY_KEY = {series.key: series for series in FASTRING_SERIES}
DEFAULT_FASTRING_PLOT_KEYS = tuple(series.key for series in FASTRING_SERIES)


def sample_x(sample: FastRingSample, use_sample_idx: bool = True) -> float:
    return float(sample.sample_idx if use_sample_idx else sample.index)


def sample_value(sample: FastRingSample, key: str) -> float:
    return float(getattr(sample, key))


def decimate_points(
    points: list[tuple[float, float]], max_points: int
) -> list[tuple[float, float]]:
    if max_points <= 0 or len(points) <= max_points:
        return list(points)

    stride = max(1, len(points) // max_points)
    reduced = points[::stride]
    if reduced[-1] != points[-1]:
        reduced.append(points[-1])
    return reduced


class FastRingPlotBuffer:
    """Keep the most recent FastRing samples for waveform rendering."""

    def __init__(self, capacity: int = 512) -> None:
        self._samples: deque[FastRingSample] = deque(maxlen=max(1, capacity))

    @property
    def capacity(self) -> int:
        return self._samples.maxlen or 0

    def clear(self) -> None:
        self._samples.clear()

    def set_capacity(self, capacity: int) -> None:
        capacity = max(1, capacity)
        if capacity == self.capacity:
            return
        self._samples = deque(self._samples, maxlen=capacity)

    def append(self, sample: FastRingSample) -> None:
        self._samples.append(sample)

    def extend(self, samples: Iterable[FastRingSample]) -> None:
        self._samples.extend(samples)

    def __len__(self) -> int:
        return len(self._samples)

    def samples(self) -> list[FastRingSample]:
        return list(self._samples)

    def last_sample(self) -> FastRingSample | None:
        return self._samples[-1] if self._samples else None

    def x_range(self, use_sample_idx: bool = True) -> tuple[float, float] | None:
        if not self._samples:
            return None
        items = self.samples()
        return sample_x(items[0], use_sample_idx), sample_x(items[-1], use_sample_idx)

    def build_series_points(
        self,
        key: str,
        *,
        use_sample_idx: bool = True,
        max_points: int | None = None,
    ) -> list[tuple[float, float]]:
        points = [
            (sample_x(sample, use_sample_idx), sample_value(sample, key))
            for sample in self._samples
        ]
        if max_points is not None:
            return decimate_points(points, max_points)
        return points

    def build_multi_series(
        self,
        keys: Iterable[str] = DEFAULT_FASTRING_PLOT_KEYS,
        *,
        use_sample_idx: bool = True,
        max_points: int | None = None,
    ) -> dict[str, list[tuple[float, float]]]:
        return {
            key: self.build_series_points(
                key, use_sample_idx=use_sample_idx, max_points=max_points
            )
            for key in keys
        }

    def value_range(self, keys: Iterable[str]) -> tuple[float, float] | None:
        values: list[float] = []
        for sample in self._samples:
            for key in keys:
                values.append(sample_value(sample, key))
        if not values:
            return None
        return min(values), max(values)
