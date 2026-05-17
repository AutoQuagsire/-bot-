#!/usr/bin/env python3
"""
Inspect dual-sided FastRing CSV exports and print compact statistics.

Supported columns:
    idx,target_iq_l,iq_ref_l,filtered_iq_l,raw_iq_l,uq_final_l,
    target_iq_r,iq_ref_r,filtered_iq_r,raw_iq_r,uq_final_r,
    bus_v,sample_idx,status_flags

Examples:
    python tools/fastring_csv_inspect.py path/to/fastring_dual.csv
    python tools/fastring_csv_inspect.py path/to/fastring_dual.csv --head 3 --tail 3
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

REQUIRED_COLUMNS = (
    "idx",
    "target_iq_l", "iq_ref_l", "filtered_iq_l", "raw_iq_l", "uq_final_l",
    "target_iq_r", "iq_ref_r", "filtered_iq_r", "raw_iq_r", "uq_final_r",
    "bus_v", "sample_idx", "status_flags",
)

LEFT_FIELDS = ("target_iq_l", "iq_ref_l", "filtered_iq_l", "raw_iq_l", "uq_final_l")
RIGHT_FIELDS = ("target_iq_r", "iq_ref_r", "filtered_iq_r", "raw_iq_r", "uq_final_r")

FIELD_LABELS = {
    "target_iq": "target_iq",
    "iq_ref": "iq_ref",
    "filtered_iq": "filtered_iq",
    "raw_iq": "raw_iq",
    "uq_final": "uq_final",
}


def load_rows(path: Path) -> list[dict]:
    with path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError("missing CSV header")
        missing = [col for col in REQUIRED_COLUMNS if col not in reader.fieldnames]
        if missing:
            raise ValueError(f"missing columns: {', '.join(missing)}")

        rows: list[dict] = []
        for row_idx, row in enumerate(reader, start=2):
            try:
                parsed = {
                    "idx": int(row["idx"]),
                    "target_iq_l": float(row["target_iq_l"]),
                    "iq_ref_l": float(row["iq_ref_l"]),
                    "filtered_iq_l": float(row["filtered_iq_l"]),
                    "raw_iq_l": float(row["raw_iq_l"]),
                    "uq_final_l": float(row["uq_final_l"]),
                    "target_iq_r": float(row["target_iq_r"]),
                    "iq_ref_r": float(row["iq_ref_r"]),
                    "filtered_iq_r": float(row["filtered_iq_r"]),
                    "raw_iq_r": float(row["raw_iq_r"]),
                    "uq_final_r": float(row["uq_final_r"]),
                    "bus_v": float(row["bus_v"]),
                    "sample_idx": int(row["sample_idx"]),
                    "status_flags": int(row["status_flags"]),
                }
            except (TypeError, ValueError) as e:
                raise ValueError(f"row {row_idx}: parse error: {e}") from e
            rows.append(parsed)
    return rows


def pearson_r(xs: list[float], ys: list[float]) -> float:
    n = len(xs)
    if n < 2:
        return float("nan")
    sum_x = sum(xs)
    sum_y = sum(ys)
    sum_xx = sum(x * x for x in xs)
    sum_yy = sum(y * y for y in ys)
    sum_xy = sum(x * y for x, y in zip(xs, ys))
    num = n * sum_xy - sum_x * sum_y
    den = math.sqrt((n * sum_xx - sum_x * sum_x) * (n * sum_yy - sum_y * sum_y))
    if den == 0.0:
        return float("nan")
    return num / den


def _col_stats(vals: list[float]) -> dict:
    n = len(vals)
    mean = sum(vals) / n
    rms = math.sqrt(sum(v * v for v in vals) / n)
    abs_peak = max(abs(v) for v in vals)
    return {
        "min": min(vals),
        "max": max(vals),
        "mean": mean,
        "rms": rms,
        "abs_peak": abs_peak,
    }


def summarize(rows: list[dict]) -> dict:
    if not rows:
        raise ValueError("no data rows")

    bus_vals = [r["bus_v"] for r in rows]
    summary: dict[str, object] = {
        "path": "",
        "count": len(rows),
        "idx_first": rows[0]["idx"],
        "idx_last": rows[-1]["idx"],
        "sample_idx_first": rows[0]["sample_idx"],
        "sample_idx_last": rows[-1]["sample_idx"],
        "bus_min": min(bus_vals),
        "bus_max": max(bus_vals),
        "bus_mean": sum(bus_vals) / len(bus_vals),
    }

    # Per-side stats
    for side, fields in [("L", LEFT_FIELDS), ("R", RIGHT_FIELDS)]:
        for f in fields:
            base = f[:-2]  # strip _l / _r
            vals = [r[f] for r in rows]
            stats = _col_stats(vals)
            for stat_key, stat_val in stats.items():
                summary[f"{base}_{stat_key}_{side}"] = stat_val

    # Diff stats: L - R
    diff_pairs = [
        ("filtered_iq", "filtered_iq_l", "filtered_iq_r"),
        ("raw_iq", "raw_iq_l", "raw_iq_r"),
        ("uq_final", "uq_final_l", "uq_final_r"),
    ]
    for label, l_key, r_key in diff_pairs:
        diffs = [r[l_key] - r[r_key] for r in rows]
        stats = _col_stats(diffs)
        for stat_key, stat_val in stats.items():
            summary[f"diff_{label}_{stat_key}"] = stat_val

    # Pearson on filtered_iq
    fl = [r["filtered_iq_l"] for r in rows]
    fr = [r["filtered_iq_r"] for r in rows]
    summary["pearson_filtered_iq"] = pearson_r(fl, fr)

    return summary


def print_summary(path: Path, summary: dict) -> None:
    print(f"[FILE] {path}")
    print(
        f"  samples={summary['count']} "
        f"idx={summary['idx_first']}..{summary['idx_last']} "
        f"sample_idx={summary['sample_idx_first']}..{summary['sample_idx_last']}"
    )
    print(
        f"  bus_v  "
        f"min={summary['bus_min']:.4f} "
        f"max={summary['bus_max']:.4f} "
        f"mean={summary['bus_mean']:.4f}"
    )

    for side in ("L", "R"):
        print(f"\n[{'LEFT' if side == 'L' else 'RIGHT'} SIDE]")
        for label, key in [("target_iq", "target_iq"), ("iq_ref", "iq_ref"),
                           ("filtered_iq", "filtered_iq"), ("raw_iq", "raw_iq"),
                           ("uq_final", "uq_final")]:
            print(
                f"  {label:12s} "
                f"min={summary[f'{key}_min_{side}']:+.6f} "
                f"max={summary[f'{key}_max_{side}']:+.6f} "
                f"mean={summary[f'{key}_mean_{side}']:+.6f} "
                f"rms={summary[f'{key}_rms_{side}']:+.6f} "
                f"abs_peak={summary[f'{key}_abs_peak_{side}']:+.6f}"
            )

    print(f"\n[DIFF  L - R]")
    for label in ("filtered_iq", "raw_iq", "uq_final"):
        print(
            f"  {label:12s} "
            f"min={summary[f'diff_{label}_min']:+.6f} "
            f"max={summary[f'diff_{label}_max']:+.6f} "
            f"mean={summary[f'diff_{label}_mean']:+.6f} "
            f"rms={summary[f'diff_{label}_rms']:+.6f} "
            f"abs_peak={summary[f'diff_{label}_abs_peak']:+.6f}"
        )

    print(f"\n[CONSISTENCY]")
    print(f"  pearson_r(filtered_iq_l, filtered_iq_r) = {summary['pearson_filtered_iq']:+.6f}")


def print_rows(label: str, rows: list[dict]) -> None:
    print(f"[{label}]")
    for row in rows:
        print(
            f"  idx={row['idx']:4d} "
            f"L(target={row['target_iq_l']:+.6f} ref={row['iq_ref_l']:+.6f} "
            f"filt={row['filtered_iq_l']:+.6f} raw={row['raw_iq_l']:+.6f} uq={row['uq_final_l']:+.6f}) "
            f"R(target={row['target_iq_r']:+.6f} ref={row['iq_ref_r']:+.6f} "
            f"filt={row['filtered_iq_r']:+.6f} raw={row['raw_iq_r']:+.6f} uq={row['uq_final_r']:+.6f}) "
            f"bus={row['bus_v']:.4f}"
        )


def main() -> int:
    parser = argparse.ArgumentParser(description="Inspect dual-sided FastRing CSV exports")
    parser.add_argument("csv", help="FastRing dual CSV file")
    parser.add_argument("--head", type=int, default=0, help="Print first N rows")
    parser.add_argument("--tail", type=int, default=0, help="Print last N rows")
    args = parser.parse_args()

    path = Path(args.csv)
    rows = load_rows(path)
    summary = summarize(rows)
    summary["path"] = str(path)
    print_summary(path, summary)

    if args.head > 0:
        print_rows(f"HEAD {path.name}", rows[: args.head])
    if args.tail > 0:
        print_rows(f"TAIL {path.name}", rows[-args.tail :])

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
