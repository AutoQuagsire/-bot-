#!/usr/bin/env python3
"""
Inspect FastCap CSV exports and print compact statistics.

Supported columns:
    idx,target_iq,iq_ref,filtered_iq,raw_iq,uq_final,source,capture_id

Examples:
    python tools/fastcap_csv_inspect.py left.csv
    python tools/fastcap_csv_inspect.py left.csv right.csv
    python tools/fastcap_csv_inspect.py left.csv --head 3 --tail 3
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

REQUIRED_COLUMNS = (
    "idx",
    "target_iq",
    "iq_ref",
    "filtered_iq",
    "raw_iq",
    "uq_final",
    "source",
    "capture_id",
)

NUMERIC_COLUMNS = (
    "target_iq",
    "iq_ref",
    "filtered_iq",
    "raw_iq",
    "uq_final",
)


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
                    "target_iq": float(row["target_iq"]),
                    "iq_ref": float(row["iq_ref"]),
                    "filtered_iq": float(row["filtered_iq"]),
                    "raw_iq": float(row["raw_iq"]),
                    "uq_final": float(row["uq_final"]),
                    "source": row["source"],
                    "capture_id": int(row["capture_id"]),
                }
            except (TypeError, ValueError) as e:
                raise ValueError(f"row {row_idx}: parse error: {e}") from e
            rows.append(parsed)
    return rows


def summarize(rows: list[dict]) -> dict:
    if not rows:
        raise ValueError("no data rows")

    summary: dict[str, object] = {
        "count": len(rows),
        "idx_first": rows[0]["idx"],
        "idx_last": rows[-1]["idx"],
        "source": rows[0]["source"],
        "capture_id": rows[0]["capture_id"],
    }

    for key in NUMERIC_COLUMNS:
        vals = [float(r[key]) for r in rows]
        mean = sum(vals) / len(vals)
        rms = math.sqrt(sum(v * v for v in vals) / len(vals))
        abs_peak = max(abs(v) for v in vals)
        summary[f"{key}_min"] = min(vals)
        summary[f"{key}_max"] = max(vals)
        summary[f"{key}_mean"] = mean
        summary[f"{key}_rms"] = rms
        summary[f"{key}_abs_peak"] = abs_peak

    return summary


def print_summary(path: Path, summary: dict) -> None:
    print(f"[FILE] {path}")
    print(
        f"  samples={summary['count']} idx={summary['idx_first']}..{summary['idx_last']} "
        f"source={summary['source']} capture_id={summary['capture_id']}"
    )
    for key in NUMERIC_COLUMNS:
        print(
            f"  {key:12s} "
            f"min={summary[f'{key}_min']:+.6f} "
            f"max={summary[f'{key}_max']:+.6f} "
            f"mean={summary[f'{key}_mean']:+.6f} "
            f"rms={summary[f'{key}_rms']:+.6f} "
            f"abs_peak={summary[f'{key}_abs_peak']:+.6f}"
        )


def print_rows(label: str, rows: list[dict]) -> None:
    print(f"[{label}]")
    for row in rows:
        print(
            f"  idx={row['idx']:4d} "
            f"target_iq={row['target_iq']:+.6f} "
            f"iq_ref={row['iq_ref']:+.6f} "
            f"filtered_iq={row['filtered_iq']:+.6f} "
            f"raw_iq={row['raw_iq']:+.6f} "
            f"uq_final={row['uq_final']:+.6f}"
        )


def print_compare(path_a: Path, sum_a: dict, path_b: Path, sum_b: dict) -> None:
    print("[COMPARE]")
    print(f"  A={path_a.name} samples={sum_a['count']} source={sum_a['source']}")
    print(f"  B={path_b.name} samples={sum_b['count']} source={sum_b['source']}")
    for key in NUMERIC_COLUMNS:
        delta_mean = float(sum_a[f"{key}_mean"]) - float(sum_b[f"{key}_mean"])
        delta_peak = float(sum_a[f"{key}_abs_peak"]) - float(sum_b[f"{key}_abs_peak"])
        print(
            f"  {key:12s} "
            f"mean_delta={delta_mean:+.6f} "
            f"abs_peak_delta={delta_peak:+.6f}"
        )


def main() -> int:
    parser = argparse.ArgumentParser(description="Inspect FastCap CSV exports")
    parser.add_argument("csv", nargs="+", help="One or two CSV files")
    parser.add_argument("--head", type=int, default=0, help="Print first N rows")
    parser.add_argument("--tail", type=int, default=0, help="Print last N rows")
    args = parser.parse_args()

    if len(args.csv) not in (1, 2):
        parser.error("please provide one or two CSV files")

    paths = [Path(p) for p in args.csv]
    datasets: list[tuple[Path, list[dict], dict]] = []

    for path in paths:
        rows = load_rows(path)
        summary = summarize(rows)
        datasets.append((path, rows, summary))
        print_summary(path, summary)

        if args.head > 0:
            print_rows(f"HEAD {path.name}", rows[: args.head])
        if args.tail > 0:
            print_rows(f"TAIL {path.name}", rows[-args.tail :])

    if len(datasets) == 2:
        print_compare(datasets[0][0], datasets[0][2], datasets[1][0], datasets[1][2])

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
