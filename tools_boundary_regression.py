#!/usr/bin/env python3
"""Regression check for boundary detection on 2/3/4 camera definitions.

This script validates the currently expected behavior on the San Jose demo map:
- 2-camera: boundary_count=5
- 3-camera: boundary_count=6 (camera_3.SB unavailable)
- 4-camera: boundary_count=8
"""

from __future__ import annotations

import argparse
import json
import subprocess
import tempfile
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parent
PIPELINE = PROJECT_ROOT / "tools_vss_boundary_pipeline.py"
NET = PROJECT_ROOT / "san_jose_downtown_gtc/osm.net.xml"
TLS = PROJECT_ROOT / "san_jose_downtown_gtc/osm.tls.xml"


BASE_COUNTS = {
    "EB": {"left": 80, "through": 220, "right": 100},
    "WB": {"left": 80, "through": 220, "right": 100},
    "NB": {"left": 40, "through": 110, "right": 50},
    "SB": {"left": 40, "through": 110, "right": 50},
}


def _build_input(intersections: list[tuple[str, float, float]]) -> dict:
    return {
        "corridor_id": "santa_clara_st",
        "observation_interval": "1h",
        "timestamp_start": "2026-03-15T08:00:00",
        "intersections": [
            {
                "intersection_id": iid,
                "lat": lat,
                "lon": lon,
                "counts": BASE_COUNTS,
            }
            for iid, lat, lon in intersections
        ],
    }


CASES = {
    "2cam": {
        "intersections": [
            ("market_santa_clara", 37.335480, -121.892050),
            ("1st_santa_clara", 37.336169, -121.890590),
        ],
        "expected_count": 5,
        "expected_kept": {
            "market_santa_clara": {"EB", "NB", "SB"},
            "1st_santa_clara": {"WB", "NB"},
        },
    },
    "3cam": {
        "intersections": [
            ("market_santa_clara", 37.335480, -121.892050),
            ("1st_santa_clara", 37.336169, -121.890590),
            ("camera_3", 37.335075, -121.892874),
        ],
        "expected_count": 6,
        "expected_kept": {
            "market_santa_clara": {"NB", "SB"},
            "1st_santa_clara": {"WB", "NB"},
            "camera_3": {"EB", "NB"},
        },
    },
    "4cam": {
        "intersections": [
            ("market_santa_clara", 37.335480, -121.892050),
            ("1st_santa_clara", 37.336169, -121.890590),
            ("camera_3", 37.335075, -121.892874),
            ("camera_4", 37.334569, -121.891404),
        ],
        "expected_count": 8,
        "expected_kept": {
            "market_santa_clara": {"NB"},
            "1st_santa_clara": {"WB", "NB"},
            "camera_3": {"EB", "NB"},
            "camera_4": {"EB", "WB", "SB"},
        },
    },
}


def _run_case(case_name: str, payload: dict, expected_count: int, expected_kept: dict[str, set[str]], keep_temp: bool) -> tuple[bool, str]:
    tmp_ctx = tempfile.TemporaryDirectory(prefix=f"boundary_{case_name}_")
    tmp_dir = Path(tmp_ctx.name)
    if keep_temp:
        tmp_ctx.cleanup = lambda: None  # type: ignore[attr-defined]

    input_json = tmp_dir / "input.json"
    input_json.write_text(json.dumps(payload), encoding="utf-8")

    boundary_json = tmp_dir / "boundary.json"
    route_out = tmp_dir / "routes.rou.xml"
    patterns_out = tmp_dir / "patterns.json"
    tripinfo_out = tmp_dir / "tripinfo.xml"
    fcd_out = tmp_dir / "fcd.xml"
    stat_out = tmp_dir / "stat.xml"
    report_out = tmp_dir / "report.json"

    cmd = [
        "python",
        str(PIPELINE),
        "--input-json",
        str(input_json),
        "--net-file",
        str(NET),
        "--tls-file",
        str(TLS),
        "--boundary-json-out",
        str(boundary_json),
        "--route-out",
        str(route_out),
        "--patterns-out",
        str(patterns_out),
        "--tripinfo-out",
        str(tripinfo_out),
        "--fcd-out",
        str(fcd_out),
        "--statistic-out",
        str(stat_out),
        "--report-out",
        str(report_out),
        "--sim-end",
        "600",
        "--max-match-distance-m",
        "60",
        "--max-corridor-hops",
        "8",
        "--internal-axis-thresh-deg",
        "45",
        "--boundary-mode",
        "junction_expand",
        "--exit-hops",
        "3",
        "--flow-scale",
        "1.0",
        "--min-route-edges",
        "5",
        "--spawn-upstream-hops",
        "2",
        "--stop-threshold-s",
        "120",
    ]
    run = subprocess.run(cmd, capture_output=True, text=True)
    if run.returncode != 0:
        return False, f"{case_name}: pipeline failed\n{run.stdout}\n{run.stderr}"

    boundary = json.loads(boundary_json.read_text(encoding="utf-8"))
    report = json.loads(report_out.read_text(encoding="utf-8"))
    got_kept = {it["intersection_id"]: set(it["counts"].keys()) for it in boundary["intersections"] if it["counts"]}
    got_count = sum(len(v) for v in got_kept.values())
    long_stop = report["vehicle_motion"]["long_stop_vehicle_count"]

    if got_count != expected_count:
        return (
            False,
            f"{case_name}: boundary_count expected {expected_count}, got {got_count}; kept={got_kept}; temp={tmp_dir}",
        )
    if got_kept != expected_kept:
        return (
            False,
            f"{case_name}: kept mismatch expected={expected_kept}, got={got_kept}; temp={tmp_dir}",
        )
    if long_stop != 0:
        return False, f"{case_name}: expected long_stop=0, got {long_stop}; temp={tmp_dir}"

    return True, f"{case_name}: PASS boundary={got_count}, kept={got_kept}, long_stop={long_stop}"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--keep-temp", action="store_true", help="Keep temporary output files for inspection.")
    args = ap.parse_args()

    all_ok = True
    for case_name in ("2cam", "3cam", "4cam"):
        case = CASES[case_name]
        ok, msg = _run_case(
            case_name=case_name,
            payload=_build_input(case["intersections"]),
            expected_count=case["expected_count"],
            expected_kept=case["expected_kept"],
            keep_temp=args.keep_temp,
        )
        print(msg)
        if not ok:
            all_ok = False

    return 0 if all_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())

