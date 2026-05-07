#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict
from pathlib import Path

from linkvision_terasim.sumo_replay import (
    build_comparison_cases,
    load_replay_events,
    run_replay_cases,
    select_replay_event,
)


PROJECT_ROOT = Path(__file__).resolve().parent
DEFAULT_EVENTS_JSON = PROJECT_ROOT / "traffic_data_analysis/linkVision_rawData/response_stalled_car_detected.json"
DEFAULT_METADATA_JSON = PROJECT_ROOT / "traffic_data_analysis/linkVision_rawData/response_its_task_metadata.json"
DEFAULT_OUTPUT_DIR = PROJECT_ROOT / "traffic_data_analysis/delay_result"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Replay real LinkVision stalled-vehicle events in the San Jose SUMO demo."
    )
    parser.add_argument("--events-json", type=Path, default=DEFAULT_EVENTS_JSON)
    parser.add_argument("--metadata-json", type=Path, default=DEFAULT_METADATA_JSON)
    parser.add_argument("--event-id", type=int, default=None)
    parser.add_argument("--min-confidence", type=float, default=0.0)
    parser.add_argument("--mode", choices=["bench", "dynamic", "compare"], default="compare")
    parser.add_argument("--net-file", type=Path, default=Path("san_jose_downtown_gtc/osm.net.xml"))
    parser.add_argument(
        "--route-file",
        type=Path,
        default=Path("san_jose_downtown_gtc/directional_traffic.rou.xml"),
    )
    parser.add_argument("--main-script", type=Path, default=PROJECT_ROOT / "main.py")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--sim-time", type=int, default=1800)
    parser.add_argument("--gui", action="store_true", default=False)
    parser.add_argument("--no-gui", dest="gui", action="store_false")
    parser.add_argument("--dry-run", action="store_true", default=False)
    parser.add_argument("--python", default=sys.executable)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    events = load_replay_events(
        args.events_json,
        args.metadata_json,
        min_confidence=args.min_confidence,
    )
    event = select_replay_event(events, event_id=args.event_id)
    modes = ("bench", "dynamic") if args.mode == "compare" else (args.mode,)
    cases = build_comparison_cases(
        event=event,
        output_dir=args.output_dir,
        python_executable=args.python,
        script_path=args.main_script,
        net_file=args.net_file,
        route_file=args.route_file,
        gui=args.gui,
        sim_time=args.sim_time,
        modes=modes,
    )

    print("Selected LinkVision stalled-vehicle event:")
    print(json.dumps(asdict(event), indent=2, sort_keys=True))
    print("\nReplay commands:")
    for case in cases:
        print(" ".join(case.command))

    if args.dry_run:
        return 0

    run_replay_cases(cases)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
