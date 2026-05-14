#!/usr/bin/env python3
from __future__ import annotations
"""CLI entrypoint for running LinkVision event replay in TeraSim."""

import argparse
import json
from dataclasses import asdict, replace
from pathlib import Path

from linkvision_terasim.sumo_replay import load_replay_events, select_replay_event
from linkvision_terasim.terasim_runner import (
    DEFAULT_TERASIM_HOME,
    TeraSimReplayConfig,
    run_terasim_replay,
)


PROJECT_ROOT = Path(__file__).resolve().parent
DEFAULT_EVENTS_JSON = PROJECT_ROOT / "traffic_data_analysis/linkVision_rawData/response_stalled_car_detected.json"
DEFAULT_METADATA_JSON = PROJECT_ROOT / "traffic_data_analysis/linkVision_rawData/response_its_task_metadata.json"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Replay real LinkVision stalled-vehicle events through TeraSim."
    )
    parser.add_argument("--events-json", type=Path, default=DEFAULT_EVENTS_JSON)
    parser.add_argument("--metadata-json", type=Path, default=DEFAULT_METADATA_JSON)
    parser.add_argument("--event-id", type=int, default=None)
    parser.add_argument("--min-confidence", type=float, default=0.0)
    parser.add_argument("--mode", choices=["bench", "dynamic", "compare"], default="compare")
    parser.add_argument("--terasim-home", type=Path, default=DEFAULT_TERASIM_HOME)
    parser.add_argument("--sumo-net-file", type=Path, default=Path("san_jose_downtown_gtc/osm.net.xml"))
    parser.add_argument("--sumo-config-file", type=Path, default=Path("san_jose_downtown_gtc/osm.sumocfg"))
    parser.add_argument(
        "--route-file",
        type=Path,
        default=Path("san_jose_downtown_gtc/directional_traffic.rou.xml"),
        help="Route file passed to SUMO via --route-files (overrides sumocfg). "
             "Defaults to directional_traffic.rou.xml to match run_linkvision_replay.py.",
    )
    parser.add_argument("--output-dir", type=Path, default=Path("outputs/linkvision_terasim"))
    parser.add_argument("--sim-time", type=int, default=1800)
    parser.add_argument("--step-length", type=float, default=0.1)
    parser.add_argument("--gui", action="store_true", default=False)
    parser.add_argument("--no-gui", dest="gui", action="store_false")
    parser.add_argument("--dry-run", action="store_true", default=False)
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

    print("Selected LinkVision stalled-vehicle event:")
    print(json.dumps(asdict(event), indent=2, sort_keys=True))
    print("\nTeraSim replay cases:")
    for mode in modes:
        case_config = TeraSimReplayConfig(
            terasim_home=args.terasim_home,
            sumo_net_file=args.sumo_net_file,
            sumo_config_file=args.sumo_config_file,
            sumo_route_file=args.route_file,
            output_path=args.output_dir / f"linkvision_{event.source_event_id}_{mode}",
            mode=mode,
            gui=args.gui,
            sim_time=args.sim_time,
            step_length=args.step_length,
        )
        print(json.dumps(asdict(case_config), indent=2, default=str, sort_keys=True))
        if not args.dry_run:
            run_terasim_replay(event, replace(case_config))

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
