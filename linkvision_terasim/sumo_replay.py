from __future__ import annotations
"""Replay converted LinkVision events directly in SUMO.

Provides helpers to load calibrated events, choose replay targets, assemble
benchmark/dynamic run commands, and execute case comparisons.
"""

import subprocess
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Iterable, Sequence

from .events import (
    RealWorldStalledVehicleEvent,
    event_to_obstacle_arg,
    iter_stalled_vehicle_events,
    load_camera_calibrations,
    load_json,
)


@dataclass(frozen=True)
class ReplayCase:
    name: str
    mode: str
    command: list[str]
    output_path: Path


def _parse_timestamp(timestamp: str) -> datetime:
    return datetime.fromisoformat(timestamp.replace("Z", "+00:00"))


def select_replay_event(
    events: Sequence[RealWorldStalledVehicleEvent],
    event_id: int | None = None,
) -> RealWorldStalledVehicleEvent:
    if not events:
        raise ValueError("no stalled vehicle events are available for replay")

    if event_id is not None:
        for event in events:
            if event.source_event_id == event_id:
                return event
        raise ValueError(f"event_id {event_id} was not found in converted stalled vehicle events")

    return max(events, key=lambda event: _parse_timestamp(event.timestamp))


def load_replay_events(
    events_json: str | Path,
    metadata_json: str | Path,
    min_confidence: float = 0.0,
) -> list[RealWorldStalledVehicleEvent]:
    calibrations = load_camera_calibrations(metadata_json)
    raw_events = load_json(events_json)
    return list(
        iter_stalled_vehicle_events(
            raw_events,
            calibrations,
            min_confidence=min_confidence,
        )
    )


def build_sumo_command(
    event: RealWorldStalledVehicleEvent,
    mode: str,
    output_path: Path,
    python_executable: str,
    script_path: Path,
    net_file: Path,
    route_file: Path,
    gui: bool,
    sim_time: int,
) -> list[str]:
    command = [
        python_executable,
        str(script_path),
        "--net-file",
        str(net_file),
        "--route-file",
        str(route_file),
        "--obstacles",
        event_to_obstacle_arg(event),
        "--mode",
        mode,
        "--output",
        str(output_path),
        "--sim-time",
        str(sim_time),
    ]
    command.append("--gui" if gui else "--no-gui")
    return command


def build_comparison_cases(
    event: RealWorldStalledVehicleEvent,
    output_dir: Path,
    python_executable: str,
    script_path: Path,
    gui: bool,
    sim_time: int,
    net_file: Path = Path("san_jose_downtown_gtc/osm.net.xml"),
    route_file: Path = Path("san_jose_downtown_gtc/directional_traffic.rou.xml"),
    modes: Iterable[str] = ("bench", "dynamic"),
) -> list[ReplayCase]:
    cases = []
    for mode in modes:
        output_path = output_dir / f"delay_linkvision_{event.source_event_id}_{mode}.json"
        cases.append(
            ReplayCase(
                name=mode,
                mode=mode,
                output_path=output_path,
                command=build_sumo_command(
                    event=event,
                    mode=mode,
                    output_path=output_path,
                    python_executable=python_executable,
                    script_path=script_path,
                    net_file=net_file,
                    route_file=route_file,
                    gui=gui,
                    sim_time=sim_time,
                ),
            )
        )
    return cases


def run_replay_cases(cases: Iterable[ReplayCase]) -> None:
    for case in cases:
        case.output_path.parent.mkdir(parents=True, exist_ok=True)
        subprocess.run(case.command, check=True)
