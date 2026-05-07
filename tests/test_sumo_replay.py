import sys
import unittest
from pathlib import Path

from linkvision_terasim.events import RealWorldStalledVehicleEvent
from linkvision_terasim.sumo_replay import (
    build_comparison_cases,
    select_replay_event,
)


def make_event(source_event_id, timestamp):
    return RealWorldStalledVehicleEvent(
        source_event_id=source_event_id,
        task_id=14159,
        event_name="Unexpected Stop",
        timestamp=timestamp,
        camera_id=694,
        location_id=133,
        timezone="America/Los_Angeles",
        object_id=f"object-{source_event_id}",
        object_name="car",
        confidence_score=0.5,
        bbox=(0.1, 0.2, 0.3, 0.4),
        pixel_x=1.0,
        pixel_y=2.0,
        latitude=37.33538562,
        longitude=-121.89221894,
    )


class SumoReplayTests(unittest.TestCase):
    def test_select_replay_event_prefers_explicit_event_id(self):
        event = select_replay_event(
            [
                make_event(1, "2026-02-25T03:32:04.414000Z"),
                make_event(2, "2026-02-25T03:33:04.414000Z"),
            ],
            event_id=1,
        )

        self.assertEqual(event.source_event_id, 1)

    def test_select_replay_event_defaults_to_latest_timestamp(self):
        event = select_replay_event(
            [
                make_event(1, "2026-02-25T03:32:04.414000Z"),
                make_event(2, "2026-02-25T03:33:04.414000Z"),
            ]
        )

        self.assertEqual(event.source_event_id, 2)

    def test_build_comparison_cases_generates_bench_and_dynamic_commands(self):
        cases = build_comparison_cases(
            make_event(1521072, "2026-02-25T03:32:04.414000Z"),
            output_dir=Path("out"),
            python_executable=sys.executable,
            script_path=Path("main.py"),
            gui=False,
            sim_time=40,
        )

        self.assertEqual([case.name for case in cases], ["bench", "dynamic"])
        self.assertEqual(cases[0].output_path, Path("out/delay_linkvision_1521072_bench.json"))
        self.assertEqual(cases[1].output_path, Path("out/delay_linkvision_1521072_dynamic.json"))
        self.assertIn("--obstacles", cases[0].command)
        self.assertIn("37.33538562,-121.89221894", cases[0].command)
        self.assertIn("--mode", cases[1].command)
        self.assertIn("dynamic", cases[1].command)
        self.assertIn("--no-gui", cases[1].command)
        self.assertIn("--sim-time", cases[1].command)
        self.assertIn("40", cases[1].command)


if __name__ == "__main__":
    unittest.main()
