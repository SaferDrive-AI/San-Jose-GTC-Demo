#!/usr/bin/env python3
"""
Overview Demo Video — fixed overhead camera, no stalled vehicle, no zoom/rotation.

Simplified version of main_demo.py:
  - No stalled vehicle (obstacle)
  - Camera stays at dwell zoom level (zoom_mid_level) throughout
  - No zoom-in animation, no rotation
  - 5s warmup, recording starts at 10s, total sim_time configurable

Usage:
  python main_overview.py \
      --config san_jose_full_new/simulation.sumocfg \
      --net-file san_jose_full_new/osm.net.xml.gz \
      --route-file san_jose_full_new/intersection_flows.rou.xml \
      --sim-time 40 --gui
"""

import sys
import os
import math
import time
import argparse
import json

from main_demo import SUMODemoRunner
from main import parse_obstacles

try:
    import traci
except ImportError:
    print("Error: Unable to import traci module")
    sys.exit(1)


class SUMOOverviewRunner(SUMODemoRunner):
    """Fixed-camera overview demo — no stalled vehicle, no zoom/rotation."""

    def __init__(self, warmup_seconds=10.0, **kwargs):
        super().__init__(**kwargs)
        self.warmup_seconds = warmup_seconds
        # Force no rotation for this mode
        self.rotate_angle = 0.0

    def run_simulation(self):
        """Simplified simulation: fixed camera, no obstacles, no freeze."""
        from datetime import datetime

        self._frame_counter = 0
        self._frame_angles = []
        self._current_rotation = 0.0
        self._capture_enabled = False
        self._timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        if self.screenshot_dir:
            self._screenshot_subdir = os.path.join(self.screenshot_dir, self._timestamp)
            os.makedirs(self._screenshot_subdir, exist_ok=True)
            print(f"  Screenshots enabled: {self._screenshot_subdir}")

        print(f"\n{'='*60}")
        print(f"  OVERVIEW Recording Mode (fixed camera)")
        print(f"  Bg vehicles: {self.bg_vehicle_count}  |  Sim: {self.sim_time}s")
        print(f"  Warmup: {self.warmup_seconds}s (no recording)")
        print(f"{'='*60}")

        # ==============================================================
        #  Phase 1 — Start SUMO
        # ==============================================================
        print("\n  Phase 1: Starting SUMO (no flows)")

        flowless_route = self._create_flowless_route_file()
        total_end = self.sim_time + 10

        sumo_cmd = [
            self.sumo_binary,
            '-c', self.config_file,
            '--route-files', flowless_route,
            '--quit-on-end',
            '--time-to-teleport', '-1',
            '--time-to-teleport.highways', '-1',
            '--time-to-teleport.disconnected', '-1',
            '--collision.action', 'warn',
            '--collision.check-junctions',
            '--end', str(int(total_end)),
            '--step-length', str(self.step_length),
            '--window-size', self._compute_window_size(),
            '--delay', '0',
        ]
        if self.tripinfo_file:
            sumo_cmd += ['--tripinfo-output', os.path.abspath(self.tripinfo_file)]
        if self.statistic_file:
            sumo_cmd += ['--statistic-output', os.path.abspath(self.statistic_file)]

        try:
            traci.start(sumo_cmd)
            print("    SUMO started")

            self.set_tls_program_via_traci()
            self.update_tls_program()
            traci.simulationStep()

            # ==============================================================
            #  Phase 2 — Place vehicles + set camera
            # ==============================================================
            print("\n  Phase 2: Placing vehicles")

            # Add stalled vehicle (obstacle) if configured
            self.add_obstacles_via_traci()
            traci.simulationStep()
            self.update_obstacle_positions()

            # Set camera at dwell zoom level, centered on intersection
            target_x, target_y = self.latlon_to_xy(37.3354, -121.8921)
            traci.gui.setOffset("View #0", target_x, target_y)
            traci.gui.setZoom("View #0", self.zoom_mid_level)
            traci.simulationStep()

            # Build spatial index + place initial vehicles
            self._preload_edge_positions()

            self.manual_vehicle_ids = []
            self._pending_destinations = {}
            n_manual = len(self.initial_vehicles) if self.initial_vehicles else 0
            for veh_idx in range(n_manual):
                self._inject_single_vehicle(veh_idx)
            traci.simulationStep()

            # Release initial vehicles immediately (no freeze)
            for veh_id in self.manual_vehicle_ids:
                try:
                    lane_id = traci.vehicle.getLaneID(veh_id)
                    lane_pos = traci.vehicle.getLanePosition(veh_id)
                    if lane_id and not lane_id.startswith(':'):
                        traci.vehicle.moveTo(veh_id, lane_id, lane_pos)
                    traci.vehicle.setLaneChangeMode(veh_id, 1621)
                    traci.vehicle.setSpeedMode(veh_id, 31)
                    traci.vehicle.setSpeed(veh_id, -1)
                except Exception:
                    pass
            print(f"    Placed and released {len(self.manual_vehicle_ids)} initial vehicles")
            self._pending_destinations = {}

            # Place background vehicles
            bg_ids = self._place_background_vehicles()
            traci.simulationStep()
            bg_ids = self._check_conflicts(bg_ids)
            n_veh = len(traci.vehicle.getIDList())
            print(f"    Total vehicles on network: {n_veh}")

            # ==============================================================
            #  Phase 3 — Warmup (no recording)
            # ==============================================================
            fps = round(1.0 / self.step_length)
            warmup_steps = int(self.warmup_seconds / self.step_length)
            print(f"\n  Phase 3: Warmup {self.warmup_seconds}s ({warmup_steps} steps, no recording)")

            for s in range(warmup_steps):
                traci.simulationStep()
                self.update_tls_program()
                if s % 300 == 0:
                    sim_t = traci.simulation.getTime()
                    n_veh = len(traci.vehicle.getIDList())
                    print(f"    warmup {sim_t:.1f}s  vehicles={n_veh}", end='\r')

            # Cancel pending bg vehicles that haven't materialized
            active_ids = set(traci.vehicle.getIDList())
            cancelled = 0
            for veh_id in bg_ids:
                if veh_id not in active_ids:
                    try:
                        traci.vehicle.remove(veh_id)
                        cancelled += 1
                    except Exception:
                        pass
            bg_ids = [v for v in bg_ids if v in active_ids]
            print(f"\n    Cancelled {cancelled} pending bg, {len(bg_ids)} active")

            # ==============================================================
            #  Phase 4 — Enable recording + start flows
            # ==============================================================
            print(f"\n  Phase 4: Enable recording + start flows")
            self._capture_enabled = True

            flow_defs = self._parse_flow_definitions()
            flow_start_sim_time = traci.simulation.getTime()
            print(f"    Flow start at sim time: {flow_start_sim_time:.1f}s")
            print(f"    Recording starts at frame {self._frame_counter}")

            # ==============================================================
            #  Phase 5 — Main simulation loop
            # ==============================================================
            print(f"\n  Phase 5: Running simulation ({self.sim_time}s)")

            step = 0
            while traci.simulation.getTime() < self.sim_time:
                self._step_and_capture()

                elapsed = traci.simulation.getTime() - flow_start_sim_time
                self._spawn_flow_vehicles(flow_defs, elapsed)

                self.update_obstacle_positions()
                self.update_tls_program()
                self.trigger_rerouting(step)
                self.assist_stuck_vehicles(step * self.step_length)
                self.remove_stuck_vehicles(step * self.step_length)
                self.collect_vehicle_data(step * self.step_length)

                if step % 100 == 0:
                    sim_t = traci.simulation.getTime()
                    n_veh = len(traci.vehicle.getIDList())
                    print(f"    sim={sim_t:.1f}/{self.sim_time}s  vehicles={n_veh}", end='\r')
                step += 1

            print("\n  Simulation completed")
            traci.close()

            if self.screenshot_dir and self._frame_counter > 0:
                print(f"\n  Screenshots saved: {self._frame_counter} frames")
                print(f"  Directory: {self._screenshot_subdir}")
                self._compose_video()

            return True

        except Exception as e:
            print(f"\nError: {e}")
            import traceback
            traceback.print_exc()
            try:
                traci.close()
            except Exception:
                pass
            return False


# ======================================================================
#  CLI
# ======================================================================
def main():
    parser = argparse.ArgumentParser(
        description='SUMO Overview Demo Recording (fixed camera, no stalled vehicle)',
    )

    parser.add_argument('--net-file',
                        default="san_jose_full_new/osm_modified.net.xml")
    parser.add_argument('--route-file',
                        default="san_jose_full_new/intersection_flows.rou.xml")
    parser.add_argument('--tls-program', default=None)
    parser.add_argument('--sim-time', type=int, default=30)
    parser.add_argument('--step-length', type=float, default=0.01667)
    parser.add_argument('--gui', action='store_true', default=True)
    parser.add_argument('--no-gui', dest='gui', action='store_false')
    parser.add_argument('--mode', choices=['bench', 'opt', 'dynamic'],
                        default='bench')
    parser.add_argument('--program-id', default=None)
    parser.add_argument('--config', default=None)
    parser.add_argument('--output',
                        default="traffic_data_analysis/delay_result/delay_overview.json")
    parser.add_argument('--tripinfo-output', default=None)
    parser.add_argument('--statistic-output', default=None)
    parser.add_argument('--demo-config', default='demo_config.json')
    parser.add_argument('--screenshot-dir', default=None)
    parser.add_argument('--bg-vehicle-count', type=int, default=None)
    parser.add_argument('--obstacles', default=None,
                        help='Stalled vehicle GPS: "lat,lon" or "lat1,lon1;lat2,lon2"')
    parser.add_argument('--warmup', type=float, default=10.0,
                        help='Warmup duration in seconds before recording starts (default 10)')

    args = parser.parse_args()

    if not os.path.exists(args.net_file):
        print(f"Error: Network file not found: {args.net_file}")
        sys.exit(1)
    if not os.path.exists(args.route_file):
        print(f"Error: Route file not found: {args.route_file}")
        sys.exit(1)

    obstacles = parse_obstacles(args.obstacles)

    # Load demo config
    demo_config = {}
    if args.demo_config and os.path.exists(args.demo_config):
        with open(args.demo_config, 'r') as f:
            demo_config = json.load(f)
        print(f"  Loaded demo config: {args.demo_config}")
    if args.bg_vehicle_count is not None:
        demo_config['bg_vehicle_count'] = args.bg_vehicle_count
    if args.screenshot_dir is not None:
        demo_config['screenshot_dir'] = args.screenshot_dir

    runner = SUMOOverviewRunner(
        warmup_seconds=args.warmup,
        demo_config=demo_config,
        screenshot_dir=demo_config.get('screenshot_dir'),
        net_file=args.net_file,
        route_file=args.route_file,
        obstacles=obstacles,
        tls_program=args.tls_program,
        sim_time=args.sim_time,
        step_length=args.step_length,
        gui=args.gui,
        output_file=args.output,
        mode=args.mode,
        tripinfo_file=args.tripinfo_output,
        statistic_file=args.statistic_output,
        program_id=args.program_id,
        config_file=args.config,
        initial_vehicles=[
            # --- Standalone car ---
            ('car',         37.335575, -121.891958, '417034224#0'),

            # --- Queue of cars (WB approach) ---
            ('car',         37.335601, -121.891849, '417034224#0'),
            ('car',         37.335640, -121.891763, '417034224#0'),
            ('car',         37.335673, -121.891698, '417034224#0'),
            ('car',         37.335711, -121.891611, '417034224#0'),
            ('car',         37.335734, -121.891560, '417034224#0'),

            # --- Vehicles east of intersection ---
            ('car',         37.335663, -121.891608, '416909351#1'),
            ('car',         37.335615, -121.891647, '416909351#1'),

            # --- SB approach ---
            ('car',         37.335589, -121.892206, '495569632'),
            ('car',         37.335633, -121.892238, '495569632'),
            ('car',         37.335603, -121.892183, '157781953#2'),

            # --- NB approach ---
            ('car',         37.335352, -121.891944, '157782193#2'),

            # --- Bus ---
            ('bus_transit',  37.335657, -121.891786, '417034224#0'),
        ],
    )

    results = runner.run()

    if results:
        print("\n  Overview demo completed!")
        sys.exit(0)
    else:
        print("\n  Overview demo failed!")
        sys.exit(1)


if __name__ == '__main__':
    main()
