#!/usr/bin/env python3
"""
Scene 2 Demo — fixed overhead camera at zoom=600, wider field of view.

Camera centered at (37.331620, -121.898827), with extra traffic density
near intersection (37.331966, -121.900180).

All vehicles placed at once and released simultaneously. Recording
starts 5-6s after release.

Usage:
  python main_scene2.py \
      --config san_jose_full_new/simulation.sumocfg \
      --net-file san_jose_full_new/osm_modified.net.xml \
      --route-file san_jose_full_new/intersection_flows.rou.xml \
      --sim-time 30 --gui
"""

import sys
import os
import math
import time
import random
import argparse
import json

from main_demo import SUMODemoRunner
from main import parse_obstacles

try:
    import traci
except ImportError:
    print("Error: Unable to import traci module")
    sys.exit(1)


class SUMOScene2Runner(SUMODemoRunner):
    """Fixed-camera scene 2 demo — wider zoom, no rotation."""

    # Scene A camera center
    CAM_LAT = 37.331739
    CAM_LON = -121.903795

    # Scene A dense intersection center
    DENSE_LAT = 37.331966
    DENSE_LON = -121.900180

    # Scene B camera center
    CAM_B_LAT = 37.331375
    CAM_B_LON = -121.881549

    def __init__(self, zoom_level=800, bbox=None, bbox_b=None,
                 scene_duration=10, **kwargs):
        super().__init__(**kwargs)
        self.zoom_level = zoom_level
        self.bbox = bbox        # Scene A: (lat_min, lon_min, lat_max, lon_max)
        self.bbox_b = bbox_b    # Scene B: (lat_min, lon_min, lat_max, lon_max)
        self.scene_duration = scene_duration  # seconds per scene
        self.rotate_angle = 0.0

    # Vehicle type mix probabilities (matches realistic_traffic_mix)
    _mix_types = [
        ('car_conservative', 0.15), ('car_normal', 0.22), ('car_aggressive', 0.10),
        ('suv_conservative', 0.10), ('suv_normal', 0.13), ('suv_aggressive', 0.07),
        ('pickup_conservative', 0.04), ('pickup_normal', 0.06), ('pickup_aggressive', 0.02),
        ('truck_delivery', 0.03), ('bus_transit', 0.03), ('bus_school', 0.02),
        ('motorcycle_normal', 0.03),
    ]

    def _pick_mix_type(self, rng):
        """Pick a random vehicle type from the realistic mix."""
        r = rng.random()
        cumulative = 0.0
        for vtype, prob in self._mix_types:
            cumulative += prob
            if r <= cumulative:
                return vtype
        return 'car_normal'

    _CACHE_FILE = 'san_jose_full_new/scene2a_vehicles_cache.json'
    _CACHE_FILE_B = 'san_jose_full_new/scene2b_vehicles_cache.json'

    def _compute_vehicle_plan(self, count, dense_extra, bbox=None,
                               dense_center=None, cache_file=None,
                               fill_edges=None):
        """Compute vehicle placement plan (edge, lane, pos, dest, type).
        fill_edges: list of edge IDs to pack full with vehicles (one per ~8m).
        """
        if dense_center:
            dense_x, dense_y = self.latlon_to_xy(*dense_center)
        else:
            dense_x, dense_y = self.latlon_to_xy(self.DENSE_LAT, self.DENSE_LON)
        dense_radius = 150

        bbox = bbox or self.bbox
        if bbox:
            lat_min, lon_min, lat_max, lon_max = bbox
            x_min, y_min = self.latlon_to_xy(lat_min, lon_min)
            x_max, y_max = self.latlon_to_xy(lat_max, lon_max)
            x_lo, x_hi = min(x_min, x_max), max(x_min, x_max)
            y_lo, y_hi = min(y_min, y_max), max(y_min, y_max)
            print(f"    BBox: ({lat_min},{lon_min}) to ({lat_max},{lon_max})")
        else:
            cam_x, cam_y = self.latlon_to_xy(self.CAM_LAT, self.CAM_LON)
            ppm = self.zoom_level / 100.0
            half_w = 3840 / ppm / 2 + 50
            half_h = 2160 / ppm / 2 + 50
            x_lo, x_hi = cam_x - half_w, cam_x + half_w
            y_lo, y_hi = cam_y - half_h, cam_y + half_h

        all_edges = traci.edge.getIDList()
        view_edges = []
        dense_edges = []
        for edge_id in all_edges:
            if edge_id.startswith(':'):
                continue
            lane0 = f"{edge_id}_0"
            try:
                shape = traci.lane.getShape(lane0)
            except Exception:
                continue
            mid = shape[len(shape) // 2]
            try:
                disallowed = traci.lane.getDisallowed(lane0)
                if 'passenger' in disallowed:
                    continue
            except Exception:
                continue
            n_lanes = traci.edge.getLaneNumber(edge_id)
            length = traci.lane.getLength(lane0)
            if length < 15:
                continue
            if x_lo <= mid[0] <= x_hi and y_lo <= mid[1] <= y_hi:
                view_edges.append((edge_id, n_lanes, length))
            dist_dense = math.sqrt((mid[0] - dense_x)**2 + (mid[1] - dense_y)**2)
            if dist_dense < dense_radius:
                dense_edges.append((edge_id, n_lanes, length))

        total_lane_km = sum(n * l for _, n, l in view_edges) / 1000
        print(f"    BBox edges: {len(view_edges)} ({total_lane_km:.1f} lane-km), Dense edges: {len(dense_edges)}")

        rng = random.Random(self.bg_seed)
        plan = []
        veh_counter = 0

        def plan_edges(edges, target_count, prefix):
            nonlocal veh_counter
            if not edges:
                return
            total_cap = sum(n * l for _, n, l in edges)
            all_eids = [e for e, _, _ in edges]
            for edge_id, n_lanes, length in edges:
                cap_frac = (n_lanes * length) / total_cap
                edge_count = max(1, round(target_count * cap_frac))
                other_edges = [e for e in all_eids if e != edge_id]
                if not other_edges:
                    other_edges = all_eids
                for j in range(edge_count):
                    vid = f"{prefix}_{veh_counter}"
                    veh_counter += 1
                    pos = (j + 1) / (edge_count + 1) * length
                    pos = max(5.0, min(pos, length - 5.0))
                    lane_idx = j % n_lanes
                    dest = rng.choice(other_edges)
                    vtype = self._pick_mix_type(rng)
                    result = traci.simulation.findRoute(edge_id, dest)
                    if not result.edges:
                        continue
                    plan.append({
                        'id': vid,
                        'edge': edge_id,
                        'lane_idx': lane_idx,
                        'pos': round(pos, 2),
                        'dest': dest,
                        'type': vtype,
                        'route': list(result.edges),
                    })

        plan_edges(view_edges, count, "s2_veh")
        plan_edges(dense_edges, dense_extra, "s2_dense")

        # Fill specific edges completely
        if fill_edges:
            all_eids = [e for e, _, _ in view_edges] or [eid for eid in traci.edge.getIDList() if not eid.startswith(':')]
            for fill_eid in fill_edges:
                lane0 = f"{fill_eid}_0"
                try:
                    length = traci.lane.getLength(lane0)
                    n_lanes = traci.edge.getLaneNumber(fill_eid)
                except Exception:
                    print(f"    WARNING: fill edge {fill_eid} not found")
                    continue
                spacing = 8.0  # meters between vehicles
                other_edges = [e for e in all_eids if e != fill_eid]
                if not other_edges:
                    other_edges = all_eids
                for lane_idx in range(n_lanes):
                    pos = 5.0
                    while pos < length - 5.0:
                        vid = f"s2_fill_{veh_counter}"
                        veh_counter += 1
                        dest = rng.choice(other_edges)
                        vtype = self._pick_mix_type(rng)
                        result = traci.simulation.findRoute(fill_eid, dest)
                        if result.edges:
                            plan.append({
                                'id': vid,
                                'edge': fill_eid,
                                'lane_idx': lane_idx,
                                'pos': round(pos, 2),
                                'dest': dest,
                                'type': vtype,
                                'route': list(result.edges),
                            })
                        pos += spacing
                print(f"    Filled edge {fill_eid}: {n_lanes} lanes, {length:.0f}m")

        # Save cache
        cache = cache_file or self._CACHE_FILE
        with open(cache, 'w') as f:
            json.dump(plan, f, indent=2)
        print(f"    Saved {len(plan)} vehicle plan to {cache}")
        return plan

    def _place_scene2_vehicles(self, count=2000, dense_extra=400,
                                bbox=None, dense_center=None, cache_file=None,
                                fill_edges=None):
        """Place background vehicles. Uses cache if available, otherwise computes."""
        cache = cache_file or self._CACHE_FILE
        # Try loading from cache
        if os.path.exists(cache):
            with open(cache, 'r') as f:
                plan = json.load(f)
            print(f"    Loaded {len(plan)} vehicles from cache: {cache}")
        else:
            print("    No cache found, computing vehicle placement...")
            plan = self._compute_vehicle_plan(count, dense_extra,
                                               bbox=bbox, dense_center=dense_center,
                                               cache_file=cache,
                                               fill_edges=fill_edges)

        # Place vehicles from plan
        placed = []
        for v in plan:
            vid = v['id']
            lane_id = f"{v['edge']}_{v['lane_idx']}"
            try:
                route_id = f"{vid}_route"
                traci.route.add(route_id, v['route'])
                traci.vehicle.add(
                    vehID=vid,
                    routeID=route_id,
                    typeID=v['type'],
                    depart='now',
                    departLane=str(v['lane_idx']),
                    departPos=str(v['pos']),
                    departSpeed='0',
                )
                traci.vehicle.moveTo(vid, lane_id, v['pos'])
                traci.vehicle.setSpeedMode(vid, 0)
                traci.vehicle.setSpeed(vid, 0)
                traci.vehicle.setLaneChangeMode(vid, 0)
                placed.append(vid)
            except Exception:
                pass

        print(f"    Placed {len(placed)} vehicles")
        return placed

    def _release_all_vehicles(self, veh_ids):
        """Release all frozen vehicles at once."""
        released = 0
        for vid in veh_ids:
            try:
                if vid not in traci.vehicle.getIDList():
                    continue
                lane_id = traci.vehicle.getLaneID(vid)
                lane_pos = traci.vehicle.getLanePosition(vid)
                if lane_id and not lane_id.startswith(':'):
                    traci.vehicle.moveTo(vid, lane_id, lane_pos)
                traci.vehicle.setLaneChangeMode(vid, 1621)
                traci.vehicle.setSpeedMode(vid, 31)
                traci.vehicle.setSpeed(vid, -1)
                released += 1
            except Exception:
                pass
        print(f"    Released {released} vehicles")

    def _compose_scene_video(self, scene_name, frame_start, frame_end):
        """Compose video from a range of frames, named by scene + timestamp."""
        import subprocess
        from PIL import Image

        frame_dir = self._screenshot_subdir
        video_output = os.path.join(
            self.screenshot_dir, f"{scene_name}_{self._timestamp}.mp4"
        )

        # Count actual frames on disk in range
        total = 0
        for i in range(frame_start, frame_end):
            if os.path.exists(os.path.join(frame_dir, f"frame_{i:06d}.png")):
                total += 1
            else:
                break
        if total == 0:
            print(f"  No frames for {scene_name}")
            return

        fps = round(1.0 / self.step_length)
        duration = total / fps
        print(f"\n  Composing {scene_name} ({total} frames @ {fps}fps = {duration:.1f}s) ...")

        # Create symlinks with sequential numbering for ffmpeg
        scene_dir = os.path.join(frame_dir, scene_name)
        os.makedirs(scene_dir, exist_ok=True)
        for j, i in enumerate(range(frame_start, frame_start + total)):
            src = os.path.join(frame_dir, f"frame_{i:06d}.png")
            dst = os.path.join(scene_dir, f"frame_{j:06d}.png")
            if not os.path.exists(dst):
                os.symlink(os.path.abspath(src), dst)

        # No rotation — center crop via ffmpeg
        out_w, out_h = self.output_w, self.output_h
        first = Image.open(os.path.join(scene_dir, 'frame_000000.png'))
        src_w, src_h = first.size
        first.close()
        crop_x = (src_w - out_w) // 2
        crop_y = (src_h - out_h) // 2
        vf_filter = f'crop={out_w}:{out_h}:{crop_x}:{crop_y}'

        cmd = [
            'ffmpeg', '-y',
            '-framerate', str(fps),
            '-i', os.path.join(scene_dir, 'frame_%06d.png'),
            '-vf', vf_filter,
            '-c:v', 'libx264', '-pix_fmt', 'yuv420p',
            '-crf', '18', '-preset', 'medium',
            '-tune', 'fastdecode',
            '-bf', '0', '-refs', '3',
            '-movflags', '+faststart',
            video_output
        ]
        subprocess.run(cmd, check=True)
        print(f"  Video: {video_output}")

    def run_simulation(self):
        """Scene 2: place all vehicles, release, record after delay."""
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

        has_scene_b = self.bbox_b is not None

        print(f"\n{'='*60}")
        print(f"  SCENE 2 Recording Mode (fixed camera, zoom={self.zoom_level})")
        print(f"  Scene A center: ({self.CAM_LAT}, {self.CAM_LON})")
        if has_scene_b:
            print(f"  Scene B center: ({self.CAM_B_LAT}, {self.CAM_B_LON})")
        print(f"  Duration per scene: {self.scene_duration}s")
        print(f"{'='*60}")

        # ==============================================================
        #  Phase 1 — Start SUMO
        # ==============================================================
        print("\n  Phase 1: Starting SUMO")

        flowless_route = self._create_flowless_route_file()
        # Total sim time: scene_duration * number_of_scenes + buffer
        n_scenes = 2 if has_scene_b else 1
        total_end = self.scene_duration * n_scenes + 20

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

            # Build spatial index (needed for both scenes)
            self._preload_edge_positions()
            self.manual_vehicle_ids = []
            self._pending_destinations = {}
            self._delayed_vehicle_indices = []
            s2_ids = []
            step = 0
            flow_defs = self._parse_flow_definitions()
            flow_start_sim_time = traci.simulation.getTime()

            # ==============================================================
            #  Phase 2 — Set camera + place Scene A vehicles (frozen)
            # ==============================================================
            print("\n  Phase 2: Setting camera + placing Scene A vehicles")

            # Add stalled vehicle (obstacle) if configured
            self.add_obstacles_via_traci()
            traci.simulationStep()
            self.update_obstacle_positions()

            # Set camera
            target_x, target_y = self.latlon_to_xy(self.CAM_LAT, self.CAM_LON)
            traci.gui.setOffset("View #0", target_x, target_y)
            traci.gui.setZoom("View #0", self.zoom_level)
            traci.simulationStep()

            # Place initial manual vehicles (frozen)
            n_manual = len(self.initial_vehicles) if self.initial_vehicles else 0
            for veh_idx in range(n_manual):
                spawn_t = self._get_spawn_time(self.initial_vehicles[veh_idx])
                if spawn_t > 0:
                    self._delayed_vehicle_indices.append(veh_idx)
                    continue
                self._inject_single_vehicle(veh_idx)
            traci.simulationStep()
            print(f"    Placed {len(self.manual_vehicle_ids)} initial vehicles (frozen)")
            self._pending_destinations = {}

            # Place scene A background vehicles (all frozen, instant via moveTo)
            s2_ids = self._place_scene2_vehicles(
                count=self.bg_vehicle_count,
                dense_extra=340,
            )
            traci.simulationStep()
            s2_ids = self._check_conflicts(s2_ids)
            n_veh = len(traci.vehicle.getIDList())
            print(f"    Total vehicles on network: {n_veh}")

            # ==============================================================
            #  Phase 3 — Release Scene A vehicles + record
            # ==============================================================
            print("\n  Phase 3: Recording Scene A")
            all_to_release = self.manual_vehicle_ids + s2_ids
            self._release_all_vehicles(all_to_release)
            self._capture_enabled = True
            flow_start_sim_time = traci.simulation.getTime()
            scene_a_end = flow_start_sim_time + self.scene_duration
            print(f"    Recording: {flow_start_sim_time:.1f}s → {scene_a_end:.1f}s")
            while traci.simulation.getTime() < scene_a_end:
                self._step_and_capture()
                elapsed = traci.simulation.getTime() - flow_start_sim_time
                self._spawn_flow_vehicles(flow_defs, elapsed)
                sim_t_now = traci.simulation.getTime()
                still_pending = []
                for _di in self._delayed_vehicle_indices:
                    _spawn_t = self._get_spawn_time(self.initial_vehicles[_di])
                    if sim_t_now >= _spawn_t:
                        self._inject_single_vehicle(_di, release=True)
                    else:
                        still_pending.append(_di)
                self._delayed_vehicle_indices = still_pending
                self.update_obstacle_positions()
                self.update_tls_program()
                self.trigger_rerouting(step)
                self.assist_stuck_vehicles(step * self.step_length)
                self.remove_stuck_vehicles(step * self.step_length)
                self.collect_vehicle_data(step * self.step_length)
                if step % 100 == 0:
                    sim_t = traci.simulation.getTime()
                    n_veh = len(traci.vehicle.getIDList())
                    print(f"    [A] sim={sim_t:.1f}/{scene_a_end:.1f}s  vehicles={n_veh}", end='\r')
                step += 1
            scene_a_frames = self._frame_counter
            print(f"\n    Scene A recording done ({scene_a_frames} frames)")

            # ==============================================================
            #  Phase 4 — Scene B (if configured)
            # ==============================================================
            if has_scene_b:
                print("\n  Phase 4: Switching to Scene B")

                # Clear Scene A background vehicles only (keep manual + stalled)
                removed = 0
                for vid in s2_ids:
                    try:
                        if vid in traci.vehicle.getIDList():
                            traci.vehicle.remove(vid)
                            removed += 1
                    except Exception:
                        pass
                traci.simulationStep()
                print(f"    Cleared {removed} Scene A bg vehicles (kept manual + stalled)")

                # Move camera to Scene B center
                target_bx, target_by = self.latlon_to_xy(self.CAM_B_LAT, self.CAM_B_LON)
                traci.gui.setOffset("View #0", target_bx, target_by)
                traci.gui.setZoom("View #0", self.zoom_level)
                traci.simulationStep()

                # Place Scene B vehicles
                print("    Placing Scene B vehicles...")
                sb_ids = self._place_scene2_vehicles(
                    count=self.bg_vehicle_count,
                    dense_extra=0,
                    bbox=self.bbox_b,
                    cache_file=self._CACHE_FILE_B,
                    fill_edges=['1058060842#1'],
                )
                traci.simulationStep()
                sb_ids = self._check_conflicts(sb_ids)
                n_veh = len(traci.vehicle.getIDList())
                print(f"    Total vehicles on network: {n_veh}")

                # Release Scene B vehicles
                self._release_all_vehicles(sb_ids)
                self._capture_enabled = True

                scene_b_end = traci.simulation.getTime() + self.scene_duration
                print(f"    Recording Scene B: {traci.simulation.getTime():.1f}s → {scene_b_end:.1f}s")

                while traci.simulation.getTime() < scene_b_end:
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
                        print(f"    [B] sim={sim_t:.1f}/{scene_b_end:.1f}s  vehicles={n_veh}", end='\r')
                    step += 1

                print(f"\n    Scene B recording done ({self._frame_counter} frames total)")

            print("\n  Simulation completed")
            traci.close()

            if self.screenshot_dir and self._frame_counter > 0:
                print(f"\n  Screenshots saved: {self._frame_counter} frames")
                print(f"  Directory: {self._screenshot_subdir}")
                if has_scene_b and scene_a_frames > 0:
                    self._compose_scene_video("sceneA", 0, scene_a_frames)
                    self._compose_scene_video("sceneB", scene_a_frames, self._frame_counter)
                else:
                    self._compose_scene_video("sceneA", 0, self._frame_counter)

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
        description='SUMO Scene 2 Demo (fixed camera, zoom=600)',
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
                        default="traffic_data_analysis/delay_result/delay_scene2.json")
    parser.add_argument('--tripinfo-output', default=None)
    parser.add_argument('--statistic-output', default=None)
    parser.add_argument('--demo-config', default='demo_config.json')
    parser.add_argument('--screenshot-dir', default=None)
    parser.add_argument('--bg-vehicle-count', type=int, default=None)
    parser.add_argument('--obstacles', default=None,
                        help='Stalled vehicle GPS: "lat,lon" or "lat1,lon1;lat2,lon2"')
    parser.add_argument('--zoom-level', type=float, default=800,
                        help='Camera zoom level (default 800)')
    parser.add_argument('--bbox', default=None,
                        help='Scene A bounding box: "lat_min,lon_min,lat_max,lon_max"')
    parser.add_argument('--bbox-b', default=None,
                        help='Scene B bounding box: "lat_min,lon_min,lat_max,lon_max"')
    parser.add_argument('--scene-duration', type=int, default=10,
                        help='Recording duration per scene in seconds (default 10)')

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

    # Parse bboxes
    bbox = None
    if args.bbox:
        parts = [float(x.strip()) for x in args.bbox.split(',')]
        bbox = (parts[0], parts[1], parts[2], parts[3])
    bbox_b = None
    if args.bbox_b:
        parts = [float(x.strip()) for x in args.bbox_b.split(',')]
        bbox_b = (parts[0], parts[1], parts[2], parts[3])

    runner = SUMOScene2Runner(
        zoom_level=args.zoom_level,
        bbox=bbox,
        bbox_b=bbox_b,
        scene_duration=args.scene_duration,
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
        print("\n  Scene 2 demo completed!")
        sys.exit(0)
    else:
        print("\n  Scene 2 demo failed!")
        sys.exit(1)


if __name__ == '__main__':
    main()
