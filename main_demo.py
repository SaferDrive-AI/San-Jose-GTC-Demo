#!/usr/bin/env python3
"""
Demo Video Recording version of SUMO Traffic Simulation.

Workflow:
  Phase 1  Start         — launch SUMO with NO flows, set overview camera
  Phase 2  Place         — add stalled + initial + background vehicles
  Phase 3  Conflict      — remove bg vehicles overlapping initial/obstacle
  Phase 4  Zoom-in       — cinematic camera move (bg vehicles moving)
  Phase 5  Pause+Release — 10s pause, remove bg outside viewport, release vehicles
  Phase 6  Start flows   — begin r_1-r_12 dynamic spawning
  Phase 7  Main loop     — run simulation with data collection

Usage:
  python main_demo.py \
      --config san_jose_full_new/simulation.sumocfg \
      --net-file san_jose_full_new/osm.net.xml.gz \
      --route-file san_jose_full_new/intersection_flows.rou.xml \
      --obstacles "37.335577,-121.891913" \
      --zoom-in-duration 5 --freeze-duration 8 \
      --gui
"""

import sys
import os
import math
import time
import random
import argparse
import json
import xml.etree.ElementTree as ET
import tempfile
import subprocess
from datetime import datetime

from main import SUMODelayCalculator, parse_obstacles

try:
    import traci
except ImportError:
    print("Error: Unable to import traci module")
    sys.exit(1)


class SUMODemoRunner(SUMODelayCalculator):
    """Extended calculator with demo video recording workflow."""

    def __init__(self, demo_config=None,
                 config_file=None,
                 initial_vehicles=None, initial_stop_duration=0,
                 screenshot_dir=None,
                 **kwargs):
        super().__init__(**kwargs)
        if config_file:
            self.config_file = os.path.abspath(config_file)
        self.initial_vehicles = initial_vehicles or []
        self.initial_stop_duration = initial_stop_duration

        # Load demo parameters from config dict (from JSON file)
        cfg = demo_config or {}
        self.screenshot_dir = screenshot_dir or cfg.get('screenshot_dir')
        self.zoom_in_duration = cfg.get('zoom_in_duration', 8.0)
        self.zoom_out_level = cfg.get('zoom_out_level', 200)
        self.zoom_mid_level = cfg.get('zoom_mid_level', 300)
        self.zoom_final_level = cfg.get('zoom_final_level', 10000)
        self.zoom_fast_fraction = cfg.get('zoom_fast_fraction', 0.4)
        self.zoom_step_delay = cfg.get('zoom_step_delay', 0.017)
        self.zoom_dwell_duration = cfg.get('zoom_dwell_duration', 6.0)
        self.bg_vehicle_count = cfg.get('bg_vehicle_count', 4000)
        self.bg_seed = cfg.get('bg_seed', 42)
        self.freeze_duration = cfg.get('freeze_duration', 10.0)
        self.conflict_distance = cfg.get('conflict_distance', 10.0)
        self.rotate_angle = cfg.get('rotate_angle', 60.0)
        self.output_w = cfg.get('output_w', 3840)
        self.output_h = cfg.get('output_h', 2160)
        self._frame_angles = []  # per-frame rotation angle for post-processing
        # Expanded downtown bounding box (lon_min, lat_min, lon_max, lat_max)
        bg_bounds_str = cfg.get('bg_bounds', '-121.8985,37.3315,-121.8855,37.3395')
        if isinstance(bg_bounds_str, str):
            self.bg_bounds = tuple(float(v) for v in bg_bounds_str.split(','))
        else:
            self.bg_bounds = tuple(bg_bounds_str)

    # ------------------------------------------------------------------
    #  Fast edge lookup (built from net file, no TraCI)
    # ------------------------------------------------------------------
    def _preload_edge_positions(self):
        """Build spatial index by parsing the network XML file directly.

        Much faster than TraCI (~2s vs minutes for large networks).
        """
        if hasattr(self, '_edge_index') and self._edge_index:
            return  # already built
        print("    Building spatial edge index ...", end='', flush=True)
        t0 = time.time()
        self._edge_index = []
        tree = ET.parse(self.net_file)
        root = tree.getroot()
        for edge_elem in root.iter('edge'):
            eid = edge_elem.get('id', '')
            if eid.startswith(':'):
                continue
            for lane_elem in edge_elem.iter('lane'):
                lidx = int(lane_elem.get('index', 0))
                shape_str = lane_elem.get('shape', '')
                if not shape_str:
                    continue
                for pt in shape_str.split():
                    sx, sy = pt.split(',')
                    self._edge_index.append((float(sx), float(sy), eid, lidx))
        dt = time.time() - t0
        print(f" {len(self._edge_index)} points in {dt:.1f}s")

    def _find_nearest_edge(self, x, y):
        """Override: use pre-built index if available."""
        if hasattr(self, '_edge_index') and self._edge_index:
            min_dist = float('inf')
            best_edge = None
            best_lane = 0
            for px, py, edge_id, lane_idx in self._edge_index:
                d = (px - x) ** 2 + (py - y) ** 2
                if d < min_dist:
                    min_dist = d
                    best_edge = edge_id
                    best_lane = lane_idx
            return best_edge, best_lane
        return super()._find_nearest_edge(x, y)

    def _find_nearest_edges(self, x, y, n=5):
        """Return top-N nearest (edge_id, lane_idx, dist2) candidates."""
        if not (hasattr(self, '_edge_index') and self._edge_index):
            edge, lane = super()._find_nearest_edge(x, y)
            return [(edge, lane, 0.0)] if edge else []
        # Collect best match per edge (avoid duplicates from multiple shape points)
        best_per_edge = {}
        for px, py, edge_id, lane_idx in self._edge_index:
            d = (px - x) ** 2 + (py - y) ** 2
            if edge_id not in best_per_edge or d < best_per_edge[edge_id][1]:
                best_per_edge[edge_id] = (lane_idx, d)
        candidates = [(eid, li, d) for eid, (li, d) in best_per_edge.items()]
        candidates.sort(key=lambda c: c[2])
        return candidates[:n]

    # ------------------------------------------------------------------
    #  Single-vehicle injection (for batch placement)
    # ------------------------------------------------------------------
    _type_id_map = {
        'car':          'car_normal',
        'suv':          'suv_normal',
        'pickup':       'pickup_normal',
        'truck':        'truck_delivery',
        'semi':         'semi_truck',
        'bus':          'bus_transit',
        'bus_transit':  'bus_transit',
        'bus_school':   'bus_school',
        'motorcycle':   'motorcycle_normal',
    }

    def _get_spawn_time(self, veh_info):
        """Return spawn_time from the optional 5th tuple element (default 0)."""
        if isinstance(veh_info, (tuple, list)) and len(veh_info) > 4:
            return float(veh_info[4])
        if isinstance(veh_info, dict):
            return float(veh_info.get('spawn_time', 0))
        return 0.0

    def _inject_single_vehicle(self, veh_idx, release=False):
        """Inject one manual vehicle by index into the running simulation.

        Tuple format: (type, lat, lon, dest_edge[, spawn_time])
        Places vehicle at (lat, lon) via moveToXY, then sets dest_edge
        as the destination. SUMO handles all routing automatically.
        If release=True, vehicle starts moving immediately (no freeze).
        """
        veh_info = self.initial_vehicles[veh_idx]
        veh_id = f'manual_veh_{veh_idx}'

        # --- Parse tuple/list: (type, lat, lon, dest_edge[, spawn_time]) ---
        if isinstance(veh_info, (tuple, list)):
            veh_type     = veh_info[0]
            lat          = veh_info[1]
            lon          = veh_info[2]
            dest_edge    = veh_info[3] if len(veh_info) > 3 else None
        else:
            veh_type     = veh_info.get('type', 'car')
            lat          = veh_info['lat']
            lon          = veh_info['lon']
            dest_edge    = veh_info.get('dest_edge', None)

        sumo_type_id = self._type_id_map.get(veh_type, veh_type)

        try:
            x, y = self.latlon_to_xy(lat, lon)

            edge_id, lane_idx = self._find_nearest_edge(x, y)
            if not edge_id:
                print(f"  [{veh_idx}] {veh_type}: no nearby edge, skipped")
                return

            angle = -1001
            try:
                angle = traci.lane.getAngle(f"{edge_id}_{lane_idx}")
            except Exception:
                pass

            # Build FULL route from start edge to dest_edge (same as main.py)
            # This way SUMO knows the destination from birth, and
            # device.rerouting will reroute TO dest_edge, not to start edge.
            route_edges = [edge_id]
            if dest_edge and dest_edge != edge_id:
                result = traci.simulation.findRoute(edge_id, dest_edge)
                if result.edges:
                    route_edges = list(result.edges)
                else:
                    print(f"    findRoute({edge_id}→{dest_edge}) failed, single-edge route")

            route_id = f'manual_route_{veh_idx}'
            traci.route.add(route_id, route_edges)

            traci.vehicle.add(
                vehID=veh_id,
                routeID=route_id,
                typeID=sumo_type_id,
                depart='now',
                departLane='best',
                departPos='base',
                departSpeed='0'
            )

            # keepRoute=1: stay on the route we just built
            traci.vehicle.moveToXY(
                vehID=veh_id,
                edgeID=edge_id,
                laneIndex=lane_idx,
                x=x, y=y,
                angle=angle,
                keepRoute=1
            )

            if release:
                traci.vehicle.setSpeedMode(veh_id, 31)
                traci.vehicle.setSpeed(veh_id, -1)
                traci.vehicle.setLaneChangeMode(veh_id, 1621)
            else:
                traci.vehicle.setSpeedMode(veh_id, 0)
                traci.vehicle.setSpeed(veh_id, 0)
                traci.vehicle.setLaneChangeMode(veh_id, 0)

            self.manual_vehicle_ids.append(veh_id)
            self._pending_destinations[veh_id] = dest_edge
            print(f"  [{veh_idx}] {veh_type}: placed at ({lat:.6f}, {lon:.6f}) → dest={dest_edge} ({len(route_edges)} edges)")

        except Exception as e:
            print(f"  [{veh_idx}] {veh_type}: injection failed - {e}")

    # ------------------------------------------------------------------
    #  Camera animation
    # ------------------------------------------------------------------
    #  Window size for rotation margin
    # ------------------------------------------------------------------
    def _compute_window_size(self):
        """Compute SUMO window dimensions large enough for rotation + crop.

        The window must be large enough so that after rotating the frame by
        rotate_angle, a center crop of output_w × output_h is fully covered
        (no black corners).
        """
        theta = math.radians(abs(self.rotate_angle))
        cos_t = abs(math.cos(theta))
        sin_t = abs(math.sin(theta))

        # Minimum source dimensions for clean crop after rotation
        min_w = self.output_w * cos_t + self.output_h * sin_t
        min_h = self.output_w * sin_t + self.output_h * cos_t

        # Add 15% margin and round up to even
        win_w = int(math.ceil(min_w * 1.15))
        win_h = int(math.ceil(min_h * 1.15))
        win_w += win_w % 2
        win_h += win_h % 2

        print(f"    Window size: {win_w}x{win_h} "
              f"(output {self.output_w}x{self.output_h}, rotate {self.rotate_angle}°)")
        return f'{win_w},{win_h}'

    # ------------------------------------------------------------------
    #  Camera animation
    # ------------------------------------------------------------------
    def smooth_camera_move(self, from_x, from_y, from_zoom,
                           to_x, to_y, to_zoom, duration):
        """Smoothly animate the GUI camera with an ease-in-out curve.

        Uses cosine easing: slow start -> fast middle -> slow end.
        Calls simulationStep() each frame so the GUI re-renders.
        """
        fps = 20
        n_frames = max(int(duration * fps), 1)
        sleep_per_frame = duration / n_frames

        for i in range(n_frames + 1):
            t = i / n_frames
            eased = 0.5 * (1 - math.cos(math.pi * t))

            x = from_x + (to_x - from_x) * eased
            y = from_y + (to_y - from_y) * eased
            zoom = from_zoom + (to_zoom - from_zoom) * eased

            traci.gui.setOffset("View #0", x, y)
            traci.gui.setZoom("View #0", zoom)
            traci.simulationStep()
            time.sleep(sleep_per_frame)

    # ------------------------------------------------------------------
    #  Route file helpers
    # ------------------------------------------------------------------
    def _create_flowless_route_file(self):
        """Get or create a route file with only types and route definitions (no flows/trips).

        The file is saved next to the original route file as *_flowless.rou.xml
        and reused on subsequent runs if the source hasn't changed.
        """
        base, ext = os.path.splitext(self.route_file)
        flowless_path = f"{base}_flowless{ext}"

        # Reuse if flowless file is newer than the source route file
        if (os.path.exists(flowless_path)
                and os.path.getmtime(flowless_path) >= os.path.getmtime(self.route_file)):
            print(f"    Reusing flowless route file: {flowless_path}")
            self._flowless_route_file = flowless_path
            return flowless_path

        tree = ET.parse(self.route_file)
        root = tree.getroot()

        new_root = ET.Element("routes")
        for key, val in root.attrib.items():
            new_root.set(key, val)

        for child in root:
            if child.tag in ('route', 'vType', 'vTypeDistribution'):
                new_root.append(child)

        new_tree = ET.ElementTree(new_root)
        ET.indent(new_tree, space="    ")
        new_tree.write(flowless_path, xml_declaration=True, encoding="UTF-8")

        self._flowless_route_file = flowless_path
        print(f"    Created flowless route file: {flowless_path}")
        return flowless_path

    def _parse_flow_definitions(self):
        """Parse flow definitions from the original route file.

        Returns list of dicts with keys:
            id, route, vehsPerHour, departLane, type, interval, counter, next_spawn
        """
        tree = ET.parse(self.route_file)
        root = tree.getroot()

        flows = []
        for elem in root:
            if elem.tag == 'flow':
                vph = int(elem.get('vehsPerHour', '0'))
                if vph <= 0:
                    continue
                interval = 3600.0 / vph
                flows.append({
                    'id': elem.get('id'),
                    'route': elem.get('route'),
                    'vehsPerHour': vph,
                    'departLane': elem.get('departLane', 'best'),
                    'type': elem.get('type', 'realistic_traffic_mix'),
                    'interval': interval,
                    'counter': 0,
                    'next_spawn': 0.0,
                })

        print(f"    Parsed {len(flows)} flow definitions "
              f"(total {sum(f['vehsPerHour'] for f in flows)} veh/hr)")
        return flows

    # ------------------------------------------------------------------
    #  Background vehicle management
    # ------------------------------------------------------------------
    def _discover_downtown_edges(self):
        """Find all passenger-allowed edges within the downtown bounding box.

        Uses TraCI to query the live network — works with any merged network.
        Returns list of (edge_id, n_lanes, length) tuples.
        """
        lon_min, lat_min, lon_max, lat_max = self.bg_bounds

        # Convert bounding box corners to SUMO x,y
        x_min, y_min = traci.simulation.convertGeo(lon_min, lat_min, fromGeo=True)
        x_max, y_max = traci.simulation.convertGeo(lon_max, lat_max, fromGeo=True)
        # Ensure min < max
        x_lo, x_hi = min(x_min, x_max), max(x_min, x_max)
        y_lo, y_hi = min(y_min, y_max), max(y_min, y_max)

        edges = []
        for edge_id in traci.edge.getIDList():
            if edge_id.startswith(':'):
                continue
            n_lanes = traci.edge.getLaneNumber(edge_id)
            if n_lanes == 0:
                continue

            lane0 = f"{edge_id}_0"
            try:
                length = traci.lane.getLength(lane0)
            except Exception:
                continue
            if length < 20:
                continue

            # Check if edge midpoint falls within bounding box
            shape = traci.lane.getShape(lane0)
            mid = shape[len(shape) // 2]
            if x_lo <= mid[0] <= x_hi and y_lo <= mid[1] <= y_hi:
                # Check passenger access
                disallowed = traci.lane.getDisallowed(lane0)
                if 'passenger' not in disallowed:
                    edges.append((edge_id, n_lanes, length))

        print(f"    Discovered {len(edges)} downtown edges "
              f"(bounds: {lon_min},{lat_min} to {lon_max},{lat_max})")
        return edges

    def _place_background_vehicles(self):
        """Place background vehicles across downtown area edges.

        Discovers edges within bg_bounds at runtime, distributes
        bg_vehicle_count vehicles proportionally by edge capacity,
        and routes each to a random other downtown edge.
        Returns list of placed vehicle IDs.
        """
        if self.bg_vehicle_count <= 0:
            print("    No background vehicles requested")
            return []

        downtown_edges = self._discover_downtown_edges()
        if not downtown_edges:
            print("    No edges found in downtown bounds")
            return []

        # Exclude edges within 100m of stalled vehicle (intersection)
        if self.obstacles:
            obs_lat, obs_lon = self.obstacles[0][0], self.obstacles[0][1]
            obs_x, obs_y = self.latlon_to_xy(obs_lat, obs_lon)
            filtered = []
            for edge_id, n_lanes, length in downtown_edges:
                try:
                    shape = traci.lane.getShape(f"{edge_id}_0")
                    mid = shape[len(shape) // 2]
                    dist = math.sqrt((mid[0] - obs_x)**2 + (mid[1] - obs_y)**2)
                    if dist >= 100.0:
                        filtered.append((edge_id, n_lanes, length))
                except Exception:
                    pass
            excluded = len(downtown_edges) - len(filtered)
            print(f"    Excluded {excluded} edges within 100m of stalled vehicle")
            downtown_edges = filtered

        if not downtown_edges:
            print("    No edges remaining after exclusion zone")
            return []

        rng = random.Random(self.bg_seed)

        # Compute capacity weight for each edge
        edges_with_weight = []
        for edge_id, n_lanes, length in downtown_edges:
            weight = n_lanes * length
            edges_with_weight.append((edge_id, n_lanes, length, weight))

        total_weight = sum(w for _, _, _, w in edges_with_weight)

        # Distribute vehicles proportionally to capacity
        edge_vehicle_counts = []
        for edge_id, n_lanes, length, weight in edges_with_weight:
            count = max(1, round(self.bg_vehicle_count * weight / total_weight))
            edge_vehicle_counts.append((edge_id, n_lanes, length, count))

        # Adjust to hit target count
        total_assigned = sum(c for _, _, _, c in edge_vehicle_counts)
        if total_assigned > self.bg_vehicle_count:
            sorted_by_weight = sorted(range(len(edge_vehicle_counts)),
                                      key=lambda i: edges_with_weight[i][3])
            for idx in sorted_by_weight:
                if total_assigned <= self.bg_vehicle_count:
                    break
                e, n, l, c = edge_vehicle_counts[idx]
                if c > 1:
                    edge_vehicle_counts[idx] = (e, n, l, c - 1)
                    total_assigned -= 1

        # Collect all edge IDs for random destination selection
        all_edge_ids = [e for e, _, _ in downtown_edges]

        bg_ids = []
        veh_counter = 0
        for edge_id, n_lanes, length, count in edge_vehicle_counts:
            other_edges = [e for e in all_edge_ids if e != edge_id]

            for j in range(count):
                veh_id = f"bg_veh_{veh_counter}"
                veh_counter += 1

                # Position: evenly spaced along the edge
                pos = (j + 1) / (count + 1) * length
                pos = max(5.0, min(pos, length - 5.0))
                lane_idx = j % n_lanes

                # Random destination in downtown area
                dest_edge = rng.choice(other_edges)

                try:
                    result = traci.simulation.findRoute(edge_id, dest_edge)
                    route_edges = list(result.edges) if result.edges else [edge_id]

                    route_id = f"bg_route_{veh_counter}"
                    traci.route.add(route_id, route_edges)
                    traci.vehicle.add(
                        vehID=veh_id,
                        routeID=route_id,
                        typeID='realistic_traffic_mix',
                        depart='now',
                        departLane=lane_idx,
                        departPos=str(pos),
                        departSpeed='max'
                    )
                    # Restrict lane changing: only allow mandatory (route/safety),
                    # disable strategic and cooperative changes to prevent chaos.
                    # Bits: strategic=0, cooperative=0, speedGain=0, keepRight=0,
                    #        mandatory-right=1, mandatory-left=1 → 0b0000_0101_0000 = 80
                    # Using 512+256=768 keeps only TraCI + mandatory change bits
                    traci.vehicle.setLaneChangeMode(veh_id, 512)
                    # speedMode 23 = 10111: respect safe speed, max accel/decel,
                    # brake for red lights, but IGNORE right-of-way at junctions
                    traci.vehicle.setSpeedMode(veh_id, 23)
                    bg_ids.append(veh_id)
                except Exception:
                    pass

        print(f"    Placed {len(bg_ids)} / {veh_counter} background vehicles "
              f"across {len(edge_vehicle_counts)} downtown edges")
        return bg_ids

    def _check_conflicts(self, bg_ids):
        """Remove background vehicles too close to initial/obstacle vehicles.

        Returns list of surviving background vehicle IDs.
        """
        # Collect positions of protected vehicles
        protected_positions = []

        if hasattr(self, 'obstacle_vehicles'):
            for obs in self.obstacle_vehicles:
                protected_positions.append((obs['x'], obs['y']))

        for veh_id in getattr(self, 'manual_vehicle_ids', []):
            try:
                pos = traci.vehicle.getPosition(veh_id)
                protected_positions.append(pos)
            except Exception:
                pass

        if not protected_positions:
            return bg_ids

        surviving = []
        removed = 0
        for veh_id in bg_ids:
            try:
                bx, by = traci.vehicle.getPosition(veh_id)
                conflict = False
                for px, py in protected_positions:
                    dist = math.sqrt((bx - px) ** 2 + (by - py) ** 2)
                    if dist < self.conflict_distance:
                        conflict = True
                        break

                if conflict:
                    traci.vehicle.remove(veh_id)
                    removed += 1
                else:
                    surviving.append(veh_id)
            except Exception:
                pass

        if removed:
            print(f"    Conflict check: removed {removed} bg vehicles (< {self.conflict_distance}m)")
        return surviving

    def _remove_background_vehicles(self, bg_ids, keep_center=None, keep_radius=300.0):
        """Remove background vehicles, optionally keeping those near intersection.

        Args:
            bg_ids: list of bg vehicle IDs
            keep_center: (x, y) tuple — center of the keep zone (intersection)
            keep_radius: vehicles within this distance (m) from center are kept

        Returns:
            list of vehicle IDs that were kept (empty if keep_center is None)
        """
        removed = 0
        kept = []
        for veh_id in bg_ids:
            try:
                if keep_center is not None:
                    vx, vy = traci.vehicle.getPosition(veh_id)
                    dist = math.sqrt((vx - keep_center[0]) ** 2 + (vy - keep_center[1]) ** 2)
                    if dist <= keep_radius:
                        kept.append(veh_id)
                        continue
                traci.vehicle.remove(veh_id)
                removed += 1
            except Exception:
                pass
        print(f"    Removed {removed} bg vehicles, kept {len(kept)} near intersection (<{keep_radius}m)")
        return kept

    # ------------------------------------------------------------------
    #  Flow spawner
    # ------------------------------------------------------------------
    def _spawn_flow_vehicles(self, flow_defs, elapsed):
        """Spawn vehicles according to flow definitions.

        Args:
            flow_defs: list of flow definition dicts (mutated in place)
            elapsed: seconds elapsed since flow start
        """
        for flow in flow_defs:
            while flow['next_spawn'] <= elapsed:
                veh_id = f"{flow['id']}_{flow['counter']}"
                try:
                    traci.vehicle.add(
                        vehID=veh_id,
                        routeID=flow['route'],
                        typeID=flow['type'],
                        depart='now',
                        departLane=flow['departLane'],
                        departSpeed='max'
                    )
                except Exception:
                    pass  # edge may be full, skip
                flow['counter'] += 1
                flow['next_spawn'] += flow['interval']

    # ------------------------------------------------------------------
    #  Screenshot helper
    # ------------------------------------------------------------------
    # Vehicles to remove as soon as they appear
    _remove_on_sight = ('bg_veh_2315', 'bg_veh_2317')
    # Vehicles to redirect to a different destination
    _redirect_targets = {
        'bg_veh_2474': '417034059#0',
        'bg_veh_2477': '417034059#0',
        'bg_veh_2475': '417034059#0',
        'bg_veh_2485': '417034059#0',
        'bg_veh_94': '417034218#1',
        'bg_veh_2313': '417034218#1',
        'bg_veh_2314': '417034218#1',
        'bg_veh_894': '417034218#1',
        'bg_veh_2316': '417034218#1',
    }

    def _step_and_capture(self):
        """Advance simulation one step and save screenshot if enabled."""
        traci.simulationStep()
        # Remove blacklisted vehicles immediately
        active = traci.vehicle.getIDList()
        for _vid in self._remove_on_sight:
            if _vid in active:
                try:
                    traci.vehicle.remove(_vid)
                except Exception:
                    pass
        # Redirect vehicles to specified destinations
        for _vid, _dest in self._redirect_targets.items():
            if _vid in active:
                try:
                    if traci.vehicle.getRoute(_vid)[-1] != _dest:
                        traci.vehicle.changeTarget(_vid, _dest)
                except Exception:
                    pass
        if self.screenshot_dir and self._capture_enabled:
            path = os.path.join(self._screenshot_subdir,
                                f"frame_{self._frame_counter:06d}.png")
            traci.gui.screenshot("View #0", path)
            self._frame_angles.append(self._current_rotation)
            self._frame_counter += 1

    def _capture_only(self):
        """Save screenshot at current camera position WITHOUT advancing simulation.

        SUMO-GUI needs simulationStep() to trigger a GUI repaint, so we
        still call it.  But because vehicles are frozen (speed=0) or this
        is called between real sim-advance frames, the visual state only
        reflects the camera change.
        """
        traci.simulationStep()
        if self.screenshot_dir and self._capture_enabled:
            path = os.path.join(self._screenshot_subdir,
                                f"frame_{self._frame_counter:06d}.png")
            traci.gui.screenshot("View #0", path)
            self._frame_angles.append(self._current_rotation)
            self._frame_counter += 1

    # ------------------------------------------------------------------
    #  Video composition
    # ------------------------------------------------------------------
    def _compose_video(self):
        """Compose video from screenshots after simulation ends.

        With step-length ≈ 1/60s, every frame is a real simulation screenshot
        at native 60fps. No frame interpolation needed — straight encode.

        Rotation is applied in post-processing (not in SUMO) to avoid
        SUMO's edge culling artifacts when the view is rotated.
        Each frame is rotated by its recorded angle, then the center
        1920x1080 is cropped from the 3840x2160 source.

        Output: <screenshot_dir>/<timestamp>.mp4
        """
        from PIL import Image

        frame_dir = self._screenshot_subdir
        video_output = os.path.join(self.screenshot_dir, f"{self._timestamp}.mp4")

        # Count actual frames on disk (screenshot is async; last frames may not flush)
        total = 0
        while os.path.exists(os.path.join(frame_dir, f"frame_{total:06d}.png")):
            total += 1
        if total == 0:
            print("  No frames to compose")
            return
        if total < self._frame_counter:
            print(f"  Note: {self._frame_counter - total} trailing frames not flushed by SUMO")

        fps = round(1.0 / self.step_length)
        duration = total / fps
        print(f"\n  Composing video ({total} frames @ {fps}fps = {duration:.1f}s) ...")

        # Truncate angle list to match actual frame count
        angles = self._frame_angles[:total]

        # Check if any frame needs rotation
        needs_rotation = any(abs(a) > 0.01 for a in angles)

        out_w, out_h = self.output_w, self.output_h

        if needs_rotation:
            # Post-process: rotate each frame, crop largest safe 16:9 area,
            # then resize to output_w × output_h
            rotated_dir = os.path.join(frame_dir, 'rotated')
            os.makedirs(rotated_dir, exist_ok=True)
            aspect = out_w / out_h  # 16:9

            # Read first frame for source dimensions
            first = Image.open(os.path.join(frame_dir, 'frame_000000.png'))
            src_w, src_h = first.size
            first.close()

            # Compute the MINIMUM safe 16:9 crop across ALL rotation angles.
            # This ensures a fixed field of view — no zoom jumps when rotation
            # starts or changes.  The worst case is at θ = atan(1/aspect).
            min_safe_h = float('inf')
            for angle in angles:
                theta = math.radians(abs(angle))
                cos_t = abs(math.cos(theta))
                sin_t = abs(math.sin(theta))
                h_w = src_w / (aspect * cos_t + sin_t)
                h_h = src_h / (aspect * sin_t + cos_t)
                min_safe_h = min(min_safe_h, min(h_w, h_h))

            fixed_h = int(min_safe_h)
            fixed_w = int(aspect * fixed_h)
            # Ensure even dimensions
            fixed_w -= fixed_w % 2
            fixed_h -= fixed_h % 2

            print(f"    Fixed crop: {fixed_w}x{fixed_h} (uniform across all angles)")
            print(f"    Rotating {total} frames → {out_w}x{out_h} (post-processing)...")
            for i in range(total):
                src = os.path.join(frame_dir, f"frame_{i:06d}.png")
                dst = os.path.join(rotated_dir, f"frame_{i:06d}.png")
                angle = angles[i] if i < len(angles) else 0.0
                img = Image.open(src)

                # Rotate (no-op when angle ≈ 0)
                if abs(angle) >= 0.01:
                    img = img.rotate(angle, resample=Image.BICUBIC, expand=False)

                # Fixed-size center crop (same for all frames)
                cx, cy = img.width // 2, img.height // 2
                box = (cx - fixed_w // 2, cy - fixed_h // 2,
                       cx + fixed_w // 2, cy + fixed_h // 2)
                result = img.crop(box)

                # Resize to target output
                if result.size != (out_w, out_h):
                    result = result.resize((out_w, out_h), Image.LANCZOS)
                result.save(dst)

                if (i + 1) % 500 == 0:
                    print(f"      {i + 1}/{total} frames rotated")
            print(f"    Rotation complete")
            input_pattern = os.path.join(rotated_dir, 'frame_%06d.png')
            vf_filter = None  # already cropped + resized
        else:
            # No rotation — crop center to output size via ffmpeg
            # Read first frame to get source dimensions
            first = Image.open(os.path.join(frame_dir, 'frame_000000.png'))
            src_w, src_h = first.size
            first.close()
            crop_x = (src_w - out_w) // 2
            crop_y = (src_h - out_h) // 2
            input_pattern = os.path.join(frame_dir, 'frame_%06d.png')
            vf_filter = f'crop={out_w}:{out_h}:{crop_x}:{crop_y}'

        cmd = [
            'ffmpeg', '-y',
            '-framerate', str(fps),
            '-i', input_pattern,
        ]
        if vf_filter:
            cmd += ['-vf', vf_filter]
        cmd += [
            '-c:v', 'libx264', '-pix_fmt', 'yuv420p',
            '-crf', '18', '-preset', 'medium',
            '-tune', 'fastdecode',
            '-bf', '0', '-refs', '3',
            '-movflags', '+faststart',
            video_output
        ]
        subprocess.run(cmd, check=True)

        print(f"  Video: {video_output}")

    # ------------------------------------------------------------------
    #  Main simulation flow (overrides base class)
    # ------------------------------------------------------------------
    def run_simulation(self):
        """Run the demo recording simulation flow (8-phase workflow)."""

        # Initialize screenshot capture with timestamped subdirectory
        self._frame_counter = 0
        self._frame_angles = []
        self._current_rotation = 0.0  # post-processing rotation angle (degrees)
        self._capture_enabled = False  # start disabled; enabled after pre-roll
        self._timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        if self.screenshot_dir:
            self._screenshot_subdir = os.path.join(self.screenshot_dir, self._timestamp)
            os.makedirs(self._screenshot_subdir, exist_ok=True)
            print(f"  Screenshots enabled: {self._screenshot_subdir}")

        print(f"\n{'='*60}")
        print(f"  DEMO Recording Mode (new workflow)")
        print(f"  Bg vehicles: {self.bg_vehicle_count}  |  Main: {self.sim_time}s")
        print(f"  Freeze: {self.freeze_duration}s  |  Zoom-in: {self.zoom_in_duration}s")
        print(f"{'='*60}")

        # ==============================================================
        #  Phase 1 — Start: launch SUMO with no flows
        # ==============================================================
        print("\n  Phase 1: Starting SUMO (no flows)")

        flowless_route = self._create_flowless_route_file()

        # SUMO --end: stop at sim_time (absolute sim clock)
        total_end = self.sim_time + 10  # small buffer

        sumo_cmd = [
            self.sumo_binary,
            '-c', self.config_file,
            '--route-files', flowless_route,  # override: types + routes only
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

            # Set TLS program immediately so offset takes effect from t=0
            self.set_tls_program_via_traci()
            self.update_tls_program()

            # Initialize: one sim step so GUI is ready
            self._step_and_capture()

            # Set camera to full network overview, shifted slightly downward
            (min_x, min_y), (max_x, max_y) = traci.simulation.getNetBoundary()
            overview_x = (min_x + max_x) / 2
            overview_y = (min_y + max_y) / 2 - (max_y - min_y) * 0.15
            traci.gui.setOffset("View #0", overview_x, overview_y)
            traci.gui.setZoom("View #0", self.zoom_out_level)
            # Force multiple render frames to lock in the view
            for _ in range(3):
                traci.gui.setOffset("View #0", overview_x, overview_y)
                traci.gui.setZoom("View #0", self.zoom_out_level)
                self._step_and_capture()

            # ==============================================================
            #  Phase 2 — Place vehicles
            # ==============================================================
            print("\n  Phase 2: Placing vehicles")

            # Build spatial index for fast edge lookup
            self._preload_edge_positions()

            # Add stalled vehicle (obstacle) first
            self.add_obstacles_via_traci()
            self._step_and_capture()
            self.update_obstacle_positions()

            # Add initial vehicles (all frozen at speed=0, skip delayed ones)
            self.manual_vehicle_ids = []
            self._pending_destinations = {}
            self._delayed_vehicle_indices = []
            n_manual = len(self.initial_vehicles) if self.initial_vehicles else 0
            for veh_idx in range(n_manual):
                spawn_t = self._get_spawn_time(self.initial_vehicles[veh_idx])
                if spawn_t > 0:
                    self._delayed_vehicle_indices.append(veh_idx)
                    continue
                self._inject_single_vehicle(veh_idx)
            self._step_and_capture()  # materialize moveToXY positions

            # Verify routes after materialization
            print("  === Phase 2: Vehicle routes ===")
            for veh_id in self.manual_vehicle_ids:
                try:
                    route = traci.vehicle.getRoute(veh_id)
                    edge = traci.vehicle.getRoadID(veh_id)
                    print(f"    {veh_id}: on={edge}, route={len(route)} edges, dest={route[-1]}")
                except Exception as e:
                    print(f"    {veh_id}: error - {e}")
            self._pending_destinations = {}
            print(f"    Placed {len(self.manual_vehicle_ids)} initial vehicles (frozen)")

            # Add pre-defined background vehicles
            bg_ids = self._place_background_vehicles()

            # ==============================================================
            #  Phase 3 — Conflict check
            # ==============================================================
            print("\n  Phase 3: Conflict check")
            self._step_and_capture()  # materialize bg vehicles
            bg_ids = self._check_conflicts(bg_ids)
            n_veh = len(traci.vehicle.getIDList())
            print(f"    Total vehicles on network: {n_veh}")

            # ==============================================================
            #  Phase 4 — Pre-roll + Two-stage zoom-in
            #
            #  Pre-roll (5s):  bg vehicles move, manual vehicles frozen, camera static
            #  Stage A:  zoom overview → downtown, bg moving, manual frozen
            #  Dwell:    hold at downtown, bg moving, manual frozen
            #  Stage B:  release all vehicles + E-W green, zoom downtown → close-up
            #            simulation runs continuously (no pause after zoom)
            # ==============================================================

            if self.obstacles:
                obs_lat, obs_lon = self.obstacles[0][0], self.obstacles[0][1]
                target_x, target_y = self.latlon_to_xy(obs_lat, obs_lon)
            else:
                target_x, target_y = self.latlon_to_xy(37.3354, -121.8921)

            # Center camera on intersection at starting zoom
            traci.gui.setOffset("View #0", target_x, target_y)
            traci.gui.setZoom("View #0", self.zoom_out_level)

            # --- Pre-roll: 5s of simulation, bg vehicles moving, manual frozen ---
            pre_roll_duration = 5.0
            pre_roll_frames = max(int(pre_roll_duration / self.step_length), 1)
            print(f"\n  Phase 4: Pre-roll {pre_roll_duration}s ({pre_roll_frames} steps), "
                  f"bg moving, manual frozen")
            for _ in range(pre_roll_frames):
                self._step_and_capture()
                if not self.screenshot_dir:
                    time.sleep(self.zoom_step_delay)

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
            print(f"    Cancelled {cancelled} pending bg vehicles, {len(bg_ids)} active")

            # --- Frame allocation ---
            fps = 60
            total_frames = max(int(self.zoom_in_duration * fps), 2)
            log_range_a = math.log(self.zoom_mid_level) - math.log(self.zoom_out_level)
            log_range_b = math.log(self.zoom_final_level) - math.log(self.zoom_mid_level)
            log_total = log_range_a + log_range_b
            a_fraction = log_range_a / log_total
            stage_a_frames = max(int(total_frames * a_fraction), 1)
            stage_b_frames = total_frames - stage_a_frames

            print(f"    Stage A: zoom {self.zoom_out_level} -> {self.zoom_mid_level} "
                  f"({stage_a_frames} frames, bg moving, manual frozen)")
            print(f"    Dwell:   {self.zoom_dwell_duration}s")
            print(f"    Stage B: zoom {self.zoom_mid_level} -> {self.zoom_final_level} "
                  f"({stage_b_frames} frames, all vehicles released)")

            # --- Stage A: zoom-in, bg moving, manual frozen ---
            log_start_a = math.log(self.zoom_out_level)
            log_end_a = math.log(self.zoom_mid_level)
            for i in range(stage_a_frames + 1):
                t = i / stage_a_frames
                eased = 0.5 * (1.0 - math.cos(math.pi * t))
                zoom = math.exp(log_start_a + (log_end_a - log_start_a) * eased)

                traci.gui.setOffset("View #0", target_x, target_y)
                traci.gui.setZoom("View #0", zoom)
                self._step_and_capture()
                if not self.screenshot_dir:
                    time.sleep(self.zoom_step_delay)

            # Enable screenshot capture from dwell stage
            self._capture_enabled = True
            print(f"    Screenshot capture enabled (starting from dwell)")

            # --- Dwell: hold camera, bg still moving, manual still frozen ---
            print(f"    Dwelling {self.zoom_dwell_duration}s ...")
            dwell_frames = max(int(self.zoom_dwell_duration * fps), 1)
            for _ in range(dwell_frames):
                self._step_and_capture()
                if not self.screenshot_dir:
                    time.sleep(self.zoom_step_delay)

            print(f"    Stage B: zoom {self.zoom_mid_level} -> {self.zoom_final_level} "
                  f"({stage_b_frames} frames, all vehicles released)")

            # --- Stage B: release all vehicles, E-W green starts ---
            # Release manual vehicles
            active_ids = traci.vehicle.getIDList()
            released_b = 0
            for veh_id in active_ids:
                if veh_id.startswith('obstacle_veh_'):
                    continue
                if veh_id.startswith('manual_veh_'):
                    try:
                        lane_id = traci.vehicle.getLaneID(veh_id)
                        lane_pos = traci.vehicle.getLanePosition(veh_id)
                        if lane_id and not lane_id.startswith(':'):
                            traci.vehicle.moveTo(veh_id, lane_id, lane_pos)
                        traci.vehicle.setLaneChangeMode(veh_id, 1621)
                        traci.vehicle.setSpeedMode(veh_id, 31)
                        traci.vehicle.setSpeed(veh_id, -1)
                        released_b += 1
                    except Exception:
                        pass

            # Lock manual_veh_5 on lane 1 for 20s (disable lane changing)
            try:
                if 'manual_veh_5' in traci.vehicle.getIDList():
                    traci.vehicle.setLaneChangeMode('manual_veh_5', 0)
                    traci.vehicle.changeLane('manual_veh_5', 1, 20.0)
            except Exception:
                pass

            _release_sim_t = traci.simulation.getTime()
            self._veh5_release_time = _release_sim_t
            print(f"    Stage B: released {released_b} manual vehicles at sim time {_release_sim_t:.1f}s")

            # Log-space interpolation for perceptually uniform zoom speed
            # Rotation is deferred to post-processing to avoid SUMO edge culling
            log_start_b = math.log(self.zoom_mid_level)
            log_end_b = math.log(self.zoom_final_level)
            angle_start_b = 0.0
            angle_end_b = self.rotate_angle  # counterclockwise
            for i in range(1, stage_b_frames + 1):
                t = i / stage_b_frames
                if t < 0.5:
                    eased = 4.0 * t * t * t
                else:
                    eased = 1.0 - (-2.0 * t + 2.0) ** 3 / 2.0

                zoom = math.exp(log_start_b + (log_end_b - log_start_b) * eased)
                self._current_rotation = angle_start_b + (angle_end_b - angle_start_b) * eased

                traci.gui.setOffset("View #0", target_x, target_y)
                traci.gui.setZoom("View #0", zoom)
                self._step_and_capture()

                if not self.screenshot_dir:
                    time.sleep(self.zoom_step_delay)

            print(f"    Zoom-in complete (final zoom: {self.zoom_final_level})")

            # Remove bg vehicles outside current viewport (expanded for rotation)
            try:
                (vp_left, vp_bottom), (vp_right, vp_top) = traci.gui.getBoundary("View #0")
                # Expand boundary by 50% to compensate for rotation
                w = (vp_right - vp_left) * 0.5
                h = (vp_top - vp_bottom) * 0.5
                vp_left -= w
                vp_right += w
                vp_bottom -= h
                vp_top += h
                removed_bg = 0
                kept_bg_ids = []
                for veh_id in bg_ids:
                    try:
                        vx, vy = traci.vehicle.getPosition(veh_id)
                        if vp_left <= vx <= vp_right and vp_bottom <= vy <= vp_top:
                            kept_bg_ids.append(veh_id)
                        else:
                            traci.vehicle.remove(veh_id)
                            removed_bg += 1
                    except Exception:
                        pass
                print(f"    Removed {removed_bg} bg vehicles outside viewport, kept {len(kept_bg_ids)}")
            except Exception:
                pass  # viewport query failed, skip cleanup

            # ==============================================================
            #  Phase 6 — Start flows
            # ==============================================================
            print(f"\n  Phase 6: Starting flows")

            flow_defs = self._parse_flow_definitions()
            flow_start_sim_time = traci.simulation.getTime()
            print(f"    Flow start at sim time: {flow_start_sim_time:.1f}s")

            # ==============================================================
            #  Phase 7 — Main simulation loop
            # ==============================================================
            print(f"\n  Phase 7: Running main simulation ({self.sim_time}s)")

            if self.screenshot_dir:
                print(f"    Main-loop starts at frame {self._frame_counter}")

            step = 0
            while traci.simulation.getTime() < self.sim_time:
                self._step_and_capture()

                # Spawn flow vehicles
                elapsed = traci.simulation.getTime() - flow_start_sim_time
                self._spawn_flow_vehicles(flow_defs, elapsed)

                # Restore manual_veh_5 lane changing after 20s
                try:
                    if (traci.simulation.getTime() - self._veh5_release_time >= 20.0
                            and 'manual_veh_5' in traci.vehicle.getIDList()):
                        traci.vehicle.setLaneChangeMode('manual_veh_5', 1621)
                except Exception:
                    pass

                # bg_veh_287 gets stuck on -416901218#1 lane 0 — move to lane 1
                try:
                    if 'bg_veh_287' in traci.vehicle.getIDList():
                        if traci.vehicle.getLaneID('bg_veh_287') == '-416901218#1_0':
                            traci.vehicle.changeLane('bg_veh_287', 1, 5.0)
                except Exception:
                    pass
                # bg_veh_88 needs to turn left but ends up in right lane — force to lane 1
                try:
                    if 'bg_veh_88' in traci.vehicle.getIDList():
                        if traci.vehicle.getLaneID('bg_veh_88') == '-416901209#1_0':
                            traci.vehicle.changeLane('bg_veh_88', 1, 5.0)
                except Exception:
                    pass

                # Inject delayed vehicles when their spawn_time is reached
                sim_t_now = traci.simulation.getTime()
                still_pending = []
                for _di in self._delayed_vehicle_indices:
                    _spawn_t = self._get_spawn_time(self.initial_vehicles[_di])
                    if sim_t_now >= _spawn_t:
                        self._inject_single_vehicle(_di, release=True)
                    else:
                        still_pending.append(_di)
                self._delayed_vehicle_indices = still_pending

                # Dump all vehicle positions at sim_time ~12s (once)
                sim_t = traci.simulation.getTime()
                if not hasattr(self, '_dumped_12s') and sim_t >= 12.0:
                    self._dumped_12s = True
                    dump_path = os.path.join(os.path.dirname(self.net_file), 'vehicle_snapshot_12s.json')
                    veh_list = []
                    for vid in traci.vehicle.getIDList():
                        if vid.startswith('obstacle_veh_'):
                            continue
                        x, y = traci.vehicle.getPosition(vid)
                        lon, lat = traci.simulation.convertGeo(x, y)
                        edge = traci.vehicle.getRoadID(vid)
                        lane = traci.vehicle.getLaneID(vid)
                        vtype = traci.vehicle.getTypeID(vid)
                        speed = traci.vehicle.getSpeed(vid)
                        veh_list.append({
                            'id': vid, 'lat': lat, 'lon': lon,
                            'edge': edge, 'lane': lane,
                            'type': vtype, 'speed': round(speed, 2)
                        })
                    import json
                    with open(dump_path, 'w') as f:
                        json.dump(veh_list, f, indent=2)
                    print(f"\n  === Snapshot at {sim_t:.1f}s: {len(veh_list)} vehicles → {dump_path} ===")

                self.update_obstacle_positions()
                self.update_tls_program()
                self.trigger_rerouting(step * self.step_length)
                self.assist_stuck_vehicles(step * self.step_length)
                self.remove_stuck_vehicles(step * self.step_length)
                self.collect_vehicle_data(step * self.step_length)

                # 60fps real-time pacing (skip in recording mode)
                if not self.screenshot_dir:
                    time.sleep(self.zoom_step_delay)

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

    def run(self):
        """Override parent run() — skip delay calculation for demo."""
        success = self.run_simulation()
        return success


# ======================================================================
#  CLI
# ======================================================================
def main():
    parser = argparse.ArgumentParser(
        description='SUMO Demo Video Recording Simulation',
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    # --- standard args (same defaults as main.py) ---
    parser.add_argument('--net-file',
                        default="san_jose_downtown_gtc/osm.net.xml",
                        help='SUMO network file')
    parser.add_argument('--route-file',
                        default="san_jose_downtown_gtc/osm.rou.xml",
                        help='Route file (source for types, routes, and flow definitions)')
    parser.add_argument('--obstacles', default=None,
                        help='Obstacle definition: "lat,lon[,w,h,angle];..."')
    parser.add_argument('--initial-stop-duration', type=float, default=10,
                        help='Seconds manual vehicles stay stopped (default 10)')
    parser.add_argument('--tls-program', default=None,
                        help='Custom TLS program (JSON file/string)')
    parser.add_argument('--sim-time', type=int, default=1800,
                        help='Main simulation duration in seconds (default 1800)')
    parser.add_argument('--step-length', type=float, default=0.01667,
                        help='Simulation step length (default 0.01667 ≈ 1/60s)')
    parser.add_argument('--gui', action='store_true', default=True)
    parser.add_argument('--no-gui', dest='gui', action='store_false')
    parser.add_argument('--mode', choices=['bench', 'opt', 'dynamic'],
                        default='dynamic')
    parser.add_argument('--program-id', default=None,
                        help='Explicit TLS program ID')
    parser.add_argument('--config', default=None,
                        help='SUMO .sumocfg file')
    parser.add_argument('--output',
                        default="traffic_data_analysis/delay_result/delay_demo.json")
    parser.add_argument('--tripinfo-output', default=None)
    parser.add_argument('--statistic-output', default=None)

    # --- demo config file ---
    parser.add_argument('--demo-config', default='demo_config.json',
                        help='JSON file with demo camera/zoom/bg parameters (default: demo_config.json)')
    parser.add_argument('--screenshot-dir', default=None,
                        help='Override screenshot_dir from demo config')
    parser.add_argument('--bg-vehicle-count', type=int, default=None,
                        help='Override bg_vehicle_count from demo config')

    args = parser.parse_args()

    if not os.path.exists(args.net_file):
        print(f"Error: Network file not found: {args.net_file}")
        sys.exit(1)
    if not os.path.exists(args.route_file):
        print(f"Error: Route file not found: {args.route_file}")
        sys.exit(1)

    obstacles = parse_obstacles(args.obstacles)

    # Load demo config from JSON
    demo_config = {}
    if args.demo_config and os.path.exists(args.demo_config):
        with open(args.demo_config, 'r') as f:
            demo_config = json.load(f)
        print(f"  Loaded demo config: {args.demo_config}")
    # CLI overrides
    if args.bg_vehicle_count is not None:
        demo_config['bg_vehicle_count'] = args.bg_vehicle_count
    if args.screenshot_dir is not None:
        demo_config['screenshot_dir'] = args.screenshot_dir

    runner = SUMODemoRunner(
        demo_config=demo_config,
        screenshot_dir=demo_config.get('screenshot_dir'),
        # base params
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
            # ('car',         37.335734, -121.891560, '417034224#0'),

            ('car',         37.335661, -121.891772, '417034224#0'),
            ('car',         37.335696, -121.891706, '157782193#2'),
            ('car',         37.335717, -121.891656, '417034224#0'),
            # ('car',         37.335772, -121.891494, '417034224#0'),

            ('car',         37.335843, -121.891390, '417034224#0'),
            ('car',         37.335864, -121.891343, '417034224#0'),
            ('car',         37.335885, -121.891303, '417034224#0'),


            # --- Vehicles east of intersection ---
            ('car',         37.335663, -121.891608, '416909351#1'),
            ('car',         37.335615, -121.891647, '416909351#1'),

            # --- EB approach ---
            # ('car',         37.335368, -121.892231, '-416901218#1'),
            # ('car',         37.335343, -121.892282, '-416901218#1'),
            # ('car',         37.335315, -121.892341, '-416901218#1'),
            # ('car',         37.335292, -121.892388, '-416901218#1'),
            # ('car',         37.335352, -121.892209, '-416901218#1'),

            # --- SB approach ---
            ('car',         37.335589, -121.892206, '495569632'),
            ('car',         37.335633, -121.892238, '495569632'),
            ('car',         37.335603, -121.892183, '157781953#2'),

            # --- NB approach ---
            ('car',         37.335352, -121.891944, '157782193#2'),

            # --- Bus ---
            ('bus_transit',  37.335601, -121.891898, '417034224#0'),
            ('bus_transit',  37.335782, -121.891526, '417034224#0'),

            # --- Delayed vehicles on -417034180 (spawn at 24s) ---
            ('car',         37.335238, -121.892497, '-416901218#1', 24.0),
            ('car',         37.335225, -121.892477, '-416901218#1', 24.0),
            ('car',         37.335211, -121.892557, '-416901218#1', 24.0),
            ('car',         37.335198, -121.892528, '-416901218#1', 24.0),

        ],
        initial_stop_duration=args.initial_stop_duration,
    )

    results = runner.run()

    if results:
        print("\n  Demo completed!")
        sys.exit(0)
    else:
        print("\n  Demo failed!")
        sys.exit(1)


if __name__ == '__main__':
    main()
