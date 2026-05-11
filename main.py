#!/usr/bin/env python3
"""
SUMO Traffic Simulation Delay Calculator with TraCI
Single-file program using TraCI for dynamic simulation control to calculate average delay in specified traffic scenarios

Features:
- Input obstacle positions using latitude/longitude coordinates
- Obstacles implemented as stationary vehicles with realistic vehicle response
- Automatic lane angle following or manual specification
- Support for dynamic traffic light program modification
- Real-time delay statistics collection

Usage:
python sumo_delay_calculator.py \
    --net-file san_jose_downtown_gtc/osm.net.xml \
    --route-file san_jose_downtown_gtc/osm.passenger.trips.xml \
    --obstacles "37.335265,-121.892334" \
    --gui

"""

import sys
import os
import argparse
import json
import time
import tempfile
import xml.etree.ElementTree as ET
from xml.dom import minidom
import math

try:
    import traci
except ImportError:
    print("Error: Unable to import traci module")
    print("Please ensure SUMO is properly installed and SUMO_HOME environment variable is set")
    print("and SUMO tools directory is in Python path")
    sys.exit(1)

# Make TeraSim packages importable using a path relative to this file
_HERE = os.path.dirname(os.path.abspath(__file__))
_TERASIM_ROOT = os.path.abspath(os.path.join(_HERE, "..", "TeraSim"))
for _pkg in ("packages/terasim", "packages/terasim-nde-nade"):
    _p = os.path.join(_TERASIM_ROOT, _pkg)
    if _p not in sys.path:
        sys.path.insert(0, _p)

try:
    from terasim_nde_nade.adversity.static.stalled_object import StalledObjectAdversity
except ImportError as _e:
    print(f"Error: Unable to import StalledObjectAdversity from terasim_nde_nade: {_e}")
    print(f"Expected TeraSim at: {_TERASIM_ROOT}")
    sys.exit(1)


class SUMODelayCalculator:
    """SUMO Delay Calculator - Using TraCI for dynamic control"""

    def __init__(self, net_file, route_file, obstacles=None, tls_program=None,
                 sim_time=3600, step_length=0.1, gui=False, output_file=None, mode='static',
                 tripinfo_file=None, statistic_file=None, program_id=None):
        """
        Initialize calculator

        Args:
            net_file: SUMO network file path
            route_file: Route file path
            obstacles: Obstacle list [(lat, lon, width, height, angle), ...] angle=None means auto-follow road
            tls_program: Custom traffic light program (JSON format or file path)
            sim_time: Simulation duration (seconds)
            step_length: Simulation step length (seconds)
            gui: Whether to use GUI mode
            output_file: Output file path
            tripinfo_file: SUMO tripinfo XML output file path
            statistic_file: SUMO overall statistic XML output file path
        """
        self.net_file = os.path.abspath(net_file)
        self.route_file = os.path.abspath(route_file)
        self.obstacles = obstacles or []
        self.tls_program = tls_program
        self.sim_time = sim_time
        self.step_length = step_length
        self.gui = gui
        self.output_file = output_file
        self.mode = mode
        self.tripinfo_file = tripinfo_file
        self.statistic_file = statistic_file
        self.program_id = program_id
        self.initial_vehicles = []
        self.applied_tls_program = None

        self.sumo_binary = 'sumo-gui' if self.gui else 'sumo'

        # Temporary configuration file
        self.temp_dir = tempfile.mkdtemp(prefix='sumo_sim_')
        self.config_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "san_jose_downtown_gtc", "osm.sumocfg")

        # Statistical data
        self.vehicle_data = {}
        self.departed_vehicles = set()
        self.arrived_vehicles = set()

        # Single source of truth for "which IDs are obstacles" — populated by
        # add_obstacles_via_traci() and consulted by the per-step filters.
        self.obstacle_ids = set()

        # Load network projection information
        self._load_network_projection()

    def _load_network_projection(self):
        """Load projection information from network file"""
        try:
            tree = ET.parse(self.net_file)
            root = tree.getroot()
            location = root.find('location')

            if location is not None:
                # Get netOffset
                net_offset = location.get('netOffset', '0,0').split(',')
                self.net_offset_x = float(net_offset[0])
                self.net_offset_y = float(net_offset[1])

                # Get original boundary
                orig_boundary = location.get('origBoundary', '0,0,0,0').split(',')
                self.orig_lon_min = float(orig_boundary[0])
                self.orig_lat_min = float(orig_boundary[1])
                self.orig_lon_max = float(orig_boundary[2])
                self.orig_lat_max = float(orig_boundary[3])

                print(f"✓ Network projection information loaded:")
                print(f"  netOffset: ({self.net_offset_x}, {self.net_offset_y})")
                print(f"  Original boundary: lon({self.orig_lon_min}, {self.orig_lon_max}), "
                      f"lat({self.orig_lat_min}, {self.orig_lat_max})")
            else:
                print("Warning: Projection information not found in network file, using default values")
                self.net_offset_x = 0
                self.net_offset_y = 0

        except Exception as e:
            print(f"Warning: Failed to load projection information: {e}")
            self.net_offset_x = 0
            self.net_offset_y = 0

    def latlon_to_xy(self, lat, lon):
        """
        Convert latitude/longitude to SUMO coordinates (Mercator projection)

        Args:
            lat: Latitude
            lon: Longitude

        Returns:
            (x, y): SUMO coordinates
        """
        x_sumo, y_sumo = traci.simulation.convertGeo(lon, lat, fromGeo=True)
        return x_sumo, y_sumo

    def create_config_file(self):
        """Create basic SUMO configuration file"""
        config = ET.Element('configuration')

        # Input
        input_elem = ET.SubElement(config, 'input')
        ET.SubElement(input_elem, 'net-file').set('value', self.net_file)
        ET.SubElement(input_elem, 'route-files').set('value', self.route_file)

        # Time
        time_elem = ET.SubElement(config, 'time')
        ET.SubElement(time_elem, 'begin').set('value', '0')
        ET.SubElement(time_elem, 'end').set('value', str(self.sim_time))
        ET.SubElement(time_elem, 'step-length').set('value', '0.1')  # Higher precision

        # Processing
        processing_elem = ET.SubElement(config, 'processing')
        ET.SubElement(processing_elem, 'time-to-teleport').set('value', '-1')
        ET.SubElement(processing_elem, 'lateral-resolution').set('value', '0.4')

        # Report
        report_elem = ET.SubElement(config, 'report')
        ET.SubElement(report_elem, 'verbose').set('value', 'false')
        ET.SubElement(report_elem, 'no-step-log').set('value', 'true')

        # Routing
        routing_elem = ET.SubElement(config, 'routing')
        ET.SubElement(routing_elem, 'device.rerouting.probability').set('value', '1.0')
        ET.SubElement(routing_elem, 'device.rerouting.period').set('value', '30')
        ET.SubElement(routing_elem, 'device.rerouting.pre-period').set('value', '0')
        ET.SubElement(routing_elem, 'device.rerouting.adaptation-steps').set('value', '18')
        ET.SubElement(routing_elem, 'device.rerouting.adaptation-interval').set('value', '10')
        ET.SubElement(routing_elem, 'device.rerouting.with-taz').set('value', 'false')

        # Format and output
        xml_str = minidom.parseString(ET.tostring(config)).toprettyxml(indent="    ")
        with open(self.config_file, 'w') as f:
            f.write(xml_str)

        print(f"✓ Configuration file created: {self.config_file}")

    def _is_vehicle_lane(self, lane_id):
        """Return True if the lane permits passenger cars (i.e. not a sidewalk or bike-only path)."""
        try:
            disallowed = set(traci.lane.getDisallowed(lane_id))
            allowed = set(traci.lane.getAllowed(lane_id))
        except Exception:
            return False
        if "passenger" in disallowed:
            return False
        # Empty `allowed` means "all classes allowed except those in disallow".
        if allowed and "passenger" not in allowed:
            return False
        return True

    def _snap_gps_to_vehicle_lane(self, lat, lon):
        """Snap a GPS position to the nearest lane that actually allows passenger cars.

        SUMO's convertRoad / _find_nearest_edge can return a pedestrian-only sidewalk
        lane when the GPS lands near the curb (which is exactly where stalled vehicles
        end up in reality). Placing an obstacle on a sidewalk lane silently breaks the
        downstream "vehicles stuck behind obstacle" logic because no real cars share
        that lane.

        Returns (lane_id, lane_position) or (None, None) if no usable lane is found.
        """
        x, y = self.latlon_to_xy(lat, lon)
        try:
            edge_id, lane_pos, lane_idx = traci.simulation.convertRoad(x, y, isGeo=False)
        except Exception as e:
            print(f"    convertRoad failed: {e}")
            return None, None
        if not edge_id or edge_id.startswith(':'):
            return None, None

        candidate = f"{edge_id}_{lane_idx}"
        if self._is_vehicle_lane(candidate):
            return candidate, lane_pos

        # The nearest lane is a sidewalk/bike path. Walk the edge's other lanes and
        # pick the nearest vehicle lane. Lanes on the same edge are parallel, so the
        # same lane_pos is a fine projection.
        print(f"    Lane {candidate} is non-vehicle (sidewalk/bike), searching siblings...")
        try:
            n_lanes = traci.edge.getLaneNumber(edge_id)
        except Exception:
            return None, None
        for li in range(n_lanes):
            sibling = f"{edge_id}_{li}"
            if sibling == candidate:
                continue
            if self._is_vehicle_lane(sibling):
                print(f"      → snapped to {sibling}")
                return sibling, lane_pos
        return None, None

    def add_obstacles_via_traci(self):
        """Add obstacles via TeraSim's StalledObjectAdversity in lane_position mode.

        We pre-snap the LinkVision GPS to the nearest actual vehicle lane (filtering
        out pedestrian/bike-only lanes), then hand the lane_id + position to
        StalledObjectAdversity. This is cleaner than latlon_degree mode because:
          - lane_index in obstacle_info reflects the *real* lane the obstacle blocks
          - assist_stuck_vehicles / update_tls_program key off the correct lane
          - no angle math needed — SUMO orients along the lane shape automatically
        """
        if not self.obstacles:
            return

        print(f"\nAdding obstacles via StalledObjectAdversity (total: {len(self.obstacles)}):")

        self.obstacle_advs = []
        # Kept for compatibility with assist_stuck_vehicles() which reads
        # obstacle['id'/'edge'/'lane'] from this list.
        self.obstacle_vehicles = []

        for idx, (lat, lon, width, height, angle) in enumerate(self.obstacles):
            try:
                print(f"\n  Obstacle {idx}:")
                print(f"    Lat/Lon: ({lat:.6f}, {lon:.6f})")

                lane_id, lane_pos = self._snap_gps_to_vehicle_lane(lat, lon)
                if lane_id is None:
                    print(f"    ✗ No usable vehicle lane found near this GPS, skipped")
                    continue
                print(f"    Snapped to {lane_id} at pos={lane_pos:.2f}m "
                      f"(allowed: passenger cars OK)")

                adv = StalledObjectAdversity(
                    placement_mode="lane_position",
                    lane_id=lane_id,
                    lane_position=lane_pos,
                    object_type="DEFAULT_VEHTYPE",
                    start_time=0,
                    end_time=-1,  # never auto-remove
                )
                if not adv.is_effective():
                    print(f"    ✗ StalledObjectAdversity rejected the config, skipped")
                    continue

                adv.initialize(time=0)

                lane_idx_int = int(adv.lane_index) if str(adv.lane_index).isdigit() else 0

                # StalledObjectAdversity (lane_position mode) does NOT call setStop.
                # When the obstacle is near the end of its single-edge route — which
                # is exactly the case when LinkVision detects a car right at the
                # intersection entry — SUMO will mark the route complete on the next
                # step and auto-remove the vehicle. Pin it with a long-duration stop
                # at the end of the lane, mirroring what the pre-TeraSim hand-rolled
                # code did, so the vehicle stays put for the whole sim.
                try:
                    lane_length = traci.lane.getLength(lane_id)
                    traci.vehicle.setStop(
                        adv.stalled_object_id,
                        edgeID=adv.edge_id,
                        pos=lane_length,
                        laneIndex=lane_idx_int,
                        duration=2**31 - 1,
                    )
                except Exception as e:
                    print(f"    ! setStop failed (vehicle may be auto-removed later): {e}")

                # Color the spawned vehicle red so it's visually identifiable
                try:
                    traci.vehicle.setColor(adv.stalled_object_id, (255, 0, 0, 255))
                except Exception:
                    pass

                self.obstacle_advs.append(adv)
                self.obstacle_vehicles.append({
                    'id':    adv.stalled_object_id,
                    'edge':  adv.edge_id,
                    'lane':  lane_idx_int,
                })
                self.obstacle_ids.add(adv.stalled_object_id)

                print(f"    ✓ Obstacle vehicle placed: id={adv.stalled_object_id}, "
                      f"edge={adv.edge_id}, lane={lane_idx_int}")

            except Exception as e:
                print(f"  ✗ Failed to add obstacle {idx}: {e}")
                import traceback
                traceback.print_exc()

    # ------------------------------------------------------------------
    #  Initial vehicle injection
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

    def inject_manual_vehicles(self):
        """Inject initial vehicles into the simulation at specified GPS positions."""
        if not self.initial_vehicles:
            return
        self.manual_vehicle_ids = []
        self._pending_destinations = {}
        print(f"\n  Injecting {len(self.initial_vehicles)} initial vehicles...")
        for veh_idx in range(len(self.initial_vehicles)):
            self._inject_single_vehicle(veh_idx)
        traci.simulationStep()  # materialize
        print(f"    Placed {len(self.manual_vehicle_ids)} initial vehicles")

    def _inject_single_vehicle(self, veh_idx):
        """Inject one manual vehicle by index. Tuple: (type, lat, lon, dest_edge)"""
        veh_info = self.initial_vehicles[veh_idx]
        veh_id = f'manual_veh_{veh_idx}'

        if isinstance(veh_info, (tuple, list)):
            veh_type  = veh_info[0]
            lat       = veh_info[1]
            lon       = veh_info[2]
            dest_edge = veh_info[3] if len(veh_info) > 3 else None
        else:
            veh_type  = veh_info.get('type', 'car')
            lat       = veh_info['lat']
            lon       = veh_info['lon']
            dest_edge = veh_info.get('dest_edge', None)

        sumo_type_id = self._type_id_map.get(veh_type, veh_type)

        try:
            x, y = self.latlon_to_xy(lat, lon)

            # Use convertRoad to snap to nearest lane center line
            edge_id, lane_pos, lane_idx = traci.simulation.convertRoad(x, y)
            if not edge_id or edge_id.startswith(':'):
                # convertRoad returned internal edge, fall back to _find_nearest_edge
                edge_id, lane_idx = self._find_nearest_edge(x, y)
                lane_pos = None
                if not edge_id:
                    print(f"  [{veh_idx}] {veh_type}: no nearby edge, skipped")
                    return

            route_edges = [edge_id]
            if dest_edge and dest_edge != edge_id:
                result = traci.simulation.findRoute(edge_id, dest_edge)
                if result.edges:
                    route_edges = list(result.edges)

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

            # moveTo places vehicle exactly on lane center line
            lane_id = f"{edge_id}_{lane_idx}"
            if lane_pos is not None:
                traci.vehicle.moveTo(veh_id, lane_id, lane_pos)
            else:
                traci.vehicle.moveToXY(
                    vehID=veh_id, edgeID=edge_id, laneIndex=lane_idx,
                    x=x, y=y, angle=-1001, keepRoute=1
                )

            self.manual_vehicle_ids.append(veh_id)
            self._pending_destinations[veh_id] = dest_edge
            print(f"  [{veh_idx}] {veh_type}: placed at ({lat:.6f}, {lon:.6f}) → dest={dest_edge}")

        except Exception as e:
            print(f"  [{veh_idx}] {veh_type}: injection failed - {e}")

    def _find_nearest_edge(self, x, y):
        """Find nearest edge and lane"""
        try:
            # Get all edges
            edges = traci.edge.getIDList()
            min_dist = float('inf')
            nearest_edge = None
            nearest_lane = 0

            for edge_id in edges:
                try:
                    # Get all lanes of the edge
                    lane_count = traci.edge.getLaneNumber(edge_id)

                    for lane_idx in range(lane_count):
                        lane_id = f"{edge_id}_{lane_idx}"

                        # Get lane shape
                        shape = traci.lane.getShape(lane_id)

                        # Calculate minimum distance to lane
                        for px, py in shape:
                            dist = math.sqrt((px - x)**2 + (py - y)**2)
                            if dist < min_dist:
                                min_dist = dist
                                nearest_edge = edge_id
                                nearest_lane = lane_idx
                except:
                    continue

            return nearest_edge, nearest_lane

        except Exception as e:
            print(f"Error finding nearest edge: {e}")
            return None, 0

    def update_obstacle_positions(self):
        """Re-pin obstacles each step via StalledObjectAdversity.update().

        Rebuilds self.obstacle_info on every call because update_tls_program()
        reads it to decide which TLS program to apply.
        """
        if not hasattr(self, 'obstacle_advs'):
            return

        current_time = traci.simulation.getTime()
        self.obstacle_info = []
        for adv in self.obstacle_advs:
            try:
                adv.update(time=current_time)
                if adv.stalled_object_id in traci.vehicle.getIDList():
                    self.obstacle_info.append({
                        'id':   adv.stalled_object_id,
                        'edge': adv.edge_id,
                        'lane': int(adv.lane_index) if str(adv.lane_index).isdigit() else 0,
                    })
            except Exception:
                pass

    def trigger_rerouting(self, current_time):
        """Proactively trigger rerouting for congested vehicles"""
        if not hasattr(self, 'last_reroute_check'):
            self.last_reroute_check = 0
            self.reroute_count = 0

        # Check every 30 seconds
        if current_time - self.last_reroute_check < 30:
            return

        self.last_reroute_check = current_time
        period_count = 0
        vehicle_ids = traci.vehicle.getIDList()

        for veh_id in vehicle_ids:
            # Skip obstacle vehicles
            if veh_id in self.obstacle_ids:
                continue

            try:
                # Get vehicle waiting time
                waiting_time = traci.vehicle.getAccumulatedWaitingTime(veh_id)
                speed = traci.vehicle.getSpeed(veh_id)

                # If vehicle waiting time exceeds 30 seconds and speed is very slow, trigger rerouting
                if waiting_time > 30 and speed < 1.0:
                    # Recalculate path using current network weights
                    traci.vehicle.rerouteTraveltime(veh_id)
                    self.reroute_count += 1
                    period_count += 1
            except Exception:
                pass

        if period_count > 0:
            print(f"\n  Triggered {period_count} reroutes (total: {self.reroute_count})")

    def _get_through_lanes(self, edge_id):
        """Get lane indices that have through-movement (straight) connections.

        Uses traci.lane.getLinks() to check each lane's outgoing connections.
        A lane is considered a through lane if any of its connections have
        direction 's' (straight).

        Returns:
            List of lane indices with straight connections, cached per edge.
        """
        if not hasattr(self, '_through_lanes_cache'):
            self._through_lanes_cache = {}

        if edge_id in self._through_lanes_cache:
            return self._through_lanes_cache[edge_id]

        through_lanes = []
        n_lanes = traci.edge.getLaneNumber(edge_id)
        for lane_idx in range(n_lanes):
            lane_id = f"{edge_id}_{lane_idx}"
            try:
                links = traci.lane.getLinks(lane_id)
                for link in links:
                    direction = link[6]  # (approachedLane, internal, prio, open, foe, state, dir, length)
                    if direction == 's':
                        through_lanes.append(lane_idx)
                        break
            except Exception:
                pass

        self._through_lanes_cache[edge_id] = through_lanes
        print(f"  [LaneInfo] Edge {edge_id}: through lanes = {through_lanes} (of {n_lanes} total)")
        return through_lanes

    def _find_target_through_lane(self, edge_id, current_lane):
        """Find the nearest through-movement lane different from current_lane.

        Returns:
            Target lane index, or None if no valid target exists.
        """
        through_lanes = self._get_through_lanes(edge_id)
        valid = [l for l in through_lanes if l != current_lane]
        if not valid:
            return None
        return min(valid, key=lambda l: abs(l - current_lane))

    def assist_stuck_vehicles(self, current_time):
        """Progressively help vehicles stuck behind obstacles to change lanes.

        Uses SUMO's sublane model parameters (lcPushy, lcAssertive, lcImpatience)
        to make stuck vehicles increasingly aggressive about lane changing.
        After 100s of waiting, forces a lane change by overriding safety checks.

        Lane change targets are restricted to through-movement lanes only
        (determined from lane connection directions) to prevent vehicles from
        ending up in turn-only lanes.
        """
        if not hasattr(self, 'obstacle_vehicles') or not self.obstacle_vehicles:
            return

        if not hasattr(self, '_stuck_timers'):
            self._stuck_timers = {}       # veh_id -> first_stuck_time
            self._lc_force_count = 0

        # Build a lookup: lane_id -> [(obstacle_lane_position, obs_info), ...]
        obstacle_lanes = {}
        veh_id_list = traci.vehicle.getIDList()
        for obs in self.obstacle_vehicles:
            if obs['id'] not in veh_id_list:
                continue
            lane_id = f"{obs['edge']}_{obs['lane']}"
            try:
                obs_pos = traci.vehicle.getLanePosition(obs['id'])
            except Exception:
                continue
            obstacle_lanes.setdefault(lane_id, []).append((obs_pos, obs))

        if not obstacle_lanes:
            return

        currently_stuck = set()

        for veh_id in veh_id_list:
            if veh_id in self.obstacle_ids:
                continue

            try:
                lane_id = traci.vehicle.getLaneID(veh_id)
                if lane_id not in obstacle_lanes:
                    continue

                veh_pos = traci.vehicle.getLanePosition(veh_id)
                speed = traci.vehicle.getSpeed(veh_id)

                # Check if vehicle is behind an obstacle and nearly stopped
                for obs_pos, obs_info in obstacle_lanes[lane_id]:
                    distance = obs_pos - veh_pos
                    if 0 < distance < 30 and speed < 1.0:
                        currently_stuck.add(veh_id)

                        if veh_id not in self._stuck_timers:
                            self._stuck_timers[veh_id] = current_time

                        wait = current_time - self._stuck_timers[veh_id]
                        cur_lane = traci.vehicle.getLaneIndex(veh_id)
                        edge_id = traci.vehicle.getRoadID(veh_id)
                        target = self._find_target_through_lane(edge_id, cur_lane)

                        if wait > 100:
                            # === Force lane change: override all safety ===
                            if target is not None:
                                traci.vehicle.setLaneChangeMode(veh_id, 0)
                                traci.vehicle.changeLane(veh_id, target, 15.0)
                            self._lc_force_count += 1
                        elif wait > 60:
                            # Very aggressive: high pushy + assertive
                            traci.vehicle.setParameter(veh_id, "laneChangeModel.lcPushy", "1.0")
                            traci.vehicle.setParameter(veh_id, "laneChangeModel.lcAssertive", "5.0")
                            traci.vehicle.setParameter(veh_id, "laneChangeModel.lcImpatience", "1.0")
                            if target is not None:
                                traci.vehicle.changeLane(veh_id, target, 5.0)
                        elif wait > 30:
                            # Moderately aggressive
                            traci.vehicle.setParameter(veh_id, "laneChangeModel.lcPushy", "0.5")
                            traci.vehicle.setParameter(veh_id, "laneChangeModel.lcAssertive", "3.0")
                            traci.vehicle.setParameter(veh_id, "laneChangeModel.lcImpatience", "0.5")
                            if target is not None:
                                traci.vehicle.changeLane(veh_id, target, 5.0)
                        break  # only match first obstacle on this lane
            except Exception:
                pass

        # Reset vehicles that are no longer stuck
        for veh_id in list(self._stuck_timers):
            if veh_id not in currently_stuck:
                try:
                    if veh_id in veh_id_list:
                        # Restore default lane change behavior
                        traci.vehicle.setLaneChangeMode(veh_id, 1621)
                        traci.vehicle.setParameter(veh_id, "laneChangeModel.lcPushy", "0")
                        traci.vehicle.setParameter(veh_id, "laneChangeModel.lcAssertive", "1")
                        traci.vehicle.setParameter(veh_id, "laneChangeModel.lcImpatience", "0")
                except Exception:
                    pass
                del self._stuck_timers[veh_id]

    def remove_stuck_vehicles(self, current_time, threshold=180):
        """Remove non-obstacle vehicles that have been waiting consecutively for too long.

        Args:
            current_time: Current simulation time in seconds.
            threshold: Consecutive waiting time (seconds) before removal. Default 180s.
        """
        if not hasattr(self, '_remove_count'):
            self._remove_count = 0

        vehicle_ids = traci.vehicle.getIDList()
        for veh_id in vehicle_ids:
            if veh_id in self.obstacle_ids:
                continue
            try:
                waiting_time = traci.vehicle.getWaitingTime(veh_id)
                if waiting_time >= threshold:
                    traci.vehicle.remove(veh_id, reason=2)  # 2 = REMOVE_TELEPORT
                    self._remove_count += 1
            except Exception:
                pass

    def _record_applied_tls(self, tls_id, program_id, reason):
        """Remember the program actually applied via setProgram so it can be saved to JSON."""
        self.applied_tls_program = {
            'tls_id': tls_id,
            'applied_program_id': program_id,
            'reason': reason,
        }

    def update_tls_program(self):
        """Update TLS programs based on simulation mode and obstacle status.

        - bench mode: always use original program "org"
        - opt mode: always use optimized program "opt"
        - dynamic mode with obstacles: switch to the signal program whose
          program ID matches the obstacle's lane_id
        - dynamic mode without obstacles: fall back to "org"
        """
        # Only execute the switch once since obstacles are static
        if hasattr(self, '_tls_program_applied'):
            return
        self._tls_program_applied = True

        # Target TLS ID (can be extended to a list in the future)
        target_tls_id = "cluster_1984576776_3478559735_3478559736_3537422682_#1more"

        # Bench mode -> always use original "org"
        if self.mode == 'bench':
            try:
                traci.trafficlight.setProgram(target_tls_id, "org")
                self._record_applied_tls(target_tls_id, "org", "bench_mode_default")
                print(f"\n  TLS {target_tls_id}: using original program 'org' (bench mode)")
            except Exception as e:
                print(f"\n  TLS {target_tls_id}: failed to set program - {e}")
            return

        # Opt mode -> always use optimized "opt"
        if self.mode == 'opt':
            try:
                traci.trafficlight.setProgram(target_tls_id, "opt")
                self._record_applied_tls(target_tls_id, "opt", "opt_mode_default")
                print(f"\n  TLS {target_tls_id}: using optimized program 'opt' (opt mode)")
            except Exception as e:
                print(f"\n  TLS {target_tls_id}: failed to set program - {e}")
            return

        # Dynamic mode: if no obstacle info, fall back to "org"
        if not hasattr(self, 'obstacle_info') or not self.obstacle_info:
            try:
                traci.trafficlight.setProgram(target_tls_id, "org")
                self._record_applied_tls(target_tls_id, "org", "dynamic_no_obstacle")
                print(f"\n  TLS {target_tls_id}: no obstacles, using default program 'org'")
            except Exception as e:
                print(f"\n  TLS {target_tls_id}: failed to set default program - {e}")
            return

        # Dynamic mode with obstacles: collect obstacle edge and lane info
        obstacle_lane_ids = []
        obstacle_edges = []
        for obs in self.obstacle_info:
            lane_id = f"{obs['edge']}_{obs['lane']}"
            obstacle_lane_ids.append(lane_id)
            obstacle_edges.append(obs['edge'])
        print(f"\n  [Dynamic] Obstacle edge(s): {obstacle_edges}")
        print(f"  [Dynamic] Obstacle lane ID(s): {obstacle_lane_ids}")

        # Get all available program IDs for this TLS
        try:
            all_logics = traci.trafficlight.getAllProgramLogics(target_tls_id)
            available_programs = [logic.programID for logic in all_logics]
            print(f"  [Dynamic] Available TLS programs: {available_programs}")
        except Exception as e:
            print(f"  [Dynamic] Failed to get TLS programs: {e}")
            return

        # Check if any obstacle lane is controlled by the target TLS
        try:
            controlled_lanes = traci.trafficlight.getControlledLanes(target_tls_id)
            matched_lane = None
            for lane_id in obstacle_lane_ids:
                if lane_id in controlled_lanes:
                    matched_lane = lane_id
                    break

            if matched_lane:
                print(f"  [Dynamic] Obstacle lane '{matched_lane}' is controlled by TLS")

                # Use the lane_id as program ID if it exists
                if matched_lane in available_programs:
                    traci.trafficlight.setProgram(target_tls_id, matched_lane)
                    self._record_applied_tls(target_tls_id, matched_lane, "exact_lane_program")
                    print(f"  TLS: switched to lane-specific program '{matched_lane}'")
                else:
                    # Fallback: find a program matching the obstacle's edge
                    obs_edge = obstacle_edges[0]
                    edge_program = None
                    for prog_id in available_programs:
                        if prog_id.startswith(obs_edge + "_"):
                            edge_program = prog_id
                            break
                    if edge_program:
                        traci.trafficlight.setProgram(target_tls_id, edge_program)
                        self._record_applied_tls(target_tls_id, edge_program, "edge_program_fallback")
                        print(f"  TLS: exact lane program not found, "
                              f"using edge-based program '{edge_program}'")
                    else:
                        traci.trafficlight.setProgram(target_tls_id, "opt")
                        self._record_applied_tls(target_tls_id, "opt", "opt_fallback")
                        print(f"  TLS: no lane-specific program for '{matched_lane}', "
                              f"falling back to 'opt'")
            else:
                traci.trafficlight.setProgram(target_tls_id, "org")
                self._record_applied_tls(target_tls_id, "org", "dynamic_obstacle_not_on_controlled_lane")
                print(f"  TLS: no obstacle on controlled lanes, using default 'org'")
        except Exception as e:
            print(f"\n  TLS {target_tls_id}: failed to switch program - {e}")
            import traceback
            traceback.print_exc()

    def set_tls_program_via_traci(self):
        """Set traffic light program via TraCI"""
        if not self.tls_program:
            print("\nUsing default network traffic light configuration")
            return

        print(f"\nSetting custom traffic light program:")

        # Parse TLS program
        tls_config = None
        if isinstance(self.tls_program, str):
            if os.path.exists(self.tls_program):
                with open(self.tls_program, 'r') as f:
                    tls_config = json.load(f)
            else:
                try:
                    tls_config = json.loads(self.tls_program)
                except:
                    print(f"  ✗ Unable to parse TLS program: {self.tls_program}")
                    return
        elif isinstance(self.tls_program, dict):
            tls_config = self.tls_program

        if not tls_config:
            return

        # Apply TLS configuration
        for tls_id, config in tls_config.items():
            try:
                # Get current traffic light logic
                current_logic = traci.trafficlight.getAllProgramLogics(tls_id)

                if not current_logic:
                    print(f"  ✗ Traffic light {tls_id} does not exist")
                    continue

                # Create new logic (based on first existing logic)
                logic = current_logic[0]

                # Update phases
                if 'phases' in config:
                    new_phases = []
                    for phase_config in config['phases']:
                        phase = traci.trafficlight.Phase(
                            duration=phase_config.get('duration', 30),
                            state=phase_config.get('state', logic.phases[0].state),
                            minDur=phase_config.get('minDur', 5),
                            maxDur=phase_config.get('maxDur', 50),
                            next=phase_config.get('next', ()),
                            name=phase_config.get('name', '')
                        )
                        new_phases.append(phase)

                    logic = traci.trafficlight.Logic(
                        programID=config.get('programID', logic.programID),
                        type=logic.type,
                        currentPhaseIndex=0,
                        phases=new_phases,
                        subParameter=logic.subParameter
                    )

                # Set new logic
                traci.trafficlight.setProgramLogic(tls_id, logic)
                traci.trafficlight.setProgram(tls_id, logic.programID)

                print(f"  ✓ Traffic light {tls_id}: Set {len(logic.phases)} phases")

            except Exception as e:
                print(f"  ✗ Failed to set traffic light {tls_id}: {e}")

    def collect_vehicle_data(self, step):
        """Collect vehicle data for current step"""
        # Get all vehicles
        vehicle_ids = traci.vehicle.getIDList()

        for veh_id in vehicle_ids:
            # Skip obstacle vehicles
            if veh_id in self.obstacle_ids:
                continue

            if veh_id not in self.vehicle_data:
                self.vehicle_data[veh_id] = {
                    'depart_time': step,
                    'total_time_loss': 0.0,
                    'total_waiting_time': 0.0,
                    'arrival_time': None
                }

            # Accumulate time loss and waiting time
            try:
                time_loss = traci.vehicle.getAccumulatedWaitingTime(veh_id)
                self.vehicle_data[veh_id]['total_waiting_time'] = time_loss
            except:
                pass

        # Check newly arrived vehicles
        arrived_ids = traci.simulation.getArrivedIDList()
        for veh_id in arrived_ids:
            if veh_id in self.vehicle_data and self.vehicle_data[veh_id]['arrival_time'] is None:
                self.vehicle_data[veh_id]['arrival_time'] = step
                self.arrived_vehicles.add(veh_id)

    def run_simulation(self):
        """Run SUMO simulation with TraCI control"""
        print(f"\n{'='*60}")
        print(f"Starting SUMO simulation (using TraCI)")
        print(f"  Network file: {self.net_file}")
        print(f"  Route file: {self.route_file}")
        print(f"  Simulation time: {self.sim_time} seconds")
        print(f"  Step length: {self.step_length} seconds")
        print(f"{'='*60}")

        try:
            # Start SUMO
            sumo_cmd = [
                self.sumo_binary,
                '-c', self.config_file,
                # '--start',  # Auto-start simulation (GUI mode) — paused for inspection
                '--quit-on-end',  # Auto-quit on end
                '--time-to-teleport', '-1',  # Disable teleportation so obstacles stay forever
                '--end', str(self.sim_time),  # Explicit end time
                '--step-length', str(self.step_length),  # Override config to match Python loop
                '--window-size', '1920,1440',
                '--delay', '40'
            ]

            if self.tripinfo_file:
                sumo_cmd += ['--tripinfo-output', os.path.abspath(self.tripinfo_file)]
            if self.statistic_file:
                sumo_cmd += ['--statistic-output', os.path.abspath(self.statistic_file)]

            traci.start(sumo_cmd, numRetries=10)
            print("✓ SUMO started\n")

            # Zoom/offset/angle handling intentionally left to the user.
            # Previous version forced a 60-degree rotation + fixed center which
            # caused a visible "flash" when the simulation started — preserve
            # whatever view the user has manually configured instead.

            # Add obstacles
            self.add_obstacles_via_traci()

            # Inject initial vehicles (calls simulationStep internally)
            self.inject_manual_vehicles()

            # Re-pin obstacle after inject's simulationStep
            self.update_obstacle_positions()

            # Set traffic light program
            self.set_tls_program_via_traci()

            # Run simulation
            print(f"\nRunning simulation...")
            step = 0
            total_steps = int(self.sim_time / self.step_length)
            freeze_step = int(0.3 / self.step_length)  # freeze after 0.3s to let positions settle

            while step < total_steps:
                traci.simulationStep()

                # Freeze at 0.3s so vehicles settle into correct positions
                if step == freeze_step:
                    print("  Freezing for 10 seconds...")
                    time.sleep(10)

                # Update obstacle positions (keep stationary)
                self.update_obstacle_positions()

                # update tls program based on mode and obstacle status
                self.update_tls_program()

                # Proactively trigger rerouting for congested vehicles
                self.trigger_rerouting(step * self.step_length)

                # Help vehicles stuck behind obstacles change lanes
                self.assist_stuck_vehicles(step * self.step_length)

                # Remove non-obstacle vehicles stuck for too long
                self.remove_stuck_vehicles(step * self.step_length)

                # Collect vehicle data
                self.collect_vehicle_data(step * self.step_length)

                step += 1

                # Progress output
                if step % 100 == 0 or step == total_steps:
                    progress = (step / total_steps) * 100
                    vehicle_count = len(traci.vehicle.getIDList())
                    print(f"  Progress: {progress:.1f}% (step: {step}/{total_steps}, "
                          f"current vehicles: {vehicle_count})", end='\r')

            print()  # Newline
            print("✓ Simulation completed")

            # Close TraCI connection
            traci.close()

            return True

        except Exception as e:
            print(f"\nError: Exception occurred while running SUMO: {e}")
            import traceback
            traceback.print_exc()

            try:
                traci.close()
            except:
                pass

            return False

    def calculate_delay(self):
        """Calculate average delay"""
        if not self.arrived_vehicles:
            print("Warning: No vehicles completed their trips")
            return {
                'average_delay': 0,
                'average_time_loss': 0,
                'average_wait_time': 0,
                'average_duration': 0,
                'vehicle_count': 0,
                'total_time_loss': 0,
                'total_wait_time': 0,
                'simulation_time': self.sim_time,
                'total_departed': len(self.vehicle_data),
                'total_arrived': 0
            }

        total_duration = 0.0
        total_time_loss = 0.0
        total_wait_time = 0.0

        for veh_id in self.arrived_vehicles:
            data = self.vehicle_data[veh_id]

            duration = data['arrival_time'] - data['depart_time']
            wait_time = data['total_waiting_time']

            total_duration += duration
            total_wait_time += wait_time

        vehicle_count = len(self.arrived_vehicles)

        # Calculate time loss (using waiting time as approximation)
        # In practice, time loss = actual_time - ideal_time
        total_time_loss = total_wait_time

        results = {
            'average_delay': total_time_loss / vehicle_count,
            'average_time_loss': total_time_loss / vehicle_count,
            'average_wait_time': total_wait_time / vehicle_count,
            'average_duration': total_duration / vehicle_count,
            'vehicle_count': vehicle_count,
            'total_time_loss': total_time_loss,
            'total_wait_time': total_wait_time,
            'simulation_time': self.sim_time,
            'total_departed': len(self.vehicle_data),
            'total_arrived': len(self.arrived_vehicles)
        }

        return results

    def print_results(self, results):
        """Print results"""
        if not results:
            return

        print(f"\n{'='*60}")
        print(f"Simulation Results Statistics")
        print(f"{'='*60}")
        print(f"Departed vehicles: {results['total_departed']}")
        print(f"Arrived vehicles: {results['total_arrived']}")
        print(f"Completion rate: {results['total_arrived']/max(results['total_departed'],1)*100:.1f}%")
        print(f"")
        print(f"Average trip duration: {results['average_duration']:.2f} seconds")
        print(f"Average delay time (timeLoss): {results['average_delay']:.2f} seconds")
        print(f"Average waiting time (waitingTime): {results['average_wait_time']:.2f} seconds")
        print(f"")
        print(f"Total delay time: {results['total_time_loss']:.2f} seconds")
        print(f"Total waiting time: {results['total_wait_time']:.2f} seconds")
        print(f"{'='*60}\n")

    def save_results(self, results):
        """Save results to file"""
        if not results or not self.output_file:
            return

        output_data = {
            'configuration': {
                'net_file': self.net_file,
                'route_file': self.route_file,
                'obstacles': [
                    {'x': x, 'y': y, 'width': w, 'height': h, 'angle': a}
                    for x, y, w, h, a in self.obstacles
                ],
                'tls_program': self.applied_tls_program if self.applied_tls_program else (
                    self.tls_program if isinstance(self.tls_program, (dict, str)) else None
                ),
                'simulation_time': self.sim_time,
                'step_length': self.step_length
            },
            'results': results
        }

        with open(self.output_file, 'w') as f:
            json.dump(output_data, f, indent=2, ensure_ascii=False)

        print(f"✓ Results saved to: {self.output_file}")

    def run(self):
        """Execute complete workflow"""
        try:
            # 1. Create configuration file
            # self.create_config_file()

            # 2. Run simulation (includes obstacle and traffic light setup)
            if not self.run_simulation():
                return None

            # 3. Calculate delay
            results = self.calculate_delay()

            # 4. Print results
            self.print_results(results)

            # 5. Save results
            self.save_results(results)

            return results

        except Exception as e:
            print(f"Error: {e}")
            import traceback
            traceback.print_exc()
            return None

        finally:
            # Clean up temporary files
            if not self.gui:
                import shutil
                try:
                    shutil.rmtree(self.temp_dir)
                except:
                    pass


def parse_obstacles(obstacle_str):
    """
    Parse obstacle string
    Format: "lat,lon[,width,height,angle];..."

    Parameters:
    - lat: Latitude (required)
    - lon: Longitude (required)
    - width: Width in meters (optional, default 0, currently unused)
    - height: Height in meters (optional, default 0, currently unused)
    - angle: Angle in degrees (optional, auto-follows lane angle if not provided)

    Examples:
    - "37.33251,-121.892360" - Only lat/lon provided, auto-follows lane angle
    - "37.33251,-121.892360,0,0,90" - Angle specified as 90 degrees
    - "37.33251,-121.892360,5,3,45;37.33252,-121.892370" - Multiple obstacles
    """
    if not obstacle_str:
        return []

    obstacles = []
    for obs_str in obstacle_str.split(';'):
        parts = obs_str.strip().split(',')

        if len(parts) < 2:
            print(f"Warning: Ignoring malformed obstacle (requires at least lat/lon): {obs_str}")
            continue

        try:
            lat = float(parts[0])
            lon = float(parts[1])
            width = float(parts[2]) if len(parts) > 2 else 0
            height = float(parts[3]) if len(parts) > 3 else 0
            angle = float(parts[4]) if len(parts) > 4 else None  # None means auto-follow

            obstacles.append((lat, lon, width, height, angle))
        except ValueError as e:
            print(f"Warning: Ignoring malformed obstacle: {obs_str} - {e}")

    return obstacles


def load_tls_program(tls_arg):
    """Load traffic light program"""
    if not tls_arg:
        return None

    # Check if it's a file path
    if os.path.exists(tls_arg):
        with open(tls_arg, 'r') as f:
            return json.load(f)

    # Try to parse as JSON string
    try:
        return json.loads(tls_arg)
    except:
        print(f"Warning: Unable to parse TLS program parameter: {tls_arg}")
        return None


def main():
    parser = argparse.ArgumentParser(
        description='SUMO Traffic Simulation Delay Calculator (TraCI Version)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Basic usage
  python sumo_delay_calculator.py \\
      --net-file san_jose_downtown_gtc/osm.net.xml \\
      --route-file san_jose_downtown_gtc/osm.passenger.trips.xml

  # Add obstacles (using lat/lon, auto-follows lane angle)
  python sumo_delay_calculator.py \\
      --net-file san_jose_downtown_gtc/osm.net.xml \\
      --route-file san_jose_downtown_gtc/osm.passenger.trips.xml \\
      --obstacles "37.33251,-121.892360"

  # Add multiple obstacles with specified angles
  python sumo_delay_calculator.py \\
      --net-file san_jose_downtown_gtc/osm.net.xml \\
      --route-file san_jose_downtown_gtc/osm.passenger.trips.xml \\
      --obstacles "37.33251,-121.892360,0,0,90;37.33252,-121.892370"

  # Use custom traffic light program
  python sumo_delay_calculator.py \\
      --net-file san_jose_downtown_gtc/osm.net.xml \\
      --route-file san_jose_downtown_gtc/osm.passenger.trips.xml \\
      --obstacles "37.33251,-121.892360" \\
      --tls-program tls_config_example.json \\
      --output results.json

  # Use GUI mode to observe obstacle effects
  python sumo_delay_calculator.py \\
      --net-file san_jose_downtown_gtc/osm.net.xml \\
      --route-file san_jose_downtown_gtc/osm.passenger.trips.xml \\
      --obstacles "37.33251,-121.892360" \\
      --gui

TLS Program JSON format example:
{
  "cluster_25977365_314061330": {
    "programID": "custom_program",
    "phases": [
      {
        "duration": 30,
        "state": "GGGrrr",
        "minDur": 10,
        "maxDur": 60
      },
      {
        "duration": 5,
        "state": "yyyrrr"
      },
      {
        "duration": 30,
        "state": "rrrGGG"
      }
    ]
  }
}
        """
    )

    obs_gps = "37.335351, -121.891935"
    output = "traffic_data_analysis/delay_result/delay_tmp.json"

    parser.add_argument('--net-file',
                        default="san_jose_downtown_gtc/osm.net.xml",
                       help='SUMO network file (.net.xml)')
    parser.add_argument('--route-file',
                        default="san_jose_downtown_gtc/directional_traffic.rou.xml",
                       help='Route file (.rou.xml or .trips.xml)')
    parser.add_argument('--obstacles', default=obs_gps,
                       help='Obstacle definition, format: "lat,lon[,width,height,angle];..." '
                            '(lat/lon required, width/height/angle optional, angle auto-follows lane if not provided)')
    parser.add_argument('--tls-program', default=None,
                       help='Custom traffic light program (JSON file path or JSON string)')
    parser.add_argument('--sim-time', type=int, default=1800,
                       help='Simulation duration (seconds), default 3600')
    parser.add_argument('--step-length', type=float, default=0.1,
                       help='Simulation step length (seconds), default 1.0')
    parser.add_argument('--gui', action='store_true', default=True,
                       help='Run in GUI mode')
    parser.add_argument('--no-gui', dest='gui', action='store_false',
                       help='Run in headless mode (no GUI)')
    parser.add_argument('--mode', choices=['bench', 'opt', 'dynamic'], default='dynamic',
                       help='Simulation mode: bench (original TLS "org"), opt (optimized TLS "opt"), or dynamic (obstacle-aware TLS switching)')
    parser.add_argument('--output', default=output,
                       help='Output JSON file path')
    parser.add_argument('--tripinfo-output', default=None,
                       help='SUMO tripinfo XML output file path')
    parser.add_argument('--statistic-output', default=None,
                       help='SUMO overall statistic XML output file path')
    parser.add_argument('--program-id', default=None,
                       help='Explicit TLS program ID to use')

    args = parser.parse_args()

    # Check if files exist
    if not os.path.exists(args.net_file):
        print(f"Error: Network file does not exist: {args.net_file}")
        sys.exit(1)

    if not os.path.exists(args.route_file):
        print(f"Error: Route file does not exist: {args.route_file}")
        sys.exit(1)

    # Parse obstacles
    obstacles = parse_obstacles(args.obstacles)

    # Load TLS program
    tls_program = load_tls_program(args.tls_program)

    # Create calculator and run
    calculator = SUMODelayCalculator(
        net_file=args.net_file,
        route_file=args.route_file,
        obstacles=obstacles,
        tls_program=tls_program,
        sim_time=args.sim_time,
        step_length=args.step_length,
        gui=args.gui,
        output_file=args.output,
        mode=args.mode,
        tripinfo_file=args.tripinfo_output,
        statistic_file=args.statistic_output,
        program_id=args.program_id
    )

    # Vehicle positions from 12s snapshot (main_demo.py simulation)
    calculator.initial_vehicles = [
            # --- In junction (transitioning through intersection) ---
            ('car_normal',           37.33556696, -121.89197582, '417034224#0'),  # manual_veh_0, junction

            # --- WB queue (San Fernando, heading west from east side) ---
            ('car_normal',           37.33597457, -121.89111547, '417034218#1'),  # bg_veh_2316, 416901218#1
            ('pickup_conservative',  37.33591348, -121.89119281, '417034218#1'),  # bg_veh_2317, junction
            # ('car_normal',           37.33590254, -121.89127401, '417034218#1'),  # bg_veh_2314, 517627277
            ('suv_normal',           37.33580404, -121.89142393, '417034218#1'),  # bg_veh_2315, 517627277
            ('car_normal',           37.33576603, -121.89150326, '417034224#0'),  # manual_veh_6, 517627277
            ('car_normal',           37.33573349, -121.89157228, '417034224#0'),  # manual_veh_5, junction
            ('car_normal',           37.33570413, -121.89163271, '417034224#0'),  # manual_veh_4, 1418903639#0
            ('car_normal',           37.33566663, -121.89171114, '417034224#0'),  # manual_veh_3, 1418903639#0
            ('car_normal',           37.33563373, -121.89177982, '417034224#0'),  # manual_veh_2, 1418903639#0
            ('bus_transit',          37.33565285, -121.89179467, '417034224#0'),  # manual_veh_13, 1418903639#0 (bus)
            ('car_normal',           37.33559809, -121.89185422, '417034224#0'),  # manual_veh_1, 1418903639#0
            ('car_normal',           37.335599,   -121.891926,   '417034224#0'),  # added

            # --- EB vehicles (San Fernando, heading east from west side) ---
            ('car_normal',           37.33567193, -121.89158943, '416909351#1'),  # manual_veh_7, -1418903639#0
            ('car_normal',           37.33562108, -121.89164041, '416909351#1'),  # manual_veh_8, -1418903639#0
            ('car_conservative',     37.33591818, -121.89100746, '-416901218#1'), # bg_veh_287, -416901218#1
            ('suv_conservative',     37.33600524, -121.89087750, '-416901218#1'), # bg_veh_291, -416901218#1
            ('car_conservative',     37.33511116, -121.89270949, '-416901218#1'), # bg_veh_382, -28463687#0
            ('car_normal',           37.335665,   -121.891609,   '416909351#1'),  # added
            ('car_normal',           37.335666,   -121.891531,   '416909351#1'),  # added

            # --- SB approach (4th St, heading south from north) ---
            ('car_conservative',     37.33587848, -121.89241835, '417034088#0'),  # bg_veh_95, -157782193#0
            ('car_normal',           37.335635, -121.892248,     '417034088#0'),  # bg_veh_88, -416901209#1
            ('pickup_normal',        37.335691, -121.892286,     '417034088#0'),  # bg_veh_92, -416901209#1
            ('car_normal',           37.33564749, -121.89221571, '417034088#0'),  # bg_veh_85, -416901209#1
            ('car_normal',           37.33562713, -121.89223402, '495569632'),    # manual_veh_10, -416901209#1
            ('suv_normal',           37.33561354, -121.89225732, '417034088#0'),  # bg_veh_86, -416901209#1
            ('car_normal',           37.33558951, -121.89217299, '157781953#2'),  # manual_veh_11, -416901209#1
            ('car_normal',           37.33557633, -121.89219659, '495569632'),    # manual_veh_9, -416901209#1
            ('car_conservative',     37.33556354, -121.89222048, '417034088#0'),  # bg_veh_89, -416901209#1
            ('car_normal',           37.335588,   -121.892214,   '417034088#0'),  # added

            # --- NB approach (from south, heading north) ---
            # ('suv_aggressive',       37.33537495, -121.89192693, '157782193#0'),  # bg_veh_325, -417034082#1
            ('car_normal',           37.335341, -121.891929, '157782193#2'),  # manual_veh_12, -417034082#1
            ('car_aggressive',       37.335359, -121.891904, '157782193#0'),  # bg_veh_332, -417034082#1
            ('car_aggressive',       37.33529182, -121.89186467, '157782193#0'),  # bg_veh_331, -417034082#1
            ('suv_normal',           37.33522849, -121.89181724, '157782193#0'),  # bg_veh_333, -417034082#1
            ('car_conservative',     37.33514847, -121.89175728, '157782193#0'),  # bg_veh_334, junction
            ('car_normal',           37.33509291, -121.89174905, '157782193#0'),  # bg_veh_316, -417034071
            ('car_normal',           37.335311,   -121.891907,   '157782193#0'),  # added
            ('car_normal',           37.335265,   -121.891870,   '157782193#0'),  # added

            # --- 4th St NB exit (past intersection, heading north) ---
            ('car_normal',           37.33601363, -121.89241835, '157782193#2'),  # bg_veh_1189, 157782193#0
        ]

    results = calculator.run()

    if results:
        print("\n✓ Calculation completed!")
        sys.exit(0)
    else:
        print("\n✗ Calculation failed!")
        sys.exit(1)


if __name__ == '__main__':
    main()
