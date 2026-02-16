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


class SUMODelayCalculator:
    """SUMO Delay Calculator - Using TraCI for dynamic control"""

    def __init__(self, net_file, route_file, obstacles=None, tls_program=None,
                 sim_time=3600, step_length=0.1, gui=False, output_file=None, mode='static',
                 tripinfo_file=None, statistic_file=None):
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

        self.sumo_binary = 'sumo-gui' if self.gui else 'sumo'

        # Temporary configuration file
        self.temp_dir = tempfile.mkdtemp(prefix='sumo_sim_')
        self.config_file = "/home/yilinwang/San Jose GTC Demo/san_jose_downtown_gtc/osm.sumocfg"

        # Statistical data
        self.vehicle_data = {}
        self.departed_vehicles = set()
        self.arrived_vehicles = set()

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

    def add_obstacles_via_traci(self):
        """Add obstacles via TraCI - using stationary vehicles"""
        if not self.obstacles:
            return

        print(f"\nAdding obstacles (total: {len(self.obstacles)}):")

        # Store obstacle vehicle IDs and position information
        self.obstacle_vehicles = []

        for idx, (lat, lon, width, height, angle) in enumerate(self.obstacles):
            obstacle_veh_id = f'obstacle_veh_{idx}'

            try:
                # Convert latitude/longitude to SUMO coordinates
                x, y = self.latlon_to_xy(lat, lon)
                print(f"\n  Obstacle {idx}:")
                print(f"    Lat/Lon: ({lat:.6f}, {lon:.6f})")
                print(f"    SUMO coordinates: ({x:.2f}, {y:.2f})")

                # Find nearest edge and lane
                edge_id, lane_idx = self._find_nearest_edge(x, y)

                if edge_id:
                    # If angle not specified, get lane angle
                    if angle is None:
                        lane_id = f"{edge_id}_{lane_idx}"
                        lane_angle = traci.lane.getAngle(lane_id)
                        angle = lane_angle
                        print(f"    Auto-detected lane angle: {angle:.1f}°")
                    else:
                        print(f"    Using specified angle: {angle:.1f}°")

                    # Create a single-edge route for the obstacle
                    obstacle_route_id = f'obstacle_route_{idx}'
                    traci.route.add(obstacle_route_id, [edge_id])

                    # Add vehicle
                    traci.vehicle.add(
                        vehID=obstacle_veh_id,
                        routeID=obstacle_route_id,
                        typeID='DEFAULT_VEHTYPE',
                        depart='now',
                        departLane=lane_idx,
                        departPos='base',
                        departSpeed='0'
                    )

                    # Move immediately to specified position
                    traci.vehicle.moveToXY(
                        vehID=obstacle_veh_id,
                        edgeID=edge_id,
                        laneIndex=lane_idx,
                        x=x,
                        y=y,
                        angle=angle,
                        keepRoute=2  # 2 means ignore route
                    )

                    # Set speed mode: disable all safety checks, allow complete stop
                    # 0x00 = disable all checks
                    traci.vehicle.setSpeedMode(obstacle_veh_id, 0)

                    # Set speed to 0
                    traci.vehicle.setSpeed(obstacle_veh_id, 0)

                    # Change vehicle color to red to indicate obstacle
                    traci.vehicle.setColor(obstacle_veh_id, (255, 0, 0, 255))

                    # Prevent SUMO from removing the vehicle due to route completion
                    stop_lane_id = f"{edge_id}_{lane_idx}"
                    traci.vehicle.setStop(
                        obstacle_veh_id,
                        edgeID=edge_id,
                        pos=traci.lane.getLength(stop_lane_id),
                        laneIndex=lane_idx,
                        duration=2**31 - 1
                    )

                    # Save obstacle information
                    self.obstacle_vehicles.append({
                        'id': obstacle_veh_id,
                        'x': x,
                        'y': y,
                        'angle': angle,
                        'edge': edge_id,
                        'lane': lane_idx
                    })

                    print(f"    ✓ Obstacle vehicle added")
                else:
                    print(f"    ✗ Could not find suitable road position")

            except Exception as e:
                print(f"  ✗ Failed to add obstacle vehicle {idx}: {e}")
                import traceback
                traceback.print_exc()

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
        """Update obstacle positions each step to keep them stationary"""
        if not hasattr(self, 'obstacle_vehicles'):
            return


        self.obstacle_info = []  # Store info of the last successfully updated obstacle
        for obs in self.obstacle_vehicles:
            try:
                # Check if vehicle is still in simulation
                if obs['id'] in traci.vehicle.getIDList():
                    # Force move to original position
                    traci.vehicle.moveToXY(
                        vehID=obs['id'],
                        edgeID=obs['edge'],
                        laneIndex=obs['lane'],
                        x=obs['x'],
                        y=obs['y'],
                        angle=obs['angle'],
                        keepRoute=2
                    )

                    # Ensure speed is 0
                    traci.vehicle.setSpeed(obs['id'], 0)
                    self.obstacle_info.append(obs)
            except:
                pass

    def trigger_rerouting(self, step):
        """Proactively trigger rerouting for congested vehicles"""
        if not hasattr(self, 'last_reroute_check'):
            self.last_reroute_check = 0
            self.reroute_count = 0

        # Check every 30 seconds
        if step - self.last_reroute_check < 5:
            return

        self.last_reroute_check = step
        vehicle_ids = traci.vehicle.getIDList()

        for veh_id in vehicle_ids:
            # Skip obstacle vehicles
            if veh_id.startswith('obstacle_veh_'):
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
            except Exception:
                pass

        if self.reroute_count > 0 and step % 100 == 0:
            print(f"\n  Triggered {self.reroute_count} reroutes")

    def assist_stuck_vehicles(self, current_time):
        """Progressively help vehicles stuck behind obstacles to change lanes.

        Uses SUMO's sublane model parameters (lcPushy, lcAssertive, lcImpatience)
        to make stuck vehicles increasingly aggressive about lane changing.
        After 100s of waiting, forces a lane change by overriding safety checks.
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
            if veh_id.startswith('obstacle_veh_'):
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

                        if wait > 100:
                            # === Force lane change: override all safety ===
                            traci.vehicle.setLaneChangeMode(veh_id, 0)
                            cur_lane = traci.vehicle.getLaneIndex(veh_id)
                            edge_id = traci.vehicle.getRoadID(veh_id)
                            n_lanes = traci.edge.getLaneNumber(edge_id)
                            target = cur_lane + 1 if cur_lane + 1 < n_lanes else cur_lane - 1
                            if 0 <= target < n_lanes:
                                traci.vehicle.changeLane(veh_id, target, 15.0)
                            self._lc_force_count += 1
                        elif wait > 60:
                            # Very aggressive: high pushy + assertive
                            traci.vehicle.setParameter(veh_id, "laneChangeModel.lcPushy", "1.0")
                            traci.vehicle.setParameter(veh_id, "laneChangeModel.lcAssertive", "5.0")
                            traci.vehicle.setParameter(veh_id, "laneChangeModel.lcImpatience", "1.0")
                        elif wait > 30:
                            # Moderately aggressive
                            traci.vehicle.setParameter(veh_id, "laneChangeModel.lcPushy", "0.5")
                            traci.vehicle.setParameter(veh_id, "laneChangeModel.lcAssertive", "3.0")
                            traci.vehicle.setParameter(veh_id, "laneChangeModel.lcImpatience", "0.5")
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
            if veh_id.startswith('obstacle_veh_'):
                continue
            try:
                waiting_time = traci.vehicle.getWaitingTime(veh_id)
                if waiting_time >= threshold:
                    traci.vehicle.remove(veh_id, reason=2)  # 2 = REMOVE_TELEPORT
                    self._remove_count += 1
            except Exception:
                pass

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
                print(f"\n  TLS {target_tls_id}: using original program 'org' (bench mode)")
            except Exception as e:
                print(f"\n  TLS {target_tls_id}: failed to set program - {e}")
            return

        # Opt mode -> always use optimized "opt"
        if self.mode == 'opt':
            try:
                traci.trafficlight.setProgram(target_tls_id, "opt")
                print(f"\n  TLS {target_tls_id}: using optimized program 'opt' (opt mode)")
            except Exception as e:
                print(f"\n  TLS {target_tls_id}: failed to set program - {e}")
            return

        # Dynamic mode: if no obstacle info, fall back to "org"
        if not hasattr(self, 'obstacle_info') or not self.obstacle_info:
            try:
                traci.trafficlight.setProgram(target_tls_id, "org")
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
                        print(f"  TLS: exact lane program not found, "
                              f"using edge-based program '{edge_program}'")
                    else:
                        traci.trafficlight.setProgram(target_tls_id, "opt")
                        print(f"  TLS: no lane-specific program for '{matched_lane}', "
                              f"falling back to 'opt'")
            else:
                traci.trafficlight.setProgram(target_tls_id, "org")
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
            if veh_id.startswith('obstacle_veh_'):
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
                '--start',  # Auto-start simulation (GUI mode)
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

            traci.start(sumo_cmd)
            print("✓ SUMO started\n")

            # Zoom into the intersection area
            if self.gui:
                # Center on the intersection (average of obstacle coords)
                center_x, center_y = self.latlon_to_xy(37.3354, -121.8921)
                traci.gui.setOffset("View #0", center_x, center_y)
                traci.gui.setZoom("View #0", 300)

            # Add obstacles
            self.add_obstacles_via_traci()

            # Set traffic light program
            self.set_tls_program_via_traci()

            # Run simulation
            print(f"\nRunning simulation...")
            step = 0
            total_steps = int(self.sim_time / self.step_length)

            while step < total_steps:
                traci.simulationStep()

                # Update obstacle positions (keep stationary)
                self.update_obstacle_positions()

                # update tls program based on mode and obstacle status
                self.update_tls_program()

                # Proactively trigger rerouting for congested vehicles
                self.trigger_rerouting(step)

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
                'vehicle_count': 0
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
                'tls_program': self.tls_program if isinstance(self.tls_program, (dict, str)) else None,
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
        statistic_file=args.statistic_output
    )

    results = calculator.run()

    if results:
        print("\n✓ Calculation completed!")
        sys.exit(0)
    else:
        print("\n✗ Calculation failed!")
        sys.exit(1)


if __name__ == '__main__':
    main()
