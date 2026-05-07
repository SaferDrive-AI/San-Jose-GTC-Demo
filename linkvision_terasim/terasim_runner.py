from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path
from xml.etree import ElementTree as ET

from .events import RealWorldStalledVehicleEvent
from .signal_optimizer import DEFAULT_TLS_ID, choose_signal_plan


DEFAULT_TERASIM_HOME = Path("/Users/happyfewmac/Desktop/Inchor/Terasim")


@dataclass(frozen=True)
class TeraSimReplayConfig:
    terasim_home: Path = DEFAULT_TERASIM_HOME
    sumo_net_file: Path = Path("san_jose_downtown_gtc/osm.net.xml")
    sumo_config_file: Path = Path("san_jose_downtown_gtc/osm.sumocfg")
    sumo_route_file: Path | None = None
    output_path: Path = Path("outputs/linkvision_terasim")
    mode: str = "dynamic"
    gui: bool = False
    sim_time: int = 1800
    step_length: float = 0.1


def add_terasim_to_path(terasim_home: Path) -> None:
    core_package = terasim_home / "packages" / "terasim"
    if not core_package.exists():
        raise FileNotFoundError(f"TeraSim core package was not found at {core_package}")
    core_package_str = str(core_package)
    if core_package_str not in sys.path:
        sys.path.insert(0, core_package_str)


def prepare_sumolib_compatible_net_file(net_file: Path, output_dir: Path) -> Path:
    """Create a copy of a SUMO net file that older sumolib parsers can read.

    The San Jose network includes traffic-light phase durations like "26.40".
    TeraSim currently reads the network through sumolib withPrograms=True, and
    that path expects integer phase durations. SUMO itself still runs with the
    original configuration; this copy is only for TeraSim's internal net object.
    """

    tree = ET.parse(net_file)
    changed = False
    for phase in tree.getroot().iter("phase"):
        duration = phase.get("duration")
        if duration is None:
            continue
        try:
            rounded = str(int(round(float(duration))))
        except ValueError:
            continue
        if duration != rounded:
            phase.set("duration", rounded)
            changed = True

    if not changed:
        return net_file

    output_dir.mkdir(parents=True, exist_ok=True)
    compatible_net_file = output_dir / f"{net_file.stem}.sumolib-compatible.net.xml"
    tree.write(compatible_net_file, encoding="utf-8", xml_declaration=True)
    return compatible_net_file


def run_terasim_replay(event: RealWorldStalledVehicleEvent, config: TeraSimReplayConfig) -> None:
    add_terasim_to_path(config.terasim_home)

    from terasim.envs.template import EnvTemplate
    from terasim.logger.infoextractor import InfoExtractor
    from terasim.overlay import traci
    from terasim.simulator import Simulator
    from terasim.vehicle.controllers.sumo_move_controller import SUMOMOVEController
    from terasim.vehicle.decision_models.sumo_model import SUMOModel
    from terasim.vehicle.factories.vehicle_factory import VehicleFactory
    from terasim.vehicle.vehicle import Vehicle

    class PassiveVehicleFactory(VehicleFactory):
        def create_vehicle(self, veh_id, simulator):
            return Vehicle(
                veh_id,
                simulator,
                sensors=[],
                decision_model=SUMOModel(),
                controller=SUMOMOVEController(simulator),
            )

    class LinkVisionReplayEnv(EnvTemplate):
        def __init__(self, vehicle_factory, info_extractor, event, mode, sim_time):
            super().__init__(vehicle_factory=vehicle_factory, info_extractor=info_extractor)
            self.event = event
            self.mode = mode
            self.sim_time = sim_time
            self.stalled_vehicle_id = f"linkvision_stalled_{event.source_event_id}"
            self.stalled_lane_id = None
            self.stalled_lane_position = None
            self.signal_decision = None

        def on_start(self, ctx):
            self._add_stalled_vehicle()
            self._apply_signal_plan()
            return True

        def on_step(self, ctx):
            self._pin_stalled_vehicle()
            return traci.simulation.getTime() < self.sim_time

        def on_stop(self, ctx):
            return True

        def _add_stalled_vehicle(self):
            if "linkvision_stalled_vehicle" not in traci.vehicletype.getIDList():
                traci.vehicletype.copy("DEFAULT_VEHTYPE", "linkvision_stalled_vehicle")
                traci.vehicletype.setColor("linkvision_stalled_vehicle", (255, 0, 0, 255))

            x, y = traci.simulation.convertGeo(self.event.longitude, self.event.latitude, fromGeo=True)
            edge_id, lane_position, lane_index = traci.simulation.convertRoad(x, y, isGeo=False)
            route_id = f"linkvision_route_{self.event.source_event_id}"
            if route_id not in traci.route.getIDList():
                traci.route.add(route_id, [edge_id])

            traci.vehicle.add(
                self.stalled_vehicle_id,
                route_id,
                typeID="linkvision_stalled_vehicle",
                depart="0",
                departLane=str(lane_index),
                departPos=str(lane_position),
                departSpeed="0",
            )
            self.stalled_lane_id = f"{edge_id}_{lane_index}"
            self.stalled_lane_position = lane_position
            traci.vehicle.setSpeedMode(self.stalled_vehicle_id, 0)
            traci.vehicle.setLaneChangeMode(self.stalled_vehicle_id, 0)
            traci.vehicle.setSpeed(self.stalled_vehicle_id, 0)
            traci.vehicle.setColor(self.stalled_vehicle_id, (255, 0, 0, 255))

        def _apply_signal_plan(self):
            if self.mode == "bench":
                program_id = "org"
                reason = "bench_mode"
            else:
                available_programs = [
                    logic.programID
                    for logic in traci.trafficlight.getAllProgramLogics(DEFAULT_TLS_ID)
                ]
                self.signal_decision = choose_signal_plan(
                    obstacle_lane_id=self.stalled_lane_id,
                    available_programs=available_programs,
                )
                program_id = self.signal_decision.program_id
                reason = self.signal_decision.reason
            traci.trafficlight.setProgram(DEFAULT_TLS_ID, program_id)
            print(f"TeraSim TLS {DEFAULT_TLS_ID}: program={program_id}, reason={reason}")

        def _pin_stalled_vehicle(self):
            if self.stalled_vehicle_id not in traci.vehicle.getIDList():
                return
            traci.vehicle.setSpeed(self.stalled_vehicle_id, 0)
            if self.stalled_lane_id is not None and self.stalled_lane_position is not None:
                traci.vehicle.moveTo(
                    self.stalled_vehicle_id,
                    self.stalled_lane_id,
                    self.stalled_lane_position,
                )

    terasim_net_file = prepare_sumolib_compatible_net_file(
        config.sumo_net_file,
        config.output_path,
    )

    additional_sumo_args = ["--time-to-teleport", "-1"]
    if config.sumo_route_file is not None:
        additional_sumo_args += ["--route-files", str(Path(config.sumo_route_file).resolve())]

    simulator = Simulator(
        sumo_net_file_path=terasim_net_file,
        sumo_config_file_path=config.sumo_config_file,
        gui_flag=config.gui,
        output_path=config.output_path,
        sumo_output_file_types=["tripinfo", "fcd_all"],
        step_length=config.step_length,
        additional_sumo_args=additional_sumo_args,
    )
    env = LinkVisionReplayEnv(
        vehicle_factory=PassiveVehicleFactory(),
        info_extractor=InfoExtractor,
        event=event,
        mode=config.mode,
        sim_time=config.sim_time,
    )
    simulator.bind_env(env)
    simulator.run()
