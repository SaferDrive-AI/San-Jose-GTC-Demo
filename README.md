# SUMO Traffic Simulation Delay Calculator

A comprehensive traffic simulation tool using SUMO (Simulation of Urban MObility) with TraCI (Traffic Control Interface) for dynamic traffic flow control and delay analysis.

## Overview

This project provides tools to:
- Simulate traffic scenarios with customizable road networks
- Add obstacles (using stationary vehicles) at specific geographic locations
- Configure custom traffic light programs
- Analyze traffic delay metrics
- Generate directional traffic flows with varying densities

## Features

- **Geographic Coordinate Support**: Input obstacle locations using latitude/longitude coordinates
- **Realistic Obstacle Simulation**: Obstacles implemented as stationary vehicles that other vehicles respond to naturally
- **Automatic Angle Detection**: Obstacles automatically align with road angles, or you can specify manually
- **Dynamic Traffic Light Control**: Modify traffic light programs during simulation
- **Real-time Data Collection**: Collect and analyze delay statistics during simulation
- **GUI Support**: Optional visual mode for observing simulation behavior
- **Directional Traffic Generation**: Create traffic flows with different densities in different directions

## Prerequisites

### Required Software
- **SUMO** (Simulation of Urban MObility) - Version 1.8.0 or higher
- **Python** 3.7 or higher
- **TraCI** (included with SUMO)

### Installation

1. Install SUMO:
   ```bash
   pip install eclipse-sumo==1.23.1
   ```
   
## Project Structure

```
San_Jose_demo/
├── main.py                              # Main simulation controller
├── generate_directional_routes.py      # Generate directional traffic routes
├── generate_12phase_traffic.py         # Generate 12-phase traffic patterns
├── san_jose_downtown_gtc/              # Network and route files
│   ├── osm.net.xml                     # Road network file
│   ├── osm.passenger.trips.xml         # Passenger trip definitions
│   └── osm.sumocfg                     # SUMO configuration file
└── tls_config_example.json             # Example traffic light configuration
```

## Usage

### Basic Usage: main.py

The main script runs a SUMO simulation with customizable parameters:

```bash
python main.py \
    --net-file san_jose_downtown_gtc/osm.net.xml \
    --route-file san_jose_downtown_gtc/osm.passenger.trips.xml \
    --sim-time 3600 \
    --gui
```

#### Command Line Arguments

| Argument | Required | Default | Description |
|----------|----------|---------|-------------|
| `--net-file` | Yes | - | SUMO network file (.net.xml) |
| `--route-file` | Yes | - | Route file (.rou.xml or .trips.xml) |
| `--obstacles` | No | None | Obstacle definitions (see format below) |
| `--tls-program` | No | None | Custom traffic light program (JSON file or string) |
| `--sim-time` | No | 3600 | Simulation duration in seconds |
| `--step-length` | No | 1.0 | Simulation step length in seconds |
| `--gui` | No | False | Use GUI mode for visualization |
| `--output` | No | None | Output JSON file path for results |

#### Obstacle Format

Obstacles are defined using geographic coordinates:

```
"lat,lon[,width,height,angle];..."
```

Parameters:
- `lat`: Latitude (required)
- `lon`: Longitude (required)
- `width`: Width in meters (optional, default 0, currently unused)
- `height`: Height in meters (optional, default 0, currently unused)
- `angle`: Angle in degrees (optional, auto-detects from road if not provided)

**Examples:**

```bash
# Single obstacle with auto angle
--obstacles "37.33251,-121.892360"

# Single obstacle with specified angle
--obstacles "37.33251,-121.892360,0,0,90"

# Multiple obstacles
--obstacles "37.33251,-121.892360;37.33252,-121.892370"

# Multiple obstacles with different angles
--obstacles "37.33251,-121.892360,0,0,90;37.33252,-121.892370,0,0,45"
```

#### Traffic Light Program Format

Traffic light programs can be specified as JSON:

```json
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
```

Save this as `tls_config.json` and use:
```bash
--tls-program tls_config.json
```

### Complete Example

Run a simulation with obstacles and custom traffic lights:

```bash
python main.py \
    --net-file san_jose_downtown_gtc/osm.net.xml \
    --route-file san_jose_downtown_gtc/osm.passenger.trips.xml \
    --obstacles "37.335265,-121.892334" \
    --tls-program tls_config_example.json \
    --sim-time 1800 \
    --gui \
    --output results.json
```

### Generate Directional Routes: generate_directional_routes.py

Generate traffic routes with different densities in different directions:

```bash
python generate_directional_routes.py \
    --net-file san_jose_downtown_gtc/osm.net.xml \
    --junction cluster_1984576776_3478559735_3478559736_3537422682_#1more \
    --obstacle-direction east-west \
    --high-flow 120 \
    --low-flow 30 \
    --output custom_routes.rou.xml
```

#### Arguments

| Argument | Required | Default | Description |
|----------|----------|---------|-------------|
| `--net-file` | Yes | - | SUMO network file |
| `--junction` | Yes | - | Target junction ID |
| `--obstacle-direction` | Yes | - | Direction with obstacles: `east-west` or `north-south` |
| `--high-flow` | No | 100 | High traffic flow (vehicles/hour) |
| `--low-flow` | No | 20 | Low traffic flow (vehicles/hour) |
| `--sim-time` | No | 1800 | Simulation duration (seconds) |
| `--output` | No | custom_routes.rou.xml | Output route file |

### Generate 12-Phase Traffic: generate_12phase_traffic.py

Generate traffic based on 12-phase traffic patterns:

```bash
python generate_12phase_traffic.py \
    --base-routes 12_phase_route.rou.xml \
    --ew-flow 150 \
    --ns-flow 30 \
    --sim-time 1800 \
    --output directional_traffic.rou.xml
```

#### Arguments

| Argument | Required | Default | Description |
|----------|----------|---------|-------------|
| `--base-routes` | No | 12_phase_route.rou.xml | Base route file with 12 route definitions |
| `--ew-flow` | No | 150 | East-west traffic flow (vehicles/hour) |
| `--ns-flow` | No | 30 | North-south traffic flow (vehicles/hour) |
| `--sim-time` | No | 1800 | Simulation duration (seconds) |
| `--output` | No | directional_traffic.rou.xml | Output file path |

## Output

The simulation generates comprehensive statistics including:

```json
{
  "configuration": {
    "net_file": "path/to/network.net.xml",
    "route_file": "path/to/routes.rou.xml",
    "obstacles": [...],
    "simulation_time": 3600
  },
  "results": {
    "total_departed": 500,
    "total_arrived": 480,
    "average_duration": 245.3,
    "average_delay": 45.2,
    "average_wait_time": 45.2,
    "total_time_loss": 21696.0,
    "vehicle_count": 480
  }
}
```

### Metrics Explained

- **total_departed**: Number of vehicles that entered the simulation
- **total_arrived**: Number of vehicles that completed their journey
- **average_duration**: Average trip duration in seconds
- **average_delay**: Average time loss per vehicle (seconds)
- **average_wait_time**: Average waiting time per vehicle (seconds)
- **total_time_loss**: Total accumulated delay for all vehicles (seconds)

## Typical Workflow

1. **Prepare Network**: Ensure you have a SUMO network file (.net.xml)

2. **Generate Traffic**: Create directional traffic patterns
   ```bash
   python generate_directional_routes.py \
       --net-file san_jose_downtown_gtc/osm.net.xml \
       --junction <junction_id> \
       --obstacle-direction east-west \
       --high-flow 150 \
       --low-flow 30 \
       --output routes.rou.xml
   ```

3. **Run Simulation**: Execute with obstacles and analysis
   ```bash
   python main.py \
       --net-file san_jose_downtown_gtc/osm.net.xml \
       --route-file routes.rou.xml \
       --obstacles "37.335265,-121.892334" \
       --sim-time 1800 \
       --gui \
       --output results.json
   ```

4. **Analyze Results**: Review the generated JSON output file

## Troubleshooting

### Common Issues

**Error: "Cannot import traci module"**
- Solution: Ensure SUMO is installed and `SUMO_HOME` is set correctly
  ```bash
  export SUMO_HOME="/usr/share/sumo"
  ```

**Error: "Network file not found"**
- Solution: Verify the path to your .net.xml file is correct
- Use absolute paths or ensure you're running from the correct directory

**Simulation runs but no vehicles appear**
- Check that your route file contains valid trips or flows
- Verify that departure times are within the simulation time range
- Use `--gui` mode to visually inspect the simulation

**Obstacles not appearing in correct location**
- Ensure latitude/longitude coordinates are correct
- Check that coordinates fall within the network boundaries
- Use GUI mode to verify obstacle placement

## Advanced Configuration

### Rerouting Parameters

The simulation includes automatic rerouting for vehicles stuck in traffic. Configure in the code:

```python
# In SUMODelayCalculator.create_config_file()
ET.SubElement(routing_elem, 'device.rerouting.period').set('value', '30')
ET.SubElement(routing_elem, 'device.rerouting.adaptation-steps').set('value', '18')
```

### Vehicle Speed Modes

Obstacle vehicles use specific speed modes to remain stationary:

```python
# Speed mode 0x00 = Disable all safety checks
traci.vehicle.setSpeedMode(obstacle_veh_id, 0)
traci.vehicle.setSpeed(obstacle_veh_id, 0)
```

## License

This project uses SUMO, which is licensed under the Eclipse Public License 2.0.

## Contributing

Contributions are welcome. Please ensure all code follows the existing style and includes appropriate documentation.

## Support

For issues related to:
- **SUMO**: Visit [SUMO Documentation](https://sumo.dlr.de/docs/)
- **This Project**: Open an issue in the project repository

## References

- [SUMO Documentation](https://sumo.dlr.de/docs/)
- [TraCI Documentation](https://sumo.dlr.de/docs/TraCI.html)
- [SUMO Traffic Light Control](https://sumo.dlr.de/docs/Simulation/Traffic_Lights.html)
