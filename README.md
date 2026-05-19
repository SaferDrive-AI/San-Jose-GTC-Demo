# Obstacle-Aware Traffic Signal Optimization — San Jose GTC Demo

A SUMO-based traffic simulation platform that evaluates how stalled vehicles (obstacles) impact intersection performance and demonstrates adaptive signal timing optimization to mitigate delay.

## Background

When a vehicle stalls at an intersection, it reduces lane capacity and increases delay for surrounding traffic. This project simulates such scenarios at the **S Market St & W Santa Clara St** intersection in downtown San Jose, and tests optimized traffic signal plans that redistribute green time to compensate for the capacity loss.

The core idea: if a westbound (WB) through lane is blocked, the signal controller can **increase east-west green time** and **reduce north-south green time** to better serve the affected approach — yielding lower overall delay.

## Key Results

| Scenario | Avg Delay | Vehicles Arrived |
|----------|-----------|-----------------|
| Default plan, no obstacle | 26.1s | 767 |
| Default plan + WB stalled veh | 31.8s | 739 |
| Plan 1 (EW: 26.5s) + WB stalled veh | 29.9s | 760 |
| Plan 2 (EW: 25.3s) + WB stalled veh | 29.7s | 732 |
| Plan 3 (EW: 24.1s) + WB stalled veh | 29.3s | 715 |
| Plan 4 / Optimal (EW: 28.2s) + WB stalled veh | 28.3s | 764 |

## Project Structure

```
San Jose GTC Demo/
├── main.py                          # Core simulation engine (TraCI-based)
├── main_demo.py                     # Demo video recording with cinematic camera (zoom, rotation)
├── main_overview.py                 # Fixed overhead camera demo (no obstacle, no zoom)
├── main_scene2.py                   # Alternate scene with wider FOV and different intersection
├── generate_12phase_traffic.py      # Generate 12-phase directional traffic flows
├── generate_directional_routes.py   # Generate routes with directional asymmetry
├── merge_networks.py                # Safe add-only merge of downtown network into full network
├── upscale_tiles.py                 # Batch 4x upscale background tiles via Real-ESRGAN
├── tls_config_example.json          # Example TLS configuration
├── demo_config.json                 # Demo video parameters (zoom, rotation, vehicle count, etc.)
├── routes.rou.xml                   # Root-level route definitions
├── SJ_scene4_4cases.mp4             # Pre-recorded demo video (4 cases)
│
├── run_cases.sh                     # Run 3-case benchmark comparison
├── run_plan_cases.sh                # Run 5-case signal plan comparison
├── run_simulations.sh               # Run full 25-case obstacle sweep
├── run_full_new.sh                  # Run demo on full San Jose network (cinematic video)
├── run_all.sh                       # Run both demo + overview simulations sequentially
├── run_overview.sh                  # Run fixed-camera overview demo
├── make_video.sh                    # Convert screenshot frames to MP4 via ffmpeg
│
├── san_jose_downtown_gtc/           # SUMO network & config (downtown intersection)
│   ├── osm.net.xml                  # Road network (S Market & Santa Clara)
│   ├── osm.tls.xml                  # Traffic light programs (org + optimized plans)
│   ├── osm.sumocfg                  # SUMO configuration
│   ├── osm.rou.xml                  # Base routes with vehicle type definitions
│   ├── directional_traffic.rou.xml  # Generated directional flows
│   └── background_images/           # Map tile images for visualization
│
├── san_jose_full_new/               # SUMO network & config (full San Jose area)
│   ├── osm_merged.net.xml.gz        # Merged full-area road network
│   ├── osm.tls.xml                  # Traffic light programs
│   ├── simulation.sumocfg           # SUMO configuration
│   ├── intersection_flows.rou.xml   # Intersection flow definitions
│   ├── background_trips.trips.xml   # Background vehicle trips
│   ├── local_preload.trips.xml      # Pre-loaded local trips
│   ├── *_vehicles_cache.json        # Cached vehicle placements for scenes
│   └── background_images/           # 7000+ upscaled map tiles
│
├── weights/
│   └── RealESRGAN_x4plus.pth        # Real-ESRGAN model weights for tile upscaling
│
└── traffic_data_analysis/
    ├── plot_delay_comparison.py      # Plot 3-case delay comparison
    ├── plot_plan_comparison.py       # Plot 4-plan signal optimization comparison
    ├── delay_result/                 # Simulation output JSONs and charts
    └── linkVision_rawData/           # LinkVision API raw data (stalled car detection, metadata)
```

## Prerequisites

- **SUMO** >= 1.22.0 with TraCI
- **Python** >= 3.7
- **matplotlib**, **numpy**

```bash
pip install eclipse-sumo matplotlib numpy
```

## Usage

### Quick Start — 3-Case Benchmark Comparison (`run_cases.sh`)

Runs 3 simulations on the **downtown intersection** network to demonstrate the impact of obstacles and adaptive signal control:

| Case | Obstacle | Signal Mode | Description |
|------|----------|-------------|-------------|
| 1 | None | `bench` (original) | Baseline: normal traffic with default signal timing |
| 2 | WB through lane | `bench` (original) | A vehicle stalls in the WB through lane; signal timing stays unchanged, showing increased delay |
| 3 | WB through lane | `dynamic` (adaptive) | Same stalled vehicle, but signal controller detects the blocked lane and switches to an optimized plan |

Outputs delay JSONs + tripinfo XMLs, then generates a comparison bar chart via `plot_delay_comparison.py`.

```bash
bash run_cases.sh
```

### 5-Case Signal Plan Comparison (`run_plan_cases.sh`)

Runs 5 simulations on the **downtown intersection** network, all with a WB stalled vehicle. Tests progressively optimized signal timing plans that shift green time from NS to EW:

| Case | Plan | EW Green | NS Green | Description |
|------|------|----------|----------|-------------|
| 1 | `org` (original) | 26.4s | 27.8s | Default timing — no adaptation |
| 2 | `plan_1` | 26.5s | 19.0s | Slight EW increase, moderate NS decrease |
| 3 | `plan_2` | 25.3s | 18.5s | Further NS reduction |
| 4 | `plan_3` | 24.1s | 18.0s | Aggressive NS reduction |
| 5 | `plan_4` (optimal) | 29.0s | 16.5s | Best balance: maximum EW green for WB blockage |

Generates a comparison chart via `plot_plan_comparison.py`.

```bash
bash run_plan_cases.sh
```

### Full 25-Case Obstacle Sweep (`run_simulations.sh`)

Comprehensive evaluation on the **downtown intersection** network. Tests all 12 obstacle positions (EB/WB/NB/SB × left/through/right lanes) in both static and dynamic signal modes:

| Runs | Mode | Description |
|------|------|-------------|
| 1 | `bench` | Benchmark: no obstacle, original signal timing |
| 2–13 | `bench` | 12 obstacle positions with original signal (static) |
| 14–25 | `dynamic` | Same 12 obstacle positions with adaptive signal switching |

Each run simulates 1800 seconds. Outputs 25 delay JSON files for analysis.

```bash
bash run_simulations.sh
```

### Full San Jose Network Demo (`run_full_new.sh`)

Runs a cinematic demo on the **full San Jose network** (`san_jose_full_new/`) with ~3500 background vehicles. Produces a video-ready simulation with zoom, rotation, and camera transitions:

| Phase | Description |
|-------|-------------|
| Setup | Generates background random trips and intersection flows via SUMO tools |
| Warmup | Background traffic fills the network for realistic density |
| Obstacle | WB stalled vehicle placed at the intersection |
| Cinematic | Camera zooms in, pauses, releases vehicles, then runs main simulation loop |
| Recording | Screenshots captured automatically for video conversion |

Supports flags: `--no-gui` (headless), `--skip-setup` (reuse generated files), `--bench` (benchmark mode).

```bash
bash run_full_new.sh
bash run_full_new.sh --no-gui --skip-setup
```

### Run Both Demo Simulations (`run_all.sh`)

Sequentially runs two demo simulations on the **full San Jose network**:

| Task | Script | Description |
|------|--------|-------------|
| 1 | `main_demo.py` | Cinematic zoom-in + rotation video with stalled vehicle |
| 2 | `main_overview.py` | Fixed overhead camera overview (no obstacle, no zoom) |

Both use ~3500 background vehicles. Supports `--no-gui` flag.

```bash
bash run_all.sh
bash run_all.sh --no-gui
```

### Overview Demo (`run_overview.sh`)

Runs a simple fixed-overhead-camera demo on the **full San Jose network** — no stalled vehicle, no zoom animation. Shows general traffic flow for 30 seconds with a 10-second warmup.

```bash
bash run_overview.sh
bash run_overview.sh --no-gui
```

### Convert Screenshots to Video (`make_video.sh`)

Converts SUMO screenshot frames (from `screenshots/` directory) into an MP4 video at 60fps using ffmpeg (H.264 codec).

```bash
bash make_video.sh
bash make_video.sh /path/to/custom/screenshots
```

### Single Simulation

```bash
python main.py \
    --obstacles "37.335577, -121.891913" \
    --mode dynamic \
    --output traffic_data_analysis/delay_result/delay_test.json \
    --gui
```

### Specify an Explicit Signal Plan

```bash
python main.py \
    --obstacles "37.335577, -121.891913" \
    --program-id "1418903639#0_2" \
    --output traffic_data_analysis/delay_result/delay_test.json \
    --gui
```

## Simulation Modes

| Mode | `--mode` | Behavior |
|------|----------|----------|
| Benchmark | `bench` | Always uses original signal plan (`org`) |
| Optimized | `opt` | Always uses optimized plan (`opt`) |
| Dynamic | `dynamic` | Detects obstacle lane and switches to matching plan |
| Explicit | `--program-id <ID>` | Uses the specified TLS program ID directly |

## Signal Plan Design

All plans share a common 18-phase structure (100s cycle). The key variable phases are:

| Phase | Function | org (default) | Plan 4 (optimal for WB) |
|-------|----------|---------------|------------------------|
| EW main green | Serves EB/WB through traffic | 26.4s | 29.0s |
| NS turning | Serves NB/SB left/right turns | 12.0s | 6.2s |
| NS main green | Serves NB/SB through traffic | 27.8s | 16.5s |

Intermediate plans (plan_1 through plan_3) progressively shift green time from NS to EW.

## Obstacle Positions

Pre-defined GPS coordinates for 12 test positions around the intersection:

| Direction | Left | Through | Right |
|-----------|------|---------|-------|
| EB | 37.335379, -121.892249 | 37.335358, -121.892248 | 37.335338, -121.892208 |
| WB | 37.335558, -121.891889 | 37.335577, -121.891913 | 37.335601, -121.891930 |
| NB | 37.335328, -121.891956 | 37.335340, -121.891927 | 37.335356, -121.891898 |
| SB | 37.335605, -121.892187 | 37.335353, -121.892234 | 37.335578, -121.892244 |

## Output Format

Each simulation produces a JSON file:

```json
{
  "configuration": {
    "net_file": "...",
    "route_file": "...",
    "obstacles": [...],
    "simulation_time": 1800,
    "step_length": 0.1
  },
  "results": {
    "average_delay": 28.26,
    "average_duration": 85.68,
    "vehicle_count": 764,
    "total_departed": 792,
    "total_arrived": 764,
    "total_time_loss": 21588.2
  }
}
```

## Vehicle Behavior Features

- **Automatic rerouting**: Vehicles stuck > 30s recalculate their path every 5s
- **Progressive lane changing**: Vehicles behind obstacles become increasingly aggressive (30s → moderate, 60s → aggressive, 100s → forced lane change)
- **Stuck vehicle removal**: Non-obstacle vehicles waiting > 180s are teleported out

## Python Modules

| File | Description |
|------|-------------|
| `main.py` | Core simulation engine — manages TraCI connection, obstacle placement, signal switching, delay calculation, rerouting, and lane-change logic |
| `main_demo.py` | Extends `main.py` for cinematic demo recording — 7-phase workflow with camera zoom/rotation, vehicle freeze/release, and automatic screenshot capture |
| `main_overview.py` | Simplified demo — fixed overhead camera, no obstacle, no animations. Inherits from `main_demo.py` |
| `main_scene2.py` | Alternate scene targeting a different intersection area with wider FOV (zoom 600), simultaneous vehicle release |
| `generate_12phase_traffic.py` | Generates 12-phase directional traffic flow definitions for the downtown intersection |
| `generate_directional_routes.py` | Generates route files with configurable directional asymmetry (e.g., heavier WB flow) |
| `merge_networks.py` | Safe add-only merge utility — adds edges/junctions from downtown network into full network without overwriting existing elements |
| `upscale_tiles.py` | Batch 4x super-resolution of background map tiles using Real-ESRGAN (256×256 → 1024×1024) |

## Two Simulation Networks

This project includes two SUMO network configurations at different scales:

| Network | Directory | Scope | Use Case |
|---------|-----------|-------|----------|
| Downtown Intersection | `san_jose_downtown_gtc/` | Single intersection (S Market & Santa Clara) | Quantitative analysis: delay comparison, signal plan optimization, obstacle sweep |
| Full San Jose | `san_jose_full_new/` | Larger area with merged road network | Demo/video: cinematic recordings with background traffic density |

## LinkVision Integration

The `traffic_data_analysis/linkVision_rawData/` directory contains raw data from the LinkVision API, including stalled car detection responses and ITS task metadata. This data feeds into the obstacle detection pipeline that triggers adaptive signal control.

## References

- [SUMO Documentation](https://sumo.dlr.de/docs/)
- [TraCI Documentation](https://sumo.dlr.de/docs/TraCI.html)
- [SUMO Traffic Light Control](https://sumo.dlr.de/docs/Simulation/Traffic_Lights.html)

---

## Branch `linker-vss` — Updates

Work on this branch covers two integrations on top of `main`:

### 1. LinkVision ↔ TeraSim pipeline (`linkvision_terasim/`)

End-to-end pipeline that ingests LinkVision detection data and drives SUMO/TeraSim scenarios. Obstacle injection now goes through TeraSim's `StalledObjectAdversity` instead of ad-hoc TraCI calls, and the replay pipeline has module-level documentation across its modules and tests. Removed the forced view rotation on simulation start so the camera no longer snaps on launch.

### 2. VSS synthesis pipeline (`vss_synth/`, `tools_vss_boundary_pipeline.py`)

Generates SUMO route files from VSS (camera) detection JSON:

- **Junction-chain boundary inference** — corridor flows are anchored on junction chains rather than single edges, so spawn/exit decisions stay stable across camera layouts.
- **Dual-axis corridor BFS** — corridor expansion runs along both axes; spawn behavior is stabilized via upstream-hop search.
- **2/3/4-camera regression harness** — covers the canonical camera counts; the 3-camera sample (`sample_vss_data_3_cameras.json`) replaces the older 4-camera sample as the default fixture.
- **TMR (turn-movement ratio) redistribution fix** — turn ratios at boundary nodes are renormalized correctly when a movement is pruned.
- **Vehicle type / shape injection** — `tools_inject_vehicle_types.py` rewrites a flat route file into a realistic vehicle mix (cars, trucks, buses, motorcycles) using `vss_synth/vehicle_types.xml`.
- **GUI helper** — `bash vss_synth/run_vss_gui.sh` runs the full pipeline and launches `sumo-gui` against the downtown network. Outputs land in `outputs/vss_gui_*` (route, boundary, patterns).

### New entry-point files

| File | Description |
|------|-------------|
| `tools_vss_boundary_pipeline.py` | VSS JSON → SUMO routes, with junction-chain boundary inference and dual-axis corridor BFS |
| `tools_inject_vehicle_types.py` | Injects a realistic vehicle-type mix into a generated route file |
| `vss_synth/run_vss_gui.sh` | Convenience runner: pipeline + type injection + `sumo-gui` |
| `vss_synth/sample_vss_data.json`, `sample_vss_data_3_cameras.json` | Sample VSS inputs (default + 3-camera) |
| `vss_synth/vehicle_types.xml` | Vehicle-type definitions used by the injector |
| `linkvision_terasim/` | LinkVision-driven TeraSim replay pipeline |
