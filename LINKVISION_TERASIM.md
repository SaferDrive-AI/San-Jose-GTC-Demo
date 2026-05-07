# LinkVision + TeraSim Replay

This branch turns LinkVision stalled-vehicle detections into replayable San Jose traffic-simulation cases.

## Data Flow

```text
LinkVision event JSON
  -> filter Unexpected Stop events
  -> select warning vehicle object
  -> match camera_id to camera metadata
  -> bbox bottom-center to image pixel
  -> homography to latitude/longitude
  -> RealWorldStalledVehicleEvent
  -> map-match inside SUMO/TeraSim
  -> inject stationary vehicle
  -> choose fixed traffic signal plan
  -> run bench/dynamic comparison
  -> output delay/tripinfo/FCD artifacts
```

## Important Boundaries

- LinkVision provides the real event source.
- The San Jose SUMO network provides the map/digital twin.
- TeraSim manages replay execution when using `run_linkvision_terasim.py`.
- The signal optimizer is deterministic: obstacle lane/program inputs produce one program ID.

## Offline SUMO Replay

Dry-run the selected real event and generated commands:

```bash
python3 run_linkvision_replay.py --dry-run --sim-time 40 --mode compare
```

Run the comparison through the existing SUMO demo:

```bash
python3 run_linkvision_replay.py --sim-time 1800 --mode compare --no-gui
```

Use a downloaded LinkVision response:

```bash
python3 run_linkvision_replay.py \
  --events-json /Users/happyfewmac/Downloads/response_stalled_car_detected.json \
  --metadata-json traffic_data_analysis/linkVision_rawData/response_its_task_metadata.json \
  --event-id 1521072 \
  --mode compare
```

## TeraSim Replay

Dry-run the TeraSim cases:

```bash
python3 run_linkvision_terasim.py --dry-run --sim-time 40 --mode compare
```

Run through the local TeraSim checkout:

```bash
python3 run_linkvision_terasim.py \
  --terasim-home /Users/happyfewmac/Desktop/Inchor/Terasim \
  --sim-time 1800 \
  --mode compare
```

Outputs are written below `outputs/linkvision_terasim/`.

## Core Files

- `linkvision_terasim/events.py`: event parsing and homography projection.
- `linkvision_terasim/signal_optimizer.py`: fixed signal-plan selection function.
- `linkvision_terasim/sumo_replay.py`: command builder for the existing SUMO demo.
- `linkvision_terasim/terasim_runner.py`: TeraSim environment and replay runner.
- `run_linkvision_replay.py`: offline SUMO replay CLI.
- `run_linkvision_terasim.py`: TeraSim replay CLI.
