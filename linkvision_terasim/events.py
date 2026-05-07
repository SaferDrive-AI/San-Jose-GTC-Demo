from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Iterator, Mapping, Sequence


DEFAULT_VEHICLE_OBJECT_NAMES = frozenset(
    {"car", "truck", "bus", "suv", "pickup", "van", "motorcycle"}
)


@dataclass(frozen=True)
class CameraCalibration:
    task_id: int
    camera_id: int
    name: str
    resolution_width: int
    resolution_height: int
    homography_matrix: tuple[tuple[float, float, float], ...]
    latitude: float | None = None
    longitude: float | None = None
    timezone: str | None = None


@dataclass(frozen=True)
class RealWorldStalledVehicleEvent:
    source_event_id: int
    task_id: int
    event_name: str
    timestamp: str
    camera_id: int
    location_id: int | None
    timezone: str | None
    object_id: str
    object_name: str
    confidence_score: float | None
    bbox: tuple[float, float, float, float]
    pixel_x: float
    pixel_y: float
    latitude: float
    longitude: float
    image_url: str | None = None
    video_url: str | None = None


def load_json(path: str | Path):
    with Path(path).open("r", encoding="utf-8") as handle:
        return json.load(handle)


def _as_homography(matrix: Sequence[Sequence[float]]) -> tuple[tuple[float, float, float], ...]:
    if len(matrix) != 3 or any(len(row) != 3 for row in matrix):
        raise ValueError("homography_matrix must be a 3x3 matrix")
    return tuple(tuple(float(value) for value in row) for row in matrix)


def load_camera_calibrations(metadata_path: str | Path) -> dict[int, CameraCalibration]:
    raw_metadata = load_json(metadata_path)
    calibrations: dict[int, CameraCalibration] = {}

    for task in raw_metadata:
        camera = task.get("camera") or {}
        camera_id = camera.get("id")
        homography = camera.get("homography_matrix")
        if camera_id is None or homography is None:
            continue

        coordinates = camera.get("coordinates") or {}
        location = task.get("location") or {}
        calibrations[int(camera_id)] = CameraCalibration(
            task_id=int(task["id"]),
            camera_id=int(camera_id),
            name=str(camera.get("name") or ""),
            resolution_width=int(camera["resolution_width"]),
            resolution_height=int(camera["resolution_height"]),
            homography_matrix=_as_homography(homography),
            latitude=coordinates.get("latitude"),
            longitude=coordinates.get("longitude"),
            timezone=camera.get("timezone") or location.get("timezone"),
        )

    return calibrations


def normalized_bbox_to_pixel(
    bbox: Sequence[float],
    resolution_width: int,
    resolution_height: int,
    anchor: str = "bottom_center",
) -> tuple[float, float]:
    if len(bbox) != 4:
        raise ValueError("bbox must contain [x, y, width, height]")

    x, y, width, height = [float(value) for value in bbox]
    if anchor == "center":
        return (x + width / 2.0) * resolution_width, (y + height / 2.0) * resolution_height
    if anchor == "bottom_center":
        return (x + width / 2.0) * resolution_width, (y + height) * resolution_height
    raise ValueError(f"unsupported bbox anchor: {anchor}")


def apply_homography(
    homography_matrix: Sequence[Sequence[float]],
    pixel_x: float,
    pixel_y: float,
) -> tuple[float, float]:
    """Project an image point to (latitude, longitude).

    LinkVision metadata stores the first homography row as longitude and the
    second row as latitude.
    """

    matrix = _as_homography(homography_matrix)
    denominator = matrix[2][0] * pixel_x + matrix[2][1] * pixel_y + matrix[2][2]
    if denominator == 0:
        raise ValueError("homography projection denominator is zero")

    longitude = (matrix[0][0] * pixel_x + matrix[0][1] * pixel_y + matrix[0][2]) / denominator
    latitude = (matrix[1][0] * pixel_x + matrix[1][1] * pixel_y + matrix[1][2]) / denominator
    return latitude, longitude


def _select_warning_vehicle(
    detected_objects: Iterable[Mapping],
    vehicle_object_names: set[str],
    warning_only: bool,
    min_confidence: float,
) -> Mapping | None:
    candidates = []
    for detected_object in detected_objects:
        object_name = str(detected_object.get("object_name") or "").lower()
        if object_name not in vehicle_object_names:
            continue
        if warning_only and not detected_object.get("warning"):
            continue
        confidence = detected_object.get("confidence_score")
        if confidence is not None and float(confidence) < min_confidence:
            continue
        candidates.append(detected_object)

    if not candidates:
        return None
    return max(candidates, key=lambda item: float(item.get("confidence_score") or 0.0))


def iter_stalled_vehicle_events(
    raw_events: Iterable[Mapping],
    camera_calibrations: Mapping[int, CameraCalibration],
    event_name: str = "Unexpected Stop",
    warning_only: bool = True,
    vehicle_object_names: Iterable[str] = DEFAULT_VEHICLE_OBJECT_NAMES,
    min_confidence: float = 0.0,
) -> Iterator[RealWorldStalledVehicleEvent]:
    vehicle_names = {name.lower() for name in vehicle_object_names}

    for raw_event in raw_events:
        if raw_event.get("event_name") != event_name:
            continue

        camera_id = raw_event.get("camera_id")
        if camera_id is None:
            continue
        calibration = camera_calibrations.get(int(camera_id))
        if calibration is None:
            continue

        detected_object = _select_warning_vehicle(
            raw_event.get("detected_objects") or [],
            vehicle_names,
            warning_only=warning_only,
            min_confidence=min_confidence,
        )
        if detected_object is None:
            continue

        bbox = tuple(float(value) for value in detected_object.get("coordinates") or [])
        if len(bbox) != 4:
            continue

        pixel_x, pixel_y = normalized_bbox_to_pixel(
            bbox,
            calibration.resolution_width,
            calibration.resolution_height,
        )
        latitude, longitude = apply_homography(calibration.homography_matrix, pixel_x, pixel_y)

        yield RealWorldStalledVehicleEvent(
            source_event_id=int(raw_event["id"]),
            task_id=int(raw_event.get("task_id") or calibration.task_id),
            event_name=str(raw_event.get("event_name") or ""),
            timestamp=str(raw_event.get("timestamp") or ""),
            camera_id=int(camera_id),
            location_id=raw_event.get("location_id"),
            timezone=raw_event.get("timezone") or calibration.timezone,
            object_id=str(detected_object.get("object_id") or ""),
            object_name=str(detected_object.get("object_name") or ""),
            confidence_score=(
                None
                if detected_object.get("confidence_score") is None
                else float(detected_object["confidence_score"])
            ),
            bbox=bbox,
            pixel_x=pixel_x,
            pixel_y=pixel_y,
            latitude=latitude,
            longitude=longitude,
            image_url=raw_event.get("image_url"),
            video_url=raw_event.get("video_url"),
        )


def event_to_obstacle_arg(event: RealWorldStalledVehicleEvent) -> str:
    return f"{event.latitude:.8f},{event.longitude:.8f}"


def events_to_obstacle_arg(events: Iterable[RealWorldStalledVehicleEvent]) -> str:
    return ";".join(event_to_obstacle_arg(event) for event in events)

