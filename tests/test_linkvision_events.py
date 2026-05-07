import tempfile
import unittest
from pathlib import Path

from linkvision_terasim.events import (
    event_to_obstacle_arg,
    iter_stalled_vehicle_events,
    load_camera_calibrations,
    normalized_bbox_to_pixel,
)


class LinkVisionEventTests(unittest.TestCase):
    def test_normalized_bbox_uses_bottom_center_anchor(self):
        pixel = normalized_bbox_to_pixel([0.5156, 0.157, 0.0581, 0.0702], 640, 428)

        self.assertAlmostEqual(pixel[0], 348.576, places=3)
        self.assertAlmostEqual(pixel[1], 97.2416, places=3)

    def test_warning_vehicle_is_projected_to_lat_lon_with_camera_homography(self):
        metadata_path = Path(
            "/Users/happyfewmac/Desktop/Inchor/San-Jose-GTC-Demo/"
            "traffic_data_analysis/linkVision_rawData/response_its_task_metadata.json"
        )
        calibrations = load_camera_calibrations(metadata_path)
        raw_events = [
            {
                "id": 1521072,
                "task_id": 14159,
                "event_name": "Unexpected Stop",
                "timestamp": "2026-02-25T03:32:04.414000Z",
                "image_url": "/image.jpg",
                "video_url": "/video.mp4",
                "detected_objects": [
                    {
                        "object_id": "ignored",
                        "object_name": "car",
                        "coordinates": [0.1, 0.2, 0.3, 0.4],
                        "confidence_score": 0.9,
                        "warning": False,
                    },
                    {
                        "object_id": "895999f5-b416-4d9c-9b2d-4c0373774aaa",
                        "object_name": "car",
                        "coordinates": [0.5156, 0.157, 0.0581, 0.0702],
                        "confidence_score": 0.33,
                        "warning": True,
                    },
                ],
                "location_id": 133,
                "camera_id": 694,
                "timezone": "America/Los_Angeles",
                "coordinates": {"latitude": None, "longitude": None},
            }
        ]

        events = list(iter_stalled_vehicle_events(raw_events, calibrations))

        self.assertEqual(len(events), 1)
        event = events[0]
        self.assertEqual(event.source_event_id, 1521072)
        self.assertEqual(event.object_id, "895999f5-b416-4d9c-9b2d-4c0373774aaa")
        self.assertEqual(event.object_name, "car")
        self.assertAlmostEqual(event.latitude, 37.33538562, places=7)
        self.assertAlmostEqual(event.longitude, -121.89221894, places=7)
        self.assertEqual(event_to_obstacle_arg(event), "37.33538562,-121.89221894")

    def test_missing_camera_metadata_is_reported_as_skipped_event(self):
        raw_events = [
            {
                "id": 1,
                "task_id": 14159,
                "event_name": "Unexpected Stop",
                "timestamp": "2026-02-25T03:32:04.414000Z",
                "detected_objects": [
                    {
                        "object_id": "stalled",
                        "object_name": "truck",
                        "coordinates": [0.5, 0.5, 0.1, 0.1],
                        "confidence_score": 0.8,
                        "warning": True,
                    }
                ],
                "camera_id": 999,
            }
        ]

        events = list(iter_stalled_vehicle_events(raw_events, {}))

        self.assertEqual(events, [])

    def test_can_load_minimal_metadata_file(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            metadata_file = Path(tmpdir) / "metadata.json"
            metadata_file.write_text(
                """
                [
                  {
                    "id": 14159,
                    "camera": {
                      "id": 694,
                      "name": "Inter_Cam_2_Stalled_Car",
                      "resolution_width": 640,
                      "resolution_height": 428,
                      "timezone": "America/Los_Angeles",
                      "homography_matrix": [[1,0,0],[0,1,0],[0,0,1]],
                      "coordinates": {"latitude": 37.0, "longitude": -121.0}
                    }
                  }
                ]
                """,
                encoding="utf-8",
            )

            calibrations = load_camera_calibrations(metadata_file)

        self.assertEqual(calibrations[694].task_id, 14159)
        self.assertEqual(calibrations[694].resolution_width, 640)
        self.assertEqual(calibrations[694].resolution_height, 428)


if __name__ == "__main__":
    unittest.main()
