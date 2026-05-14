"""Unit tests for TeraSim replay runner configuration and I/O helpers."""

import tempfile
import unittest
from pathlib import Path
from xml.etree import ElementTree as ET

from linkvision_terasim.terasim_runner import prepare_sumolib_compatible_net_file


class TeraSimRunnerTests(unittest.TestCase):
    def test_prepare_sumolib_compatible_net_file_rounds_float_phase_durations(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp = Path(tmpdir)
            net_file = tmp / "input.net.xml"
            net_file.write_text(
                """<?xml version="1.0" encoding="UTF-8"?>
                <net>
                  <tlLogic id="tls" programID="org">
                    <phase duration="26.40" state="GGrr"/>
                    <phase duration="5" state="yyrr"/>
                  </tlLogic>
                </net>
                """,
                encoding="utf-8",
            )

            compatible = prepare_sumolib_compatible_net_file(net_file, tmp / "out")

            self.assertNotEqual(compatible, net_file)
            root = ET.parse(compatible).getroot()
            durations = [phase.get("duration") for phase in root.iter("phase")]

        self.assertEqual(durations, ["26", "5"])


if __name__ == "__main__":
    unittest.main()
