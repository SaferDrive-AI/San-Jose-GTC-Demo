import unittest

from linkvision_terasim.signal_optimizer import (
    DEFAULT_TLS_ID,
    choose_signal_plan,
)


class SignalOptimizerTests(unittest.TestCase):
    def test_exact_lane_program_wins_when_available(self):
        decision = choose_signal_plan(
            obstacle_lane_id="-416901209#1_2",
            available_programs=["org", "opt", "-416901209#1_2"],
        )

        self.assertEqual(decision.tls_id, DEFAULT_TLS_ID)
        self.assertEqual(decision.program_id, "-416901209#1_2")
        self.assertEqual(decision.reason, "exact_lane_program")

    def test_edge_program_is_used_when_lane_specific_plan_is_missing(self):
        decision = choose_signal_plan(
            obstacle_lane_id="-416901209#1_0",
            available_programs=["org", "opt", "-416901209#1_2"],
        )

        self.assertEqual(decision.program_id, "-416901209#1_2")
        self.assertEqual(decision.reason, "edge_program")

    def test_westbound_obstacle_uses_fixed_optimal_plan_when_no_lane_program_exists(self):
        decision = choose_signal_plan(
            direction="WB",
            available_programs=["org", "opt", "1418903639#0_2", "wb_plan_3"],
        )

        self.assertEqual(decision.program_id, "1418903639#0_2")
        self.assertEqual(decision.reason, "directional_optimal_plan")

    def test_unknown_obstacle_falls_back_to_opt(self):
        decision = choose_signal_plan(
            obstacle_lane_id="unknown_0",
            available_programs=["org", "opt"],
        )

        self.assertEqual(decision.program_id, "opt")
        self.assertEqual(decision.reason, "fallback_opt")


if __name__ == "__main__":
    unittest.main()
