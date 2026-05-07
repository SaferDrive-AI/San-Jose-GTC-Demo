import unittest

from main import SUMODelayCalculator


class MainResultsTests(unittest.TestCase):
    def test_calculate_delay_returns_complete_schema_when_no_vehicles_arrive(self):
        calculator = SUMODelayCalculator.__new__(SUMODelayCalculator)
        calculator.arrived_vehicles = set()
        calculator.vehicle_data = {"veh_1": {}, "veh_2": {}}
        calculator.sim_time = 1

        results = calculator.calculate_delay()

        self.assertEqual(results["vehicle_count"], 0)
        self.assertEqual(results["total_departed"], 2)
        self.assertEqual(results["total_arrived"], 0)
        self.assertEqual(results["total_time_loss"], 0)
        self.assertEqual(results["total_wait_time"], 0)
        self.assertEqual(results["simulation_time"], 1)


if __name__ == "__main__":
    unittest.main()
