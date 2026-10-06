import unittest

from debris_orbit.runner import parameter_sets, parser


class RunnerTests(unittest.TestCase):
    def test_default_config_wires_three_nodes_to_estimates(self):
        values, prefix, token, root = parameter_sets(parser().parse_args([]))
        self.assertFalse(values["guidance"]["enabled"])
        self.assertEqual(prefix, "/ksp_vessel")
        self.assertEqual(token, "demo_vehicle")
        self.assertEqual(values["guidance"]["target_topic"], values["recognition"]["target_topic"])
        for name in ("pose_topic", "twist_topic", "setpoint_topic", "world_frame"):
            self.assertEqual(values["guidance"][name], values["controller"][name])
        self.assertEqual(values["controller"]["body_twist_topic"], root + "/navigation/twist_body")
        self.assertTrue(values["controller"]["extrapolate_setpoint"])
        self.assertEqual(values["controller"]["max_force"], 2000.0)

    def test_cli_overrides_are_propagated_to_both_estimation_and_guidance(self):
        args = parser().parse_args(["--enabled", "--instance", "Test-A", "--orbit-radius", "25",
                                   "--prefix", "/experiment", "--lidar-sensor-id", "other_lidar"])
        values, _, token, root = parameter_sets(args)
        self.assertEqual(token, "test_a")
        self.assertTrue(values["guidance"]["enabled"])
        for role in ("guidance", "recognition"):
            self.assertEqual(values[role]["orbit_radius"], 25.0)
            self.assertEqual(values[role]["lidar_sensor_id"], "other_lidar")
        for parameters in values.values():
            self.assertEqual(parameters["world_frame"], "pylon_debris_inertial_test_a")
        self.assertEqual(values["controller"]["controller_status_topic"], root + "/controller_status")
