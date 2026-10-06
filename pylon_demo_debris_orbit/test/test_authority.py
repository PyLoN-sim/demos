import unittest

from debris_orbit.authority import SharedAuthority


class AuthorityTests(unittest.TestCase):
    def setUp(self):
        self.authority = SharedAuthority(0.5)
        self.authority.observe_vessel("chaser", True, 4)

    def test_acquire_then_renew_shared_authority(self):
        self.assertEqual(self.authority.due_action(0).action, "acquire")
        self.assertIsNone(self.authority.due_action(0.2))
        self.assertFalse(self.authority.observe_authority("chaser", True, 4))
        self.assertEqual(self.authority.due_action(0.5).action, "renew")
        self.assertTrue(self.authority.observe_authority("chaser", False, 4))

    def test_same_vessel_new_generation_retires_control(self):
        self.authority.due_action(0)
        self.authority.observe_authority("chaser", True, 4)
        self.assertTrue(self.authority.observe_vessel("chaser", True, 5))
        self.authority.observe_authority("chaser", True, 4)
        self.assertFalse(self.authority.owned)
        self.assertIsNone(self.authority.release_action())

    def test_release_cancels_pending_acquire(self):
        self.authority.due_action(0)
        self.assertEqual(self.authority.release_action().action, "release")
        self.assertIsNone(self.authority.release_action())

    def test_idle_demo_does_not_release_another_nodes_shared_authority(self):
        self.authority.observe_authority("chaser", True, 4)
        self.assertIsNone(self.authority.release_action())
        # An explicit acquire sets this demo's SAS policy even if PyLoN owns it.
        self.assertEqual(self.authority.due_action(0).action, "acquire")
