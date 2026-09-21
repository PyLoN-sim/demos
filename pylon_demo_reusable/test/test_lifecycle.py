"""Exercise real rclpy lifecycle callbacks and fault transitions, offline."""
import time
import unittest
import rclpy
from rclpy.lifecycle import TransitionCallbackReturn as Result
from pylon_interfaces.msg import FlightState, VesselLifecycle, ControlAuthorityState, EngineState, SeparationState
from pylon_demo_reusable.node import ReusableMissionNode
from pylon_demo_reusable.mission import Phase


class LifecycleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.n = ReusableMissionNode()
        self.assertEqual(self.n.trigger_configure(), Result.SUCCESS)

    def tearDown(self):
        self.n.stop('test_end')
        self.n.destroy_node()

    def feed_preflight(self):
        n = self.n
        life = VesselLifecycle(state=VesselLifecycle.STATE_ACTIVE, vessel_id='v', vessel_name='PyLoN Phoenix',
                               runtime_instance='test', runtime_epoch='one', generation=1)
        n.on_vessel(life)
        f = FlightState(vessel_id='v', body_name='Kerbin', universal_time=100., altitude_asl=80.,
                        altitude_agl=12., mass=80000., liquid_fuel=5760., oxidizer=7000.,
                        electric_charge=1000., gravity=9.81, body_radius=600000.,
                        gravitational_parameter=3.5316e12, landed=True)
        f.up_body.x = f.east_body.y = f.north_body.z = 1.
        n.on_flight(f)
        n.on_engine(EngineState(id='engine', max_thrust=1500000.))
        n.on_separation(SeparationState(id='payload', mechanism='decoupler', available=True))
        n.on_authority(ControlAuthorityState(vessel_id='v', state=ControlAuthorityState.STATE_UNOWNED))
        return life, f

    def own(self):
        n = self.n
        n.on_authority(ControlAuthorityState(vessel_id='v', state=ControlAuthorityState.STATE_OWNED,
                       controller_id=n.lease.controller_id, lease_id=n.lease.lease_id))
        n.lease.next_request_at = time.monotonic()+.2
        n.tick()

    def test_heartbeat_has_its_own_tick(self):
        self.feed_preflight(); self.n.trigger_activate(); self.own()
        self.n.lease.next_request_at = 0.
        before = self.n.sequence
        self.n.tick()
        self.assertEqual(self.n.sequence, before+1)

    def test_inactive_never_acquires_or_actuates(self):
        self.feed_preflight()
        self.n.tick()
        self.assertEqual(self.n.sequence, 0)
        self.assertEqual(self.n.mission.phase, Phase.READY)

    def test_activation_is_guarded(self):
        self.assertEqual(self.n.trigger_activate(), Result.FAILURE)
        self.feed_preflight()
        self.assertEqual(self.n.trigger_activate(), Result.SUCCESS)
        self.assertTrue(self.n.active)
        self.assertEqual(self.n.mission.phase, Phase.READY)
        self.own()
        self.assertEqual(self.n.mission.phase, Phase.IGNITION)

    def test_deactivation_cuts_off_and_requires_reconfigure(self):
        self.feed_preflight(); self.n.trigger_activate(); self.own()
        before = self.n.sequence
        self.assertEqual(self.n.trigger_deactivate(), Result.SUCCESS)
        self.assertEqual(self.n.sequence-before, 3)  # engine zero, inputs zero, release
        self.assertEqual(self.n.mission.phase, Phase.ABORT)
        self.assertFalse(self.n.active)
        self.assertEqual(self.n.trigger_activate(), Result.FAILURE)

    def test_session_change_latches_abort_without_commanding_new_vessel(self):
        life, _ = self.feed_preflight(); self.n.trigger_activate(); self.own()
        before = self.n.sequence
        life.vessel_id = 'other'; life.generation = 2
        self.n.on_vessel(life)
        self.assertFalse(self.n.active)
        self.assertEqual(self.n.mission.reason, 'vessel_or_session_changed')
        self.assertEqual(self.n.sequence, before)
        self.assertIsNone(self.n.flight)

    def test_stale_telemetry_and_pause_cut_off(self):
        self.feed_preflight(); self.n.trigger_activate(); self.own()
        self.n.progress_seen = time.monotonic()-3.
        self.n.tick()
        self.assertEqual(self.n.mission.phase, Phase.ABORT)
        self.assertFalse(self.n.active)

    def test_authority_loss_never_reacquires_automatically(self):
        self.feed_preflight(); self.n.trigger_activate(); self.own()
        self.n.on_authority(ControlAuthorityState(vessel_id='v', state=ControlAuthorityState.STATE_UNOWNED))
        before = self.n.sequence
        self.n.tick()
        self.assertEqual(self.n.sequence, before)
        self.assertEqual(self.n.mission.reason, 'authority_lost')
        self.assertFalse(self.n.active)

    def prepare_vacuum_coast(self):
        life, f = self.feed_preflight(); self.n.trigger_activate(); self.own()
        f.landed = False; f.altitude_asl = 80000.; f.periapsis = 78000.
        self.n.on_flight(f)
        self.n.mission.transition(Phase.DEORBIT_WAIT, f.universal_time, 'test')
        return life, f

    def prepare_orbital_hold(self):
        life, f = self.prepare_vacuum_coast()
        self.n.flight_seen = time.monotonic()-1.
        self.n.tick()
        self.assertEqual(self.n.mission.phase, Phase.HOLD)
        self.assertTrue(self.n.active)
        return life, f

    def test_orbit_holds_when_authority_telemetry_is_first_to_expire(self):
        self.prepare_vacuum_coast()
        now = time.monotonic()
        self.n.flight_seen = self.n.lifecycle_seen = now-.5
        self.n.authority_seen = now-.7
        self.n.tick()
        self.assertEqual(self.n.mission.phase, Phase.HOLD)
        self.assertTrue(self.n.active)

    def test_expired_lease_in_unpowered_orbit_enters_hold(self):
        self.prepare_vacuum_coast()
        self.n.on_authority(ControlAuthorityState(vessel_id='v', state=ControlAuthorityState.STATE_UNOWNED,
                           reason='lease_expired'))
        self.assertEqual(self.n.mission.phase, Phase.HOLD)
        self.assertTrue(self.n.active)

    def test_unpowered_orbit_recovers_with_a_new_lease_after_freshness_gate(self):
        life, f = self.prepare_orbital_hold()
        old_lease = self.n.lease.lease_id
        self.n.on_authority(ControlAuthorityState(vessel_id='v', state=ControlAuthorityState.STATE_UNOWNED))
        self.n.on_vessel(life); self.n.on_flight(f)
        self.n.on_engine(EngineState(id='engine', max_thrust=1500000.))
        self.n.tick()
        self.assertEqual(self.n.lease.lease_id, old_lease)
        self.n.coast_hold['ready_since'] = time.monotonic()-.6
        self.n.tick()
        self.assertNotEqual(self.n.lease.lease_id, old_lease)
        self.assertEqual(self.n.mission.phase, Phase.HOLD)
        self.own()
        self.assertEqual(self.n.mission.phase, Phase.DEORBIT_WAIT)
        self.assertIsNone(self.n.coast_hold)

    def test_orbital_hold_is_bounded_and_does_not_resume_changed_fuel(self):
        life, f = self.prepare_orbital_hold()
        self.n.on_authority(ControlAuthorityState(vessel_id='v', state=ControlAuthorityState.STATE_UNOWNED))
        f.liquid_fuel -= 1.
        self.n.on_vessel(life); self.n.on_flight(f)
        self.n.on_engine(EngineState(id='engine', max_thrust=1500000.))
        self.n.tick()
        self.assertEqual(self.n.mission.reason, 'orbital_hold_state_changed')

    def test_orbital_hold_timeout_aborts(self):
        self.prepare_orbital_hold()
        self.n.coast_hold['started'] = time.monotonic()-6.
        self.n.tick()
        self.assertEqual(self.n.mission.reason, 'orbital_telemetry_hold_expired')

    def test_orbital_hold_never_crosses_sessions(self):
        life, _ = self.prepare_orbital_hold()
        life.generation = 2
        self.n.on_vessel(life)
        self.assertFalse(self.n.active)
        self.assertEqual(self.n.mission.reason, 'vessel_or_session_changed')

    def test_orbital_hold_yields_to_another_controller(self):
        life, f = self.prepare_orbital_hold()
        self.n.on_vessel(life); self.n.on_flight(f)
        self.n.on_engine(EngineState(id='engine', max_thrust=1500000.))
        self.n.on_authority(ControlAuthorityState(vessel_id='v', state=ControlAuthorityState.STATE_OWNED,
                           controller_id='operator', lease_id='other'))
        self.n.tick()
        self.assertFalse(self.n.active)
        self.assertEqual(self.n.mission.reason, 'orbital_hold_authority_conflict')

    def test_preflight_rejects_ambiguous_engine_selection(self):
        self.feed_preflight()
        self.n.on_engine(EngineState(id='another', max_thrust=1500000.))
        self.assertEqual(self.n.trigger_activate(), Result.FAILURE)

    def test_launch_clamps_wait_for_measured_thrust(self):
        from unittest.mock import Mock
        from pylon_demo_reusable.node import sample
        from pylon_demo_reusable.mission import Demand
        _, flight = self.feed_preflight()
        self.n.on_separation(SeparationState(id='support', mechanism='launch_clamp', available=True))
        self.assertEqual(self.n.trigger_activate(), Result.SUCCESS)
        self.own()
        publisher = Mock()
        self.n.separation_pub = publisher
        self.n.publish_demand(Demand(throttle=1.), 1500000., sample(flight))
        publisher.publish.assert_not_called()
        self.n.on_engine(EngineState(id='engine', max_thrust=1500000., thrust=1000000.))
        self.n.publish_demand(Demand(throttle=1.), 1500000., sample(flight))
        self.assertEqual(publisher.publish.call_args[0][0].id, 'support')

    def test_expected_separation_rebinds_only_after_receipt(self):
        life, flight = self.feed_preflight(); self.n.trigger_activate(); self.own()
        self.n.pending_separation = 'support'
        self.n.separation_deadline = time.monotonic()+3
        updated = VesselLifecycle(state=life.state, vessel_id='v', vessel_name='PyLoN Phoenix',
            runtime_instance='test', runtime_epoch='two', runtime_generation=1, generation=2)
        self.n.on_vessel(updated)
        self.assertTrue(self.n.rebinding)
        self.assertTrue(self.n.active)
        before = self.n.sequence
        self.n.tick()
        self.assertEqual(before, self.n.sequence)
        self.n.on_flight(flight)
        self.n.on_engine(EngineState(id='engine', max_thrust=1500000.))
        self.n.on_authority(ControlAuthorityState(vessel_id='v', state=ControlAuthorityState.STATE_UNOWNED))
        self.n.on_separation(SeparationState(id='support', mechanism='launch_clamp', separated=True))
        self.n.tick()
        self.assertFalse(self.n.rebinding)
        self.assertTrue(self.n.active)
        self.assertGreater(self.n.sequence, before)

    def test_pending_separation_does_not_allow_another_vessel(self):
        life, _ = self.feed_preflight(); self.n.trigger_activate(); self.own()
        self.n.pending_separation = 'support'
        self.n.separation_deadline = time.monotonic()+3
        self.n.on_vessel(VesselLifecycle(state=life.state, vessel_id='other', runtime_instance='test',
            runtime_epoch='two', runtime_generation=1, generation=2))
        self.assertFalse(self.n.active)
