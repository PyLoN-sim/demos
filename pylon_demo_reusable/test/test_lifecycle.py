"""Exercise actual lifecycle callbacks, correlated results, and recovery offline."""
from copy import deepcopy
import time
import unittest
from unittest.mock import Mock
import rclpy
from rclpy.lifecycle import TransitionCallbackReturn as Result
from pylon_interfaces.msg import (
    FlightState, VesselLifecycle, ControlAuthorityState, EngineState, SeparationState,
    SeparationResult, ControlSnapshot, SimulatorState,
)
from pylon_demo_reusable.node import ReusableMissionNode, sample
from pylon_demo_reusable.mission import Phase, Demand


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

    def identity(self, message, observation=None):
        life = self.n.lifecycle
        message.runtime_instance, message.runtime_epoch = life.runtime_instance, life.runtime_epoch
        message.runtime_generation, message.vessel_id = life.runtime_generation, life.vessel_id
        message.observation_sequence = observation or (self.n.flight.observation_sequence if self.n.flight else 1)
        return message

    def engine(self, **kwargs):
        return self.identity(EngineState(id='engine', max_thrust=1500000., **kwargs))

    def simulator(self, **kwargs):
        defaults = dict(state=SimulatorState.STATE_ADVANCING, communication_alive=True,
                        control_available=True, simulation_advancing=True, warp_rate=1.)
        defaults.update(kwargs)
        message = self.identity(SimulatorState(**defaults))
        message.universal_time = self.n.flight.universal_time if self.n.flight else 0.
        self.n.on_simulator(message)
        return message

    def refresh(self, flight=None, engines=None, separators=None):
        flight = deepcopy(flight or self.n.flight)
        if self.n.flight:
            flight.universal_time = max(flight.universal_time, self.n.flight.universal_time+.05)
        observation = (self.n.flight.observation_sequence+1) if self.n.flight else 1
        self.identity(flight, observation)
        engines = engines if engines is not None else [item[0] for item in self.n.engines.values()]
        separators = separators if separators is not None else [item[0] for item in self.n.separators.values()]
        engines, separators = deepcopy(engines), deepcopy(separators)
        for item in (*engines, *separators):
            self.identity(item, observation)
            item.universal_time = flight.universal_time
        self.n.on_snapshot(ControlSnapshot(flight=flight, engines=engines, separations=separators))
        self.n.on_vessel(self.n.lifecycle)
        self.simulator()
        return flight

    def feed_preflight(self):
        n = self.n
        life = VesselLifecycle(state=VesselLifecycle.STATE_ACTIVE, vessel_id='v', vessel_name='PyLoN Phoenix',
                               runtime_instance='test', runtime_epoch='one', runtime_generation=1, generation=1)
        n.on_vessel(life)
        f = FlightState(vessel_id='v', body_name='Kerbin', universal_time=100., altitude_asl=80.,
                        altitude_agl=12., mass=80000., liquid_fuel=5760., oxidizer=7000.,
                        electric_charge=1000., gravity=9.81, body_radius=600000.,
                        gravitational_parameter=3.5316e12, landed=True)
        f.up_body.x = f.east_body.y = f.north_body.z = 1.
        self.refresh(f, [self.engine()], [SeparationState(id='payload', mechanism='decoupler', available=True)])
        n.on_authority(ControlAuthorityState(vessel_id='v', state=ControlAuthorityState.STATE_UNOWNED))
        return life, n.flight

    def own(self):
        n = self.n
        n.on_authority(ControlAuthorityState(vessel_id='v', state=ControlAuthorityState.STATE_OWNED,
                       controller_id=n.lease.controller_id, lease_id=n.lease.lease_id))
        n.lease.next_request_at = time.monotonic()+.2
        n.tick()

    def test_renewal_and_control_share_one_batch_without_skipping_demands(self):
        self.feed_preflight(); self.n.trigger_activate(); self.own()
        self.n.lease.next_request_at = 0.
        self.n.batch_pub = Mock()
        before = self.n.sequence
        self.n.tick()
        self.assertEqual(self.n.sequence, before+1)
        batch = self.n.batch_pub.publish.call_args[0][0]
        self.assertTrue(batch.renew_lease)
        self.assertTrue(batch.has_flight)
        self.assertEqual(len(batch.engines), 1)
        self.assertGreater(batch.engines[0].target_thrust, 0.)

    def test_inactive_never_acquires_or_actuates(self):
        self.feed_preflight(); self.n.tick()
        self.assertEqual(self.n.sequence, 0)
        self.assertEqual(self.n.mission.phase, Phase.READY)

    def test_activation_is_guarded(self):
        self.assertEqual(self.n.trigger_activate(), Result.FAILURE)
        self.feed_preflight()
        self.assertEqual(self.n.trigger_activate(), Result.SUCCESS)
        self.assertEqual(self.n.mission.phase, Phase.READY)
        self.own()
        self.assertEqual(self.n.mission.phase, Phase.IGNITION)

    def test_deactivation_zeros_in_one_batch_and_releases(self):
        self.feed_preflight(); self.n.trigger_activate(); self.own()
        self.n.batch_pub = Mock()
        before = self.n.sequence
        self.assertEqual(self.n.trigger_deactivate(), Result.SUCCESS)
        self.assertEqual(self.n.sequence-before, 2)
        batch = self.n.batch_pub.publish.call_args[0][0]
        self.assertEqual(batch.engines[0].target_thrust, 0.)
        self.assertEqual((batch.flight.pitch, batch.flight.yaw, batch.flight.roll), (0., 0., 0.))
        self.assertEqual(self.n.mission.phase, Phase.ABORT)
        self.assertFalse(self.n.active)
        self.assertEqual(self.n.trigger_activate(), Result.FAILURE)

    def test_session_change_aborts_without_commanding_new_vessel(self):
        life, _ = self.feed_preflight(); self.n.trigger_activate(); self.own()
        before = self.n.sequence
        changed = deepcopy(life); changed.vessel_id = 'other'; changed.generation = 2
        self.n.on_vessel(changed)
        self.assertFalse(self.n.active)
        self.assertEqual(self.n.mission.reason, 'vessel_or_session_changed')
        self.assertEqual(self.n.sequence, before)
        self.assertIsNone(self.n.flight)

    def test_snapshot_rejects_mixed_epochs_and_reordered_samples(self):
        _, flight = self.feed_preflight()
        mixed = deepcopy(flight); mixed.observation_sequence += 1
        engine = self.engine(); engine.observation_sequence = mixed.observation_sequence
        engine.runtime_epoch = 'other'
        self.n.on_snapshot(ControlSnapshot(flight=mixed, engines=[engine]))
        self.assertIs(self.n.flight, flight)
        self.n.on_snapshot(ControlSnapshot(flight=deepcopy(flight), engines=[]))
        self.assertIn('engine', self.n.engines)

    def test_powered_pause_is_distinct_from_communication_loss(self):
        self.feed_preflight(); self.n.trigger_activate(); self.own()
        self.simulator(paused=True, simulation_advancing=False, state=SimulatorState.STATE_PAUSED)
        self.n.tick()
        self.assertEqual(self.n.mission.reason, 'simulator_paused')
        self.assertFalse(self.n.active)

    def test_powered_communication_loss_aborts(self):
        self.feed_preflight(); self.n.trigger_activate(); self.own()
        self.n.simulator_seen = time.monotonic()-1.
        self.n.tick()
        self.assertEqual(self.n.mission.reason, 'simulator_communication_stale')

    def test_powered_physics_warp_aborts_with_explicit_reason(self):
        self.feed_preflight(); self.n.trigger_activate(); self.own()
        self.simulator(warp_rate=2., physics_warp=True)
        self.n.tick()
        self.assertEqual(self.n.mission.reason, 'simulator_warping')
        self.assertFalse(self.n.active)

    def test_authority_loss_never_reacquires_automatically(self):
        self.feed_preflight(); self.n.trigger_activate(); self.own()
        self.n.on_authority(ControlAuthorityState(vessel_id='v', state=ControlAuthorityState.STATE_UNOWNED))
        before = self.n.sequence; self.n.tick()
        self.assertEqual(self.n.sequence, before)
        self.assertEqual(self.n.mission.reason, 'authority_lost')

    def prepare_vacuum_coast(self):
        life, f = self.feed_preflight(); self.n.trigger_activate(); self.own()
        f = deepcopy(f); f.landed = False; f.altitude_asl = 80000.; f.periapsis = 78000.; f.apoapsis = 82000.
        f = self.refresh(f)
        self.n.mission.transition(Phase.DEORBIT_WAIT, f.universal_time, 'test')
        return life, f

    def prepare_orbital_hold(self):
        life, f = self.prepare_vacuum_coast()
        self.n.flight_seen = time.monotonic()-1.
        self.n.tick()
        self.assertEqual(self.n.mission.phase, Phase.HOLD)
        return life, f

    def test_orbit_holds_when_authority_topic_expires_first(self):
        self.prepare_vacuum_coast()
        self.n.authority_seen = time.monotonic()-.7
        self.n.tick()
        self.assertEqual(self.n.mission.phase, Phase.HOLD)

    def test_expired_lease_in_unpowered_orbit_enters_hold(self):
        self.prepare_vacuum_coast()
        self.n.on_authority(ControlAuthorityState(vessel_id='v', state=ControlAuthorityState.STATE_UNOWNED,
                           reason='lease_expired'))
        self.assertEqual(self.n.mission.phase, Phase.HOLD)

    def test_orbit_resumes_after_fresh_snapshot_and_new_lease(self):
        _, f = self.prepare_orbital_hold(); old_lease = self.n.lease.lease_id
        self.n.on_authority(ControlAuthorityState(vessel_id='v', state=ControlAuthorityState.STATE_UNOWNED))
        self.refresh(f)
        self.n.tick(); self.assertEqual(self.n.lease.lease_id, old_lease)
        self.n.coast_hold['ready_since'] = time.monotonic()-.6
        self.n.tick(); self.assertNotEqual(self.n.lease.lease_id, old_lease)
        self.assertEqual(self.n.mission.phase, Phase.HOLD)
        self.own()
        self.assertEqual(self.n.mission.phase, Phase.DEORBIT_WAIT)
        self.assertIsNone(self.n.coast_hold)

    def test_orbital_hold_rejects_changed_fuel(self):
        _, f = self.prepare_orbital_hold()
        self.n.on_authority(ControlAuthorityState(vessel_id='v', state=ControlAuthorityState.STATE_UNOWNED))
        f = deepcopy(f); f.liquid_fuel -= 1.
        self.refresh(f); self.n.tick()
        self.assertEqual(self.n.mission.reason, 'orbital_hold_state_changed')

    def test_communication_hold_is_bounded_to_five_seconds(self):
        self.prepare_orbital_hold()
        self.n.coast_hold['started'] = time.monotonic()-6.
        self.n.tick()
        self.assertEqual(self.n.mission.reason, 'orbital_telemetry_hold_expired')

    def test_explicit_orbital_pause_can_wait_and_has_operator_bound(self):
        self.prepare_vacuum_coast()
        self.simulator(paused=True, simulation_advancing=False, state=SimulatorState.STATE_PAUSED)
        self.n.tick(); self.assertEqual(self.n.mission.phase, Phase.HOLD)
        self.n.coast_hold['started'] = time.monotonic()-30.
        before = self.n.sequence; self.n.tick()
        self.assertTrue(self.n.active)
        self.assertEqual(self.n.sequence, before)
        self.n.coast_hold['started'] = time.monotonic()-301.
        self.n.tick(); self.assertFalse(self.n.active)

    def test_pause_heartbeat_after_stale_guard_promotes_same_hold_bound(self):
        self.prepare_orbital_hold()
        original_start = self.n.coast_hold['started'] = time.monotonic()-6.
        self.assertEqual(self.n.coast_hold['timeout'], 5.)
        self.simulator(paused=True, simulation_advancing=False, state=SimulatorState.STATE_PAUSED)
        self.n.tick()
        self.assertTrue(self.n.active)
        self.assertEqual(self.n.coast_hold['timeout'], 300.)
        self.assertEqual(self.n.coast_hold['started'], original_start)
        self.assertEqual(self.n.mission.reason, 'simulator_paused')

    def test_communication_loss_during_explicit_pause_keeps_short_bound(self):
        self.prepare_vacuum_coast()
        self.simulator(paused=True, simulation_advancing=False, state=SimulatorState.STATE_PAUSED)
        self.n.tick()
        self.n.simulator_seen = time.monotonic()-1.
        self.n.tick()
        self.n.coast_hold['communication_lost_at'] = time.monotonic()-6.
        self.n.tick()
        self.assertEqual(self.n.mission.reason, 'orbital_communication_hold_expired')

    def test_orbital_hold_never_crosses_sessions(self):
        life, _ = self.prepare_orbital_hold()
        life = deepcopy(life); life.generation = 2; life.runtime_epoch = 'two'
        self.n.on_vessel(life)
        self.assertFalse(self.n.active)

    def test_orbital_hold_yields_to_another_controller(self):
        _, f = self.prepare_orbital_hold(); self.refresh(f)
        self.n.on_authority(ControlAuthorityState(vessel_id='v', state=ControlAuthorityState.STATE_OWNED,
                           controller_id='operator', lease_id='other'))
        self.n.tick()
        self.assertEqual(self.n.mission.reason, 'orbital_hold_authority_conflict')

    def test_preflight_rejects_ambiguous_engine_selection(self):
        self.feed_preflight()
        self.refresh(engines=[self.engine(), self.identity(EngineState(id='another', max_thrust=1500000.))])
        self.assertEqual(self.n.trigger_activate(), Result.FAILURE)

    def test_clamp_release_includes_control_and_waits_for_measured_thrust(self):
        _, flight = self.feed_preflight()
        separators = [item[0] for item in self.n.separators.values()] + [SeparationState(id='support', mechanism='launch_clamp', available=True)]
        self.refresh(separators=separators)
        self.assertEqual(self.n.trigger_activate(), Result.SUCCESS); self.own()
        self.n.batch_pub = Mock()
        self.n.publish_demand(Demand(throttle=1.), 1500000., sample(flight))
        self.assertFalse(self.n.batch_pub.publish.call_args[0][0].has_separation)
        self.refresh(engines=[self.engine(thrust=1000000.)])
        self.n.publish_demand(Demand(throttle=1.), 1500000., sample(flight))
        batch = self.n.batch_pub.publish.call_args[0][0]
        self.assertTrue(batch.has_separation)
        self.assertTrue(batch.has_flight)
        self.assertEqual(batch.separation.id, 'support')
        self.assertTrue(batch.separation.operation_id)
        self.assertGreater(batch.engines[0].target_thrust, 0.)

    def begin_separation(self):
        life, flight = self.feed_preflight(); self.n.trigger_activate(); self.own()
        command = self.n.new_separation('support')
        updated = deepcopy(life); updated.runtime_epoch = 'two'; updated.runtime_generation = 2; updated.generation = 2
        receipt = SeparationResult(operation_id=command.operation_id, original_runtime_instance='test',
            original_runtime_epoch='one', original_vessel_id='v', id='support', controller_id=self.n.lease.controller_id,
            retained=True, completed=True, success=True, reason='separated', result_runtime_epoch='two',
            result_runtime_generation=2, active_vessel_id='v', resulting_vessel_ids=['v', 'support-vessel'])
        return updated, flight, receipt

    def finish_separation_observation(self, life, flight):
        self.n.on_vessel(life)
        self.refresh(flight, [EngineState(id='engine', max_thrust=1500000.)], [])
        self.n.on_authority(ControlAuthorityState(vessel_id='v', state=ControlAuthorityState.STATE_UNOWNED))

    def test_separation_rebinds_on_matching_operation_result_then_acquires_fresh_lease(self):
        life, flight, receipt = self.begin_separation(); old_lease = self.n.lease.lease_id
        self.finish_separation_observation(life, flight)
        self.assertTrue(self.n.rebinding)
        before = self.n.sequence; self.n.tick()
        self.assertEqual(before, self.n.sequence)
        self.n.on_separation_result(receipt); self.n.tick()
        self.assertFalse(self.n.rebinding)
        self.assertIsNone(self.n.pending_operation)
        self.assertNotEqual(self.n.lease.lease_id, old_lease)
        self.assertFalse(self.n.lease.owned)
        self.assertEqual(self.n.sequence, before+1)  # acquire only, no powered demand

    def test_separation_waits_for_new_epoch_simulator_heartbeat(self):
        life, flight, receipt = self.begin_separation()
        self.finish_separation_observation(life, flight)
        self.n.simulator = None  # Snapshot/result can arrive before simulator topic.
        self.n.on_separation_result(receipt)
        before = self.n.sequence
        self.n.tick()
        self.assertTrue(self.n.active)
        self.assertTrue(self.n.rebinding)
        self.assertIsNotNone(self.n.pending_operation)
        self.assertEqual(self.n.sequence, before)
        self.simulator()
        self.n.tick()
        self.assertIsNone(self.n.pending_operation)
        self.assertTrue(self.n.active)
        self.assertEqual(self.n.sequence, before+1)

    def test_result_before_lifecycle_is_retained_until_matching_snapshot(self):
        life, flight, receipt = self.begin_separation()
        self.n.on_separation_result(receipt)
        before = self.n.sequence; self.n.tick()
        self.assertEqual(self.n.sequence, before)
        self.finish_separation_observation(life, flight); self.n.tick()
        self.assertIsNone(self.n.pending_operation)
        self.assertTrue(self.n.active)

    def test_unrelated_result_and_legacy_state_cannot_confirm_separation(self):
        life, flight, receipt = self.begin_separation()
        self.finish_separation_observation(life, flight)
        receipt.operation_id = 'wrong-operation'; self.n.on_separation_result(receipt)
        self.n.on_separation(SeparationState(id='support', separated=True))
        self.n.tick()
        self.assertTrue(self.n.rebinding)
        self.assertIsNotNone(self.n.pending_operation)

    def test_missing_result_is_queried_with_original_identity(self):
        self.begin_separation()
        self.n.result_client = Mock()
        self.n.query_separation_result(time.monotonic())
        request = self.n.result_client.call_async.call_args[0][0]
        self.assertEqual(request.original_runtime_epoch, 'one')
        self.assertEqual(request.operation_id, self.n.pending_operation.operation_id)

    def test_late_query_response_from_previous_operation_cannot_abort_current_operation(self):
        self.begin_separation()
        future = Mock()
        future.result.return_value.found = False
        future.result.return_value.reason = 'result_not_retained'
        self.n.received_result_query(future, 'previous-operation')
        self.assertTrue(self.n.active)

    def test_unknown_result_topic_before_command_acceptance_remains_pending(self):
        life, flight, receipt = self.begin_separation()
        unknown = deepcopy(receipt)
        unknown.retained, unknown.success = False, False
        unknown.reason, unknown.id, unknown.controller_id = 'result_not_retained', '', ''
        self.n.on_separation_result(unknown)
        self.assertTrue(self.n.active)
        self.assertIsNone(self.n.separation_result)
        self.n.on_separation_result(receipt)
        self.finish_separation_observation(life, flight)
        self.n.tick()
        self.assertTrue(self.n.active)
        self.assertIsNone(self.n.pending_operation)

    def test_unknown_service_response_before_command_acceptance_remains_pending(self):
        life, flight, receipt = self.begin_separation()
        operation_id = self.n.pending_operation.operation_id
        future = Mock()
        future.result.return_value.found = False
        future.result.return_value.reason = 'result_not_retained'
        self.n.received_result_query(future, operation_id)
        self.assertTrue(self.n.active)
        self.assertIsNone(self.n.separation_result)
        future.result.return_value.found = True
        future.result.return_value.result = receipt
        self.n.received_result_query(future, operation_id)
        self.finish_separation_observation(life, flight)
        self.n.tick()
        self.assertTrue(self.n.active)
        self.assertIsNone(self.n.pending_operation)

    def test_persistently_unknown_receipt_still_expires_at_existing_deadline(self):
        _, _, receipt = self.begin_separation()
        receipt.retained, receipt.success = False, False
        receipt.reason = 'result_not_retained'
        self.n.on_separation_result(receipt)
        self.n.separation_deadline = time.monotonic()-1.
        self.n.tick()
        self.assertFalse(self.n.active)
        self.assertEqual(self.n.mission.reason, 'separation_result_not_confirmed')

    def test_retained_failure_stops_immediately_without_waiting_for_deadline(self):
        _, _, receipt = self.begin_separation()
        receipt.success, receipt.reason = False, 'completion_not_observed'
        self.n.on_separation_result(receipt)
        self.assertFalse(self.n.active)
        self.assertEqual(self.n.mission.reason, 'separation_failed:completion_not_observed')

    def test_pending_separation_does_not_allow_another_vessel(self):
        life, _, _ = self.begin_separation()
        life.vessel_id = 'other'; self.n.on_vessel(life)
        self.assertFalse(self.n.active)
