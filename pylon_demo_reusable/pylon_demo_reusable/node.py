"""Lifecycle-managed flight executive with explicit lease and freshness guards."""
from dataclasses import asdict
import json
import math
import time
import uuid

import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn as Result
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from pylon_interfaces.msg import (
    ControlAuthorityCommand, ControlAuthorityState, FlightControlCommand,
    EngineCommand, SeparationCommand, SeparationResult, SimulatorState,
    ControlBatch, ControlSnapshot, VesselLifecycle,
)
from pylon_interfaces.srv import GetSeparationResult
from pylon_vehicle_control.application.lease import LeaseCoordinator
from pylon_vehicle_control.application.checkpoint import MissionCheckpoint, validate_resume
from .mission import Config, Mission, Sample, Demand, Phase, attitude_inputs, dot


def vector(v):
    return (v.x, v.y, v.z)


def sample(message):
    return Sample(
        time=message.universal_time, altitude=message.altitude_asl, agl=message.altitude_agl,
        mass=message.mass, fuel=message.liquid_fuel, gravity=message.gravity,
        vertical_speed=message.vertical_speed, horizontal_speed=message.horizontal_speed,
        apoapsis=message.apoapsis, periapsis=message.periapsis,
        time_to_apoapsis=message.time_to_apoapsis, latitude=message.latitude,
        longitude=message.longitude, radius=message.body_radius,
        mu=message.gravitational_parameter, dynamic_pressure=message.dynamic_pressure,
        landed=message.landed, splashed=message.splashed,
        up=vector(message.up_body), east=vector(message.east_body), north=vector(message.north_body),
        velocity=vector(message.surface_velocity_body), orbital_velocity=vector(message.orbital_velocity_body),
        angular_velocity=vector(message.angular_velocity_body),
    )


class ReusableMissionNode(LifecycleNode):
    def __init__(self):
        super().__init__('reusable_mission')
        for name, value in asdict(Config()).items():
            self.declare_parameter(name, value)
        self.declare_parameter('vessel_name', 'PyLoN Phoenix')
        self.declare_parameter('telemetry_timeout', .6)
        self.declare_parameter('controller_id', 'pylon_reusable')
        self.config = Config()
        self.mission = Mission(self.config)
        self.active = False
        self.configured = False
        self.lease = LeaseCoordinator('pylon_reusable', .2)
        self.sequence = 0
        self.flight = None
        self.flight_seen = 0.
        self.progress_seen = 0.
        self.lifecycle_seen = 0.
        self.authority_seen = 0.
        self.lifecycle = None
        self.authority = None
        self.engines = {}
        self.separators = {}
        self.engine_id = self.separator_id = ''
        self.activation_time = 0.
        self.ever_owned = False
        self.last_event = 0
        self.last_status = 0.
        self.next_clamp_release = 0.
        self.pending_separation = ''
        self.separation_deadline = 0.
        self.rebinding = False
        self.coast_hold = None
        self.simulator = None
        self.simulator_seen = 0.
        self.pending_operation = None
        self.separation_result = None
        self.next_result_query = 0.
        self.result_query_future = None
        root = '/ksp_vessel'
        durable = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, '~/status', durable)
        self.event_pub = self.create_publisher(String, '~/events', durable)
        self.diagnostic_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        # The final zero command and release must be sent before deactivation
        # disables these lifecycle publishers.
        self.authority_pub = self.create_lifecycle_publisher(ControlAuthorityCommand, root+'/control/authority/command', 10)
        self.batch_pub = self.create_lifecycle_publisher(ControlBatch, root+'/control/batch', 10)
        self.create_subscription(VesselLifecycle, root+'/lifecycle', self.on_vessel, durable)
        self.create_subscription(ControlAuthorityState, root+'/control/authority/state', self.on_authority, durable)
        self.create_subscription(ControlSnapshot, root+'/control/snapshot', self.on_snapshot, qos_profile_sensor_data)
        self.create_subscription(SimulatorState, root+'/simulator/state', self.on_simulator, durable)
        self.create_subscription(SeparationResult, root+'/actuators/separation/result', self.on_separation_result, durable)
        self.result_client = self.create_client(GetSeparationResult, root+'/actuators/separation/get_result')
        self.create_service(Trigger, '~/abort', self.on_abort_request)
        self.create_timer(.05, self.tick)

    def on_configure(self, state):
        try:
            self.config = Config(**{name: self.get_parameter(name).value for name in asdict(Config())})
            timeout = self.get_parameter('telemetry_timeout').value
            if not math.isfinite(timeout) or not .1 <= timeout <= 1.:
                raise ValueError('telemetry_timeout must be in [0.1, 1.0]')
            self.lease = LeaseCoordinator(self.get_parameter('controller_id').value, .2)
            if self.lifecycle:
                self.observe_lease_vessel(self.lifecycle)
            self.mission = Mission(self.config)
            self.coast_hold = None
            self.last_event = 0
            self.configured = True
            return Result.SUCCESS
        except (ValueError, TypeError) as exc:
            self.get_logger().error(str(exc))
            return Result.FAILURE

    def preflight_error(self, now):
        timeout = self.get_parameter('telemetry_timeout').value
        if not self.lifecycle or not self.lease.vessel_id or now-self.lifecycle_seen > timeout:
            return 'waiting_for_active_vessel'
        if self.lifecycle.vessel_name != self.get_parameter('vessel_name').value:
            return 'unexpected_vessel_name'
        if not self.flight or self.flight.vessel_id != self.lease.vessel_id or now-self.flight_seen > timeout:
            return 'waiting_for_flight_state'
        if self.simulator_issue(now):
            return 'waiting_for_advancing_simulator'
        if self.flight.body_name != 'Kerbin' or not self.flight.landed or self.flight.splashed:
            return 'requires_Kerbin_launchpad'
        if self.flight.liquid_fuel <= 0 or self.flight.oxidizer <= 0 or self.flight.electric_charge < 5:
            return 'fuel_or_power_unavailable'
        engines = [key for key, (_, seen) in self.engines.items() if now-seen < timeout]
        separators = [key for key, (s, seen) in self.separators.items()
                      if now-seen < timeout and s.mechanism == 'decoupler' and s.available and not s.separated]
        if len(engines) != 1 or len(separators) != 1:
            return 'requires_exactly_one_engine_and_payload_decoupler'
        if self.authority is None or now-self.authority_seen > timeout:
            return 'waiting_for_authority_state'
        if self.authority.state != ControlAuthorityState.STATE_UNOWNED:
            return 'control_authority_unavailable'
        return ''

    def on_activate(self, state):
        now = time.monotonic()
        error = self.preflight_error(now)
        if error or self.mission.phase != Phase.READY:
            self.get_logger().error(error or 'cleanup_and_configure_required')
            return Result.FAILURE
        self.engine_id = next(key for key, (_, seen) in self.engines.items()
                              if now-seen < self.get_parameter('telemetry_timeout').value)
        self.separator_id = next(key for key, (s, seen) in self.separators.items()
                                 if s.available and not s.separated and s.mechanism == 'decoupler'
                                 and now-seen < self.get_parameter('telemetry_timeout').value)
        self.active, self.ever_owned = True, False
        self.pending_separation = ''
        self.pending_operation = self.separation_result = None
        self.rebinding = False
        self.next_clamp_release = 0.
        self.activation_time = now
        self.lease.next_request_at = 0.
        self.progress_seen = now
        super().on_activate(state)
        return Result.SUCCESS

    def on_deactivate(self, state):
        self.stop('operator_deactivated')
        return super().on_deactivate(state)

    def on_cleanup(self, state):
        self.stop('cleanup')
        self.configured = False
        self.mission = Mission(self.config)
        self.last_event = 0
        return Result.SUCCESS

    def on_shutdown(self, state):
        self.stop('shutdown')
        return super().on_shutdown(state)

    def on_error(self, state):
        self.stop('lifecycle_error')
        return Result.SUCCESS

    def observe_lease_vessel(self, message):
        active = message.state in (message.STATE_ACTIVE, message.STATE_CHANGED)
        return self.lease.observe_vessel(message.vessel_id, active,
                                        (message.runtime_instance, message.runtime_epoch, message.generation))

    def on_vessel(self, message):
        old = self.lifecycle
        expected = (self.active and self.pending_separation and time.monotonic() < self.separation_deadline
                    and old is not None and message.vessel_id == old.vessel_id
                    and message.runtime_instance == old.runtime_instance
                    and message.runtime_generation == old.runtime_generation+1
                    and message.state in (message.STATE_ACTIVE, message.STATE_CHANGED))
        changed = self.observe_lease_vessel(message)
        self.lifecycle, self.lifecycle_seen = message, time.monotonic()
        if changed:
            if expected:
                self.rebinding = True
                self.ever_owned = False
                self.activation_time = time.monotonic()
                self.get_logger().info('Awaiting separation receipt in the new flight epoch')
            elif self.active:
                self.stop('vessel_or_session_changed')
            self.flight = None
            self.engines.clear()
            self.separators.clear()
            self.authority = None
            self.simulator = None

    def on_snapshot(self, message):
        life = self.lifecycle
        if life is None:
            return
        flight = message.flight
        identity = lambda item: (item.runtime_instance, item.runtime_epoch, item.runtime_generation,
                                 item.vessel_id, item.observation_sequence)
        if identity(flight)[:4] != (life.runtime_instance, life.runtime_epoch, life.runtime_generation, life.vessel_id):
            return
        if any(identity(item) != identity(flight) for item in (*message.engines, *message.separations)):
            return
        if self.flight is not None and flight.observation_sequence <= self.flight.observation_sequence:
            return
        self.on_flight(flight)
        now = self.flight_seen
        self.engines = {item.id: (item, now) for item in message.engines}
        self.separators = {item.id: (item, now) for item in message.separations}

    def on_simulator(self, message):
        life = self.lifecycle
        if life is None or (message.runtime_instance, message.runtime_epoch, message.vessel_id) != (
                life.runtime_instance, life.runtime_epoch, life.vessel_id):
            return
        self.simulator, self.simulator_seen = message, time.monotonic()

    def simulator_issue(self, now):
        simulator = self.simulator
        timeout = self.get_parameter('telemetry_timeout').value
        if simulator is None or now-self.simulator_seen > timeout or not simulator.communication_alive:
            return 'simulator_communication_stale'
        if simulator.paused:
            return 'simulator_paused'
        if simulator.warp_rate != 1.:
            return 'simulator_warping'
        if simulator.packed or not simulator.control_available:
            return 'simulator_control_unavailable'
        if simulator.state == SimulatorState.STATE_STALLED:
            return 'simulation_stalled'
        return ''

    def on_separation_result(self, result):
        operation = self.pending_operation
        if operation is None:
            return
        if (result.operation_id, result.original_runtime_instance, result.original_runtime_epoch,
                result.original_vessel_id) != (operation.operation_id, operation.original_runtime_instance,
                operation.original_runtime_epoch, operation.original_vessel_id):
            return
        if not result.retained and result.reason == 'result_not_retained':
            # A read query uses another DDS path and can arrive before the batch
            # that creates this receipt. Absence is unresolved until the deadline.
            return
        if not result.retained or result.id != operation.id or result.controller_id != self.lease.controller_id:
            self.stop('separation_result_unavailable_or_conflicting')
            return
        if result.completed and not result.success:
            self.stop('separation_failed:'+result.reason)
            return
        if self.separation_result is None or not self.separation_result.completed:
            self.separation_result = result

    def query_separation_result(self, now):
        if self.pending_operation is None or now < self.next_result_query:
            return
        if self.result_query_future is not None and not self.result_query_future.done():
            return
        if not self.result_client.service_is_ready():
            return
        self.next_result_query = now+.25
        request = GetSeparationResult.Request()
        for field in ('operation_id', 'original_runtime_instance', 'original_runtime_epoch', 'original_vessel_id'):
            setattr(request, field, getattr(self.pending_operation, field))
        self.result_query_future = self.result_client.call_async(request)
        self.result_query_future.add_done_callback(
            lambda future, operation_id=request.operation_id: self.received_result_query(future, operation_id))

    def received_result_query(self, future, operation_id):
        if self.pending_operation is None or self.pending_operation.operation_id != operation_id:
            return
        try:
            response = future.result()
            if response.found:
                self.on_separation_result(response.result)
            # A negative lookup can precede acceptance of the original command.
            # Keep querying within separation_deadline rather than infer failure.
        except Exception as exc:
            self.get_logger().warning(f'Separation result query failed: {exc}')

    def on_authority(self, message):
        if message.vessel_id != self.lease.vessel_id:
            return
        self.authority, self.authority_seen = message, time.monotonic()
        lost = self.lease.observe_authority(message.vessel_id, message.controller_id, message.lease_id,
                                            message.state == message.STATE_OWNED)
        if lost and self.active and self.coast_hold is None and not (self.pending_separation and time.monotonic() < self.separation_deadline):
            issue = self.simulator_issue(time.monotonic())
            if not (message.state == ControlAuthorityState.STATE_UNOWNED
                    and message.reason == 'lease_expired' and self.hold_orbit(time.monotonic(), issue or 'orbital_telemetry_hold')):
                self.stop(issue or 'authority_lost')

    def on_flight(self, message):
        if message.vessel_id != self.lease.vessel_id:
            return
        now = time.monotonic()
        if not self.flight or message.universal_time > self.flight.universal_time:
            self.progress_seen = now
        elif message.universal_time < self.flight.universal_time and self.active:
            self.stop('simulation_time_reversed')
        self.flight, self.flight_seen = message, now

    def on_engine(self, message):
        if self.lease.vessel_id:
            self.engines[message.id] = (message, time.monotonic())

    def on_separation(self, message):
        if self.lease.vessel_id:
            self.separators[message.id] = (message, time.monotonic())

    def identify(self, message):
        self.sequence += 1
        message.header.stamp = self.get_clock().now().to_msg()
        message.vessel_id = self.lease.vessel_id
        message.controller_id = self.lease.controller_id
        message.lease_id = self.lease.lease_id
        message.sequence = self.sequence
        return message

    def send_authority(self, action):
        message = self.identify(ControlAuthorityCommand())
        message.action = {'acquire': message.ACTION_ACQUIRE, 'renew': message.ACTION_RENEW,
                          'release': message.ACTION_RELEASE}[action]
        message.lease_duration_sec = 1.
        message.suppress_sas = True
        self.authority_pub.publish(message)

    def new_separation(self, actuator_id):
        command = SeparationCommand()
        command.id, command.separate = actuator_id, True
        command.operation_id = uuid.uuid4().hex
        command.original_runtime_instance = self.lifecycle.runtime_instance
        command.original_runtime_epoch = self.lifecycle.runtime_epoch
        command.original_vessel_id = self.lease.vessel_id
        self.pending_operation = command
        self.separation_result = None
        self.pending_separation = actuator_id
        self.separation_deadline = time.monotonic()+5.
        self.next_result_query = 0.
        return command

    def publish_demand(self, demand, thrust, s, renew=False):
        batch = self.identify(ControlBatch())
        batch.renew_lease = renew
        batch.lease_duration_sec = 1.
        batch.suppress_sas = True
        # Release supports only after measured thrust exceeds vehicle weight.
        # Flight inputs, propulsion, and the final release share one transaction.
        now = time.monotonic()
        engine_state = self.engines.get(self.engine_id, (None, 0.))[0]
        if (self.mission.phase == Phase.IGNITION and not self.pending_separation and engine_state is not None
                and engine_state.thrust > s.mass*s.gravity*1.05 and now >= self.next_clamp_release):
            for key, (clamp, seen) in self.separators.items():
                if (clamp.mechanism == 'launch_clamp' and clamp.available and not clamp.separated
                        and now-seen < self.get_parameter('telemetry_timeout').value):
                    batch.has_separation = True
                    batch.separation = self.new_separation(key)
                    self.next_clamp_release = now+.2
                    break
        if demand.separate and not self.pending_operation:
            batch.has_separation = True
            batch.separation = self.new_separation(self.separator_id)
        command = FlightControlCommand()
        command.pitch, command.yaw, command.roll = attitude_inputs(demand.direction, s.angular_velocity, self.config, s.dynamic_pressure)
        command.landing_gear = demand.gear
        command.timeout_sec = .3
        batch.has_flight, batch.flight = True, command
        engine = EngineCommand()
        engine.id = self.engine_id
        engine.enabled = True
        # Avoid accelerating the wrong way during coast/flip manoeuvres.
        aligned = dot(demand.direction, (1., 0., 0.)) > .9
        engine.target_thrust = thrust*demand.throttle if aligned else 0.
        engine.timeout_sec = .3
        batch.engines = [engine]
        self.batch_pub.publish(batch)

    def send_zero_batch(self, landing_gear=False):
        batch = self.identify(ControlBatch())
        batch.has_flight = True
        batch.flight.landing_gear, batch.flight.timeout_sec = landing_gear, .3
        engine = EngineCommand()
        engine.id, engine.enabled, engine.target_thrust, engine.timeout_sec = self.engine_id, True, 0., .3
        batch.engines = [engine]
        self.batch_pub.publish(batch)

    def stop(self, reason):
        if self.active:
            self.mission.abort(reason)
            if self.lease.owned:
                # No attitude impulse during cutoff. KSP enforces its own
                # timeout even if the final UDP packet cannot be delivered.
                self.send_zero_batch(landing_gear=True)
                self.send_authority('release')
                self.lease.owned = False
            self.active = False
        self.publish_status(time.monotonic(), force=True)

    def on_abort_request(self, request, response):
        self.stop('operator_abort')
        response.success, response.message = True, 'Actuation stopped; cleanup/configure required before another flight.'
        return response

    def hold_orbit(self, now, reason='orbital_telemetry_hold'):
        f = self.flight
        # Only an already established, unpowered orbit can wait for a simulator
        # frame stall. Powered flight and atmospheric descent still abort.
        engine = self.engines.get(self.engine_id, (None, 0.))[0]
        if (self.mission.phase != Phase.DEORBIT_WAIT or f is None or self.pending_operation is not None
                or min(f.altitude_asl, f.periapsis) < 70000. or f.dynamic_pressure != 0.
                or engine is None or engine.throttle > .001 or engine.thrust > 1.):
            return False
        try:
            checkpoint = MissionCheckpoint.capture(f, self.simulator,
                [item[0] for item in self.engines.values()], [item[0] for item in self.separators.values()],
                self.authority)
        except ValueError as exc:
            self.get_logger().warning(f'Cannot capture safe orbital hold: {exc}')
            return False
        self.coast_hold = dict(started=now, checkpoint=checkpoint,
                               ready_since=None, acquiring=False,
                               timeout=300. if reason == 'simulator_paused' else 5.,
                               reason=reason)
        self.mission.transition(Phase.HOLD, f.universal_time, reason)
        if self.lease.owned:
            # These are explicit zeros, not renewed flight demands. The normal
            # actuator/lease watchdogs remain in force throughout the hold.
            self.send_zero_batch()
            self.send_authority('release')
        self.lease.owned = self.ever_owned = False
        return True

    def recover_orbit(self, now):
        hold = self.coast_hold
        issue = self.simulator_issue(now)
        if issue == 'simulator_paused' and hold['reason'] != 'simulator_paused':
            # The pause heartbeat can arrive after an engine/authority freshness
            # guard. Give the same proven pause the same bound in either order.
            hold['timeout'], hold['reason'] = 300., 'simulator_paused'
            hold.pop('communication_lost_at', None)
            self.mission.transition(Phase.HOLD, self.flight.universal_time, 'simulator_paused')
        if now-hold['started'] > hold['timeout']:
            self.stop('orbital_telemetry_hold_expired')
            return
        timeout = self.get_parameter('telemetry_timeout').value
        if issue:
            hold['ready_since'] = None
            # An explicit pause can wait; losing communication during that pause
            # uses the short communications bound, not the operator pause bound.
            if issue != 'simulator_paused':
                hold['communication_lost_at'] = hold.get('communication_lost_at', now)
                if now-hold['communication_lost_at'] > 5.:
                    self.stop('orbital_communication_hold_expired')
            return
        hold.pop('communication_lost_at', None)
        engine, engine_seen = self.engines.get(self.engine_id, (None, 0.))
        fresh = (self.flight is not None and self.authority is not None and engine is not None
                 and max(now-self.flight_seen, now-self.lifecycle_seen,
                         now-self.authority_seen, now-engine_seen, now-self.progress_seen) < timeout)
        if not fresh:
            hold['ready_since'] = None
            return
        f = self.flight
        validation = validate_resume(hold['checkpoint'], f, self.simulator,
            [item[0] for item in self.engines.values()], [item[0] for item in self.separators.values()],
            self.authority, now_monotonic=now,
            received_at=min(self.flight_seen, self.simulator_seen, self.authority_seen,
                *(item[1] for item in self.engines.values()), *(item[1] for item in self.separators.values())),
            max_sample_age_sec=timeout,
            allowed_controller_id=self.lease.controller_id if hold['acquiring'] else '',
            allowed_lease_id=self.lease.lease_id if hold['acquiring'] else '')
        if not validation.allowed:
            if validation.reason in ('telemetry_stale', 'fresh_post_checkpoint_observation_required',
                                     'observation_time_mismatch', 'simulator_not_advancing', 'authority_not_released'):
                hold['ready_since'] = None
                return
            self.get_logger().warning(f'Orbital resume rejected: {validation.reason}')
            self.stop('orbital_hold_authority_conflict' if validation.reason == 'authority_conflict'
                      else 'orbital_hold_state_changed')
            return
        hold['ready_since'] = hold['ready_since'] or now
        if now-hold['ready_since'] < .5:
            return
        if not hold['acquiring']:
            self.lease = LeaseCoordinator(self.get_parameter('controller_id').value, .2)
            self.observe_lease_vessel(self.lifecycle)
            hold['acquiring'] = True
        if self.lease.owned:
            self.coast_hold = None
            self.ever_owned = True
            self.mission.transition(Phase.DEORBIT_WAIT, f.universal_time, 'orbital_telemetry_recovered')
            return
        action = self.lease.due_action(now)
        if action:
            self.send_authority(action.action)

    def tick(self):
        now = time.monotonic()
        self.publish_status(now)
        if not self.active:
            return
        if self.coast_hold is not None:
            self.recover_orbit(now)
            return
        if self.pending_operation is not None:
            if now > self.separation_deadline:
                self.stop('separation_result_not_confirmed')
                return
            self.query_separation_result(now)
            receipt = self.separation_result
            if receipt is not None and receipt.completed:
                if not receipt.success:
                    self.stop('separation_failed:'+receipt.reason)
                    return
                life = self.lifecycle
                if receipt.result_runtime_generation > life.runtime_generation:
                    return  # Result arrived before its session heartbeat.
                if (receipt.result_runtime_epoch != life.runtime_epoch
                        or receipt.result_runtime_generation != life.runtime_generation
                        or life.vessel_id not in receipt.resulting_vessel_ids):
                    self.stop('separation_result_session_mismatch')
                    return
                if (self.flight is None or self.engine_id not in self.engines
                        or self.authority is None or self.simulator is None):
                    return
                if self.pending_separation == self.separator_id:
                    self.mission.separation_confirmed = True
                self.rebinding = False
                self.pending_separation = ''
                self.pending_operation = None
                self.get_logger().info('Separation receipt verified against the current flight session')
            elif self.rebinding or not self.lease.owned:
                return
        timeout = self.get_parameter('telemetry_timeout').value
        issue = self.simulator_issue(now)
        if issue:
            if not self.hold_orbit(now, issue):
                self.stop(issue)
            return
        if (self.flight is None or now-self.flight_seen > timeout or now-self.lifecycle_seen > timeout
                or now-self.progress_seen > 2.):
            if not self.hold_orbit(now):
                self.stop('control_snapshot_stale' if now-self.flight_seen > timeout else 'simulation_not_progressing')
            return
        engine, seen = self.engines.get(self.engine_id, (None, 0.))
        if engine is None or now-seen > timeout or (engine.flameout and self.ever_owned):
            if (engine is not None and engine.flameout) or not self.hold_orbit(now):
                self.stop('engine_state_unavailable')
            return
        if self.ever_owned and now-self.authority_seen > timeout:
            if not self.hold_orbit(now):
                self.stop('authority_state_stale')
            return
        if self.flight.electric_charge <= .1:
            self.stop('power_exhausted')
            return
        if not self.lease.owned and now-self.activation_time > 5:
            self.stop('authority_acquisition_timeout')
            return
        action = self.lease.due_action(now)
        if action and action.action == 'acquire':
            self.send_authority(action.action)
            return
        if not self.lease.owned:
            return
        self.ever_owned = True
        s = sample(self.flight)
        try:
            if self.mission.phase == Phase.READY:
                self.mission.start(s)
            demand = self.mission.step(s, engine.max_thrust, self.mission.separation_confirmed)
            if self.mission.terminal:
                self.stop(self.mission.reason)
                return
            self.publish_demand(demand, engine.max_thrust, s, renew=bool(action))
        except (ValueError, ArithmeticError) as exc:
            self.get_logger().error(str(exc))
            self.stop('invalid_flight_state')

    def publish_status(self, now, force=False):
        while self.last_event < len(self.mission.transitions):
            event = asdict(self.mission.transitions[self.last_event])
            self.event_pub.publish(String(data=json.dumps(event)))
            self.get_logger().info(f"{event['previous']} -> {event['phase']}: {event['reason']}")
            self.last_event += 1
        if not force and now-self.last_status < .5:
            return
        self.last_status = now
        data = dict(phase=self.mission.phase.value, reason=self.mission.reason,
                    profile=self.config.profile, active=self.active, lease_owned=self.lease.owned,
                    vessel_id=self.lease.vessel_id, payload_deployed=self.mission.separation_confirmed,
                    preflight=self.preflight_error(now) if not self.active and self.mission.phase == Phase.READY else '',
                    rebinding=self.rebinding, pending_separation=self.pending_separation)
        if self.flight:
            data.update(altitude_asl=self.flight.altitude_asl, altitude_agl=self.flight.altitude_agl,
                        vertical_speed=self.flight.vertical_speed, fuel=self.flight.liquid_fuel,
                        apoapsis=self.flight.apoapsis, periapsis=self.flight.periapsis)
        self.status_pub.publish(String(data=json.dumps(data, allow_nan=False)))
        status = DiagnosticStatus(name='reusable_mission', hardware_id=self.lease.vessel_id,
                                  message=self.mission.reason,
                                  level=DiagnosticStatus.ERROR if self.mission.phase == Phase.ABORT else DiagnosticStatus.OK)
        status.values = [KeyValue(key=k, value=str(v)) for k, v in data.items()]
        self.diagnostic_pub.publish(DiagnosticArray(header=self.identical_header(), status=[status]))

    def identical_header(self):
        from std_msgs.msg import Header
        return Header(stamp=self.get_clock().now().to_msg())


def main(args=None):
    rclpy.init(args=args)
    node = ReusableMissionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.stop('process_exit')
            node.destroy_node()
            rclpy.shutdown()
