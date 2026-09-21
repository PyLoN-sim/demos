"""Lifecycle-managed flight executive with explicit lease and freshness guards."""
from dataclasses import asdict
import json
import math
import time

import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn as Result
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from pylon_interfaces.msg import (
    ControlAuthorityCommand, ControlAuthorityState, FlightControlCommand,
    FlightState, EngineCommand, EngineState, SeparationCommand, SeparationState,
    VesselLifecycle,
)
from pylon_vehicle_control.application.lease import LeaseCoordinator
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
        root = '/ksp_vessel'
        durable = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, '~/status', durable)
        self.event_pub = self.create_publisher(String, '~/events', durable)
        self.diagnostic_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        # The final zero command and release must be sent before deactivation
        # disables these lifecycle publishers.
        self.authority_pub = self.create_lifecycle_publisher(ControlAuthorityCommand, root+'/control/authority/command', 10)
        self.engine_pub = self.create_lifecycle_publisher(EngineCommand, root+'/actuators/propulsion/command', 10)
        self.flight_pub = self.create_lifecycle_publisher(FlightControlCommand, root+'/control/flight_command', 10)
        self.separation_pub = self.create_lifecycle_publisher(SeparationCommand, root+'/actuators/separation/command', 10)
        self.create_subscription(VesselLifecycle, root+'/lifecycle', self.on_vessel, durable)
        self.create_subscription(ControlAuthorityState, root+'/control/authority/state', self.on_authority, durable)
        self.create_subscription(FlightState, root+'/ground_truth/flight', self.on_flight, qos_profile_sensor_data)
        self.create_subscription(EngineState, root+'/actuators/propulsion/state', self.on_engine, qos_profile_sensor_data)
        self.create_subscription(SeparationState, root+'/actuators/separation/state', self.on_separation, durable)
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

    def on_authority(self, message):
        if message.vessel_id != self.lease.vessel_id:
            return
        self.authority, self.authority_seen = message, time.monotonic()
        lost = self.lease.observe_authority(message.vessel_id, message.controller_id, message.lease_id,
                                            message.state == message.STATE_OWNED)
        if lost and self.active and self.coast_hold is None and not (self.pending_separation and time.monotonic() < self.separation_deadline):
            if not (message.state == ControlAuthorityState.STATE_UNOWNED
                    and message.reason == 'lease_expired' and self.hold_orbit(time.monotonic())):
                self.stop('authority_lost')

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

    def publish_demand(self, demand, thrust, s):
        # Release supports only after measured thrust exceeds vehicle weight.
        # One clamp per tick avoids racing several commands against the shared
        # authority sequence. A still-attached clamp can be retried safely.
        now = time.monotonic()
        engine_state = self.engines.get(self.engine_id, (None, 0.))[0]
        if (self.mission.phase == Phase.IGNITION and not self.pending_separation and engine_state is not None
                and engine_state.thrust > s.mass*s.gravity*1.05 and now >= self.next_clamp_release):
            for key, (clamp, seen) in self.separators.items():
                if (clamp.mechanism == 'launch_clamp' and clamp.available and not clamp.separated
                        and now-seen < self.get_parameter('telemetry_timeout').value):
                    release = self.identify(SeparationCommand())
                    release.id, release.separate = key, True
                    self.separation_pub.publish(release)
                    self.pending_separation = key
                    self.separation_deadline = now+3.
                    self.next_clamp_release = now+.2
                    return
        if demand.separate:
            separation = self.identify(SeparationCommand())
            separation.id = self.separator_id
            separation.separate = True
            self.separation_pub.publish(separation)
            self.pending_separation = self.separator_id
            self.separation_deadline = now+3.
            return
        command = self.identify(FlightControlCommand())
        command.pitch, command.yaw, command.roll = attitude_inputs(demand.direction, s.angular_velocity, self.config, s.dynamic_pressure)
        command.landing_gear = demand.gear
        command.timeout_sec = .3
        self.flight_pub.publish(command)
        engine = self.identify(EngineCommand())
        engine.id = self.engine_id
        engine.enabled = True
        # Avoid accelerating the wrong way during coast/flip manoeuvres.
        aligned = dot(demand.direction, (1., 0., 0.)) > .9
        engine.target_thrust = thrust*demand.throttle if aligned else 0.
        engine.timeout_sec = .3
        self.engine_pub.publish(engine)

    def stop(self, reason):
        if self.active:
            self.mission.abort(reason)
            if self.lease.owned:
                # No attitude impulse during cutoff. KSP enforces its own
                # timeout even if the final UDP packet cannot be delivered.
                engine = self.identify(EngineCommand())
                engine.id, engine.enabled, engine.target_thrust, engine.timeout_sec = self.engine_id, True, 0., .3
                self.engine_pub.publish(engine)
                command = self.identify(FlightControlCommand())
                command.landing_gear, command.timeout_sec = True, .3
                self.flight_pub.publish(command)
                self.send_authority('release')
                self.lease.owned = False
            self.active = False
        self.publish_status(time.monotonic(), force=True)

    def on_abort_request(self, request, response):
        self.stop('operator_abort')
        response.success, response.message = True, 'Actuation stopped; cleanup/configure required before another flight.'
        return response

    def hold_orbit(self, now):
        f = self.flight
        # Only an already established, unpowered orbit can wait for a simulator
        # frame stall. Powered flight and atmospheric descent still abort.
        if (self.mission.phase != Phase.DEORBIT_WAIT or f is None
                or min(f.altitude_asl, f.periapsis) < 70000. or f.dynamic_pressure != 0.):
            return False
        self.coast_hold = dict(started=now, fuel=f.liquid_fuel, mass=f.mass,
                               ready_since=None, acquiring=False)
        self.mission.transition(Phase.HOLD, f.universal_time, 'orbital_telemetry_hold')
        if self.lease.owned:
            # These are explicit zeros, not renewed flight demands. The normal
            # actuator/lease watchdogs remain in force throughout the hold.
            engine = self.identify(EngineCommand())
            engine.id, engine.enabled, engine.target_thrust, engine.timeout_sec = self.engine_id, True, 0., .3
            self.engine_pub.publish(engine)
            command = self.identify(FlightControlCommand())
            command.timeout_sec = .3
            self.flight_pub.publish(command)
            self.send_authority('release')
        self.lease.owned = self.ever_owned = False
        return True

    def recover_orbit(self, now):
        hold = self.coast_hold
        if now-hold['started'] > 5.:
            self.stop('orbital_telemetry_hold_expired')
            return
        timeout = self.get_parameter('telemetry_timeout').value
        engine, engine_seen = self.engines.get(self.engine_id, (None, 0.))
        fresh = (self.flight is not None and self.authority is not None and engine is not None
                 and max(now-self.flight_seen, now-self.lifecycle_seen,
                         now-self.authority_seen, now-engine_seen, now-self.progress_seen) < timeout)
        if not fresh:
            hold['ready_since'] = None
            return
        f = self.flight
        if (min(f.altitude_asl, f.periapsis) < 70000. or f.dynamic_pressure != 0.
                or abs(f.liquid_fuel-hold['fuel']) > .2 or abs(f.mass-hold['mass']) > 5.
                or engine.flameout or engine.throttle > .001 or engine.thrust > 1.):
            self.stop('orbital_hold_state_changed')
            return
        if self.authority.state != ControlAuthorityState.STATE_UNOWNED:
            # Allow the old lease's release acknowledgment to arrive first.
            if (not hold['acquiring'] and self.authority.controller_id == self.lease.controller_id
                    and self.authority.lease_id == self.lease.lease_id):
                return
            if not (hold['acquiring'] and self.lease.owned):
                self.stop('orbital_hold_authority_conflict')
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
        if self.pending_separation and now > self.separation_deadline:
            self.stop('separation_epoch_not_confirmed')
            return
        if self.rebinding:
            receipt, seen = self.separators.get(self.pending_separation, (None, 0.))
            if now > self.separation_deadline:
                self.stop('separation_epoch_not_confirmed')
                return
            if (receipt is None or not receipt.separated or self.flight is None
                    or self.engine_id not in self.engines or self.authority is None):
                return
            if self.pending_separation == self.separator_id:
                self.mission.separation_confirmed = True
            self.rebinding = False
            self.pending_separation = ''
            self.get_logger().info('Separation confirmed; acquiring a fresh control lease')
        elif self.pending_separation and not self.lease.owned:
            if now > self.separation_deadline:
                self.stop('separation_epoch_not_confirmed')
            return
        timeout = self.get_parameter('telemetry_timeout').value
        if (self.flight is None or now-self.flight_seen > timeout or now-self.lifecycle_seen > timeout
                or now-self.progress_seen > 2.):
            if not self.hold_orbit(now):
                self.stop('telemetry_stale_or_simulation_paused')
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
        if action:
            self.send_authority(action.action)
            # DDS does not order different topics. Reserve this tick for the
            # lease heartbeat so a newer actuator sequence cannot overtake it.
            # The 50 ms gap remains within the 300 ms actuator watchdog.
            return
        if not self.lease.owned:
            return
        self.ever_owned = True
        s = sample(self.flight)
        try:
            if self.mission.phase == Phase.READY:
                self.mission.start(s)
            separated = self.separators.get(self.separator_id, (None, 0.))[0]
            demand = self.mission.step(s, engine.max_thrust, bool(separated and separated.separated))
            if self.mission.terminal:
                self.stop(self.mission.reason)
                return
            self.publish_demand(demand, engine.max_thrust, s)
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
