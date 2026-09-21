"""Deterministic, guarded mission executive. No ROS or game dependencies.

Mission time is simulation time; transport/authority watchdogs belong to the
adapter and use monotonic wall time. FlightState is simulator truth, not a
sensor-derived navigation solution.
"""
from dataclasses import dataclass
from enum import Enum
import math


def clip(value, low, high):
    return max(low, min(high, value))


def norm(v):
    return math.sqrt(sum(x*x for x in v))


def unit(v):
    n = norm(v)
    return tuple(x/n for x in v) if n > 1e-9 else (1., 0., 0.)


def dot(a, b):
    return sum(x*y for x, y in zip(a, b))


def combine(a, b, x, y):
    return tuple(x*u+y*v for u, v in zip(a, b))


class Phase(str, Enum):
    READY = 'READY'
    IGNITION = 'IGNITION'
    ASCENT = 'ASCENT'
    COAST = 'COAST'
    CIRCULARIZE = 'CIRCULARIZE'
    DEPLOY = 'DEPLOY'
    CLEARANCE = 'CLEARANCE'
    DEORBIT_WAIT = 'DEORBIT_WAIT'
    HOLD = 'HOLD'
    DEORBIT = 'DEORBIT'
    ENTRY = 'ENTRY'
    BRAKING = 'BRAKING'
    APPROACH = 'APPROACH'
    LANDING = 'LANDING'
    TOUCHDOWN = 'TOUCHDOWN'
    COMPLETE = 'COMPLETE'
    ABORT = 'ABORT'


@dataclass(frozen=True)
class Config:
    profile: str = 'orbital'
    orbit_altitude: float = 80000.
    minimum_periapsis: float = 78000.
    hop_altitude: float = 1500.
    reserve_fraction: float = .07
    landing_clearance: float = 12.
    settle_altitude: float = 300.
    # Flat grass south of the pad, clear of the released launch-clamp towers.
    target_latitude: float = -.1072
    target_longitude: float = -74.5577
    deorbit_lead_angle: float = 110.
    deorbit_periapsis: float = 18000.
    landing_speed: float = .8
    rate_limit: float = .12
    rate_gain: float = 3.
    aero_rate_gain: float = 8.
    attitude_gain: float = .55
    max_dynamic_pressure: float = 25000.

    def __post_init__(self):
        if self.profile not in ('orbital', 'hop'):
            raise ValueError('profile must be orbital or hop')
        for name, value in vars(self).items():
            if name != 'profile' and not math.isfinite(value):
                raise ValueError(f'{name} must be finite')
        if not 70000 < self.minimum_periapsis < self.orbit_altitude <= 150000:
            raise ValueError('require 70 km < minimum periapsis < orbit altitude <= 150 km')
        if not 100 <= self.hop_altitude <= 10000 or not 0 < self.reserve_fraction < .4:
            raise ValueError('invalid hop altitude or landing reserve')
        if not all(getattr(self, k) > 0 for k in (
            'landing_clearance', 'landing_speed', 'rate_limit', 'rate_gain', 'aero_rate_gain',
            'attitude_gain', 'max_dynamic_pressure', 'deorbit_lead_angle')):
            raise ValueError('control limits must be positive')
        if (not -90 <= self.target_latitude <= 90 or not -180 <= self.target_longitude <= 180
                or not 0 < self.deorbit_lead_angle < 180 or not -10000 <= self.deorbit_periapsis < 70000
                or self.rate_limit > .25):
            raise ValueError('invalid landing target, deorbit trajectory or angular-rate limit')
        if not 200 <= self.settle_altitude <= 1000:
            raise ValueError('settle_altitude must be in [200, 1000] metres')


@dataclass(frozen=True)
class Sample:
    time: float
    altitude: float
    agl: float
    mass: float
    fuel: float
    gravity: float
    vertical_speed: float
    horizontal_speed: float = 0.
    apoapsis: float = 0.
    periapsis: float = -600000.
    time_to_apoapsis: float = 100.
    latitude: float = -.0972
    longitude: float = -74.5577
    radius: float = 600000.
    mu: float = 3.5316e12
    dynamic_pressure: float = 0.
    landed: bool = False
    splashed: bool = False
    up: tuple = (1., 0., 0.)
    east: tuple = (0., 1., 0.)
    north: tuple = (0., 0., 1.)
    velocity: tuple = (0., 0., 0.)
    orbital_velocity: tuple = (0., 200., 0.)
    angular_velocity: tuple = (0., 0., 0.)


@dataclass(frozen=True)
class Demand:
    throttle: float = 0.
    direction: tuple = (1., 0., 0.)
    gear: bool = True
    separate: bool = False


@dataclass(frozen=True)
class Transition:
    time: float
    previous: str
    phase: str
    reason: str


class Mission:
    def __init__(self, config=Config()):
        self.config = config
        self.phase = Phase.READY
        self.entered = 0.
        self.last_time = None
        self.initial_fuel = 0.
        self.initial_altitude = 0.
        self.initial_agl = 0.
        self.start_time = 0.
        self.reason = 'awaiting_activation'
        self.transitions = []
        self.separation_sent = False
        self.separation_confirmed = False
        self.ever_flew = False
        self.settled_since = None
        self.approach_braking = False
        self.touchdown_stable_since = None

    @property
    def terminal(self):
        return self.phase in (Phase.ABORT, Phase.COMPLETE)

    def transition(self, phase, t, reason):
        self.transitions.append(Transition(t, self.phase.value, phase.value, reason))
        self.phase, self.entered, self.reason = phase, t, reason

    def abort(self, reason):
        if not self.terminal:
            self.transition(Phase.ABORT, self.last_time or self.start_time, reason)

    def start(self, s):
        if self.phase != Phase.READY or not s.landed or s.splashed:
            raise ValueError('activation requires a fresh mission on dry ground')
        if s.fuel <= 0 or s.mass <= 0:
            raise ValueError('activation requires fuel and valid mass')
        self.initial_fuel, self.initial_altitude, self.initial_agl = s.fuel, s.altitude, s.agl
        self.start_time = self.last_time = s.time
        self.transition(Phase.IGNITION, s.time, 'preflight_passed')

    def landing_demand(self, s, thrust, approach=False):
        c = self.config
        clearance = max(c.landing_clearance, self.initial_agl)
        h = max(0., s.agl-clearance)
        max_acc = thrust/s.mass
        braking = max(.5, .65*max_acc-s.gravity)
        desired_v = -min(70., math.sqrt(2*braking*h), max(c.landing_speed, h*.45))
        desired_v = min(-c.landing_speed, desired_v)
        if approach:
            descent_limit = (min(180., math.sqrt(2*braking*max(0., h-c.settle_altitude)))
                             if c.profile == 'orbital' else 35.)
            desired_v = clip((c.settle_altitude-h)*.45, -descent_limit, 10.)
        vertical_acc = clip(s.gravity + (desired_v-s.vertical_speed)*2.5, 0., max_acc)
        # Track the launch site only in the final neighbourhood. This guidance
        # does not claim a terrain-aware global landing-site planner.
        north_error = math.radians(c.target_latitude-s.latitude)*s.radius
        east_error = math.radians((c.target_longitude-s.longitude+180)%360-180)*s.radius*math.cos(math.radians(s.latitude))
        near_target = math.hypot(north_error, east_error) < 15000
        east_v, north_v = dot(s.velocity, s.east), dot(s.velocity, s.north)
        # Finish translating while there is height to settle the attitude loop.
        # Near the ground, arrest drift instead of chasing the exact waypoint.
        translate = approach and near_target and not self.approach_braking
        target_e = clip(east_error*.04, -30., 30.) if translate else 0.
        target_n = clip(north_error*.04, -30., 30.) if translate else 0.
        max_lateral = max(0., vertical_acc)*math.tan(math.radians(20 if h > 40 else 5))
        # Outer velocity loop must be slower than this tall vehicle's attitude
        # response. A 0.45/s gain produced growing lateral oscillations in KSP.
        east_a, north_a = (target_e-east_v)*.12, (target_n-north_v)*.12
        scale = min(1., max_lateral/max(.001, math.hypot(east_a, north_a)))
        direction = tuple(vertical_acc*u+scale*(east_a*e+north_a*n)
                          for u, e, n in zip(s.up, s.east, s.north))
        alignment = max(.3, dot(unit(direction), (1., 0., 0.)))
        throttle = clip(norm(direction)/max_acc/alignment, 0., 1.)
        return Demand(throttle, unit(direction) if norm(direction) > .01 else s.up, True)

    def step(self, s, thrust, separation_confirmed=False):
        c = self.config
        if self.terminal or self.phase == Phase.READY:
            return Demand(direction=s.up)
        if self.last_time is not None and s.time < self.last_time:
            self.abort('simulation_time_reversed')
            return Demand(direction=s.up)
        self.last_time = s.time
        elapsed = s.time-self.entered
        if s.mass <= 0 or thrust <= 0 or not math.isfinite(thrust):
            self.abort('propulsion_unavailable')
        elif s.time-self.start_time > 7200:
            self.abort('mission_deadline')
        elif s.splashed:
            self.abort('water_contact')
        elif s.fuel <= 0 and not s.landed:
            self.abort('fuel_exhausted')
        if self.terminal:
            return Demand(direction=s.up)
        self.ever_flew |= not s.landed and s.agl > self.initial_agl+5
        self.separation_confirmed |= separation_confirmed and self.separation_sent
        if self.ever_flew and s.landed:
            if (self.phase == Phase.LANDING and abs(s.vertical_speed) < 2.
                    and s.horizontal_speed < 2. and dot(s.up, (1., 0., 0.)) > .94):
                self.transition(Phase.TOUCHDOWN, s.time, 'ground_contact')
            elif self.phase != Phase.TOUCHDOWN:
                self.abort('unexpected_ground_contact')
        if self.phase == Phase.IGNITION:
            if thrust/(s.mass*s.gravity) < 1.15:
                self.abort('insufficient_launch_twr')
                return Demand(direction=s.up)
            if elapsed > 10 and not self.ever_flew:
                self.abort('liftoff_timeout')
                return Demand(direction=s.up)
            if self.ever_flew:
                self.transition(Phase.ASCENT, s.time, 'liftoff_confirmed')
            return Demand(min(1., s.mass*s.gravity*1.4/thrust), s.up, True)
        if self.phase == Phase.ASCENT:
            if s.fuel < self.initial_fuel*c.reserve_fraction:
                self.abort('landing_reserve_reached_before_insertion')
                return Demand(direction=s.up)
            if c.profile == 'hop':
                predicted_ap = s.altitude+max(0., s.vertical_speed)**2/(2*s.gravity)
                if predicted_ap >= self.initial_altitude+c.hop_altitude:
                    self.transition(Phase.COAST, s.time, 'hop_apogee_target')
                    return Demand(direction=s.up, gear=False)
                return Demand(clip(s.mass*(s.gravity+8.)/thrust, 0., 1.), s.up, False)
            horizontal_orbit = norm(combine(s.orbital_velocity, s.up, 1., -dot(s.orbital_velocity, s.up)))
            circular = math.sqrt(s.mu/(s.radius+s.altitude))
            if s.apoapsis >= c.orbit_altitude and horizontal_orbit >= .5*circular:
                self.transition(Phase.COAST, s.time, 'target_apoapsis')
                return Demand(direction=s.east, gear=False)
            if elapsed > 600:
                self.abort('ascent_timeout')
                return Demand(direction=s.up)
            # Keep vertical until 2 km, then turn gradually. In dense air the
            # velocity vector is the stronger constraint: a tall rocket must
            # not command a large angle of attack to catch up with the schedule.
            pitch = clip(90.-(max(0., s.altitude-2000)/28000.)*85, 5., 90.)
            direction = combine(s.up, s.east, math.sin(math.radians(pitch)), math.cos(math.radians(pitch)))
            if s.dynamic_pressure > 3000. and norm(s.velocity) > 50.:
                prograde = unit(s.velocity)
                angle = math.acos(clip(dot(direction, prograde), -1., 1.))
                if angle > math.radians(3.):
                    fraction = math.radians(3.)/angle
                    direction = unit(combine(prograde, direction, 1.-fraction, fraction))
            throttle = min(1., 2.5*s.mass*s.gravity/thrust)
            if s.dynamic_pressure > c.max_dynamic_pressure:
                throttle *= clip(c.max_dynamic_pressure/s.dynamic_pressure, .35, 1.)
            return Demand(throttle, direction, False)
        if self.phase == Phase.COAST:
            if c.profile == 'hop':
                if s.vertical_speed <= 5.:
                    self.transition(Phase.DEPLOY, s.time, 'hop_apogee_reached')
            else:
                circular = math.sqrt(s.mu/(s.radius+s.altitude))
                horizontal_orbit = norm(combine(s.orbital_velocity, s.up, 1., -dot(s.orbital_velocity, s.up)))
                burn_time = max(0., circular-horizontal_orbit)/(thrust/s.mass)
                if s.time_to_apoapsis <= burn_time*.55+8. or s.vertical_speed < 0.:
                    self.transition(Phase.CIRCULARIZE, s.time, 'insertion_burn_window')
            return Demand(direction=s.up if c.profile == 'hop' else s.east, gear=False)
        if self.phase == Phase.CIRCULARIZE:
            if s.periapsis >= c.minimum_periapsis and s.altitude >= 70000:
                self.transition(Phase.DEPLOY, s.time, 'stable_orbit_confirmed')
                return Demand(direction=s.east, gear=False)
            if elapsed > 300 or s.fuel < self.initial_fuel*c.reserve_fraction:
                self.abort('orbit_insertion_failed')
                return Demand(direction=s.east, gear=False)
            tangential = unit(combine(s.orbital_velocity, s.up, 1., -dot(s.orbital_velocity, s.up)))
            pitch = clip((c.orbit_altitude-s.altitude)*.001-s.vertical_speed*.3, -15., 25.)
            direction = combine(tangential, s.up, math.cos(math.radians(pitch)), math.sin(math.radians(pitch)))
            circular = math.sqrt(s.mu/(s.radius+s.altitude))
            deficit = circular-dot(s.orbital_velocity, tangential)
            throttle = clip(deficit*s.mass/thrust*.35, .03, 1.)
            return Demand(throttle, direction, False)
        if self.phase == Phase.DEPLOY:
            if self.separation_confirmed:
                self.transition(Phase.CLEARANCE, s.time, 'payload_separation_confirmed')
                return Demand(direction=s.up, gear=False)
            if elapsed > 8:
                self.abort('separation_not_confirmed')
                return Demand(direction=s.up)
            # One irreversible request per mission. A lost request fails closed;
            # an unconfirmed command is never treated as successful deployment.
            send = not self.separation_sent
            self.separation_sent = True
            return Demand(direction=s.up, gear=False, separate=send)
        if self.phase == Phase.CLEARANCE:
            if elapsed >= 5:
                next_phase = Phase.ENTRY if c.profile == 'hop' else Phase.DEORBIT_WAIT
                self.transition(next_phase, s.time, 'payload_clearance_elapsed')
            return Demand(direction=s.up, gear=False)
        if self.phase == Phase.DEORBIT_WAIT:
            entry_lon = c.target_longitude-c.deorbit_lead_angle
            delta = (entry_lon-s.longitude+180)%360-180
            if abs(delta) < 1.:
                self.transition(Phase.DEORBIT, s.time, 'deorbit_window')
            return Demand(direction=tuple(-v for v in unit(s.orbital_velocity)), gear=False)
        if self.phase == Phase.DEORBIT:
            retrograde = tuple(-v for v in unit(s.orbital_velocity))
            if s.periapsis <= c.deorbit_periapsis:
                self.transition(Phase.ENTRY, s.time, 'entry_trajectory_confirmed')
                return Demand(direction=retrograde, gear=False)
            if elapsed > 120 or s.fuel < self.initial_fuel*c.reserve_fraction:
                self.abort('deorbit_failed')
                return Demand(direction=retrograde, gear=False)
            return Demand(.35, retrograde, False)
        if self.phase == Phase.ENTRY:
            h = max(0., s.agl-max(c.landing_clearance, self.initial_agl))
            braking = max(.1, thrust/s.mass-s.gravity)
            stopping = max(0., -s.vertical_speed)**2/(2*braking)
            if (c.profile == 'orbital' and s.agl < 4000 and s.vertical_speed < 0 and s.horizontal_speed > 40
                    and h < norm(s.velocity)**2/(2*braking)*2.+abs(s.vertical_speed)*5.+1000.):
                self.transition(Phase.BRAKING, s.time, 'surface_velocity_braking_window')
                return Demand(1., unit(combine(s.velocity, s.up, -.5, s.gravity)), False)
            if s.vertical_speed < 0 and h <= max(250., stopping*1.6+abs(s.vertical_speed)*3):
                self.transition(Phase.APPROACH, s.time, 'landing_burn_window')
                return self.landing_demand(s, thrust, approach=True)
            retrograde = tuple(-v for v in unit(s.velocity)) if norm(s.velocity) > 20 else s.up
            return Demand(0., retrograde, False)
        if self.phase == Phase.BRAKING:
            if s.horizontal_speed < 20 and abs(s.vertical_speed) < 65:
                self.transition(Phase.APPROACH, s.time, 'horizontal_velocity_arrested')
                return self.landing_demand(s, thrust, approach=True)
            if elapsed > 180:
                self.abort('surface_braking_timeout')
                return Demand(direction=s.up)
            # Gravity-compensated retropropulsion removes horizontal velocity
            # before the near-vertical terminal descent controller takes over.
            desired = combine(s.velocity, s.up, -.5, s.gravity)
            return Demand(clip(norm(desired)*s.mass/thrust, 0., 1.), unit(desired), False)
        if self.phase == Phase.APPROACH:
            h = max(0., s.agl-max(c.landing_clearance, self.initial_agl))
            # Establish a clear landing footprint, then stop translating at
            # altitude. Terminal descent cannot begin while drifting/tilted.
            north_error = math.radians(c.target_latitude-s.latitude)*s.radius
            east_error = math.radians((c.target_longitude-s.longitude+180)%360-180)*s.radius*math.cos(math.radians(s.latitude))
            distance = math.hypot(north_error, east_error)
            if distance < 30. or distance >= 15000.:
                self.approach_braking = True
            stable = (self.approach_braking and abs(h-c.settle_altitude) < 5.
                      and abs(s.vertical_speed) < .5 and s.horizontal_speed < .3
                      and dot(s.up, (1., 0., 0.)) > .999 and norm(s.angular_velocity) < .01)
            self.settled_since = (self.settled_since if self.settled_since is not None else s.time) if stable else None
            if self.settled_since is not None and s.time-self.settled_since >= 3.:
                self.transition(Phase.LANDING, s.time, 'high_altitude_drift_arrest_confirmed')
                return self.landing_demand(s, thrust)
            if elapsed > 180.:
                self.abort('approach_not_settled')
                return Demand(direction=s.up)
            return self.landing_demand(s, thrust, approach=True)
        if self.phase == Phase.LANDING:
            if elapsed > 240:
                self.abort('landing_timeout')
                return Demand(direction=s.up)
            return self.landing_demand(s, thrust)
        if self.phase == Phase.TOUCHDOWN:
            h = s.agl-max(c.landing_clearance, self.initial_agl)
            if (s.horizontal_speed > 2 or abs(s.vertical_speed) > 5 or h > 3
                    or dot(s.up, (1., 0., 0.)) < .98):
                self.abort('unstable_touchdown')
                return Demand(direction=s.up)
            # Stock suspension can rebound after a soft contact. Keep engines
            # off, and require a continuous settled interval after that rebound.
            stable = (s.landed and s.horizontal_speed < .2 and abs(s.vertical_speed) < .2
                      and norm(s.angular_velocity) < .01)
            self.touchdown_stable_since = (self.touchdown_stable_since if self.touchdown_stable_since is not None else s.time) if stable else None
            if self.touchdown_stable_since is not None and s.time-self.touchdown_stable_since >= 10.:
                if self.separation_confirmed:
                    self.transition(Phase.COMPLETE, s.time, 'payload_deployed_and_landed')
                else:
                    self.abort('payload_not_deployed')
            elif s.time-self.entered > 45:
                self.abort('touchdown_not_settled')
            return Demand(direction=s.up)
        return Demand(direction=s.up)


def attitude_inputs(direction, angular_velocity, config, dynamic_pressure=0.):
    """Point the thrust (+X) axis; damp roll and bound the target body rate."""
    target = unit(direction)
    angle = math.acos(clip(target[0], -1., 1.))
    cross = (0., -target[2], target[1])
    if norm(cross) < 1e-7 and target[0] < 0:
        cross = (0., 1., 0.)
    rate = tuple(x*min(config.rate_limit, angle*config.attitude_gain) for x in unit(cross)) if angle > 1e-6 else (0., 0., 0.)
    # Retain the verified low-speed landing response, but use the available fin
    # and gimbal authority against aerodynamic disturbance during ascent/entry.
    gain = config.rate_gain+(config.aero_rate_gain-config.rate_gain)*clip(dynamic_pressure/10000., 0., 1.)
    # Roll inertia is much smaller than pitch/yaw inertia. The larger dense-air
    # gain excited roll oscillation and wasted reaction-wheel power in flight.
    torque = tuple(clip((config.rate_gain if axis == 0 else gain)*(r-w), -1., 1.)
                   for axis, (r, w) in enumerate(zip(rate, angular_velocity)))
    # Positive stock flight inputs generate negative vessel-local angular
    # acceleration. Combine that with the axial-vector handedness conversion;
    # confirmed against KSP flight telemetry on all three axes.
    return -torque[1], -torque[2], torque[0]
