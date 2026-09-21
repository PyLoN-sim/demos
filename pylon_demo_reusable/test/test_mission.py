from dataclasses import replace
import math
import unittest
from pylon_demo_reusable.mission import Config, Mission, Sample, Phase, attitude_inputs


class MissionTests(unittest.TestCase):
    def setUp(self):
        self.s = Sample(0., 80., 12., 80000., 5760., 9.81, 0., landed=True)
        self.m = Mission()

    def test_preflight_rejects_midflight_and_insufficient_twr(self):
        with self.assertRaises(ValueError):
            self.m.start(replace(self.s, landed=False))
        self.m.start(self.s)
        self.assertEqual(self.m.step(self.s, 100000).throttle, 0.)
        self.assertEqual(self.m.reason, 'insufficient_launch_twr')

    def test_orbit_gate_prevents_premature_payload_release(self):
        self.m.start(self.s)
        self.m.transition(Phase.CIRCULARIZE, 0., 'test')
        s = replace(self.s, time=1., landed=False, altitude=90000, agl=90000,
                    apoapsis=90000, periapsis=65000, fuel=1000., mass=20000.)
        self.assertFalse(self.m.step(s, 1500000).separate)
        self.assertEqual(self.m.phase, Phase.CIRCULARIZE)
        self.m.step(replace(s, time=2., periapsis=79000), 1500000)
        self.assertEqual(self.m.phase, Phase.DEPLOY)

    def test_dense_air_turn_limits_angle_of_attack(self):
        self.m.start(self.s)
        self.m.transition(Phase.ASCENT, 0., 'test')
        s = replace(self.s, time=30., landed=False, altitude=10000., agl=10000.,
                    vertical_speed=300., velocity=(300., 10., 0.), dynamic_pressure=20000.)
        demand = self.m.step(s, 1500000.)
        velocity = math.sqrt(sum(v*v for v in s.velocity))
        angle = math.degrees(math.acos(sum(a*b for a, b in zip(demand.direction, s.velocity))/velocity))
        self.assertLessEqual(angle, 3.01)
        s = replace(s, time=31., altitude=1900., dynamic_pressure=0.)
        self.assertAlmostEqual(self.m.step(s, 1500000.).direction[0], 1.)

    def test_ballistic_apoapsis_alone_does_not_end_ascent(self):
        self.m.start(self.s)
        self.m.transition(Phase.ASCENT, 0., 'test')
        s = replace(self.s, time=100., landed=False, altitude=40000., agl=40000.,
                    apoapsis=81000., orbital_velocity=(750., 400., 0.))
        self.assertGreater(self.m.step(s, 1500000.).throttle, 0.)
        self.assertEqual(self.m.phase, Phase.ASCENT)

    def test_separation_is_once_and_must_be_confirmed(self):
        self.m.start(self.s)
        self.m.transition(Phase.DEPLOY, 0., 'test')
        self.assertTrue(self.m.step(self.s, 1500000).separate)
        self.assertFalse(self.m.step(replace(self.s, time=1.), 1500000).separate)
        self.m.step(replace(self.s, time=9.), 1500000)
        self.assertEqual(self.m.phase, Phase.ABORT)
        self.assertFalse(self.m.separation_confirmed)

    def test_ack_before_command_does_not_confirm_deployment(self):
        self.m.start(self.s)
        self.m.step(self.s, 1500000, separation_confirmed=True)
        self.assertFalse(self.m.separation_confirmed)

    def test_clock_reversal_aborts_and_cannot_resume(self):
        self.m.start(self.s)
        self.m.step(replace(self.s, time=2.), 1500000)
        self.assertEqual(self.m.step(replace(self.s, time=1.), 1500000).throttle, 0.)
        self.assertEqual(self.m.phase, Phase.ABORT)
        self.assertEqual(self.m.step(replace(self.s, time=3.), 1500000).throttle, 0.)

    def test_ballistic_prediction_and_closed_loop_hop_lands(self):
        # Independent vertical point-mass plant, gravity and fuel consumption.
        # This exercises the entire hop mission, not KSP aerodynamics/attitude.
        mission = Mission(Config(profile='hop', hop_altitude=500., target_latitude=self.s.latitude))
        mission.start(self.s)
        h, v, mass, fuel, deployed = 12., 0., 80000., 5760., False
        peak, impact_speed = h, None
        for i in range(1, 10000):
            t, dt = i*.05, .05
            s = replace(self.s, time=t, altitude=h+68, agl=h, vertical_speed=v,
                        mass=mass, fuel=fuel, landed=h <= 12., velocity=(v, 0., 0.))
            demand = mission.step(s, 1500000., deployed)
            if demand.separate:
                deployed = True
            force = demand.throttle*1500000.
            used = force/(300.*9.80665)*dt
            mass -= used
            fuel -= used*.09
            v += (force/mass-9.81)*dt
            h += v*dt
            if h < 12.:
                if mission.ever_flew and impact_speed is None:
                    impact_speed = abs(v)
                h, v = 12., 0.
            peak = max(peak, h)
            if mission.terminal:
                break
        self.assertEqual(mission.phase, Phase.COMPLETE, mission.reason)
        self.assertTrue(deployed)
        self.assertGreater(peak, 450.)
        self.assertLess(impact_speed, 2.)
        self.assertGreater(fuel, 0.)
        phases = [e.phase for e in mission.transitions]
        self.assertEqual(phases, ['IGNITION', 'ASCENT', 'COAST', 'DEPLOY', 'CLEARANCE',
                                 'ENTRY', 'APPROACH', 'LANDING', 'TOUCHDOWN', 'COMPLETE'])
        touchdown, complete = mission.transitions[-2:]
        self.assertGreaterEqual(complete.time-touchdown.time, 10.)

    def test_touchdown_must_settle_before_success(self):
        self.m.start(self.s); self.m.ever_flew = True
        self.m.separation_sent = self.m.separation_confirmed = True
        self.m.transition(Phase.LANDING, 0., 'test')
        self.m.step(replace(self.s, time=1.), 1500000.)
        self.m.step(replace(self.s, time=5., angular_velocity=(0., .02, 0.)), 1500000.)
        self.assertEqual(self.m.phase, Phase.TOUCHDOWN)
        self.m.step(replace(self.s, time=12., up=(.95, .312, 0.)), 1500000.)
        self.assertEqual(self.m.phase, Phase.ABORT)

    def test_orbital_approach_preserves_fuel_and_lands_in_vertical_plant(self):
        # Independent point-mass descent from the post-braking altitude. This
        # catches prolonged high-altitude hovering, but does not model KSP aero.
        mission = Mission(Config(target_latitude=self.s.latitude))
        mission.start(self.s)
        mission.ever_flew = mission.separation_sent = mission.separation_confirmed = True
        mission.transition(Phase.APPROACH, 0., 'test')
        h, v, mass, fuel, impact = 3000., -50., 27500., 650., None
        for i in range(1, 10000):
            dt = .05
            s = replace(self.s, time=i*dt, altitude=h+68., agl=h,
                        vertical_speed=v, velocity=(v, 0., 0.), mass=mass,
                        fuel=fuel, landed=h <= 12.)
            force = mission.step(s, 1500000.).throttle*1500000.
            used = force/(300.*9.80665)*dt
            mass -= used
            fuel -= used*.09
            v += (force/mass-9.81)*dt
            h += v*dt
            if h < 12.:
                if impact is None:
                    impact = abs(v)
                h, v = 12., 0.
            if mission.terminal:
                break
        self.assertEqual(mission.phase, Phase.COMPLETE, mission.reason)
        self.assertLess(impact, 2.)
        self.assertGreater(fuel, 650.*mission.config.reserve_fraction)

    def test_terminal_descent_waits_for_high_altitude_drift_arrest(self):
        self.m.start(self.s)
        self.m.transition(Phase.APPROACH, 0., 'test')
        s = replace(self.s, landed=False, agl=312., altitude=380.,
                    latitude=self.m.config.target_latitude, horizontal_speed=4.,
                    velocity=(0., 4., 0.))
        self.m.step(replace(s, time=1.), 1500000.)
        self.assertEqual(self.m.phase, Phase.APPROACH)
        s = replace(s, horizontal_speed=0., velocity=(0., 0., 0.))
        self.m.step(replace(s, time=2.), 1500000.)
        self.m.step(replace(s, time=4.), 1500000.)
        self.assertEqual(self.m.phase, Phase.APPROACH)
        self.m.step(replace(s, time=5.1), 1500000.)
        self.assertEqual(self.m.phase, Phase.LANDING)

    def test_suspension_rebound_restarts_settling_timer(self):
        self.m.start(self.s); self.m.ever_flew = True
        self.m.separation_sent = self.m.separation_confirmed = True
        self.m.transition(Phase.LANDING, 0., 'test')
        self.m.step(replace(self.s, time=1.), 1500000.)
        self.m.step(replace(self.s, time=2., landed=False, agl=12.2, vertical_speed=2.3), 1500000.)
        self.assertEqual(self.m.phase, Phase.TOUCHDOWN)
        self.m.step(replace(self.s, time=3.), 1500000.)
        self.m.step(replace(self.s, time=12.), 1500000.)
        self.assertEqual(self.m.phase, Phase.TOUCHDOWN)
        self.m.step(replace(self.s, time=13.1), 1500000.)
        self.assertEqual(self.m.phase, Phase.COMPLETE)

    def test_stock_input_signs_match_flight_observation(self):
        pitch, yaw, roll = attitude_inputs((1., 0., 0.), (.1, .2, .3), Config())
        # Measured KSP responses: positive pitch/yaw produce negative body Y/Z
        # angular acceleration; positive roll produces positive body X.
        self.assertGreater(pitch, 0.)
        self.assertGreater(yaw, 0.)
        self.assertLess(roll, 0.)

    def test_aerodynamic_authority_preserves_low_speed_landing_gains(self):
        c = Config(); direction = (1., .1, .1); rates = (.02, .01, -.01)
        baseline = attitude_inputs(direction, rates, c)
        self.assertEqual(baseline, attitude_inputs(direction, rates, replace(c, aero_rate_gain=15.), 0.))
        dense = attitude_inputs(direction, rates, c, 25000.)
        self.assertTrue(all(abs(x) <= 1. for x in dense))
        self.assertGreater(abs(dense[0]), abs(baseline[0]))
        self.assertEqual(dense[2], baseline[2])  # Roll must not inherit fin pitch/yaw gain.

    def test_water_or_crash_is_never_success(self):
        for kwargs in [dict(splashed=True), dict(landed=True, vertical_speed=-20.)]:
            m = Mission(); m.start(self.s); m.ever_flew = True
            m.transition(Phase.LANDING, 0., 'test')
            m.step(replace(self.s, time=1., **kwargs), 1500000.)
            self.assertEqual(m.phase, Phase.ABORT)

    def test_attitude_opposite_direction_and_damping(self):
        for target in [(1., 0., 0.), (-1., 0., 0.), (0., 1., 0.), (0., 0., 1.)]:
            inputs = attitude_inputs(target, (.1, .2, .3), Config())
            self.assertTrue(all(math.isfinite(x) and abs(x) <= 1 for x in inputs))
        self.assertNotEqual(attitude_inputs((-1., 0., 0.), (0., 0., 0.), Config()), (0., 0., 0.))

    def test_invalid_configuration(self):
        for kwargs in [dict(profile='bad'), dict(orbit_altitude=50000.),
                       dict(rate_limit=0.), dict(reserve_fraction=float('nan'))]:
            with self.assertRaises(ValueError):
                Config(**kwargs)


if __name__ == '__main__':
    unittest.main()
