import math
import unittest
from dataclasses import replace
import numpy as np
from scipy.spatial.transform import Rotation
from pylon_demo_mun_rover.core import Wheel,Geometry,Estimator,limited_speed
from pylon_demo_mun_rover.terrain import Terrain


def geometry():
    wheels=[Wheel(f'{x}_{y}',x,y,-0.7,0.3,sign,1.,x>0,0.5) for x in (-1.,1.) for y,sign in ((-1.,-1.),(1.,1.))]
    return Geometry(wheels,[-1.5,-1.5,-1.],[1.5,1.5,1.])


class CoreTests(unittest.TestCase):
    def test_ackermann_inner_outer_and_reverse_mount(self):
        g=geometry();cmd=g.commands(.4,.08)
        self.assertGreater(cmd['1.0_1.0'][1],cmd['1.0_-1.0'][1])
        self.assertLess(abs(cmd['1.0_1.0'][0]),abs(cmd['1.0_-1.0'][0]))
        self.assertLess(cmd['-1.0_-1.0'][0],0)
        self.assertEqual(cmd['-1.0_1.0'][1],0.)
        self.assertTrue(all(abs(x[1])<=.5 for x in g.commands(.5,100).values()))
        self.assertTrue(all(x==(0.,0.) for x in g.commands(0.,1.).values()))
    def test_geometry_rejects_unsupported(self):
        g=geometry()
        with self.assertRaises(ValueError):Geometry(g.wheels[:3],[-1]*3,[1]*3)
        w=list(g.wheels);w[0]=Wheel('bad',-1.,0.2,-1.,.3,0.,1.,False,0.)
        with self.assertRaises(ValueError):Geometry(w,[-1]*3,[1]*3)
    def test_speed_sign_and_acceleration(self):
        g=geometry();cmd=g.commands(.4,0.)
        speed,slip=g.speed({k:(v[0],0.,v[1]) for k,v in cmd.items()},0.)
        self.assertAlmostEqual(speed,.4);self.assertAlmostEqual(slip,0.)
        self.assertAlmostEqual(limited_speed(0.,.5,.05),.01)
        with self.assertRaises(ValueError):g.commands(-1,0)
    def test_all_wheel_counter_steering_and_measured_odometry(self):
        original=geometry()
        for middle in (False,True):
            wheels=[replace(w,steering=True,steering_sign=-1.) for w in original.wheels]
            if middle:
                wheels += [Wheel(f'mid{y}',0.,y,-.7,.3,1.,-1.,False,.5) for y in (-1.,1.)]
            g=Geometry(wheels,[-1.5]*3,[1.5]*3)
            self.assertEqual(g.steering_mode,'counter_phase')
            self.assertEqual(g.rear_x,0.)
            self.assertLess(g.min_radius,original.min_radius)
            for omega in (-.08,.08):
                cmd=g.commands(.4,omega)
                self.assertLess(cmd['1.0_1.0'][1]*cmd['-1.0_1.0'][1],0.)
                for w in wheels:
                    angular,angle=cmd[w.id]
                    speed=angular*w.radius*w.rolling_sign
                    self.assertAlmostEqual(speed*math.cos(angle*w.steering_sign),.4-omega*w.y)
                    if w.steering:self.assertAlmostEqual(speed*math.sin(angle*w.steering_sign),omega*w.x)
                speed,slip=g.speed({k:(c[0],0.,c[1]) for k,c in cmd.items()},omega)
                self.assertAlmostEqual(speed,.4);self.assertAlmostEqual(slip,0.)
            self.assertTrue(all(abs(c[1])<=.5 for c in g.commands(.5,100).values()))

    def test_front_steering_odometry_during_turn(self):
        g=geometry();cmd=g.commands(.4,.08)
        speed,slip=g.speed({k:(v[0],0.,v[1]) for k,v in cmd.items()},.08)
        self.assertAlmostEqual(speed,.4);self.assertAlmostEqual(slip,0.)
    def test_gravity_and_prediction(self):
        est=Estimator();R=Rotation.from_euler('y',.1).as_matrix()
        self.assertTrue(est.initialize([R.T@np.array([0,0,1.63])]*35,[[0,0,0]]*35))
        self.assertTrue(np.allclose(est.pose[:3,:3]@R.T@np.array([0,0,1.]),[0,0,1.]))
        for _ in range(100):est.predict([0,0,0],.4,0.,.1)
        self.assertAlmostEqual(np.linalg.norm(est.pose[:3,3]),4.)
        old=est.pose.copy();self.assertFalse(est.predict([0,0,0],1.,0.,.6));np.testing.assert_equal(old,est.pose)
    def test_plane_registration_does_not_invent_tangential_motion(self):
        est=Estimator();est.initialize([[0,0,1.63]]*35,[[0,0,0]]*35)
        plane=np.array([[x,y,-1.] for x in np.arange(-4,4,.2) for y in np.arange(-4,4,.2)])
        self.assertTrue(est.correct(plane))
        est.pose[0,3]=.13;self.assertTrue(est.correct(plane-np.array([.13,0,0])))
        self.assertLess(est.observed_rank,6)
        self.assertAlmostEqual(est.pose[0,3],.13,places=4)
        before=est.sigma;est.predict([0,0,0],.4,0,.1,slip=1.)
        self.assertGreater(est.sigma,before)
    def test_terrain_flat_slope_step_and_no_returns(self):
        for grade,blocked in ((0.,False),(.4,True)):
            t=Terrain(size=20)
            p=np.array([[x,y,grade*x] for x in np.arange(-2,2,.1) for y in np.arange(-2,2,.1)])
            t.update(p);ij=t.index([0,0]);self.assertEqual(t.grid[ij[1],ij[0]],100 if blocked else 0)
            ij=t.index([5,5]);self.assertEqual(t.grid[ij[1],ij[0]],-1)
        t=Terrain(size=20);p=np.array([[x,y,.6 if x>=0 else 0.] for x in np.arange(-2,2,.1) for y in np.arange(-2,2,.1)])
        t.update(p);ij=t.index([0,0]);self.assertEqual(t.grid[ij[1],ij[0]],100)
        self.assertFalse(t.corridor_safe([4,4],0,[[-.2,-.2],[.2,-.2],[.2,.2],[-.2,.2]],.5,0))
    def test_gyro_turn_prediction_about_fixed_rear_axle(self):
        est=Estimator();est.initialize([[0,0,1.63]]*35,[[0,0,0]]*35)
        rear_x=-1.;initial=est.pose[:3,3]+np.array([rear_x,0.,0.])
        for _ in range(100):est.predict([0,0,.03],.3,rear_x,.1)
        rear=est.pose[:3,3]+est.pose[:3,:3]@np.array([rear_x,0.,0.])
        np.testing.assert_allclose(rear-initial,[10*math.sin(.3),10*(1-math.cos(.3)),0.],atol=1e-4)

    def test_three_dimensional_returns_correct_wheel_slip(self):
        est=Estimator();est.initialize([[0,0,1.63]]*35,[[0,0,0]]*35)
        grid=np.arange(-3,3,.2)
        world=np.array([[x,y,-1.] for x in grid for y in grid]+
                       [[4.,y,z] for y in grid for z in np.arange(-1,2,.2)]+
                       [[x,4.,z] for x in grid for z in np.arange(-1,2,.2)])
        self.assertTrue(est.correct(world))
        est.pose[0,3]=.35
        self.assertTrue(est.correct(world-np.array([.2,0,0])))
        self.assertEqual(est.observed_rank,6)
        self.assertAlmostEqual(est.pose[0,3],.2,delta=.04)

    def test_rocks_and_cliff_gap_are_not_cleared(self):
        t=Terrain(size=20)
        ground=np.array([[x,y,0.] for x in np.arange(-4,4,.1) for y in np.arange(-3,3,.1) if abs(x)>1.25])
        rock=np.array([[-2.+x,-.3+y,.4] for x in np.arange(0,.5,.1) for y in np.arange(0,.5,.1)])
        t.update(np.vstack([ground,rock]))
        gap=t.index([0.,0.]);self.assertEqual(t.grid[gap[1],gap[0]],-1)
        hit=t.index([-1.75,0.]);self.assertEqual(t.grid[hit[1],hit[0]],100)

    def test_stationary_init_rejects_wrong_body_gravity_and_noise(self):
        est=Estimator();self.assertFalse(est.initialize([[0,0,9.81]]*35,[[0,0,0]]*35))
        self.assertFalse(est.initialize([[0,0,0]]*35,[[0,0,0]]*35))
        self.assertFalse(est.correct(np.empty((0,3))))

if __name__=='__main__':unittest.main()
