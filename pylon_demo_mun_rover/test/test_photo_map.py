import unittest
import numpy as np
from pylon_demo_mun_rover.photo_map import PhotoMap,photo_samples,unobstructed


class PhotoMapTests(unittest.TestCase):
    def setUp(self):
        self.ground=np.array([[x,y,0.] for x in np.arange(-4,4.1,.5) for y in np.arange(-4,4.1,.5)])
        self.camera=np.diag([1.,-1.,-1.,1.]);self.camera[:3,3]=[0.,0.,2.]
        self.body=np.eye(4);self.body[0,3]=-20.
        self.footprint=[[-1,-1],[1,-1],[1,1],[-1,1]]
        self.K=np.array([[50.,0,60],[0,50.,60],[0,0,1.]])
        y,x=np.mgrid[:120,:120];self.rgb=np.stack([x,y,np.full_like(x,80)],axis=2).astype(np.uint8)

    def samples(self,ground=None,returns=None):
        return photo_samples(self.ground if ground is None else ground,
            np.empty((0,3)) if returns is None else returns,self.camera,self.K,self.rgb,
            self.body,self.footprint,0.)

    def test_projected_colours_match_calibration_and_image_orientation(self):
        world,rgb,score=self.samples()
        self.assertGreater(len(world),1000)
        expected=np.column_stack([world[:,0]*25+60,-world[:,1]*25+60])
        np.testing.assert_allclose(rgb[:,:2],expected,atol=1.)
        self.assertTrue(np.all(rgb[:,2]==80));self.assertTrue(np.all(score>0))
        self.assertTrue(np.all(world[:,2]==0))

    def test_unknown_gap_self_footprint_and_occluder_are_not_photographed(self):
        gap=self.ground[np.abs(self.ground[:,0])>=1.5]
        world,_,_=self.samples(gap)
        self.assertFalse(np.any(np.abs(world[:,0])<1.4))
        self.body[0,3]=0.
        world,_,_=self.samples()
        self.assertFalse(np.any((np.abs(world[:,0])<1)&(np.abs(world[:,1])<1)))
        self.body[0,3]=-20.
        blocker=np.array([[x,y,1.] for x in np.arange(-.25,.26,.05) for y in np.arange(-.25,.26,.05)])
        world,_,_=self.samples(returns=blocker)
        self.assertFalse(np.any((np.abs(world[:,0])<.2)&(np.abs(world[:,1])<.2)))

    def test_upward_camera_has_no_ground_projection(self):
        self.camera[:3,:3]=np.eye(3)
        world,_,_=self.samples();self.assertEqual(len(world),0)

    def test_vehicle_proxy_masks_rays_beyond_wheels_not_just_the_footprint(self):
        pose=np.eye(4);pose[:3,3]=[2.,0.,0.]
        world=np.array([[4.,0.,0.],[4.,2.,0.],[1.,0.,0.],[0.,2.,0.]])
        np.testing.assert_array_equal(unobstructed(world,np.zeros(3),[(pose,np.full(3,.4))]),
                                      [False,True,True,True])
        # A camera housing enclosing the optical origin must not blank the map.
        self.assertTrue(unobstructed(world,np.zeros(3),[(np.eye(4),np.ones(3))]).all())

    def test_atlas_retains_old_coverage_and_prefers_better_views(self):
        atlas=PhotoMap(size=10.)
        self.assertEqual(atlas.add(np.array([[0.,0.,0.]]),np.array([[10,20,30]],np.uint8),np.array([1.])),1)
        atlas.add(np.array([[0.,0.,0.],[1.,0.,0.]]),np.array([[99,99,99],[40,50,60]],np.uint8),np.array([.5,1.]))
        cloud=atlas.cloud();self.assertEqual(len(cloud),2)
        self.assertEqual(cloud['rgb'][0],(10<<16)|(20<<8)|30)
        atlas.add(np.array([[100.,0.,0.]]),np.zeros((1,3),np.uint8),np.array([1.]))
        self.assertEqual(len(atlas.cloud()),2)
        self.assertEqual(len(PhotoMap(size=10.).cloud()),0)
