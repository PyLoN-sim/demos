"""Project camera colours onto observed terrain; no navigation or world truth."""
import numpy as np
from scipy.spatial import Delaunay, QhullError
from scipy.ndimage import minimum_filter


def unobstructed(world,camera,boxes):
    """Conservative segment/OBB visibility for the vehicle's primitive proxies."""
    visible=np.ones(len(world),dtype=bool)
    for pose,half in boxes:
        start=(camera-pose[:3,3])@pose[:3,:3]
        # The optical origin may be inside its own camera housing proxy.
        if np.all(np.abs(start)<=half):continue
        direction=(world-camera)@pose[:3,:3]
        parallel=np.abs(direction)<1e-9
        safe=np.where(parallel,1.,direction)
        first=(-half-start)/safe;last=(half-start)/safe
        low=np.where(parallel,-np.inf,np.minimum(first,last))
        high=np.where(parallel,np.inf,np.maximum(first,last))
        miss=np.any(parallel&(np.abs(start)>half),axis=1)
        enters=np.maximum(low.max(axis=1),0.)
        leaves=np.minimum(high.min(axis=1),1.)
        visible&=miss|(enters>=leaves)
    return visible


def photo_samples(ground, returns, map_camera, intrinsics, image, map_body,
                  footprint, rear_x, resolution=.1, max_range=12., occluders=()):
    """Orthorectify RGB on short LiDAR ground triangles, excluding sky and self.

    Optical coordinates are right/down/forward. Output XY lies in the local
    estimated map; Z is retained for visibility tests, not invented from a plane.
    """
    ground=np.asarray(ground,float).reshape(-1,3)
    camera=map_camera[:3,3]
    ground=ground[np.isfinite(ground).all(axis=1)]
    ground=ground[np.linalg.norm(ground-camera,axis=1)<max_range+2]
    empty=(np.empty((0,3)),np.empty((0,3),np.uint8),np.empty(0))
    if len(ground)<6: return empty
    try: mesh=Delaunay(ground[:,:2])
    except QhullError: return empty
    triangles=ground[mesh.simplices]
    edges=np.maximum.reduce([np.linalg.norm(triangles[:,i,:2]-triangles[:,j,:2],axis=1)
                              for i,j in ((0,1),(1,2),(2,0))])
    good=edges<=2.
    lo=np.floor(ground[:,:2].min(0)/resolution).astype(int)
    hi=np.ceil(ground[:,:2].max(0)/resolution).astype(int)
    yy,xx=np.mgrid[lo[1]:hi[1]+1,lo[0]:hi[0]+1]
    xy=np.column_stack([xx.ravel()+.5,yy.ravel()+.5])*resolution
    faces=mesh.find_simplex(xy)
    keep=(faces>=0)&good[np.maximum(faces,0)]
    xy,faces=xy[keep],faces[keep]
    if not len(xy): return empty
    bary=np.einsum('nij,nj->ni',mesh.transform[faces,:2],xy-mesh.transform[faces,2])
    weights=np.column_stack([bary,1-bary.sum(1)])
    z=np.sum(triangles[faces,:,2]*weights,axis=1)
    world=np.column_stack([xy,z])
    local=(world-map_body[:3,3])@map_body[:3,:3]
    poly=np.asarray(footprint)
    occupied=(local[:,0]>=poly[:,0].min()+rear_x)&(local[:,0]<=poly[:,0].max()+rear_x)&\
             (local[:,1]>=poly[:,1].min())&(local[:,1]<=poly[:,1].max())
    optical=(world-camera)@map_camera[:3,:3]
    distance=np.linalg.norm(optical,axis=1)
    incidence=(camera[2]-z)/np.maximum(distance,1e-6)
    valid=(~occupied)&(optical[:,2]>.3)&(distance<max_range)&(incidence>.18)
    valid&=unobstructed(world,camera,occluders)
    world,optical,distance,incidence=world[valid],optical[valid],distance[valid],incidence[valid]
    if not len(world): return empty
    K=np.asarray(intrinsics).reshape(3,3)
    pixels=optical@K.T; pixels=pixels[:,:2]/pixels[:,2,None]
    h,w=image.shape[:2]
    valid=(pixels[:,0]>=1)&(pixels[:,0]<w-2)&(pixels[:,1]>=1)&(pixels[:,1]<h-2)
    world,optical,distance,incidence,pixels=[v[valid] for v in (world,optical,distance,incidence,pixels)]
    if not len(world): return empty
    # A coarse first-return depth buffer masks terrain behind observed rocks.
    returns=np.asarray(returns,float).reshape(-1,3)
    obs=(returns-camera)@map_camera[:3,:3]
    obs=obs[np.isfinite(obs).all(axis=1)&(obs[:,2]>.3)]
    depth=np.full(((h+3)//4,(w+3)//4),np.inf)
    if len(obs):
        uv=obs@K.T; uv=uv[:,:2]/uv[:,2,None]
        inside=(uv[:,0]>=0)&(uv[:,0]<w)&(uv[:,1]>=0)&(uv[:,1]<h)
        bins=(uv[inside]/4).astype(int)
        np.minimum.at(depth,(bins[:,1],bins[:,0]),obs[inside,2])
        depth=minimum_filter(depth,size=3)
    bins=(pixels/4).astype(int)
    visible=optical[:,2]<=depth[bins[:,1],bins[:,0]]+.5
    world,pixels,distance,incidence=[v[visible] for v in (world,pixels,distance,incidence)]
    # Bilinear sampling avoids speckle as the camera moves between pixels.
    ij=pixels.astype(int); fraction=pixels-ij
    u,v=ij[:,0],ij[:,1]; fx,fy=fraction[:,0,None],fraction[:,1,None]
    rgb=(image[v,u]*(1-fx)*(1-fy)+image[v,u+1]*fx*(1-fy)+
         image[v+1,u]*(1-fx)*fy+image[v+1,u+1]*fx*fy)
    return world,np.clip(rgb,0,255).astype(np.uint8),incidence/np.maximum(distance,1.)**2


class PhotoMap:
    """Bounded 120 m atlas. Keep the closest, least grazing observation per cell."""
    def __init__(self,resolution=.1,size=120.):
        self.resolution=resolution;self.size=size;self.n=int(round(size/resolution))
        self.rgb=np.zeros((self.n,self.n,3),np.uint8)
        self.score=np.zeros((self.n,self.n),np.float32)
        self.frames=0

    def add(self,world,rgb,score):
        ij=np.floor((world[:,:2]+self.size/2)/self.resolution).astype(int)
        valid=np.all((ij>=0)&(ij<self.n),axis=1)
        ij,rgb,score=ij[valid],rgb[valid],score[valid]
        if not len(ij): return 0
        x,y=ij.T;valid=score>self.score[y,x]*1.03
        x,y,rgb,score=x[valid],y[valid],rgb[valid],score[valid]
        self.rgb[y,x]=rgb;self.score[y,x]=score
        if len(x):self.frames+=1
        return len(x)

    def cloud(self):
        y,x=np.nonzero(self.score)
        result=np.empty(len(x),dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('rgb','<u4')])
        result['x']=(x+.5)*self.resolution-self.size/2
        result['y']=(y+.5)*self.resolution-self.size/2
        # Orthographic photo layer above the 2D traversability map.
        result['z']=.04
        rgb=self.rgb[y,x].astype(np.uint32)
        result['rgb']=(rgb[:,0]<<16)|(rgb[:,1]<<8)|rgb[:,2]
        return result
