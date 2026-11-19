"""Numerical equivalence of the optional accelerator and original projection."""
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from contact_fast import kernel,project

@unittest.skipIf(kernel is None,'Build contact_kernel.so to test the optional backend')
class ContactKernel(unittest.TestCase):
    def test_random_rotated_contact_projection_matches_numpy(self):
        rng=np.random.default_rng(17)
        for _ in range(20):
            transforms=np.repeat(np.eye(4)[None],16,axis=0)
            transforms[:,:3,:3]=Rotation.random(16,random_state=rng).as_matrix()
            transforms[:,:3,3]=rng.uniform(-.04,.04,(16,3))
            half=rng.uniform(.008,.04,(16,3));bounds=np.stack([-half,half],axis=1)
            extent=np.einsum('bij,bj->bi',abs(transforms[:,:3,:3]),half+.0004)
            centre=transforms[:,:3,3];aabbs=np.stack([centre-extent,centre+extent],axis=1)
            before=rng.uniform(-.09,.09,(150,3));expected=before.copy();actual=before.copy()
            pinned=(rng.random(150)<.05).astype(np.uint8)
            for world,b in zip(transforms,bounds):
                p=(expected-world[:3,3])@world[:3,:3];lo=b[0]-.0004;hi=b[1]+.0004
                ids=np.flatnonzero(np.all((p>lo)&(p<hi),axis=1)&~pinned.astype(bool))
                if not len(ids):continue
                low=p[ids]-lo;high=hi-p[ids];axis=np.argmin(np.minimum(low,high),axis=1)
                p[ids,axis]=np.where(low[np.arange(len(ids)),axis]<high[np.arange(len(ids)),axis],lo[axis],hi[axis])
                expected[ids]=p[ids]@world[:3,:3].T+world[:3,3]
            project(actual,transforms,bounds,aabbs,pinned)
            np.testing.assert_allclose(actual,expected,atol=1e-12,rtol=0)
            np.testing.assert_array_equal(actual[pinned.astype(bool)],before[pinned.astype(bool)])

if __name__=='__main__':unittest.main()
