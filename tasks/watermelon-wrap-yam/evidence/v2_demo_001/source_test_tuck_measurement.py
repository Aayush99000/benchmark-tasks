import unittest
import numpy as np
from tuck_measurement import measure_tucks

def patch(sign=1,half_width=.02):
    # Physical spherical rind patch: 40 mm wide by 18 mm projected height.
    x,z=np.meshgrid(np.linspace(-half_width,half_width,9),np.linspace(-.075,-.057,9))
    y=sign*np.sqrt(.09**2-x*x-z*z)
    return np.stack([x,y,z],axis=-1)

class TuckTests(unittest.TestCase):
    def test_one_patch_does_not_count_twice(self):
        p=measure_tucks(patch(),np.zeros(3),np.eye(3))
        self.assertTrue(p[0]['passed']);self.assertFalse(p[1]['passed'])
    def test_narrow_patch_fails(self):
        self.assertFalse(measure_tucks(patch(half_width=.004),np.zeros(3),np.eye(3))[0]['passed'])
    def test_far_from_rind_fails(self):
        self.assertFalse(measure_tucks(patch()*1.3,np.zeros(3),np.eye(3))[0]['passed'])
    def test_rotation_invariant(self):
        p=patch();r=np.array([[0,-1,0],[1,0,0],[0,0,1]])
        self.assertEqual(measure_tucks(p,np.zeros(3),np.eye(3)),measure_tucks(p@r.T,np.zeros(3),r))
    def test_duplicate_faces_do_not_multiply_area(self):
        from tuck_measurement import mesh_faces
        p=patch();f=mesh_faces(p)
        self.assertEqual(measure_tucks(p,np.zeros(3),np.eye(3),f),
                         measure_tucks(p,np.zeros(3),np.eye(3),np.vstack([f,f,f])))
    def test_nonfinite_rejected(self):
        p=patch();p[0,0]=float('nan')
        self.assertFalse(any(t['passed'] for t in measure_tucks(p,np.zeros(3),np.eye(3))))

if __name__=='__main__':unittest.main()
