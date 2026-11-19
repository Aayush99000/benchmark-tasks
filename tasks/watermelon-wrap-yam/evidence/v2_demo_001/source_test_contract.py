import unittest
import numpy as np
from film import Film,EXIT,WIDTH,MELON,RADIUS
from scoring import Evaluator,coverage,tucks

class ContractTests(unittest.TestCase):
    def test_reset_has_no_score(self):
        f=Film();e=Evaluator();r=e.update(f,MELON,np.eye(3),np.zeros((2,3)),.1)
        self.assertEqual(r['score'],0);self.assertFalse(r['cut'])
    def test_remote_closed_gripper_does_not_attach(self):
        f=Film();f.update_pinch(np.array([[0,0,2],[0,0,2]]),np.array([np.eye(3)]*2),[0,0]);self.assertEqual(len(f.pins),0)
    def test_open_jaws_do_not_attach(self):
        f=Film();f.update_pinch(np.array([f.x[0,0],f.x[0,-1]]),np.array([np.eye(3)]*2),[.06,.06]);self.assertEqual(len(f.pins),0)
    def test_closed_edge_pinches_and_release(self):
        f=Film();p=np.array([f.x[0,0],f.x[0,-1]]);r=np.array([np.eye(3)]*2)
        f.update_pinch(p,r,[.004,.004]);self.assertEqual(len(f.pins),2)
        f.update_pinch(p,r,[.05,.05]);self.assertEqual(len(f.pins),0)
    def test_same_edge_two_pinches_not_bimanual_draw(self):
        f=Film();p=np.array([f.x[0,0],f.x[1,0]])
        f.update_pinch(p,np.array([np.eye(3)]*2),[.004,.004])
        self.assertEqual(len(f.pins),2);self.assertFalse(f.both_pinch_seen)
    def test_opposite_edges_do_witness_bimanual_draw(self):
        f=Film();p=np.array([f.x[0,0],f.x[0,-1]])
        f.update_pinch(p,np.array([np.eye(3)]*2),[.004,.004])
        self.assertTrue(f.both_pinch_seen)
    def test_partial_sweep_cannot_cut(self):
        f=Film()
        for y in np.linspace(.215,0,10):f.cutter(EXIT+[0,y,.023],True)
        self.assertFalse(f.cut)
    def test_off_rail_cannot_cut(self):
        f=Film()
        for y in np.linspace(.215,-.215,20):f.cutter(EXIT+[.03,y,.023],True)
        self.assertFalse(f.cut)
    def test_full_sweep_cuts(self):
        f=Film()
        for y in np.linspace(.215,-.215,30):f.cutter(EXIT+[0,y,.023],True)
        self.assertTrue(f.cut)
    def test_premature_cut_latches_zero(self):
        f=Film();e=Evaluator();f.both_pinch_seen=True;f.paid=.4
        for y in np.linspace(.215,-.215,30):f.cutter(EXIT+[0,y,.023],False)
        self.assertTrue(f.cut_before_tuck);self.assertEqual(e.update(f,MELON,np.eye(3),np.zeros((2,3)),1.)['score'],0)
    def test_film_above_face_not_coverage(self):
        p=np.array([[x,y,.03] for x,y in [(-.11,-.11),(.11,-.11),(.11,.11),(-.11,.11)]])+MELON
        self.assertEqual(coverage(p,np.array([[0,1,2],[0,2,3]]),MELON,np.eye(3)),0)
    def test_surface_cover_is_coverage(self):
        p=np.array([[x,y,.002] for x,y in [(-.11,-.11),(.11,-.11),(.11,.11),(-.11,.11)]])+MELON
        self.assertEqual(coverage(p,np.array([[0,1,2],[0,2,3]]),MELON,np.eye(3)),1)
        self.assertEqual(tucks(p,MELON,np.eye(3)),[False,False])
    def test_torn_film_zero(self):
        f=Film();f.both_pinch_seen=True;f.paid=.4;f.torn=True
        self.assertEqual(Evaluator().update(f,MELON,np.eye(3),np.zeros((2,3)),1.)['score'],0)
    def test_nonfinite_zero(self):
        f=Film();f.x[0,0,0]=float('nan')
        self.assertEqual(Evaluator().update(f,MELON,np.eye(3),np.zeros((2,3)),1.)['score'],0)
    def test_one_arm_not_bimanual_draw(self):
        f=Film();f.paid=.4
        self.assertEqual(Evaluator().update(f,MELON,np.eye(3),np.zeros((2,3)),1.)['score'],0)
    def test_idle_model_no_feed_or_wrap(self):
        f=Film()
        for _ in range(5):f.step(np.array([[0,0,1.2],[0,0,1.2]]),np.array([np.eye(3)]*2),[.09,.09])
        self.assertAlmostEqual(f.paid,.075);self.assertFalse(f.cut);self.assertFalse(f.torn)
if __name__=='__main__':unittest.main()
