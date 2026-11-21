import unittest
from runtime_checks import LatchMonitor,FrameDeliveryMonitor
from scorer import RecoveryScorer

class Checks(unittest.TestCase):
 def seated(self):
  m=LatchMonitor()
  for k in range(31):m.update((0,0,.03-k*.001),0,.02,0)
  for _ in range(4):m.update((0,0,0),0,0,0)
  self.assertTrue(m.attached)
  return m
 def test_endpoint_without_entry(self):
  m=LatchMonitor()
  for _ in range(100):self.assertFalse(m.update((0,0,0),0,0,0))
 def test_unlock_clears_score_and_can_reengage_if_still_seated(self):
  m=self.seated();s=RecoveryScorer()
  args=dict(lifted=False,held=False,position_error_m=0,rotation_error_rad=0,other_hand_clear=True,live_image=True)
  for _ in range(41):v=s.update(latched=m.attached,**args)
  self.assertEqual(v['score'],1.)
  self.assertFalse(m.update((0,0,0),0,0,.003))
  v=s.update(latched=m.attached,**args);self.assertLess(v['score'],.75);self.assertEqual(v['release_time'],0.)
  self.assertTrue(m.update((0,0,0),0,0,0))
  self.assertLess(s.update(latched=m.attached,**args)['score'],1.)
 def test_leaving_receiver_clears_history(self):
  m=self.seated();self.assertFalse(m.update((0,0,.02),0,.01,0))
  for _ in range(10):self.assertFalse(m.update((0,0,0),0,0,0))
 def test_frame_delivery(self):
  m=FrameDeliveryMonitor();self.assertFalse(m.update(1,True));self.assertTrue(m.update(2,True))
  self.assertFalse(m.update(2,True));self.assertFalse(m.update(3,False));self.assertTrue(m.update(4,True));self.assertFalse(m.update(0,True))
if __name__=='__main__':unittest.main()
