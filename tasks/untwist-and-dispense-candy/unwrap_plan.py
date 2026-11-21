"""Bimanual playback expressed as robot joint targets. Poses use the reset layout."""
import numpy as np
from yam_arena import station
from yam_arena.constants import READY
from yam_arena.urdf import axis_angle_matrix
from reach_audit import solve
from grasp_probe import BASELINE

END = 430


def symmetric_plan():
    plan = np.tile(np.r_[READY,1.,READY,1.],(END+1,1))
    for side,sign in [('left',1),('right',-1)]:
        down = np.diag([1.,-1.,-1.])
        # Tilt both wrists outward so their housings clear at a shared grasp.
        rotation = axis_angle_matrix(np.array([1.,0.,0.]),np.radians(-sign*35))@down
        base = station.BASES[side]
        anchor = np.array([.38, sign*.031, .756])
        lifted = anchor+[0.,sign*.003,.050]
        pull = lifted+[0.,sign*.046,0.]
        knots=[(BASELINE,None,1.),(124,anchor+[0.,sign*.02,.075],1.),
               (153,anchor,1.),(175,anchor,0.),(202,lifted,0.),(233,pull,0.),
               (243,pull,0.)]
        if side=='left':
            knots += [(272,pull+[-.035,.025,.045],0.),(303,pull+[-.045,.08,.015],0.),
                      (321,pull+[-.045,.08,.015],1.),(350,None,1.),(END,None,1.)]
        else:
            knots += [(256,pull,1.),(277,anchor+[0.,-.10,.085],1.),
                      (300,(.38,0.,.82),1.),(328,(.38,0.,.757),1.),
                      (346,(.38,0.,.757),0.),(365,(.38,0.,.84),0.),
                      (391,(.38,-.22,.84),0.),(407,(.38,-.22,.767),0.),
                      (416,(.38,-.22,.767),1.),(END,(.38,-.22,.84),1.)]
        prev=np.r_[READY,1.]
        prev_step=0
        sl=slice(0,7) if side=='left' else slice(7,14)
        for step,point,opening in knots:
            q=np.array(READY)
            if point is not None:
                q,ep,er=solve(point,rotation,prev[:6],base)
                if ep>.002 or er>.03:
                    raise RuntimeError(f'{side} waypoint {step}: {ep:g} m {er:g} rad')
            curr=np.r_[q,opening]
            t=np.linspace(0.,1.,step-prev_step+1)[:,None]
            plan[prev_step:step+1,sl]=prev+(curr-prev)*(t*t*(3.-2.*t))
            prev,prev_step=curr,step
    return plan


def build():
    """Stabilize the package before pinching and peeling its positive-Y tail."""
    plan = np.tile(np.r_[READY,1.,READY,1.],(END+1,1))
    left = [
        (BASELINE, None, 1.), (154, (.38,.046,.838), 1.),
        (180, (.38,.031,.752), 1.), (200, (.38,.031,.752), 0.),
        (229, (.369,.052,.767), 0.), (261, (.345,.08,.800), 0.),
        (294, (.325,.1,.840), 0.), (320, (.32,.12,.82), 0.),
        (340, (.32,.14,.79), 1.), (370, None, 1.), (END, None, 1.)]
    right = [
        (BASELINE,None,1.), (120,(.38,-.015,.84),1.), (144,(.38,-.002,.758),1.),
        (162,(.38,-.002,.758),0.), (203,(.38,-.002,.758),0.),
        (232,(.385,-.012,.763),0.), (266,(.39,-.042,.78),0.),
        (292,(.393,-.085,.800),0.), (320,(.39,-.14,.84),0.),
        (356,(.38,-.22,.84),0.), (377,(.38,-.22,.768),0.),
        (390,(.38,-.22,.768),1.),(405,(.38,-.22,.84),1.),(END,None,1.)]
    for side,knots,sl in [('left',left,slice(0,7)),('right',right,slice(7,14))]:
        rotation = np.diag([1.,-1.,-1.])
        if side=='left':
            rotation = axis_angle_matrix(np.array([1.,0,0]),np.radians(-22)) @ axis_angle_matrix(
                np.array([0.,0,1.]),np.radians(-90)) @ rotation
        else:
            rotation = axis_angle_matrix(np.array([1.,0,0]),np.radians(35)) @ rotation
        prev=np.r_[READY,1.]
        prev_step=0
        for step,point,opening in knots:
            q=np.array(READY)
            if point is not None:
                q,ep,er=solve(point,rotation,prev[:6],station.BASES[side])
                if ep>.002 or er>.03:
                    raise RuntimeError(f'{side} waypoint {step}: {ep:g} m {er:g} rad')
            curr=np.r_[q,opening]
            t=np.linspace(0.,1.,step-prev_step+1)[:,None]
            plan[prev_step:step+1,sl]=prev+(curr-prev)*(t*t*(3.-2.*t))
            prev,prev_step=curr,step
    return plan

def turn_and_peel():
    """Hold a twisted tail while rotating and drawing the body across the seam."""
    end=480
    plan=np.tile(np.r_[READY,1.,READY,1.],(end+1,1))
    down=np.diag([1.,-1.,-1.])
    # Keep the end visible to the left gripper during its approach. The right
    # secures the body only once the left jaws have closed around the tail.
    left=[(100,None,1.,0),(124,(.38,.047,.829),1.,0),
          (151,(.38,.031,.755),1.,0),(174,(.38,.031,.755),0.,0),
          (194,(.380,.032,.782),0.,0),(220,(.380,.032,.782),0.,0),
          (250,(.380,.035,.783),0.,0),(277,(.376,.049,.789),0.,-22),
          (308,(.363,.064,.806),0.,-48),(335,(.347,.078,.815),0.,-60),
          (365,(.332,.090,.820),0.,-60),(388,(.328,.100,.820),1.,-50),
          (422,None,1.,0),(end,None,1.,0)]
    right=[(160,None,1.,0),(184,(.38,-.038,.852),1.,0),
           (207,(.38,-.007,.767),1.,0),(229,(.38,-.007,.764),0.,0),
           (249,(.38,-.007,.770),0.,0),(277,(.399,-.012,.783),0.,3.2),
           (308,(.424,-.023,.799),0.,5.6),(335,(.449,-.036,.807),0.,7.6),
           (365,(.463,-.06,.82),0.,8),(394,(.425,-.153,.839),0.,4),
           (421,(.38,-.22,.841),0.,0),(438,(.38,-.22,.769),0.,0),
           (449,(.38,-.22,.769),1.,0),(466,(.38,-.22,.844),1.,0),
           (end,None,1.,0)]
    for side,knots,sl in [('left',left,slice(0,7)),('right',right,slice(7,14))]:
        tilt,yaw=(-24,-90) if side=='left' else (42,0)
        rot=axis_angle_matrix(np.array([1.,0.,0.]),np.radians(tilt))@axis_angle_matrix(
            np.array([0.,0.,1.]),np.radians(yaw))@down
        first=0
        previous=np.r_[READY,1.]
        for step,point,opening,turn in knots:
            q=READY
            if point is not None:
                orientation=axis_angle_matrix(np.array([0.,1.,0.]),np.radians(turn))@rot
                q,ep,er=solve(point,orientation,previous[:6],station.BASES[side])
                if ep>.002 or er>.03:
                    raise ValueError(f'{side} turn-and-peel waypoint {step} failed: {ep:.4f} m {er:.4f} rad')
            current=np.r_[q,opening]
            t=np.linspace(0.,1.,step-first+1)[:,None]
            plan[first:step+1,sl]=previous+(current-previous)*t*t*(3.-2.*t)
            previous,first=current,step
    return plan


def axial_roll():
    """Pinch the two paper tails, lift, unwind them and let the sweet fall."""
    from yam_arena import ik
    from reach_audit import fk
    end=420
    plan=np.tile(np.r_[READY,1.,READY,1.],(end+1,1))
    down=np.diag([1.,-1.,-1.])
    for side,sign,sl in [('left',1,slice(0,7)),('right',-1,slice(7,14))]:
        base=station.BASES[side]
        yaw=-30 if side=='left' else 30
        start=np.r_[READY,1.]
        first=0
        knots=[(100,None,0,1.,0.), (129,(.34,sign*.052,.822),8,1.,0.),
               (155,(.34,sign*.051,.754),8,1.,0.),(174,(.34,sign*.051,.754),8,0.,0.),
               (202,(.34,sign*.051,.849),8,0.,0.),(238,(.34,sign*.051,.86),65,0.,0.),
               (270,(.34,sign*.053,.86),65,0.,-70.),
               (300,(.34,sign*.057,.86),65,0.,-140.),
               (326,(.34,sign*.077,.86),65,0.,-140.),
               (350,(.34,sign*.092,.852),65,0.,-140.),
               (368,(.34,sign*.092,.852),65,1.,-140.),(386,None,0,1.,0.),
               (end,None,0,1.,0.)]
        for step,pos,tilt,opening,roll in knots:
            q=np.array(READY)
            if pos is not None:
                orientation=axis_angle_matrix([1.,0.,0.],np.deg2rad(-sign*tilt))@axis_angle_matrix(
                    [0.,0.,1.],np.deg2rad(yaw))@down
                q,ep,er=solve(pos,orientation,READY,base)
                if ep>.002 or er>.03:
                    raise ValueError(f'{side} at {step}: {ep:g} m, {er:g} rad')
                # A parallel-jaw gripper is invariant to a half-turn about its
                # long axis. Choose the start roll with room for the unwind.
                alternatives=[q[-1]+np.pi*k for k in (-1,0,1)]
                q[-1]=max(val for val in alternatives if ik.LOWER[-1]<=val<=ik.UPPER[-1])
                q[-1]+=np.deg2rad(roll)
                if not ik.LOWER[-1]<=q[-1]<=ik.UPPER[-1]:raise ValueError((side,step,q[-1]))
            goal=np.r_[q,opening]
            t=np.linspace(0.,1.,step-first+1)[:,None]
            plan[first:step+1,sl]=start+(goal-start)*t*t*(3-2*t)
            first,start=step,goal
    return plan


def lift_unwind(extract=False, clear=False):
    """Secure the package body, grip a paper end in free space, and unwind it."""
    from yam_arena import ik
    end=500
    plan=np.tile(np.r_[READY,1.,READY,1.],(end+1,1))
    down=np.diag([1.,-1.,-1.])
    paths={
        'right':[(100,None,0,0,1),(125,(.34,-.005,.82),15,0,1),
                 (153,(.34,-.005,.756),15,0,1),(178,(.34,-.005,.756),15,0,0),
                 (208,(.34,-.005,.853),15,0,0),(278,(.34,-.005,.853),15,0,0),
                 (340,(.34,-.005,.853),15,0,0),(368,(.341,-.018,.866),20,0,0),
                 (390,(.341,-.018,.866),20,0,1),(420,None,0,0,1),(end,None,0,0,1)],
        'left':[(202,None,0,0,1),(230,(.34,.082,.860),65,0,1),
                (254,(.34,.026,.812),65,0,1),(278,(.34,.026,.812),65,0,0),
                (307,(.34,.029,.816),65,-75,0),(338,(.34,.037,.828),65,-165,0),
                (369,(.34,.063,.847),65,-165,0),(398,(.34,.041,.823),65,-165,0),
                (423,(.34,.035,.800),65,-165,0),(443,(.32,.06,.789),55,-145,0),
                (461,(.32,.06,.789),55,-145,1),(478,None,0,0,1),(end,None,0,0,1)]}
    if clear:
        end=600
        plan=np.tile(np.r_[READY,1.,READY,1.],(end+1,1))
        # After releasing the body, carry the opened wrapper clear of the
        # landing point. Give the round sweet time to settle on the table.
        paths['right']=paths['right'][:-1]+[(end,None,0,0,1)]
        paths['left']=paths['left'][:7]+[
            (380,(.351,.061,.849),65,-165,0),(399,(.375,.063,.862),65,-165,0),
            (425,(.405,.068,.887),65,-150,0),(455,(.425,.125,.885),60,-125,0),
            (480,(.431,.190,.857),50,-95,0),(503,(.423,.216,.823),40,-65,0),
            (526,(.423,.216,.823),40,-65,1),(555,None,0,0,1),(end,None,0,0,1)]
    if extract:
        end=630
        plan=np.tile(np.r_[READY,1.,READY,1.],(end+1,1))
        # Open the body grip progressively while the other hand draws the
        # unwound paper across the seam. Deposit and release on the same table.
        paths['right']=paths['right'][:8]+[
            (403,(.338,-.004,.865),35,0,.08),(435,(.332,.010,.838),35,0,.13),
            (467,(.323,.036,.810),35,0,.14),(500,(.308,.079,.785),25,0,.14),
            (528,(.285,.095,.762),0,0,.16),(552,(.285,.095,.762),0,0,1),
            (590,None,0,0,1),(end,None,0,0,1)]
        paths['left']=paths['left'][:8]+[
            (403,(.353,.055,.808),65,-160,0),(435,(.374,.052,.795),60,-150,0),
            (467,(.400,.030,.796),60,-145,0),(500,(.424,.016,.801),55,-135,0),
            (528,(.438,.012,.795),55,-135,0),(545,(.438,.012,.795),55,-135,1),
            (586,None,0,0,1),(end,None,0,0,1)]
    for side,sgn,sl in [('right',-1,slice(7,14)),('left',1,slice(0,7))]:
        start=np.r_[READY,1.];first=0
        for step,point,tilt,roll,opening in paths[side]:
            q=np.array(READY)
            if point is not None:
                yaw=-30 if side=='left' else 0.
                rot=axis_angle_matrix([1.,0.,0.],np.deg2rad(-sgn*tilt))@axis_angle_matrix([0.,0.,1.],np.deg2rad(yaw))@down
                q,ep,er=solve(point,rot,READY,station.BASES[side])
                if ep>.002 or er>.03:raise ValueError((side,step,ep,er))
                if side=='left':
                    q[-1]=max(v for v in [q[-1]-np.pi,q[-1],q[-1]+np.pi] if ik.LOWER[-1]<=v<=ik.UPPER[-1])
                    q[-1]+=np.deg2rad(roll)
                if clear and side=='left' and step>369:
                    q[-1]=start[5]
                if not ik.LOWER[-1]<=q[-1]<=ik.UPPER[-1]:raise ValueError((side,step,q[-1]))
            goal=np.r_[q,opening]
            t=np.linspace(0.,1.,step-first+1)[:,None]
            plan[first:step+1,sl]=start+(goal-start)*t*t*(3-2*t)
            first,start=step,goal
    return plan
