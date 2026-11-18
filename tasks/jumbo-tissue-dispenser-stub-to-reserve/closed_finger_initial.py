"""Declared READY initial configuration and settling targets with closed fingers.

Configuration wrappers assign ArticulationCfg defaults before construction.
The settling wrapper updates only the ordinary ready action tensor used by the
adapter after reset; no simulator body/joint pose or velocity is written.
"""
import mechkit

def patch_robot_configuration(module):
 original=module.yam_arm_cfg
 if getattr(original,'_singlearm90_closed_defaults',False):return
 def make(*args,**kwargs):
  cfg=original(*args,**kwargs)
  cfg.init_state.joint_pos['joint[7-8]']=0.
  return cfg
 make._singlearm90_closed_defaults=True
 module.yam_arm_cfg=make

def patch_settling_configuration(module):
 cls=module.YamArenaEmbodiment
 old=cls._ensure_sim
 if getattr(old,'_singlearm90_closed_settling',False):return
 def ensure(self,*args,**kwargs):
  result=old(self,*args,**kwargs)
  # This is the reset-settling action target, not a simulator state tensor.
  self._ready=self._ready.clone()
  self._ready[:,6]=0.
  self._ready[:,13]=0.
  return result
 ensure._singlearm90_closed_settling=True
 cls._ensure_sim=ensure

# embodiment imports yam_arm_cfg into its own global namespace. Patch that real
# bound alias too, even if it was imported before this task module.
mechkit._when_imported('yam_arena.robot',patch_robot_configuration)
mechkit._when_imported('yam_arena.embodiment',patch_robot_configuration)
mechkit._when_imported('yam_arena.inspect_embodiment',patch_settling_configuration)
