# Camera plate and push-button receiver

The camera housing is screw-fastened to a keyed plate; the wrist bracket is screw-fastened to a mating receiver. The plate enters through a tapered mouth and slides along guide shoulders to an end stop. Contact with the leading ramp retracts the spring tooth. At the seated position, the spring returns the tooth into the plate notch.

A compact button on the left of the receiver connects to the tooth on the right through one rigid return yoke passing behind the backplate. Pressing the button in camera-frame +X retracts the tooth in the same direction. A 5 mm prismatic joint provides travel, with spring stiffness150 N/m and damping0.15 N s/m. Button, yoke, tooth, guide and plate have collision geometry. The camera is retained by contact with these parts.

The camera remains connected to the left arm by a nominal0.74 m USB cable with a side plug and strain-relief boot. The cable uses the collidable capsule chain and elastic joints from the camera-cable recovery task. The original optical mounting position, orientation and insertion axis are unchanged.

Reference: [Falcam F22](https://falcamstore.com/pages/f22-introduction), for the push-to-attach/press-to-detach principle. This task specifies its own adapter geometry in `mechanism_dimensions.json`, `quick_release.py` and `wrist_adapter.py`.

## Assembly before the trial

Fasten the camera plate through its two flush rear screw locations into the housing; the model locates these at camera-frame (X,Z)=(-1,-3) and(-1,-9) mm, with screw axes parallel to Y. The receiver supports are screw-fastened at the two wrist-bracket feet; their positions are specified in `quick_release.py` and `wrist_adapter.py`. These permanent assemblies remain intact during the task.

The tooth, return yoke and side button form one slider. Assemble it in a transverse guide with5 mm travel along camera-frame X. The guide constrains rotation and motion in Y/Z. A return spring opposes positive-X retraction, with stiffness0.15 N/mm and nominal0.2 mm preload, giving approximately0.78 N return force at full travel. The rear cassette encloses the yoke and exposes the side button. Fit the keyed plate with0.2 mm nominal guide clearance on each side. Full part dimensions, slot positions, spring travel and drive parameters are supplied in `mechanism_dimensions.json` and the procedural geometry source.
