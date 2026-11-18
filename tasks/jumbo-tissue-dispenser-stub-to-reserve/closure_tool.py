"""Loose physical closing tool and read-only measured clearance geometry."""
import numpy as np
import dispenser_device as device
from surface_access_geometry import CollisionBox
from surface_paper import _numpy

PARTS = [
    ('foot', (.060, .040, .030), (0., -.012, 0.)),
    ('neck', (.008, .048, .008), (0., .032, 0.)),
    ('bridge', (.028, .008, .008), (.010, .060, .008)),
    ('riser', (.008, .012, .015), (.020, .060, .014)),
    ('handle', (.018, .020, .050), (.020, .060, .040)),
    ('base', (.050, .080, .005), (0., .030, -.0175)),
]
TOOL_MASS = sum(np.prod(size) * 2700 for _, size, _ in PARTS)
TCP_OFFSET = np.array([.020, .060, .065 + device.TIP - .045])
RACK = np.array([.160, .500, device.TOP + .020])
toolkit = device.mk.Kit()
toolkit.part('cover_tool', RACK,
    [device.mk.box(size, at=offset) for _, size, offset in PARTS],
    mass=TOOL_MASS, color=(.62, .64, .66), contact_offset=.001)
OBJECTS = [*device.OBJECTS, *toolkit.objects()]


EXPECTED_TOOL_NAMES = frozenset(f'cover_tool/{i}' for i in range(len(PARTS)))


def measured_boxes(base):
    body = base.scene.rigid_objects['cover_tool']
    position = _numpy(body.data.root_pos_w)[0]
    quaternion = _numpy(body.data.root_quat_w)[0]
    if (position.shape != (3,) or quaternion.shape != (4,)
            or not np.isfinite(position).all() or not np.isfinite(quaternion).all()):
        raise ValueError('Incomplete measured closing-tool pose')
    rotation = device.mk.quat_to_mat(quaternion)
    return tuple(CollisionBox(f'cover_tool/{i}', rotation @ offset + position,
                              np.asarray(size)/2, quaternion.copy())
                 for i, (_, size, offset) in enumerate(PARTS))


device.paper.tool_boxes_reader = measured_boxes
device.paper.expected_tool_names = EXPECTED_TOOL_NAMES
