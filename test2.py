import os
import robosuite as suite
from robosuite.models.objects import MujocoXMLObject
from robosuite.utils.mjcf_utils import xml_path_completion, array_to_string, find_elements
from robosuite.models import MujocoWorldBase
from mujoco_py import load_model_from_xml, MjSim, MjViewer

MODEL_XML = """
<?xml version="1.0" ?>
<mujoco model="hole">
  <worldbody>
    <body>
      <body name="object">
        <geom pos="0 0 0" size="0.06 0.17 0.01" type="box" group="0" material="plate_mat" />
        <geom pos="0.22 0 0" size="0.06 0.17 0.01" type="box" group="0" material="plate_mat" />
        <geom pos="0.11 0.11 0" size="0.05 0.06 0.01" type="box" group="0" material="plate_mat" />
        <geom pos="0.11 -0.11 0" size="0.05 0.06 0.01" type="box" group="0" material="plate_mat" />
      </body>
      <site rgba="1 0 0 1" size="0.005" pos="0 0 -0.02" name="bottom_site"/>
      <site rgba="1 0 0 1" size="0.005" pos="0 0 0.02" name="top_site"/>
      <site rgba="1 0 0 1" size="0.005" pos="0.25 0.25 0.1" name="horizontal_radius_site"/>
    </body>
    <body name="floor" pos="0 0 0.025">
        <geom size="1.0 1.0 0.02" rgba="0 1 0 1" type="box"/>
    </body>
  </worldbody>
</mujoco>
"""

CURRENT_DIR = os.path.dirname(os.path.realpath(__file__))
XML_PATH = os.path.join(CURRENT_DIR, "models/objects/plate-with-hole.xml")

def print_box_xpos(sim):
    print("object xpos:", sim.data.get_body_xpos("object"))


class PlateWithHoleObject(MujocoXMLObject):
    """
    Square plate with a hole in the center (used in PegInHole)
    """

    def __init__(self, name):
        super().__init__(os.path.join(CURRENT_DIR, "models/objects/plate-with-hole.xml"), # xml_path_completion("objects/plate-with-hole.xml"),
                         name=name, joints=None, obj_type="all", duplicate_collision_geoms=True)

world = MujocoWorldBase()

plate_with_hole = PlateWithHoleObject(name="PlateWithHole")
plate_with_hole
world.merge(plate_with_hole)


# model = load_model_from_xml(MODEL_XML)
model = world.get_model(mode="mujoco_py")
sim = MjSim(model)
viewer = MjViewer(sim)
viewer.vopt.geomgroup[0] = 0  # disable visualization of collision mesh

states = [{'box:x': +0.8, 'box:y': +0.8},
          {'box:x': -0.8, 'box:y': +0.8},
          {'box:x': -0.8, 'box:y': -0.8},
          {'box:x': +0.8, 'box:y': -0.8},
          {'box:x': +0.0, 'box:y': +0.0}]

# MjModel.joint_name2id returns the index of a joint in
# MjData.qpos.
x_joint_i = sim.model.get_joint_qpos_addr("box:x")
y_joint_i = sim.model.get_joint_qpos_addr("box:y")

print_box_xpos(sim)

while True:
    for state in states:
        sim_state = sim.get_state()
        # sim_state.qpos[x_joint_i] = state["box:x"]
        # sim_state.qpos[y_joint_i] = state["box:y"]
        # sim.set_state(sim_state)
        # sim.forward()
        # print("updated state to", state)
        # print_box_xpos(sim)
        viewer.render()

    if os.getenv('TESTING') is not None:
        break