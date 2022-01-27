import numpy as np
import robosuite as suite
from robosuite.models import MujocoWorldBase
from robosuite.models.robots import Panda
from robosuite.models.grippers import gripper_factory
from robosuite.models.arenas import TableArena
from robosuite.models.objects import BallObject
from robosuite.utils.mjcf_utils import new_joint
import mujoco_py
import os
from robosuite.models.objects import PlateWithHoleObject
from robosuite.utils.placement_samplers import SequentialCompositeSampler, UniformRandomSampler

def print_box_xpos(sim):
    print("box xpos:", sim.data.get_body_xpos("box"))

# Creating the world
world = MujocoWorldBase()

# # Creating the robot
# mujoco_robot = Panda()
# # add a gripper to the robot
# gripper = gripper_factory('PandaGripper')
# mujoco_robot.add_gripper(gripper)
# # add the robot to the world
# mujoco_robot.set_base_xpos([0, 0, 0])
# world.merge(mujoco_robot)
#
# # Creating the table
# mujoco_arena = TableArena()
# mujoco_arena.set_origin([0.8, 0, 0])
# # add the table to the world
# world.merge(mujoco_arena)

# # Adding the object
# sphere = BallObject(
#     name="sphere",
#     size=[0.04],
#     rgba=[0, 0.5, 0.5, 1]).get_obj()
# sphere.set('pos', '1.0 0 1.0')
# world.worldbody.append(sphere)
#
# # Adding another object
# plate_with_hole = PlateWithHoleObject(name="PlateWithHole")
# plate_with_hole.set_sites_visibility()
# world.merge(plate_with_hole)
#
#
# # Running the simulation
# model = world.get_model(mode="mujoco_py")


mj_path = mujoco_py.utils.discover_mujoco()
xml_path = os.path.join(mj_path, 'model', 'rope.xml')
model = mujoco_py.load_model_from_path(xml_path)
sim = mujoco_py.MjSim(model)
viewer = mujoco_py.MjViewer(sim)
viewer.vopt.geomgroup[0] = 0  # disable visualization of collision mesh

for i in range(10000):
  # sim.data.ctrl[:] = 0
  sim.step()
  viewer.render()