import os
import numpy as np
import mujoco_py
from mujoco_py import load_model_from_xml, MjSim, MjViewer
import robosuite as suite
from robosuite.models.arenas import TableArena, PegsArena


mj_path = mujoco_py.utils.discover_mujoco()
# XML_PATH = os.path.join('/home/kejia/Documents/assembly-mujoco/models/robots', '/', 'two_franka_panda.xml')
XML_PATH = '/home/kejia/Documents/assembly-mujoco/models/franka_sim/two_franka_panda.xml'
XML_PATH_ROPE = '/home/kejia/Documents/assembly-mujoco/models/franka_sim/assets/rope_expanded.xml'
# CURRENT_DIR = os.path.dirname(os.path.realpath(__file__))
# XML_PATH = os.path.join(CURRENT_DIR, "models/objects/rope.xml")

def print_box_xpos(sim):
    print("box xpos:", sim.data.get_body_xpos("box"))

# Creating the table
mujoco_arena = PegsArena(
            table_full_size=(0.8, 0.8, 0.05),
            table_friction=(1, 0.005, 0.0001),
            table_offset=np.array((0, 0, 0.82)),
        )
mujoco_arena.set_origin([0, 0, 0])

# Creating the rope
# model = load_model_from_xml(MODEL_XML)
model = mujoco_py.load_model_from_path(XML_PATH)
sim = MjSim(model)
viewer = MjViewer(sim)

# states = [{'box:x': +0.8, 'box:y': +0.8},
#           {'box:x': -0.8, 'box:y': +0.8},
#           {'box:x': -0.8, 'box:y': -0.8},
#           {'box:x': +0.8, 'box:y': -0.8},
#           {'box:x': +0.0, 'box:y': +0.0}]
#
# # MjModel.joint_name2id returns the index of a joint in
# # MjData.qpos.
# x_joint_i = sim.model.get_joint_qpos_addr("box:x")
# y_joint_i = sim.model.get_joint_qpos_addr("box:y")
#
# print_box_xpos(sim)

while True:
    viewer.render()
    sim.step()
    # for state in states:
    #     sim_state = sim.get_state()
        # sim_state.qpos[x_joint_i] = state["box:x"]
        # sim_state.qpos[y_joint_i] = state["box:y"]
        # sim.set_state(sim_state)
        # sim.forward()
        # print("updated state to", state)
        # print_box_xpos(sim)
        # viewer.render()

    # if os.getenv('TESTING') is not None:
    #     break