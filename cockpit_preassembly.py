import numpy as np
import robosuite
from robosuite import load_controller_config
from robosuite.environments.manipulation.pick_place import PickPlace
from envs.two_arm_wire_harness import TwoArmFitting

# controller config
# TODO: create a custom controller json
controller_name = 'JOINT_POSITION'
controller_config = load_controller_config(custom_fpath=None, default_controller=controller_name)

# environment
'''Method 1: from robosuite.make'''
# env = suite.make("PickPlace", ...)
'''Method 2: instantiate the class'''
# env = PickPlace(
#     robots=["Panda"],                       # load a Sawyer robot and a Panda robot
#     gripper_types="default",                # use default grippers per robot arm
#     controller_configs=controller_config,   # each arm is controlled using OSC
#     env_configuration="default",            # (two-arm envs only) arms parallel to each other
#     has_renderer=True,                      # on-screen rendering
#     render_camera="frontview",              # visualize the "frontview" camera
#     has_offscreen_renderer=False,           # no off-screen rendering
#     control_freq=20,                        # 20 hz control for applied actions
#     horizon=200,                            # each episode terminates after 200 steps
#     use_object_obs=False,                   # no observations needed
#     use_camera_obs=False,                   # provide image observations to agent
#     reward_shaping=False,                   # use a dense reward signal for learning
#     single_object_mode=2,
#     object_type='milk'                      # TODO: how to add/specify an object
# )

env = TwoArmFitting(
    robots=["Panda", "Panda"],              # load two Panda robots
    gripper_types="default",                # use default grippers per robot arm
    controller_configs=controller_config,   # each arm is controlled using OSC
    env_configuration="single-arm-parallel", # (two-arm envs only) arms parallel to each other
    has_renderer=True,                      # on-screen rendering
    render_camera="frontview",              # visualize the "frontview" camera
    has_offscreen_renderer=False,            # no off-screen rendering
    control_freq=20,                        # 20 hz control for applied actions
    horizon=200,                            # each episode terminates after 200 steps
    use_object_obs=False,                   # no observations needed
    use_camera_obs=False,                    # provide image observations to agent
    camera_names="agentview",               # use "agentview" camera for observations
    camera_heights=84,                      # image height
    camera_widths=84,                       # image width
    reward_shaping=False,                   # use a dense reward signal for learning
)
env.reset()
env.viewer.set_camera(camera_id=0)

# Get action limits
low, high = env.action_spec

# do visualization
for i in range(10000):
    action = np.random.uniform(low, high)
    obs, reward, done, _ = env.step(action)
    env.render()
