# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Script to run an environment with zero action agent."""

"""Launch Isaac Sim Simulator first."""

import argparse

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Zero agent for Isaac Lab environments.")
parser.add_argument(
    "--disable_fabric", action="store_true", default=False, help="Disable fabric and use USD I/O operations."
)
parser.add_argument("--num_envs", type=int, default=3, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
parser.add_argument("--teleop_device", type=str, default="keyboard", help="Device for interacting with environment")
parser.add_argument("--sensitivity", type=float, default=1.0, help="Sensitivity factor.")

# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli) 
simulation_app = app_launcher.app


"""Rest everything follows."""
import gymnasium as gym
import torch
import numpy as np

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils import parse_env_cfg

# PLACEHOLDER: Extension template (do not remove this comment)
from isaaclab.devices import Se3Gamepad, Se3Keyboard, Se3SpaceMouse, Se2Keyboard


def pre_process_actions(
    teleop_data: tuple[np.ndarray, bool] | list[tuple[np.ndarray, np.ndarray, np.ndarray]], num_envs: int, device: str
) -> torch.Tensor:
    
    delta_pose, gripper_command = teleop_data # gripper_command is also gripper status (suction and finger based)
    delta_pose = torch.tensor(delta_pose, dtype=torch.float, device=device).repeat((num_envs, 1))
    gripper_vel = torch.zeros((num_envs, 1), dtype=torch.float, device=device)
    gripper_vel[:, 0] = -1 if gripper_command else 1
    action = torch.concat([delta_pose, gripper_vel], dim=1)

    return action # space size would be (-7-)

# just for testing purposes
def pre_process_motor_actions(
    teleop_data: tuple[np.ndarray, bool] | list[tuple[np.ndarray, np.ndarray, np.ndarray]], num_envs: int, device: str
) -> torch.Tensor:
    teleop_cmds, _ = teleop_data
    motor_input = torch.tensor(teleop_cmds[:4], dtype=torch.float, device=device).repeat((num_envs, 1))
    action = motor_input
    return action 


def main():
    """Teleoperation actions agent with Isaac Lab environment."""
    # parse configuration
    env_cfg = parse_env_cfg(
        args_cli.task, device=args_cli.device, num_envs=args_cli.num_envs, use_fabric=not args_cli.disable_fabric
    )
    env_cfg.terminations.time_out = None #type:ignore (to prevent resetting after episode length expiry)
    # create environment
    env = gym.make(args_cli.task, cfg=env_cfg)

    # Flags for controlling teleoperation flow
    teleoperation_active = True

    def start_teleoperation():
        """Activate teleoperation control of the robot.

        This callback enables active control of the robot through the input device.
        It's typically triggered by a specific gesture or button press and is used when:
        - Beginning a new teleoperation session
        - Resuming control after temporarily pausing
        - Switching from observation mode to control mode

        While active, all commands from the device will be applied to the robot.
        """
        nonlocal teleoperation_active
        teleoperation_active = True

    def stop_teleoperation():
        """Deactivate teleoperation control of the robot.

        This callback temporarily suspends control of the robot through the input device.
        It's typically triggered by a specific gesture or button press and is used when:
        - Taking a break from controlling the robot
        - Repositioning the input device without moving the robot
        - Pausing to observe the scene without interference

        While inactive, the simulation continues to render but device commands are ignored.
        """
        nonlocal teleoperation_active
        teleoperation_active = False

    # create controller
    if args_cli.teleop_device.lower() == "keyboard":
        teleop_interface = Se3Keyboard(
            pos_sensitivity=0.05 * args_cli.sensitivity, rot_sensitivity=0.05 * args_cli.sensitivity
        )
    elif args_cli.teleop_device.lower() == "2d_keyboard":
        teleop_interface = Se2Keyboard(
            v_x_sensitivity=0.05 * args_cli.sensitivity, v_y_sensitivity=0.05 * args_cli.sensitivity, omega_z_sensitivity=0.05 * args_cli.sensitivity
        )
    elif args_cli.teleop_device.lower() == "spacemouse":
        teleop_interface = Se3SpaceMouse(
            pos_sensitivity=0.05 * args_cli.sensitivity, rot_sensitivity=0.05 * args_cli.sensitivity
        )
    elif args_cli.teleop_device.lower() == "gamepad":
        teleop_interface = Se3Gamepad(
            pos_sensitivity=0.1 * args_cli.sensitivity, rot_sensitivity=0.1 * args_cli.sensitivity
        )
    else:
        raise ValueError(
            f"Invalid device interface '{args_cli.teleop_device}'. Supported: 'keyboard', 'spacemouse', 'gamepad',"
        )

    # print info (this is vectorized environment)
    print(f"[INFO]: Gym observation space: {env.observation_space}")
    print(f"[INFO]: Gym action space: {env.action_space}")
    # reset environment
    init_obs, _ = env.reset()
    teleop_interface.reset()
    print(f"[INFO] Teleop Interface:\n{teleop_interface}")

    # simulate environment
    while simulation_app.is_running():
        # run everything in inference mode
        with torch.inference_mode():
            # compute zero actions
            teleop_data = teleop_interface.advance()
            # actions = pre_process_actions(teleop_data=teleop_data, num_envs=env.unwrapped.num_envs, device=env.unwrapped.device) #type:ignore
            actions = pre_process_motor_actions(
                teleop_data=teleop_data, # type: ignore
                num_envs=env.unwrapped.num_envs,  # type: ignore
                device=env.unwrapped.device # type: ignore
            )
            actions = actions*1e10
            print(f"Taken Actions: {actions}")
            print(f"{actions.shape}\n")
            # apply actions
            obs, reward, terminated, trunc, info  = env.step(actions)
            dones = terminated & trunc
            # print(f"observations: {obs}, rewards: {reward}")

    # close the simulator
    env.close()


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
