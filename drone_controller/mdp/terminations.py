from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

from isaaclab.utils.math import euler_xyz_from_quat, wrap_to_pi

def terminate_on_unsafe_orientation(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")    
)-> torch.Tensor:
    robot: Articulation = env.scene[asset_cfg.name]
    robot_quat_env = robot.data.root_quat_w # world/inertial frame is basically env origin frame
    roll, pitch, _ = euler_xyz_from_quat(robot_quat_env) # converting to Euler angles from quat for now for easy logic implementation (but not very efficient to do things this way)
    roll, pitch = wrap_to_pi(roll), wrap_to_pi(pitch) # wrap angle ranges between [-pi,pi]
    return ((torch.abs(roll) >= torch.pi/2) | (torch.abs(pitch) >= torch.pi/2))

def terminate_on_altitude_bounds(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    target_height: float = 10.0, # in [m]
    bound_tolerance: float = 2.0 # in [m]
)-> torch.Tensor:
    
    robot: Articulation = env.scene[asset_cfg.name]
    robot_pos_z_w = robot.data.root_pos_w[:, 2]
    bound_height = target_height + bound_tolerance
    return (robot_pos_z_w-bound_height > 0)