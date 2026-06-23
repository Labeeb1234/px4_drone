from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import Articulation, RigidObject
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers.manager_base import ManagerTermBase
from isaaclab.managers.manager_term_cfg import RewardTermCfg
from isaaclab.sensors import ContactSensor, RayCaster

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

import isaaclab.utils.math as math_utils

# hold pose drone


def pos_prog_exp_l2(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    sigma: float = 2.0
)->torch.Tensor:
    
    pos_target = torch.tensor([0.0, 0.0], device=env.device).repeat((env.num_envs,1))
    robot: Articulation = env.scene[asset_cfg.name]
    robot_xy_pos = robot.data.root_pos_w[:, :2]
    # pos (x-y) error based on x-y vec norm
    pos_error = torch.norm(
        pos_target - robot_xy_pos,
        dim=1
    )
    return torch.exp(-pos_error**2/sigma**2)

def altitude_exp_l2(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    target_height: float = 10.0, # in [m],
    sigma: float = 8.0 # std dev of the error model   
) -> torch.Tensor:

    robot: Articulation = env.scene[asset_cfg.name] # drone articulation asset object
    # get bot height wrt ground
    robot_z_pos = robot.data.root_pos_w[:, 2] # wrt world frame/env frame (inertial frame of reference)
    height_error = target_height - robot_z_pos  # target height is measured wrt to the same coordinates as robot_z_pos
    # using exp l2 kernel
    return torch.exp(-height_error**2/sigma**2)


# level the drone (objective) !!
# target roll, pitch and yaw ===> 0.0
# smoother func using angular error (exp l2 kernel)
def level_drone_exp_l2(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    sigma: float = 0.5 # std_dev
) -> torch.Tensor:
    robot: Articulation = env.scene[asset_cfg.name]
    # target orientation in quaternion
    q_target = torch.tensor([1.0, 0.0, 0.0, 0.0], device=env.scene.device).repeat((env.scene.num_envs,1)) # [w,x,y,z], 1 --> repeat to num_envs
    robot_orientation_q = robot.data.root_quat_w # [w,x,y,z] , num_envs
    q_dot = torch.abs(torch.sum(q_target*robot_orientation_q, dim=1))
    q_dot = torch.clamp(q_dot, 0.0, 1.0)
    # using angle error based quantity
    angle_error = 2.0*torch.acos(q_dot)
    # exp l2 kernel
    return torch.exp(-(angle_error**2)/(sigma**2))

