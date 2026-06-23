from __future__ import annotations


from dataclasses import dataclass
import torch
from collections.abc import Sequence
from typing import TYPE_CHECKING

import omni.log

from isaaclab.utils import configclass
import isaaclab.utils.string as string_utils
from isaaclab.assets.articulation import Articulation
from isaaclab.managers.action_manager import ActionTerm, ActionTermCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv

class RotorForceAction(ActionTerm):
    cfg: RotorForceActionCfg
    _asset: Articulation
    _scale: torch.Tensor | float
    _offset: torch.Tensor | float
    _clip: torch.Tensor
    _kf: float
    _km: float
    _motor_time_constant: float = 0.05


    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self._rotor_ids, self._rotor_names = self._asset.find_bodies(self.cfg.rotor_names)
        self._num_rotors = len(self._rotor_ids)
        
        # actuator desired setpoints storage params
        self.actuator_setpoint = torch.zeros(self.num_envs, 4, device=self.device)

        # rotor_ids[0,3] --> (CCW) and rotor_ids[1,2] --> (CW)
        self._rotor_dirs = torch.tensor([1.0, -1.0, -1.0, 1.0], device=self.device) # hardcoding these axes dirs of rotors for now
        # log the resolved joint names for debugging
        omni.log.info(
            f"Resolved rotor names for the action term {self.__class__.__name__}:"
            f" {self._rotor_names} [{self._rotor_ids}]"
        )

        # create tensors for raw and processed actions # (num_envs,1,num_rotors)
        self._raw_actions = torch.zeros(self.num_envs, self.action_dim, device=self.device)
        self._processed_actions = torch.zeros_like(self.raw_actions)

        # parse scale
        if isinstance(self.cfg.scale, (float, int)):
            self._scale = float(self.cfg.scale)
        elif isinstance(self.cfg.scale, dict):
            self._scale = torch.ones(self.num_envs, self.action_dim, device=self.device)
            # resolve the dictionary config
            index_list, _, value_list = string_utils.resolve_matching_names_values(self.cfg.scale, self._rotor_names)
            self._scale[:, index_list] = torch.tensor(value_list, device=self.device)
        else:
            raise ValueError(f"Unsupported scale type: {type(self.cfg.scale)}. Supported types are float and dict.")

        # parse offset
        if isinstance(self.cfg.offset, (float, int)):
            self._offset = float(self.cfg.offset)
        elif isinstance(self.cfg, dict):
            self._offset = torch.zeros_like(self._raw_actions)
            # resolve the dictionary config
            index_list, _, value_list = string_utils.resolve_matching_names_values(self.cfg.offset, self._rotor_names)
            self._offset[:, index_list] = torch.tensor(value_list, device=self.device)
        else:
            raise ValueError(
                f"Unsupported type for offset param in the configuration settings of this action term. Supported types are float and dict"
            )
    
        # parse clip
        if self.cfg.clip is not None:
            if isinstance(cfg.clip, dict):
                self._clip = torch.tensor([[-float("inf"), float("inf")]], device=self.device).repeat(
                    self.num_envs, self.action_dim, 1
                )
                index_list, _, value_list = string_utils.resolve_matching_names_values(self.cfg.clip, self._rotor_names)
                self._clip[:, index_list] = torch.tensor(value_list, device=self.device)
            else:
                raise ValueError(f"Unsupported clip type: {type(cfg.clip)}. Supported types are dict.")
        

        # parsing motor constants
        self._omega_max = float(self.cfg.omega_max)
        self._kf = float(self.cfg.thrust_coefficient)
        self._km = float(self.cfg.moment_coefficient)
        # sanity checks
        if self._omega_max <= 0.0:
            raise ValueError(
                f"omega_max must be > 0. Received {self._omega_max}"
            )
        if self._kf <= 0.0:
            raise ValueError(
                f"thrust_coefficient must be > 0. Received {self._kf}"
            )
        if self._km < 0.0:
            raise ValueError(
                f"moment_coefficient must be >= 0. Received {self._km}"
            )
        omni.log.info(
            f"[{self.__class__.__name__}] "
            f"omega_max={self._omega_max:.3f} rad/s, "
            f"kf={self._kf:.3e}, "
            f"km={self._km:.3e}"
        )


    """
    Properties
    """
    @property
    def action_dim(self) -> int:
        return self._num_rotors

    @property
    def raw_actions(self) -> torch.Tensor:
        return self._raw_actions

    @property
    def processed_actions(self) -> torch.Tensor:
        return self._processed_actions

    """
    Operations
    """
    def process_actions(self, actions):
        self.raw_actions[:] = actions
        self._processed_actions = self.raw_actions*self._scale + self._offset # raw processing here
        if self.cfg.clip is not None:
            self._processed_actions = torch.clamp(
                self._processed_actions, min=self._clip[:, :, 0], max=self._clip[:, :, 1]
            )
    
    def apply_actions(self):
        ''' logic to apply force and torque actions to the rotor body links to simulate thrust due to rotating rotors'''
        forces = torch.zeros((self.num_envs, self._num_rotors, 3), device=self.device)
        torques = torch.zeros_like(forces)
        rotor_thrust, rotor_torque = self._compute_dynamics(self.processed_actions)
        forces[:, :, 2] = rotor_thrust
        torques[:, :, 2] = rotor_torque
        # print(f"rotor thrust: {forces}")
        # print(f"rotor_torque: {torques}\n")
        # apply force and torque to the body links
        self._asset.set_external_force_and_torque(forces=forces, torques=torques, body_ids=self._rotor_ids) # applied in local frame of ref (body frame)

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        self._raw_actions[env_ids] = 0.0

    """
    Helpers
    """
    def _compute_dynamics(self, actuator_cmds):
        '''
        actuator_cmds --> motor cmds (processed actions --> after scaling and offsetting) 
        --> not this is not drone based motor dynamics just for each motor thrust cal and implementation
        --> using the force and torque model of motors for this (simple model for now)
        --> this model needs rework, currently the drone takes off as if no other forces affect it like a normal block given an upward force
        '''
        kf =  self._kf # force constant
        km = self._km # motor constant
        omega_max = self._omega_max
        # first-order motor lag
        tau = torch.clamp(torch.tensor(self._env.physics_dt / self._motor_time_constant, device=self.device), 0.0, 1.0) # time constant
        cmds = torch.clamp(actuator_cmds, -1.0, 1.0)
        print(f"actuator commands: {cmds}\n")
        self.actuator_setpoint = self.actuator_setpoint + tau*(torch.sqrt(cmds)-self.actuator_setpoint)
        throttle = self.actuator_setpoint
        rotor_omega = self._rotor_dirs.view(1,-1)*throttle*omega_max
        thrust = kf*(throttle**2)*(rotor_omega**2)
        torque = -self._rotor_dirs.view(1,-1)*km*(throttle**2)*(rotor_omega**2)
        return thrust, torque
    

@configclass
class RotorForceActionCfg(ActionTermCfg):
    """
    Configuration for the rotor link force action term based on quadcoptor base dynamics.
    """
    class_type: type[ActionTerm] = RotorForceAction

    rotor_names: list[str] = ["rotor0", "rotor1", "rotor2", "rotor3"]
    """List of joint names or regex expressions that the action will be mapped to."""
    scale: float | dict[str, float] = 1.0
    """Scale factor for the action (float or dict of regex expressions). Defaults to 1.0."""
    offset: float | dict[str, float] = 0.0
    """Offset factor for the action (float or dict of regex expressions). Defaults to 0.0."""

    # need testing params
    """ motor max angular velocity in rad/s"""
    omega_max: float = 1200 # rads/s
    """ kf in T = kf * omega² [N/(rad/s)^2]. Tunable """
    thrust_coefficient: float = 7.0e-6 # N/(rad/s)^2 
    """ km in Q = km * omega² [N*m/(rad/s)^2]. Tunable """
    moment_coefficient: float = 1.0e-7 # N*m/(rad/s)^2
    


