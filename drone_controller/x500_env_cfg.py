import isaaclab.sim as sim_utils
from isaaclab.utils import configclass

import isaaclab.envs.mdp as mdp
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers.action_manager import ActionTermCfg as ActionTerm
from isaaclab.managers.curriculum_manager import CurriculumTermCfg as CurrTerm

from isaaclab.managers import SceneEntityCfg
from isaaclab.assets import AssetBaseCfg, ArticulationCfg

from isaaclab_assets.robots.x500 import X500_DRONE_CFG
import isaaclab_tasks.manager_based.locomotion.drone_controller.mdp as cmdp


@configclass
class DroneTestSceneCfg(InteractiveSceneCfg):
    # ground plane
    ground = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane", 
        spawn=sim_utils.GroundPlaneCfg()
    )
    # lights
    lights = AssetBaseCfg(
        prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0,)
    )

    # robot asset for sim
    drone: ArticulationCfg = X500_DRONE_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot") # type: ignore

# action space
@configclass
class ActionsCfg:
    '''
    4 output nodes for the PPO net/ action spaces 
    4 motor joint torque or velocity control, go for torque or force control
    '''
    rotor_actions = mdp.RotorForceActionCfg(
        asset_name="drone", 
        scale=3.0
    )
    # joint_actions = mdp.JointEffortActionCfg(
    #     asset_name="drone",
    #     joint_names=["rotor_[0-3]"],
    #     scale=100.0, # isaaclab env max motor speed scale
    #     offset=0.0
    # ) # ---> will be replacing this is only moves the mesh at most and erratic thrust dynamics with just this

# observation space
@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        # we require drone pose (pos+quat), vel(lin/ang)
        # Drone/Quadcoptor Pose Estimate (art centre pose)
        # GPS/EKF pose (simulated)
        drone_pos_w = ObsTerm(
            func=mdp.root_pos_w,
            params={"asset_cfg": SceneEntityCfg("drone")}
        )
        drone_quat_w = ObsTerm(
            func=mdp.root_quat_w,
            params={"asset_cfg": SceneEntityCfg("drone")}
        ) # drone orientation in quaternions (faster calcs)

        # Drone/Quadcoptor Linear Vel (art lin vel)
        linear_vel_drone = ObsTerm(
            func=mdp.base_lin_vel, # base or root (try both and check and then fix one)
            params={"asset_cfg": SceneEntityCfg("drone")}
        )
        # Drone/Quadcoptor Angular Vel (art ang vel)
        angular_vel_drone = ObsTerm(
            func=mdp.base_ang_vel,
            params={"asset_cfg": SceneEntityCfg("drone")}
        )


        # pre-defined ones down here
        # last action states (testable)
        last_agent_action = ObsTerm(
            func=mdp.last_action,
            params={"action_name": "rotor_actions"}
        )

        def __post_init__(self):
            self.enable_corruption = False # to add noise to feedback/observation data
            self.concatenate_terms = True
    
    policy: PolicyCfg = PolicyCfg()

# reward model
@configclass
class RewardCfg:
    ''' The required task is takeoff to required fixed target height and also stablize '''

    # -------------------------------------------------------

    # (3) pos(x,y) tracking reward/penalty
    pos_tracking_reward = RewTerm(
        func=cmdp.pos_prog_exp_l2,
        weight=0.5,
        params={
            "asset_cfg": SceneEntityCfg("drone"),
            "sigma": 2.0
        }
    )
    # (1) takeoff/altitude rewarding/penalty
    altitude_reward = RewTerm(
        func=cmdp.altitude_exp_l2,
        weight=5.0, # rew_term
        params={
            "asset_cfg": SceneEntityCfg("drone"),
            "sigma": 8.0
        }
    )
    # (2) floor is lava reward
    floor_is_lava = RewTerm(
        func=cmdp.ground_penalty_l2,
        weight=1.0,
        params={
            "asset_cfg": SceneEntityCfg("drone"),
            "drone_ground_clearance": 0.4540 # in [m]
        }
    )

    # (4) level drone objective
    smooth_level_drone_reward = RewTerm(
        func=cmdp.level_drone_exp_l2,
        weight=0.5,
        params={
            "asset_cfg": SceneEntityCfg("drone"),
            "sigma": 0.8
        }
    )
    # -------------------------------------------------------
    
    # pre-existing reward/penalty terms
    # penalty for just existing
    termination_penalty = RewTerm(
        func=mdp.is_terminated,
        weight=-20.0
    )
    alive_penalty = RewTerm(
        func=mdp.is_alive,
        weight=0.0
    ) # small incentive for being alive
    # action penalties
    action_rate_penalty = RewTerm(
        func=mdp.action_rate_l2,
        weight=-0.0
    )
    action_penalty = RewTerm(
        func=mdp.action_l2,
        weight=-0.0
    )


# event handle cfg
@configclass
class EventCfg:
    # ---------------------------- events on reset ---------------------------
    scene_state = EventTerm(
        func=mdp.reset_scene_to_default,
        mode="reset"
    )
    # ------------------------------------------------------------------------

# termination handling cfg (simple termination setup)
@configclass
class TerminationCfg:
    # reset on timeout
    time_out = DoneTerm(func=mdp.time_out, time_out=False)
    # reset on toppling to unsafe drone orientations
    toppling_time_out = DoneTerm(
        func=cmdp.terminate_on_topple,
        params={"asset_cfg": SceneEntityCfg("drone")}
    )


@configclass
class DroneEnvCfg(ManagerBasedRLEnvCfg):
    '''
    let drone coords be in normal ENU cartesian coordinate system for now (for simple simulation understanding)
    '''

    # scene settings and setup
    scene: DroneTestSceneCfg = DroneTestSceneCfg(num_envs=3, env_spacing=2.5)
    # Basic Settings
    actions: ActionsCfg = ActionsCfg()
    observations: ObservationsCfg = ObservationsCfg()
    events: EventCfg = EventCfg()
    # # MDP settings
    rewards: RewardCfg = RewardCfg()
    terminations: TerminationCfg = TerminationCfg()
    
    # curriculum: CurriculumCfg = CurriculumCfg() (to be added later on....maybe)

    def __post_init__(self):
        """Post initialization."""
        '''
        general info:
            In Isaac Lab, the use of substeps has been replaced by a combination of the simulation dt and the decimation parameters. 
            For example, in IsaacGymEnvs, having dt=1/60 and substeps=2 is equivalent to taking 2 simulation steps with dt=1/120, but running the task step at 1/60 seconds. 
            The decimation parameter is a task parameter that controls the number of simulation steps to take for each task (or RL) step, 
            replacing the controlFrequencyInv parameter in IsaacGymEnvs. Thus, the same setup in Isaac Lab will become dt=1/120 and decimation=2.
        
        '''
        # general settings
        self.decimation = 2
        self.episode_length_s = 12.0
        # viewer settings
        self.viewer.eye = (3.5, 3.5, 3.5)
        # simulation settings
        self.sim.dt = 1.0/60.0
        self.sim.render_interval = self.decimation


