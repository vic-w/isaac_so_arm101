# Copyright (c) 2024-2025, Muammer Bay (LycheeAI), Louis Le Lay
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
#
# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import isaaclab_tasks.manager_based.manipulation.lift.mdp as mdp
from isaaclab.assets import RigidObjectCfg

# from isaaclab.managers NotImplementedError
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import (
    FrameTransformerCfg,
    OffsetCfg,
)
from isaaclab.sim.schemas.schemas_cfg import RigidBodyPropertiesCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import UsdFileCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
from SO_100.robots import SO_ARM100_CFG, SO_ARM100_ROSCON_CFG  # noqa: F401
from SO_100.tasks.lift.lift_env_cfg import LiftEnvCfg

from isaaclab.markers.config import FRAME_MARKER_CFG  # isort: skip

from isaaclab.sensors.camera import CameraCfg
#from isaaclab.managers import SceneEntityCfg
import isaaclab.sim as sim_utils
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.envs.mdp import observations as mdp_obs
from isaaclab.managers import SceneEntityCfg
# ----------------------------------------------------------------
# --------------- LycheeAI live asset ----------------------------
# ----------------------------------------------------------------


@configclass
class SoArm100LiftCubeEnvCfg(LiftEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # Set so arm as robot
        self.scene.robot = SO_ARM100_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        # override actions
        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot",
            joint_names=["Shoulder_Rotation", "Shoulder_Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll"],
            scale=0.5,
            use_default_offset=True,
        )
        self.actions.gripper_action = mdp.BinaryJointPositionActionCfg(
            asset_name="robot",
            joint_names=["Gripper"],
            open_command_expr={"Gripper": 0.5},
            close_command_expr={"Gripper": 0.0},
        )
        # Set the body name for the end effector
        self.commands.object_pose.body_name = ["Fixed_Gripper"]

        # Set Cube as object
        self.scene.object = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Object",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.2, 0.0, 0.015], rot=[1, 0, 0, 0]),
            spawn=UsdFileCfg(
                usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/DexCube/dex_cube_instanceable.usd",
                scale=(0.3, 0.3, 0.3),
                rigid_props=RigidBodyPropertiesCfg(
                    solver_position_iteration_count=16,
                    solver_velocity_iteration_count=1,
                    max_angular_velocity=1000.0,
                    max_linear_velocity=1000.0,
                    max_depenetration_velocity=5.0,
                    disable_gravity=False,
                ),
            ),
        )

        # Listens to the required transforms
        marker_cfg = FRAME_MARKER_CFG.copy()
        marker_cfg.markers["frame"].scale = (0.05, 0.05, 0.05)
        marker_cfg.prim_path = "/Visuals/FrameTransformer"
        self.scene.ee_frame = FrameTransformerCfg(
            prim_path="{ENV_REGEX_NS}/Robot/Base",
            debug_vis=False,
            visualizer_cfg=marker_cfg,
            target_frames=[
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/Fixed_Gripper",
                    name="end_effector",
                    offset=OffsetCfg(
                        pos=[0.01, 0.0, 0.1],
                    ),
                ),
            ],
        )


@configclass
class SoArm100LiftCubeEnvCfg_PLAY(SoArm100LiftCubeEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # disable randomization for play
        self.observations.policy.enable_corruption = False


@configclass
class SoArm100CameraLiftCubeEnvCfg(SoArm100LiftCubeEnvCfg):
    """Camera-ready variant of the default cube lifting environment."""
    
    @configclass
    class ObservationsCfg:
        """Override observations to add image features."""

        @configclass
        class PolicyCfg(ObsGroup):

            joint_pos = ObsTerm(func=mdp.joint_pos_rel)
            joint_vel = ObsTerm(func=mdp.joint_vel_rel)
            #object_position = ObsTerm(func=mdp.object_position_in_robot_root_frame)
            target_object_position = ObsTerm(func=mdp.generated_commands, params={"command_name": "object_pose"})
            actions = ObsTerm(func=mdp.last_action)

            # 🔥 只保留你需要的 observation term
            camera_image_features = ObsTerm(
                func=mdp_obs.image_features,
                params={
                    "sensor_cfg": SceneEntityCfg("camera"),
                    "data_type": "rgb",
                    "model_name": "resnet18",
                },
            )

            # 决定是否 concat（一般保持 true）
            def __post_init__(self):
                self.enable_corruption = False
                self.concatenate_terms = True

        policy: PolicyCfg = PolicyCfg()


    def __post_init__(self):
        """Currently identical to the base variant but defined separately for camera tasks."""

        super().__post_init__()

        self.scene.camera = CameraCfg(
            prim_path="{ENV_REGEX_NS}/Robot/Fixed_Gripper/Camera",
            update_period=0.0,
            height=480,
            width=640,
            data_types=("rgb",),
            offset=CameraCfg.OffsetCfg(
                pos=[0.0, -0.1, 0.0],
                rot=[0.96593, -0.25882, 0.0, 0.0],
            ),
            spawn=sim_utils.PinholeCameraCfg(
                focal_length=18.0, focus_distance=4.0, horizontal_aperture=32.0, clipping_range=(0.1, 1.0e5)
            ),
        )
                
        self.observations = SoArm100CameraLiftCubeEnvCfg.ObservationsCfg()
