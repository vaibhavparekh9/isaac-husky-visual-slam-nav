#!/usr/bin/env python3
"""Drive OmniGraph: /cmd_vel -> 4 wheels, IsaacComputeOdometry -> /odom + TF + /clock.
A SimulationApp must already be running before importing this module.
"""
import omni.graph.core as og
from isaacsim.core.nodes.scripts.utils import set_target_prims

# Husky kinematics (from husky_description URDF).
WHEEL_RADIUS = 0.1651
WHEEL_TRACK = 0.5708
WHEEL_DRIVE_DAMPING = 17453.0   # velocity-control damping (per NVIDIA reference)
WHEEL_DRIVE_STIFFNESS = 0.0

WHEEL_JOINT_NAMES = [
    "front_left_wheel_joint",
    "front_right_wheel_joint",
    "rear_left_wheel_joint",
    "rear_right_wheel_joint",
]


def build_drive_graph(
    robot_prim="/Husky",
    art_root="/Husky/base_link",
    wheel_prims=("front_left_wheel", "front_right_wheel",
                 "rear_left_wheel", "rear_right_wheel"),
    cmd_vel_topic="/cmd_vel",
    domain_id=0,
    graph_path="/Husky/ActionGraph",
    extra_tf_prims=(),
):
    keys = og.Controller.Keys
    og.Controller.edit(
        {
            "graph_path": graph_path,
            "evaluator_name": "execution",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
        },
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("SubTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                ("BreakLin", "omni.graph.nodes.BreakVector3"),
                ("BreakAng", "omni.graph.nodes.BreakVector3"),
                ("DiffCtrl", "isaacsim.robot.wheeled_robots.DifferentialController"),
                ("IdxL", "omni.graph.nodes.ArrayIndex"),
                ("IdxR", "omni.graph.nodes.ArrayIndex"),
                ("VelArray", "omni.graph.nodes.ConstructArray"),
                ("TokFL", "omni.graph.nodes.ConstantToken"),
                ("TokFR", "omni.graph.nodes.ConstantToken"),
                ("TokRL", "omni.graph.nodes.ConstantToken"),
                ("TokRR", "omni.graph.nodes.ConstantToken"),
                ("NameArray", "omni.graph.nodes.ConstructArray"),
                ("ArtCtrl", "isaacsim.core.nodes.IsaacArticulationController"),
                ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("ComputeOdom", "isaacsim.core.nodes.IsaacComputeOdometry"),
                ("PubOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                ("PubRawTF", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
                ("PubTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                ("PubClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            # ConstructArray starts with input0 only; add the other 3 inputs.
            keys.CREATE_ATTRIBUTES: [
                ("VelArray.inputs:input1", "double"),
                ("VelArray.inputs:input2", "double"),
                ("VelArray.inputs:input3", "double"),
                ("NameArray.inputs:input1", "token"),
                ("NameArray.inputs:input2", "token"),
                ("NameArray.inputs:input3", "token"),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", "SubTwist.inputs:execIn"),
                ("OnTick.outputs:tick", "ArtCtrl.inputs:execIn"),
                ("OnTick.outputs:tick", "ComputeOdom.inputs:execIn"),
                ("OnTick.outputs:tick", "PubTF.inputs:execIn"),
                ("OnTick.outputs:tick", "PubClock.inputs:execIn"),
                ("OnTick.outputs:deltaSeconds", "DiffCtrl.inputs:dt"),
                ("Context.outputs:context", "SubTwist.inputs:context"),
                ("Context.outputs:context", "PubOdom.inputs:context"),
                ("Context.outputs:context", "PubRawTF.inputs:context"),
                ("Context.outputs:context", "PubTF.inputs:context"),
                ("Context.outputs:context", "PubClock.inputs:context"),
                ("SubTwist.outputs:execOut", "DiffCtrl.inputs:execIn"),
                ("SubTwist.outputs:linearVelocity", "BreakLin.inputs:tuple"),
                ("SubTwist.outputs:angularVelocity", "BreakAng.inputs:tuple"),
                ("BreakLin.outputs:x", "DiffCtrl.inputs:linearVelocity"),
                ("BreakAng.outputs:z", "DiffCtrl.inputs:angularVelocity"),
                # 2-wheel [left,right] -> 4-wheel [FL,FR,RL,RR]=[left,right,left,right]
                ("DiffCtrl.outputs:velocityCommand", "IdxL.inputs:array"),
                ("DiffCtrl.outputs:velocityCommand", "IdxR.inputs:array"),
                ("IdxL.outputs:value", "VelArray.inputs:input0"),
                ("IdxR.outputs:value", "VelArray.inputs:input1"),
                ("IdxL.outputs:value", "VelArray.inputs:input2"),
                ("IdxR.outputs:value", "VelArray.inputs:input3"),
                # joint names array [FL,FR,RL,RR]
                ("TokFL.inputs:value", "NameArray.inputs:input0"),
                ("TokFR.inputs:value", "NameArray.inputs:input1"),
                ("TokRL.inputs:value", "NameArray.inputs:input2"),
                ("TokRR.inputs:value", "NameArray.inputs:input3"),
                ("VelArray.outputs:array", "ArtCtrl.inputs:velocityCommand"),
                ("NameArray.outputs:array", "ArtCtrl.inputs:jointNames"),
                # odometry -> /odom + odom->base_link TF
                ("ComputeOdom.outputs:execOut", "PubOdom.inputs:execIn"),
                ("ComputeOdom.outputs:execOut", "PubRawTF.inputs:execIn"),
                ("ComputeOdom.outputs:position", "PubOdom.inputs:position"),
                ("ComputeOdom.outputs:orientation", "PubOdom.inputs:orientation"),
                ("ComputeOdom.outputs:linearVelocity", "PubOdom.inputs:linearVelocity"),
                ("ComputeOdom.outputs:angularVelocity", "PubOdom.inputs:angularVelocity"),
                ("ComputeOdom.outputs:position", "PubRawTF.inputs:translation"),
                ("ComputeOdom.outputs:orientation", "PubRawTF.inputs:rotation"),
                # timestamps
                ("SimTime.outputs:simulationTime", "PubOdom.inputs:timeStamp"),
                ("SimTime.outputs:simulationTime", "PubRawTF.inputs:timeStamp"),
                ("SimTime.outputs:simulationTime", "PubTF.inputs:timeStamp"),
                ("SimTime.outputs:simulationTime", "PubClock.inputs:timeStamp"),
            ],
            keys.SET_VALUES: [
                ("Context.inputs:domain_id", domain_id),
                ("Context.inputs:useDomainIDEnvVar", False),
                ("SubTwist.inputs:topicName", cmd_vel_topic),
                ("DiffCtrl.inputs:wheelDistance", WHEEL_TRACK),
                ("DiffCtrl.inputs:wheelRadius", WHEEL_RADIUS),
                ("IdxL.inputs:index", 0),
                ("IdxR.inputs:index", 1),
                ("VelArray.inputs:arraySize", 4),
                ("VelArray.inputs:arrayType", "double[]"),
                ("NameArray.inputs:arraySize", 4),
                ("NameArray.inputs:arrayType", "token[]"),
                ("TokFL.inputs:value", WHEEL_JOINT_NAMES[0]),
                ("TokFR.inputs:value", WHEEL_JOINT_NAMES[1]),
                ("TokRL.inputs:value", WHEEL_JOINT_NAMES[2]),
                ("TokRR.inputs:value", WHEEL_JOINT_NAMES[3]),
                ("PubOdom.inputs:odomFrameId", "odom"),
                ("PubOdom.inputs:chassisFrameId", "base_link"),
                ("PubOdom.inputs:topicName", "odom"),
                ("PubRawTF.inputs:parentFrameId", "odom"),
                ("PubRawTF.inputs:childFrameId", "base_link"),
                ("PubClock.inputs:topicName", "clock"),
                ("PubTF.inputs:topicName", "tf"),
            ],
        },
    )

    # Relationships 
    set_target_prims(primPath=f"{graph_path}/ArtCtrl",
                     targetPrimPaths=[art_root], inputName="inputs:targetPrim")
    set_target_prims(primPath=f"{graph_path}/ComputeOdom",
                     targetPrimPaths=[art_root], inputName="inputs:chassisPrim")
    set_target_prims(primPath=f"{graph_path}/PubTF",
                     targetPrimPaths=[art_root], inputName="inputs:parentPrim")
    # base_link -> wheels on /tf
    tf_targets = [f"{robot_prim}/{w}" for w in wheel_prims] + list(extra_tf_prims)
    set_target_prims(primPath=f"{graph_path}/PubTF",
                     targetPrimPaths=tf_targets,
                     inputName="inputs:targetPrims")
    return graph_path


def set_wheel_drive_gains(stage, art_root="/Husky/base_link",
                          joint_names=WHEEL_JOINT_NAMES):
    """Velocity-control the wheels: stiffness 0, high damping.

    The imported joints live under /Husky/joints/<name>. Sets the angular drive
    gains so the articulation tracks commanded wheel velocities.
    """
    from pxr import UsdPhysics
    robot = art_root.rsplit("/", 1)[0]  # "/Husky"
    for jn in joint_names:
        for candidate in (f"{robot}/joints/{jn}", f"{art_root}/{jn}"):
            prim = stage.GetPrimAtPath(candidate)
            if not prim or not prim.IsValid():
                continue
            drive = UsdPhysics.DriveAPI.Get(prim, "angular")
            if not drive:
                drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
            drive.GetDampingAttr().Set(WHEEL_DRIVE_DAMPING)
            drive.GetStiffnessAttr().Set(WHEEL_DRIVE_STIFFNESS)
            break
