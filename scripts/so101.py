#!/usr/bin/env python3
import logging
import time
import math
from dataclasses import asdict
from pprint import pformat
from typing import Optional, Dict, List

import rclpy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

# LeRobot
from lerobot.configs import parser
from lerobot.processor import RobotProcessorPipeline, make_default_processors
from lerobot.teleoperators import make_teleoperator_from_config, Teleoperator
from lerobot.robots import make_robot_from_config, Robot
from lerobot.utils.import_utils import register_third_party_devices
from lerobot.utils.utils import init_logging
from lerobot.utils.visualization_utils import log_rerun_data
from lerobot.scripts.lerobot_teleoperate import TeleoperateConfig

# Optional: rerun (only used when cfg.display_data is True)
try:
    import rerun as rr
    from lerobot.utils.visualization_utils import init_rerun
    _HAS_RERUN = True
except Exception:
    rr = None
    init_rerun = None
    _HAS_RERUN = False

"""
UR5 teleop bridge (SO101 leader -> UR5 in MuJoCo/ros2_control)
- Reads SO101 leader angles via LeRobot teleoperator
- Captures leader0 (zero) at start to avoid UR5 jump
- Captures UR5 start pose from /joint_states (ur5_0)
- Publishes UR5 joint commands as: ur5_cmd = ur5_0 + mapped(leader_now - leader0)
- DOES NOT require SO101 follower observation (avoids "no status packet" crashes)
Publishes:
  /arm_controller/commands  (std_msgs/Float64MultiArray)
Subscribes:
  /joint_states (sensor_msgs/JointState)
"""


# =====================================================================================
# UR5 joint order (MUST match your controller arm_controller joints list)
# =====================================================================================
UR5_JOINT_NAMES: List[str] = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


# =====================================================================================
# JointState reader (captures UR5 start pose)
# =====================================================================================
class UR5JointStateReader:
    def __init__(self, node):
        self._latest: Optional[JointState] = None
        self._sub = node.create_subscription(JointState, "/joint_states", self._cb, 10)

    def _cb(self, msg: JointState) -> None:
        self._latest = msg

    def get_current(self, node, joint_names: List[str], timeout: float = 3.0) -> List[float]:
        """
        Spin `node` until we have positions for all `joint_names`.
        """
        start = time.time()
        while rclpy.ok():
            if self._latest is not None and len(self._latest.name) == len(self._latest.position):
                name_to_pos = dict(zip(self._latest.name, self._latest.position))
                if all(j in name_to_pos for j in joint_names):
                    return [float(name_to_pos[j]) for j in joint_names]

            if (time.time() - start) > timeout:
                raise RuntimeError(
                    f"Timed out waiting for joints {joint_names} on /joint_states. "
                    "Make sure joint_state_broadcaster is active."
                )

            rclpy.spin_once(node, timeout_sec=0.05)

        raise RuntimeError("ROS shutdown while waiting for /joint_states")


# =====================================================================================
# Leader key extraction (matches your printed LeRobot dict keys)
# =====================================================================================
def get_leader_vec_deg(robot_action_to_send: dict) -> Dict[str, float]:
    """
    Extract leader DOFs (degrees) from LeRobot robot_action_to_send dict.
    Keys seen in your logs:
      shoulder_pan.pos
      shoulder_lift.pos
      elbow_flex.pos
      wrist_flex.pos
      wrist_roll.pos
      gripper.pos (ignored)
    """
    def f(key: str) -> float:
        v = robot_action_to_send.get(key, 0.0)
        
        try:
            return float(v)
        except Exception:
            return 0.0

    return {
        "shoulder_pan":  f("shoulder_pan.pos"),
        "shoulder_lift": f("shoulder_lift.pos"),
        "elbow":         f("elbow_flex.pos"),
        "wrist_flex":    f("wrist_flex.pos"),
        "wrist_roll":    f("wrist_roll.pos"),
    }


def deg2rad(d: float) -> float:
    return float(d) * math.pi / 180.0


# =====================================================================================
# Map leader deltas -> UR5 deltas (5DOF -> 6DOF; hold wrist_3)
# =====================================================================================
def compute_ur5_cmd_from_deltas(
    leader_now_deg: Dict[str, float],
    leader0_deg: Dict[str, float],
    ur5_0: List[float],
) -> List[float]:
    UR5_ELBOW_NEUTRAL = -1.5708

    d_pan   = deg2rad(leader_now_deg["shoulder_pan"]  - leader0_deg["shoulder_pan"])
    d_lift  = deg2rad(leader_now_deg["shoulder_lift"] - leader0_deg["shoulder_lift"])
    d_elbow = deg2rad(leader_now_deg["elbow"]         - leader0_deg["elbow"])
    d_wflex = deg2rad(leader_now_deg["wrist_flex"]    - leader0_deg["wrist_flex"])
    d_wroll = deg2rad(leader_now_deg["wrist_roll"]    - leader0_deg["wrist_roll"])

    # Signs may need tuning. This version matches your earlier mapping assumption.
    return [
        ur5_0[0] + (-d_pan),                  # shoulder_pan_joint
        ur5_0[1] + (-d_lift),                 # shoulder_lift_joint
        -ur5_0[3] + (d_elbow),       # elbow_joint
        -1.5708,                              # wrist_1_joint (locked)
        -1.5708,                             # wrist_2_joint (free but stable)
        ur5_0[5] + ( d_wroll),                # wrist_3_joint
    ]

# =====================================================================================
# TELEOP LOOP (UR5-only; follower optional and non-blocking)
# =====================================================================================
def teleop_loop(
    teleop: Teleoperator,
    robot: Robot,
    fps: int,
    teleop_action_processor: RobotProcessorPipeline,
    robot_action_processor: RobotProcessorPipeline,
    robot_observation_processor: RobotProcessorPipeline,
    display_data: bool = False,
    duration: Optional[float] = None,
    cmd_pub=None,
    ros_node=None,
    print_every_n: int = 10,
    ur5_0: Optional[List[float]] = None,
    drive_follower: bool = False,   # set True if you want to also move the SO101 follower
):
    if cmd_pub is None:
        raise RuntimeError("cmd_pub is None")
    if ros_node is None:
        raise RuntimeError("ros_node is None")
    if ur5_0 is None or len(ur5_0) != 6:
        raise RuntimeError("ur5_0 must be a 6-element list captured from /joint_states")

    logging.info("Starting teleop loop (UR5 delta teleop)")
    period = 1.0 / float(fps)
    start_time = time.time()
    step = 0

    leader0: Optional[Dict[str, float]] = None

    while rclpy.ok():
        loop_start = time.time()

        if duration is not None and (loop_start - start_time) > duration:
            logging.info("Teleop duration reached")
            break

        # 1) leader action
        teleop_action = teleop.get_action()
        if teleop_action is None:
            time.sleep(0.001)
            continue

        # IMPORTANT:
        # For UR5 teleop, we do NOT need follower observation.
        # We also don't want crashes from bus sync_read failures.
        obs = None

        # 2) process -> action dict (LeRobot processors expect a tuple; obs can be None)
        try:
            robot_action_to_send = robot_action_processor((teleop_action, obs))
        except Exception as e:
            logging.warning("robot_action_processor failed: %s", e)
            time.sleep(0.001)
            continue

        if not isinstance(robot_action_to_send, dict):
            logging.warning("robot_action_to_send is not a dict: %s", type(robot_action_to_send))
            time.sleep(0.001)
            continue

        # 3) leader pose extraction
        leader_now = get_leader_vec_deg(robot_action_to_send)

        # Capture leader0 on first valid frame (zeroing)
        if leader0 is None:
            leader0 = leader_now
            logging.info("Captured leader0 (zero reference). No UR5 command sent this frame.")
            step += 1
            continue

        # 4) compute UR5 command (delta-based)
        ur5_cmd = compute_ur5_cmd_from_deltas(leader_now, leader0, ur5_0)

        # 5) publish to ros2_control
        msg = Float64MultiArray()
        msg.data = ur5_cmd
        cmd_pub.publish(msg)

        # Debug prints
        if (step % max(1, print_every_n)) == 0:
            print("\n================ ROBOT ACTION TO SEND ================")
            for k, v in robot_action_to_send.items():
                try:
                    print(f"{k:<20} : {float(v): .4f}")
                except Exception:
                    print(f"{k:<20} : {v}")

            print("------------------------------------------------------")
            print("UR5_0  :", [f"{q: .4f}" for q in ur5_0])
            print("UR5_CMD:", [f"{q: .4f}" for q in ur5_cmd])
            print("======================================================\n")

        # keep ROS callbacks responsive
        rclpy.spin_once(ros_node, timeout_sec=0.0)

        # OPTIONAL: drive the SO101 follower too, but never crash if it disconnects
        if drive_follower:
            try:
                robot.send_action(robot_action_to_send)
            except Exception as e:
                logging.warning("Follower send_action failed (ignored): %s", e)

            if display_data and _HAS_RERUN:
                try:
                    log_rerun_data(
                        teleop_action=teleop_action,
                        robot_action=robot_action_to_send,
                        robot_observation=None,
                    )
                except Exception:
                    pass

        # rate control
        elapsed = time.time() - loop_start
        time.sleep(max(0.0, period - elapsed))
        step += 1


# =====================================================================================
# ENTRYPOINT (LeRobot CLI + ROS)
# =====================================================================================
@parser.wrap()
def teleoperate(cfg: TeleoperateConfig):
    init_logging()
    logging.info("Teleoperate config:\n%s", pformat(asdict(cfg)))
    register_third_party_devices()

    # ROS init once
    rclpy.init()
    ros_node = rclpy.create_node("ur5_teleop_bridge")
    cmd_pub = ros_node.create_publisher(Float64MultiArray, "/arm_controller/commands", 10)
    logging.info("ROS2 publisher created on /arm_controller/commands")

    # Rerun
    if cfg.display_data:
        if not _HAS_RERUN:
            logging.warning("cfg.display_data=True but rerun is not installed; continuing without rerun.")
        else:
            init_rerun(session_name="teleoperation")

    # Create devices
    teleop = make_teleoperator_from_config(cfg.teleop)
    robot = make_robot_from_config(cfg.robot)

    teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()

    # Connect devices
    teleop.connect()
    robot.connect()
    logging.info("Teleop and robot connected")

    # Capture UR5 start pose from /joint_states
    js_reader = UR5JointStateReader(ros_node)
    logging.info("Waiting for /joint_states to capture UR5 start pose...")
    ur5_0 = js_reader.get_current(ros_node, UR5_JOINT_NAMES, timeout=5.0)
    logging.info("Captured UR5 start pose: %s", [f"{q:.4f}" for q in ur5_0])

    try:
        teleop_loop(
            teleop=teleop,
            robot=robot,
            fps=int(cfg.fps),
            display_data=bool(cfg.display_data),
            duration=cfg.teleop_time_s,
            teleop_action_processor=teleop_action_processor,
            robot_action_processor=robot_action_processor,
            robot_observation_processor=robot_observation_processor,
            cmd_pub=cmd_pub,
            ros_node=ros_node,
            print_every_n=10,
            ur5_0=ur5_0,
            drive_follower=False,   # set True only if you want follower motion
        )
    except KeyboardInterrupt:
        logging.info("Teleop interrupted by user")
    finally:
        # Shutdown cleanly
        try:
            if cfg.display_data and _HAS_RERUN:
                rr.rerun_shutdown()
        except Exception:
            pass

        try:
            teleop.disconnect()
        except Exception:
            logging.exception("Error while disconnecting teleop; ignoring during shutdown.")

        try:
            robot.disconnect()
        except Exception:
            logging.exception("Error while disconnecting robot; ignoring during shutdown.")

        ros_node.destroy_node()
        rclpy.shutdown()
        logging.info("Teleop shutdown complete")


def main():
    teleoperate()


if __name__ == "__main__":
    main()
