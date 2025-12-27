#!/usr/bin/env python3
import logging
import time
import math
from dataclasses import asdict
from pprint import pformat
from typing import Optional

import rclpy
from std_msgs.msg import Float64MultiArray

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
make sure you have lerobot library installed
ros2 run panda_mujoco so101.py --robot.type=so101_follower   --robot.port=/dev/ttyACM0   --teleop.type=so101_leader   --teleop.port=/dev/ttyACM1

"""
# =====================================================================================
# PANDA MAPPING
# =====================================================================================
def extract_panda_joint_vector(robot_action_to_send: dict) -> list[float]:
    """
    Convert SO101 robot_action_to_send dict → Panda 7-DoF joint vector (radians)

    Expected keys in robot_action_to_send (degrees):
      - shoulder_pan.pos
      - shoulder_lift.pos
      - elbow_flex.pos
      - wrist_flex.pos
      - wrist_roll.pos
      - gripper.pos (ignored here)

    Returns:
      List[float] length 7 (radians)
    """

    def deg2rad(x: float) -> float:
        return float(x) * math.pi / 180.0

    def get(name: str) -> float:
        # defensive default to 0
        v = robot_action_to_send.get(name, 0.0)
        try:
            return float(v)
        except Exception:
            return 0.0

    q1 = -deg2rad(get("shoulder_pan.pos"))
    q2 = 0.0
    q3 = deg2rad(get("shoulder_lift.pos"))
    q4 = deg2rad(get("elbow_flex.pos"))
    q5 = deg2rad(get("wrist_flex.pos"))
    q6 = deg2rad(get("wrist_roll.pos"))
    q7 = 0.0 #deg2rad(get("wrist_roll.pos"))

    panda_q = [q1, q2, q3, q4, q5, q6, q7]
    return panda_q


# =====================================================================================
# TELEOP LOOP
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
    panda_pub=None,
    ros_node=None,
    print_every_n: int = 1,   # set to 10 to reduce spam
):
    logging.info("Starting teleop loop")
    period = 1.0 / float(fps)
    start_time = time.time()
    step = 0

    while True:
        loop_start = time.time()

        if duration is not None and (loop_start - start_time) > duration:
            logging.info("Teleop duration reached")
            break

        # 1) leader action
        teleop_action = teleop.get_action()
        if teleop_action is None:
            time.sleep(0.001)
            continue

        # 2) follower observation (or robot observation)
        obs = robot.get_observation()
        obs = robot_observation_processor(obs)

        # 3) process -> action for robot
        robot_action_to_send = robot_action_processor((teleop_action, obs))

        # 4) map -> panda
        panda_joint_cmd = extract_panda_joint_vector(robot_action_to_send)

        # Debug prints
        if (step % max(1, print_every_n)) == 0:
            print("\n================ ROBOT ACTION TO SEND ================")
            if isinstance(robot_action_to_send, dict):
                for k, v in robot_action_to_send.items():
                    try:
                        vv = float(v)
                        print(f"{k:<20} : {vv: .4f}")
                    except Exception:
                        print(f"{k:<20} : {v}")
            else:
                print("robot_action_to_send is not a dict:", type(robot_action_to_send))

            print("------------------------------------------------------")
            print("PANDA_CMD:", [f"{q: .4f}" for q in panda_joint_cmd])
            print("======================================================\n")

        # 5) publish to ros2_control
        if panda_pub is not None:
            if len(panda_joint_cmd) != 7:
                logging.error("Panda cmd wrong length: %d (expected 7)", len(panda_joint_cmd))
            else:
                msg = Float64MultiArray()
                msg.data = panda_joint_cmd
                panda_pub.publish(msg)

        # keep ROS callbacks responsive (if any)
        if ros_node is not None:
            rclpy.spin_once(ros_node, timeout_sec=0.0)

        # 6) send to follower (keep this if you still want the so101 follower moving)
        robot.send_action(robot_action_to_send)

        # 7) optional rerun
        if display_data and _HAS_RERUN:
            log_rerun_data(
                teleop_action=teleop_action,
                robot_action=robot_action_to_send,
                robot_observation=obs,
            )

        # 8) rate control
        elapsed = time.time() - loop_start
        sleep_time = max(0.0, period - elapsed)
        time.sleep(sleep_time)
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
    ros_node = rclpy.create_node("panda_teleop_bridge")
    panda_pub = ros_node.create_publisher(Float64MultiArray, "/arm_controller/commands", 10)
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

    teleop.connect()
    robot.connect()
    logging.info("Teleop and robot connected")

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
            panda_pub=panda_pub,
            ros_node=ros_node,
            print_every_n=1,  # change to 10 if spammy
        )
    except KeyboardInterrupt:
        logging.info("Teleop interrupted by user")
    finally:
        try:
            if cfg.display_data and _HAS_RERUN:
                rr.rerun_shutdown()
        except Exception:
            pass

        try:
            teleop.disconnect()
        except Exception:
            pass
        try:
            robot.disconnect()
        except Exception:
            pass

        ros_node.destroy_node()
        rclpy.shutdown()
        logging.info("Teleop shutdown complete")

def main():
    teleoperate()


if __name__ == "__main__":
    # parser.wrap() means this reads CLI args in LeRobot style
    main()
