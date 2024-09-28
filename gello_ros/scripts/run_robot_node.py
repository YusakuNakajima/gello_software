#!/usr/bin/env python3
import signal
import sys
from dataclasses import dataclass
from pathlib import Path

from gello_ros.robots.robot import BimanualRobot, PrintRobot
from gello_ros.zmq_core.robot_node import ZMQServerRobot

import rospy


def signal_handler(sig, frame):
    print("Exiting...")
    rospy.signal_shutdown("Ctrl+C pressed")
    sys.exit(0)


def main():
    rospy.init_node("gello_robot_node", anonymous=True)
    port: int = 6001
    hostname: str = "127.0.0.1"
    controller_type: str = rospy.get_param("~controller_type")
    no_gripper: bool = not rospy.get_param("~use_gripper")

    if controller_type == "joint_trajectory_controller":
        from gello_ros.robots.ros_joint_trajectory_control_robot import (
            JointTrajectoryControlRobot,
        )

        robot = JointTrajectoryControlRobot(no_gripper)
    elif controller_type == "cartesian_compliance_controller":
        from gello_ros.robots.ros_cartesian_compliance_control_robot import (
            CartesianComplianceControlRobot,
        )

        robot = CartesianComplianceControlRobot(no_gripper)
    elif controller_type == "cartesian_motion_controller":
        from gello_ros.robots.ros_cartesian_motion_control_robot import (
            CartesianMotionControlRobot,
        )

        robot = CartesianMotionControlRobot(no_gripper)

    elif robot == "none" or robot == "print":
        robot = PrintRobot(8)

    else:
        raise NotImplementedError(
            f"Robot {robot} not implemented, choose one of: sim_ur, xarm, ur, bimanual_ur, none"
        )
    server = ZMQServerRobot(robot, port=port, host=hostname)
    print(f"Starting robot server on port {port}")
    server.serve()


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    main()
