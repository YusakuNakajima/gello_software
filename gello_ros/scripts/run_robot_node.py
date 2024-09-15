#!/usr/bin/env python3
import signal
import sys
from dataclasses import dataclass
from pathlib import Path

from gello_ros.robots.robot import BimanualRobot, PrintRobot
from gello_ros.zmq_core.robot_node import ZMQServerRobot

import rospy
import time


def signal_handler(sig, frame):
    print("Exiting...")
    rospy.signal_shutdown("Ctrl+C pressed")
    sys.exit(0)


def main():
    rospy.init_node("gello_robot_node", anonymous=True)
    robot_type: str = "cartesian_compliance_control"
    robot_port: int = 6001
    hostname: str = "127.0.0.1"

    port = robot_port
    if robot_type == "joint_position_control":
        from gello_ros.robots.ros_joint_position_control_robot import (
            JointPositionControlRobot,
        )

        robot = JointPositionControlRobot()
    elif robot_type == "cartesian_compliance_control":
        from gello_ros.robots.ros_cartesian_compliance_control_robot import (
            CartesianComplianceControlRobot,
        )

        robot = CartesianComplianceControlRobot()

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
