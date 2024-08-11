#!/usr/bin/env python3
from dataclasses import dataclass
from pathlib import Path

from gello_ros.robots.robot import BimanualRobot, PrintRobot
from gello_ros.zmq_core.robot_node import ZMQServerRobot

import rospy


def main():
    rospy.init_node("gello_robot_node", anonymous=True)
    robot: str = "ros"
    robot_port: int = 6001
    hostname: str = "127.0.0.1"

    port = robot_port
    if robot == "ros":
        from gello_ros.robots.ros_robot import ROSRobot

        robot = ROSRobot()

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
    main()
