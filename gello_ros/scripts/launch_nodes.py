#!/usr/bin/env python3
from dataclasses import dataclass
from pathlib import Path

import tyro

from gello_ros.robots.robot import BimanualRobot, PrintRobot
from gello_ros.zmq_core.robot_node import ZMQServerRobot

import rospy


@dataclass
class Args:
    robot: str = "ros"
    robot_port: int = 6001
    hostname: str = "127.0.0.1"
    robot_ip: str = "192.168.58.42"


def launch_robot_server(args: Args):
    port = args.robot_port
    if args.robot == "sim_ur":
        MENAGERIE_ROOT: Path = (
            Path(__file__).parent.parent / "third_party" / "mujoco_menagerie"
        )
        xml = MENAGERIE_ROOT / "universal_robots_ur5e" / "ur5e.xml"
        gripper_xml = MENAGERIE_ROOT / "robotiq_2f85" / "2f85.xml"
        from gello_ros.robots.sim_robot import MujocoRobotServer

        server = MujocoRobotServer(
            xml_path=xml, gripper_xml_path=gripper_xml, port=port, host=args.hostname
        )
        server.serve()
    elif args.robot == "sim_panda":
        from gello_ros.robots.sim_robot import MujocoRobotServer

        MENAGERIE_ROOT: Path = (
            Path(__file__).parent.parent / "third_party" / "mujoco_menagerie"
        )
        xml = MENAGERIE_ROOT / "franka_emika_panda" / "panda.xml"
        gripper_xml = None
        server = MujocoRobotServer(
            xml_path=xml, gripper_xml_path=gripper_xml, port=port, host=args.hostname
        )
        server.serve()
    elif args.robot == "sim_xarm":
        from gello_ros.robots.sim_robot import MujocoRobotServer

        MENAGERIE_ROOT: Path = (
            Path(__file__).parent.parent / "third_party" / "mujoco_menagerie"
        )
        xml = MENAGERIE_ROOT / "ufactory_xarm7" / "xarm7.xml"
        gripper_xml = None
        server = MujocoRobotServer(
            xml_path=xml, gripper_xml_path=gripper_xml, port=port, host=args.hostname
        )
        server.serve()

    else:
        if args.robot == "ros":
            from gello_ros.robots.ros_robot import ROSRobot

            robot = ROSRobot()
        elif args.robot == "xarm":
            from gello_ros.robots.xarm_robot import XArmRobot

            robot = XArmRobot(ip=args.robot_ip)
        elif args.robot == "ur":
            from gello_ros.robots.ur import URRobot

            robot = URRobot(robot_ip=args.robot_ip, no_gripper=True)
        elif args.robot == "panda":
            from gello_ros.robots.panda import PandaRobot

            robot = PandaRobot(robot_ip=args.robot_ip)
        elif args.robot == "bimanual_ur":
            from gello_ros.robots.ur import URRobot

            # IP for the bimanual robot setup is hardcoded
            _robot_l = URRobot(robot_ip="192.168.2.10")
            _robot_r = URRobot(robot_ip="192.168.1.10")
            robot = BimanualRobot(_robot_l, _robot_r)
        elif args.robot == "none" or args.robot == "print":
            robot = PrintRobot(8)

        else:
            raise NotImplementedError(
                f"Robot {args.robot} not implemented, choose one of: sim_ur, xarm, ur, bimanual_ur, none"
            )
        server = ZMQServerRobot(robot, port=port, host=args.hostname)
        print(f"Starting robot server on port {port}")
        server.serve()


def main(args):
    rospy.init_node("gello_env_launch", anonymous=True)
    launch_robot_server(args)


if __name__ == "__main__":
    main(tyro.cli(Args))
