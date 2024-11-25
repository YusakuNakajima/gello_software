import os
from dataclasses import dataclass
from typing import Dict, Optional, Sequence, Tuple

import numpy as np

from gello_ros.agents.agent import Agent
from gello_ros.robots.dynamixel import DynamixelRobot
import time

import rospy
import moveit_commander


@dataclass
class DynamixelRobotConfig:
    joint_ids: Sequence[int]
    """The joint ids of GELLO (not including the gripper). Usually (1, 2, 3 ...)."""

    joint_offsets: Sequence[float]
    """The joint offsets of GELLO. There needs to be a joint offset for each joint_id and should be a multiple of pi/2."""

    joint_signs: Sequence[int]
    """The joint signs of GELLO. There needs to be a joint sign for each joint_id and should be either 1 or -1.

    This will be different for each arm design. Refernce the examples below for the correct signs for your robot.
    """

    gripper_config: Tuple[int, int, int]
    """The gripper config of GELLO. This is a tuple of (gripper_joint_id, degrees in open_position, degrees in closed_position)."""

    def __post_init__(self):
        assert len(self.joint_ids) == len(self.joint_offsets)
        assert len(self.joint_ids) == len(self.joint_signs)

    def make_robot(
        self, port: str = "/dev/ttyUSB0", start_joints: Optional[np.ndarray] = None
    ) -> DynamixelRobot:
        return DynamixelRobot(
            joint_ids=self.joint_ids,
            joint_offsets=list(self.joint_offsets),
            real=True,
            joint_signs=list(self.joint_signs),
            port=port,
            gripper_config=self.gripper_config,
            start_joints=start_joints,
            baudrate=2000000,  # 2M bps
        )


PORT_CONFIG_MAP: Dict[str, DynamixelRobotConfig] = {
    # UR
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8ISUQE-if00-port0": DynamixelRobotConfig(
        joint_ids=(1, 2, 3, 4, 5, 6),
        joint_offsets=(
            6 * np.pi / 2,
            3 * np.pi / 2,
            2 * np.pi / 2,
            3 * np.pi / 2,
            1 * np.pi / 2,
            0 * np.pi / 2,
        ),
        joint_signs=(1, 1, -1, 1, 1, 1),
        gripper_config=None,  # (7, 113.091015625, 71.291015625),
    ),
    # Cobotta
    # "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT88YXAT-if00-port0": DynamixelRobotConfig(
    #     joint_ids=(1, 2, 3, 4, 5, 6),
    #     joint_offsets=(
    #         3 * np.pi / 2,
    #         4 * np.pi / 2,
    #         2 * np.pi / 2,
    #         0 * np.pi / 2,
    #         3 * np.pi / 2 + np.pi / 4,
    #         3 * np.pi / 2,
    #     ),
    #     joint_signs=(1, 1, -1, 1, -1, 1),
    #     gripper_config=(7, 96, 54),
    # ),
    # FR3
    # "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8ISUQE-if00-port0": DynamixelRobotConfig(
    #     joint_ids=(1, 2, 3, 4, 5, 6),
    #     joint_offsets=(
    #         5 * np.pi / 2,
    #         3 * np.pi / 2,
    #         2 * np.pi / 2,
    #         3 * np.pi / 2,
    #         1 * np.pi / 2,
    #         0 * np.pi / 2,
    #         2 * np.pi / 2,
    #         6 * np.pi / 2,
    #         3 * np.pi / 2,
    #         3 * np.pi / 2,
    #     ),
    #     joint_signs=(1, 1, -1, 1, 1, 1),
    #     gripper_config=(7, 113.091015625, 71.291015625),
    # ),
}


class GelloAgent(Agent):
    def __init__(
        self,
        port: str,
        dynamixel_config: Optional[DynamixelRobotConfig] = None,
        start_joints: Optional[np.ndarray] = None,
        mode: Optional[str] = "unilateral_position",
    ):
        if dynamixel_config is not None:
            if rospy.get_param("~use_gripper", False) == False:
                dynamixel_config.gripper_config = None
            self._robot = dynamixel_config.make_robot(
                port=port, start_joints=start_joints
            )
        else:
            assert os.path.exists(port), port
            assert port in PORT_CONFIG_MAP, f"Port {port} not in config map"
            config = PORT_CONFIG_MAP[port]
            if rospy.get_param("~use_gripper", False) == False:
                config.gripper_config = None
            self._robot = config.make_robot(port=port, start_joints=start_joints)
        self._mode = mode

        # Set constants for torque feedback
        self.stall_torque = rospy.get_param("~stall_torque", 0.52)
        self.stall_current = rospy.get_param("~stall_current", 1.5)
        self.torque_constant = self.stall_torque / self.stall_current
        self.torque_rate = rospy.get_param(
            "~torque_rate", [0.005, 0.005, 0.005, 0.005, 0.005, 0.005]
        )
        self.current_goal_constant = rospy.get_param("~current_goal_constant", 0.001)

        # Set control mode
        self._robot.set_control_mode(
            "CURRENT_MODE"
        )  # POSITION_MODE,CURRENT_BASED_POSITION_MODE
        # Set torque
        self._robot.set_torque_mode(True)
        if self._mode == "bilateral":
            self._robot.set_read_only(False)
        else:
            self._robot.set_read_only(True)

    def act(self, obs: Dict[str, np.ndarray]) -> np.ndarray:
        dynamixel_joints = self._robot.get_joint_state()
        if self._mode == "bilateral":
            jacobian_inv = np.linalg.pinv(obs["jacobian"])
            wrench = obs["ee_wrench"]
            wrench[2] *= -1
            joint_torques = np.dot(jacobian_inv, wrench)
            joint_currents = joint_torques / self.torque_constant
            dynamixel_current_goals = joint_currents / self.current_goal_constant
            dynamixel_current_goals = np.round(
                dynamixel_current_goals * self.torque_rate
            ).astype(int)
            self._robot.command_joint_torque(dynamixel_current_goals)

        return dynamixel_joints
        # current_q = dynamixel_joints[:-1]  # last one dim is the gripper
        current_gripper = dynamixel_joints[-1]  # last one dim is the gripper

        print(current_gripper)
        if current_gripper < 0.2:
            self._robot.set_torque_mode(False)
            return obs["joint_positions"]
        else:
            self._robot.set_torque_mode(False)
            return dynamixel_joints

    def set_torque_mode(self, torque_mode: bool):
        self._robot.set_torque_mode(torque_mode)
