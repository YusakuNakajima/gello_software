from typing import Dict

import numpy as np

from gello_ros.robots.robot import Robot

import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class ROSRobot(Robot):
    """A class representing a UR robot."""

    def __init__(self, no_gripper: bool = True):
        if not no_gripper:
            print("supposed only no gripper")
            exit()
        rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)
        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        # controller_name = "/scaled_pos_joint_traj_controller/command"
        controller_name = "/cobotta/arm_controller/command"
        self.trajectory_publisher = rospy.Publisher(
            controller_name, JointTrajectory, queue_size=10
        )
        self._use_gripper = not no_gripper

    def joint_state_callback(self, msg: JointState):
        self.robot_joints = np.array(msg.position)
        # print(self.robot_joints)

    def num_dofs(self) -> int:
        """Get the number of joints of the robot.

        Returns:
            int: The number of joints of the robot.
        """
        if self._use_gripper:
            return 7
        return 6

    def get_joint_state(self) -> np.ndarray:
        """Get the current state of the leader robot.

        Returns:
            T: The current state of the leader robot.
        """
        return self.robot_joints

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        """Command the leader robot to a given state.

        Args:
            joint_state (np.ndarray): The state to command the leader robot to.
        """
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = joint_state[:6]
        point.time_from_start = rospy.Duration(0.01)  # Move immediately
        trajectory_msg.points = [point]

        self.trajectory_publisher.publish(trajectory_msg)

    def get_observations(self) -> Dict[str, np.ndarray]:
        joints = self.get_joint_state()
        pos_quat = np.zeros(7)
        gripper_pos = np.array([joints[-1]])
        return {
            "joint_positions": joints,
            "joint_velocities": joints,
            "ee_pos_quat": pos_quat,
            "gripper_position": gripper_pos,
        }


def main():
    rospy.init_node("ros_robot")
    ros_robot = ROSRobot(no_gripper=True)


if __name__ == "__main__":
    main()
