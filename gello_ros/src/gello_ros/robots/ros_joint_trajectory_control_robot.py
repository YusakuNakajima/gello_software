from typing import Dict

import numpy as np

from gello_ros.robots.robot import Robot

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, WrenchStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_commander import MoveGroupCommander


class JointTrajectoryControlRobot(Robot):
    """A class representing a UR robot."""

    def __init__(self, no_gripper: bool = True, no_wrench: bool = True):
        if not no_gripper:
            print("supposed only cobotta gripper")
            if not no_gripper:
                from gello_ros.robots.cobotta_gripper import CobottaGripper

                self.gripper = CobottaGripper()
                self.gripper.connect()
                print("gripper connected")

        self.joint_names_order = rospy.get_param("~joint_names_order")
        self.joint_max_vel = rospy.get_param("~joint_max_vel")
        self.joint_pos_limits_upper = rospy.get_param("~joint_pos_limits_upper")
        self.joint_pos_limits_lower = rospy.get_param("~joint_pos_limits_lower")
        self.trajectory_publisher = rospy.Publisher(
            rospy.get_param("~joint_trajectory_controller_command_topic"),
            JointTrajectory,
            queue_size=1,
        )
        self.move_group = MoveGroupCommander(
            rospy.get_param("~move_group_name", "manipulator")
        )
        rospy.Subscriber(
            rospy.get_param("~joint_states_topic"),
            JointState,
            self.joint_states_callback,
        )
        if not no_wrench:
            rospy.Subscriber(
                rospy.get_param("~wrench_topic"),
                WrenchStamped,
                self.wrench_callback,
            )

        control_freq = 100
        self._min_traj_dur = 5.0 / control_freq
        self._speed_scale = 1
        self._use_gripper = not no_gripper
        self._use_FTsensor = not no_wrench

    def joint_states_callback(self, msg: JointState):
        self.ros_joint_state = msg

    def wrench_callback(self, msg: WrenchStamped):
        self._wrench = msg

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

        # Create a dictionary for easy lookup
        joint_positions_dict = dict(
            zip(self.ros_joint_state.name, self.ros_joint_state.position)
        )
        # Reorder the joints according to self.joint_names
        self.robot_joints = np.array(
            [joint_positions_dict[name] for name in self.joint_names_order]
        )

        return self.robot_joints

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        """Command the leader robot to a given state.

        Args:
            joint_state (np.ndarray): The state to command the leader robot to.
        """
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names_order
        point = JointTrajectoryPoint()
        dur = []
        current_robot_joints = self.get_joint_state()
        for i, name in enumerate(trajectory_msg.joint_names):
            pos = joint_state[i]
            pos_lower = self.joint_pos_limits_lower[i]
            pos_upper = self.joint_pos_limits_upper[i]
            if pos < pos_lower:
                pos = pos_lower
            elif pos > pos_upper:
                pos = pos_upper
            point.positions.append(pos)

            dur.append(
                max(
                    abs(joint_state[i] - current_robot_joints[i])
                    / self.joint_max_vel[i],
                    self._min_traj_dur,
                )
            )
        if self._use_gripper:
            gripper_pos = joint_state[-1]
            print(gripper_pos)
            # self.gripper.move(gripper_pos, 255, 10)

        # set the target convergence time of the JTC to match the joint that tasks the longest time to move
        point.time_from_start = rospy.Duration(max(dur) / self._speed_scale)
        trajectory_msg.points.append(point)
        self.trajectory_publisher.publish(trajectory_msg)

    def get_observations(self) -> Dict[str, np.ndarray]:
        joints = self.get_joint_state()
        pos_quat = np.zeros(7)
        gripper_pos = np.array([joints[-1]])
        if self._use_FTsensor:
            wrench = np.array(
                [
                    self._wrench.wrench.force.x,
                    self._wrench.wrench.force.y,
                    self._wrench.wrench.force.z,
                    self._wrench.wrench.torque.x,
                    self._wrench.wrench.torque.y,
                    self._wrench.wrench.torque.z,
                ]
            )
            jacobian = self.move_group.get_jacobian_matrix(list(joints))

        else:
            wrench = np.zeros(6)
            jacobian = None
        return {
            "joint_positions": joints,
            "joint_velocities": joints,
            "ee_pos_quat": pos_quat,
            "gripper_position": gripper_pos,
            "ee_wrench": wrench,
            "jacobian": jacobian,
        }


def main():
    rospy.init_node("ros_robot")
    ros_robot = ROSRobot(no_gripper=True)


if __name__ == "__main__":
    main()
