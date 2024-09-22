#!/usr/bin/env python3
import rospy
import moveit_commander
import numpy as np
import sys
import time


class DynamixelCurrentController:
    def __init__(
        self,
        joint_torques,
        torque_rate,
        stall_torque,
        stall_current,
        current_goal_constant,
    ):
        # 初期パラメータを保存
        self.joint_torques = joint_torques
        self.torque_rate = torque_rate
        self.stall_torque = stall_torque
        self.stall_current = stall_current
        self.current_goal_constant = current_goal_constant

        # トルク定数を事前計算
        self.torque_constant = self.stall_torque / self.stall_current

    def calculate_dynamixel_currents(self):
        # Dynamixelのカレントを計算
        dynamixel_currents = (
            self.joint_torques * self.torque_rate / self.torque_constant
        )
        return dynamixel_currents

    def calculate_dynamixel_current_goals(self, dynamixel_currents):
        # カレントゴール値を計算
        dynamixel_current_goals = dynamixel_currents / self.current_goal_constant
        return dynamixel_current_goals

    def display_current_goals(self):
        # Dynamixelのカレントゴールを整数で出力
        dynamixel_currents = self.calculate_dynamixel_currents()
        dynamixel_current_goals = self.calculate_dynamixel_current_goals(
            dynamixel_currents
        )

        # カレントを整数に丸める
        dynamixel_currents_int = np.round(dynamixel_currents).astype(int)
        dynamixel_current_goals_int = np.round(dynamixel_current_goals).astype(int)

        # 結果を表示
        print("Dynamixel currents (int):")
        print(dynamixel_currents_int)
        print("Dynamixel current goal values (int):")
        print(dynamixel_current_goals_int)
        print("Required total current (absolute sum of currents):")
        print(np.sum(np.abs(dynamixel_currents_int)))


def main():
    rospy.init_node("jacobian_calculator", anonymous=True)

    # moveit_commanderの初期化
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    move_group = moveit_commander.MoveGroupCommander("manipulator")

    # 関節値とJacobian計算
    joint_values = move_group.get_current_joint_values()
    jacobian = move_group.get_jacobian_matrix(joint_values)
    jacobian_inv = np.linalg.pinv(jacobian)

    # 例の力/トルク値
    force_torque_values = np.array([0, 0, 3, 0, 0, 0])
    joint_torques = np.dot(jacobian_inv, force_torque_values)

    # パラメータの取得
    stall_torque = rospy.get_param("~stall_torque", 0.52)
    stall_current = rospy.get_param("~stall_current", 1.5)
    torque_rate = rospy.get_param("~torque_rate", [0.01, 0.01, 0.01, 0.01, 0.01, 0.01])
    current_goal_constant = rospy.get_param("~current_goal_constant", 0.001)

    # DynamixelCurrentControllerのインスタンス化
    controller = DynamixelCurrentController(
        joint_torques,
        torque_rate,
        stall_torque,
        stall_current,
        current_goal_constant,
    )

    # Dynamixelのカレントゴールを表示
    controller.display_current_goals()

    # moveit_commanderのシャットダウン
    moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
