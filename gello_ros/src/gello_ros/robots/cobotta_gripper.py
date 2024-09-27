#!/usr/bin/env python3
"""Module to control Cobotta gripper"""

import socket
import threading
import time
from enum import Enum
from typing import OrderedDict, Tuple, Union

import sys

sys.path.append("pybcapclient")
import pybcapclient.bcapclient as bcapclient


class CobottaGripper:
    """Communicates with the gripper directly, via socket with string commands, leveraging string names for variables."""

    def __init__(self) -> None:
        self.socket = None
        self.command_lock = threading.Lock()
        self._min_position = 10
        self._max_position = 30
        self._min_speed = 0
        self._max_speed = 100
        self._min_force = 0
        self._max_force = 10

        self.COMP = 1

    def connect(self, host="192.168.56.11", port=5007, timeout=2000):
        self.m_bcapclient = bcapclient.BCAPClient(host, port, timeout)
        ### start b_cap Service
        self.m_bcapclient.service_start("")
        ### Connect to RC8 (RC8(VRC)provider)
        Name = ""
        Provider = "CaoProv.DENSO.VRC"
        Machine = "localhost"
        Option = ""
        hCtrl = self.m_bcapclient.controller_connect(Name, Provider, Machine, Option)
        self.hCtrl = hCtrl
        ### get Robot Object Handl
        HRobot = self.m_bcapclient.controller_getrobot(hCtrl, "Arm", "")
        self.HRobot = HRobot

    # def is_active(self):
    #     """Returns whether the gripper is active."""
    #     status = self._get_var(self.STA)
    #     return (
    #         RobotiqGripper.GripperStatus(status) == RobotiqGripper.GripperStatus.ACTIVE
    #     )

    # def get_min_position(self) -> int:
    #     """Returns the minimum position the gripper can reach (open position)."""
    #     return self._min_position

    # def get_max_position(self) -> int:
    #     """Returns the maximum position the gripper can reach (closed position)."""
    #     return self._max_position

    # def get_open_position(self) -> int:
    #     """Returns what is considered the open position for gripper (minimum position value)."""
    #     return self.get_min_position()

    # def get_closed_position(self) -> int:
    #     """Returns what is considered the closed position for gripper (maximum position value)."""
    #     return self.get_max_position()

    # def is_open(self):
    #     """Returns whether the current position is considered as being fully open."""
    #     return self.get_current_position() <= self.get_open_position()

    # def is_closed(self):
    #     """Returns whether the current position is considered as being fully closed."""
    #     return self.get_current_position() >= self.get_closed_position()

    # def get_current_position(self) -> int:
    #     """Returns the current position as returned by the physical hardware."""
    #     return self._get_var(self.POS)

    def move_position(self, position: float, speed: int) -> Tuple[bool, int]:
        """Sends commands to start moving towards the given position, with the specified speed and force.

        :param position: Position to move to [min_position, max_position]
        :param speed: Speed to move at [min_speed, max_speed]
        :param force: Force to use [min_force, max_force]
        :return: A tuple with a bool indicating whether the action it was successfully sent, and an integer with
        the actual position that was requested, after being adjusted to the min/max calibrated range.
        """
        position = float(position)
        speed = int(speed)

        def clip_val(min_val, val, max_val):
            return max(min_val, min(val, max_val))

        clip_pos = clip_val(self._min_position, position, self._max_position)
        clip_spe = clip_val(self._min_speed, speed, self._max_speed)

        param = [clip_pos, clip_spe]
        succ = self.m_bcapclient.controller_execute(self.hCtrl, "HandMoveA", param)

        return succ, clip_pos

    def move_force(self, force: float) -> Tuple[bool, int]:
        """Sends commands to start moving towards the given position, with the specified speed and force.

        :param position: Position to move to [min_position, max_position]
        :param speed: Speed to move at [min_speed, max_speed]
        :param force: Force to use [min_force, max_force]
        :return: A tuple with a bool indicating whether the action it was successfully sent, and an integer with
        the actual position that was requested, after being adjusted to the min/max calibrated range.
        """
        force = float(force)

        def clip_val(min_val, val, max_val):
            return max(min_val, min(val, max_val))

        clip_for = clip_val(self._min_force, force, self._max_force)

        param = [clip_for, True, "DetectOn"]
        succ = self.m_bcapclient.controller_execute(self.hCtrl, "HandMoveH", param)

        return succ, clip_for


def main():
    # test open and closing the gripper
    gripper = CobottaGripper()
    gripper.connect(host="192.168.56.11")
    gripper.move_position(10, 10)
    time.sleep(0.2)
    gripper.move_position(20, 20)
    time.sleep(0.2)
    gripper.move_position(30, 30)


if __name__ == "__main__":
    main()
