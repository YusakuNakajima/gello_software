import time
from threading import Event, Lock, Thread
from typing import Protocol, Sequence

import numpy as np
from dynamixel_sdk.group_sync_read import GroupSyncRead
from dynamixel_sdk.group_sync_write import GroupSyncWrite
from dynamixel_sdk.packet_handler import PacketHandler
from dynamixel_sdk.port_handler import PortHandler
from dynamixel_sdk.robotis_def import (
    COMM_SUCCESS,
    DXL_HIBYTE,
    DXL_HIWORD,
    DXL_LOBYTE,
    DXL_LOWORD,
)

# Constants
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
LEN_GOAL_POSITION = 4
ADDR_PRESENT_POSITION = 132
ADDR_OPERATING_MODE = 11
ADDR_GOAL_CURRENT = 102
LEN_PRESENT_POSITION = 4
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

CONTROL_MODES = {
    "CURRENT_MODE": 0,
    "VELOCITY_MODE": 1,
    "PWM_MODE": 2,
    "POSITION_MODE": 3,
    "CURRENT_BASED_POSITION_MODE": 5,
}


class DynamixelDriverProtocol(Protocol):
    def set_joints(self, joint_angles: Sequence[float]):
        """Set the joint angles for the Dynamixel servos.

        Args:
            joint_angles (Sequence[float]): A list of joint angles.
        """
        ...

    def torque_enabled(self) -> bool:
        """Check if torque is enabled for the Dynamixel servos.

        Returns:
            bool: True if torque is enabled, False if it is disabled.
        """
        ...

    def set_torque_mode(self, enable: bool):
        """Set the torque mode for the Dynamixel servos.

        Args:
            enable (bool): True to enable torque, False to disable.
        """
        ...

    def get_joints(self) -> np.ndarray:
        """Get the current joint angles in radians.

        Returns:
            np.ndarray: An array of joint angles.
        """
        ...

    def close(self):
        """Close the driver."""


class FakeDynamixelDriver(DynamixelDriverProtocol):
    def __init__(self, ids: Sequence[int]):
        self._ids = ids
        self._joint_angles_read = np.zeros(len(ids), dtype=int)
        self._joint_angles_command = np.zeros(len(ids), dtype=int)
        self._torque_enabled = False

    def set_joints(self, joint_angles: Sequence[float]):
        if len(joint_angles) != len(self._ids):
            raise ValueError(
                "The length of joint_angles must match the number of servos"
            )
        if not self._torque_enabled:
            raise RuntimeError("Torque must be enabled to set joint angles")
        self._joint_angles_read = np.array(joint_angles)

    def torque_enabled(self) -> bool:
        return self._torque_enabled

    def set_torque_mode(self, enable: bool):
        self._torque_enabled = enable

    def get_joints(self) -> np.ndarray:
        return self._joint_angles_read.copy()

    def close(self):
        pass


class DynamixelDriver(DynamixelDriverProtocol):
    def __init__(
        self, ids: Sequence[int], port: str = "/dev/ttyUSB0", baudrate: int = 57600
    ):
        """Initialize the DynamixelDriver class.

        Args:
            ids (Sequence[int]): A list of IDs for the Dynamixel servos.
            port (str): The USB port to connect to the arm.
            baudrate (int): The baudrate for communication.
        """
        self._ids = ids
        self._joint_angles_read = None
        self._joint_angles_command = np.zeros(len(ids), dtype=float)
        self._lock = Lock()

        # Initialize the port handler, packet handler, and group sync read/write
        self._portHandler = PortHandler(port)
        self._packetHandler = PacketHandler(2.0)
        self._groupSyncRead = GroupSyncRead(
            self._portHandler,
            self._packetHandler,
            ADDR_PRESENT_POSITION,
            LEN_PRESENT_POSITION,
        )
        self._groupSyncRead_debug = GroupSyncRead(
            self._portHandler,
            self._packetHandler,
            ADDR_GOAL_POSITION,
            LEN_GOAL_POSITION,
        )
        self._groupSyncWrite = GroupSyncWrite(
            self._portHandler,
            self._packetHandler,
            ADDR_GOAL_POSITION,
            LEN_GOAL_POSITION,
        )

        # Open the port and set the baudrate
        if not self._portHandler.openPort():
            raise RuntimeError("Failed to open the port")

        if not self._portHandler.setBaudRate(baudrate):
            raise RuntimeError(f"Failed to change the baudrate, {baudrate}")

        # Add parameters for each Dynamixel servo to the group sync read
        for dxl_id in self._ids:
            if not self._groupSyncRead.addParam(dxl_id):
                raise RuntimeError(
                    f"Failed to add parameter for Dynamixel with ID {dxl_id}"
                )
            if not self._groupSyncRead_debug.addParam(dxl_id):
                raise RuntimeError(
                    f"Failed to add parameter for Dynamixel with ID {dxl_id}"
                )

        # Disable torque for each Dynamixel servo
        self._torque_enabled = False
        try:
            self.set_torque_mode(self._torque_enabled)
        except Exception as e:
            print(f"port: {port}, {e}")

        self._pause_event = Event()
        self._pause_event.set()
        self._stop_event = Event()
        self._start_thread()

    def pause_thread(self):
        self._pause_event.clear()

    def resume_thread(self):
        self._pause_event.set()

    def torque_enabled(self) -> bool:
        return self._torque_enabled

    def set_torque_mode(self, enable: bool):
        self.pause_thread()
        torque_value = TORQUE_ENABLE if enable else TORQUE_DISABLE
        with self._lock:
            for dxl_id in self._ids:
                dxl_comm_result, dxl_error = self._packetHandler.write1ByteTxRx(
                    self._portHandler, dxl_id, ADDR_TORQUE_ENABLE, torque_value
                )
                if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
                    print(dxl_comm_result)
                    print(dxl_error)
                    raise RuntimeError(
                        f"Failed to set torque mode for Dynamixel with ID {dxl_id}"
                    )
        self._torque_enabled = enable
        self.resume_thread()

    def set_control_mode(self, mode_name: str):
        self.pause_thread()
        mode = CONTROL_MODES[mode_name]
        with self._lock:
            for dxl_id in self._ids:
                dxl_comm_result, dxl_error = self._packetHandler.write1ByteTxRx(
                    self._portHandler, dxl_id, ADDR_OPERATING_MODE, mode
                )
                if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
                    print(dxl_comm_result)
                    print(dxl_error)
                    raise RuntimeError(
                        f"Failed to set control mode for Dynamixel with ID {dxl_id}"
                    )
        self._control_mode = mode
        self.resume_thread()

    def _start_thread(self):
        self._read_and_write_thread = Thread(target=self._read_and_write_joint_angles)
        self._read_and_write_thread.daemon = True
        self._read_and_write_thread.start()

    def _read_and_write_joint_angles(self):
        # Continuously read joint angles and update the joint_angles array
        while not self._stop_event.is_set():
            self._pause_event.wait()

            with self._lock:
                ######### Write the goal position for each Dynamixel servo
                if len(self._joint_angles_command) != len(self._ids):
                    raise ValueError(
                        "The length of joint_angles must match the number of servos"
                    )
                    # if not self._torque_enabled:
                    raise RuntimeError("Torque must be enabled to set joint angles")
                if all(angle == 0 for angle in self._joint_angles_command):
                    pass
                else:
                    # print("self._joint_angles_command", self._joint_angles_command)
                    for dxl_id, angle in zip(self._ids, self._joint_angles_command):
                        # Convert the angle to the appropriate value for the servo
                        position_value = int(angle * 2048 / np.pi)

                        # Allocate goal position value into byte array
                        param_goal_position = [
                            DXL_LOBYTE(DXL_LOWORD(position_value)),
                            DXL_HIBYTE(DXL_LOWORD(position_value)),
                            DXL_LOBYTE(DXL_HIWORD(position_value)),
                            DXL_HIBYTE(DXL_HIWORD(position_value)),
                        ]

                        # Add goal position value to the Syncwrite parameter storage
                        dxl_addparam_result = self._groupSyncWrite.addParam(
                            dxl_id, param_goal_position
                        )
                        if not dxl_addparam_result:
                            raise RuntimeError(
                                f"Failed to set joint angle for Dynamixel with ID {dxl_id}"
                            )

                    # Syncwrite goal position
                    dxl_comm_result = self._groupSyncWrite.txPacket()
                    if dxl_comm_result != COMM_SUCCESS:
                        raise RuntimeError("Failed to syncwrite goal position")

                    # Clear syncwrite parameter storage
                    self._groupSyncWrite.clearParam()
                # time.sleep(0.001)
                ######### Read the joint angles for each Dynamixel servo
                _joint_angles_read = np.zeros(len(self._ids), dtype=int)
                _goal_joint_angles_read = np.zeros(len(self._ids), dtype=int)
                dxl_comm_result = self._groupSyncRead.txRxPacket()
                if dxl_comm_result != COMM_SUCCESS:
                    print(f"warning, comm failed: {dxl_comm_result}")
                    continue
                dxl_comm_result = self._groupSyncRead_debug.txRxPacket()
                if dxl_comm_result != COMM_SUCCESS:
                    print(f"warning, debug comm failed: {dxl_comm_result}")
                    continue
                for i, dxl_id in enumerate(self._ids):
                    if self._groupSyncRead.isAvailable(
                        dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
                    ):
                        angle = self._groupSyncRead.getData(
                            dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
                        )
                        angle = np.int32(np.uint32(angle))
                        _joint_angles_read[i] = angle
                    else:
                        raise RuntimeError(
                            f"Failed to get joint angles for Dynamixel with ID {dxl_id}"
                        )
                    # debug
                    # if self._groupSyncRead_debug.isAvailable(
                    #     dxl_id, ADDR_GOAL_POSITION, LEN_GOAL_POSITION
                    # ):
                    #     angle = self._groupSyncRead_debug.getData(
                    #         dxl_id, ADDR_GOAL_POSITION, LEN_GOAL_POSITION
                    #     )
                    #     angle = np.int32(np.uint32(angle))
                    #     _goal_joint_angles_read[i] = angle
                    # else:
                    #     raise RuntimeError(
                    #         f"Failed to get joint angles for Dynamixel with ID {dxl_id}"
                    #     )
                self._joint_angles_read = _joint_angles_read
                # self._goal_joint_angles_read = _goal_joint_angles_read
                # print(f"joint angles: {self._joint_angles_read}")
                # print(f"goal joint angles: {self._goal_joint_angles_read}")
            # self._groupSyncRead.clearParam() # TODO what does this do? should i add it

    def get_joints(self) -> np.ndarray:
        # Return a copy of the joint_angles array to avoid race conditions
        while self._joint_angles_read is None:
            time.sleep(0.1)
        # with self._lock:
        _j = self._joint_angles_read.copy()
        return _j / 2048.0 * np.pi

    def set_joints(self, joint_angles: Sequence[float]):
        self._joint_angles_command = joint_angles.copy()

    def close(self):
        self._stop_event.set()
        self._pause_event.set()
        self._read_and_write_thread.join()
        self._portHandler.closePort()


def main():
    # Set the port, baudrate, and servo IDs
    ids = [1]

    # Create a DynamixelDriver instance
    try:
        driver = DynamixelDriver(ids)
    except FileNotFoundError:
        driver = DynamixelDriver(ids, port="/dev/cu.usbserial-FT7WBMUB")

    # Test setting torque mode
    driver.set_torque_mode(True)
    driver.set_torque_mode(False)

    # Test reading the joint angles
    try:
        while True:
            joint_angles = driver.get_joints()
            print(f"Joint angles for IDs {ids}: {joint_angles}")
            # print(f"Joint angles for IDs {ids[1]}: {joint_angles[1]}")
    except KeyboardInterrupt:
        driver.close()


if __name__ == "__main__":
    main()  # Test the driver
