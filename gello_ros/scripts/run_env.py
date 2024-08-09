#!/usr/bin/env python3
import datetime
import glob
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional, Tuple, List

import numpy as np
import tyro

from gello_ros.agents.agent import BimanualAgent, DummyAgent
from gello_ros.agents.gello_agent import GelloAgent
from gello_ros.data_utils.format_obs import save_frame
from gello_ros.env import RobotEnv
from gello_ros.robots.robot import PrintRobot
from gello_ros.zmq_core.robot_node import ZMQClientRobot

import rospy


def print_color(*args, color=None, attrs=(), **kwargs):
    import termcolor

    if len(args) > 0:
        args = tuple(termcolor.colored(arg, color=color, attrs=attrs) for arg in args)
    print(*args, **kwargs)


@dataclass
class Args:
    agent: str = "gello"
    robot_port: int = 6001
    wrist_camera_port: int = 5000
    base_camera_port: int = 5001
    hostname: str = "127.0.0.1"
    robot_type: str = None  # only needed for quest agent or spacemouse agent
    hz: int = 100
    start_joints: List[float] = field(
        # default_factory=lambda: np.deg2rad([0, -90, 90, -90, -90, 0, 0]).tolist() # UR
        # default_factory=lambda: np.deg2rad([0, 0, 90, 0, 90, 0, 0]).tolist() # Cobotta
        default_factory=lambda: np.deg2rad([-90, -90, -90, -90, 90, 0, 0]).tolist() # FR3
    )

    gello_port: Optional[str] = None
    mock: bool = False
    use_save_interface: bool = False
    data_dir: str = "~/bc_data"
    verbose: bool = False
    no_gripper: bool = True


def main(args):
    rospy.init_node("gello_env", anonymous=True)
    if args.mock:
        robot_client = PrintRobot(8, dont_print=True)
        camera_clients = {}
    else:
        camera_clients = {
            # you can optionally add camera nodes here for imitation learning purposes
            # "wrist": ZMQClientCamera(port=args.wrist_camera_port, host=args.hostname),
            # "base": ZMQClientCamera(port=args.base_camera_port, host=args.hostname),
        }
        robot_client = ZMQClientRobot(port=args.robot_port, host=args.hostname)
    env = RobotEnv(robot_client, control_rate_hz=args.hz, camera_dict=camera_clients)

    if args.agent == "gello":
        print("Using Gello agent")
        gello_port = args.gello_port
        if gello_port is None:
            usb_ports = glob.glob("/dev/serial/by-id/*")
            print(f"Found {len(usb_ports)} ports")
            if len(usb_ports) > 0:
                gello_port = usb_ports[0]
                print(f"using port {gello_port}")
            else:
                raise ValueError(
                    "No gello port found, please specify one or plug in gello"
                )

        gello_reset_joints = np.array(args.start_joints)
        agent = GelloAgent(port=gello_port, start_joints=gello_reset_joints)
        gello_curr_joints = np.array(env.get_obs()["joint_positions"])

        if gello_reset_joints.shape == gello_curr_joints.shape:
            max_delta = (np.abs(gello_curr_joints - gello_reset_joints)).max()
            steps = min(int(max_delta / 0.01), 100)
            for jnt in np.linspace(gello_curr_joints, gello_reset_joints, steps):
                env.step(jnt)
                time.sleep(0.001)

    elif args.agent == "quest":
        from gello_ros.agents.quest_agent import SingleArmQuestAgent

        agent = SingleArmQuestAgent(robot_type=args.robot_type, which_hand="l")
    elif args.agent == "spacemouse":
        from gello_ros.agents.spacemouse_agent import SpacemouseAgent

        agent = SpacemouseAgent(robot_type=args.robot_type, verbose=args.verbose)
    elif args.agent == "dummy" or args.agent == "none":
        agent = DummyAgent(num_dofs=robot_client.num_dofs())
    elif args.agent == "policy":
        raise NotImplementedError("add your imitation policy here if there is one")
    else:
        raise ValueError("Invalid agent name")

    # going to start position
    print("Going to start position")
    agent_start_pos = agent.act(env.get_obs())
    if args.no_gripper:
        agent_start_pos = agent_start_pos[0:-1]
    obs = env.get_obs()
    robot_joints = obs["joint_positions"]

    # check if the joints are close
    abs_deltas = np.abs(agent_start_pos - robot_joints)
    id_max_joint_delta = np.argmax(abs_deltas)
    print(f"Agent start pos: {agent_start_pos}", f"Robot joints: {robot_joints}")
    max_joint_delta = 0.8
    if abs_deltas[id_max_joint_delta] > max_joint_delta:
        print("Joint deltas are too big, please check the following joints")

        id_mask = abs_deltas > max_joint_delta
        ids = np.arange(len(id_mask))[id_mask]
        for i, delta, joint, current_j in zip(
            ids,
            abs_deltas[id_mask],
            agent_start_pos[id_mask],
            robot_joints[id_mask],
        ):
            print(
                f"joint[{i}]: \t delta: {delta:4.3f} , leader: \t{joint:4.3f} , follower: \t{current_j:4.3f}"
            )
        return

    assert len(agent_start_pos) == len(
        robot_joints
    ), f"agent output dim = {len(agent_start_pos)}, but env dim = {len(robot_joints)}"

    # soft startup
    max_delta = 0.003
    for _ in range(100):
        obs = env.get_obs()
        command_joints = agent.act(obs)
        current_joints = obs["joint_positions"]
        if args.no_gripper:
            command_joints = command_joints[0:-1]
        delta = command_joints - current_joints
        max_joint_delta = np.abs(delta).max()
        if max_joint_delta > max_delta:
            delta = delta / max_joint_delta * max_delta
        env.step(current_joints + delta)

    # check if the joints are close
    obs = env.get_obs()
    joints = obs["joint_positions"]
    action = agent.act(obs)
    if args.no_gripper:
        action = action[0:-1]
    if (action - joints > 0.5).any():
        print("Action is too big")

        # print which joints are too big
        joint_index = np.where(action - joints > 0.8)
        for j in joint_index:
            print(
                f"Joint [{j}], leader: {action[j]}, follower: {joints[j]}, diff: {action[j] - joints[j]}"
            )
        exit()

    if args.use_save_interface:
        from gello_ros.data_utils.keyboard_interface import KBReset

        kb_interface = KBReset()

    print_color("\nStart 🚀🚀🚀", color="green", attrs=("bold",))

    save_path = None
    start_time = time.time()
    while True:
        num = time.time() - start_time
        message = f"\rTime passed: {round(num, 2)}          "
        print_color(
            message,
            color="white",
            attrs=("bold",),
            end="",
            flush=True,
        )
        action = agent.act(obs)
        if args.no_gripper:
            action = action[0:-1]
        dt = datetime.datetime.now()
        if args.use_save_interface:
            state = kb_interface.update()
            if state == "start":
                dt_time = datetime.datetime.now()
                save_path = (
                    Path(args.data_dir).expanduser()
                    / args.agent
                    / dt_time.strftime("%m%d_%H%M%S")
                )
                save_path.mkdir(parents=True, exist_ok=True)
                print(f"Saving to {save_path}")
            elif state == "save":
                assert save_path is not None, "something went wrong"
                save_frame(save_path, dt, obs, action)
            elif state == "normal":
                save_path = None
            else:
                raise ValueError(f"Invalid state {state}")
        obs = env.step(action)


if __name__ == "__main__":
    main(tyro.cli(Args))
