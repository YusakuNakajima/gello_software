#!/usr/bin/env python3
import os
import signal
import sys
import datetime
import argparse
import glob
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional, Tuple, List

import numpy as np
from policy_config import (
    POLICY_CONFIG,
    TASK_CONFIG,
    TRAIN_CONFIG,
)  # must import first
from gello_ros.agents.agent import BimanualAgent, DummyAgent
from gello_ros.agents.gello_agent import GelloAgent
from gello_ros.agents.act_agent import ACTAgent
from gello_ros.data_utils.save_episode import save_episode
from gello_ros.data_utils.keyboard_interface import KBReset
from gello_ros.env import RobotEnv
from gello_ros.robots.robot import PrintRobot
from gello_ros.zmq_core.robot_node import ZMQClientRobot
from gello_ros.zmq_core.camera_node import ZMQClientCamera
from gello_ros.policy.utils import *
import rospy
from geometry_msgs.msg import Wrench

agent = None


def signal_handler(sig, frame):
    print("Exiting...")
    if agent is not None:
        agent.set_torque_mode(False)
    rospy.signal_shutdown("Ctrl+C pressed")
    sys.exit(0)


def print_color(*args, color=None, attrs=(), **kwargs):
    import termcolor

    if len(args) > 0:
        args = tuple(termcolor.colored(arg, color=color, attrs=attrs) for arg in args)
    print(*args, **kwargs)


def main():
    rospy.init_node("gello_agent_node", anonymous=True)

    agent_name: str = rospy.get_param("~agent_name", "gello")
    hostname: str = rospy.get_param("~default_hostname", "127.0.0.1")
    robot_port: int = rospy.get_param("~default_robot_port", 6001)
    camera_port: int = rospy.get_param("~default_camera_port", 7001)
    camera_names: List[str] = rospy.get_param("~camera_names", ["base"])

    robot_type: str = None  # only needed for quest agent or spacemouse agent
    hz: int = rospy.get_param("~control_hz", 100)
    start_joints: List[float] = rospy.get_param("~gello_start_joints")
    gello_mode: str = rospy.get_param("~gello_mode")

    print(f"start_joints: {start_joints}")

    mock: bool = False
    use_save_interface: bool = rospy.get_param("~save_episode", False)
    verbose: bool = False
    gello_port: str = rospy.get_param("~gello_port", None)
    number_of_episodes: int = rospy.get_param("~number_of_episodes", 1)
    number_of_steps: int = rospy.get_param("~number_of_steps", 1000)
    task_name: str = rospy.get_param("~task_name", "cup_push")

    if use_save_interface:
        kb_interface = KBReset()

    if mock:
        robot_client = PrintRobot(8, dont_print=True)
        camera_clients = {}
    else:
        camera_clients = {}
        for i, cam_name in enumerate(camera_names):
            camera_clients[cam_name] = ZMQClientCamera(
                port=camera_port + i, host=hostname
            )
        robot_client = ZMQClientRobot(port=robot_port, host=hostname)
    env = RobotEnv(robot_client, control_rate_hz=hz, camera_dict=camera_clients)

    if agent_name == "gello":
        print("Using Gello agent")
        gello_port = gello_port
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

        gello_reset_joints = np.array(start_joints)
        agent = GelloAgent(
            port=gello_port, start_joints=gello_reset_joints, mode=gello_mode
        )
        time.sleep(1)

        # Start the gello agent

        obs = env.get_obs()
        robot_joints = obs["joint_positions"]
        print(f"Robot joints: {robot_joints}")

        gello_curr_joints = np.array(env.get_obs()["joint_positions"])

        if gello_reset_joints.shape == gello_curr_joints.shape:
            max_delta = (np.abs(gello_curr_joints - gello_reset_joints)).max()
            steps = min(int(max_delta / 0.01), 100)
            for jnt in np.linspace(gello_curr_joints, gello_reset_joints, steps):
                env.step(jnt)
                time.sleep(0.001)

        # preprocess the agent start position
        agent_start_pos = agent.act(env.get_obs())
        print(f"Gello agent start pos: {agent_start_pos}")

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
        startup_iterations = 100
        for i in range(startup_iterations):
            obs = env.get_obs()
            command_joints = agent.act(obs)
            current_joints = obs["joint_positions"]
            delta = command_joints - current_joints
            max_joint_delta = np.abs(delta).max()
            if max_joint_delta > max_delta:
                delta = delta / max_joint_delta * max_delta
            env.step(current_joints + delta)
            # print("start up iteration", i, "/", startup_iterations)

        # check if the joints are close
        obs = env.get_obs()
        joints = obs["joint_positions"]
        action = agent.act(obs)
        if (action - joints > 0.5).any():
            print("Action is too big")
            print("action", action)
            print("jpints", joints)

            # print which joints are too big
            joint_index = np.where(action - joints > 0.8)
            for j in joint_index:
                print(
                    f"Joint [{j}], leader: {action[j]}, follower: {joints[j]}, diff: {action[j] - joints[j]}"
                )
            exit()

    elif agent_name == "quest":
        from gello_ros.agents.quest_agent import SingleArmQuestAgent

        agent = SingleArmQuestAgent(robot_type=robot_type, which_hand="l")
    elif agent_name == "spacemouse":
        from gello_ros.agents.spacemouse_agent import SpacemouseAgent

        agent = SpacemouseAgent(robot_type=robot_type, verbose=verbose)
    elif agent_name == "dummy" or agent_name == "none":
        agent = DummyAgent(num_dofs=robot_client.num_dofs())
    elif agent_name == "act":
        # load config
        cfg = TASK_CONFIG
        policy_config = POLICY_CONFIG
        train_cfg = TRAIN_CONFIG
        device = os.environ["DEVICE"]
        # load the policy
        ckpt_path = os.path.join(
            train_cfg["checkpoint_dir"], task_name, train_cfg["eval_ckpt_name"]
        )
        policy = make_policy(policy_config["policy_class"], policy_config)
        print("Loading checkpoint")
        print(ckpt_path)
        loading_status = policy.load_state_dict(
            torch.load(ckpt_path, map_location=torch.device(device))
        )
        print(loading_status)
        policy.to(device)
        policy.eval()
        print("ACT policy loaded")
        agent = ACTAgent(
            policy, camera_names, train_cfg, policy_config, task_cfg=cfg, device=device
        )
        obs = env.get_obs()
    elif agent_name == "policy":
        raise NotImplementedError("add your imitation policy here if there is one")
    else:
        raise ValueError("Invalid agent name")

    print_color("\nStart 🚀🚀🚀", color="green", attrs=("bold",))

    save_path = None
    start_time = time.time()
    obs_replay = []
    action_replay = []
    current_episode_number = 0
    while True:
        # st_per_step = time.time()
        if use_save_interface:
            start_time = time.time()
            # init buffers
            obs_replay = []
            action_replay = []
            state = kb_interface.update()  # update requires up to approximately 3ms
            if state == "start":
                if (current_episode_number + 1) > number_of_episodes:
                    print("All episodes done")
                    exit()
                for i in range(number_of_steps):
                    num = time.time() - start_time
                    message = f"\r Episode number: {current_episode_number} Time passed: {round(num, 2)}          "
                    print_color(
                        message,
                        color="white",
                        attrs=("bold",),
                        end="",
                        flush=True,
                    )
                    action = agent.act(obs)
                    obs = env.step(action)
                    action_replay.append(action)
                    obs_replay.append(obs)

                print("Episode done, saving now")
                save_episode(current_episode_number, obs_replay, action_replay)
                current_episode_number += 1
            elif state == "normal":
                action = agent.act(obs)
                obs = env.step(action)
            elif state == "quit":
                print("Quite episode recording")
                exit()
            else:
                raise ValueError(f"Invalid state {state}")
        elif agent_name == "act":
            for t in range(number_of_steps):
                num = time.time() - start_time
                message = f"\rTime passed: {round(num, 2)} Step: {t}   "
                print_color(
                    message,
                    color="white",
                    attrs=("bold",),
                    end="",
                    flush=True,
                )

                action = agent.act(obs, t)
                obs = env.step(action)
        else:
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
            obs = env.step(action)

        # print("Time per step", time.time() - st_per_step)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    main()
