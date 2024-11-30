#!/usr/bin/env python3
import sys
import signal
from dataclasses import dataclass
from multiprocessing import Process

from gello_ros.cameras.realsense_camera import RealSenseCamera, get_device_ids
from gello_ros.zmq_core.camera_node import ZMQServerCamera

import rospy


def signal_handler(sig, frame):
    print("Exiting...")
    rospy.signal_shutdown("Ctrl+C pressed")
    sys.exit(0)


def launch_server(
    host: str,
    port: int,
    camera_id: int,
    height: int = 480,
    width: int = 640,
    fps: int = 60,
):
    camera = RealSenseCamera(device_id=camera_id, height=height, width=width, fps=fps)
    server = ZMQServerCamera(camera, port=port, host=host)
    print(f"Starting camera server on port {port}")
    server.serve()


def main():
    rospy.init_node("gello_camera_node", anonymous=True)

    ids = get_device_ids()
    camera_host = rospy.get_param("~default_hostname", "127.0.0.1")
    camera_port = rospy.get_param("~default_camera_port", 7001)
    height = rospy.get_param("~camera_height", 480)
    width = rospy.get_param("~camera_width", 640)
    fps = rospy.get_param("~camera_fps", 60)
    camera_servers = []
    for camera_id in ids:
        # start a python process for each camera
        print(f"Launching camera {camera_id} on port {camera_port}")
        camera_servers.append(
            Process(
                target=launch_server,
                args=(camera_host, camera_port, camera_id, height, width, fps),
            )
        )
        camera_port += 1

    for server in camera_servers:
        server.start()


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    main()
