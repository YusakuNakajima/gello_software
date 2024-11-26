#!/usr/bin/env python3
import sys
import signal
from dataclasses import dataclass
from multiprocessing import Process

from gello_ros.cameras.realsense_ros import RealSenseROS
from gello_ros.zmq_core.camera_node import ZMQServerCamera

import rospy


def signal_handler(sig, frame):
    print("Exiting...")
    rospy.signal_shutdown("Ctrl+C pressed")
    sys.exit(0)


def launch_server(
    prefix: str,
    host: str,
    port: int,
):
    camera = RealSenseROS(prefix=prefix)
    server = ZMQServerCamera(camera, port=port, host=host)
    print(f"Starting camera server on port {port}")
    server.serve()


def main():
    rospy.init_node("gello_camera_node", anonymous=True)

    camera_host = rospy.get_param("~default_hostname", "127.0.0.1")
    camera_port = rospy.get_param("~default_camera_port", 7001)
    camera_names = rospy.get_param("~camera_names", ["base"])
    camera_servers = []
    for camera_name in camera_names:
        # start a python process for each camera
        print(f"Launching {camera_name} camera on port {camera_port}")
        camera_servers.append(
            Process(
                target=launch_server,
                args=(
                    camera_name,
                    camera_host,
                    camera_port,
                ),
            )
        )
        camera_port += 1

    for server in camera_servers:
        server.start()


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    main()
