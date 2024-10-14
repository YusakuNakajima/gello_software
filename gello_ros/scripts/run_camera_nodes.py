#!/usr/bin/env python3

from dataclasses import dataclass
from multiprocessing import Process

from gello_ros.cameras.realsense_camera import RealSenseCamera, get_device_ids
from gello_ros.zmq_core.camera_node import ZMQServerCamera

import rospy

def launch_server(host: str,port: int, camera_id: int):
    camera = RealSenseCamera(camera_id)
    server = ZMQServerCamera(camera, port=port, host=host)
    print(f"Starting camera server on port {port}")
    server.serve()


def main():
    ids = get_device_ids()
    camera_host = rospy.get_param("~default_hostname", "127.0.0.1")
    camera_port = rospy.get_param("~default_camera_port", 6002)
    camera_servers = []
    for camera_id in ids:
        # start a python process for each camera
        print(f"Launching camera {camera_id} on port {camera_port}")
        camera_servers.append(
            Process(target=launch_server, args=(camera_host,camera_port, camera_id))
        )
        camera_port += 1

    for server in camera_servers:
        server.start()


if __name__ == "__main__":
    main()
