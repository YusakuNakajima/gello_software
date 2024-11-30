import os
import time
from typing import List, Optional, Tuple
import numpy as np
import rospy
from sensor_msgs.msg import Image, CompressedImage
import cv2


def get_device_ids() -> List[str]:
    import pyrealsense2 as rs

    ctx = rs.context()
    devices = ctx.query_devices()
    device_ids = []
    for dev in devices:
        dev.hardware_reset()
        device_ids.append(dev.get_info(rs.camera_info.serial_number))
    time.sleep(2)
    return device_ids


class RealSenseROS:

    def __init__(
        self,
        prefix: Optional[str] = None,
        flip: bool = False,
    ):
        self._flip = flip
        height = rospy.get_param("~camera_height", 480)
        width = rospy.get_param("~camera_width", 640)

        self._empty_color_image = np.zeros((height, width, 3), dtype=np.uint8)
        self._empty_depth_image = np.zeros((height, width), dtype=np.uint16)
        self._color_image = None
        self._depth_image = None

        if prefix is None:
            prefix = ""
        else:
            prefix = prefix + "_"  # Add underscore to the prefix

        # ROS Subscribers
        rospy.Subscriber(
            "/" + prefix + "camera/color/image_raw",
            Image,
            self._color_callback,
            queue_size=1,
        )
        # rospy.Subscriber(
        #     "/" + prefix + "/depth/image_raw/compressed",
        #     CompressedImage,
        #     self._depth_callback,
        #     queue_size=1,
        # )
        print("Subscribers initialized.")

    def _color_callback(self, msg: Image):
        """Callback for the color image."""
        try:
            self._color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            if self._flip:
                self._color_image = cv2.rotate(self._color_image, cv2.ROTATE_180)
        except Exception as e:
            rospy.logerr(f"Failed to convert image: {e}")

    def _depth_callback(self, msg: CompressedImage):
        """Callback for the depth image."""
        np_arr = np.frombuffer(msg.data, np.uint8)
        depth_image = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
        if self._flip:
            depth_image = cv2.rotate(depth_image, cv2.ROTATE_180)
        self._depth_image = depth_image

    def read(
        self,
        img_size: Optional[Tuple[int, int]] = None,
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Get the latest color and depth images."""
        if self._color_image is None:
            return self._empty_color_image, self._empty_depth_image
        return self._color_image, self._empty_depth_image


if __name__ == "__main__":
    device_ids = get_device_ids()
    print(f"Found {len(device_ids)} devices")
    print(device_ids)

    rs_camera = RealSenseCameraROS(flip=True)
    _debug_read(rs_camera, save_datastream=True)
