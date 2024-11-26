import os
import time
from typing import List, Optional, Tuple
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
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

        self._color_image = np.zeros((480, 640, 3), dtype=np.uint8)
        self._depth_image = np.zeros((480, 640), dtype=np.uint16)
        if prefix is None:
            prefix = ""
        else:
            prefix = prefix + "_"  # Add underscore to the prefix

        # ROS Subscribers
        rospy.Subscriber(
            "/" + prefix + "camera/color/image_raw/compressed",
            CompressedImage,
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

    def _color_callback(self, msg: CompressedImage):
        """Callback for the color image."""
        np_arr = np.frombuffer(msg.data, np.uint8)
        color_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if self._flip:
            color_image = cv2.rotate(color_image, cv2.ROTATE_180)
        self._color_image = color_image

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
        color_image = self._color_image
        depth_image = self._depth_image
        if img_size is None:
            image = color_image[:, :, ::-1]
            depth = depth_image
        else:
            image = cv2.resize(color_image, img_size)[:, :, ::-1]
            depth = cv2.resize(depth_image, img_size)

        # rotate 180 degree's because everything is upside down in order to center the camera
        if self._flip:
            image = cv2.rotate(image, cv2.ROTATE_180)
            depth = cv2.rotate(depth, cv2.ROTATE_180)[:, :, None]
        else:
            depth = depth[:, :, None]

        return image, depth


def _debug_read(camera, save_datastream=False):
    import cv2

    cv2.namedWindow("image")
    cv2.namedWindow("depth")
    counter = 0
    if not os.path.exists("images"):
        os.makedirs("images")
    if save_datastream and not os.path.exists("stream"):
        os.makedirs("stream")
    while not rospy.is_shutdown():
        time.sleep(0.1)
        image, depth = camera.read()
        if image is None or depth is None:
            continue
        depth_vis = cv2.applyColorMap(
            cv2.convertScaleAbs(depth, alpha=0.03), cv2.COLORMAP_JET
        )

        key = cv2.waitKey(1)
        cv2.imshow("image", image)
        cv2.imshow("depth", depth_vis)
        if key == ord("s"):
            cv2.imwrite(f"images/image_{counter}.png", image)
            cv2.imwrite(f"images/depth_{counter}.png", depth_vis)
        if save_datastream:
            cv2.imwrite(f"stream/image_{counter}.png", image)
            cv2.imwrite(f"stream/depth_{counter}.png", depth_vis)
        counter += 1
        if key == 27:  # ESC to exit
            break


if __name__ == "__main__":
    device_ids = get_device_ids()
    print(f"Found {len(device_ids)} devices")
    print(device_ids)

    rs_camera = RealSenseCameraROS(flip=True)
    _debug_read(rs_camera, save_datastream=True)
