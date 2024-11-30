import pickle
import threading
from typing import Optional, Tuple

import numpy as np
import zmq

from gello_ros.cameras.camera import CameraDriver

DEFAULT_CAMERA_PORT = 5000


class ZMQClientCamera(CameraDriver):
    """A class representing a ZMQ client for a leader robot."""

    def __init__(self, port: int = DEFAULT_CAMERA_PORT, host: str = "127.0.0.1"):
        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.REQ)
        self._socket.connect(f"tcp://{host}:{port}")

    def read(
        self,
    ) -> Tuple[np.ndarray]:
        """Request the latest camera image."""
        image_data = self._socket.recv()
        image = np.frombuffer(image_data, dtype=np.uint8)

        return image


class ZMQServerCamera:
    def __init__(
        self,
        camera: CameraDriver,
        port: int = DEFAULT_CAMERA_PORT,
        host: str = "127.0.0.1",
    ):
        self._camera = camera
        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.REP)
        addr = f"tcp://{host}:{port}"
        debug_message = f"Camera Sever Binding to {addr}, Camera: {camera}"
        print(debug_message)
        self._timout_message = f"Timeout in Camera Server, Camera: {camera}"
        self._socket.bind(addr)
        self._stop_event = threading.Event()

    def serve(self):
        """Serve the camera images over ZMQ."""
        self._socket.setsockopt(zmq.RCVTIMEO, 1000)  # Set timeout to 1000 ms
        while not self._stop_event.is_set():
            try:
                # Capture the image
                image = self._camera.read()
                self._socket.send(image.tobytes())
            except KeyboardInterrupt:
                print("Shutting down server...")
                break

    def stop(self) -> None:
        """Signal the server to stop serving."""
        self._stop_event.set()
