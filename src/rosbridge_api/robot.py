"""Robot camera and velocity interface implemented with roslibpy."""

from __future__ import annotations

import base64
import threading
import time
from typing import Optional

import cv2
import numpy as np
import roslibpy


class RosbridgeRobot:
    """Access a robot's velocity command and compressed camera topics.

    Args:
        host: Hostname or IP address of the robot's rosbridge server.
        port: rosbridge WebSocket port.
        cmd_vel_topic: Topic accepting ``geometry_msgs/msg/Twist``.
        image_topic: Topic publishing ``sensor_msgs/msg/CompressedImage``.
        connection_timeout: Maximum seconds to wait while connecting.
    """

    def __init__(
        self,
        host: str,
        port: int = 9090,
        cmd_vel_topic: str = "/cmd_vel",
        image_topic: str = "/camera/image_raw/compressed",
        connection_timeout: float = 10.0,
    ) -> None:
        self.host = host
        self.port = port
        self.cmd_vel_topic = cmd_vel_topic
        self.image_topic = image_topic
        self.connection_timeout = connection_timeout

        self._client: Optional[roslibpy.Ros] = None
        self._cmd_vel: Optional[roslibpy.Topic] = None
        self._joint_state_sub: Optional[roslibpy.Topic] = None
        self._joint_states: Optional[dict] = None
        self._image_sub: Optional[roslibpy.Topic] = None
        self._latest_image: Optional[np.ndarray] = None
        self._image_condition = threading.Condition()
        self._image_sequence = 0
        self._last_query_sequence = 0
        self._is_setup = False

    @property
    def is_connected(self) -> bool:
        """Return whether setup completed and rosbridge remains connected."""
        return bool(
            self._is_setup
            and self._client is not None
            and self._client.is_connected
        )

    def setup(self) -> None:
        """Connect to rosbridge and prepare the command and camera topics."""
        if self._is_setup:
            return

        self._client = roslibpy.Ros(host=self.host, port=self.port)
        try:
            self._client.run(timeout=self.connection_timeout)
            if not self._client.is_connected:
                raise ConnectionError(
                    f"Could not connect to rosbridge at {self.host}:{self.port}"
                )

            self._cmd_vel = roslibpy.Topic(
                self._client, self.cmd_vel_topic, "geometry_msgs/msg/Twist"
            )
            self._cmd_vel.advertise()

            self._image_sub = roslibpy.Topic(
                self._client,
                self.image_topic,
                "sensor_msgs/msg/CompressedImage",
            )
            self._image_sub.subscribe(self._on_image)
            
            self._joint_state_sub = roslibpy.Topic(
                self._client,
                "/joint_states",
                "sensor_msgs/msg/JointState",
            )
            self._joint_state_sub.subscribe(self._on_joint_state)
            
            self._is_setup = True
        except Exception:
            self.cleanup()
            raise

    def send_command(self, linear: float = 0.0, angular: float = 0.0) -> None:
        """Send linear-x (m/s) and angular-z (rad/s) velocity commands."""
        self._require_connection()
        self._publish_command(linear, angular)

    def stop(self) -> None:
        """Send an immediate zero-velocity command."""
        self.send_command(0.0, 0.0)
        
    def query_joint_states(self) -> Optional[dict[str, float]]:
        """Return the latest joint states received from the robot.

        Returns:
            A dictionary containing the joint states, or ``None`` if no joint
            states have been received yet.
        """
        self._require_connection()
        return self._joint_states

    def query_image(
        self, timeout: Optional[float] = None, *, wait_for_new: bool = False
    ) -> Optional[np.ndarray]:
        """Return the latest camera image as a BGR NumPy array copy.

        Args:
            timeout: Seconds to wait for a suitable image. ``None`` waits
                indefinitely and ``0`` returns immediately.
            wait_for_new: Wait for an image received after the preceding query.

        Returns:
            The image, or ``None`` if the timeout expires or cleanup interrupts
            the query.
        """
        self._require_connection()
        if timeout is not None and timeout < 0:
            raise ValueError("timeout must be non-negative or None")

        deadline = None if timeout is None else time.monotonic() + timeout
        with self._image_condition:
            while self._latest_image is None or (
                wait_for_new and self._image_sequence <= self._last_query_sequence
            ):
                remaining = (
                    None if deadline is None else deadline - time.monotonic()
                )
                if remaining is not None and remaining <= 0:
                    return None
                self._image_condition.wait(timeout=remaining)
                if not self.is_connected:
                    return None

            self._last_query_sequence = self._image_sequence
            return self._latest_image.copy()

    def cleanup(self) -> None:
        """Stop the robot, unsubscribe from topics, and disconnect safely."""
        if self._cmd_vel is not None and self._client is not None:
            if self._client.is_connected:
                try:
                    self._publish_command(0.0, 0.0)
                except Exception as exc:
                    print(f"Warning: could not send the stop command: {exc}")
            try:
                self._cmd_vel.unadvertise()
            except Exception:
                pass

        if self._image_sub is not None:
            try:
                self._image_sub.unsubscribe()
            except Exception:
                pass

        if self._client is not None:
            try:
                self._client.terminate()
            except Exception:
                pass

        self._cmd_vel = None
        self._image_sub = None
        self._client = None
        self._is_setup = False
        with self._image_condition:
            self._latest_image = None
            self._image_condition.notify_all()

    def _publish_command(self, linear: float, angular: float) -> None:
        message = roslibpy.Message(
            {
                "linear": {"x": float(linear), "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": float(angular)},
            }
        )
        if self._cmd_vel is None:
            raise RuntimeError("The command topic is not available")
        self._cmd_vel.publish(message)

    def _require_connection(self) -> None:
        if not self.is_connected:
            raise RuntimeError("Robot is not connected. Call setup() first.")
        
    def _on_joint_state(self, message: dict) -> None:
        try:
            self._joint_states = message
        except (KeyError, TypeError, ValueError) as exc:
            print(f"Warning: could not decode joint state message: {exc}")

    def _on_image(self, message: dict) -> None:
        try:
            image_bytes = base64.b64decode(message["data"], validate=True)
            encoded_image = np.frombuffer(image_bytes, dtype=np.uint8)
            image = cv2.imdecode(encoded_image, cv2.IMREAD_COLOR)
            if image is None:
                return
            with self._image_condition:
                self._latest_image = image
                self._image_sequence += 1
                self._image_condition.notify_all()
        except (KeyError, TypeError, ValueError) as exc:
            print(f"Warning: could not decode camera image: {exc}")

    def __enter__(self) -> "RosbridgeRobot":
        self.setup()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.cleanup()

