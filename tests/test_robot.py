"""Tests for the public robot API without a physical robot."""

import base64
import threading
import time
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from rosbridge_api import RosbridgeRobot
from rosbridge_api import robot as robot_module


class FakeRos:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.is_connected = False
        self.terminated = False

    def run(self, timeout=10):
        self.is_connected = True

    def terminate(self):
        self.is_connected = False
        self.terminated = True


class FakeTopic:
    instances = []

    def __init__(self, client, name, message_type):
        self.client = client
        self.name = name
        self.message_type = message_type
        self.messages = []
        self.callback = None
        self.advertised = False
        self.unsubscribed = False
        self.__class__.instances.append(self)

    def advertise(self):
        self.advertised = True

    def unadvertise(self):
        self.advertised = False

    def publish(self, message):
        self.messages.append(message)

    def subscribe(self, callback):
        self.callback = callback

    def unsubscribe(self):
        self.unsubscribed = True


class RosbridgeRobotTest(unittest.TestCase):
    def setUp(self):
        FakeTopic.instances.clear()
        self.ros_patch = patch.object(robot_module.roslibpy, "Ros", FakeRos)
        self.topic_patch = patch.object(robot_module.roslibpy, "Topic", FakeTopic)
        self.ros_patch.start()
        self.topic_patch.start()
        self.addCleanup(self.ros_patch.stop)
        self.addCleanup(self.topic_patch.stop)

        self.robot = RosbridgeRobot("robot.local")
        self.robot.setup()
        self.addCleanup(self.robot.cleanup)

    def topic(self, name):
        return next(topic for topic in FakeTopic.instances if topic.name == name)

    def test_setup_and_send_command(self):
        self.assertTrue(self.robot.is_connected)
        self.robot.send_command(0.2, -0.4)

        message = self.topic("/cmd_vel").messages[-1]
        self.assertEqual(message["linear"]["x"], 0.2)
        self.assertEqual(message["angular"]["z"], -0.4)

    def test_query_decodes_compressed_image_and_returns_copy(self):
        source = np.full((3, 4, 3), 127, dtype=np.uint8)
        success, encoded = cv2.imencode(".png", source)
        self.assertTrue(success)
        payload = base64.b64encode(encoded).decode("ascii")

        self.topic("/camera/image_raw/compressed").callback({"data": payload})
        first = self.robot.query_image(timeout=0)
        second = self.robot.query_image(timeout=0)

        np.testing.assert_array_equal(first, source)
        self.assertIsNot(first, second)

    def test_query_timeout(self):
        self.assertIsNone(self.robot.query_image(timeout=0.01))

    def test_cleanup_stops_robot_and_wakes_query(self):
        result = []
        query_thread = threading.Thread(
            target=lambda: result.append(self.robot.query_image())
        )
        query_thread.start()
        time.sleep(0.02)

        self.robot.cleanup()
        query_thread.join(timeout=1.0)

        self.assertFalse(query_thread.is_alive())
        self.assertEqual(result, [None])
        self.assertEqual(self.topic("/cmd_vel").messages[-1]["linear"]["x"], 0.0)
        self.assertFalse(self.robot.is_connected)

    def test_methods_require_setup(self):
        self.robot.cleanup()
        with self.assertRaisesRegex(RuntimeError, "setup"):
            self.robot.send_command(0.1, 0.0)

    def test_negative_timeout_is_rejected(self):
        with self.assertRaises(ValueError):
            self.robot.query_image(timeout=-1)


if __name__ == "__main__":
    unittest.main()

