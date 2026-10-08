"""Control robot velocity from the keyboard using the public package API."""

import argparse
import threading
import time

from rosbridge_api import RosbridgeRobot


def teleop_example(robot: RosbridgeRobot, publish_rate: float = 10.0) -> None:
    """Control the robot using W/S/A/D, Space to stop, and Escape to exit."""
    try:
        from pynput import keyboard
    except ImportError as exc:
        raise RuntimeError(
            'Teleop requires: pip install "rosbridge-api[teleop]"'
        ) from exc

    if publish_rate <= 0:
        raise ValueError("publish_rate must be greater than zero")

    command_lock = threading.Lock()
    command = {"linear": 0.0, "angular": 0.0}

    def on_press(key) -> None:
        should_stop = False
        with command_lock:
            try:
                if key.char == "w":
                    command["linear"] = 0.1
                elif key.char == "s":
                    command["linear"] = -0.1
                elif key.char == "a":
                    command["angular"] = 0.8
                elif key.char == "d":
                    command["angular"] = -0.8
            except AttributeError:
                if key == keyboard.Key.space:
                    command["linear"] = 0.0
                    command["angular"] = 0.0
                    should_stop = True
        if should_stop:
            robot.stop()

    def on_release(key):
        with command_lock:
            try:
                if key.char in ("w", "s"):
                    command["linear"] = 0.0
                elif key.char in ("a", "d"):
                    command["angular"] = 0.0
            except AttributeError:
                pass
        if key == keyboard.Key.esc:
            return False
        return None

    print("W/S: forward/backward | A/D: turn | Space: stop | Escape: quit")
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()
    try:
        while listener.running and robot.is_connected:
            with command_lock:
                linear = command["linear"]
                angular = command["angular"]
            robot.send_command(linear, angular)
            time.sleep(1.0 / publish_rate)
    finally:
        listener.stop()
        listener.join()
        if robot.is_connected:
            robot.stop()


def main() -> None:
    """Connect to a robot and run the keyboard teleop example."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("host", help="Robot hostname or IP address")
    parser.add_argument("--port", type=int, default=9090)
    parser.add_argument("--rate", type=float, default=10.0, help="Publish rate")
    args = parser.parse_args()

    with RosbridgeRobot(args.host, port=args.port) as robot:
        teleop_example(robot, publish_rate=args.rate)


if __name__ == "__main__":
    main()

