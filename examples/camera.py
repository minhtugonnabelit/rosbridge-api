"""Display the robot camera using the public student_rosbridge API."""

import argparse

import cv2

from rosbridge_api import RosbridgeRobot


def camera_example(robot: RosbridgeRobot) -> None:
    """Display camera images until Q or Escape is pressed."""
    print("Camera active. Press Q or Escape in the camera window to quit.")
    try:
        while robot.is_connected:
            image = robot.query_image(timeout=1.0, wait_for_new=True)
            if image is None:
                continue
            cv2.imshow("Robot Camera Feed", image)
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break
    finally:
        cv2.destroyAllWindows()


def main() -> None:
    """Connect to a robot and run the camera example."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("host", help="Robot hostname or IP address")
    parser.add_argument("--port", type=int, default=9090)
    args = parser.parse_args()

    with RosbridgeRobot(args.host, port=args.port) as robot:
        camera_example(robot)


if __name__ == "__main__":
    main()

