"""Visualise the robot's laser scan using the public rosbridge_api package."""

import argparse
import math

import cv2
import numpy as np

from rosbridge_api import LaserScan, RosbridgeRobot


def render_laser_scan(
    scan: LaserScan, image_size: int = 700, display_range: float | None = None
) -> np.ndarray:
    """Render a top-down laser scan as a BGR OpenCV image."""
    if image_size < 100:
        raise ValueError("image_size must be at least 100 pixels")

    valid = np.isfinite(scan.ranges)
    valid &= scan.ranges >= scan.range_min
    valid &= scan.ranges <= scan.range_max

    if display_range is None:
        display_range = scan.range_max
        if not math.isfinite(display_range) and np.any(valid):
            display_range = float(np.max(scan.ranges[valid]))
    if (
        display_range is None
        or not math.isfinite(display_range)
        or display_range <= 0
    ):
        display_range = 1.0

    canvas = np.zeros((image_size, image_size, 3), dtype=np.uint8)
    centre = image_size // 2
    scale = (image_size * 0.45) / display_range

    for fraction in (0.25, 0.5, 0.75, 1.0):
        radius = round(display_range * fraction * scale)
        cv2.circle(canvas, (centre, centre), radius, (50, 50, 50), 1)

    visible = valid & (scan.ranges <= display_range)
    ranges = scan.ranges[visible]
    angles = scan.angles[visible]
    # ROS: +x is forward and +y is left. Display forward at the top.
    pixel_x = np.rint(centre - ranges * np.sin(angles) * scale).astype(int)
    pixel_y = np.rint(centre - ranges * np.cos(angles) * scale).astype(int)
    inside = (
        (pixel_x >= 0)
        & (pixel_x < image_size)
        & (pixel_y >= 0)
        & (pixel_y < image_size)
    )
    canvas[pixel_y[inside], pixel_x[inside]] = (0, 255, 255)

    cv2.circle(canvas, (centre, centre), 6, (0, 255, 0), -1)
    cv2.arrowedLine(
        canvas,
        (centre, centre),
        (centre, centre - 40),
        (0, 255, 0),
        2,
        tipLength=0.25,
    )
    cv2.putText(
        canvas,
        f"range: {display_range:.1f} m | points: {int(np.count_nonzero(visible))}",
        (12, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (220, 220, 220),
        1,
        cv2.LINE_AA,
    )
    return canvas


def laser_scan_example(
    robot: RosbridgeRobot, image_size: int = 700, display_range: float | None = None
) -> None:
    """Display live laser scans until Q or Escape is pressed."""
    print("Laser scan active. Press Q or Escape in the window to quit.")
    try:
        while robot.is_connected:
            scan = robot.query_laser_scan(timeout=1.0, wait_for_new=True)
            if scan is None:
                continue
            image = render_laser_scan(scan, image_size, display_range)
            cv2.imshow("Robot Laser Scan", image)
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break
    finally:
        cv2.destroyAllWindows()


def main() -> None:
    """Connect to a robot and run the laser-scan example."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("host", help="Robot hostname or IP address")
    parser.add_argument("--port", type=int, default=9090)
    parser.add_argument("--topic", default="/scan", help="LaserScan topic")
    parser.add_argument("--size", type=int, default=700, help="Window size")
    parser.add_argument(
        "--range",
        dest="display_range",
        type=float,
        default=None,
        help="Maximum displayed range in metres",
    )
    args = parser.parse_args()

    with RosbridgeRobot(
        args.host, port=args.port, laser_scan_topic=args.topic
    ) as robot:
        laser_scan_example(robot, args.size, args.display_range)


if __name__ == "__main__":
    main()
