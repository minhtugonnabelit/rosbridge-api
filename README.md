# Student Rosbridge API

<!-- > **Temporary package name:** `rosbridge_api` is a working name for
> packaging tests. Change both the distribution name in `pyproject.toml` and,
> if desired, the `student_rosbridge` import-package directory before release. -->

A small Python API for students who use an ML model or robotics modelling
toolbox to control a ROS robot through rosbridge. It exposes only the main
operations a student control loop needs:

- connect to the robot;
- query the latest compressed camera image as a BGR NumPy array;
- publish linear and angular velocity to `/cmd_vel`;
- stop and disconnect safely.

## Robot requirements

The robot must:

1. run a rosbridge WebSocket server reachable by the student's computer;
2. accept `geometry_msgs/msg/Twist` messages on `/cmd_vel`;
3. publish `sensor_msgs/msg/CompressedImage` messages on
   `/camera/image_raw/compressed`.

Topic names can be changed when constructing `RosbridgeRobot`.

## Install locally for testing

From this directory:

```bash
python -m pip install .
```

Include the keyboard teleoperation dependency with:

```bash
python -m pip install ".[teleop]"
```

For active development, use an editable installation:

```bash
python -m pip install -e ".[teleop,dev]"
```

## ML/control-loop example

```python
from student_rosbridge import RosbridgeRobot

with RosbridgeRobot("192.168.1.213") as robot:
    while True:
        image = robot.query_image(timeout=2.0, wait_for_new=True)
        if image is None:
            continue

        linear_velocity, angular_velocity = controller(image)
        robot.send_command(linear_velocity, angular_velocity)
```

`query_image()` returns a copy, so student code may safely annotate or resize it.
Its colour order is BGR, as expected by OpenCV.

## Included examples

The runnable examples are kept separately from the library in `examples/`.
After installing the package, run them from the project directory:

```bash
python examples/camera.py 192.168.1.213
python examples/teleop.py 192.168.1.213
```

The camera example exits with `Q` or Escape. The teleop controls are W/S for
forward/backward, A/D for turning, Space for an immediate stop, and Escape to
exit.

## Test and build

```bash
python -m unittest discover -s tests -v
python -m build
python -m twine check --strict dist/*
```

The tests mock rosbridge and do not require a physical robot.

Before publishing, choose the final project name, add author/project metadata,
choose a licence, and verify installation from the generated wheel in a clean
virtual environment.
