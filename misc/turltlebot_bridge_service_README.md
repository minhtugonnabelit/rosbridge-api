# TurtleBot rosbridge systemd service

Run the installer from any directory. By default, it configures the account that
invoked `sudo`; pass a username explicitly when installing as another user.

```bash
./misc/turtlebot_bridge_install.sh
# Or:
./misc/turtlebot_bridge_install.sh ubuntu
```

The ROS distribution defaults to Humble and can be overridden when needed:

```bash
ROS_DISTRO=jazzy ./misc/turtlebot_bridge_install.sh ubuntu
```

The installed service starts TurtleBot bringup, the camera node, and rosbridge at
boot. Its bound script is installed as `/home/<robot-user>/start_robot.sh`. It
uses the workspace overlay at
`/home/<robot-user>/turtlebot3_ws/install/setup.bash` when that file exists.

Common operations:

```bash
# Inspect service state and recent logs.
sudo systemctl status robot_bringup.service
sudo journalctl -u robot_bringup.service -n 100 --no-pager

# Restart after changing robot configuration.
sudo systemctl restart robot_bringup.service

# Stop the service for the current boot.
sudo systemctl stop robot_bringup.service

# Prevent it from starting at boot.
sudo systemctl disable robot_bringup.service
```
