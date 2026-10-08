import time
import numpy as np
from pynput import keyboard
import roboticstoolbox as rtb
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
import onRobot.gripper as gripper

print("Loading UR3e URDF model from Robotics Toolbox...")
robot = rtb.models.URDF.UR3()  # using UR3 model for this example

ROBOT_IP = "192.168.0.191" 
HAVE_GRIPPER = True  # Set to True if you have the RG2 gripper connected

def setup_RTDE() -> tuple[RTDEControl, RTDEReceive]:
    """Sets up RTDE control and receive interfaces."""
    try:
        print("Connecting to robot via ur_rtde...")
        rtde_c = RTDEControl(ROBOT_IP)
        rtde_r = RTDEReceive(ROBOT_IP)
        if not rtde_c.isConnected() or not rtde_r.isConnected():
            raise ConnectionError("Failed to connect to the robot via RTDE.")
    except Exception as e:
        print(f"Failed to connect to the robot via RTDE: {e}")
        exit()  
        return None, None

    return rtde_c, rtde_r

def setup_gripper() -> gripper.RG:
    """Sets up the RG2 gripper using the onRobot library."""
    print("Connecting to RG2 gripper via onRobot library...")
    try:
        rg2 = gripper.RG(ROBOT_IP, 0)
        initial_width = rg2.get_rg_width()
        print(f"RG2 Gripper connected successfully! Current width: {initial_width} mm")
    except Exception as e:
        print(f"Failed to connect to RG2 gripper: {e}")
        rtde_c.disconnect()
        rtde_r.disconnect()
        exit()
        return None
    
    return rg2

# Connect to the robot via ur_rtde
print("Connecting to robot via ur_rtde...")
rtde_c, rtde_r = setup_RTDE()

# Connect to the RG2 gripper using the onRobot library
print("Connecting to RG2 gripper via onRobot library...")
rg2 = setup_gripper() if HAVE_GRIPPER else None

print("\n--- Teleop Controls ---")
print("Linear  - W/S: Y+/Y- | A/D: X-/X+ | Q/E: Z+/Z-")
print("Angular - R/F: Roll  | T/G: Pitch | Y/H: Yaw")
print("Gripper - Z: Close (10mm) | X: Open (80mm)")
print("ESC     - Exit program\n")

# Speed and control configurations
LINEAR_SPEED = 0.05    # meters per second
ANGULAR_SPEED = 0.5    # radians per second
ACCELERATION = 1.5     # joint acceleration limit
DAMPING_FACTOR = 0.05  # DLS damping factor

pressed_keys = set()

def on_press(key):
    try:
        if key.char in ['w', 's', 'a', 'd', 'q', 'e', 'r', 'f', 't', 'g', 'y', 'h', 'z', 'x']:
            if key.char == 'z' and rg2 is not None:
                print("Commanding Gripper: CLOSE (10 mm)")
                try:
                    rg2.rg_grip(10.0, 20.0)
                except Exception as e:
                    print(f"Gripper error: {e}")
            elif key.char == 'x' and rg2 is not None:
                print("Commanding Gripper: OPEN (80 mm)")
                try:
                    rg2.rg_grip(80.0, 20.0)
                except Exception as e:
                    print(f"Gripper error: {e}")
            else:
                pressed_keys.add(key.char)
    except AttributeError:
        if key == keyboard.Key.esc:
            return False

def on_release(key):
    try:
        if key.char in pressed_keys:
            pressed_keys.remove(key.char)
    except AttributeError:
        pass

# Start keyboard listener thread
listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

def qdot_from_cartesian_vel(cartesian_vel, q_current) -> np.ndarray: 
    """Compute joint velocities from Cartesian velocities using DLS pseudoinverse."""
    J = robot.jacob0(q_current)
    m, n = J.shape
    lam_sq = DAMPING_FACTOR ** 2
    
    if m <= n:
        J_damped_pinv = J.T @ np.linalg.inv(J @ J.T + lam_sq * np.eye(m))
    else:
        J_damped_pinv = np.linalg.inv(J.T @ J + lam_sq * np.eye(n)) @ J.T

    return J_damped_pinv @ cartesian_vel

try:
    while listener.is_alive():
        # 1. Initialize 6-DOF Cartesian velocity vector [vx, vy, vz, wx, wy, wz]
        vx, vy, vz = 0.0, 0.0, 0.0
        wx, wy, wz = 0.0, 0.0, 0.0
        
        # Translation mapping
        if 'w' in pressed_keys: vy += LINEAR_SPEED
        if 's' in pressed_keys: vy -= LINEAR_SPEED
        if 'd' in pressed_keys: vx += LINEAR_SPEED
        if 'a' in pressed_keys: vx -= LINEAR_SPEED
        if 'q' in pressed_keys: vz += LINEAR_SPEED
        if 'e' in pressed_keys: vz -= LINEAR_SPEED
        
        # Orientation mapping
        if 'r' in pressed_keys: wx += ANGULAR_SPEED
        if 'f' in pressed_keys: wx -= ANGULAR_SPEED
        if 't' in pressed_keys: wy += ANGULAR_SPEED
        if 'g' in pressed_keys: wy -= ANGULAR_SPEED
        if 'y' in pressed_keys: wz += ANGULAR_SPEED
        if 'h' in pressed_keys: wz -= ANGULAR_SPEED

        cartesian_vel = np.array([vx, vy, vz, wx, wy, wz])
        
        # 2. Get current actual joint positions from the UR3e
        q_current = rtde_r.getActualQ()
        
        # 3. Compute joint velocities from Cartesian velocities using DLS pseudoinverse
        q_dot = qdot_from_cartesian_vel(cartesian_vel, q_current)

        # 4. Command joint velocities via speedJ
        rtde_c.speedJ(q_dot.tolist(), ACCELERATION, 0.02)

        # Maintain ~125 Hz loop rate
        time.sleep(0.008)

except KeyboardInterrupt:
    pass

finally:
    print("\nStopping robot and cleaning up...")
    rtde_c.speedStop()
    rtde_c.disconnect()
    rtde_r.disconnect()
    listener.stop()
    print("Disconnected.")