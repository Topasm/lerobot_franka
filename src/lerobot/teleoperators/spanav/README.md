# SpaceMouse Teleoperator for Franka Robot

This directory contains the SpaceMouse teleoperator implementation for controlling Franka robots through the LeRobot framework.

## Overview

The SpaceMouse teleoperator provides 6-DOF pose control (position + orientation) plus gripper control for the Franka robot. It reads input from a 3Dconnexion SpaceMouse device and translates it into robot commands compatible with the LeRobot framework.

## Features

- **6-DOF Control**: Full 6 degrees of freedom control (X, Y, Z translation + Roll, Pitch, Yaw rotation)
- **Gripper Control**: Two-button gripper control (open/close)
- **Configurable Parameters**: Adjustable deadzone, scaling factors, and increments
- **LeRobot Integration**: Fully compatible with LeRobot's teleoperator interface
- **Real-time Control**: Low-latency control suitable for teleoperation

## Files

- `spacemouse_teleop.py`: Main teleoperator implementation
- `config_spacemouse.py`: Configuration class for SpaceMouse parameters
- `__init__.py`: Module initialization and exports

## Prerequisites

### Hardware
- 3Dconnexion SpaceMouse device (wireless or wired)
- Franka Emika Panda robot

### Software Dependencies
- `spnav`: SpaceNavigator library for SpaceMouse support
- `panda_py`: Franka robot control library
- `multiprocessing`: For shared memory communication
- `numpy`: For numerical operations

### Installation

1. Install system dependencies for SpaceMouse:
   ```bash
   # Ubuntu/Debian
   sudo apt-get install spacenavd libspnav-dev
   
   # Start the spacenav daemon
   sudo systemctl start spacenavd
   sudo systemctl enable spacenavd
   ```

2. Install Python dependencies:
   ```bash
   pip install spnav panda_py numpy
   ```

## Configuration

The SpaceMouse teleoperator can be configured using the `SpacemouseTeleopConfig` class:

```python
from lerobot.teleoperators.spanav import SpacemouseTeleopConfig

config = SpacemouseTeleopConfig(
    id="my_spacemouse",
    deadzone=0.3,              # Deadzone for input filtering (0-1)
    move_increment=0.01,       # Translation scaling factor
    rotation_scale=0.03,       # Rotation scaling factor  
    use_gripper=True,          # Enable gripper control
    robot_ip="172.16.0.2"     # Franka robot IP address
)
```

### Parameters

- **deadzone** (float): Input values below this threshold are ignored (range: 0-1)
- **move_increment** (float): Scaling factor for translation movements
- **rotation_scale** (float): Scaling factor for rotational movements
- **use_gripper** (bool): Whether to include gripper control in actions
- **robot_ip** (str): IP address of the Franka robot

## Usage

### Basic Usage

```python
from lerobot.teleoperators import make_teleoperator_from_config
from lerobot.teleoperators.spanav import SpacemouseTeleopConfig

# Create configuration
config = SpacemouseTeleopConfig(id="spacemouse")

# Create teleoperator
teleop = make_teleoperator_from_config(config)

# Connect and use
teleop.connect()
action = teleop.get_action()
teleop.disconnect()
```

### With LeRobot Teleoperation Script

Use the main LeRobot teleoperation script with the provided configuration:

```bash
python -m lerobot.teleoperate \
    --config-path ./configs \
    --config-name spacemouse_franka
```

### Standalone Example

Run the example script:

```bash
cd examples
python franka_spacemouse_example.py --robot_ip 172.16.0.2
```

## Controls

### SpaceMouse Input Mapping

- **Translation**: Move the SpaceMouse puck in X/Y/Z directions
  - X-axis: Left/Right movement
  - Y-axis: Forward/Backward movement  
  - Z-axis: Up/Down movement

- **Rotation**: Twist/tilt the SpaceMouse puck
  - Roll: Twist around X-axis
  - Pitch: Tilt around Y-axis
  - Yaw: Twist around Z-axis

- **Gripper Control**: 
  - Button 0: Close gripper
  - Button 1: Open gripper

### Action Output Format

The teleoperator outputs actions in the following format:

```python
{
    "delta_x": 0.01,     # X translation increment
    "delta_y": 0.0,      # Y translation increment  
    "delta_z": 0.0,      # Z translation increment
    "delta_rx": 0.0,     # Roll rotation increment
    "delta_ry": 0.0,     # Pitch rotation increment
    "delta_rz": 0.02,    # Yaw rotation increment
    "gripper": 0.01      # Gripper position increment (if enabled)
}
```

## Integration with Franka Robot

The SpaceMouse teleoperator is designed to work with the existing Franka robot integration in LeRobot. The action outputs can be directly used with the Franka robot's control interface.

### Example Integration

```python
# Configure teleoperator
teleop_config = SpacemouseTeleopConfig(id="spacemouse")
teleop = make_teleoperator_from_config(teleop_config)

# Configure robot (example - adjust based on your setup)
from lerobot.robots import make_robot_from_config, RobotConfig
robot_config = RobotConfig(type="franka", robot_ip="172.16.0.2")
robot = make_robot_from_config(robot_config)

# Connect both
teleop.connect()
robot.connect()

# Teleoperation loop
try:
    while True:
        action = teleop.get_action()
        robot.send_action(action)
        time.sleep(1/30)  # 30 Hz control loop
finally:
    teleop.disconnect()
    robot.disconnect()
```

## Troubleshooting

### SpaceMouse Not Detected

1. Check if spacenavd is running:
   ```bash
   sudo systemctl status spacenavd
   ```

2. Test SpaceMouse connection:
   ```bash
   # Should show device events when moving SpaceMouse
   evtest /dev/input/spacenavigator
   ```

3. Check permissions:
   ```bash
   # Add user to input group
   sudo usermod -a -G input $USER
   # Log out and back in
   ```

### Connection Issues

1. Verify robot IP address is correct
2. Check network connectivity to robot
3. Ensure robot is powered on and accessible

### Performance Issues

1. Adjust deadzone if inputs are too sensitive
2. Reduce scaling factors for finer control
3. Check system load and process priorities

## Development

### Adding New Features

To extend the SpaceMouse teleoperator:

1. Modify `SpacemouseTeleopConfig` to add new parameters
2. Update `SpacemouseTeleop.get_action()` to include new functionality
3. Update `action_features` property to reflect new action space

### Testing

Run the test script to verify functionality:

```bash
python test_spacemouse_teleop.py
```

This will test the teleoperator without requiring a connected robot.

## Related Files

- `lerobot/common/robot_devices/robots/franka_control.py`: Franka robot integration
- `franka/utils/inputs/spacemouse_shared_memory.py`: Low-level SpaceMouse interface
- `examples/franka_spacemouse_example.py`: Complete usage example
