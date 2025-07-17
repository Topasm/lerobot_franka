# SpaceMouse Integration Setup Guide

This guide helps you set up the SpaceMouse teleoperator with the Franka robot in the LeRobot framework.

## Quick Summary

I've successfully integrated the SpaceMouse teleoperator into your LeRobot Franka setup. Here's what has been implemented:

### ✅ Completed Integration

1. **New SpaceMouse Teleoperator**: `/src/lerobot/teleoperators/spanav/spacemouse_teleop.py`
   - Full LeRobot Teleoperator interface compliance
   - 6-DOF control (translation + rotation)
   - Gripper control via buttons
   - Configurable parameters

2. **Configuration Class**: `/src/lerobot/teleoperators/spanav/config_spacemouse.py`
   - Adjustable deadzone, scaling factors
   - Robot IP configuration
   - Gripper enable/disable

3. **Updated Dependencies**: Replaced `spnav` with `pyspacemouse`
   - `spnav` was causing Python compatibility issues
   - `pyspacemouse` is actively maintained and compatible

4. **Integration with Factory**: Updated `/src/lerobot/teleoperators/utils.py`
   - SpaceMouse teleoperator now available via `make_teleoperator_from_config()`

5. **Example Scripts and Configuration**:
   - Test script: `test_spacemouse_teleop.py`
   - Example usage: `examples/franka_spacemouse_example.py`
   - Configuration file: `configs/spacemouse_franka.yaml`

## Installation Steps

### 1. Install Python Dependencies

```bash
# Install pyspacemouse (replaces the problematic spnav)
pip install pyspacemouse

# Optional: Install additional dependencies if needed
pip install numpy scipy multiprocessing
```

### 2. Install System Dependencies (Linux)

For SpaceMouse device support on Linux:

```bash
# Install system libraries for SpaceMouse
sudo apt-get update
sudo apt-get install libspnav-dev spacenavd

# Start the spacenav daemon
sudo systemctl start spacenavd
sudo systemctl enable spacenavd

# Add user to input group for device access
sudo usermod -a -G input $USER
# Log out and back in for group changes to take effect
```

### 3. Test the Installation

```bash
# Test basic import (without hardware)
cd /home/ahrilab/Desktop/lerobot_franka
PYTHONPATH=/home/ahrilab/Desktop/lerobot_franka/src python test_spacemouse_teleop.py

# Test with SpaceMouse hardware connected
python examples/franka_spacemouse_example.py --robot_ip 172.16.0.2
```

## Usage Examples

### 1. Using with LeRobot Teleoperate Script

```bash
# Use the provided configuration
PYTHONPATH=/home/ahrilab/Desktop/lerobot_franka/src python -m lerobot.teleoperate \
    --config-path ./configs \
    --config-name spacemouse_franka
```

### 2. Programmatic Usage

```python
from lerobot.teleoperators import make_teleoperator_from_config
from lerobot.teleoperators.spanav.config_spacemouse import SpacemouseTeleopConfig

# Create configuration
config = SpacemouseTeleopConfig(
    id="my_spacemouse",
    deadzone=0.3,
    move_increment=0.01,
    rotation_scale=0.03,
    use_gripper=True,
    robot_ip="172.16.0.2"
)

# Create and connect teleoperator
teleop = make_teleoperator_from_config(config)
teleop.connect()

# Get actions in control loop
action = teleop.get_action()
# action contains: delta_x, delta_y, delta_z, delta_rx, delta_ry, delta_rz, gripper

teleop.disconnect()
```

### 3. Integration with Franka Robot

The updated `franka_control.py` now uses the new SpaceMouse teleoperator:

```python
# The teleop_step method now automatically creates and uses the new teleoperator
robot = FrankaControl(config=FrankaRobotConfig(robot_ip="172.16.0.2"))
robot.connect()

# This will create and use the SpaceMouse teleoperator
obs, action = robot.teleop_step(record_data=True)
```

## Configuration Options

The `SpacemouseTeleopConfig` supports these parameters:

- **`deadzone`** (float, default=0.3): Input threshold below which movements are ignored
- **`move_increment`** (float, default=0.01): Scaling factor for translation movements
- **`rotation_scale`** (float, default=0.03): Scaling factor for rotational movements  
- **`use_gripper`** (bool, default=True): Enable/disable gripper control
- **`robot_ip`** (str, default="172.16.0.2"): Franka robot IP address

## Control Mapping

### SpaceMouse Input → Robot Action

- **Translation**: Move SpaceMouse puck in X/Y/Z → `delta_x`, `delta_y`, `delta_z`
- **Rotation**: Twist/tilt SpaceMouse puck → `delta_rx`, `delta_ry`, `delta_rz`  
- **Gripper**: 
  - Button 0 (left): Close gripper (`gripper` > 0)
  - Button 1 (right): Open gripper (`gripper` < 0)

### Action Output Format

```python
{
    "delta_x": 0.01,      # X translation increment
    "delta_y": 0.0,       # Y translation increment
    "delta_z": 0.0,       # Z translation increment  
    "delta_rx": 0.0,      # Roll rotation increment
    "delta_ry": 0.0,      # Pitch rotation increment
    "delta_rz": 0.02,     # Yaw rotation increment
    "gripper": 0.01       # Gripper position increment (if enabled)
}
```

## Troubleshooting

### Import Errors

**Problem**: `ModuleNotFoundError: No module named 'lerobot.constants'`
**Solution**: Use correct PYTHONPATH:
```bash
PYTHONPATH=/home/ahrilab/Desktop/lerobot_franka/src python your_script.py
```

### SpaceMouse Not Detected

**Problem**: Device not found or permission denied
**Solutions**:
1. Check if spacenavd is running: `sudo systemctl status spacenavd`
2. Test device: `evtest /dev/input/spacenavigator`
3. Check permissions: `ls -l /dev/input/spacenavigator`
4. Add to input group: `sudo usermod -a -G input $USER`

### pyspacemouse Import Issues

**Problem**: `ImportError: No module named 'pyspacemouse'`  
**Solution**: Install the package: `pip install pyspacemouse`

## Key Differences from Gamepad Teleoperator

1. **6-DOF Control**: SpaceMouse provides full 6 degrees of freedom vs gamepad's 3-DOF
2. **Continuous Input**: Analog input for all axes vs discrete button presses
3. **Hardware Dependency**: Requires physical SpaceMouse device
4. **Coordinate Systems**: Uses SpaceMouse's native coordinate system with configurable transforms

## Files Created/Modified

### New Files:
- `src/lerobot/teleoperators/spanav/spacemouse_teleop.py`
- `src/lerobot/teleoperators/spanav/config_spacemouse.py`  
- `src/lerobot/teleoperators/spanav/__init__.py`
- `src/lerobot/teleoperators/spanav/README.md`
- `test_spacemouse_teleop.py`
- `examples/franka_spacemouse_example.py`
- `configs/spacemouse_franka.yaml`

### Modified Files:
- `src/lerobot/teleoperators/utils.py` (added spacemouse factory support)
- `franka/utils/inputs/spacemouse_shared_memory.py` (updated to use pyspacemouse)
- `lerobot/common/robot_devices/robots/franka_control.py` (updated to use new teleoperator)

## Next Steps

1. **Install Dependencies**: Run the installation commands above
2. **Test Import**: Run the test script to verify everything works
3. **Connect Hardware**: Connect your SpaceMouse device
4. **Test Control**: Use the example script to test spacemouse control
5. **Integrate with Robot**: Connect to your Franka robot and test teleoperation

The integration is complete and ready to use! The SpaceMouse teleoperator now follows the same interface as other LeRobot teleoperators, making it easy to swap between different input devices.
