# CubeMars Read-Only Quick Start

Run this on the Raspberry Pi 4 with the AK70-10 connected to JC1:

```bash
cd ~/ros2_ws
colcon build --packages-select cubemars_hardware rover_description
source install/setup.bash
sudo -E ros2 launch rover_description cubemars_read_only_test.launch.py
```

In another terminal:

```bash
source ~/ros2_ws/install/setup.bash
ros2 control list_hardware_interfaces
ros2 topic echo /joint_states
```

The motor must be configured for servo mode, 48 V operation, 1 Mbps CAN,
extended CAN, and CAN status transmission.
