# cubemars_hardware

This package contains a [ros2_control](https://control.ros.org/master/index.html) hardware interface to control the CubeMars AK series motors with a `SystemInterface`. It was tested on two AK70-10.

## Motor Setup
The package assumes that the motor is setup in servo mode.

To adjust the motor settings you connect the motor with the R-link and use the "Upper Computer" program. More information is available on the CubeMars [Technical Support and Download page](https://www.cubemars.com/article.php?id=261).

Make sure that you enable "Send status over CAN" (called `send_can_status` in AppParams). Otherwise the state interfaces will not work properly. Also, the upload frequency of the actuators should be at least as fast as the update rate of the controller manager. If no CAN message is received during one update loop you will get a warning in the `read` function.

Example motor settings can be found in [our main repo](https://github.com/OpenFieldAutomation-OFA/ros-weed-control/tree/main/.motor_params/cubemars).

## mjbots pi3hat
The package uses the mjbots pi3hat C++ library instead of SocketCAN. The
vendored pi3hat submodule is the Johns Hopkins fork, whose flashed firmware
uses extended CAN frames for all CAN identifiers. Configure the CubeMars
motors for servo mode, 1 Mbit/s CAN, and enabled CAN status responses.

The `can_channel` parameter selects the pi3hat connector:

| Connector | Channel |
|-----------|---------|
| JC1       | 1       |
| JC2       | 2       |
| JC3       | 3       |
| JC4       | 4       |
| JC5       | 5       |

The pi3hat driver accesses Raspberry Pi hardware directly and normally
requires root privileges. Build and run the hardware interface on the
Raspberry Pi.

## Hardware Interface
The following command interfaces are published:
- `position`: Position(-Speed) Loop Mode
- `velocity`: Speed Loop Mode
- `effort`: Current Loop Mode

Your `ros2_control` controller can only claim one of the command interfaces. Claiming multiple command interfaces at the same time is not possible.

The following state interfaces are published:
- `position`
- `velocity`
- `effort`
- `temperature`

The hardware interfaces can also be listed by starting the controller manager and running the following command.
```
ros2 control list_hardware_interfaces
```

## `ros2_control` Parameters
An example `ros2_control` URDF config with this hardware interface can be found in [our main repo](https://github.com/OpenFieldAutomation-OFA/ros-weed-control/blob/main/ofa_moveit_config/ros2_control/ofa_robot.ros2_control.xacro).

`hardware` tag:
- `can_channel`: pi3hat CAN channel, 1 through 5

`joint` tag:
- `can_id`: CAN ID of the actuator
- `pole_pairs`: Pole pairs. Used for unit conversion
- `gear_ratio`: Gear ratio. Used for unit conversion
- `kt`: Torque constant. Used to convert current to torque
- `trq_limit`: OPTIONAL. Torque limit. If the limit is reached the motor is disabled and the hardware interface is shutdown.
- `max_current`: OPTIONAL. Maximum commanded motor current in amperes. Defaults to 18 A.
- `max_torque`: OPTIONAL. Maximum commanded/output torque in Nm. Defaults to 18 Nm.
- `max_velocity`: OPTIONAL. Maximum commanded joint velocity in rad/s. Defaults to 300 rpm (approximately 31.4 rad/s). Position-only mode cannot enforce this limit; use both `vel_limit` and `acc_limit` to select position-speed mode.
- `enc_off`: OPTIONAL. Encoder offset in $\text{rad}$. (see explanation below)
- `vel_limit`: OPTIONAL. Velocity limit in $\text{rad}/\text{s}$. (see explanation below)
- `acc_limit`: OPTIONAL. Acceleration limit in $\text{rad}/\text{s}^2$. (see explanation below)
- `read_only`: OPTIONAL. If set to 1, the current position is logged and no commands are sent to the motors.

### Encoder Offset
For single-encoder motors there might be an offset between your desired origin and the zero position of the motor on startup. You can compensate for this offset by using `enc_off`. Note that for this to work properly, the motor has to be close to the origin on startup, otherwise the encoder value will wrap around.

Models with an output encoder can remember a permanent zero position. It can be set using the following command (replace `XX` with the hexadecimal representation of the CAN ID).
```
cansend can0 000005XX#01
```

### Velocity and Acceleration Limits
The `position` command interface will by default use the Position Mode (servo mode 4) where the motor runs to the specified position at maximum speed and acceleration. If you want to use the Position-Speed Loop Mode (servo mode 6) you have to specify BOTH `vel_limit` and `acc_limit`. This will limit the maximum acceleration and velocity of the motor (trajectory planning). This does not work well together with a `joint_trajectory_controller`.

Note that these limits have no effect if only one is set or if you don't use the `position` command interface.
