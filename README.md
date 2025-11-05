![GitHub forks](https://img.shields.io/github/forks/kuriatsu/ros-g29-force-feedback?style=flat&color=%#D78695)   ![GitHub Repo stars](https://img.shields.io/github/stars/kuriatsu/ros-g29-force-feedback?style=flat&color=%23FCCC06)  ![GitHub last commit](https://img.shields.io/github/last-commit/kuriatsu/ros-g29-force-feedback)    ![ros](https://img.shields.io/badge/ROS-Humble-blue)    ![ubuntu](https://img.shields.io/badge/Ubuntu-22.04-purple)

# ros-g29-force-feedback
ROS2 package to control force feedback of logitech g29 steering wheel with ros message, written in c++, for human beings all over the world.
This is useful for the user interface of autonomous driving, driving simulator like [CARLA](https://carla.org/), [LGSVL](https://www.lgsvlsimulator.com/) etc.

`examples/carla_control.py` 

![demo_gif](https://github.com/kuriatsu/ros-g29-force-feedback/blob/image/images/force_feedback_test.gif)

# Features
* Standalone ros package to control steering wheel. (doesn't depend on the other ros packages like ros-melodic-joy etc.)
* Two control modes

    * Control mode (`config/g29.yaml/auto_centering=false`)  
    Rotate wheel to the specified angle (`position`) with specified `torque` as specified with rostopic.

    * Auto centering mode (`config/g29.yaml/auto_centering=true`)  
    Automatically centering position, without publishing rostopic.


* ROS1 and ROS2 support. (if you use ROS1, checkout and refer [ros1 branch](https://github.com/kuriatsu/ros-g29-force-feedback/tree/ros1))
    |ROS version|g29|g923|
    |:--|:--|:--|
    |ROS1|--|--|
    |Kinetic|tested|no|
    |Melodic|tested|no|
    |Noetic|tested|no|
    |ROS2|--|--|
    |Dashing|no|no|
    |Foxy|tested|no|
    |Galactic|tested|no|
    |Humble|no|no|

# Requirement of `master` branch
* ubuntu18-22
* ROS2
* Logitech G29 Driving Force Racing Wheel (Planning to test with g923)

To check whether your kernel supports force feedback, do as follows
```bash
$ cat /boot/config-$(uname -r) | grep CONFIG_LOGIWHEELS_FF
CONFIG_LOGIWHEELS_FF=y
```
This command uses `$(uname -r)` to automatically detect your current kernel version instead of hardcoding it.
If you cannot get `CONFIG_LOGIWHEELS_FF=y`, try to find patch or use latest kernel...

# Hardware Setup and Troubleshooting

## 1. Check if Logitech Module is Loaded
First, verify that the Logitech driver module is being used instead of the generic HID driver:
```bash
$ sudo dmesg -wH
```
Look for messages indicating that the `logitech` driver is being used. If you see `hid-generic` instead, continue with the following steps.

## 2. Ensure G29 is in PS3 Mode
**Important:** The G29 must be in PS3 mode to work properly with Linux. PS4 mode is currently not working on Ubuntu.

Check the mode switch on the back of your wheel:
- The switch should be set to **PS3** (not PS4 or PS5)
- **Check the physical switch carefully:** In some cases, the switch mechanism can be broken internally. If you suspect this, you may need to open the device and verify the PCB is actually making contact in PS3 position. You might need to manually position the switch or repair the connection.

## 3. Install Custom Linux Module (if still using hid-generic)
If your G29 is still being detected as `hid-generic` even in PS3 mode, install the improved custom Linux module:

```bash
$ git clone https://github.com/berarma/new-lg4ff
$ cd new-lg4ff
# Follow the installation instructions in the repository
```

This module provides better support for Logitech racing wheels.

## 4. Debug udev Rules (if needed)
If you're still having issues with the driver not loading correctly, you may need to configure udev rules. Follow the debugging guide from Oversteer:

See: https://github.com/berarma/oversteer

## 5. Verify Device Detection
Once you have the Logitech module loaded, check the event and joystick devices:
```bash
$ cat /proc/bus/input/devices | grep -iA4 logitech
```
This will show you the event device (e.g., `event25`) and joystick device (e.g., `js0`) assigned to your G29.

## 6. Test Force Feedback and Joystick
Before using this ROS package, verify that force feedback and joystick input work correctly:

**Test force feedback:**
```bash
$ fftest /dev/input/eventX
```
Replace `eventX` with your device number from step 5. The test should show "OK" for force feedback capabilities.

**Test joystick input:**
```bash
$ jstest /dev/input/jsY
```
Replace `jsY` with your joystick device number. You should see real-time updates of wheel position, pedals, and buttons.

Once `fftest` shows OK for force feedback, you're ready to use this ROS package.

# Install
1. create ros2_ws
    ```bash
    mkdir -p ros2_ws/src
    cd /ros2_ws
    colcon build
    ```
2. download and build package
    ```bash
    cd /ros2_ws/src
    git clone https://github.com/kuriatsu/ros-g29-force-feedback.git
    cd ../
    colcon build --symlink-install
    ```
    
# Usage
1. Get device name

    If you followed the **Hardware Setup and Troubleshooting** section above, you should already know your event device (e.g., `event25`). If not, find it with:
    ```bash
    $ cat /proc/bus/input/devices | grep -iA4 logitech
    ```
    Look for the **Handlers** line showing the event device (e.g., `event25`).

    Alternatively, you can list all input devices:
    ```bash
    $ cat /proc/bus/input/devices
    ```
    Find **Logitech G29 Driving Force Racing Wheel** and check Handlers (ex. event19)

2. Change `device_name` in `config/g29.yaml` to the event device you obtained in step 1

    Update the line `device_name: "/dev/input/eventX"` where X is your event number (e.g., `/dev/input/event25`)

3. Launch ros node
    ```bash
    $ source ros2_ws/install/setup.bash
    $ ros2 run ros_g29_force_feedback g29_force_feedback --ros-args --params-file ros2_ws/src/ros_g29_force_feedback/config/g29.yaml 
    ```

1. Publish message (It's better to use tab completion)  
    ```bash
    $ ros2 topic pub /ff_target ros_g29_force_feedback/msg/ForceFeedback "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, position: 0.3, torque: 0.5}"
    ```
    Once the message is published, the wheel rotates to 0.3*<max_angle> (g29: max_angle=450° clockwise, -450° counterclockwise).
    Publish rate is not restricted.
    
# Parameter

**g29_force_feedback.yaml**
|parameter|default|description|
|:--|:--|:--|
|device_name|/dev/input/event19|device name, change the number|
|loop_rate|0.1|Loop of retrieving wheel position and uploading control to the wheel|
|max_torque|1.0|As for g29, 1.0 = 2.5Nm (min_torque < max_torque < 1.0)|
|min_torque|0.2|Less than 0.2 cannot rotate wheel|
|brake_torque|0.2|Braking torque to stop at the position (descrived below)|
|brake_position|0.1|Brake angle (`position`-0.1*max_angle)|
|auto_centering_max_torque|0.3|Max torque for auto centering|
|auto_centering_max_position|0.2|Max torque position while auto centering (`position`±0.2*max_angle)|
|eps|0.01|Wheel in the range (position-eps to position+eps) is considered as it has reached the `position`|
|auto_centering|false|Anto centering if true|

![ros_g29_ff](https://user-images.githubusercontent.com/38074802/167057448-1fa21956-ae91-4e51-bee4-1fcdc05cae51.png)


# Troubleshoot
### No module named 'ros_g29_force_feedback' when run carla_control.py
* If you use conda, deactivate and try again. rclpy and conda seem to conflict.
* If you have multiple ros2 versions, source one of them or remove one of them.

### Cannot open the device
Double check the output of `cat /proc/bus/input/devices` and update `device_name` in config/g29.yaml.
Reboot can change the ID.

### config.yaml is not refrected to ros
Try to update the parameters inside the source code.


# Author

[kuriatsu](https://github.com/kuriatsu)

# Change Log

## 2023-03-19
Bag fix in ros1 branch thanks to [pedrohdsimoes](https://github.com/pedrohdsimoes)

## 2022-11-6
### examples added
An example script of CARLA connection was added. Rotate the wheel according to the ego vehicle spawned in CARLA as shown in the GIF image.

## 2022-04-21

### ROS2-Foxy integration
Now available in ROS2-Foxy thanks to [JLBicho](https://github.com/JLBicho)

## 2021-11-03
### Huge Improvement !!! 
Oscillation problem solved.
Wheel stops at the specified position, then starts auto centering.
Auto centering mode are set by rosparam in config/g29.yaml, not by rostopic.
Name of topic variables changed.

## 2020-10-10
### changed
PID-Constant mode can be changed dynamically!!
Removed mode selection from rosparam.
Rotation force is ignored when PID mode. (max force can be specified with rosparam (not dynamic))

# Reference
https://www.kernel.org/doc/html/v5.4/input/ff.html  
https://github.com/flosse/linuxconsole/tree/master
