<a href="https://lxrobotics.com/"><img align="right" src="https://raw.githubusercontent.com/lxrobotics/.github/main/logo/lxrobotics.png" width="15%"></a>
:floppy_disk: `tof-tmf8829-micro-ros`
=====================================

Arduino sketch for interfacing a TMF8829 ToF sensor with ROS.

### Required hardware
* 1 x Arduino [Uno R4 Minima](https://docs.arduino.cc/hardware/uno-r4-minima/)
* 1 x AMS [TMF8829_EVM_EB_SHIELD](https://ams-osram.com/de/products/boards-kits-accessories/kits/ams-tmf8829-evm-eb-shield-evaluation-kit)

### How-to-compile/upload
```bash
git clone https://github.com/lxrobotics/tof-tmf8829-micro-ros
arduino-cli compile --fqbn arduino:renesas_uno:minima tof-tmf8829-micro-ros -u -p /dev/ttyACM0
```

### How-to-run micro-ROS agent
```bash
docker run -it --rm --net=host --device=/dev/ttyACM0 microros/micro-ros-agent:jazzy serial --dev /dev/ttyACM0
```

### How-to-visualize using Rviz2
```bash
source /opt/ros/jazzy/setup.bash
ros2 run rviz2 rviz2 -d tof-tmf8829-micro-ros/config/tmf8829.rviz
```

