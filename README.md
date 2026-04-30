:floppy_disk: `tof-tmf8829-micro-ros`
=====================================

Arduino sketch for interfacing a TMF8829 ToF sensor with ROS.

### How-to-compile/upload
```bash
git clone https://github.com/lxrobotics/tof-tmf8829-micro-ros
arduino-cli compile --fqbn arduino:renesas_uno:minima tof-tmf8829-micro-ros -u -p /dev/ttyACM0
```

### How-to-run micro-ROS agent
```bash
docker run -it --rm --net=host --device=/dev/ttyACM0 microros/micro-ros-agent:jazzy serial --dev /dev/ttyACM0
```
