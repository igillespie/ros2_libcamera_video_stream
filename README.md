# ROS 2 Libcamera Video Stream

This project provides a ROS 2 package for capturing video streams using `libcamera` and publishing them as ROS 2 topics. It was tested using ROS2 Jazzy installed on a Raspberry Pi 5.

---

## Features

- Captures video using `libcamera-vid`.
- Publishes both raw (`sensor_msgs/msg/Image`) and compressed (`sensor_msgs/msg/CompressedImage`) frames.


---

## Installation

### Setup
```bash
git clone https://github.com/igillespie/ros2_libcamera_video_stream.git
cd ros2_libcamera_video_stream

pip install  opencv-python cv-bridge

source /opt/ros/<ros-distro>/setup.bash  # Replace <ros-distro> with your ROS 2 distribution (e.g., humble, rolling, etc.)
colcon build
source install/setup.bash

ros2 run video_stream video_publisher
```
Or if you want to pass specific arguments:

```bash
ros2 run video_stream video_publisher --ros-args -p codec:=mjpeg -p width:=640 -p height:=480 -p fps:=15
```
## Topics

The node publishes video frames on the following topics:

| **Topic**                | **Message Type**             | **Description**               |
|---------------------------|------------------------------|--------------------------------|
| `/video_frames`           | `sensor_msgs/msg/Image`      | Publishes raw video frames with uncompressed pixel data for processing. |
| `/video_frames/compressed`| `sensor_msgs/msg/CompressedImage` | Publishes compressed video frames (JPEG) for bandwidth-efficient streaming. |

