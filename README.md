# pcl_obstacle_avoidance
It is a obstacle avoidance package based on PCL and realsense. This package is developing with UbiquityRobot Magni.

## Prerequisites
- ROS Melodic

- Ubiquity Robots Rasberry PI Image 2020-11-07-ubiquity-xenial-lxde 

- Install jsk_pcl (optional but recommanded)

```
sudo apt-get install ros-melodic-jsk-pcl-ros
sudo apt-get install ros-melodic-jsk-rviz-plugins
sudo apt-get install ros-melodic-rgbd-launch
sudo apt-get install ros-melodic-ros-numpy
pip install python_pcl-0.3.0rc1-cp27-cp27mu-linux_x86_64
```

- Install realsense-ros
```
sudo apt install ros-$ROS_DISTRO-librealsense2 ros-$ROS_DISTRO-realsense2-camera ros-$ROS_DISTRO-realsense2-description
wget https://github.com/IntelRealSense/librealsense/raw/master/config/99-realsense-libusb.rules
sudo cp 99-realsense-libusb.rules /etc/udev/rules.d/
```

- Setup Raspberry Pi in UbiquityRobot as ROS master
- Setup a computer for this repository and act as a ROS node
** Please refer to https://github.com/laitathei/Gazebo-rosserial-rescue-robot/tree/main/yolov4-tiny.

## Startup
- Execute the launch file
```
source deve/setup.bash
roslaunch startup system_start.launch
```

- Bring up PCL vision control (developing)
```
rosrun vision_control vision_control_node
```

