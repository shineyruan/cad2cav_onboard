# cad2cav_onboard
Welcome to the on-board part of CAD2CAV project. For a detailed project documentation, see [the main repo.](https://github.com/mlab-upenn/ISP2021-cad2cav).

![](docs/img/unreal.gif)
![](docs/img/automapper.gif)

## System Requirements
- Nvidia Jetson Xavier NX Development Kit (hardware device)
- Linux Ubuntu 20.04 LTS/18.04 LTS
- ROS Noetic/Melodic
- GCC 7+/Clang 7+

## Software Requirements
- Boost 1.71
- [OpenCV 4.5.1](https://github.com/opencv/opencv/tree/4.5.1)
- [TensorRT 8.0.1](https://developer.nvidia.com/tensorrt-getting-started)

First run and install the following packages from Ubuntu repository:
```bash
sudo apt install libboost-all-dev
```
Then install OpenCV 4.5.1 on Xavier NX board by running
```bash
cd scripts
sudo ./Install_OpenCV-4-5-1.sh
```
***Note.* It might take ~2 hrs to fully install OpenCV 4.5.1 on the device.**

Then install other dependencies manually, by either building from source or following the official documentation.

## Installation
The entire process is ready for installation on an F1/10 Autonomous Racing Car, which uses an **Nvidia Jetson Xavier** Board. Users must also install the system-level drivers for F1/10 cars from [this repository.](https://github.com/f1tenth/f1tenth_system.git)

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_init_workspace
cd src
git clone https://github.com/shineyruan/cad2cav_onboard.git
git clone https://github.com/f1tenth/f1tenth_system.git
git clone --recursive https://github.com/shineyruan/cad2cav_common.git
cd ..
```

To build the project on the car (ROS Melodic), **you have to manually specify Python 3 path for cv_bridge building compliance.** Run:
```bash
catkin_make_isolated --use-ninja -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/aarch64-linux-gnu/libpython3.6m.so
source devel_isolated/setup.bash
```

### Optional Dependencies
For object detection with YOLO in darknet architecture, one could also experiment with [Shumin326/darknet_ros](https://github.com/Shumin326/darknet_ros). The package has been adapted from the upstream repo and is verified to run successfully on the F1Tenth car.
```bash
cd ~/catkin_ws/src
git clone --recursive https://github.com/Shumin326/darknet_ros
cd ..
```

## Usage
1. For object detection, run

```bash
rosrun object_detection object_detection_node --model=darknet
```
You can also specify a pre-recorded video to visualize by adding `--video=<PATH_TO_VIDEO>` as a command line argument.

## Acknowledgements
**The object detection** part of project uses a pre-trained model of Tiny-YOLO v4 directly from [the author's website.](https://github.com/AlexeyAB/darknet)
