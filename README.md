# Industrial Parts Sorting Using Eye-in-Hand Vision with ROS 2 Humble

This project implements a robotic system that sorts industrial parts **screws and nuts** in real-time as they move along a conveyor belt. It combines **vision algorithm**, **robotics**, and **AI** using the following components:

- ***Intel RealSense D435*** depth camera
- ***Nvidia RTX 4000*** GPU 
- ***UR3e*** robotic arm
- ***Robotiq Hand-E Gripper***
- ***ROS 2 Humble***
- ***YOLO-OBB*** (Oriented Bounding Box) object detection model

The camera is mounted in an **eye-in-hand configuration**, meaning it moves with the robot arm, offering dynamic visual perception. The robot detects and picks objects based on their type and orientation using a fine-tuned YOLO-OBB model.

---
## Table of Contents

- [Project Overview](#industrial-parts-sorting-using-eye-in-hand-vision-with-ros-2-humble)
- [Project Theory](#project-theory)
  - [1. YOLO-OBB Detection](#1-yolo-obb-detection)
  - [2. Installation of ROS2](#2-Installation-of-ROS2)
  - [3. Eye-in-Hand Extrinsic Calibration](#3-eye-in-hand-extrinsic-calibration)
    - [Moveit2_calibration installation](#Moveit2_calibration-installation)



---
## Project Theory


### 1. YOLO-OBB Detection

*[YOLO-OBB](https://docs.ultralytics.com/fr/tasks/obb/)* extends standard object detection by predicting rotated bounding boxes. This is essential for understanding how the part is oriented on the conveyor and for computing the correct approach angle for the gripper.

- Input: RGB images
- Output: Object class + position + rotation angle
- Runs in real-time with GPU acceleration

For this application we've finetuned a yolo OBB model on [this dataset](https://universe.roboflow.com/ram-0ay3p/objectdetection-tovrk/dataset/2). The image bellow is a screenshot of the execution of [*yolo_inference.py*](https://github.com/Cedric-Loic/ur3e/blob/main/yolo_inference.py)

<p align="center">
  <img src="images/inference_screenshot.png" alt="Description de l'image" width="800"/>
</p>



### 2. Installation of ROS2

```bash
locale  # check for UTF-8
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # verify settings

sudo apt install -y software-properties-common
sudo add-apt-repository universe

install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade

sudo apt install -y ros-humble-desktop
sudo apt install -y ros-humble-ros-base
sudo apt install -y ros-dev-tools
```
Then we install moveit2
```bash

cd ~/ws_moveit/src
git clone --branch humble https://github.com/moveit/moveit2_tutorials
vcs import --recursive < moveit2_tutorials/moveit2_tutorials.repos
sudo apt remove ros-$ROS_DISTRO-moveit*
sudo rosdep init
rosdep update
sudo apt update && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

cd ~/ws_moveit
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default
colcon build --mixin release --executor sequential
```

And then we install the ur drivers
```bash
sudo apt-get install -y ros-humble-ur
```


### 3. Eye-in-Hand Extrinsic Calibration
An **eye-in-hand** setup places the camera directly on the robot’s end-effector (the wrist or gripper). This offers several advantages:

- The robot can look at different positions dynamically.
- Calibration between camera and gripper becomes easier to perform.
- Scene interpretation is from the robot’s end effector point of view.

This configuration is particularly beneficial for tasks requiring precise interaction with objects whose positions and orientations change. To transform camera detections into the robot’s coordinate frame, we use ***extrinsic calibration***. This determines the transformation between the camera and the robot’s tool frame (the end of the wrist).

<p align="center">
  <img src="images/image.png" alt="Description de l'image" width="450"/>
</p>


The following equation describes how to transform a single 3D point from the depth camera to the robot base coordinate system:

<p align="center">
  <img src="images/image-1.png" alt="Description de l'image" width="300"/>
</p>

Our hand-Eye calibration was done using [`MoveIt2 Calibration`](https://github.com/AndrejOrsula/moveit2_calibration).


#### Moveit2_calibration installation

> We assume you have **ROS2 humble** already installed. If not, please install it from the [official installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).
```bash
cd ~/ros2_ws/src
git clone https://github.com/AndrejOrsula/moveit2_calibration.git
vcs import src < src/moveit_calibration/moveit_calibration.repos --skip-existing
rosdep install -r --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```
Please follow the [Hand-Eye Calibration](https://moveit.picknik.ai/humble/doc/examples/hand_eye_calibration/hand_eye_calibration_tutorial.html) tutorial to perform your extrinsic calibration before going to the next step.


