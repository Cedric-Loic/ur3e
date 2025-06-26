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

## Project Theory


### 1. YOLO-OBB Detection

*[YOLO-OBB](https://docs.ultralytics.com/fr/tasks/obb/)* extends standard object detection by predicting rotated bounding boxes. This is essential for understanding how the part is oriented on the conveyor and for computing the correct approach angle for the gripper.

- Input: RGB images
- Output: Object class + position + rotation angle
- Runs in real-time with GPU acceleration

For this application we've finetuned a yolo OBB model on [this dataset](https://universe.roboflow.com/ram-0ay3p/objectdetection-tovrk/dataset/2). The image bellow is a screenshot of the execution of ***`inference.py`*** 
![alt text](images/inference_screenshot.png)


### 2. Eye-in-Hand Extrinsic Calibration
An **eye-in-hand** setup places the camera directly on the robot’s end-effector (the wrist or gripper). This offers several advantages:

- The robot can look at different positions dynamically.
- Calibration between camera and gripper becomes easier to perform.
- Scene interpretation is from the robot’s end effector point of view.

This configuration is particularly beneficial for tasks requiring precise interaction with objects whose positions and orientations change—such as picking screws and nuts from a moving conveyor belt.

![alt text](images/image.png)

To transform camera detections into the robot’s coordinate frame, we use ***extrinsic calibration***. This determines the transformation between the camera and the robot’s tool frame (the end of the wrist).

Our hand-Eye calibration was done using this [***`dedicated ROS2-based tool`***](https://moveit.picknik.ai/humble/doc/examples/hand_eye_calibration/hand_eye_calibration_tutorial.html).




The following equation describes how to transform a single 3D point from the Zivid camera to the robot base coordinate system:

<p align="center">
  <img src="images/image-1.png" alt="Description de l'image" width="300"/>
</p>



---

## Installation Instructions

> These steps assume you are running **Ubuntu 22.04**.

### Step 1: Install ROS 2 Humble

Follow the [official installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html):

```bash

sudo apt update && sudo apt upgrade -y
sudo apt install curl gnupg2 lsb-release

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-humble-desktop -y

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```


### Step 2: Create a ROS 2 Workspace
```bash

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc

```
