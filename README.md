---

This document provides detailed step-by-step instructions on how to set up and run the person detection software on the Pepper physical robot or using drivers.

## Prerequisites

The person detection module was developed using Ubuntu 18.04 ROS 1 Melodic. The following instructions assume you have the Ubuntu 18.04 operating system and the ROS 1 Melodic distribution setup. If you don’t already have them, get and install the Ubuntu 18.04 image from [https://releases.ubuntu.com/18.04/](https://releases.ubuntu.com/18.04/) and follow the installation guide on [https://wiki.ros.org/melodic/Installation/Ubuntu](https://wiki.ros.org/melodic/Installation/Ubuntu) to install ROS Melodic.

## Create and Configure a ROS Workspace

```bash
mkdir -p $HOME/ros/workspace/src
cd $HOME/ros/workspace
catkin_make
```

**Note:** The person detection module requires Python 3 compatibility. To achieve Python 3 compatibility, the first `catkin_make` command in a clean catkin workspace must be:

```bash
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
```

This will configure `catkin_make` with Python 3. You may then proceed to use just `catkin_make` for subsequent builds.

In your current directory, you should now have a `build` and `devel` folder. Inside the `devel` folder, you can see that there are now several `setup.*sh` files. Sourcing any of these files will overlay this workspace on top of your environment.

### Source your new `setup.*sh` file:

```bash
source devel/setup.bash
```

### Add the setup to your `.bashrc` file so that you don't have to do this every time you open a new terminal

```bash
echo "source $HOME/ros/workspace/devel/setup.bash" >> $HOME/.bashrc
```

## Install the Person Detection Software

### Move to the source directory of the workspace

```bash
cd $HOME/ros/workspace/src
```

### Clone the person detection software from GitHub

```bash
git clone https://github.com/NMutangana/personDetection.git
```

### Build the source files

```bash
cd .. && catkin_make
```

You Only Look Once (YOLO) is one of the state-of-the-art, real-time object detection algorithms used in the implementation of the person detection module. Darknet’s implementation of YOLOv3 was modified to fit the person detection task. Follow the instructions detailed below to install the modified implementation of Darknet’s YOLO algorithm to meet the requirements of the person detection task:

```bash
cd $HOME/ros/workspace/src
git clone https://github.com/NMutangana/cssr4africa_darknet.git
cd cssr4africa_darknet
git fetch origin
git checkout person_detection_modifications
make
```

### Download the pre-trained weight file

```bash
wget https://pjreddie.com/media/files/yolov3.weights
```

Based on the module specification, the interface of the person detection module will use the physical Pepper robot as the primary driver to generate test data. To get access to data obtained from the physical robot, the development environment for the physical robot needs to be set up. To do that, follow the instructions detailed in Section 3.2 of the Software Installation Manual (https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D3.3.pdf).

## Running the Person Detection Software

The person detection software can be run using the physical pepper robot or drivers to generate test data. Since two algorithms were used in the design of the person detection module, when running the software, it is important to specify which algorithm you want to use.

### Running the Person Detection Software on the Physical Pepper Robot

#### Open a new terminal, and bring up the Pepper robot. This is assuming the robot is turned on.

```bash
roslaunch pepper_dcm_bringup pepper_bringup.launch robot_ip:=<robot_ip> roscore_ip:=<roscore_ip> network_interface:=<network_interface_name>
```

#### Open another terminal and launch the NAOqi driver.

```bash
roslaunch naoqi_driver naoqi_driver.launch nao_ip:=<robot_ip> roscore_ip:=<roscore_ip> network_interface:=<network_interface_name>
```

#### Run the person detection software

```bash
cd $HOME/ros/workspace/src/
```

#### Open a new terminal
- If you want to test the person detection module using the Histogram of Oriented Gradients (HOG) algorithm

```bash
rosrun personDetection personDetectionHog
```

- If you want to test the person detection module using the You Only Look Once ((YOLO) algorithm

```bash
rosrun personDetection personDetectionYolo
```

#### Open a new terminal to visualize the outputs

```bash
rosrun personDetection personDetectionStub
```

## Running the Person Detection Software Using Drivers

### Change the directory to the src of the ROS workspace

```bash
cd $HOME/ros/workspace/src/
```

### Open a new terminal, and execute the driver that generates RGB image test data

```bash
rosrun personDetection personDetectionRGBImageDriver
```

### Open a new terminal, and execute the driver that generates depth image test data

```bash
rosrun personDetection personDetectionDepthImageDriver
```

### Open a new terminal
- If you want to test the person detection module using the Histogram of Oriented Gradients (HOG) algorithm

```bash
rosrun personDetection personDetectionHog
```

- If you want to test the person detection module using the You Only Look Once (YOLO) algorithm

```bash
rosrun personDetection personDetectionYolo
```

### Open a new terminal to visualize the outputs

```bash
rosrun personDetection personDetectionStub
```

---
