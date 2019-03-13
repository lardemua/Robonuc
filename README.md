# ROBONUC: Mobile manipulator
My Masters Thesis Project 

## Thesis itle

*Integrated Tele-Operation of the ROBONUC Platform for Mobile Bin-Picking*

Department of Mechanical Engineering (DEM), University of Aveiro (UA)

LAR: Laboratory of Automation and Robotics

2019

## Advisor
<!-- Miguel Riem de Oliveira [GitHub](https://github.com/miguelriemoliveira/) -->

Professor Doutor Vítor Manuel Ferreira dos Santos
DEM, UA
Aveiro, Portugal

# Table of contents
- [ROBONUC: Mobile manipulator](#robonuc--mobile-manipulator)
  * [Thesis itle](#thesis-itle)
  * [Advisor](#advisor)
- [Table of contents](#table-of-contents)
- [Built with](#built-with)
  * [Hardware](#hardware)
- [Installation guides](#installation-guides)
    + [ROS Industrial and FANUC](#ros-industrial-and-fanuc)
    + [Calibration](#calibration)

# Built with

- [ROS Melodic](http://www.ros.org/)


## Hardware

Fanuc Robot LR Mate 200iD

Microsoft Kinect

Laser Sensor DT20 Hi

# Installation guides

##System: Robonuc mobile plataform


##System: Robot + kinect + laser

### ROS Industrial and FANUC

```
sudo apt-get install ros-melodic-xacro 
sudo apt-get install ros-melodic-industrial-core
sudo apt-get install ros-melodic-ros-canopen
```

Ah this stage, ROS Industrial and xacro are installed. 
Now, you need to install this 2 packs:
```
git clone https://github.com/ros-industrial/fanuc.git
git clone https://github.com/ros-industrial/fanuc_experimental.git

```
If you have errors while compiling, you need to follow these steps:
```
cd ~/catkin_ws/src/fanuc_experimental/
find . -type f -exec sed -i 's/boost\:\:shared_ptr/std\:\:shared_ptr/g' {} \;
find . -type f -exec sed -i 's/boost\:\:const_pointer_cast/std\:\:const_pointer_cast/g' {} \;

cd ~/catkin_ws/src/fanuc/
find . -type f -exec sed -i 's/boost\:\:shared_ptr/std\:\:shared_ptr/g' {} \;
find . -type f -exec sed -i 's/boost\:\:const_pointer_cast/std\:\:const_pointer_cast/g' {} \;
```

To see the robot on rviz, you should launch:
```
export LC_NUMERIC="en_US.UTF-8"
(if the colors arent good)

roslaunch fanuc_lrmate200id_support test_lrmate200id.launch 
```

Case you want to see the actual position of your robot:

On the TP, run rosstate

On terminal, launch:
```
roslaunch fanuc_lrmate200id_support robot_state_visualize_lrmate200id.launch robot_ip:=192.168.0.231
```
###Kinect drives

```
sudo apt-get install ros-melodic-openni-camera ros-melodic-openni-launch

```
Too see if the kinect is connected, launch:
```
roslaunch openni_launch openni.launch 
```
And on Rviz add an image .

### Calibration

```
cd ~/catkin_ws/src

git clone https://github.com/lagadic/vision_visp

git clone https://github.com/UbiquityRobotics/fiducials.git

<!-- git clone https://github.com/pal-robotics/aruco_ros -->

git clone https://github.com/jhu-lcsr/aruco_hand_eye

sudo apt-get install libvisp-dev libvisp-doc visp-images-data

sudo apt-get install ros-melodic-vision*

sudo apt-get install ros-melodic-hector-xacro-tools 
sudo apt-get install ros-melodic-hector-sensors-description 

cd ..

catkin_make
```

