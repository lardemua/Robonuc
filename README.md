# ROBONUC: Mobile manipulator
My Masters Thesis Project

You can see my weekly blog at: https://tiagoatavares.github.io/tiagotavares_blog/

## Thesis itle

*Integrated Tele-Operation of the ROBONUC Platform for Mobile Bin-Picking*

Department of Mechanical Engineering (DEM), University of Aveiro (UA)

LAR: Laboratory of Automation and Robotics [ http://lars.mec.ua.pt/ ]

2019

## Advisor
<!-- Miguel Riem de Oliveira [GitHub](https://github.com/miguelriemoliveira/) -->

Professor Doutor Vítor Manuel Ferreira dos Santos
DEM, UA
Aveiro, Portugal

<!-- # Table of contents -->
- [ROBONUC: Mobile manipulator](#robonuc--mobile-manipulator)
  * [Thesis itle](#thesis-itle)
  * [Advisor](#advisor)
- [Built with](#built-with)
  * [Hardware](#hardware)
- [Installation guides](#installation-guides)
  * [System: Robonuc mobile plataform](#system--robonuc-mobile-plataform)
  * [System: Robot + kinect + laser](#system--robot---kinect---laser)
    + [ROS Industrial and FANUC](#ros-industrial-and-fanuc)
    + [Kinect drives](#kinect-drives)
    + [Calibration](#calibration)

# Built with

- [ROS Melodic](http://www.ros.org/)


## Hardware

Robutter II restructured (for more info see: http://lars.mec.ua.pt/public/LAR%20Projects/MobileManipulation/2017_BrunoVieira/ and http://lars.mec.ua.pt/public/LAR%20Projects/MobileManipulation/2017_VitorSilva/ )

Arduino Leonardo ETH
Arduino Micro

Hokuyo URG-04LX-UG01
Hokuyo UTM-30LX

JoyStick/Gamepad Microsoft XBox 360

-----
Fanuc Robot LR Mate 200iD

Microsoft Kinect

Laser Sensor DT20 Hi

# Installation guides

## System: Robonuc mobile plataform

(This section needs to be updated, because there are packages that have been cloned from other sources)

Install moveit, navigation, and costmap-converter:
```
sudo apt-get update
sudo apt-get install ros-melodic-moveit
sudo apt-get install ros-melodic-navigation
sudo apt-get install ros-melodic-costmap-converter

sudo apt-get install ros-melodic-teb-local-planner
```

To install the package for xbox joystick:
```
sudo apt-get install ros-melodic-joy
```

Intall hector_slam by source, em catkin_ws/src/ : 
```
git clone https://github.com/tu-darmstadt-ros-pkg/hector_slam
```

Maybe you need to install q4 and q5 (not sure, that q5 is necessary):
```
sudo apt-get install build-essential 
sudo apt-get install qtcreator
sudo apt-get install qt5-default
sudo apt-get install qt4-default 
```

To compile, it is necessary to follow a certain order, because there are dependencies not included.
```
cd ~/catkin_ws
catkin_make -pkg driver_base
catkin_make -pkg hokuyo_node
catkin_make -pkg r_plataform
catkin_make -pkg comm_tcp

catkin_make
```

If you have some runtime errors in the automatic mode, or in map construction, check the position of the lasers.
You just need to change the parameter "ttyACM", on launch file.
Example:
```
<param name="port" type="string" value="/dev/ttyACM0" /> 
                    --->
<param name="port" type="string" value="/dev/ttyACM1" /> 
```



## System: Robot + kinect + laser

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
export LC_NUMERIC="en_US.UTF-8" or echo 'export LC_NUMERIC="en_US.UTF-8"' >>~/.bashrc
(if the colors arent good)

roslaunch fanuc_lrmate200id_support test_lrmate200id.launch 
```

Case you want to see the actual position of your robot:

On the TP, run rosstate

On terminal, launch:
```
roslaunch fanuc_lrmate200id_support robot_state_visualize_lrmate200id.launch robot_ip:=192.168.0.231
```
### Kinect drives

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
### URDF
To see total URDF, launch:

```
roslaunch robonuc_integration display_urdf_total.launch
```