#!/usr/bin/env python2

import sys
import copy
import rospy
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from math import pi
import numpy as np
# import tf
from tf.msg import tfMessage
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Vector3, Pose2D, Pose, PointStamped, Quaternion
from std_msgs.msg  import Float32, Header
import roslaunch
import math
from bin_picking.msg import TargetsPose
from robonuc.msg import ios

from generate_plan_move import generate_plan, move_robot
from generate_plan_sinosoide import generate_plan_sinosoide

# initialize moveit_commander and rospy
print "============ Starting movement setup"
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_fanuc_sinosoide',
                anonymous=True)

# This RobotCommander object is an interface to the robot as a whole.
robot = moveit_commander.RobotCommander()

# This PlanningSceneInterface object is an interface to the world surrounding the robot.
scene = moveit_commander.PlanningSceneInterface()

# MoveGroupCommander object. This object is an interface to one group of joints. 
# In this case the group is the joints in the manipulator. This interface can be used
# to plan and execute motions on the manipulator.
group = moveit_commander.MoveGroupCommander("manipulator")

rate = rospy.Rate(10) # 10hz

# # Publisher of pointStamped of the grasping point
# grasping_point_pub = rospy.Publisher(
#                                     '/graspingPoint',
#                                     PointStamped,
#                                     queue_size = 10)
#                                     # Publisher of pointStamped of the grasping point
io_pub = rospy.Publisher(
                        '/io_client_messages',
                        ios,
                        queue_size = 10)

normal = Vector3()
approx_point = Vector3()
eef_position_laser = Vector3()
roll = np.pi
pitch = Float32()
yaw = Float32()
laser_reading = Float32()

# Function for sending a ROS msg to the vs_IO_client.cpp node responsable for altering the state of the IOs
# function - 1 to read, 2 to switch on and 3 to switch of the respective IO number (ionumber)
def monitoring_ios(function,ionumber):
    cod = function*10 + ionumber
    io_msg = ios()
    io_msg.code = cod
    print "Setting IOs code:"
    print cod
    io_pub.publish(io_msg)


def callback_targets_pose(targets_pose):

    normal.x = targets_pose.normal.x
    normal.y = targets_pose.normal.y
    normal.z = targets_pose.normal.z

    approx_point.x = targets_pose.approx_point.x
    approx_point.y = targets_pose.approx_point.y
    approx_point.z = targets_pose.approx_point.z

    eef_position_laser.x = targets_pose.eef_position.x
    eef_position_laser.y = targets_pose.eef_position.y
    eef_position_laser.z = targets_pose.eef_position.z

    pitch.data = targets_pose.euler_angles.y
    yaw.data = targets_pose.euler_angles.x

def callback_laser_sensor(output_laser_reading):
    
    laser_reading.data = output_laser_reading.data

# rospy.Subscriber("/targets_pose", TargetsPose, callback_targets_pose)
# rospy.Subscriber("/output_laser_sensor", Float32, callback_laser_sensor)

group.set_planning_time(10.0)
group.set_planner_id("RRTConnectkConfigDefault")



#Rotate robot
joint_values = group.get_current_joint_values()
joint_values[0] = -2.8
joint_values[1] = 0
joint_values[2] = 0
joint_values[3] = -1.50
joint_values[4] = -1.37
joint_values[5] = 0
group.set_joint_value_target(joint_values)
group.go(wait=True)


#go to first point
starting_point = Vector3()
starting_point.x = -0.341
starting_point.y = -0.365
starting_point.z = 0.670
# Quaternions of the Euler angles
quaternion_init = quaternion_from_euler(np.pi/2, 0, 0)
# 1st POSITION - Visualize Workspace
# GENERATING PLAN
plan1, fraction1 = generate_plan(group, starting_point, 5, quaternion_init)
# MOVEMENT
move_robot(plan1, fraction1, group)
#---------------------START SINOSOIDE--------------------
upper_point = Vector3()
upper_point = starting_point
upper_point.z= upper_point.z + 0.20 
print upper_point

sinosoide_aux=1

#------------------------OKK--------------------------
# while sinosoide_aux <= 2 :
#     #------------vai a cima
#     # Quaternions of the Euler angles

#     # # GENERATING PLAN
#     plan2, fraction2 = generate_plan(group, upper_point, 5, quaternion_init)

#     # # MOVING
#     move_robot(plan2, fraction2, group)

#     # #-----------vem a baixo-----

#     bottom_point = Vector3()
#     bottom_point = starting_point

#     bottom_point.z= bottom_point.z - 0.25


#     # #------------vai a cima
#     # # GENERATING PLAN
#     plan3, fraction3 = generate_plan(group, bottom_point, 5, quaternion_init)

#     # # MOVING
#     move_robot(plan3, fraction3, group)

#     sinosoide_aux=sinosoide_aux + 1 ;

#----------------------------------TRY ALL PLAN------

#------------vai a cima
# Quaternions of the Euler angles
# # GENERATING PLAN
plan2, fraction2 = generate_plan(group, upper_point, 5, quaternion_init)
# # MOVING
move_robot(plan2, fraction2, group)

# #------------SINOSOIDE
uuid3 = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid3)
launch_plat_move = roslaunch.parent.ROSLaunchParent(uuid3, ["/home/robuter/catkin_ws/src/Robonuc/robonuc_integration/demonstracao_sinosoide/move_linear_platform.launch"])
# Start Launch node sensorRS232
launch_plat_move.start()

amplitude=0.30
sinosoide_number=3

# # GENERATING PLAN
plan3, fraction3 = generate_plan_sinosoide(group, amplitude, sinosoide_number, quaternion_init)
# # MOVING
group.execute(plan3)
sinosoide_aux=sinosoide_aux + 1 ;

    
# IO number 8 activates the suction
# First activate IO number for for IO number 8 to work
# monitoring_ios(2,4)
# monitoring_ios(2,8)


# monitoring_ios(3,8)
# monitoring_ios(3,4)
exit()
