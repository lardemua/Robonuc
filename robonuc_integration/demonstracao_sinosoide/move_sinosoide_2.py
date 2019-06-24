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

from sensor_msgs.msg import Joy

from std_msgs.msg import String, Int8

class myclass :
    def __init__(self):
        self.cancell_var = False
        self.move_var = False

    def joyCallback(self, msg) :

        #print "here" #debug
        if msg.buttons[2] == 1:
            self.move_var= True
            self.cancell_var= False

            #print "buuton 1"
            #rospy.loginfo("Button B pressed: %d", self.continue_move)
        if msg.axes[2] >= -0.90:
            #print "Axis 2"
            self.cancell_var= True
            self.move_var= False

        return



if __name__ == "__main__":

    myclass1= myclass()

    normal = Vector3()
    approx_point = Vector3()
    eef_position_laser = Vector3()
    roll = np.pi
    pitch = Float32()
    yaw = Float32()
    laser_reading = Float32()
    # initialize moveit_commander and rospy
    print "============ Starting movement setup"
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_fanuc_sinosoide',anonymous=True)

    # This RobotCommander object is an interface to the robot as a whole.
    robot = moveit_commander.RobotCommander()

    # This PlanningSceneInterface object is an interface to the world surrounding the robot.
    scene = moveit_commander.PlanningSceneInterface()

    # MoveGroupCommander object. This object is an interface to one group of joints. 
    # In this case the group is the joints in the manipulator. This interface can be used
    # to plan and execute motions on the manipulator.
    group = moveit_commander.MoveGroupCommander("manipulator")

    io_pub = rospy.Publisher('/io_client_messages', ios, queue_size = 10)
    pub_status = rospy.Publisher('/RobotStatus', Int8, queue_size=10)
    rospy.Subscriber("/joy", Joy, myclass1.joyCallback)


    rate = rospy.Rate(10) # 10hz

    group.set_planning_time(10.0)
    group.set_planner_id("RRTConnectkConfigDefault")

    robot_status= Int8()
    robot_status=1

    while not rospy.is_shutdown():

        #print myclass1.cancell_var

        if myclass1.move_var:

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
            plan1, fraction1 = generate_plan(group, starting_point, 5,quaternion_init)
            # MOVEMENT
            move_robot(plan1, fraction1, group)
            #---------------------START SINOSOIDE--------------------
            upper_point = Vector3()
            upper_point = starting_point
            upper_point.z= upper_point.z + 0.20 
            print upper_point
            #------------vai a cima
            # Quaternions of the Euler angles
            # # GENERATING PLAN
            plan2, fraction2 = generate_plan(group, upper_point, 5, quaternion_init)
            # # MOVING
            move_robot(plan2, fraction2, group)

            # #------------SINOSOIDE
            robot_status=2
            pub_status.publish(robot_status) #exit navigation


            uuid3 = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid3)
            launch_plat_move = roslaunch.parent.ROSLaunchParent(uuid3, ["/home/robuter/catkin_ws/src/Robonuc/robonuc_integration/demonstracao_sinosoide/move_linear_platform.launch"])
            # Start Launch node sensorRS232
            launch_plat_move.start()

            #======PARAMETERS=========
            amplitude=0.35
            sinosoide_number=8
            #===========================

            # # GENERATING PLAN
            plan3, fraction3 = generate_plan_sinosoide(group, amplitude, sinosoide_number, quaternion_init)
            # # MOVING
            group.execute(plan3)

            robot_status=1
            pub_status.publish(robot_status) #return navigation

            launch_plat_move.shutdown()
            myclass1.move_var= False

            
            #exit()




    rospy.spin()  # spin away!
