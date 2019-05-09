#!/usr/bin/env python2
# Tiago Almeida Tavares , DEM-UA  77001 , 17 April 2019
import rospy
import time  # sleep
import sys
import copy

from actionlib.action_server import ActionServer
from binpicking_action.msg import Robot_binpickingAction, Robot_binpickingFeedback, Robot_binpickingResult

# include for moveit
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

# include for picking
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from math import pi
import numpy as np
# import tf
from tf.msg import tfMessage
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Vector3, Pose2D, Pose, PointStamped, Quaternion
from std_msgs.msg import Float32, Header
import roslaunch
import math
from bin_picking.msg import TargetsPose
from robonuc.msg import ios

from generate_plan_move import generate_plan, move_robot

#import subprocess
# import threading
# import collections

from bin_picking.srv import *

from sensor_msgs.msg import Joy

normal = Vector3()
approx_point = Vector3()
eef_position_laser = Vector3()
roll = np.pi
pitch = Float32()
yaw = Float32()
laser_reading = Float32()


class RefServer (ActionServer):

    def __init__(self, name):
        self.server_name = name
        action_spec = Robot_binpickingAction
        ActionServer.__init__(
            self, name, action_spec, self.goalCallback, self.cancelCallback, False)
        self.start()  # como metemos o ultimo parametro a False, damos o start aqui
        rospy.loginfo("Creating ActionServer [%s]\n", name)

        self.saved_goals = []
        self._feedback = Robot_binpickingFeedback()
        self._result = Robot_binpickingResult()



        # Publisher of pointStamped of the grasping point
        grasping_point_pub = rospy.Publisher(
            '/graspingPoint',
            PointStamped,
            queue_size=10)
        # Publisher of pointStamped of the grasping point
        self.io_pub = rospy.Publisher(
            '/io_client_messages',
            ios,
            queue_size=10)

        self.continue_move = False #variable to confirm planning for picking
        self.cancellAction_var=False
        rospy.Subscriber("/joy", Joy, self.joyCallback)


    def joyCallback(self, msg) :
        
        if msg.buttons[1] == 1:
            self.continue_move= True;
            rospy.loginfo("Button B pressed: continue_move=%d", self.continue_move)

        if msg.axes[2] >= -0.90:
            self.cancellAction_var= True;
            #rospy.loginfo("DeathmanSwich realeased%d", self.cancellAction_var)
        #else :
            #self.cancellAction_var= True;



        return
    # Function for sending a ROS msg to the vs_IO_client.cpp node responsable for altering the state of the IOs
    # function - 1 to read, 2 to switch on and 3 to switch of the respective IO number (ionumber)

    def monitoring_ios(self, function, ionumber):
        cod = function*10 + ionumber
        io_msg = ios()
        io_msg.code = cod
        print "Setting IOs code:"
        print cod
        self.io_pub.publish(io_msg)

    # ======================GOALCALLBACK================================
    def goalCallback(self, gh):

        self.cancellAction_var= False;
        success = True
        goal = gh.get_goal()

        id = gh.get_goal_id().id
        #rospy.loginfo("Received request for goal " + str(id))

        self._feedback.sequence = []

        rospy.loginfo("Got goal %d", int(goal.mode))

        # if self.is_preempt_requested():
        #     rospy.loginfo('%s: Preempted' % self.name)
        #     self.set_preempted()
        #     success = False


        if goal.mode == 1:
            gh.set_accepted()

            # process =subprocess.call(["rosrun", "bin_picking", "move_fanuc_demo.py", "&"])
            #process = subprocess.call(["roslaunch", "robonuc_integration", "display_urdf_total.launch"])
            self.monitoring_ios(3,4)
        if success:
            # self._result.sequence = self._feedback.sequence
            self._feedback.sequence.append(1)
            gh.publish_feedback(self._feedback)
            self._result.result = True
            gh.set_succeeded(self._result, "Server succeeded")
            rospy.loginfo('%s: Succeeded', self)
            return
        else :
            gh.set_aborted(None, "The ref server has aborted")
            return


    def cancelCallback(self, gh):
        # process.kill()
        rospy.loginfo("XXX action canceled XXX")
        return
        # if self._as.is_preempt_requested():
        #     self._as.set_preempted()
        # pass


if __name__ == "__main__":
    rospy.init_node("BinPickingAction")
    ref_server = RefServer("BinPickingAction")

    rospy.spin()
