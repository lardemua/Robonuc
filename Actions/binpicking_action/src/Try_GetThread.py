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
import threading
import collections


# Named tuple for storing the goal handle and the corresponding processing thread
GoalHandleThread = collections.namedtuple('GoalHandleThread', 'goal_handle thread')

class RefServer (ActionServer):

    _threads = {}  # a dictionary containing a GoalHandleThread tuple for each goal processing thread

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

    # G======================GOALCALLBACK================================
    def goalCallback(self, gh):

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


            #LAUNCH obj detection thread
            thread = threading.Thread(target=self.launch_objDetection_pointTFtranfer_thread, args=(gh,))  # create a thread to process this goal
            self._threads[id] = (GoalHandleThread(gh, thread))  # add to tasks dictionary
            
            thread.start()  # initiate thread
            thread.join()

            time.sleep(8)
            rospy.loginfo("END IF GOAL==1")


            # =========//===Endpicking===//===========
        else:
            success = False
            gh.set_aborted(None, "The ref server has aborted")
            return

        if success:
            # self._result.sequence = self._feedback.sequence
            self._feedback.sequence.append(1)
            self._feedback.sequence.append(1)
            self._feedback.sequence.append(1)
            self._feedback.sequence.append(1)

            gh.publish_feedback(self._feedback)

            self._result.result = True
            rospy.loginfo('%s: Succeeded', self)
            return

    def launch_objDetection_pointTFtranfer_thread(self,gh):
        
        goal = gh.get_goal()
        id = gh.get_goal_id().id
        _, thread = self._threads[id] 
        
        rospy.logwarn("Launched a processing thread for goal id " + str(id))

        subprocess.call(["roslaunch", "robonuc_integration", "display_urdf_total.launch"])
        #Launch objDetection and pointTFtransfer nodes
        # uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        # roslaunch.configure_logging(uuid)
        # launch_objDetect_pointTF = roslaunch.parent.ROSLaunchParent(uuid, ["/home/tiago/catkin_ws/src/Bin-picking/bin_picking/launch/objDetection_pointTFtranfer.launch"])
        # # Start Launch node objDetection and pointTFtransfer
        # launch_objDetect_pointTF.start()

        time.sleep(5)
        #Stop Launch node objDetection and pointTFtransfer
        # launch_objDetect_pointTF.shutdown()
        # after having stopped both nodes the subscribed topics will be the last published and will be a fixed value
        rospy.logwarn("Thread end ")
        print("Thread")


        del self._threads[id]  # remove from dictionary
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
