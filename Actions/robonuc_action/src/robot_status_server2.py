#!/usr/bin/env python
# Tiago Almeida Tavares , DEM-UA  77001 , 9 April 2019
import rospy
import time  # sleep
import sys
import copy

from actionlib.action_server import ActionServer
from robonuc_action.msg import Robot_statusAction, Robot_statusFeedback, Robot_statusResult

# include for moveit
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

# DEFENITION OFF ROBOT MODES
def mode_0(joint_values):
    print "mode_0_OFF"
    joint_values[0] = 0
    joint_values[1] = 0
    joint_values[2] = 0
    joint_values[3] = 0
    joint_values[4] = 0
    joint_values[5] = 0
    return joint_values

def mode_1(joint_values):
    print "mode_1_navigation"
    joint_values[0] = 0.05
    joint_values[1] = -0.67
    joint_values[2] = -1.07
    joint_values[3] = -0.07
    joint_values[4] = -0.74
    joint_values[5] = -0.01
    return joint_values

def mode_2(joint_values):
    print "mode_2_aprox_laser"

    joint_values[0] = 0.05
    joint_values[1] = -0.67
    joint_values[2] = -1.07
    joint_values[3] = -0.07
    joint_values[4] = -0.74
    joint_values[5] = -0.01

    return joint_values

def mode_3(joint_values):
    print "mode_3_aprox_for_picking"

    joint_values[0] = -1.61
    joint_values[1] = -1.12
    joint_values[2] = -0.94
    joint_values[3] = -0.64
    joint_values[4] = -1.13
    joint_values[5] = 0.08

    return joint_values

def mode_4(joint_values):
    print "mode_4_picking"

    joint_values[0] = 0
    joint_values[1] = 0
    joint_values[2] = 0
    joint_values[3] = 0
    joint_values[4] = 0
    joint_values[5] = 0

    return joint_values

class RefServer (ActionServer):

    def __init__(self, name):
        self.server_name = name
        action_spec = Robot_statusAction
        ActionServer.__init__(
            self, name, action_spec, self.goalCallback, self.cancelCallback, False)
        self.start()  # como metemos o ultimo parametro a False, damos o start aqui
        rospy.loginfo("Creating ActionServer [%s]\n", name)

        self.saved_goals = []
        self._feedback = Robot_statusFeedback()
        self._result = Robot_statusResult()

        # construtor para moveit
        # This RobotCommander object is an interface to the robot as a whole.
        robot = moveit_commander.RobotCommander()

        # This PlanningSceneInterface object is an interface to the world surrounding the robot.
        scene = moveit_commander.PlanningSceneInterface()

        # MoveGroupCommander object. This object is an interface to one group of joints.
        # In this case the group is the joints in the manipulator. This interface can be used
        # to plan and execute motions on the manipulator.
        self.group = moveit_commander.MoveGroupCommander("manipulator")

    def goalCallback(self, gh):

        success = True
        goal = gh.get_goal()

        self._feedback.sequence = []

        rospy.loginfo("Got goal %d", int(goal.mode))
        joint_values = [0,0,0,0,0,0]

        joint_values = self.group.get_current_joint_values()  # incialize joint_values

        # if self.is_preempt_requested():
        #     rospy.loginfo('%s: Preempted' % self.name)
        #     self.set_preempted()
        #     success = False

        if goal.mode == 0:
            gh.set_accepted()
            # clear pose target
            self.group.clear_pose_targets()
            joint_values = mode_0(joint_values)

            self.group.set_joint_value_target(joint_values)
            self.group.go(wait=True)

        if goal.mode == 1:
            gh.set_accepted()

            self.group.clear_pose_targets()  # clear pose target
            joint_values = mode_1(joint_values)
            # time.sleep(5)
            #gh.set_succeeded(None, "The ref server has succeeded")
            self.group.set_joint_value_target(joint_values)
            self.group.go(wait=True)

        elif goal.mode == 2:
            gh.set_accepted()
            self.group.clear_pose_targets()  # clear pose target
            joint_values = mode_2(joint_values)
            self.group.go(wait=True)
            #gh.set_aborted(None, "The ref server has aborted")

            self.group.set_joint_value_target(joint_values)

        elif goal.mode == 3:
            gh.set_accepted()
            self.group.clear_pose_targets()  # clear pose target
            joint_values = mode_3(joint_values)

            self.group.set_joint_value_target(joint_values)

            self.group.go(wait=True)
            # gh.set_succeeded()

        elif goal.mode == 4:
            gh.set_accepted()
            self.group.clear_pose_targets()  # clear pose target
            joint_values = mode_4(joint_values)

            self.group.set_joint_value_target(joint_values)
            self.group.go(wait=True)
            # gh.set_succeeded()
        else:
            success = False
            gh.set_aborted(None, "The ref server has aborted")

        self._feedback.sequence.append(joint_values[0])
        self._feedback.sequence.append(joint_values[1])
        self._feedback.sequence.append(joint_values[2])
        self._feedback.sequence.append(joint_values[3])
        self._feedback.sequence.append(joint_values[4])
        self._feedback.sequence.append(joint_values[5])

        if success:
            #self._result.sequence = self._feedback.sequence

            gh.publish_feedback(self._feedback)

            self._result.result = True
            rospy.loginfo('%s: Succeeded', self.server_name)
            gh.set_succeeded(self._result)

    def cancelCallback(self, gh):
        pass


if __name__ == "__main__":
    rospy.init_node("RobotStatusAction_server")
    ref_server = RefServer("RobotStatusAction")

    rospy.spin()
