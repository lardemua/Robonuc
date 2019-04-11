#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg


#DEFENITION OFF ROBOT MODES
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
    print "mode_4_picking"

    joint_values[0] = 0
    joint_values[1] = 0
    joint_values[2] = 0
    joint_values[3] = 0
    joint_values[4] = 0
    joint_values[5] = 0

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


# if __name__ == '__main__':
#     try:
#         talker()
#     except rospy.ROSInterruptException:
#         pass
# initialize moveit_commander and rospy
print "============ Starting movement setup"
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('bin_picking_move_fanuc_debug',
                anonymous=True)

# This RobotCommander object is an interface to the robot as a whole.
robot = moveit_commander.RobotCommander()

# This PlanningSceneInterface object is an interface to the world surrounding the robot.
scene = moveit_commander.PlanningSceneInterface()

# MoveGroupCommander object. This object is an interface to one group of joints.
# In this case the group is the joints in the manipulator. This interface can be used
# to plan and execute motions on the manipulator.
group = moveit_commander.MoveGroupCommander("manipulator")

joint_values = group.get_current_joint_values() #incialize joint_values

print "============ Joint values: ", joint_values
#group.clear_pose_targets()

# joint_values = mode_4(joint_values)

#joint_values = mode_2(joint_values)


#group.set_joint_value_target(joint_values)

# plan2 = group.plan()

#group.go(wait=True)


# rate = rospy.Rate(10) # 10hz
