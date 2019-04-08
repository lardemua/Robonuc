#! /usr/bin/env python
import rospy

import actionlib
#changes
import robonuc_action.msg

class FibonacciAction(object):
    # create messages that are used to publish feedback/result
    _feedback = robonuc_action.msg.Robot_statusActionFeedback()
    _result = robonuc_action.msg.Robot_statusActionResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.ActionServer(self._action_name, robonuc_action.msg.Robot_statusAction, self.execute_cb, self.internal_preempt_callback, auto_start = False)
        self._as.start() #server on
      
    def internal_preempt_callback(self):
        pass

    def execute_cb(self, goal):

        

        # rospy.loginfo("Received GOAL " + str(goal.mode) )
        # helper variables
        r = rospy.Rate(1)
        success = True
        
        # goal.set
        # self._as.accept_new_goal();
        # self._as.getState()
        success=True
        
        # # start executing the action
        # for i in range(1, goal.order):
        #     # check that preempt has not been requested by the client
        #     if self._as.is_preempt_requested():
        #         rospy.loginfo('%s: Preempted' % self._action_name)
        #         self._as.set_preempted()
        #         success = False
        #         break
        #     self._feedback.sequence.append(self._feedback.sequence[i] + self._feedback.sequence[i-1])
        #     # publish the feedback
        #     self._as.publish_feedback(self._feedback)
        #     # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
        #     r.sleep()
          
        if success:
            # self._result.sequence = self._feedback.sequence
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
    rospy.init_node('RobotStatusAction')
    server = FibonacciAction(rospy.get_name())
    rospy.spin()