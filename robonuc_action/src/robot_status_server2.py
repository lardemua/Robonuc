#!/usr/bin/env python
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIghT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIghT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Author: Alexander Sorokin.
# Based on code from ref_server.cpp by Vijay Pradeep

import rospy

from actionlib.action_server import ActionServer
#from actionlib.msg import TestAction, TestFeedback, TestResult

from robonuc_action.msg import Robot_statusAction, Robot_statusFeedback, Robot_statusResult

class RefServer (ActionServer):

    def __init__(self, name):
        self.server_name= name
        action_spec = Robot_statusAction
        ActionServer.__init__(
            self, name, action_spec, self.goalCallback, self.cancelCallback, False)
        self.start() #como metemos o ultimo parametro a False, damos o start aqui
        rospy.loginfo("Creating ActionServer [%s]\n", name)

        self.saved_goals = []
        self._feedback = Robot_statusFeedback()
        self._result = Robot_statusResult()

    def goalCallback(self, gh):
        
        success = True
        goal = gh.get_goal()

        self._feedback.sequence = []
        self._feedback.sequence.append(0)
        self._feedback.sequence.append(1)

        rospy.loginfo("Got goal %d", int(goal.mode))

        # if self.is_preempt_requested():
        #     rospy.loginfo('%s: Preempted' % self.name)
        #     self.set_preempted()
        #     success = False
        if goal.mode == 0:
            gh.set_accepted()

        if goal.mode == 1:
            gh.set_accepted()

            #gh.set_succeeded(None, "The ref server has succeeded")
            
        elif goal.mode == 2:
            gh.set_accepted()
            #gh.set_aborted(None, "The ref server has aborted")
        elif goal.mode == 3:
            gh.set_accepted()

            #gh.set_succeeded()
        elif goal.mode == 3:
            gh.set_accepted()

            #gh.set_succeeded()
        else:
            success = False
            gh.set_aborted(None, "The ref server has aborted")

        if success:
            #self._result.sequence = self._feedback.sequence

            gh.publish_feedback(self._feedback)

            self._result.result=True
            rospy.loginfo('%s: Succeeded', self.server_name)
            gh.set_succeeded(self._result)
        
    def cancelCallback(self, gh):
        pass


if __name__ == "__main__":
    rospy.init_node("RobotStatusAction_server")
    ref_server = RefServer("RobotStatusAction")

    rospy.spin()