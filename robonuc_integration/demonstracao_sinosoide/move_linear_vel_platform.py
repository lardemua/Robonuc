#!/usr/bin/env python
# Tiago Almeida Tavares , DEM-UA  77001 , 17 junho 2019
import rospy
import time  # sleep
import sys
import copy

from r_platform.msg import navi 
from std_msgs.msg import String, Int8

from sensor_msgs.msg import Joy

class myclass ():
    def __init__(self):
        rospy.init_node('move_linear_platform', anonymous=True)

        #self.pub_status = rospy.Publisher('/RobotStatus', Int8, queue_size=10)
        pub_vel = rospy.Publisher('/navi_commands', navi, queue_size=1)

        rospy.Subscriber("/joy", Joy, self.joyCallback)

        self.cancell_var = False
        self.move_var = True



        rate = rospy.Rate(10) # 10hz

        self.robot_status= Int8()
        self.robot_status=2

        velocidade=navi()
        velocidade.linear_vel= 0.075;
        velocidade.angular_vel= 0;

        #self.pub_status.publish(self.robot_status)
        while not rospy.is_shutdown():

            #pub_vel.publish(velocidade) #debug
            #rospy.loginfo("while")
            #rospy.loginfo("cancel= %d", self.cancell_var)

            if self.move_var :
                velocidade.linear_vel= 0.06;
                velocidade.angular_vel= 0;
                pub_vel.publish(velocidade)
                self.robot_status=2
                #self.pub_status.publish(self.robot_status)
            else:
                self.robot_status=1
                #self.pub_status.publish(self.robot_status)


            if self.cancell_var:
                print "cancell_var"
                velocidade.linear_vel= 0;
                velocidade.angular_vel= 0;
                pub_vel.publish(velocidade)
                move_var= False

            rate.sleep()

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

if __name__ == '__main__':
    try:
        myclass()
    except rospy.ROSInterruptException:
        pass