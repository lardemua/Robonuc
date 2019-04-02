/**
 * @file integrated_referee.cpp
 * @author Tiago Tavares (tiagoa.tavares@hotmail.com)
 * @brief 
 * @version 0.1
 * @date 2019-04-01
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include "ros/ros.h"
#include <iostream>

#include "std_msgs/String.h"

#include <sensor_msgs/Joy.h>

#include <r_platform/navi.h>

using namespace std;

class checker // class checker
{
  public:
    ros::NodeHandle n;
    std::stringstream ss;
    r_platform::navi vel_msg;

    bool robot_allowed = false;

    int linear_, angular_;    // id of angular and linear axis (position in the array)
    float l_scale_, a_scale_; // linear and angular scale

    ros::Publisher vel_pub=n.advertise<r_platform::navi>("/navi_commands", 20);

    checker() : linear_(1),
                angular_(3),
                l_scale_(0.025),
                a_scale_(0.025)
    {
        ss.str("");
    }

    void chatterCallback(const std_msgs::String::ConstPtr &msg)
    {
        //ROS_INFO("I heard: [%d]", msg->data);
        cout << "[integrated_Referee] Iam reading:" << msg->data << endl;
        ss.str(msg->data);
        cout << "ss=" << ss.str() << endl;
    }

    void joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
    {
        // decrease velocity rate
        if (joy->buttons[4] == 1 && a_scale_ > 0 && l_scale_ > 0.0)
        {
            l_scale_ = l_scale_ - 0.025;
            a_scale_ = a_scale_ - 0.025;
            ROS_INFO("DEC v_rate l[%f] a[%f]", l_scale_, a_scale_);
        }

        //increase velocity rate
        if (joy->buttons[5] == 1 && a_scale_ < 0.5 && l_scale_ < 0.5)
        {
            l_scale_ = l_scale_ + 0.025;
            a_scale_ = a_scale_ + 0.025;
            ROS_INFO("INC v_rate l[%f] a[%f]", l_scale_, a_scale_);
        }

        if (joy->axes[2] <= -0.89) // deadman switch active
        {
            if (joy->buttons[0] == 1)
            {
                robot_allowed = true;
                ROS_INFO("Button A pressed! Robot=%d", robot_allowed);
            }
        }
        else
        {
            robot_allowed = false;
        }

        // vel_pub_.publish(vel_msg);
    }

    void action_Callback()
    {

        //     vel_msg.linear_vel = l_scale_ * joy->axes[linear_];
        //     vel_msg.angular_vel = a_scale_ * joy->axes[angular_];

        if (robot_allowed == true && ss.str() == "Platform should move.")
        {
            vel_msg.linear_vel = 0.025;
            vel_msg.angular_vel = 0;
            //vel_msg.robot = 0; //este campo nao interessa para o decompose_vel (acho eu)
            cout << "[integrated_referee]PLAT will be moved!" << endl;
            vel_pub.publish(vel_msg);

        }
        else
        {

            cout << "[integrated_referee]PLAT will NOT be moved!" << endl;
            cout << "ss=" << ss.str() << endl;
        }

        
    }

  private:
};

int main(int argc, char **argv)
{
    /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
    ros::init(argc, argv, "integrated_referee");

    /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
    ros::NodeHandle n;

    //class checker
    checker my_checker;

    ros::Subscriber feasibility_sub = n.subscribe("/feasibility", 100, &checker::chatterCallback, &my_checker);

    ros::Subscriber joy_sub_ = n.subscribe<sensor_msgs::Joy>("/joy", 20, &checker::joyCallback, &my_checker);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        // my_checker.send_msg.data = my_checker.ss.str();
        // ROS_INFO("%s", msg.data.c_str());

        // cout << "[check_feasibility] Iam pub:" << my_checker.send_msg << endl;
        // chatter_pub.publish(my_checker.send_msg);

        cout << "[integrated_referee]robot_allowed=" << my_checker.robot_allowed << endl;
        my_checker.action_Callback();

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}