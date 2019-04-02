/**
 * @file check_feasibility.cpp
 * @author Tiago Tavares (tiagoa.tavares@hotmail.com)
 * @brief Check the laser values (subscribe /output_laser_sensor ) and publish the action that robot needs to execute (pub on topic /feasibility)
 * @version 0.1
 * @date 2019-04-01
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#include "ros/ros.h"
#include <iostream>

#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

using namespace std;

class checker // class checker
{
  public:

    ros::NodeHandle n;
    std_msgs::String send_msg; //This is a message object. You stuff it with data, and then publish it.
    std::stringstream ss;

    float laser_limit; //p carregar o parametro

    checker()
    {
        ss.str("");
        n.getParam("/laser_limit", laser_limit);
    }

    void chatterCallback(const std_msgs::Float32::ConstPtr &msg)
    {
        //ROS_INFO("I heard: [%d]", msg->data);
        cout << "[check_feasibility] Iam reading:" << msg->data << endl;

        if ( (msg->data >= laser_limit) && (msg->data >0) )
        {
            ss.str("Platform should move.");
        }
        else
        {
            ss.str("Platform should stop.");
        }
    }
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
    ros::init(argc, argv, "check_feasibility");

    /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
    ros::NodeHandle n;

    //class checker
    checker my_checker;

    ros::Subscriber sub = n.subscribe("/output_laser_sensor", 100, &checker::chatterCallback, &my_checker);

    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("/feasibility", 100);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        my_checker.send_msg.data = my_checker.ss.str();
        // ROS_INFO("%s", msg.data.c_str());

        cout << "[check_feasibility] Iam pub:" << my_checker.send_msg << endl;
        chatter_pub.publish(my_checker.send_msg);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}