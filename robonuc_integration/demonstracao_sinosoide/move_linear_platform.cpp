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
#include "std_msgs/String.h" // for /feasibility

#include <r_platform/navi.h> //for /navi_commands

#include <unistd.h> //for sleep

class AproximationAction
{
  public:
    //para criar o server action
    ros::NodeHandle n;
    std::stringstream ss;

    actionlib::SimpleActionServer<robonuc_aprox_laser_action::Robot_PlatformLaserAproximationAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    std::string action_name_;

    // create messages that are used to published feedback/result
    robonuc_aprox_laser_action::Robot_PlatformLaserAproximationFeedback feedback_;
    robonuc_aprox_laser_action::Robot_PlatformLaserAproximationResult result_;

    //navigation pack
    r_platform::navi vel_msg;
    ros::Publisher vel_pub;

    //checkar o topico de feasibility
    ros::Subscriber feasibility_sub;

    AproximationAction(std::string name) : as_(n, name, boost::bind(&AproximationAction::executeCB, this, _1), false),
                                           action_name_(name)
    {

        feasibility_sub = n.subscribe("/feasibility", 1, &AproximationAction::chatterCallback, this); //topic that says if we need to move or not

        vel_pub = n.advertise<r_platform::navi>("/navi_commands", 2);

        as_.start();
    }

    ~AproximationAction(void)
    {
    }

    void chatterCallback(const std_msgs::String::ConstPtr &msg)
    {
        // cout << "[integrated_Referee] Iam reading:" << msg->data << endl;
        ss.str(msg->data);
        //std::cout << "ss=" << ss.str() << std::endl;
        //ROS_INFO("[LASER ACTION] Iam reading: [%s]", ss.str() );
    }

    void executeCB(const robonuc_aprox_laser_action::Robot_PlatformLaserAproximationGoalConstPtr &goal)
    {
        float velocidade = 0.04;
        //helper variables
        //ros::Rate r(0.05);
        bool success = true;

        while (ss.str() == "Platform should move.")
        {
            //ROS_INFO("I'M MOVING");
            //usleep(100000);

            if (as_.isPreemptRequested() || !ros::ok() )
            {
                ROS_INFO("%s: Preempted", action_name_.c_str());
                // set the action state to preempted
                as_.setPreempted();
                success = false;
                break;
            }

            vel_msg.linear_vel = velocidade;
            vel_msg.angular_vel = 0;
            //vel_msg.robot = 0; //este campo nao interessa para o decompose_vel (acho eu)
            // std::cout << "[integrated_referee]PLAT will be moved!" << std::endl;
            vel_pub.publish(vel_msg);

            feedback_.sequence.clear();
            feedback_.sequence.push_back(1);
            as_.publishFeedback(feedback_);
        }

        feedback_.sequence.clear();
        feedback_.sequence.push_back(-1);
        as_.publishFeedback(feedback_);

        if (success)
        {
            vel_msg.linear_vel = 0;
            vel_msg.angular_vel = 0;
            vel_pub.publish(vel_msg);

            result_.result = true;
            ROS_INFO("%s: Succeeded", action_name_.c_str());
        }
        else
        {
            result_.result = false;
        }

        as_.setSucceeded(result_);
    }
};


class myclass

int main(int argc, char **argv)
{
    ros::init(argc, argv, "linear_move_platform");
    ros::NodeHandle n;

    myclass my_class;
    ros::Rate loop_rate(200);

    while (ros::ok())
    {

        my_class.Callback();

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}