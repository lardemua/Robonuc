#include "ros/ros.h"
#include <iostream>

#include "std_msgs/Int8.h" //for /referee_mode

#include "action_srv/Robot_status.h"

using namespace std;

class class_robot // class checker
{
public:
  ros::NodeHandle n;
  std_msgs::Int8 referee_mode_msg;
  int referee_mode;

  class_robot()
  {
    // referee_mode = -1;
    // referee_mode_msg.data = -1;
  }

private:
};

bool Robot_Status_Callback(action_srv::Robot_status::Request &req,
                           action_srv::Robot_status::Response &res)
{
  res.result = false;

  ROS_INFO("I heard: [%ld]", req.mode);

  if (req.mode == 0) //OFF MODE
  {
    res.result = true;
  }
  else if (req.mode == 1) //Navigation Mode
  {
    res.result = true;
  }
  else if (req.mode == 2) //Aprox Laser Mode
  {
    res.result = true;
  }
  else if (req.mode == 3) //Aprox Orientacao camara
  {
    res.result = true;
  }
  else if (req.mode == 4) //Picking mode
  {
    res.result = true;
  }
  else
  {
    res.result = false;
  }
  

  return true;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "robonuc_control_workspace_service");

  ros::NodeHandle n;

  //class checker
  class_robot my_class_robot;

  // ros::Subscriber referee_mode_sub = n.subscribe("/referee_mode", 100, &checker::chatterCallback, &my_checker);

  ros::ServiceServer service = n.advertiseService("Robot_status", Robot_Status_Callback);

  ROS_INFO("Service Ready");
  ros::spin();

  // ros::Rate loop_rate(20);
  // while (ros::ok())
  // {
  //   ros::spinOnce();
  //   loop_rate.sleep();
  // }

  return 0;
}