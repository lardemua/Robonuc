#include "ros/ros.h"
#include <iostream>

#include "std_msgs/Int8.h" //for /referee_mode

using namespace std;

class checker // class checker
{
public:
  ros::NodeHandle n;
  std_msgs::Int8 referee_mode_msg;
  int referee_mode;

  checker()
  {
    // referee_mode = -1;
    // referee_mode_msg.data = -1;
  }

  void chatterCallback(const std_msgs::Int8::ConstPtr &msg)
  {

    referee_mode_msg = (*msg);

    referee_mode = referee_mode_msg.data;
    ROS_INFO("I heard: [%d]", referee_mode);
    // cout << "[robonuc_control_workspace] referee_mode:" << referee_mode << endl;
  }

private:
};

class fanuc_arm : public checker
{
public:
  void robot_actionCallback(void)
  {

    ROS_INFO("I will put the robot on moving pos, because referee_mode is [%d]", referee_mode);
    if (referee_mode == 1)
    {
      ROS_INFO("IFFF I will put the robot on moving pos, because referee_mode is [%d]", referee_mode);
      return;
    }
    referee_mode = 50;
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
  ros::init(argc, argv, "robonuc_control_workspace");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  //class checker
  checker my_checker;
  fanuc_arm my_fanuc_arm;

  ros::Subscriber referee_mode_sub = n.subscribe("/referee_mode", 100, &checker::chatterCallback, &my_checker);

  // ros::ServiceServer service = n.advertiseService("Modo_2_Aprox_laser", add);
  ROS_INFO("Ready to add two ints.");
  ros::spin();

  // ros::Rate loop_rate(20);
  // while (ros::ok())
  // {
  //   ros::spinOnce();
  //   loop_rate.sleep();
  // }

  return 0;
}