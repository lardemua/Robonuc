#include "ros/ros.h"
#include <iostream>

#include "std_msgs/Int8.h" //for /referee_mode

// #include "action_srv/Robot_status.h" //srv

//moveit includes
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

//

//using namespace std;

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

// bool Robot_Status_Callback(action_srv::Robot_status::Request &req,
//                            action_srv::Robot_status::Response &res)
// {
//   res.result = false;

//   ROS_INFO("I heard: [%ld]", req.mode);

//   if (req.mode == 0) //OFF MODE
//   {
//     res.result = true;
//   }
//   else if (req.mode == 1) //Navigation Mode
//   {
//     res.result = true;
//   }
//   else if (req.mode == 2) //Aprox Laser Mode
//   {
//     res.result = true;
//   }
//   else if (req.mode == 3) //Aprox Orientacao camara
//   {
//     res.result = true;
//   }
//   else if (req.mode == 4) //Picking mode
//   {
//     res.result = true;
//   }
//   else
//   {
//     res.result = false;
//   }

//   return true;
// }

int main(int argc, char **argv)
{

  ros::init(argc, argv, "robonuc_control_workspace_service");

  ros::NodeHandle n;

  //class checker
  class_robot my_class_robot;

  // ros::Subscriber referee_mode_sub = n.subscribe("/referee_mode", 100, &checker::chatterCallback, &my_checker);

  // ros::ServiceServer service = n.advertiseService("Robot_status", Robot_Status_Callback);

  const std::string PLANNING_GROUP = "manipulator";

  // moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // moveit::planning_interface::MoveGroupInterface 

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  // const robot_state::JointModelGroup *joint_model_group =
  //     move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // RobotState is the object that contains all the current position/velocity/acceleration data.
  // moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  //
  // Next get the current set of joint values for the group.
  // std::vector<double> joint_group_positions;
  // current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
  // joint_group_positions[0] = -1.0; // radians
  // move_group.setJointValueTarget(joint_group_positions);

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