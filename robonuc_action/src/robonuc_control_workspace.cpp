#include "ros/ros.h"
#include <iostream>

#include "std_msgs/Int8.h" //for /referee_mode

#include <actionlib/server/simple_action_server.h>
#include <robonuc_action/Robot_statusAction.h>

// typedef actionlib::SimpleActionServer<robonuc_action::Robot_statusAction> Server_1;

// using namespace std;

class Robot_statusAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<robonuc_action::Robot_statusAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  robonuc_action::Robot_statusFeedback feedback_;
  robonuc_action::Robot_statusResult result_;

public:

  Robot_statusAction(std::string name) :
    as_(nh_, name, boost::bind(&Robot_statusAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~Robot_statusAction(void)
  {
  }

  void executeCB(const robonuc_action::Robot_statusGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;

    // push_back the seeds for the Robot_status sequence
    feedback_.sequence.clear();
    feedback_.sequence.push_back(0);
    feedback_.sequence.push_back(1);

    // publish info to the console for the user
    ROS_INFO("%s: Executing, creating Robot_status sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);

    // start executing the action
    for(int i=1; i<=goal->order; i++)
    {
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok() )
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
      }
      feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);
      // publish the feedback
      as_.publishFeedback(feedback_);
      // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
      r.sleep();
    }

    if(success)
    {
      result_.sequence = feedback_.sequence;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }
};

int main(int argc, char **argv) {


  ros::init(argc, argv, "robonuc_control_workspace");
  ros::NodeHandle n;

  Robot_statusAction robonuc_control_workspace("robonuc_control_workspace");

  // Server server(n, "mode_1", boost::bind(&execute, _1, &server), false);
  // server.start();
  ros::spin();
  return 0;

}