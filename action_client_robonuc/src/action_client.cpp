#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
//#include <actionlib_tutorials/FibonacciAction.h>

#include <robonuc_action/Robot_statusAction.h>
#include <iostream> 

using namespace robonuc_action;
typedef actionlib::SimpleActionClient<robonuc_action::Robot_statusAction> Client;

class MyNode
{
  public:
    MyNode() : ac("RobotStatusAction", true)
    {
        ROS_INFO("Waiting for action server to start.");
        ac.waitForServer();
        ROS_INFO("Action server started, can send goal.");
    }

    void doStuff(int order)
    {
        robonuc_action::Robot_statusGoal goal;
        goal.mode = order; //mode porque é o que defini no goal da action

        ac.sendGoal(goal);
        ac.sendGoal(goal,
                boost::bind(&MyNode::doneCb, this, _1, _2),
                Client::SimpleActiveCallback(),
                Client::SimpleFeedbackCallback());

        //wait for the action to return
        bool finished_before_timeout = ac.waitForResult(ros::Duration(15.0));

        if (finished_before_timeout)
        {
            actionlib::SimpleClientGoalState state = ac.getState();
            ROS_INFO("Action finished: %s", state.toString().c_str());

            Robot_statusResultConstPtr myresult = ac.getResult();

            ROS_INFO("Action finished, with result: %d", myresult->result);
        }
        else
            ROS_INFO("Action did not finish before the time out.");
    }

    void doneCb(const actionlib::SimpleClientGoalState &state,
                const Robot_statusResultConstPtr &result)
    {
        ROS_INFO("Finished in state [%s]", state.toString().c_str());
        // ROS_INFO("Answer: %d", result);
        ros::shutdown();
    }

  private:
    Client ac;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_action_client");

    // create the action client
    // true causes the client to spin its own thread
    //actionlib::SimpleActionClient<actionlib_tutorials::FibonacciAction> ac("fibonacci", true);

    MyNode my_node;
    std::cout << "Enter a number: "; 
    int a; // declare one variable 
    std::cin >> a; 

    my_node.doStuff(a);
    ros::spin();

    //actionlib::SimpleActionClient<robonuc_action::Robot_statusAction> ac("RobotStatusAction", true); //server name , e vaiavel booleana spin a thread

    //ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    //ac.waitForServer(); //will wait for infinite time

    //ROS_INFO("Action server started, sending goal.");
    // send a goal to the action

    //exit
    return 0;
}