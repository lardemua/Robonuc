#include <iostream>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
// #include <actionlib_tutorials/FibonacciAction.h>
#include <robonuc_plat_orientation_action/Robot_PlatformOrientationAction.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

//#include "fiducial_msgs/Fiducial.h"
//#include "fiducial_msgs/FiducialArray.h"
//#include "fiducial_msgs/FiducialTransform.h"
#include "fiducial_msgs/FiducialTransformArray.h"
#include <vector>

#include <r_platform/navi.h> //for /navi_commands
#include <unistd.h>

class OrientationAction
{
  public:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<robonuc_plat_orientation_action::Robot_PlatformOrientationAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    std::string action_name_;

    // create messages that are used to published feedback/result
    robonuc_plat_orientation_action::Robot_PlatformOrientationFeedback feedback_;
    robonuc_plat_orientation_action::Robot_PlatformOrientationResult result_;

    tf::TransformBroadcaster br;
    tf::TransformListener listener;

    fiducial_msgs::FiducialTransformArray fta;
    ros::Subscriber transform_sub;

    tf::Transform baselink_T_fobject;
    tf::Transform camera_T_fobject;
    tf::StampedTransform baselink_T_camera;

    r_platform::navi vel_msg;
    ros::Publisher vel_pub;

    OrientationAction(std::string name) : as_(nh_, name, boost::bind(&OrientationAction::executeCB, this, _1), false),
                                          action_name_(name)
    {
        transform_sub = nh_.subscribe("/fiducial_transforms", 1,
                                      &OrientationAction::fiducialtransformCB, this);

        vel_pub = nh_.advertise<r_platform::navi>("/navi_commands", 3);

        as_.start();
    }

    ~OrientationAction(void)
    {
    }

    void fiducialtransformCB(const fiducial_msgs::FiducialTransformArray::ConstPtr &msg)
    {
        //float x= msg->transform.translation.x ;
        // std::cout << "CALLBACK fiducial transform" << msg->transforms[0].transform.translation.x << std::endl;
        if (msg->transforms.size() == 0)
        {
            return;
        }

        float x = msg->transforms[0].transform.translation.x;
        float y = msg->transforms[0].transform.translation.y;
        float z = msg->transforms[0].transform.translation.z;

        float xr = msg->transforms[0].transform.rotation.x;
        float yr = msg->transforms[0].transform.rotation.y;
        float zr = msg->transforms[0].transform.rotation.z;
        float wr = msg->transforms[0].transform.rotation.w;

        tf::Quaternion q(xr, yr, zr, wr);
        // tf::Matrix3x3 m(q);
        // double roll, pitch, yaw;
        // m.getRPY(roll, pitch, yaw);

        // std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;

        camera_T_fobject.setOrigin(tf::Vector3(x, y, z));
        // tf::Quaternion q;
        // q.setRPY(0, 0, a);
        camera_T_fobject.setRotation(q);

        br.sendTransform(tf::StampedTransform(camera_T_fobject, ros::Time::now(), "camera_rgb_optical_frame", "fiducial_object"));
    }

    void executeCB(const robonuc_plat_orientation_action::Robot_PlatformOrientationGoalConstPtr &goal)
    {
        ros::Rate r(0.2);
        // helper variables
        float velocidade = 0.035;
        bool success = true;
        float x, y, z;
        double th_R = 0.100;
        double th_P = 0.100;
        double th_Y = 0.100;

        bool orientation_not_ok = true;

        // listener.lookupTransform("fiducial_object", "robot_base_link", ros::Time(0), baselink_T_fobject);
        // listener.waitForTransform("fiducial_object", "robot_base_link", ros::Time::now(),ros::Duration(1.0));
        // listener.lookupTransform("fiducial_object", "robot_base_link", ros::Time(0), baselink_T_fobject);
        ROS_INFO("Goal recived");
        while (orientation_not_ok)
        {
            ROS_INFO("Executing orientation");
            if (as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("%s: Preempted", action_name_.c_str());
                // set the action state to preempted
                as_.setPreempted();
                success = false;
                break;
            }

            try
            {
                listener.lookupTransform("robot_base_link", "camera_rgb_optical_frame", ros::Time(0), baselink_T_camera);

                baselink_T_fobject = baselink_T_camera * camera_T_fobject;
                x = baselink_T_fobject.getOrigin().x();
                y = baselink_T_fobject.getOrigin().y();
                z = baselink_T_fobject.getOrigin().z();
                std::cout << "x,y,z= " << x << ", " << y << ", " << z << std::endl;

                // th_Y = tf::getYaw(baselink_T_fobject.getRotation());

                tf::Quaternion q = baselink_T_fobject.getRotation();
                tf::Matrix3x3 matrix(q);
                matrix.getRPY(th_R, th_P, th_Y);

                // std::cout << "YAW= " << th << std::endl;
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s", ex.what());
                ros::Duration(0.1).sleep();
                success = false;
                break;
            }

            // push_back the seeds for the Robot_PlatformOrientation sequence
            feedback_.sequence.clear();

            // feedback_.sequence.push_back(y);

            ROS_INFO("Executing, PlatformOrientaion, x=%f, y=%f, z=%f yaw=%f, roll=%f, pitch=%f", x, y, z, th_Y, th_R, th_P);
            usleep(5000);
            if (th_Y >= 0.05)
            {
                orientation_not_ok = true;
                feedback_.sequence.push_back(1);
                vel_msg.linear_vel = 0;
                // vel_msg.angular_vel = 0.025;
                vel_msg.angular_vel = velocidade;
                ROS_INFO("th>0.15");
                vel_pub.publish(vel_msg);
            }
            else if (th_Y <= -0.05)
            {
                orientation_not_ok = true;
                feedback_.sequence.push_back(1);
                vel_msg.linear_vel = 0;
                // vel_msg.angular_vel = -0.025;
                vel_msg.angular_vel = -velocidade;
                ROS_INFO("th < -0.15");
                vel_pub.publish(vel_msg);
            }
            else
            {
                vel_msg.linear_vel = 0;
                vel_msg.angular_vel = 0;
                vel_pub.publish(vel_msg);
                orientation_not_ok = false;
                feedback_.sequence.push_back(10);
                // orientation_not_ok=true;
            }

            as_.publishFeedback(feedback_);
        }

        if (success)
        {
            vel_msg.linear_vel = 0;
            vel_msg.angular_vel = 0;
            vel_pub.publish(vel_msg);

            result_.result = true; //feedback_.sequence;
            //ROS_INFO("%s: Succeeded", action_name_.c_str());
            // set the action state to succeeded
        }
        else
        {
            result_.result = false;
        }
        as_.setSucceeded(result_);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "GetPlatformOrientation");

    OrientationAction PlatformOrientation("GetPlatformOrientation");
    ros::Rate r(20); //20hz
    ros::spin();

    return 0;
}