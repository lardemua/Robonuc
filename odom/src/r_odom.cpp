#include "std_msgs/String.h"
#include <comm_tcp/pid.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

class OdometryPublisher
{
public:
  struct Params
  {
    int msg_freq;
    int p_turn;
    float length;
    float radius;
  };

  struct Position
  {
    double x = 0;
    double y = 0;
    double th = 0;
  };

  OdometryPublisher(std::string pid_topic, std::string odom_topic,
                    Params params);

protected:
  void pidCallback(const comm_tcp::pid::ConstPtr &msg);

  bool getDistance();

private:
  ros::NodeHandle _nh;

  ros::Subscriber _pid_sub;
  ros::Publisher _odom_pub;
  tf::TransformBroadcaster _odom_broadcaster;

  tf::TransformListener _listener;

  Params _params;
  Position _position;

  ros::Time _last_time;
};

OdometryPublisher::OdometryPublisher(std::string pid_topic,
                                     std::string odom_topic, Params params)
    : _pid_sub(_nh.subscribe<comm_tcp::pid>(
          pid_topic, 2, &OdometryPublisher::pidCallback, this)),
      _odom_pub(_nh.advertise<nav_msgs::Odometry>(odom_topic, 2)),
      _params(params), _last_time(ros::Time::now()) {}

void OdometryPublisher::pidCallback(const comm_tcp::pid::ConstPtr &msg)
{
  int pulse1, pulse2;
  float xR, xL, dx, dy, dth;
  double vx, vy, vth;

  pulse1 = msg->encoder_read1;
  pulse2 = msg->encoder_read2;

  ros::Time current_time = ros::Time::now();

  /*
    vR_rps=pulse1 * _params.msg_freq;
    vL_rps=pulse2 * _params.msg_freq;

    vR=(vR_rps/_params.p_turn) * 2.0 * M_PI;
    vL=(vL_rps/_params.p_turn) * 2.0 * M_PI;

    vx = _params.radius / 2.0 * (vR + vL);
    vth = _params.radius / _params.length * (vR - vL);
    vy = 0.0;

    ROS_INFO("vx = %f vth = %f dif=%d",vx,vth,pulse1-pulse2);
    ros::Time current_time = ros::Time::now();
    double dt = (current_time - _last_time).toSec();
    double delta_x = (vx * cos(_position.th) - vy * sin(_position.th)) * dt;
    double delta_y = (vx * sin(_position.th) + vy * cos(_position.th)) * dt;
    double delta_th = vth * dt;

    _last_time = current_time;*/

  xR = (float)pulse1 / _params.p_turn * 2.0f * M_PI;
  xL = (float)pulse2 / _params.p_turn * 2.0f * M_PI;

  dx = _params.radius / 2.0f * (xR + xL);
  dy = 0.0;
  dth = _params.radius / _params.length * (xR - xL);

  double delta_x = dx * cos(_position.th) - dy * sin(_position.th);
  double delta_y = dx * sin(_position.th) + dy * cos(_position.th);
  double delta_th = dth;
  double dt = (current_time - _last_time).toSec();

  //  ROS_INFO("pulse1 = %d;p_turn = %d; xR=%f",pulse1, _params.p_turn,xR );

  vx = delta_x / dt;
  vy = delta_y / dt;
  vth = delta_th / dt;

  _last_time = current_time;

  _position.x += delta_x;
  _position.y += delta_y;
  _position.th += delta_th;

  if (_listener.frameExists("/scanmatcher_frame"))
  {

    if (getDistance())
    {

      tf::StampedTransform transform;
      try
      {
        _listener.lookupTransform("/map", "/scanmatcher_frame", ros::Time(0),
                                  transform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
      }

      _position.x = transform.getOrigin().x();
      _position.y = transform.getOrigin().y();
      _position.th = tf::getYaw(transform.getRotation());
    }
  }

  // ROS_INFO("th=%f",_position.th);
  // since all odometry is 6DOF we'll need a quaternion created from yaw
  geometry_msgs::Quaternion odom_quat =
      tf::createQuaternionMsgFromYaw(_position.th);

  // first, we'll publish the transform over tf
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "odom_wheel";
  odom_trans.child_frame_id = "odom_frame";

  odom_trans.transform.translation.x = _position.x;
  odom_trans.transform.translation.y = _position.y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  // send the transform
  _odom_broadcaster.sendTransform(odom_trans);

  // next, we'll publish the odometry message over ROS
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom_wheel";

  // set the position
  odom.pose.pose.position.x = _position.x;
  odom.pose.pose.position.y = _position.y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  // set the velocity
  odom.child_frame_id = "odom_frame";
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.angular.z = vth;

  // publish the message
  _odom_pub.publish(odom);
}

bool OdometryPublisher::getDistance()
{

  tf::StampedTransform transform;
  try
  {
    _listener.lookupTransform("/scanmatcher_frame", "/odom_frame", ros::Time(0),
                              transform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }

  double distance = sqrt(transform.getOrigin().y() * transform.getOrigin().y() +
                         transform.getOrigin().x() * transform.getOrigin().x());
  if (distance > 0.1)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;

  OdometryPublisher::Params params;

  n.getParam("/msg_freq", params.msg_freq);
  n.getParam("/wheel_radius", params.radius);
  n.getParam("/pulses_1turn", params.p_turn);
  n.getParam("/base_length", params.length);

  ROS_INFO("\nmsg freq = %d \np_turn = %d\nlength = %lf\nradius = %f",
           params.msg_freq, params.p_turn, params.length, params.radius);

  OdometryPublisher op("pid_data", "odom_wheel", params);

  ros::spin();
}

///////////////////////////////////////////////////////////////////////////////////////

/*
int main2(int argc, char **argv) {
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;




  tf::TransformBroadcaster odom_broadcaster;//DN



  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(1.0);
  while (n.ok()) {

    ros::spinOnce(); // check for incoming messages
    current_time = ros::Time::now();

    // compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    // since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    // first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    // send the transform
    odom_broadcaster.sendTransform(odom_trans);

    // next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    // set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    // set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    // publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }
}
*/
