// Copyright 2018 Chengzhi Chen

#ifndef ZEROBOT_PLUGIN_TELEOP_H
#define ZEROBOT_PLUGIN_TELEOP_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

namespace zerobot_plugin
{
class Teleop
{
public:
  Teleop(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)
    : nh_(node_handle), pnh_(private_node_handle)
  {
    twist_publisher_ = pnh_.advertise<geometry_msgs::Twist>("twist", 1);
    joy_subscriber_ = pnh_.subscribe("joy", 1, &Teleop::subJoyCallback, this);
  }

  ~Teleop()
  {
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Publisher twist_publisher_;
  ros::Subscriber joy_subscriber_;

  inline bool notEqualZero(double value)
  {
    return (value < 0.0) || (value > 0.0);
  }

  void subJoyCallback(sensor_msgs::JoyConstPtr joy)
  {
    geometry_msgs::Twist twist;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;

    twist.linear.x = 0.4 * joy->axes[1];
    twist.angular.z = 0.5 * M_PI * joy->axes[0];

    twist_publisher_.publish(twist);
  }
};

}  // namespace zerobot_plugin

#endif  // ZEROBOT_PLUGIN_TELEOP_H
