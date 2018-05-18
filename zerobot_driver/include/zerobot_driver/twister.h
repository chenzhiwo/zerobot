// Copyright 2018 Chengzhi Chen

#ifndef ZEROBOT_DRIVER_TWISTER_H
#define ZEROBOT_DRIVER_TWISTER_H

#include <zerobot_driver/plugin.h>
#include <zerobot_driver/msg.h>
#include <geometry_msgs/Twist.h>

namespace zerobot_driver
{
class Twister : zerobot_driver::Plugin
{
public:
  Twister(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle) : Plugin(node_handle, private_node_handle)
  {
    twist_subscriber_ = pnh_.subscribe("cmd_vel", 1, &Twister::subTwistCallback, this);
  }

  virtual ~Twister()
  {
  }

private:
  ros::Subscriber twist_subscriber_;

  void subTwistCallback(geometry_msgs::TwistConstPtr twist)
  {
    MsgTwist msg;

    msg.linear_x = twist->linear.x;
    msg.linear_y = twist->linear.y;
    msg.angular_z = twist->angular.z;

    msgPost(MSG_TOPIC_CMD_VEL, &msg, sizeof(msg));
  }
};

}  // namespace zerobot_driver

#endif  // ZEROBOT_DRIVER_TWISTER_H
