// Copyright 2018 Chengzhi Chen

#ifndef ZEROBOT_DRIVER_TEST_H
#define ZEROBOT_DRIVER_TEST_H

#include <zerobot_driver/plugin.h>
#include <zerobot_driver/msg.h>
#include <geometry_msgs/Twist.h>

namespace zerobot_driver
{
class Test : zerobot_driver::Plugin
{
public:
  Test(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle) : Plugin(node_handle, private_node_handle)
  {
    connectCallback(MSG_TOPIC_ENCODER, boost::bind(&Test::encoderCallback, this, _1));
    twist_subscriber_ = pnh_.subscribe("cmd_vel", 1, &Test::subTwistCallback, this);
  }

  virtual ~Test()
  {
  }

private:
  ros::Subscriber twist_subscriber_;

  void subTwistCallback(geometry_msgs::TwistConstPtr twist)
  {
    MsgWheel wheel;
    wheel.data[0] = twist->linear.x + (twist->angular.z * 0.265) / 2;
    wheel.data[1] = twist->linear.x - (twist->angular.z * 0.265) / 2;
    msgPost(MSG_TOPIC_WHEEL, &wheel, sizeof(wheel));
  }

  void encoderCallback(void* payload)
  {
    static float value[MSG_ENCODER_DATA_COUNT] = { 0 };
    MsgEncoder* encoder = reinterpret_cast<MsgEncoder*>(payload);

    for (int i = 0; i < MSG_ENCODER_DATA_COUNT; i++)
    {
      value[i] += encoder->data[i] * 0.025;
    }

    ROS_INFO("%f %f %f %f", value[0], value[1], value[2], value[3]);
  }
};

}  // namespace zerobot_driver

#endif  // ZEROBOT_DRIVER_TEST_H
