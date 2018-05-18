// Copyright 2018 Chengzhi Chen

#ifndef ZEROBOT_DRIVER_RANGER_H
#define ZEROBOT_DRIVER_RANGER_H

#include <zerobot_driver/plugin.h>
#include <zerobot_driver/msg.h>
#include <sensor_msgs/Range.h>

namespace zerobot_driver
{
class Ranger : zerobot_driver::Plugin
{
public:
  Ranger(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle) : Plugin(node_handle, private_node_handle)
  {
    pnh_.param<float>("field_of_view", field_of_view_, 0.1);
    pnh_.param<float>("min_range", min_range_, 0.05);
    pnh_.param<float>("max_range", max_range_, 2.4);

    range_publisher_ = pnh_.advertise<sensor_msgs::Range>("range", 128);

    connectCallback(MSG_TOPIC_RANGE, boost::bind(&Ranger::rangeCallback, this, _1));
  }

  virtual ~Ranger()
  {
  }

private:
  float field_of_view_;
  float min_range_;
  float max_range_;
  ros::Publisher range_publisher_;

  void rangeCallback(void* payload)
  {
    MsgRange* range = reinterpret_cast<MsgRange*>(payload);
    sensor_msgs::Range msg;

    msg.header.stamp = ros::Time::now();
    msg.radiation_type = sensor_msgs::Range::INFRARED;
    msg.field_of_view = field_of_view_;
    msg.min_range = min_range_;
    msg.max_range = max_range_;

    for (int i = 0; i < MSG_RANGE_DATA_COUNT; i++)
    {
      msg.header.frame_id = "ranger" + std::to_string(i) + "_link";
      msg.range = range->data[i] / 10000.0;
      range_publisher_.publish(msg);
    }
  }
};

}  // namespace zerobot_driver

#endif  // ZEROBOT_DRIVER_RANGER_H
