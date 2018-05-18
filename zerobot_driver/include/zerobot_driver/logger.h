// Copyright 2018 Chengzhi Chen

#ifndef ZEROBOT_DRIVER_LOGGER_H
#define ZEROBOT_DRIVER_LOGGER_H

#include <zerobot_driver/plugin.h>
#include <zerobot_driver/msg.h>

namespace zerobot_driver
{
class Logger : zerobot_driver::Plugin
{
public:
  Logger(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle) : Plugin(node_handle, private_node_handle)
  {
      connectCallback(MSG_TOPIC_LOG, boost::bind(&Logger::logCallback, this, _1));
  }

  virtual ~Logger()
  {
  }

private:
  void logCallback(void* payload)
  {
    MsgLog* log = reinterpret_cast<MsgLog*>(payload);

    switch (log->level)
    {
      case MSG_LOG_DEBUG:
        ROS_DEBUG("%s", reinterpret_cast<const char*>(log->info));
        break;

      case MSG_LOG_INFO:
        ROS_INFO("%s", reinterpret_cast<const char*>(log->info));
        break;

      case MSG_LOG_WARN:
        ROS_WARN("%s", reinterpret_cast<const char*>(log->info));
        break;

      case MSG_LOG_ERROR:
        ROS_ERROR("%s", reinterpret_cast<const char*>(log->info));
        break;

      case MSG_LOG_FATAL:
        ROS_FATAL("%s", reinterpret_cast<const char*>(log->info));
        break;

      default:
        break;
    }
  }
};

}  // namespace zerobot_driver

#endif  // ZEROBOT_DRIVER_LOGGER_H
