// Copyright 2018 Chengzhi Chen

#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <zerobot_plugin/wit_imu.h>

namespace zerobot_nodelet
{
class WitImu : public nodelet::Nodelet
{
public:
  WitImu()
  {
  }

  ~WitImu()
  {
  }

private:
  boost::shared_ptr<zerobot_plugin::WitImu> ptr_;

  virtual void onInit()
  {
    ptr_ = boost::make_shared<zerobot_plugin::WitImu>(getMTNodeHandle(), getMTPrivateNodeHandle());
  }
};
}  // namespace zerobot_nodelet

PLUGINLIB_EXPORT_CLASS(zerobot_nodelet::WitImu, nodelet::Nodelet)
