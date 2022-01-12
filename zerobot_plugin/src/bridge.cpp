// Copyright 2018 Chengzhi Chen

#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <zerobot_plugin/bridge.h>

namespace zerobot_nodelet
{
class Bridge : public nodelet::Nodelet
{
public:
  Bridge()
  {
  }

  ~Bridge()
  {
  }

private:
  boost::shared_ptr<zerobot_plugin::Bridge> ptr_;

  virtual void onInit()
  {
    ptr_ = boost::make_shared<zerobot_plugin::Bridge>(getMTNodeHandle(), getMTPrivateNodeHandle());
  }
};
}  // namespace zerobot_nodelet

PLUGINLIB_EXPORT_CLASS(zerobot_nodelet::Bridge, nodelet::Nodelet)
