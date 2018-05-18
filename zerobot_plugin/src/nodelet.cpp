// Copyright 2018 Chengzhi Chen

#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <zerobot_plugin/teleop.h>

namespace zerobot_nodelet
{
class Teleop : public nodelet::Nodelet
{
public:
  Teleop()
  {
  }

  ~Teleop()
  {
  }

private:
  boost::shared_ptr<zerobot_plugin::Teleop> ptr_;

  virtual void onInit()
  {
    ptr_ = boost::make_shared<zerobot_plugin::Teleop>(getMTNodeHandle(), getMTPrivateNodeHandle());
  }
};

}  // namespace zerobot_nodelet

PLUGINLIB_EXPORT_CLASS(zerobot_nodelet::Teleop, nodelet::Nodelet)
