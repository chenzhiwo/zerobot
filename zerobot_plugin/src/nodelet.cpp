// Copyright 2018 Chengzhi Chen

#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <zerobot_plugin/bridge.h>
#include <zerobot_plugin/teleop.h>
#include <zerobot_plugin/wit_imu.h>

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
  boost::shared_ptr<zerobot_plugin::Teleop> ptr_;

  virtual void onInit()
  {
    ptr_ = boost::make_shared<zerobot_plugin::Teleop>(getMTNodeHandle(), getMTPrivateNodeHandle());
  }
};

}  // namespace zerobot_nodelet

PLUGINLIB_EXPORT_CLASS(zerobot_nodelet::Bridge, nodelet::Nodelet)
PLUGINLIB_EXPORT_CLASS(zerobot_nodelet::Teleop, nodelet::Nodelet)
PLUGINLIB_EXPORT_CLASS(zerobot_nodelet::WitImu, nodelet::Nodelet)
