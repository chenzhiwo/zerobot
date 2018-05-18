// Copyright 2018 Chengzhi Chen

#include <boost/shared_ptr.hpp>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <zerobot_driver/diff_drive.h>
#include <zerobot_driver/logger.h>
#include <zerobot_driver/odometer.h>
#include <zerobot_driver/params.h>
#include <zerobot_driver/ranger.h>
#include <zerobot_driver/serial.h>
#include <zerobot_driver/test.h>
#include <zerobot_driver/twister.h>

namespace zerobot_nodelet
{
class DiffDrive : public nodelet::Nodelet
{
public:
  DiffDrive()
  {
  }

  ~DiffDrive()
  {
  }

private:
  boost::shared_ptr<zerobot_driver::DiffDrive> ptr_;

  virtual void onInit()
  {
    ptr_ = boost::make_shared<zerobot_driver::DiffDrive>(getMTNodeHandle(), getMTPrivateNodeHandle());
  }
};

class Logger : public nodelet::Nodelet
{
public:
  Logger()
  {
  }

  ~Logger()
  {
  }

private:
  boost::shared_ptr<zerobot_driver::Logger> ptr_;

  virtual void onInit()
  {
    ptr_ = boost::make_shared<zerobot_driver::Logger>(getMTNodeHandle(), getMTPrivateNodeHandle());
  }
};

class Odometer : public nodelet::Nodelet
{
public:
  Odometer()
  {
  }

  ~Odometer()
  {
  }

private:
  boost::shared_ptr<zerobot_driver::Odometer> ptr_;

  virtual void onInit()
  {
    ptr_ = boost::make_shared<zerobot_driver::Odometer>(getMTNodeHandle(), getMTPrivateNodeHandle());
  }
};

class Params : public nodelet::Nodelet
{
public:
  Params()
  {
  }

  ~Params()
  {
  }

private:
  boost::shared_ptr<zerobot_driver::Params> ptr_;

  virtual void onInit()
  {
    ptr_ = boost::make_shared<zerobot_driver::Params>(getMTNodeHandle(), getMTPrivateNodeHandle());
  }
};

class Ranger : public nodelet::Nodelet
{
public:
  Ranger()
  {
  }

  ~Ranger()
  {
  }

private:
  boost::shared_ptr<zerobot_driver::Ranger> ptr_;

  virtual void onInit()
  {
    ptr_ = boost::make_shared<zerobot_driver::Ranger>(getMTNodeHandle(), getMTPrivateNodeHandle());
  }
};

class Serial : public nodelet::Nodelet
{
public:
  Serial()
  {
  }

  ~Serial()
  {
  }

private:
  boost::shared_ptr<zerobot_driver::Serial> ptr_;

  virtual void onInit()
  {
    ptr_ = boost::make_shared<zerobot_driver::Serial>(getMTNodeHandle(), getMTPrivateNodeHandle());
  }
};

class Test : public nodelet::Nodelet
{
public:
  Test()
  {
  }

  ~Test()
  {
  }

private:
  boost::shared_ptr<zerobot_driver::Test> ptr_;

  virtual void onInit()
  {
    ptr_ = boost::make_shared<zerobot_driver::Test>(getMTNodeHandle(), getMTPrivateNodeHandle());
  }
};

class Twister : public nodelet::Nodelet
{
public:
  Twister()
  {
  }

  ~Twister()
  {
  }

private:
  boost::shared_ptr<zerobot_driver::Twister> ptr_;

  virtual void onInit()
  {
    ptr_ = boost::make_shared<zerobot_driver::Twister>(getMTNodeHandle(), getMTPrivateNodeHandle());
  }
};

}  // namespace zerobot_nodelet

PLUGINLIB_EXPORT_CLASS(zerobot_nodelet::DiffDrive, nodelet::Nodelet)
PLUGINLIB_EXPORT_CLASS(zerobot_nodelet::Logger, nodelet::Nodelet)
PLUGINLIB_EXPORT_CLASS(zerobot_nodelet::Odometer, nodelet::Nodelet)
PLUGINLIB_EXPORT_CLASS(zerobot_nodelet::Params, nodelet::Nodelet)
PLUGINLIB_EXPORT_CLASS(zerobot_nodelet::Ranger, nodelet::Nodelet)
PLUGINLIB_EXPORT_CLASS(zerobot_nodelet::Serial, nodelet::Nodelet)
PLUGINLIB_EXPORT_CLASS(zerobot_nodelet::Test, nodelet::Nodelet)
PLUGINLIB_EXPORT_CLASS(zerobot_nodelet::Twister, nodelet::Nodelet)
