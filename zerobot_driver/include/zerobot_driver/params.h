// Copyright 2018 Chengzhi Chen

#ifndef ZEROBOT_DRIVER_PARAMS_H
#define ZEROBOT_DRIVER_PARAMS_H

#include <unordered_map>
#include <string>
#include <ros/ros.h>
#include <zerobot_driver/plugin.h>
#include <zerobot_driver/msg.h>
#include <zerobot_msgs/ParamSet.h>
#include <zerobot_msgs/ParamGet.h>
#include <zerobot_msgs/ParamPull.h>
#include <zerobot_msgs/ParamPush.h>

namespace zerobot_driver
{
class Params : zerobot_driver::Plugin
{
public:
  enum Type
  {
    PARAM_INT = 0,
    PARAM_FLOAT
  };

  typedef union
  {
    int32_t i;
    float f;
  }
  Value;

  class Parameter
  {
  public:
    int id;
    enum Type type;
    Value value;
    std::string name;
  };

  Params(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle) : Plugin(node_handle, private_node_handle)
  {
    pnh_.param<int>("service_timeout", service_timeout_, 100);

      connectCallback(MSG_TOPIC_PARAM_INFO, boost::bind(&Params::msgParamInfoCallback, this, _1));

    get_server_ = pnh_.advertiseService("get", &Params::srvGetCallback, this);
    set_server_ = pnh_.advertiseService("set", &Params::srvSetCallback, this);
    pull_server_ = pnh_.advertiseService("pull", &Params::srvPullCallback, this);
    push_server_ = pnh_.advertiseService("push", &Params::srvPushCallback, this);

    MsgParam param;
    param.action = MSG_PARAM_ACTION_PULL;
      msgPost(MSG_TOPIC_PARAM, &param, sizeof(param));
  }

  virtual ~Params()
  {
  }

private:
  int service_timeout_;

  boost::mutex params_mutex_;
  boost::condition_variable params_cv_;
  std::unordered_map<int, Parameter> params_;

  ros::ServiceServer get_server_;
  ros::ServiceServer set_server_;
  ros::ServiceServer pull_server_;
  ros::ServiceServer push_server_;

  void msgParamInfoCallback(void* payload)
  {
    MsgParamInfo* param_info = reinterpret_cast<MsgParamInfo*>(payload);

    int id = param_info->id;

    Parameter p;
    p.id = id;
    p.name = std::string(reinterpret_cast<char*>(param_info->name));
    p.type = static_cast<enum Type>(param_info->type);

    if (param_info->type == PARAM_INT)
    {
      p.value.i = param_info->i;

      boost::unique_lock<boost::mutex> lock(params_mutex_);
      params_[id] = p;
      lock.unlock();
      params_cv_.notify_one();

      pnh_.setParam(p.name, p.value.i);
    }
    else if (param_info->type == PARAM_FLOAT)
    {
      p.value.f = param_info->f;

      boost::unique_lock<boost::mutex> lock(params_mutex_);
      params_[id] = p;
      lock.unlock();
      params_cv_.notify_one();

      pnh_.setParam(p.name, p.value.f);
    }
  }

  bool srvGetCallback(zerobot_msgs::ParamGetRequest& request, zerobot_msgs::ParamGetResponse& response)
  {
    MsgParam param;
    int id = request.id;

    param.action = MSG_PARAM_ACTION_GET;
    param.id = id;
      msgPost(MSG_TOPIC_PARAM, &param, sizeof(param));

    // Wait for param update.
    boost::unique_lock<boost::mutex> lock(params_mutex_);
    if (params_cv_.wait_for(lock, boost::chrono::milliseconds(service_timeout_)) == boost::cv_status::timeout)
    {
      return false;
    }
    auto p = params_[id];
    lock.unlock();

    response.id = id;
    if (p.type == PARAM_INT)
    {
      response.i = p.value.i;
    }
    else if (p.type == PARAM_FLOAT)
    {
      response.f = p.value.f;
    }
    return true;
  }

  bool srvSetCallback(zerobot_msgs::ParamSetRequest& request, zerobot_msgs::ParamSetResponse& response)
  {
    (void)response;
    MsgParam param;
    param.action = MSG_PARAM_ACTION_SET;
    param.id = request.id;
    param.i = request.i;
    param.f = request.f;
      msgPost(MSG_TOPIC_PARAM, &param, sizeof(param));
    // Wait for param update.
    boost::unique_lock<boost::mutex> lock(params_mutex_);
    if (params_cv_.wait_for(lock, boost::chrono::milliseconds(service_timeout_)) == boost::cv_status::timeout)
    {
      return false;
    }
    return true;
  }

  bool srvPullCallback(zerobot_msgs::ParamPullRequest& request, zerobot_msgs::ParamPullResponse& response)
  {
    (void)request;
    (void)response;
    MsgParam param;
    param.action = MSG_PARAM_ACTION_PULL;
      msgPost(MSG_TOPIC_PARAM, &param, sizeof(param));
    // Wait for param update.
    boost::unique_lock<boost::mutex> lock(params_mutex_);
    if (params_cv_.wait_for(lock, boost::chrono::milliseconds(service_timeout_)) == boost::cv_status::timeout)
    {
      return false;
    }
    return true;
  }

  bool srvPushCallback(zerobot_msgs::ParamPushRequest& request, zerobot_msgs::ParamPushResponse& response)
  {
    (void)request;
    (void)response;
    boost::unique_lock<boost::mutex> lock(params_mutex_);

    for (auto& p : params_)
    {
      if (!pnh_.hasParam(p.second.name))
      {
        continue;
      }

      if (p.second.type == PARAM_INT)
      {
        pnh_.getParam(p.second.name, p.second.value.i);
      }
      else if (p.second.type == PARAM_FLOAT)
      {
        pnh_.getParam(p.second.name, p.second.value.f);
      }
    }

    MsgParam param;
    for (auto& p : params_)
    {
      param.action = MSG_PARAM_ACTION_SET;
      param.id = p.second.id;

      if (p.second.type == PARAM_INT)
      {
        param.i = p.second.value.i;
      }
      else if (p.second.type == PARAM_FLOAT)
      {
        param.f = p.second.value.f;
      }
        msgPost(MSG_TOPIC_PARAM, &param, sizeof(param));

      if (params_cv_.wait_for(lock, boost::chrono::milliseconds(service_timeout_)) == boost::cv_status::timeout)
      {
        return false;
      }
    }

    return true;
  }
};

}  // namespace zerobot_driver

#endif  // ZEROBOT_DRIVER_PARAMS_H
