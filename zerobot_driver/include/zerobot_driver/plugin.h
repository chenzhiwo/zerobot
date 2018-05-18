// Copyright 2018 Chengzhi Chen

#ifndef ZEROBOT_DRIVER_PLUGIN_H
#define ZEROBOT_DRIVER_PLUGIN_H

#include <array>
#include <vector>
#include <boost/circular_buffer.hpp>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/signals2.hpp>
#include <boost/thread.hpp>
#include <ros/ros.h>

#define BUFFER_COUNT 256
#define CALLBACK_COUNT UINT8_MAX

namespace zerobot_driver
{
class Plugin
{
public:
  typedef uint8_t TopicID;

  typedef boost::function<void(void*)> Callback;
  typedef boost::signals2::signal<void(void*)> Signal;

  class Buffer
  {
  public:
    Buffer(TopicID tid, void* payload, size_t len) noexcept
        : tid_(tid),
          payload_(boost::make_shared<std::vector<uint8_t>>(reinterpret_cast<uint8_t*>(payload),
                                                            reinterpret_cast<uint8_t*>(payload) + len))
    {
    }

    Buffer() noexcept = default;

    Buffer(const Buffer& rhs) noexcept = default;

    ~Buffer() = default;

    Buffer& operator=(const Buffer& rhs) noexcept = default;

    TopicID tid() noexcept
    {
      return tid_;
    }

    size_t len() noexcept
    {
      return payload_->size();
    }

    uint8_t* payload() noexcept
    {
      return payload_->data();
    }

  private:
    TopicID tid_;
    boost::shared_ptr<std::vector<uint8_t>> payload_;
  };

  Plugin(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle) : nh_(node_handle), pnh_(private_node_handle)
  {
  }

  virtual ~Plugin()
  {
    for (auto& c : connections_)
    {
      c.disconnect();
    }
  }

protected:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::vector<boost::signals2::connection> connections_;

  static std::array<Signal, CALLBACK_COUNT> callbacks_;

  static boost::mutex buffer_mutex_;
  static boost::condition_variable buffer_cv_;
  static boost::circular_buffer<Buffer> buffer_;

  bool connectCallback(TopicID tid, Callback callback)
  {
    connections_.push_back(callbacks_[tid].connect(callback));
    return true;
  }

  bool invokeCallback(TopicID tid, void* payload)
  {
    if (callbacks_[tid].empty())
    {
      return false;
    }
    callbacks_[tid](payload);
    return true;
  }

  bool msgPost(TopicID tid, void *payload, size_t len)
  {
    boost::unique_lock<boost::mutex> lock(buffer_mutex_);
    buffer_.push_back(boost::move(Buffer(tid, payload, len)));
    lock.unlock();
    buffer_cv_.notify_one();
    return true;
  }

  bool msgPop(Buffer &buffer)
  {
    boost::unique_lock<boost::mutex> lock(buffer_mutex_);
    while (buffer_.empty())
    {
      buffer_cv_.wait(lock);
    }
    buffer = boost::move(buffer_.front());
    buffer_.pop_front();
    lock.unlock();
    return true;
  }
};

}  // namespace zerobot_driver

#endif  // ZEROBOT_DRIVER_PLUGIN_H
