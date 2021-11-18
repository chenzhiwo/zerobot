// Copyright 2018 Chengzhi Chen

#ifndef ZEROBOT_PLUGIN_BRIDGE_H
#define ZEROBOT_PLUGIN_BRIDGE_H

#include <vector>
#include <boost/circular_buffer.hpp>
#include <boost/thread.hpp>
#include <asio.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/Twist.h>

#define START_SIGN 0x55

#define MSG_IMU_TOPIC 1
#define MSG_CONTROL_TOPIC 2

namespace zerobot_plugin
{
class Bridge
{
public:
  typedef uint8_t TopicID;

#pragma pack(push)
#pragma pack(1)
  typedef struct
  {
    uint8_t start;
    uint8_t topic_id;
    uint8_t length;
    uint8_t checksum;
  }
  NetworkHeader;

  typedef struct
  {
    double time_stamp;
    float gyr_x;
    float gyr_y;
    float gyr_z;
    float acc_x;
    float acc_y;
    float acc_z;
    float mag_x;
    float mag_y;
    float mag_z;
  }
  IMUBuffer;

  typedef struct
  {
    float linear_x;
    float angular_z;
  }
  CmdVelMsg;
#pragma pack(pop)

  class Buffer
  {
  public:
    Buffer(TopicID tid, void* payload, size_t len) noexcept
      : tid_(tid)
      , payload_(boost::make_shared<std::vector<uint8_t>>(reinterpret_cast<uint8_t*>(payload),
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

  Bridge(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)
    : nh_(node_handle), pnh_(private_node_handle), buffer_(128)
  {
    twist_publisher_ = pnh_.advertise<geometry_msgs::Twist>("cmd_vel", 128);
    imu_subscriber_ = pnh_.subscribe("imu", 128, &Bridge::imuCallback, this);
    magnet_subscriber_ = pnh_.subscribe("magnet", 128, &Bridge::magnetCallback, this);

    // Fire up all threads.
    network_thread_ = boost::thread(&Bridge::networkThread, this);
    control_thread_ = boost::thread(&Bridge::controlThread, this);
  }

  ~Bridge()
  {
    control_thread_.interrupt();
    control_thread_.join();
    network_thread_.interrupt();
    network_thread_.join();
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  boost::mutex buffer_mutex_;
  boost::condition_variable buffer_cv_;
  boost::circular_buffer<Buffer> buffer_;

  ros::Publisher twist_publisher_;
  ros::Subscriber imu_subscriber_;
  ros::Subscriber magnet_subscriber_;

  asio::io_service io_srv_;

  ros::Time start_time_;

  IMUBuffer imu_buffer_;
  boost::thread network_thread_;
  boost::thread control_thread_;

  bool msgPost(TopicID tid, void* payload, size_t len)
  {
    boost::unique_lock<boost::mutex> lock(buffer_mutex_);
    buffer_.push_back(boost::move(Buffer(tid, payload, len)));
    lock.unlock();
    buffer_cv_.notify_one();
    return true;
  }

  bool msgPop(Buffer& buffer)
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

  bool msgClear()
  {
    boost::unique_lock<boost::mutex> lock(buffer_mutex_);
    buffer_.clear();
    lock.unlock();
    return true;
  }

  void imuCallback(sensor_msgs::ImuConstPtr imu)
  {
    imu_buffer_.acc_x = imu->linear_acceleration.x;
    imu_buffer_.acc_y = imu->linear_acceleration.y;
    imu_buffer_.acc_z = imu->linear_acceleration.z;
    imu_buffer_.gyr_x = imu->angular_velocity.x;
    imu_buffer_.gyr_y = imu->angular_velocity.y;
    imu_buffer_.gyr_z = imu->angular_velocity.z;
  }

  void magnetCallback(sensor_msgs::MagneticFieldConstPtr magnet)
  {
    imu_buffer_.mag_x = magnet->magnetic_field.x;
    imu_buffer_.mag_y = magnet->magnetic_field.y;
    imu_buffer_.mag_z = magnet->magnetic_field.z;
    imu_buffer_.time_stamp = (ros::Time::now() - start_time_).toSec();
    msgPost(MSG_IMU_TOPIC, &imu_buffer_, sizeof(imu_buffer_));
  }

  void networkThread()
  {
    NetworkHeader header;
    Buffer buffer;
    std::array<asio::const_buffer, 2> buffers;
    buffers[0] = asio::buffer(&header, sizeof(header));
    asio::error_code ec;
    asio::ip::tcp::acceptor acceptor(io_srv_, asio::ip::tcp::endpoint(asio::ip::tcp::v4(), 1234));
    asio::ip::tcp::socket sock(io_srv_);

    while (ros::ok())
    {
      ROS_INFO("Wait for client.");
      acceptor.accept(sock);
      start_time_ = ros::Time::now();
      msgClear();
      ROS_INFO("Client connected.");
      while (ros::ok())
      {
        if (!msgPop(buffer))
        {
          continue;
        }

        header.start = START_SIGN;
        header.topic_id = buffer.tid();
        header.length = buffer.len();
        header.checksum = header.start + header.topic_id + header.length;

        buffers[1] = asio::buffer(buffer.payload(), buffer.len());
        asio::write(sock, buffers, ec);
        if (ec)
        {
          break;
        }
      }
      sock.close();
      ROS_INFO("Client disconnected.");
    }
  }

  void controlThread()
  {
    NetworkHeader header;
    uint8_t buffer[254];
    CmdVelMsg* cmd_vel;
    geometry_msgs::Twist twist;
    asio::error_code ec;
    asio::ip::tcp::acceptor acceptor(io_srv_, asio::ip::tcp::endpoint(asio::ip::tcp::v4(), 1233));
    asio::ip::tcp::socket sock(io_srv_);

    while (ros::ok())
    {
      ROS_INFO("Wait for control client.");
      acceptor.accept(sock);
      ROS_INFO("Control client connected.");
      while (ros::ok())
      {
        asio::read(sock, asio::buffer(&header.start, sizeof(header.start)), ec);
        if (ec)
        {
          break;
        }
        if (header.start != START_SIGN)
        {
          continue;
        }

        asio::read(sock, asio::buffer(&header.start + 1, sizeof(header) - sizeof(header.start)), ec);
        if (ec)
        {
          break;
        }
        if (header.checksum != header.start + header.topic_id + header.length)
        {
          continue;
        }

        asio::read(sock, asio::buffer(&buffer, header.length), ec);
        if (ec)
        {
          break;
        }

        if (header.topic_id == MSG_CONTROL_TOPIC)
        {
          cmd_vel = reinterpret_cast<CmdVelMsg*>(buffer);
          twist.linear.x = cmd_vel->linear_x;
          twist.angular.z = cmd_vel->angular_z;
          twist_publisher_.publish(twist);
        }
      }
      sock.close();
      ROS_INFO("Control client disconnected.");
    }
  }
};  // namespace zerobot_plugin

}  // namespace zerobot_plugin

#endif  // ZEROBOT_PLUGIN_BRIDGE_H
