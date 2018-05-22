// Copyright 2018 Chengzhi Chen

#ifndef ZEROBOT_PLUGIN_WIT_IMU_H
#define ZEROBOT_PLUGIN_WIT_IMU_H

#include <string>
#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

namespace zerobot_plugin
{
class WitImu
{
public:
#pragma pack(push)
#pragma pack(1)
  typedef struct
  {
    uint8_t start;
    uint8_t type;
    int16_t field0;
    int16_t field1;
    int16_t field2;
    int16_t field3;
    uint8_t sum;
  } Buffer;
#pragma pack(pop)

  WitImu(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)
    : nh_(node_handle), pnh_(private_node_handle), serial_(io_srv_)
  {
    pnh_.param<std::string>("serial_port", serial_port_, "/dev/ttyUSB0");
    pnh_.param<int>("baud_rate", baud_rate_, 115200);
    pnh_.param<float>("acc_fullscale", acc_fullscale_, 9.7883f * 2.0f);
    pnh_.param<float>("gyr_fullscale", gyr_fullscale_, 0.01745329251994329577f * 250.0f);

    imu_publisher_ = pnh_.advertise<sensor_msgs::Imu>("imu", 128);
    magnet_publisher_ = pnh_.advertise<sensor_msgs::MagneticField>("magnet", 128);

    serial_.open(serial_port_);
    serial_.set_option(boost::asio::serial_port::baud_rate(baud_rate_));
    serial_.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
    serial_.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
    serial_.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
    serial_.set_option(boost::asio::serial_port::character_size(8));

    recv_thread_ = boost::thread(&WitImu::recvThread, this);
  }

  ~WitImu()
  {
    recv_thread_.interrupt();
    recv_thread_.join();
    serial_.close();
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  std::string serial_port_;
  int baud_rate_;

  float acc_fullscale_;
  float gyr_fullscale_;

  ros::Publisher imu_publisher_;
  ros::Publisher magnet_publisher_;

  boost::asio::io_service io_srv_;
  boost::asio::serial_port serial_;

  boost::thread recv_thread_;

  bool checksum(Buffer* buffer)
  {
    uint8_t* ptr = reinterpret_cast<uint8_t*>(buffer);
    uint8_t sum = 0;
    for (size_t i = 0; i < sizeof(Buffer) - sizeof(Buffer::sum); i++)
    {
      sum += ptr[i];
    }
    if (sum == buffer->sum)
    {
      return true;
    }
    return false;
  }

  void recvThread()
  {
    Buffer buffer = {};
    sensor_msgs::Imu imu;
    sensor_msgs::MagneticField magnet;
    bool acc_ready = false, gyr_ready = false, orient_ready = false, mag_ready = false;

    imu.header.frame_id = "imu_link";
    magnet.header.frame_id = "imu_link";

    while (ros::ok())
    {
      boost::asio::read(serial_, boost::asio::buffer(&buffer.start, 1));
      boost::this_thread::interruption_point();
      if (buffer.start != 0x55)
      {
        continue;
      }

      boost::asio::read(serial_, boost::asio::buffer(&buffer.start + 1, sizeof(buffer) - sizeof(buffer.start)));
      boost::this_thread::interruption_point();

      if (!checksum(&buffer))
      {
        continue;
      }

      switch (buffer.type)
      {
        case 0x51:
          // acc
          imu.linear_acceleration.x = buffer.field0 * acc_fullscale_ / INT16_MAX;
          imu.linear_acceleration.y = buffer.field1 * acc_fullscale_ / INT16_MAX;
          imu.linear_acceleration.z = buffer.field2 * acc_fullscale_ / INT16_MAX;
          acc_ready = true;
          break;

        case 0x52:
          // gyr
          imu.angular_velocity.x = buffer.field0 * gyr_fullscale_ / INT16_MAX;
          imu.angular_velocity.y = buffer.field1 * gyr_fullscale_ / INT16_MAX;
          imu.angular_velocity.z = buffer.field2 * gyr_fullscale_ / INT16_MAX;
          gyr_ready = true;
          break;

        case 0x54:
          // magnet
          magnet.magnetic_field.x = buffer.field0;
          magnet.magnetic_field.y = buffer.field1;
          magnet.magnetic_field.z = buffer.field2;
          mag_ready = true;
          break;

        case 0x59:
          // quat
          imu.orientation.x = buffer.field0 / static_cast<float>(INT16_MAX);
          imu.orientation.y = buffer.field1 / static_cast<float>(INT16_MAX);
          imu.orientation.z = buffer.field2 / static_cast<float>(INT16_MAX);
          imu.orientation.w = buffer.field3 / static_cast<float>(INT16_MAX);
          orient_ready = true;
          break;

        default:
          break;
      }

      if (acc_ready && gyr_ready && orient_ready)
      {
        acc_ready = false;
        gyr_ready = false;
        orient_ready = false;
        imu.header.stamp = ros::Time::now();
        imu_publisher_.publish(imu);
      }

      if (mag_ready)
      {
        mag_ready = false;
        magnet.header.stamp = ros::Time::now();
        magnet_publisher_.publish(magnet);
      }
    }
  }
};

}  // namespace zerobot_plugin

#endif  // ZEROBOT_PLUGIN_WIT_IMU_H
