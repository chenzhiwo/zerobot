// Copyright 2018 Chengzhi Chen

#ifndef ZEROBOT_DRIVER_DIFF_DRIVE_H
#define ZEROBOT_DRIVER_DIFF_DRIVE_H

#include <string>
#include <zerobot_driver/plugin.h>
#include <zerobot_driver/msg.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

namespace zerobot_driver
{
class DiffDrive : zerobot_driver::Plugin
{
public:
  DiffDrive(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)
    : Plugin(node_handle, private_node_handle), x_(0.0), y_(0.0), theta_(0.0)
  {
    pnh_.param<std::string>("frame_id", frame_id_, "base_footprint");
    pnh_.param<float>("wheel_distance", wheel_distance_, 0.265);

    odom_publisher_ = pnh_.advertise<nav_msgs::Odometry>("odom", 128);

    twist_subscriber_ = pnh_.subscribe("cmd_vel", 1, &DiffDrive::subTwistCallback, this);

    connectCallback(MSG_TOPIC_ENCODER, boost::bind(&DiffDrive::encoderCallback, this, _1));
  }

  virtual ~DiffDrive()
  {
  }

private:
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  ros::Publisher odom_publisher_;
  ros::Subscriber twist_subscriber_;
  float wheel_distance_;
  std::string frame_id_;
  double x_, y_, theta_;

  void subTwistCallback(geometry_msgs::TwistConstPtr twist)
  {
    MsgWheel wheel;
    wheel.data[0] = twist->linear.x + (twist->angular.z * wheel_distance_) / 2;
    wheel.data[1] = twist->linear.x - (twist->angular.z * wheel_distance_) / 2;
    msgPost(MSG_TOPIC_WHEEL, &wheel, sizeof(wheel));
  }

  void encoderCallback(void* payload)
  {
    static bool first_run = true;
    static ros::Time last_time;
    MsgEncoder* encoder = reinterpret_cast<MsgEncoder*>(payload);
    ros::Time now = ros::Time::now();
    ros::Duration delta_time = now - last_time;

    if (first_run)
    {
      last_time = ros::Time::now();
      first_run = false;
      return;
    }

    // Robot coordinate.
    double vel_x = (encoder->data[0] + encoder->data[1]) / 2;
    double vel_theta = (encoder->data[0] - encoder->data[1]) / wheel_distance_;

    // Odom coordinate.
    double delta_x = vel_x * cos(theta_) * delta_time.toSec();
    double delta_y = vel_x * sin(theta_) * delta_time.toSec();
    double delta_theta = vel_theta * delta_time.toSec();

    x_ += delta_x;
    y_ += delta_y;
    theta_ += delta_theta;

    tf2::Quaternion rot;
    rot.setRPY(0, 0, theta_);

    nav_msgs::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = "odom";
    odom.child_frame_id = frame_id_;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.x = rot.x();
    odom.pose.pose.orientation.y = rot.y();
    odom.pose.pose.orientation.z = rot.z();
    odom.pose.pose.orientation.w = rot.w();
    odom.twist.twist.linear.x = vel_x;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = vel_theta;
    odom_publisher_.publish(odom);

    geometry_msgs::TransformStamped transform;
    transform.header.stamp = now;
    transform.header.frame_id = "odom";
    transform.child_frame_id = frame_id_;
    transform.transform.translation.x = x_;
    transform.transform.translation.y = y_;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation.x = rot.x();
    transform.transform.rotation.y = rot.y();
    transform.transform.rotation.z = rot.z();
    transform.transform.rotation.w = rot.w();
    tf_broadcaster_.sendTransform(transform);

    last_time = now;
  }
};

}  // namespace zerobot_driver

#endif  // ZEROBOT_DRIVER_DIFF_DRIVE_H
