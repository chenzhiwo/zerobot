// Copyright 2018 Chengzhi Chen

#ifndef ZEROBOT_DRIVER_ODOMETER_H
#define ZEROBOT_DRIVER_ODOMETER_H

#include <string>
#include <zerobot_driver/plugin.h>
#include <zerobot_driver/msg.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

namespace zerobot_driver
{
class Odometer : zerobot_driver::Plugin
{
public:
  Odometer(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle) : Plugin(node_handle, private_node_handle)
  {
    pnh_.param<std::string>("frame_id", frame_id_, "base_link");

    odom_publisher_ = pnh_.advertise<nav_msgs::Odometry>("odom", 128);

    connectCallback(MSG_TOPIC_ODOM, boost::bind(&Odometer::OdometryCallback, this, _1));
  }

  virtual ~Odometer()
  {
  }

private:
  std::string frame_id_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  ros::Publisher odom_publisher_;

  void OdometryCallback(void* payload)
  {
    MsgOdometry* odometry = reinterpret_cast<MsgOdometry*>(payload);

    ros::Time now = ros::Time::now();

    tf2::Quaternion rot;
    rot.setRPY(0, 0, odometry->orientation_z);

    nav_msgs::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = "odom";
    odom.child_frame_id = frame_id_;
    odom.pose.pose.position.x = odometry->position_x;
    odom.pose.pose.position.y = odometry->position_y;
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation.x = rot.x();
    odom.pose.pose.orientation.y = rot.y();
    odom.pose.pose.orientation.z = rot.z();
    odom.pose.pose.orientation.w = rot.w();
    odom.twist.twist.linear.x = odometry->linear_x;
    odom.twist.twist.linear.y = odometry->linear_y;
    odom.twist.twist.angular.z = odometry->angular_z;
    odom_publisher_.publish(odom);

    geometry_msgs::TransformStamped transform;
    transform.header.stamp = now;
    transform.header.frame_id = "odom";
    transform.child_frame_id = frame_id_;
    transform.transform.translation.x = odometry->position_x;
    transform.transform.translation.y = odometry->position_y;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation.x = rot.x();
    transform.transform.rotation.y = rot.y();
    transform.transform.rotation.z = rot.z();
    transform.transform.rotation.w = rot.w();
    tf_broadcaster_.sendTransform(transform);
  }
};

}  // namespace zerobot_driver

#endif  // ZEROBOT_DRIVER_ODOMETER_H
