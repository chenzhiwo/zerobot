# zerobot
zerobot is a simple and low-cost mobile robot controller.
This is the ROS driver for zerobot.

## Features
- fast zero-copy protocol.
- auto generate message definition C header from ROS msg files.
- PID controller for brushed DC motor with encoder.
- diff drive, mecanum wheel, omni wheel.
- odometer, ranger.
- dynamic parameter reconfigure and data logger.

## zerobot_driver
Robot drivers, communicate with robot.

## zerobot_msgs
ROS interfaces, providing messages descriptions.

## zerobot_plugin
ROS nodelet plugins.

## zerobot_bringup
ROS launch files.

## zerobot_robot
ROS meta package.
