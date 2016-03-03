#ifndef ROBOT_DRIVER_NODE_HPP__
#define ROBOT_DRIVER_NODE_HPP__

#include <sstream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "ev3dev.h"

namespace ev3 = ev3dev;

class Robot {
 public:
  Robot(const ev3dev::address_type left, const ev3dev::address_type right,
	const float wheelBase);

  void setVelocity(const geometry_msgs::Twist vel);
  void logVelocity();

 private:
  ev3dev::large_motor leftMotor;
  ev3dev::large_motor rightMotor;
  float wheelBase;
  geometry_msgs::Twist vel;
};


#endif
