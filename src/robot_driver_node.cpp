#include "robot_driver_node.hpp"



Robot::Robot(const ev3dev::address_type left, const ev3dev::address_type right,
	     const float wheelBase)
  : leftMotor(left), rightMotor(right), wheelBase(wheelBase) {
  this->leftMotor.stop();
  this->rightMotor.stop();
}



void Robot::setVelocity(const geometry_msgs::Twist vel){
  this->vel = vel;

  double x_l = vel.linear.x;
  double z_a = vel.angular.z;

  double left_sp = x_l - (1.0 - 2.0*(x_l < 0)) * z_a * this->wheelBase;
  double right_sp = x_l + (1.0 - 2.0*(x_l < 0)) * z_a * this->wheelBase;

  if(left_sp > 100.0)
    left_sp = 100.0;
  else if(left_sp < -100.0)
    left_sp = -100.0;

  if(right_sp > 100.0)
    right_sp = 100.0;
  else if(right_sp < -100.0)
    right_sp = -100.0;


  this->leftMotor.set_duty_cycle_sp(left_sp);
  this->rightMotor.set_duty_cycle_sp(right_sp);
  this->leftMotor.run_forever();
  this->rightMotor.run_forever();

  this->logVelocity();
}



void Robot::logVelocity(){
  std::stringstream ss;
  ss << "linear: ["
     << this->vel.linear.x << ", "
     << this->vel.linear.y << ", "
     << this->vel.linear.z
     << "]   "
     << "angular: ["
     << this->vel.angular.x << ", "
     << this->vel.angular.y << ", "
     << this->vel.angular.z
     << "]" << std::endl;

  ROS_INFO(ss.str().c_str());
}



int main(int argc, char **argv){
  ros::init(argc,argv,"robot",ros::init_options::AnonymousName);

  ros::NodeHandle node;

  Robot robot(ev3dev::OUTPUT_A,ev3dev::OUTPUT_D,2.25);

  std::string node_name = ros::this_node::getName();
  node_name.erase(0,1);

  ros::Subscriber cmd_vel = node.subscribe("cmd_vel_"+node_name,2,
					   &Robot::setVelocity, &robot);

  ros::spin();
}
