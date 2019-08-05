#pragma once
#include "controller_interface/controller.h"
#include "hardware_interface/joint_command_interface.h"
#include "gazebo_sim.hpp"//the hardware interface of the robot
#include "gazebo_ros_control_plugin_kp.hpp"//the plugin of the gazebo_ros_control_kp
#include <pluginlib/class_list_macros.hpp>

namespace controller_ns {

class single_joint_controller:public controller_interface::Controller<hardware_interface::PositionJointInterface>
{
public:
  single_joint_controller();
  ~single_joint_controller();
  bool init(hardware_interface::PositionJointInterface *hw, ros::NodeHandle &n);
  void update(const ros::Time &time, const ros::Duration &period);
  void starting(const ros::Time& time);
  void stopping(const ros::Time& time);

private:
  std::vector<hardware_interface::JointHandle> joints_;
  std::vector<std::string> joint_names_;
  unsigned int num_joints_;

  const double gain_ = 1.25;
  const double setpoint_ = 3.00;
};
}//namespace
