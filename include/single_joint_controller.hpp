#pragma once
#include "controller_interface/controller.h"
#include "hardware_interface/joint_command_interface.h"
#include "gazebo_sim.hpp"//the hardware interface of the robot
#include "gazebo_ros_control_plugin_kp.hpp"//the plugin of the gazebo_ros_control_kp
#include <pluginlib/class_list_macros.hpp>
#include <control_toolbox/pid.h>
#include <urdf/model.h>

namespace controller_ns {

class single_joint_controller:public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
  single_joint_controller();
  ~single_joint_controller();
  bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &n);
  void update(const ros::Time &time, const ros::Duration &period);
  void starting(const ros::Time& time);
  void stopping(const ros::Time& time);
  double ComputeTorqueFromPositionCommand(double command, int i, const ros::Duration& period);

private:
  std::vector<hardware_interface::JointHandle> joints_;
  std::vector<std::string> joint_names_;
  unsigned int num_joints_;

  std::vector<urdf::JointConstSharedPtr> joint_urdfs_;
  std::vector<control_toolbox::Pid> pid_controllers_;
};
}//namespace
