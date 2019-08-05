#include "single_joint_controller.hpp"
#include "ros/ros.h"
namespace controller_ns {

single_joint_controller::single_joint_controller()
{
  ROS_INFO("Let's start");
}

single_joint_controller::~single_joint_controller()
{
  ROS_INFO("End");
}


bool single_joint_controller::init(hardware_interface::PositionJointInterface *hardware, ros::NodeHandle &node_handle)
{
  ROS_INFO("Initializing single_joint_controller");

  std::string param_name = "joints";
  if(!node_handle.getParam(param_name,joint_names_))
  {
    ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << node_handle.getNamespace() << ").");
    return false;
  }
  num_joints_ = joint_names_.size();
  if(num_joints_ == 0)
  {
    ROS_ERROR_STREAM("List of joint names is empty.");
  }
  ROS_INFO("there are %d joints", num_joints_);
  for(unsigned int i = 0; i < num_joints_; i++)
  {
    try{
      joints_.push_back(hardware->getHandle(joint_names_[i]));
      ROS_INFO("Get '%s' Handle", joint_names_[i].c_str());
    } catch (const hardware_interface::HardwareInterfaceException& ex){
      ROS_ERROR_STREAM("Exception thrown : " << ex.what());
      return false;
    }
  }
  return true;
}

void single_joint_controller::update(const ros::Time &time, const ros::Duration &period)
{
  for (unsigned int i = 0; i < num_joints_; ++i) {
    joints_[i].setCommand(i/10);
  }
}

void single_joint_controller::starting(const ros::Time &time){
  ROS_INFO("starting single joint controller");
}

void single_joint_controller::stopping(const ros::Time &){}

}//namespace

PLUGINLIB_EXPORT_CLASS(controller_ns::single_joint_controller,controller_interface::ControllerBase)


