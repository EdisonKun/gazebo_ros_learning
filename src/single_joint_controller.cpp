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


bool single_joint_controller::init(hardware_interface::EffortJointInterface *hardware, ros::NodeHandle &node_handle)
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

    urdf::Model urdf;
    if (!urdf.initParam("/robot_description"))
    {
      ROS_ERROR("Failed to parse urdf file");
      return false;
    }

    pid_controllers_.resize(num_joints_);

    for(unsigned int i = 0; i < num_joints_; i++)
    {
      const auto& joint_name = joint_names_[i];

      urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_name);
      if(!joint_urdf)
      {
        ROS_ERROR("Could not find joint '%s' in urdf", joint_name.c_str());
        return false;
      }

    joint_urdfs_.push_back(joint_urdf);


    if (!pid_controllers_[i].init(ros::NodeHandle(node_handle,"/single_joint_controller/"
                                                  + joint_names_[i] + "/pid")))
    {
        ROS_ERROR_STREAM("Failed to load PID parameters from" << joint_names_[i] + "/pid");
    }

    }
  return true;
}

void single_joint_controller::update(const ros::Time &time, const ros::Duration &period)
{
  ROS_INFO("UPDATE THE CONTROLLER!");
  double command = 0.0;
  for (unsigned int i = 0; i < num_joints_; ++i) {
    double effort_command = ComputeTorqueFromPositionCommand(command, i, period);
    joints_[i].setCommand(effort_command);
    ROS_INFO_STREAM(effort_command);
  }
}

void single_joint_controller::starting(const ros::Time &time){
  ROS_INFO("starting single joint controller");
}

void single_joint_controller::stopping(const ros::Time &){}

double single_joint_controller::ComputeTorqueFromPositionCommand(double command, int i, const ros::Duration &period)
{
  double command_position = command;

  double error;
  double command_effort;

  double current_position = joints_[i].getPosition();

  //compute position error
  if (joint_urdfs_[i]->type == urdf::Joint::REVOLUTE )
  {
    angles::shortest_angular_distance_with_limits(
          current_position, command_position,
          joint_urdfs_[i]->limits->lower,
          joint_urdfs_[i]->limits->upper,
          error);
  }
  else if (joint_urdfs_[i]->type == urdf::Joint::CONTINUOUS) {
    error = angles::shortest_angular_distance(current_position, command_position);
  }
  else if (joint_urdfs_[i]->type == urdf::Joint::REVOLUTE) {
    error = command_position - current_position;
  }

  //set PID error and compute the PID command with nonuniform time step size
  command_effort = pid_controllers_[i].computeCommand(error,period);
  return command_effort;
}

}//namespace

PLUGINLIB_EXPORT_CLASS(controller_ns::single_joint_controller,controller_interface::ControllerBase)


