#include "two_joints_control.hpp"
#include "ros/ros.h"
namespace controller_ns {

two_joints_controller::two_joints_controller()
{
  ROS_INFO("Let's start");
}

two_joints_controller::~two_joints_controller()
{
  ROS_INFO("End");
}


bool two_joints_controller::init(hardware_interface::EffortJointInterface *hardware, ros::NodeHandle &node_handle)
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

//  joints_[i] = hardware->getHandle("base_link");

  for(unsigned int i = 0; i < num_joints_; i++)
  {
    try{
      joints_.push_back(hardware->getHandle(joint_names_[i]));
//                        joint_effort_interfaces.getHandle(joint_names_[i]));
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


    if (!pid_controllers_[i].init(ros::NodeHandle(node_handle,"/two_joints_controller/"
                                                  + joint_names_[i] + "/pid")))
    {
        ROS_ERROR_STREAM("Failed to load PID parameters from" << joint_names_[i] + "/pid");
    }

    }
    jnt_para_init();
  return true;
}

void two_joints_controller::update(const ros::Time &time, const ros::Duration &period)
{
  ROS_INFO("UPDATE THE CONTROLLER!");
  for (unsigned int i = 0; i < joints_.size(); i++) {
//      std::cout << "looping....." << std::endl;
      pv_mode.qq[i] = joints_[i].getPosition();
      pv_mode.qv[i] = joints_[i].getVelocity();
      std::cout <<"qq is " << i << pv_mode.qq[i] << std::endl;
      std::cout <<"qv is " << i << pv_mode.qv[i] << std::endl;
  }
//  ROS_INFO_STREAM("1");
  pid_crtl(pv_mode, tau_pd);
//  ROS_INFO_STREAM("2");
  feed_forward(pv_mode, tau_ff);
//  ROS_INFO_STREAM("3");
  for (unsigned int i = 0; i < joints_.size(); i++) {
      final_tau[i] = tau_ff[i] + tau_pd[i];
  }
  for (unsigned int i = 0; i < joints_.size(); i++) {
      joints_[i].setCommand(final_tau[i]);
      ROS_INFO_STREAM("command is " << joints_[i].getCommand());
  }
//  ROS_INFO("success update~");
}

void two_joints_controller::starting(const ros::Time &time){
  ROS_INFO("starting single joint controller");
}

void two_joints_controller::stopping(const ros::Time &){}

double two_joints_controller::ComputeTorqueFromPositionCommand(double command, int i, const ros::Duration &period)
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

PLUGINLIB_EXPORT_CLASS(controller_ns::two_joints_controller,controller_interface::ControllerBase)


