#include "robot_display_with_gazebo_rviz.hpp"

namespace controller_ns {

sync_gazebo_rviz_controller::sync_gazebo_rviz_controller()
{
  ROS_INFO("Let's start");
}

sync_gazebo_rviz_controller::~sync_gazebo_rviz_controller()
{
  ROS_INFO("End");
}


bool sync_gazebo_rviz_controller::init(hardware_interface::RobotStateInterfaceKP *hardware, ros::NodeHandle &node_handle)
{
  ROS_INFO("Initializing sync_gazebo_rviz_controller");

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

  robot_state_handle_ = hardware->getHandle("base_link");

  for(unsigned int i = 0; i < num_joints_; i++)
  {
    try{
      joints_.push_back(hardware->joint_effort_interfaces.getHandle(joint_names_[i]));
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


    if (!pid_controllers_[i].init(ros::NodeHandle(node_handle,"/sync_gazebo_rviz_controller/"
                                                  + joint_names_[i] + "/pid")))
    {
        ROS_ERROR_STREAM("Failed to load PID parameters from" << joint_names_[i] + "/pid");
    }

    }

  joints_publisher_ = node_handle.advertise<sensor_msgs::JointState>("/joint_states",1);
//  robot_state_publisher_ = node_handle.advertise<new_quadruped_model_kp::>()
  q_state.name.resize(joint_names_.size());
  q_state.effort.resize(joint_names_.size());
  q_state.position.resize(joint_names_.size());
  q_state.velocity.resize(joint_names_.size());
  return true;
}

void sync_gazebo_rviz_controller::update(const ros::Time &time, const ros::Duration &period)
{
  ROS_INFO("UPDATE THE CONTROLLER!");
  double command = 0.0;
  for (unsigned int i = 0; i < num_joints_; ++i) {
    double effort_command = ComputeTorqueFromPositionCommand(command, i, period);
    joints_[i].setCommand(effort_command);
    ROS_INFO_STREAM(effort_command);
  }
  q_state.header.stamp = ros::Time::now();
  for (unsigned int i = 0; i < joint_names_.size(); ++i) {
    q_state.name[i] = joint_names_[i];
    q_state.effort[i] = joints_[i].getEffort();
    q_state.position[i] = joints_[i].getPosition();
    q_state.velocity[i] = joints_[i].getVelocity();
  }

  joints_publisher_.publish(q_state);
  ROS_INFO_STREAM("The position of base is" << *robot_state_handle_.getPosition()<<"****");

}

void sync_gazebo_rviz_controller::starting(const ros::Time &time){
  ROS_INFO("starting single joint controller");
}

void sync_gazebo_rviz_controller::stopping(const ros::Time &){}

double sync_gazebo_rviz_controller::ComputeTorqueFromPositionCommand(double command, int i, const ros::Duration &period)
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

PLUGINLIB_EXPORT_CLASS(controller_ns::sync_gazebo_rviz_controller,controller_interface::ControllerBase)
