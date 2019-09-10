#pragma once
#include "controller_interface/controller.h"
#include "hardware_interface/joint_command_interface.h"
#include <robot_state_interface_kp.hpp>
#include <control_toolbox/pid.h>
#include <pluginlib/class_list_macros.h>
#include <urdf/model.h>
#include <new_quadruped_model_kp/joint_state.h>
#include <angles/angles.h>
#include <tf/transform_broadcaster.h>

namespace controller_ns {
class sync_gazebo_rviz_controller: public controller_interface::Controller<hardware_interface::RobotStateInterfaceKP>
{
public:
    sync_gazebo_rviz_controller();
    ~sync_gazebo_rviz_controller();
    bool init(hardware_interface::RobotStateInterfaceKP *hw, ros::NodeHandle &n);
    void update(const ros::Time & time, const ros::Duration &period);
    void starting(const ros::Time &time);
    void stopping(const ros::Time &time);
    double ComputeTorqueFromPositionCommand(double command, int i, const ros::Duration &period);

private:
    std::vector<hardware_interface::JointHandle> joints_;
    hardware_interface::RobotStateHandleKP robot_state_handle_;
    std::string base_name_;

    std::vector<std::string> joint_names_;
    unsigned int num_joints_;
    ros::Publisher joints_publisher_, robot_state_publisher_;

    std::vector<urdf::JointConstSharedPtr> joint_urdfs_;
    std::vector<control_toolbox::Pid> pid_controllers_;
    sensor_msgs::JointState q_state;

    tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;


};
}//namespace
