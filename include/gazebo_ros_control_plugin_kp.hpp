#pragma once
//Boost
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

//ROS
#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <std_msgs/Bool.h>

//Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

//ROS Control
#include <gazebo_ros_control/robot_hw_sim.h>
//#include <gazebo_sim.hpp>
#include <controller_manager/controller_manager.h>
#include <transmission_interface/transmission_parser.h>

namespace gazebo_ros_control {

class GazeboRosControlPluginKP : public gazebo::ModelPlugin
{
public:

  virtual ~GazeboRosControlPluginKP();

  //overloaded gazebo entry point;
  //Called when a Plugin is first created, and after the World has been loaded
  virtual void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf);

  //called by the world update start event;
  void Update();

  //called on world reset
  virtual void Reset();

  //Get the URDF XML from the parameter server;
  std::string getURDF(std::string param_name) const;

  //Get transmissions from the URDF
  bool parseTransmissionsFromURDF(const std::string& urdf_string);

protected:
  void eStopCB(const std_msgs::BoolConstPtr& e_stop_active);

  //Node Handles
  ros::NodeHandle model_nh_; // namespace to robot name

  //pointer to the model
  gazebo::physics::ModelPtr parent_model_;
  sdf::ElementPtr sdf_;

  //deferred load in case ros is blocking
  boost::thread deferred_load_thread_;

  //Pointer to the update event connection
  gazebo::event::ConnectionPtr update_connection_;

  //Interface loader
  boost::shared_ptr<pluginlib::ClassLoader<gazebo_ros_control::RobotHWSim> > robot_hw_sim_loader_;
  void load_robot_hw_sim_srv();

  //Strings
  std::string robot_namespace_;
  std::string robot_description_;

  //Transmissions in this plgin's scope
  std::vector<transmission_interface::TransmissionInfo> transmissions_;

  //Robot simulator interface
  std::string robot_hw_sim_type_str_;
  boost::shared_ptr<gazebo_ros_control::RobotHWSim> robot_hw_sim_;

  //controller manager
  boost::shared_ptr <controller_manager::ControllerManager> controller_manager_;

  //Timing
  ros::Duration control_period_;
  ros::Time last_update_sim_time_ros_;
  ros::Time last_write_sim_time_ros_;

  //e_stop_active_ is true if the emergency stop is active.
  bool e_stop_active_, last_e_stop_active_;
  ros::Subscriber e_stop_sub_;//Emergency stop subscriber;
};

}
