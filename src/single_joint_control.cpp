#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

namespace controller_ns {
class PositionController:public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
  bool init(hardware_interface::EffortJointInterface * hw, ros::NodeHandle &n)
  {
    std::string my_joint;
    if (!n.getParam("joint",my_joint))
    {
      ROS_ERROR("could not find joint name");
    return false;
     }
  joint_ = hw->getHandle(my_joint);
  return true;
  }

  void update(const ros::Time&time, const ros::Duration& period)
  {
    double error = setpoint_ - joint_.getPosition();
    joint_.setCommand(error*gain_);
  }

  void starting(const ros::Time &time){}
  void stopping(const ros::Time &time){}

private:
hardware_interface::JointHandle joint_;
const double gain_ = 1.25;
const double setpoint_ = 3.00;

};
PLUGINLIB_DECLARE_CLASS(new_quadruped_model_kp,PositionController, controller_ns::PositionController,controller_interface::ControllerBase)
}
