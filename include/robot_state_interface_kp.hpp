#pragma once

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <string>

namespace hardware_interface {

class RobotStateHandleKP: public JointHandle
{
public:struct Data
  {
    Data():
      name(),
      frame_id(),
      orientation(0),
      angular_velocity(0),
      linear_acceleration(0),
      linear_velocity(0),
      joint_position_read(0),
      joint_position_write(0),
      joint_velocity_read(0),
      joint_velocity_write(0),
      joint_effort_read(0),
      joint_effort_write(0),
      foot_contact(0){} //initialize the struct Data.

    std::string name;                       ///< The name of the sensor
    std::string frame_id;                   ///< The reference frame to which this sensor is associated
    double* orientation;                    ///< A pointer to the storage of the orientation value: a quaternion (x,y,z,w)
    double* position;         ///< A pointer to the storage of the orientation covariance value: a row major 3x3 matrix about (x,y,z)
    double* angular_velocity;               ///< A pointer to the storage of the angular velocity value: a triplet (x,y,z)
    double* linear_acceleration;            ///< A pointer to the storage of the linear acceleration value: a triplet (x,y,z)
    double* linear_velocity; ///< A pointer to the storage of the linear acceleration covariance value: a row major 3x3 matrix about (x,y,z)
    double* joint_position_read;
    double* joint_position_write;
    double* joint_velocity_read;
    double* joint_velocity_write;
    double* joint_effort_read;
    double* joint_effort_write;
    bool* foot_contact;
  };

  RobotStateHandleKP(const Data& data = Data())
    :
      name_(data.name),
      frame_id_(data.frame_id),
      orientation_(data.orientation),
      position_(data.position),
      angular_velocity_(data.angular_velocity),
      linear_acceleration_(data.linear_acceleration),
      linear_velocity_(data.linear_velocity),
      joint_position_read_(data.joint_position_read),
      joint_position_write_(data.joint_position_write),
      joint_velocity_read_(data.joint_velocity_read),
      joint_velocity_write_(data.joint_velocity_write),
      joint_effort_read_(data.joint_effort_read),
      joint_effort_write_(data.joint_effort_write),
      foot_contact_(data.foot_contact){}

  RobotStateHandleKP(const std::string& name,                      ///< The name of the sensor
                   const std::string& frame_id,                  ///< The reference frame to which this sensor is associated
                   const double* orientation,                    ///< A pointer to the storage of the orientation value: a quaternion (x,y,z,w)
                   const double* position,         ///< A pointer to the storage of the orientation covariance value: a row major 3x3 matrix about (x,y,z)
                   const double* angular_velocity,               ///< A pointer to the storage of the angular velocity value: a triplet (x,y,z)
                   const double* linear_acceleration,            ///< A pointer to the storage of the linear acceleration value: a triplet (x,y,z)
                   const double* linear_velocity,  ///< A pointer to the storage of the linear acceleration covariance value: a row major 3x3 matrix about (x,y,z)
                   const double* joint_position_read,
                   double* joint_position_write,
                   const double* joint_velocity_read,
                   double* joint_velocity_write,
                   const double* joint_effort_read,
                   double* joint_effort_write,
                   const bool* foot_contact            ///< A pointer to store if a leg is contact [LF,RF,RH,LH]

                 )
               : name_(name),
                 frame_id_(frame_id),
                 orientation_(orientation),
                 position_(position),
                 angular_velocity_(angular_velocity),
                 linear_acceleration_(linear_acceleration),
                 linear_velocity_(linear_velocity),
                 joint_position_read_(joint_position_read),
                 joint_position_write_(joint_position_write),
                 joint_velocity_read_(joint_velocity_read),
                 joint_velocity_write_(joint_velocity_write),
                 joint_effort_read_(joint_effort_read),
                 joint_effort_write_(joint_effort_write),
                 foot_contact_(foot_contact)
             {}

 std::string getName()                           const {return name_;}
 std::string getFrameId()                        const {return frame_id_;}
 const double* getOrientation()                  const {return orientation_;}
 const double* getPosition()                     const {return position_;}
 const double* getAngularVelocity()              const {return angular_velocity_;}
 const double* getLinearAcceleration()           const {return linear_acceleration_;}
 const double* getLinearVelocity()               const {return linear_velocity_;}
 const double* getJointPositionRead()            const {return joint_position_read_;}
 double* getJointPositionWrite()                       {return joint_position_write_;}
 const double* getJointVelocityRead()            const {return joint_velocity_read_;}
 double* getJointVelocityWrite()                       {return joint_velocity_write_;}
 const double* getJointEffortRead()              const {return joint_effort_read_;}
 double* getJointEffortWrite()                         {return joint_effort_write_;}
 const bool* getFootContact()                    const {return foot_contact_;}

private:
  std::string name_;
  std::string frame_id_;

  const double* orientation_;
  const double* position_;
  const double* angular_velocity_;
  const double* linear_acceleration_;
  const double* linear_velocity_;
  const double* joint_position_read_;
  double* joint_position_write_;
  const double* joint_velocity_read_;
  double* joint_velocity_write_;
  const double* joint_effort_read_;
  double* joint_effort_write_;
  const bool* foot_contact_;

};

class RobotStateInterfaceKP:public HardwareResourceManager<RobotStateHandleKP>{
public:
  hardware_interface::JointCommandInterface joint_effort_interfaces;
};

}
