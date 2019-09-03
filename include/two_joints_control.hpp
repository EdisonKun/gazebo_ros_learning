#pragma once
#include "controller_interface/controller.h"
#include "hardware_interface/joint_command_interface.h"
#include <angles/angles.h>
#include <pluginlib/class_list_macros.hpp>
#include <control_toolbox/pid.h>
#include <urdf/model.h>
#include <iomanip>
//#include <robot_state_interface_kp.hpp>

namespace controller_ns {

class two_joints_controller:public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
  two_joints_controller();
  ~two_joints_controller();
  bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &n);
  void update(const ros::Time &time, const ros::Duration &period);
  void starting(const ros::Time& time);
  void stopping(const ros::Time& time);
  double ComputeTorqueFromPositionCommand(double command, int i, const ros::Duration& period);

private:
  std::vector<hardware_interface::JointHandle> joints_;
  hardware_interface::JointHandle robot_state_handle_;
  std::string base_name_;
  std::vector<std::string> joint_names_;
  unsigned int num_joints_;

  std::vector<urdf::JointConstSharedPtr> joint_urdfs_;
  std::vector<control_toolbox::Pid> pid_controllers_;

  std::vector<double> tau_pd;
  std::vector<double> tau_ff;
  std::vector<double> errori;
  std::vector<double> final_tau;

  int DIM = 2;
  double g = 9.8;
  double pi = 3.14159;

  struct jnt_para{
      double kp;
      double ki;
      double kd;
      std::vector<double> qqd, qvd, qad, qq, qv, qa;
  };

  jnt_para pv_mode;

  void jnt_para_init(){
      pv_mode.kp=5;
      pv_mode.kd=5;
      pv_mode.ki=0;
      errori.resize(2);
      errori[0] = 0;
      errori[1] = 0;
      tau_ff.resize(2);
      tau_ff[0] = 0;
      tau_ff[1] = 0;
      tau_pd.resize(2);
      tau_pd[0] = 0;
      tau_pd[1] = 0;
      final_tau.resize(2);
      final_tau[0] = 0;
      final_tau[1] = 0;
      pv_mode.qqd.resize(2);
      pv_mode.qvd.resize(2);
      pv_mode.qad.resize(2);
      pv_mode.qq.resize(2);
      pv_mode.qv.resize(2);
      pv_mode.qa.resize(2);

      for (unsigned int i=0;i<DIM;i++)
      {
          pv_mode.qqd[i]=0;
          pv_mode.qvd[i]=0;
          pv_mode.qad[i]=0;
          pv_mode.qq[i]=0;
          pv_mode.qv[i]=0;
          pv_mode.qa[i]=0;
      }
  };

  void pid_crtl(jnt_para& pv_mode,std::vector<double>& tau_pd)
  {
      double error[2],errord[2];
//      std::cout <<"1.1"<<std::endl;
      for (unsigned int i = 0; i < 2; i++)
      {
          error[i]=pv_mode.qqd[i]-pv_mode.qq[i];
//          std::cout <<"1.2"<<std::endl;
          errord[i]=pv_mode.qvd[i]-pv_mode.qv[i];
//          std::cout <<"1.3"<<std::endl;
          errori[i]=errori[i]+error[i];
//          std::cout <<"1.4"<<std::endl;
          tau_pd[i]=pv_mode.kp*error[i]+pv_mode.kd*errord[i]+pv_mode.ki*errori[i];
          std::cout << "tau_pd is " << tau_pd[i] << std::endl;
//          std::cout <<"1.5"<<std::endl;
      }
  };

  void feed_forward(jnt_para& pv_mode, std::vector<double>& tau_ff)
  {
      float Il1=0.003418,Il2=0.99921423,Im1=0.05275,Im2=0.05275,ml1=0.386,ml2=0.75551356,mm1=1.1,mm2=1.1,l1=0.3,l2=0.3,xc1=0.15,xc2=0.21265;
      double grav[DIM],Mqq[DIM][DIM],Mq[DIM],Cqqv[DIM],h,C[DIM][DIM];

      grav[0]=(ml1*xc1+mm2*l1+ml2*l1)*g*cos(pv_mode.qq[0]+pi/2)+ml2*g*xc2*cos(pv_mode.qq[0]+pv_mode.qq[1]+pi/2);
      grav[1]=ml2*xc2*g*cos(pv_mode.qq[0]+pv_mode.qq[1]+pi/2);

      ROS_INFO("2.1");

      Mqq[0][0]=Il1+ml1*xc1*xc1+Im1+Il2+ml2*(l1*l1+xc2*xc2+2*l1*xc2*cos(pv_mode.qq[1]))+mm2*l1*l1;
      Mqq[0][1]=Il2+ml2*(xc2*xc2+l1*xc2*cos(pv_mode.qq[1]));
      Mqq[1][0]=Il2+ml2*(xc2*xc2+l1*xc2*cos(pv_mode.qq[1]));
      Mqq[1][1]=Il2+ml2*xc2*xc2+Im2;
      Mq[0]=Mqq[0][0]*pv_mode.qa[0]+Mqq[0][1]*pv_mode.qa[1];
      Mq[1]=Mqq[1][0]*pv_mode.qa[0]+Mqq[1][1]*pv_mode.qa[1];

      ROS_INFO("2.2");
      h=-ml2*l1*xc2*sin(pv_mode.qq[1]);
      C[0][0]=h*pv_mode.qv[1];
      C[0][1]=h*(pv_mode.qv[0]+pv_mode.qv[1]);
      C[1][0]=-h*pv_mode.qv[0];
      C[1][1]=0;
      Cqqv[0]=C[0][0]*pv_mode.qv[0]+C[0][1]*pv_mode.qv[1];
      Cqqv[1]=C[1][0]*pv_mode.qv[0]+C[1][1]*pv_mode.qv[1];
      for (unsigned int i=0;i < DIM;i++)
      {
          tau_ff[i]=grav[i]+Mq[i]+Cqqv[i];
          std::cout << "tau_ff is " << tau_ff[i] << std::endl;
      }
      ROS_INFO("2.3");
  };

};
}//namespace
