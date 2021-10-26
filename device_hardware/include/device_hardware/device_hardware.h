/* device_hardware.h
    DEVICE communication

    author:hanbing
*/

#ifndef DEVICE_HARDWARE_H
#define DEVICE_HARDWARE_H


#include <ros/ros.h>
#include <ros/node_handle.h>
#include <urdf/model.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <pluginlib/class_list_macros.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>


#include <fstream>
#include <iomanip>
#include <string>

namespace device_hardware
{
struct Robot_Information
{
  const char* robot_name;
  int _dof;
  std::vector<signed char> device_mode;

  std::vector<std::string> joints;
  size_t joints_num;

  std::vector<double> joint_position_state_;
  std::vector<double> joint_velocity_state_;
  std::vector<double> joint_effort_state_;

  std::vector<double> joint_position_command_;
  std::vector<double> joint_velocity_command_;
  std::vector<double> joint_effort_command_;

  std::vector<std::string> joint_name_;

  std::vector< hardware_interface::JointHandle >  pj_handle_;
  std::vector< hardware_interface::JointHandle >  vj_handle_;
  std::vector< hardware_interface::JointHandle >  ej_handle_;

  std::vector<joint_limits_interface::JointLimits>     joint_limit_;
  std::vector<joint_limits_interface::SoftJointLimits> joint_solft_limit_;

  std::vector<double> joint_position_sim_;
  std::vector<double> joint_velocity_sim_;
  std::vector<double> joint_effort_sim_;
};
class DeviceHardware : public hardware_interface::RobotHW

{

  public:

    DeviceHardware();

    DeviceHardware(bool sim_flag);


    virtual ~DeviceHardware(){}

    virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh);

    virtual void read(const ros::Time& time, const ros::Duration& period);

    virtual void write(const ros::Time& time, const ros::Duration& period);

    virtual bool prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                              const std::list<hardware_interface::ControllerInfo>& stop_list);

    virtual void doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                          const std::list<hardware_interface::ControllerInfo>& stop_list);



  protected:


  private:


    std::vector<Robot_Information> Robots;
    int robot_num;
    // hardware_interface
    hardware_interface::JointStateInterface     js_interface_;

    hardware_interface::PositionJointInterface  pj_interface_;
    hardware_interface::VelocityJointInterface  vj_interface_;
    hardware_interface::EffortJointInterface    ej_interface_;


    //joint_limits_interface


    joint_limits_interface::PositionJointSaturationInterface  pj_limits_interface_;
    joint_limits_interface::PositionJointSoftLimitsInterface  pj_soft_limits_interface_;
    joint_limits_interface::VelocityJointSaturationInterface  vj_limits_interface_;
    joint_limits_interface::VelocityJointSoftLimitsInterface  vj_soft_limits_interface_;
    joint_limits_interface::EffortJointSaturationInterface    ej_limits_interface_;
    joint_limits_interface::EffortJointSoftLimitsInterface    ej_soft_limits_interface_;

    typedef boost::shared_ptr<const urdf::Joint>   JointConstSharedPtr;
    //std::vector<JointConstSharedPtr> joint_urdfs_;
    std::vector<urdf::JointConstSharedPtr> joint_urdfs_; //kinetic-devel

    std::string commond_controller;
    std::string hyy_controller;
    int controller_flag;
    //-----------------------------------------------------------------------------
    bool _sim_flag;

    double joint_state_test[2][7];
    double joint_state_command_[2][7];

};

}





#endif
