/*
 * device_hardware.cpp
 *
 *  Created on: Jan 6, 2021
 *      Author: hanbing
 */
#include <algorithm>
#include "device_hardware/device_hardware.h"
#include "device_interface/DeviceDriver/device_interface.h"
#include "device_interface/Base/robotStruct.h"
#include "device_interface/Base/RobotSystem.h"
#include "device_interface/Move/MovePlan.h"
namespace device_hardware
{

   DeviceHardware::DeviceHardware()
   {
      _sim_flag=false;
      controller_flag=0;
   }


   DeviceHardware::DeviceHardware(bool sim_flag)
   {
      _sim_flag=sim_flag;
      controller_flag=0;
   }

   bool DeviceHardware::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh)
   {
      using namespace hardware_interface;
      using namespace joint_limits_interface;
      using namespace HYYRobotBase;

      size_t i=0;
      size_t j=0;

      // Robots Initialize
      robot_num = hasNumber_robot_device(get_deviceName(0,NULL));
      ROS_INFO("Robot Number = %d", robot_num);
      // Avoiding the Address Error Caused by Release the Push Back Function
      for (i = 0; i < robot_num; i++)
      {
         Robot_Information Robot;
         Robots.push_back(Robot);
      }
      // Active every robot
      for (i = 0; i < robot_num; i++)
      {
         Robots[i].robot_name = get_name_robot_device(get_deviceName(0,NULL), i);
         Robots[i]._dof = get_group_dof(Robots[i].robot_name);
         Robots[i].device_mode.resize(Robots[i]._dof,0);

         int e_m=0;
         if (!_sim_flag)
         {
            if(!root_nh.getParam("device_mode", e_m))
            {
               ROS_ERROR_STREAM("ros con't find paramber \"device_mode\"");
               return false;
            }
            for (j=0;j<Robots[i]._dof;j++)
            {
               Robots[i].device_mode[j]=(signed char)e_m;
            }
            ROS_INFO("device mode = %d",Robots[i].device_mode[0]);
            set_group_mode(Robots[i].robot_name, &(Robots[i].device_mode[0]));
         }

         //get joints name
         bool load_dual_robot;
         if(!root_nh.getParam("load_dual_robot", load_dual_robot))
         {
            ROS_ERROR_STREAM("ros con't find paramber \"load_dual_robot\"");
            return 0;
         }
         if(load_dual_robot)
         {
            if(i == 0)
            {
               if (!robot_hw_nh.getParam("left_joint_trajectory_controller/joints", Robots[i].joints))
               {
                  ROS_ERROR_STREAM("hardware can't get robot joints!");
                  return false;
               }
            }
            else
            {
               if (!robot_hw_nh.getParam("right_joint_trajectory_controller/joints", Robots[i].joints))
               {
                  ROS_ERROR_STREAM("hardware can't get robot joints!");
                  return false;
               }
            }
         }
         else
         {
               if (!robot_hw_nh.getParam("joint_trajectory_controller/joints", Robots[i].joints))
               {
                  ROS_ERROR_STREAM("hardware can't get robot joints!");
                  return false;
               }
         }

         Robots[i].joints_num = Robots[i].joints.size();
         if (Robots[i]._dof!=Robots[i].joints_num)
         {
            ROS_ERROR_STREAM("ros joint numbers is different to device!"<<"ros:"<<Robots[i].joints_num<<"  "<<"device:"<<Robots[i]._dof);
            return false;
         }

         // Initialize raw data
         Robots[i].joint_position_state_.resize(Robots[i].joints_num);
         Robots[i].joint_velocity_state_.resize(Robots[i].joints_num);
         Robots[i].joint_effort_state_.resize(Robots[i].joints_num);

         Robots[i].joint_position_commond_.resize(Robots[i].joints_num);
         Robots[i].joint_velocity_commond_.resize(Robots[i].joints_num);
         Robots[i].joint_effort_commond_.resize(Robots[i].joints_num);

         Robots[i].joint_name_.resize(Robots[i].joints_num);

         Robots[i].pj_handle_.resize(Robots[i].joints_num);
         Robots[i].vj_handle_.resize(Robots[i].joints_num);
         Robots[i].ej_handle_.resize(Robots[i].joints_num);

         Robots[i].joint_limit_.resize(Robots[i].joints_num);
         Robots[i].joint_solft_limit_.resize(Robots[i].joints_num);


         for (j=0;j<Robots[i].joints_num;j++)
         {
            Robots[i].joint_name_[j] = Robots[i].joints[j];
            Robots[i].joint_position_state_[j] = GetAxisPosition(Robots[i].robot_name,j+1);
            Robots[i].joint_velocity_state_[j] = 0.0;
            Robots[i].joint_effort_state_[j] = 0.0;
            Robots[i].joint_position_commond_[j] = Robots[i].joint_position_state_[j];
            Robots[i].joint_velocity_commond_[j] = Robots[i].joint_velocity_state_[j];
            Robots[i].joint_effort_commond_[j] = Robots[i].joint_effort_state_[j];

            // Populate hardware interfaces
            js_interface_.registerHandle(
               JointStateHandle(  Robots[i].joint_name_[j],
                                 &(Robots[i].joint_position_state_[j]),
                                 &(Robots[i].joint_velocity_state_[j]),
                                 &(Robots[i].joint_effort_state_[j]))
                                 );

            Robots[i].pj_handle_[j] = JointHandle(
               js_interface_.getHandle(Robots[i].joints[j]),
               &(Robots[i].joint_position_commond_[j]));
            Robots[i].vj_handle_[j] = JointHandle(
               js_interface_.getHandle(Robots[i].joints[j]),
               &(Robots[i].joint_velocity_commond_[j]));
            Robots[i].ej_handle_[j] = JointHandle(
               js_interface_.getHandle(Robots[i].joints[j]),
               &(Robots[i].joint_effort_commond_[j]));

            pj_interface_.registerHandle(Robots[i].pj_handle_[j]);
            vj_interface_.registerHandle(Robots[i].vj_handle_[j]);
            ej_interface_.registerHandle(Robots[i].ej_handle_[j]);
         }
         Robots[i].joint_position_sim_.resize(Robots[i].joints_num,0.0);
         Robots[i].joint_velocity_sim_.resize(Robots[i].joints_num,0.0);
         Robots[i].joint_effort_sim_.resize(Robots[i].joints_num,0.0);

         double angle_tmp[20]={0,0,0,Robots[i].joint_position_sim_[4],0,0,0};

         if (_sim_flag)
         {
            SetGroupPosition(Robots[i].robot_name, angle_tmp);
            for (j=0;j<Robots[i].joints_num;j++)
            {
               Robots[i].joint_position_commond_[j] = Robots[i].joint_position_sim_[j];
               Robots[i].joint_position_state_[j] = Robots[i].joint_position_sim_[j];
            }
         }
         else
         {
            GetGroupPosition(Robots[i].robot_name, angle_tmp);
            SetGroupPosition(Robots[i].robot_name, angle_tmp);
            for (j = 0;j < Robots[i]._dof;j++)
            {
               Robots[i].joint_position_commond_[j] = angle_tmp[j];
               Robots[i].joint_position_state_[j] = angle_tmp[j];
            }
         }
         ROS_INFO("Program has pushed the %dth ",i+1);
      }

      registerInterface(&js_interface_);
      registerInterface(&pj_interface_);
      registerInterface(&vj_interface_);
      registerInterface(&ej_interface_);

      controller_flag=0;
      ROS_INFO("device initialize finish");
      return true;
   }


   void DeviceHardware::read(const ros::Time& time, const ros::Duration& period)
   {
      // read from device_interface write joint_position_state_, joint_velocity_state_, joint_effort_state_
      size_t i=0;
      if (_sim_flag)
      {
         for(i = 0; i < robot_num; i++)
         {
            memcpy(  &(Robots[i].joint_position_state_[0]),
                     &(Robots[i].joint_position_sim_[0]),
                     Robots[i].joints_num*sizeof(double));
            memcpy(  &(Robots[i].joint_velocity_state_[0]),
                     &(Robots[i].joint_velocity_sim_[0]),
                     Robots[i].joints_num*sizeof(double));
            memcpy(  &(Robots[i].joint_effort_state_[0]),
                     &(Robots[i].joint_effort_sim_[0]),
                     Robots[i].joints_num*sizeof(double));
         }
      }
      else
      {
         for(i = 0; i < robot_num; i++)
         {
            HYYRobotBase::GetGroupPosition(Robots[i].robot_name, &(Robots[i].joint_position_state_[0]));
            HYYRobotBase::GetGroupVelocity(Robots[i].robot_name, &(Robots[i].joint_velocity_state_[0]));
            HYYRobotBase::GetGroupTorque(Robots[i].robot_name, &(Robots[i].joint_effort_state_[0]));
         }
      }

   }



   void DeviceHardware::write(const ros::Time& time, const ros::Duration& period)
   {
      size_t i=0;
      double _tmp=0;
      if (_sim_flag)
      {
         for(i = 0; i<robot_num; i++)
         {
            memcpy(  &(Robots[i].joint_position_sim_[0]),
                     &(Robots[i].joint_position_commond_[0]),
                     Robots[i].joints_num*sizeof(double));
            memcpy(  &(Robots[i].joint_velocity_sim_[0]),
                     &(Robots[i].joint_velocity_commond_[0]),
                     Robots[i].joints_num*sizeof(double));
            memcpy(  &(Robots[i].joint_effort_sim_[0]),
                     &(Robots[i].joint_effort_commond_[0]),
                     Robots[i].joints_num*sizeof(double));
         }
      }
      else
      {
         for(i = 0; i < robot_num; i++)
         {
            HYYRobotBase::SetGroupPosition(Robots[i].robot_name, &(Robots[i].joint_position_commond_[0]));
            HYYRobotBase::SetGroupVelocity(Robots[i].robot_name, &(Robots[i].joint_velocity_commond_[0]));
            HYYRobotBase::SetGroupTorque(Robots[i].robot_name, &(Robots[i].joint_effort_commond_[0]));
         }
      }
   }
bool DeviceHardware::prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                               const std::list<hardware_interface::ControllerInfo>& stop_list)
{
   if (!start_list.empty())
   {
      for (std::list<hardware_interface::ControllerInfo>::const_iterator it = start_list.begin(); it != start_list.end(); ++it)
      {
         if (it->claimed_resources.empty())
         {
            continue;
         }
         else
         {
        	 //
         }
      }
   }
   if (!stop_list.empty())
   {
      for (std::list<hardware_interface::ControllerInfo>::const_iterator it = stop_list.begin(); it != stop_list.end(); ++it)
      {
         if (it->claimed_resources.empty())
         {
            continue;
         }
         else
         {
        	 //
         }
      }
   }
   return true;
}



void DeviceHardware::doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                          const std::list<hardware_interface::ControllerInfo>& stop_list)
{
   if (!start_list.empty())
   {
      for (std::list<hardware_interface::ControllerInfo>::const_iterator it = start_list.begin(); it != start_list.end(); ++it)
      {
         if (it->claimed_resources.empty())
         {
            continue;
         }
         else
         {
        	 ROS_INFO("start controller:%s",it->name.c_str());
            if (it->name==commond_controller)
            {
            	if (0==controller_flag)
            	{
                	controller_flag=1;
					if (!_sim_flag)
					{
						if(0==HYYRobotBase::group_power_on(Robots[0].robot_name))
						{
							ROS_INFO("power success");
						}
						else
						{
							ROS_INFO("power failure");
						}
					}
					if (!_sim_flag)
					{
						if(0==HYYRobotBase::group_power_on(Robots[1].robot_name))
						{
							ROS_INFO("power success");
						}
						else
						{
							ROS_INFO("power failure");
						}
					}
            	}
            	else
            	{
            		ROS_WARN_STREAM("hyy_controller is running,can't run commond_controller.");
            	}
            }
            if (it->name==hyy_controller)
            {
              if (0==controller_flag)
              {
                  controller_flag=2;
                  if (!_sim_flag)
                  {
                	  HYYRobotBase::move_start();
                	  ROS_INFO("power");
                  }
              }
              else
              {
            	  ROS_WARN_STREAM("commond_controller is running,can't run hyy_controller.");
              }
            }
         }
      }
   }
   if (!stop_list.empty())
   {
      for (std::list<hardware_interface::ControllerInfo>::const_iterator it = stop_list.begin(); it != stop_list.end(); ++it)
      {
         ROS_INFO("stop controller %s",it->name.c_str());
         if (it->claimed_resources.empty())
         {
            continue;
         }
         else
         {
            if (it->name==commond_controller)
            {
            	controller_flag=0;
				if (!_sim_flag)
				{
					if(0==HYYRobotBase::group_power_off(Robots[0].robot_name))
					{
					   ROS_INFO("poweroff success");
					}
					else
					{
					   ROS_INFO("poweroff failure");
					}
				}
				if (!_sim_flag)
				{
					if(0==HYYRobotBase::group_power_off(Robots[1].robot_name))
					{
					   ROS_INFO("poweroff success");
					}
					else
					{
					   ROS_INFO("poweroff failure");
					}
				}
            }
            if (it->name==hyy_controller)
            {
            	controller_flag=0;
				if (!_sim_flag)
				{
              	  HYYRobotBase::move_stop();
              	  ROS_INFO("poweroff");
				}
            }
         }
      }
   }
}
}//namespace ec_hardware

PLUGINLIB_EXPORT_CLASS( device_hardware::DeviceHardware, hardware_interface::RobotHW)