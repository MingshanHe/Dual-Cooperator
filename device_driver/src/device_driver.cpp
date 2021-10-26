/*
 * device_driver.cpp
 *
 *  Created on: Jan 7, 2021
 *      Author: hanbing
 */

#include "device_hardware/device_hardware.h"
#include "device_interface/DeviceDriver/device_timer.h"
#include "device_interface/Base/RobotSystem.h"
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <urdf/model.h>
#include <controller_manager/controller_manager.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <pthread.h>

int main(int argc, char **argv)
{
   ROS_INFO("start ros robot\n");
   if (0!=HYYRobotBase::initPriority(38))
   {
      ROS_ERROR_STREAM("program priority set failure!");
      return 0;
   }

   std::string controllers_namespace;
   bool load_dual_robot;


   std::string state_controller;
   bool sim_flag=false;
   int communication_time=0;
   ros::init(argc, argv, "hbrobot_control_node");
   ros::NodeHandle node_root;
   if(!node_root.getParam("controllers_namespace", controllers_namespace))
   {
      ROS_ERROR_STREAM("ros con't find paramber \"controllers_namespace\"");
      return 0;
   }
   if(!node_root.getParam("load_dual_robot", load_dual_robot))
   {
      ROS_ERROR_STREAM("ros con't find paramber \"load_dual_robot\"");
      return 0;
   }

   if(!node_root.getParam("state_controller", state_controller))
   {
      ROS_ERROR_STREAM("ros con't find paramber \"state_controller\"");
      return 0;
   }

   if(!node_root.getParam("sim_flag", sim_flag))
   {
      ROS_ERROR_STREAM("ros con't find paramber \"sim_flag\"");
      return 0;
   }

   if(!node_root.getParam("communication_time", communication_time))
   {
      ROS_ERROR_STREAM("ros con't find paramber \"communication_time\"");
      return 0;
   }

   //robot initialize
   std::string system_arg;
   if(!node_root.getParam("system_arg", system_arg))
   {
      ROS_ERROR_STREAM("ros con't find paramber \"system_arg\"");
      return false;
   }
   HYYRobotBase::command_arg arg;
   if (0==HYYRobotBase::commandLineParser1(system_arg.c_str(), &arg))
   {
      if (0!=HYYRobotBase::system_initialize(&arg))
      {
      ROS_ERROR_STREAM("device initialize failure! system_initialize faiure!");
      return false;
      }
   }
   else
   {
      ROS_ERROR_STREAM("device initialize failure! commandLineParser1 faiure!");
      return false;
   }

   ros::NodeHandle node_robothw(node_root, controllers_namespace);

   //ec_hardware::ECHardware robot_hw;
   device_hardware::DeviceHardware robot_hw(sim_flag);
   if (!robot_hw.init(node_root, node_robothw))
   {
      ROS_ERROR_STREAM("hardware init failure!");
      return 0;
   }
   controller_manager::ControllerManager cm(&robot_hw,node_root);

  //load controller
   if(load_dual_robot)
   {
      std::string left_arm_controller;
      std::string right_arm_controller;
      if(!node_root.getParam("left_arm_controller", left_arm_controller))
      {
         ROS_ERROR_STREAM("ros con't find paramber \"left_arm_controller\"");
         return 0;
      }
      if(!node_root.getParam("right_arm_controller", right_arm_controller))
      {
         ROS_ERROR_STREAM("ros con't find paramber \"right_arm_controller\"");
         return 0;
      }

      if (!cm.loadController(controllers_namespace+"/"+left_arm_controller))
      {
         ROS_ERROR_STREAM(left_arm_controller<<" controller load failure!");
         return 0;
      }
      if (!cm.loadController(controllers_namespace+"/"+right_arm_controller))
      {
         ROS_ERROR_STREAM(right_arm_controller<<" controller load failure!");
         return 0;
      }
      if (!cm.loadController(controllers_namespace+"/"+state_controller))
      {
         ROS_ERROR_STREAM(state_controller<<" controller load failure!");
         return 0;
      }
   }
   else
   {
      std::string trajectory_controller;
      std::string hyy_controller;
      if(!node_robothw.getParam("trajectory_controller", trajectory_controller))
      {
         ROS_ERROR_STREAM("ros con't find paramber \"trajectory_controller\"");
         return 0;
      }
      if(!node_robothw.getParam("hyy_controller", hyy_controller))
      {
         ROS_ERROR_STREAM("ros con't find paramber \"hyy_controller\"");
         return 0;
      }

      if (!cm.loadController(controllers_namespace+"/"+trajectory_controller))
      {
         ROS_ERROR_STREAM(trajectory_controller<<" controller load failure!");
         return 0;
      }
      if (!cm.loadController(controllers_namespace+"/"+hyy_controller))
      {
         ROS_ERROR_STREAM(hyy_controller<<" controller load failure!");
         return 0;
      }
      if (!cm.loadController(controllers_namespace+"/"+state_controller))
      {
         ROS_ERROR_STREAM(state_controller<<" controller load failure!");
         return 0;
      }
   }

   ros::Time _time(0,0);
   ros::Duration d_time(0,communication_time);//s,ns, =1ms

   ros::AsyncSpinner spinner(1);
   spinner.start();

   HYYRobotBase::RTimer timer;
   if (0!=HYYRobotBase::initUserTimer(&timer, 0, 1))
   {
      ROS_ERROR_STREAM("device_timer start failure!");
   }
   ROS_INFO("------------start robothw loop----------------");
   struct timeval start,end;
   int __dtime=0;
   _time=ros::Time::now();
   while (ros::ok())
   {
      if (sim_flag)
      {
         d_time.sleep();
      }
      else
      {
         HYYRobotBase::userTimer(&timer);
      }
      gettimeofday(&start,NULL);
      robot_hw.read(_time, d_time);
      _time=_time+d_time;
      cm.update(_time, d_time);
      robot_hw.write(_time, d_time);
      gettimeofday(&end,NULL);
      __dtime=(end.tv_sec*1000000+end.tv_usec)-(start.tv_sec*1000000+start.tv_usec);
      if (__dtime>(communication_time/1000*0.8))
      {
         ROS_ERROR("over time, %d us",__dtime);
      }

   }
   return 1;
}


