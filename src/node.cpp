/***********************************************************
 * 
 * Implementation of node.cpp
 * 
 * Date: 19.09.2023
 * Version: 1.0
 * Creator: Sebastian Ehnert
 * 
 * Description:
 *   Implements a publisher to send Commands from ROS Master
 *   to Control the Servos.
 * 
 *   Arguments:
 *   - NodeName=<Name>
 *   - SubscriberName=<Name_Of_Subcribed_Element>
 *  
 ***********************************************************/

#include "ros/ros.h"
#include "swappy_ros_servo_control/ServoControlState.h"
#include "protocol/Handler.hpp"
#include "configuration/NodeConfiguration.hpp"
#include <ros/console.h>
#include "Arguments.hpp"


Arguemnts getKeyValue(const std::string& arg)
{
   Arguemnts tmp;

   size_t pos = arg.find("=");
   tmp.Key = arg.substr(0, pos);
   tmp.Value = arg.substr((pos + 1), (arg.size() - (pos + 1)));

   return tmp;
}

NodeConfiguration parseArguments(int argc, char **argv)
{
   NodeConfiguration tmp;

   // parse elements:
   for (int i = 1; i < argc; i++)
   {
      Arguemnts argument = getKeyValue(std::string(argv[i]));

      if (argument.Key == std::string("NodeName"))
      {
         tmp.NodeName = argument.Value; //"SercoControlClient";
      }
      else if (argument.Key == std::string("PublisherNode"))
      {
         tmp.PublisherName = argument.Value; //"servos";
      }
   }

   return tmp;   
}

void printServoState(const swappy_ros_servo_control::ServoControlState& msg)
{
   ROS_DEBUG("publish Message:");
   ROS_DEBUG("Servo1: {ActiveMode: %d Angle[deg]: %d Speed[ms]: %d}", msg.Servo1.ActiveMode, msg.Servo1.Angle, msg.Servo1.Speed);
   ROS_DEBUG("Servo2: {ActiveMode: %d Angle[deg]: %d Speed[ms]: %d}", msg.Servo2.ActiveMode, msg.Servo2.Angle, msg.Servo2.Speed);
   ROS_DEBUG("Servo3: {ActiveMode: %d Angle[deg]: %d Speed[ms]: %d}", msg.Servo3.ActiveMode, msg.Servo3.Angle, msg.Servo3.Speed);
   ROS_DEBUG("Servo4: {ActiveMode: %d Angle[deg]: %d Speed[ms]: %d}", msg.Servo4.ActiveMode, msg.Servo4.Angle, msg.Servo4.Speed);
   ROS_DEBUG("Servo5: {ActiveMode: %d Angle[deg]: %d Speed[ms]: %d}", msg.Servo5.ActiveMode, msg.Servo5.Angle, msg.Servo5.Speed);
   ROS_DEBUG("Servo6: {ActiveMode: %d Angle[deg]: %d Speed[ms]: %d}", msg.Servo6.ActiveMode, msg.Servo6.Angle, msg.Servo6.Speed);
   ROS_DEBUG("Servo7: {ActiveMode: %d Angle[deg]: %d Speed[ms]: %d}", msg.Servo7.ActiveMode, msg.Servo7.Angle, msg.Servo7.Speed);
   ROS_DEBUG("Servo8: {ActiveMode: %d Angle[deg]: %d Speed[ms]: %d}", msg.Servo8.ActiveMode, msg.Servo8.Angle, msg.Servo8.Speed);
}

int main(int argc, char **argv)
{
   // This needs to happen before we start fooling around with logger levels.  Otherwise the level we set may be overwritten by
   // a configuration file
   ROSCONSOLE_AUTOINIT;

   NodeConfiguration config = parseArguments(argc, argv);

   ROS_INFO("Start Sensor Publisher: %s", config.NodeName.c_str());

   ros::init(argc, argv, config.NodeName);
   ros::NodeHandle n;

   ros::Publisher node_pub = n.advertise<swappy_ros_servo_control::ServoControlState>(config.PublisherName, 1000);
   ros::Rate loop_rate(1); // 1Hz == 1 cycle per second

   while (ros::ok())
   {
      swappy_ros_servo_control::ServoControlState msg;
   
      node_pub.publish(msg);

      // Debug Log:
      printServoState(msg);

      ros::spinOnce();

      loop_rate.sleep();
   }

   return 0;
}