/***********************************************************
 * 
 * Implementation of client.cpp
 * 
 * Date: 19.09.2023
 * Version: 1.0
 * Creator: Sebastian Ehnert
 * 
 * Description:
 *   Implements a subscriber to get Commands from ROS Master
 *   to Control the Servos.
 * 
 *   Arguments:
 *   - NodeName=<Name>
 *   - SubscriberName=<Name_Of_Subcribed_Element>
 *   - SpiDevice=<SpiDevice>
 *  
 ***********************************************************/

#include "ros/ros.h"
#include "configuration/ClientConfiguration.hpp"
#include <ros/console.h>
#include "Arguments.hpp"
#include "protocol/Handler.hpp"
#include "Wrapper.hpp"

Arguemnts getKeyValue(const std::string& arg)
{
   Arguemnts tmp;

   size_t pos = arg.find("=");
   tmp.Key = arg.substr(0, pos);
   tmp.Value = arg.substr((pos + 1), (arg.size() - (pos + 1)));

   return tmp;
}

ClientConfiguration parseArguments(int argc, char **argv)
{
   ClientConfiguration tmp;

   // parse elements:
   for (int i = 1; i < argc; i++)
   {
      Arguemnts argument = getKeyValue(std::string(argv[i]));

      if (argument.Key == std::string("NodeName"))
      {
         tmp.NodeName = argument.Value; //"SensorHClientFront";
      }
      else if (argument.Key == std::string("SubscriberName"))
      {
         tmp.SubscriberName = argument.Value; //"frontSensors";
      }
      else if (argument.Key == std::string("SpiDevice"))
      {
         tmp.SpiConfig.SpiDevice = argument.Value; //"/dev/spi/spi1.0"
         tmp.SpiConfig.SpiSpeed_Hz = 17000000;
         tmp.SpiConfig.CsChange = 1;
         tmp.SpiConfig.BitsPerWord = 8;
      }
   }

   return tmp;   
}

int main(int argc, char **argv)
{
   // This needs to happen before we start fooling around with logger levels.  Otherwise the level we set may be overwritten by
   // a configuration file
   ROSCONSOLE_AUTOINIT;

   ClientConfiguration config = parseArguments(argc, argv);

   ROS_INFO("Start Servo Control Subscriber: %s", config.NodeName.c_str());

   ros::init(argc, argv, config.NodeName);

   Handler handler(config.SpiConfig);
   Wrapper wrapper(handler);

   ros::NodeHandle n;
   ros::Subscriber sub = n.subscribe(config.SubscriberName, 1000, &Wrapper::servoCallback, &wrapper);

   ros::spin();

   return 0;
}