/***********************************************************
 * 
 * Implementation of Wrapper.hpp
 * 
 * Date: 20.09.2023
 * Version: 1.0
 * Creator: Sebastian Ehnert
 *  
 ***********************************************************/

#include "Wrapper.hpp"
#include <ros/console.h>

Wrapper::Wrapper(Handler& handler)
: m_Handler(handler)
{
    
}

void Wrapper::servoCallback(const swappy_ros_servo_control::ServoControlState::ConstPtr& msg)
{
    ROS_INFO("TEST");
}