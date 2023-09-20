/***********************************************************
 * 
 * Definition of Wrapper.hpp
 * 
 * Date: 20.09.2023
 * Version: 1.0
 * Creator: Sebastian Ehnert
 *  
 ***********************************************************/

#ifndef WRAPPER_HPP
#define WRAPPER_HPP

#include "protocol/Handler.hpp"
#include "swappy_ros_servo_control/ServoControlState.h"

class Wrapper
{
public:
    Wrapper(Handler& handler);
    ~Wrapper() = default;

    void servoCallback(const swappy_ros_servo_control::ServoControlState::ConstPtr& msg);

private:
    Handler& m_Handler;
};

#endif // - WRAPPER_HPP