/***********************************************************
 * 
 * Definition of Device.hpp
 * 
 * Date: 18.09.2023
 * Version: 1.0
 * Creator: Sebastian Ehnert
 *  
 ***********************************************************/

#ifndef PROTOCOL_DEVICE_HPP
#define PROTOCOL_DEVICE_HPP

#include <inttypes.h>

enum class Device : uint8_t
{ 
    Unknown = 0x00,
    Servo1 = 0x01,
    Servo2 = 0x02,
    Servo3 = 0x03,
    Servo4 = 0x04,
    Servo5 = 0x05,
    Servo6 = 0x06,
    Servo7 = 0x06,
    Servo8 = 0x08,
    All = 0x09
};

#endif // - PROTOCOL_DEVICE_HPP
