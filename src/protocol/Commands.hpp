/***********************************************************
 * 
 * Definition of Commands.hpp
 * 
 * Date: 18.09.2023
 * Version: 1.0
 * Creator: Sebastian Ehnert
 *  
 ***********************************************************/

#ifndef PROTOCOL_COMMANDS_HPP
#define PROTOCOL_COMMANDS_HPP

#include <inttypes.h>

enum class Commands : uint8_t
{ 
    ChangeAngleAndSpeed = 0x10,
    CalibrateDevice = 0x11
};

#endif // - PROTOCOL_COMMANDS_HPP