/***********************************************************
 * 
 * Definition of ReturnState.hpp
 * 
 * Date: 11.09.2023
 * Version: 1.0
 * Creator: Sebastian Ehnert
 *  
 ***********************************************************/

#ifndef PROTOCOL_RETURNSTATE_HPP
#define PROTOCOL_RETURNSTATE_HPP

#include <inttypes.h>

enum class ReturnState : uint8_t
{ 
    Success = 0x01,
    Error = 0x02,
};

#endif // - PROTOCOL_RETURNSTATE_HPP