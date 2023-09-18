/***********************************************************
 * 
 * Definition of SpiConfiguration.hpp
 * 
 * Date: 18.09.2023
 * Version: 1.0
 * Creator: Sebastian Ehnert
 *  
 ***********************************************************/

#ifndef CONFIGURATION_SPICONFIGURATION_HPP
#define CONFIGURATION_SPICONFIGURATION_HPP

#include <inttypes.h>
#include <string>

struct SpiConfiguration
{
   std::string SpiDevice;
   uint32_t SpiSpeed_Hz;
   uint8_t CsChange;
   uint8_t BitsPerWord;
};

#endif // - CONFIGURATION_SPICONFIGURATION_HPP