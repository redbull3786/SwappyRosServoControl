/***********************************************************
 * 
 * Definition of NodeConfiguration.hpp
 * 
 * Date: 18.09.2023
 * Version: 1.0
 * Creator: Sebastian Ehnert
 *  
 ***********************************************************/

#ifndef CONFIGURATION_NODECONFIGURATION_HPP
#define CONFIGURATION_NODECONFIGURATION_HPP

#include <string>

struct NodeConfiguration
{
   // Name of the node 
   std::string NodeName;

   // Name of published element
   std::string PublisherName;
};

#endif // - CONFIGURATION_NODECONFIGURATION_HPP