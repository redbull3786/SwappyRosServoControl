/***********************************************************
 * 
 * Definition of Handler.hpp
 * 
 * Date: 18.09.2023
 * Version: 1.0
 * Creator: Sebastian Ehnert
 *  
 ***********************************************************/

#ifndef PROTOCOL_HANDLER_HPP
#define PROTOCOL_HANDLER_HPP

#include <inttypes.h>
#include <string>
#include "Device.hpp"
#include "ReturnState.hpp"
#include "../configuration/SpiConfiguration.hpp"

class Handler
{
public:
	Handler(const SpiConfiguration& spiConfig);
	~Handler();

    void changeAngleAndSpeed(const Device device, const uint8_t angle_degree, const uint16_t speed_ms);
	void calibrateDevice(const Device device);

private:
	static const uint8_t S_MAX_BUFFER_SIZE = 31;

    SpiConfiguration m_SpiConfiguration;
	int m_FileDescriptor;
	uint8_t m_SendBuffer[S_MAX_BUFFER_SIZE];
	uint8_t m_ReceiveBuffer[S_MAX_BUFFER_SIZE];

    // SPI:
	ReturnState transfer(uint8_t* sendBuffer, uint8_t* receivedBuffer, uint16_t bufferSize);
	ReturnState init();
	ReturnState deinit();

    // Commands:
	void prepairMemory();

    void createChangeAngleAndSpeedRequest(const Device device, const uint8_t angle_degree, const uint16_t speed_ms);
    ReturnState parseChangeAngleAndSpeedResponse();

	void createCalibrateDeviceRequest(const Device device);
	ReturnState parseCalibrateDeviceResponse(); 
};

#endif // - PROTOCOL_HANDLER_HPP