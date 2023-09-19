/***********************************************************
 * 
 * Implementation of Handler.hpp
 * 
 * Date: 18.09.2023
 * Version: 1.0
 * Creator: Sebastian Ehnert
 *  
 ***********************************************************/

#include "Handler.hpp"
#include "Commands.hpp"
#include <cstring>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <ros/console.h>

Handler::Handler(const SpiConfiguration& spiConfig)
: m_SpiConfiguration(spiConfig),
  m_FileDescriptor(-1),
  m_SendBuffer(),
  m_ReceiveBuffer()
{
   init();
}

Handler::~Handler()
{
   deinit();
}

/********************
* Public Functions:
*********************/
void Handler::changeAngleAndSpeed(const Device device, const uint8_t angle_degree, const uint16_t speed_ms)
{
    ReturnState rc = ReturnState::Error;
    createChangeAngleAndSpeedRequest(device, angle_degree, speed_ms);

    rc = transfer(m_SendBuffer, m_ReceiveBuffer, 10);
    if (rc != ReturnState::Success)
    {
       ROS_ERROR("Command: %s failed.", __FUNCTION__);
    }

    prepairMemory(); 
}

void Handler::calibrateDevice(const Device device)
{
   ReturnState rc = ReturnState::Error;

   prepairMemory();
   createCalibrateDeviceRequest(device);

   rc = transfer(m_SendBuffer, m_ReceiveBuffer, 5);
   if (rc != ReturnState::Success)
   {
      ROS_ERROR("Command: %s failed.", __FUNCTION__);
   }
}

/********************
* Private Functions:
*********************/

/********************
* SPI Functions:
*********************/
ReturnState Handler::transfer(uint8_t* sendBuffer, uint8_t* receivedBuffer, uint16_t bufferSize)
{
   ReturnState rc = ReturnState::Success;
   uint8_t tmpBuffer[S_MAX_BUFFER_SIZE];
   memset(&tmpBuffer, 0, sizeof(tmpBuffer));

   //Transfer Request command to the sensor node:
   spi_ioc_transfer msg_request;
   memset(&msg_request, 0, sizeof(msg_request));

   msg_request.tx_buf = reinterpret_cast<unsigned long long>(sendBuffer);
   msg_request.rx_buf = reinterpret_cast<unsigned long long>(tmpBuffer);
   msg_request.len = bufferSize;
   msg_request.speed_hz = m_SpiConfiguration.SpiSpeed_Hz;
   msg_request.delay_usecs = 10;
   msg_request.bits_per_word = m_SpiConfiguration.BitsPerWord;
   msg_request.cs_change = m_SpiConfiguration.CsChange;

   const int statusValueRequest = ioctl(m_FileDescriptor, SPI_IOC_MESSAGE(1), &msg_request);
   if (statusValueRequest < 0)
   {
      ROS_ERROR("Could not do SPI request transfer {FileDEscriptor: %d, Errno: %s}", statusValueRequest, strerror(errno));
      rc = ReturnState::Error;
   }

   //Wait a bit:
   ros::Duration(0.1).sleep();

   //Transfer Resonse:
   spi_ioc_transfer msg_response;
   memset(&msg_response, 0, sizeof(msg_response));
   memset(&tmpBuffer, 0, sizeof(tmpBuffer));

   uint8_t tmpReceivedBuffer[S_MAX_BUFFER_SIZE];
   memset(&tmpReceivedBuffer, 0, sizeof(tmpReceivedBuffer));

   msg_response.tx_buf = reinterpret_cast<unsigned long long>(tmpBuffer);
   msg_response.rx_buf = reinterpret_cast<unsigned long long>(tmpReceivedBuffer);
   msg_response.len = bufferSize;
   msg_response.speed_hz = m_SpiConfiguration.SpiSpeed_Hz;
   msg_response.delay_usecs = 10;
   msg_response.bits_per_word = m_SpiConfiguration.BitsPerWord;
   msg_response.cs_change = m_SpiConfiguration.CsChange;

   const int statusValueResponse = ioctl(m_FileDescriptor, SPI_IOC_MESSAGE(1), &msg_request);
   if (statusValueResponse < 0)
   {
      ROS_ERROR("Could not do SPI response transfer {FileDEscriptor: %d, Errno: %s}", statusValueResponse, strerror(errno));
      rc = ReturnState::Error;
   }
   else
   {
      // Remove first byte from received buffer:
      memcpy(receivedBuffer, &tmpReceivedBuffer[1], (S_MAX_BUFFER_SIZE - 1));
   }


   return rc;
}

ReturnState Handler::init()
{
   ROS_DEBUG("Init Handler");

   ReturnState rc = ReturnState::Success;

   m_FileDescriptor = ::open(m_SpiConfiguration.SpiDevice.c_str(), O_RDWR);
   if(m_FileDescriptor < 0)
   {
      ROS_ERROR("Could not open SPI device");
      rc = ReturnState::Error;
   }

   const uint8_t mode = SPI_MODE_3;
   int ioctlStatus = ioctl(m_FileDescriptor, SPI_IOC_WR_MODE, &mode);
   if (ioctlStatus < 0)
   {
      ROS_ERROR("Could not set SPI Mode {FileDEscriptor: %d, Errno: %s}", ioctlStatus, strerror(errno));
      rc = ReturnState::Error;
   }

   ioctlStatus = ioctl(m_FileDescriptor, SPI_IOC_WR_BITS_PER_WORD, &m_SpiConfiguration.BitsPerWord);
   if (ioctlStatus < 0)
   {
      ROS_ERROR("Could not set SPI bits per word {FileDEscriptor: %d, Errno: %s}", ioctlStatus, strerror(errno));
      rc = ReturnState::Error;
   }

   ioctlStatus = ioctl(m_FileDescriptor, SPI_IOC_WR_MAX_SPEED_HZ, &m_SpiConfiguration.SpiSpeed_Hz);
   if (ioctlStatus < 0)
   {
      ROS_ERROR("Could not set SPI max speed {FileDEscriptor: %d, Errno: %s}", ioctlStatus, strerror(errno));
      rc = ReturnState::Error;
   }

   return rc;
}

ReturnState Handler::deinit()
{
   ROS_DEBUG("Deinit Handler");

   ReturnState rc = ReturnState::Success;

   int statusValue = ::close(m_FileDescriptor);
   if (statusValue < 0)
   {
      ROS_ERROR("Could not close SPI device {FileDEscriptor: %d, Errno: %s}", statusValue, strerror(errno));
      rc = ReturnState::Error;
   }

   return rc;
}

/********************
* Command Functions:
*********************/
void Handler::prepairMemory()
{
   memset(m_SendBuffer, 0, S_MAX_BUFFER_SIZE);
   memset(m_ReceiveBuffer, 0, S_MAX_BUFFER_SIZE);
}

void Handler::createChangeAngleAndSpeedRequest(const Device device, const uint8_t angle_degree, const uint16_t speed_ms)
{
   m_SendBuffer[0] = static_cast<uint8_t>(Commands::ChangeAngleAndSpeed);
   m_SendBuffer[1] = static_cast<uint8_t>(device);
   m_SendBuffer[2] = static_cast<uint8_t>(angle_degree);
   m_SendBuffer[3] = static_cast<uint8_t>(speed_ms);
   m_SendBuffer[4] = static_cast<uint8_t>(speed_ms >> 8);
}

ReturnState Handler::parseChangeAngleAndSpeedResponse()
{
    return static_cast<ReturnState>(m_ReceiveBuffer[0]);
}

void Handler::createCalibrateDeviceRequest(const Device device)
{
   m_SendBuffer[0] = static_cast<uint8_t>(Commands::CalibrateDevice);
   m_SendBuffer[1] = static_cast<uint8_t>(device);
}

ReturnState Handler::parseCalibrateDeviceResponse()
{
   return static_cast<ReturnState>(m_ReceiveBuffer[0]);
}
