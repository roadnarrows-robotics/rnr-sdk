////////////////////////////////////////////////////////////////////////////////
//
// Package:   Dynamixel
//
// Library:   libdxl
//
// File:      dxl_hal.h
//
/*! \file
 *
 * \brief Dynamixel Hardware Abstraction Layer implementation interface.
 *
 * Based on Robotis Inc. dxl_sdk-1.01
 *
 * \author Robotis
 * \author Robin Knight (robin.knight@roadnarrows.com)
 */
/*
 * This file is a modified version of the file freely provided by Robotis Inc.
 * No restrictions by Robotis Inc. nor RoadNarrows LLC are made.
 */
////////////////////////////////////////////////////////////////////////////////

#ifndef _DXLHAL_H
#define _DXLHAL_H

#include <sys/types.h>
#include <stdint.h>

#include <string>

namespace libdxl
{
#define MAX_RTD       254         ///< max return time delay value for servos
#define SEC_PER_RTD   (0.000002)  ///< seconds per RTD value (2 usec)
#define MAX_T_RTD     ((double)MAX_RTD * SEC_PER_RTD)
                                  ///< maximum time delay at maximum RTD

  class dxlhal
  {
  public:
    /*!
     * \brief Default constructor.
     */
    dxlhal();

    /*!
     * \brief Destructor.
     */
    ~dxlhal();

    /*!
     * \brief Open Dynamixel Bus USB serial device by name.
     *
     * The serial device is opened and attributes set.
     *
     * \param deviceName    Name of serial device.
     * \param baudrate      Serial baud rate.
     *
     * \return Returns 1 on success, 0 on failure.
     */
    int open(const char *deviceName, int baudrate);

    /*!
     * \brief Open Dynamixel Bus USB serial device by index.
     *
     * The serial device is opened and attributes set.
     *
     * \note This method is deprecated. The preferred method is open().
     *
     * \param deviceIndex   Device number of /dev/ttyUSBn, n == deviceIndex.
     * \param baudrate      Serial baud rate.
     *
     * \return Returns 1 on success, 0 on failure.
     */
    int dxl_hal_open(int deviceIndex, float baudrate);
    
    /*!
     * \brief Close Dynamixel Bus serial interface.
     */
    void close();
    
    /*!
     * \brief Get the Dynamixel Bus serial device name.
     *
     * \return String.
     */
    std::string getDeviceName();
    
    /*!
     * \brief Get the Dynamixel Bus serial device baud rate.
     *
     * \return Returns baud rate.
     */
    int getBaudRate();
    
    /*!
     * \brief Get the Dynamixel Bus serial device file descriptor.
     *
     * \return Returns \>= 0 if the device is open, -1 otherwise.
     */
    int getFd();
    
    /*!
     * \brief Set opened Dynamixel Bus serial interface baud rate.
     *
     * \param baudrate  Baud rate.
     *
     * \return Returns 1 on success, 0 on failure.
     */
    int setBaudRate(int baudrate);
    
    /*!
     * \brief Discard any pending recieve data.
     */
    void clear();
    
    /*!
     * \brief Transmit Dynamixel instruction packet.
     *
     * \param [in] pPacket  Packed packet.
     * \param numPacket     Number of bytes to transmit.
     *
     * \return Returns 1 on success, 0 on failure.
     */
    int tx(unsigned char *pPacket, int numPacket);
    
    /*!
     * \brief Receive Dynamixel status packet.
     *
     * \param [out] pPacket   Packed packet.
     * \param numPacket       Number of bytes to read.
     *
     * \return On success, number of bytes received. -1 on error.
     */
    int rx(unsigned char *pPacket, int numPacket);
    
    /*!
     * \brief Set receive timeout.
     *
     * \param NumRcvByte  Number of expected receive bytes 
     */
    void setTimeout(int NumRcvByte);
    
    /*!
     * \brief Test if receive timeout has expired.
     *
     * \return Returns 1 if true, false otherwise.
     */
    bool hasTimedOut();
    
  protected:
    std::string m_strDeviceName;  ///< device name
    int         m_nBaudRate;      ///< device baud rate
    int         m_fd;             ///< device file descriptor
    double      m_fSecPerByte;    ///< seconds per byte transfer time 
    double      m_fStartTime;     ///< event start time
    double      m_fRcvWaitTime;   ///< receive max elapse wait time

    /*!
     * \brief Calculate seconds/byte transmit/recieve time in seconds.
     *
     * \note Over serial, START_BIT byte STOP_BIT = 10 bits.
     *
     * \param baudrate    Serial baud rate.
     *
     * \return Seconds per byte.
     */
    double calcSecPerByte(int baudrate);

    /*!
     * \brief Mark time.
     *
     * \return Return marked time in seconds.
     */
    double now();
  };


} // namespace libdxl

#endif // _DXLHAL_H
