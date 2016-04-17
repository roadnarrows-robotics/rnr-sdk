////////////////////////////////////////////////////////////////////////////////
//
// Package:   Dynamixel
//
// Library:   libdxl
//
// File:      dxl_hal.c
//
/*! \file
 *
 * \brief Dynamixel Hardware Abstraction Layer implementation.
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

#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <fcntl.h>
#include <termios.h>
#include <linux/serial.h>

#include <string>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/serdev.h"

#include "dxlhal.h"

using namespace std;
using namespace libdxl;

dxlhal::dxlhal()
{
  m_nBaudRate     = 0;
  m_fd            = -1;
  m_fSecPerByte   = 0.0;
  m_fStartTime    = 0;
  m_fRcvWaitTime  = 0.0;
}

dxlhal::~dxlhal()
{
  close();
}

int dxlhal::open(const char *deviceName, int baudrate)
{
  close();

  m_strDeviceName = deviceName;
  m_nBaudRate     = baudrate;

  m_fd = SerDevOpen(m_strDeviceName.c_str(), baudrate, 8, 'N', 1, false, false);

  if( m_fd >= 0 )
  {
    m_fSecPerByte = calcSecPerByte(baudrate);
  
    return 1;
  }
  else
  {
    LOGSYSERROR("%s: Failed to open.", m_strDeviceName.c_str());
  
    m_strDeviceName.clear();
    m_nBaudRate      = 0;

    return 0;
  }
}

int dxlhal::dxl_hal_open(int deviceIndex, float baudrate)
{
  char buf[256];
  struct termios newtio;
  struct serial_struct serinfo;

  close();

  snprintf(buf, sizeof(buf), "/dev/ttyUSB%d", deviceIndex);

  buf[sizeof(buf)-1] = 0;

  m_strDeviceName = buf;
  m_nBaudRate     = baudrate;

  memset(&newtio, 0, sizeof(newtio));
  
  if( (m_fd = open(m_strDeviceName.c_str(), O_RDWR|O_NOCTTY|O_NONBLOCK)) < 0 )
  {
    LOGSYSERROR("%s: Device open error.", m_strDeviceName.c_str());
    goto DXL_HAL_OPEN_ERROR;
  }

  newtio.c_cflag    = B38400|CS8|CLOCAL|CREAD;
  newtio.c_iflag    = IGNPAR;
  newtio.c_oflag    = 0;
  newtio.c_lflag    = 0;
  newtio.c_cc[VTIME]  = 0;  // time-out 값 (TIME * 0.1초) 0 : disable
  newtio.c_cc[VMIN]  = 0;  // MIN 은 read 가 return 되기 위한 최소 문자 개수

  tcflush(m_fd, TCIFLUSH);
  tcsetattr(m_fd, TCSANOW, &newtio);
  
  if(ioctl(m_fd, TIOCGSERIAL, &serinfo) < 0)
  {
    LOGSYSERROR("%s(fd=%d): Cannot get serial info.",
        m_strDeviceName.c_str(), m_fd);
    return 0;
  }
  
  serinfo.flags &= (int)(~ASYNC_SPD_MASK);
  serinfo.flags |= (int)ASYNC_SPD_CUST;
  serinfo.custom_divisor = (int)((float)serinfo.baud_base / baudrate);
  
  if(ioctl(m_fd, TIOCSSERIAL, &serinfo) < 0)
  {
    LOGSYSERROR("%s(fd=%d): Cannot set serial info.",
        m_strDeviceName.c_str(), m_fd);
    goto DXL_HAL_OPEN_ERROR;
  }
  
  m_fSecPerByte = calcSecPerByte(baudrate);
  
  return 1;

DXL_HAL_OPEN_ERROR:
  close();
  return 0;
}

void dxlhal::close()
{
  if( m_fd != -1 )
  {
    ::close(m_fd);
  }

  m_strDeviceName.clear();
  m_nBaudRate = 0;
  m_fd        = -1;
}

string dxlhal::getDeviceName()
{
  return m_strDeviceName;
}

int dxlhal::getBaudRate()
{
  return m_nBaudRate;
}

int dxlhal::getFd()
{
  return m_fd;
}

int dxlhal::setBaudRate(int baudrate)
{
  struct serial_struct serinfo;
  
  if( m_fd == -1 )
  {
    return 0;
  }
  
  if(ioctl(m_fd, TIOCGSERIAL, &serinfo) < 0)
  {
    LOGERROR("%s(fd=%d): Cannot get serial info.",
        m_strDeviceName.c_str(), m_fd);
    return 0;
  }
  
  serinfo.flags &= (int)(~ASYNC_SPD_MASK);
  serinfo.flags |= (int)ASYNC_SPD_CUST;
  serinfo.custom_divisor = (int)((float)serinfo.baud_base / (float)baudrate);
  
  if(ioctl(m_fd, TIOCSSERIAL, &serinfo) < 0)
  {
    LOGERROR("%s(fd=%d): Cannot set serial info.",
        m_strDeviceName.c_str(), m_fd);
    return 0;
  }
  
  m_fSecPerByte = calcSecPerByte(baudrate);

  m_nBaudRate = baudrate;

  return 1;
}

void dxlhal::clear(void)
{
  tcflush(m_fd, TCIFLUSH);
}

int dxlhal::tx( unsigned char *pPacket, int numPacket )
{
  ssize_t       n;

  n = SerDevWrite(m_fd, pPacket, (size_t)numPacket, 0);

  if( n > 0 )
  {
    return (int)n;
  }
  else
  {
    LOGERROR("tx failed.");
    return 0;
  }
}

int dxlhal::rx(unsigned char *pPacket, int numPacket)
{
  static unsigned int Slop = 1000;    // 1 millisecond slop

  unsigned int  usec;
  ssize_t       n;

  memset(pPacket, 0, (size_t)numPacket);

  usec = (unsigned int)((double)numPacket * m_fSecPerByte * 1000000.0) + Slop;

  //fprintf(stderr, "DBG: rx wait time for %d bytes is %uus.\n",
  //    numPacket, usec);

  n = SerDevRead(m_fd, pPacket, (size_t)numPacket, usec);

  return n < 0? 0: (int)n;
}

void dxlhal::setTimeout(int NumRcvByte)
{
  static double  Fudge = 0.010;    // yea ol fudge factor

  m_fStartTime = now();

  m_fRcvWaitTime = m_fSecPerByte * (double)NumRcvByte + MAX_T_RTD + Fudge;

  //fprintf(stderr, "DBG: to wait time = %lfs from now = %lfs.\n",
  //    m_fRcvWaitTime, m_fStartTime);
}

bool dxlhal::hasTimedOut()
{
  double  t1, t;
  
  t1  = now();
  t   = t1 - m_fStartTime;
  
  //fprintf(stderr, "DBG: st=%lf, now=%lf, elapse=%lf\n", m_fStartTime, t1, t);

  if( t > m_fRcvWaitTime )
  {
    return true;
  }

  else
  {
    return false;
  }
}

double dxlhal::now()
{
  struct timespec ts;

  clock_gettime(CLOCK_REALTIME, &ts);
  
  return (double)ts.tv_sec + (double)ts.tv_nsec / 1000000000.0;
}

double dxlhal::calcSecPerByte(int baudrate)
{
  if( baudrate > 0 )
  {
    return 10.0/(double)baudrate;
  }
  else
  {
    return 0.0;
  }
}
