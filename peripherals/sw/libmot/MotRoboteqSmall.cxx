////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Peripherals
//
// Library:   libmot
//
// File:
// MotRoboteqSmall.cxx
//
/*! \file
 *
 * $LastChangedDate: 2013-09-24 09:34:42 -0600 (Tue, 24 Sep 2013) $
 * $Rev: 3323 $
 *
 * \brief Common Human Interface Device Interface.
 *
 * \author: Robin Knight      (robin.knight@roadnarrows.com)
 * \author: Daniel Packard    (daniel@roadnarrows.com)
 * \author: Jessica Trujillo  (jessica@roadnarrows.com)
 * \author: Maurice Woods III (maurice@roadnarrows.com)
 *
 *
 * \copyright
 *   \h_copy 2012-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
 */
// Permission is hereby granted, without written agreement and without
// license or royalty fees, to use, copy, modify, and distribute this
// software and its documentation for any purpose, provided that
// (1) The above copyright notice and the following two paragraphs
// appear in all copies of the source code and (2) redistributions
// including binaries reproduces these notices in the supporting
// documentation.   Substantial modifications to this software may be
// copyrighted by their authors and need not follow the licensing terms
// described here, provided that the new terms are clearly indicated in
// all files where they apply.
//
// IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
// OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
// PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
// EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
// THE POSSIBILITY OF SUCH DAMAGE.
//
// THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
// INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
// FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
// "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
// PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
//
////////////////////////////////////////////////////////////////////////////////


#include <vector>
#include <stdio.h>
#include <string.h>
#include <sstream>
#include "rnr/log.h"
#include "rnr/rnrconfig.h"
#include "rnr/units.h"
//#include "rnr/mot/MotRoboteq.h"
#include "rnr/mot/MotRoboteqSmall.h"
#include "rnr/serdev.h"

using namespace rnr;
using namespace std;

// DHP - Todo: move to libserial
int SerDevReadline(int fd, byte_t *buf, int timeout)
{
  stringstream sstr;
  char c;

  while ((c = (char)SerDevGetc(fd, timeout)) != '\r')
  {
    sstr << c;
  }

  fprintf(stderr, "response received: %s\n", sstr.str().c_str());

  strcpy((char*)buf,sstr.str().c_str());
  
  return sstr.str().size();
}

int MotRoboteqSmall::open(const string &devName, int baudRate)
{
  LOGDIAG3("jnt devName= %s baudRate= %d \n", devName.c_str(), baudRate);
  if ((m_fd=SerDevOpen(devName.c_str(),baudRate,8,'N',1,0,0))==-1)
  {
    fprintf(stderr, "dhp -here\n");
    return -1;
  }
    fprintf(stderr, "dhp -here 2\n");
  return 0;
}

int MotRoboteqSmall::close()
{
  return SerDevClose(m_fd);
}

// DHP - Move this to the base class
int MotRoboteqSmall::sendCommand(int fd, byte_t* buf, int nBytes, int timeout)
{
  SerDevWrite(fd, buf, nBytes, timeout);
  fprintf(stderr, "sent command: %s\n", buf);
  return 0;
}

// DHP - Move these to the base class
int MotRoboteqSmall::recvResponse(int fd, byte_t *buf, int timeout)
{
  char c;
  int nBytes=0;

  // DHP - need to add variable to check if echo is turned on
  // if(m_bEchoIsOn) 
  {
    // grab echo response
    SerDevReadline(fd, buf, timeout);
  }

  // grab actual response
  nBytes = SerDevReadline(fd, buf, timeout);

  return nBytes;
}

bool MotRoboteqSmall::motIDIsValid(int motID)
{
  if (motID >2 || motID <1)
  {
    LOGDIAG3("Not a valid motor ID.\n");
    return false;
  }
  return true;
}

int MotRoboteqSmall::setSpeed(int motID, float speed, units_t units)
{
  int rawSpeed;
  if (!motIDIsValid(motID))
  {
    return -1;
  }

  if (units == units_norm)
  {
    if (speed >1 || speed <-1)
    {
      LOGDIAG3("Not a valid normalized motor speed.\n");
      return -1;
    }
    rawSpeed=(int)(speed*1000);
  }

  return setSpeedRaw(motID, rawSpeed);
}

int MotRoboteqSmall::setSpeed(VecSpeedTupples vecSpeedTupple, units_t units)
{
  VecSpeedTupples::iterator iter;
  VecSpeedRawTupples vecSpeedRawTupple;
  for (iter=vecSpeedTupple.begin();iter!=vecSpeedTupple.end();iter++)
      {
        IDFloatTupple src = *iter;
        IDIntTupple dest;
        if (!motIDIsValid(src.m_nMotID))
        {
          return -1;
        }
        dest.m_nMotID = src.m_nMotID;
        if (src.m_fVal >1 || src.m_fVal <-1)
        {
          LOGDIAG3("Not a valid normalized motor speed. \n");
          return -1;
        }
        dest.m_nVal = (int)(1000 * src.m_fVal);
        vecSpeedRawTupple.push_back(dest);
      }
  setSpeedRaw(vecSpeedRawTupple);
  return -1;
}

int MotRoboteqSmall::setSpeedRaw(int motID, int rawSpeed)
{
  char cmd[32];
  char res[32];
  
  sprintf(cmd,"!G %d %d\r", motID, rawSpeed);
  int nBytes = strlen(cmd);

  sendCommand(m_fd, (byte_t*)cmd, nBytes, m_ntimeout);
  nBytes = recvResponse(m_fd, (byte_t*)res, m_ntimeout);

  if(res[0] == '-')
  {
    LOGERROR("setSpeedRaw(id,speed) command failed.\n");
  }

}

int MotRoboteqSmall::setSpeedRaw(VecSpeedRawTupples vecSpeedRawTupple)
{
  stringstream str;
  char buf[32];

  VecSpeedRawTupples::iterator iter;
  for (iter=vecSpeedRawTupple.begin();iter!=vecSpeedRawTupple.end();iter++)
  {
    str << "!G " << (*iter).m_nMotID << " " << (*iter).m_nVal << "_";
  }

  sendCommand(m_fd, (byte_t *)str.str().c_str(), str.str().length(), m_ntimeout);

  // check ALL responses
  for( int i=0; i < vecSpeedRawTupple.size(); i++)
  {
    int nBytes = recvResponse(m_fd, (byte_t*)buf , m_ntimeout);

    // check  (+/-)
    if (buf[0] =='-')
    {
      LOGERROR("setSpeedRaw(vec) command failed\n");
      return -1;
    }
  }
}

int MotRoboteqSmall::stop(int motID)
{
  setSpeedRaw(motID, 0);
}
 
int MotRoboteqSmall::stop()
{
  setSpeedRaw(1, 0);
  setSpeedRaw(2, 0);
}

int MotRoboteqSmall::eStop()
{
  setSpeedRaw(1, 0);
  setSpeedRaw(2, 0);

  char buf[32];

  char cmd[32];
  char res[32];
  sprintf(cmd,"!EX\r");
  int nBytes = strlen(cmd);

  sendCommand(m_fd, (byte_t*)cmd, nBytes, m_ntimeout);
  recvResponse(m_fd, (byte_t*)res, m_ntimeout);
  // check +/-
    if (buf[0] =='-')
    {
      LOGERROR("setSpeedRaw(vec) command failed\nIs firmware updated?\n");
      return -1;
    }
}

int MotRoboteqSmall::eStopRelease()
{
  setSpeedRaw(1, 0);
  setSpeedRaw(2, 0);
  
  char buf[32];

  char cmd[32];
  char res[32];
  sprintf(cmd,"!MG\r");
  int nBytes = strlen(cmd);

  sendCommand(m_fd, (byte_t*)cmd, nBytes, m_ntimeout);
  recvResponse(m_fd, (byte_t*)res, m_ntimeout);
  // check +/-
  if (buf[0] =='-')
  {
    LOGERROR("setSpeedRaw(vec) command failed\nIs firmware updated?\n");
    return -1;
  }
}

int MotRoboteqSmall::getCurrent(int motID, units_t units)
{
  if (!motIDIsValid(motID))
  {
    return -1;
  }

  char cmd[32];
  char res[32];
  sprintf(cmd,"?A %d\r", motID);
  int nBytes = strlen(cmd);

  sendCommand(m_fd, (byte_t *)cmd, nBytes, m_ntimeout);
  recvResponse(m_fd, (byte_t *)res, m_ntimeout);

  // TODO : parse response

  return 0;
}

int MotRoboteqSmall::getCurrentLimits()
{
  char cmd[32];
  char res[32];
  sprintf(cmd,"~ALIM\r");
  int nBytes = strlen(cmd);

  sendCommand(m_fd, (byte_t *)cmd, nBytes, m_ntimeout);
  recvResponse(m_fd, (byte_t*)res, m_ntimeout);

  // TODO : parse response

  return 0;
}

int MotRoboteqSmall::setCurrentLimits(int motID, int current, units_t units)
{
  char buf[32];

  char cmd[32];
  char res[32];
  //todo: compare current argument to 75% of SDC2130 rating. 
  int rawCurrent = current *10;
  sprintf(cmd,"^ALIM %d %d\r", motID, rawCurrent);
  int nBytes = strlen(cmd);

  sendCommand(m_fd, (byte_t *)cmd, nBytes, m_ntimeout);
  recvResponse(m_fd, (byte_t*)res, m_ntimeout);

  // check response +/-
  if (buf[0] =='-')
  {
    LOGERROR("setSpeedRaw(vec) command failed\n");
    return -1;
  }

  return 0;
}

int MotRoboteqSmall::getVoltageLimits()
{
  char cmd[32];
  char res[32];
  sprintf(cmd,"~UVL_~OVL\r");
  int nBytes = strlen(cmd);

  sendCommand(m_fd, (byte_t *)cmd, nBytes, m_ntimeout);

  recvResponse(m_fd,(byte_t *)res, m_ntimeout);
  // TODO : parse response
  recvResponse(m_fd,(byte_t *)res, m_ntimeout);
  // TODO : parse response
  return 0;
}

int MotRoboteqSmall::getVoltage()
{
  char cmd[32];
  char res[32];
  sprintf(cmd,"?V\r");
  int nBytes = strlen(cmd);

  sendCommand(m_fd, (byte_t *)cmd, nBytes, m_ntimeout);
  recvResponse(m_fd,(byte_t *)res, m_ntimeout);

  // TODO : parse response
  return 0;
}

int MotRoboteqSmall::setVoltageLimits(int lowVoltage, int overVoltage)
{
  if (lowVoltage < 5)
  {
    LOGDIAG3("Not a valid lower voltage limit. Must be greater than 5V \n");
    return -1;
  }

  int lowVoltageRaw=lowVoltage*10;
  int overVoltageRaw=overVoltage*10;

  char buf[32];

  char cmd[32];
  char res[32];
  sprintf(cmd,"^UVL %d_^OVL %d\r", lowVoltageRaw, overVoltageRaw);
  int nBytes = strlen(cmd);

  sendCommand(m_fd, (byte_t *)cmd, nBytes, m_ntimeout);
  recvResponse(m_fd,(byte_t *)res, m_ntimeout);
  // check +/-
  if (buf[0] =='-')
  {
    LOGERROR("setSpeedRaw(vec) command failed\n");
    return -1;
  }
  recvResponse(m_fd,(byte_t *)res, m_ntimeout);
  // check +/-
  if (buf[0] =='-')
  {
    LOGERROR("setSpeedRaw(vec) command failed\n");
    return -1;
  }

  return 0;
}

// TODO : Check to ensure reversed motors aren't effected by accel/decel sign
int MotRoboteqSmall::setSpeedProfile(int motID, int accel, int decel)
{
  if (decel==DEF_DEC)
  {
    decel=accel;
  }

  char buf[32];

  char cmd[32];
  char res[32];
  sprintf(cmd,"^MAC %d %d_^MDEC %d %d\r", motID, accel, motID, decel);
  int nBytes = strlen(cmd);

  sendCommand(m_fd, (byte_t *)cmd, nBytes, m_ntimeout);
  recvResponse(m_fd,(byte_t *)res, m_ntimeout);
  // check +/-
  if (buf[0] =='-')
  {
    LOGERROR("setSpeedProfile() command failed\n");
    return -1;
  }
  recvResponse(m_fd,(byte_t *)res, m_ntimeout);
  // check +/-
  if (buf[0] =='-')
  {
    LOGERROR("setSpeedProfile() command failed\n");
    return -1;
  }

  return 0;

}
