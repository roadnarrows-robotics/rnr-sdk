////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// File:      laeImu.cxx
//
/*! \file
 *
 * \brief Laelaps built-in Inertial Measurement Unit class implementation.
 *
 * The current Laelaps uses the open-source CleanFlight firmware loaded
 * on a Naze32 controller. The interface is serial USB.
 *
 * \sa https://github.com/cleanflight
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright
 *   \h_copy 2015-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
 */
/*
 * @EulaBegin@
 * 
 * Unless otherwise stated explicitly, all materials contained are copyrighted
 * and may not be used without RoadNarrows LLC's written consent,
 * except as provided in these terms and conditions or in the copyright
 * notice (documents and software) or other proprietary notice provided with
 * the relevant materials.
 * 
 * IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY 
 * MEMBERS/EMPLOYEES/CONTRACTORS OF ROADNARROWS OR DISTRIBUTORS OF THIS SOFTWARE
 * BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
 * DOCUMENTATION, EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE AUTHORS AND  ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
 * "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 * 
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////

#include <pthread.h>
#include <unistd.h>
#include <math.h>

#include <string>
#include <sstream>

#include "rnr/rnrconfig.h"
#include "rnr/serdev.h"
#include "rnr/log.h"

#include  "Laelaps/laelaps.h"
#include  "Laelaps/laeUtils.h"
#include  "Laelaps/laeDesc.h"
#include  "Laelaps/laeTune.h"
#include  "Laelaps/laeImu.h"
#include  "Laelaps/laeDb.h"

using namespace std;
using namespace laelaps;
using namespace sensor::imu;
using namespace sensor::imu::msp;


//------------------------------------------------------------------------------
// Quaternion Class
//------------------------------------------------------------------------------

Quaternion Quaternion::operator=(const Quaternion &rhs)
{
  m_x = rhs.m_x;
  m_y = rhs.m_y;
  m_z = rhs.m_z;
  m_w = rhs.m_w;

  return *this;
}

void Quaternion::clear()
{
  m_x = 0.0;
  m_y = 0.0;
  m_z = 0.0;
  m_w = 1.0;
}

void Quaternion::convertEuler(double phi, double theta, double psi)
{
  double half_phi, half_theta, half_psi;
  double cos_phi, sin_phi;
  double cos_theta, sin_theta;
  double cos_psi, sin_psi;

  half_phi   = phi / 2.0;
  half_theta = theta / 2.0;
  half_psi   = psi / 2.0;

  cos_phi   = cos(half_phi);
  sin_phi   = sin(half_phi);
  cos_theta = cos(half_theta);
  sin_theta = sin(half_theta);
  cos_psi   = cos(half_psi);
  sin_psi   = sin(half_psi);

  m_x = cos_phi*cos_theta*cos_psi + sin_phi*sin_theta*sin_psi;
  m_y = sin_phi*cos_theta*cos_psi - cos_phi*sin_theta*sin_psi;
  m_z = cos_phi*sin_theta*cos_psi + sin_phi*cos_theta*sin_psi;
  m_w = cos_phi*cos_theta*sin_psi - sin_phi*sin_theta*cos_psi;
}

void Quaternion::convertTaitBryan(double roll, double pitch, double yaw)
{
  double half_roll  = 0.5 * roll;
  double half_pitch = 0.5 * pitch;
  double half_yaw   = 0.5 * yaw;

  double t0 = cos(half_yaw);
  double t1 = sin(half_yaw);
  double t2 = cos(half_roll);
  double t3 = sin(half_roll);
  double t4 = cos(half_pitch);
  double t5 = sin(half_pitch);

  m_w = t0 * t2 * t4 + t1 * t3 * t5;
  m_x = t0 * t3 * t4 - t1 * t2 * t5;
  m_y = t0 * t2 * t5 + t1 * t3 * t4;
  m_z = t1 * t2 * t4 - t0 * t3 * t5;
}


//------------------------------------------------------------------------------
// LaeImu Virtual Base Class
//------------------------------------------------------------------------------
 
LaeImu::LaeImu(string strIdent) : m_strIdent(strIdent)
{
  m_fd           = -1;
  m_bBlackListed = false;

  clearSensedData();

  pthread_mutex_init(&m_mutexIo, NULL);
  pthread_mutex_init(&m_mutexOp, NULL);
}

LaeImu::~LaeImu()
{
  close();

  pthread_mutex_destroy(&m_mutexOp);
  pthread_mutex_destroy(&m_mutexIo);
}

int LaeImu::open(const string &strDevName, int nBaudRate)
{
  string  strDevNameReal;
  int     rc;

  lockIo();

  // Get the real device name, not any symbolic links.
  strDevNameReal  = getRealDeviceName(strDevName);

  m_fd = SerDevOpen(strDevNameReal.c_str(), nBaudRate, 8, 'N', 1, false, false);

  if( m_fd < 0 )
  {
    LOGERROR("Failed to open %s@%d.", strDevNameReal.c_str(), nBaudRate);
    rc = -LAE_ECODE_NO_DEV;
  }

  else
  {
    m_strDevName    = strDevNameReal;
    m_nBaudRate     = nBaudRate;
    m_bBlackListed  = false;

    RtDb.m_enable.m_bImu = true;

    clearSensedData();

    LOGDIAG2("Opened interface to IMU on %s@%d.",
      strDevNameReal.c_str(), nBaudRate);

    rc = LAE_OK;
  }

  unlockIo();

  return rc;
}

int LaeImu::close()
{
  lockIo();

  if( m_fd >= 0 )
  {
    SerDevClose(m_fd);
    LOGDIAG2("Closed %s interface to IMU.", m_strDevName.c_str());
  }

  m_strDevName.clear();
  m_nBaudRate = 0;
  m_fd        = -1;

  unlockIo();

  return LAE_OK;
}

void LaeImu::blacklist()
{
  if( isOpen() )
  {
    close();
  }

  m_bBlackListed       = true;
  RtDb.m_enable.m_bImu = false;
}

void LaeImu::whitelist()
{
  m_bBlackListed       = false;
  RtDb.m_enable.m_bImu = true;
}

int LaeImu::configure(const LaeDesc &desc)
{
  return LAE_OK;
}

int LaeImu::configure(const LaeTunes &tunes)
{
  return LAE_OK;
}

int LaeImu::reload(const LaeTunes &tunes)
{
  return LAE_OK;
}

void LaeImu::clearSensedData()
{
  for(int i = 0; i < NumOfAxes; ++i)
  {
    m_accelRaw[i] = 0;
    m_gyroRaw[i]  = 0;
    m_magRaw[i]   = 0;
    m_rpyRaw[i]   = 0;

    m_accel[i]    = 0.0;
    m_gyro[i]     = 0.0;
    m_mag[i]      = 0.0;
    m_rpy[i]      = 0.0;
  }

  m_quaternion.clear();
}

void LaeImu::compute()
{
  computeQuaternion();
  computeDynamics();
}

void LaeImu::computeQuaternion()
{
  m_quaternion.convertTaitBryan(m_rpy[ROLL], m_rpy[PITCH], m_rpy[YAW]);
}

void LaeImu::computeDynamics()
{
}

void LaeImu::exec()
{
  if( isBlackListed() )
  {
    return;
  }

  lockOp();

  if( readRawImu() == LAE_OK )
  {
    convertRawToSI();
    compute();
  }

  unlockOp();
}

void LaeImu::getRawInertiaData(int accel[], int gyro[])
{
  for(int i = 0; i < NumOfAxes; ++i)
  {
    accel[i] = m_accelRaw[i];
    gyro[i]  = m_gyroRaw[i];
  }
}

void LaeImu::getInertiaData(double accel[], double gyro[])
{
  for(int i = 0; i < NumOfAxes; ++i)
  {
    accel[i] = m_accel[i];
    gyro[i]  = m_gyro[i];
  }
}

void LaeImu::getMagnetometerData(double mag[])
{
  for(int i = 0; i < NumOfAxes; ++i)
  {
    mag[i] = m_mag[i];
  }
}

void LaeImu::getAttitude(double rpy[])
{
  for(int i = 0; i < NumOfAxes; ++i)
  {
    rpy[i] = m_rpy[i];
  }
}

void LaeImu::getAttitude(double &roll, double &pitch, double &yaw)
{
  roll  = m_rpy[ROLL];
  pitch = m_rpy[PITCH];
  yaw   = m_rpy[YAW];
}

void LaeImu::getQuaternion(Quaternion &q)
{
  q = m_quaternion;
}

void LaeImu::getImuData(double accel[], double gyro[], double mag[],
                        double rpy[], Quaternion &q)
{
  lockOp();

  getInertiaData(accel, gyro);
  getMagnetometerData(mag);
  getAttitude(rpy);
  getQuaternion(q);

  unlockOp();
}


//------------------------------------------------------------------------------
// LaeImuCleanFlight IMU class with Cleanflight firmware on a supported board.
//------------------------------------------------------------------------------

LaeImuCleanFlight::LaeImuCleanFlight() :
  LaeImu("CleanFlight Naze32 with MPU-6050 IMU sensor")
{
}

LaeImuCleanFlight::~LaeImuCleanFlight()
{
}

int LaeImuCleanFlight::readIdentity(string &strIdent)
{
  MspIdent  ident;
  int       rc;

  if( isBlackListed() )
  {
    strIdent = "Unknown - blacklisted";
    rc = LAE_OK; 
  }

  else if( (rc = mspReadIdent(ident)) == LAE_OK )
  {
    stringstream  ss;

    ss << "CleanFlight v" << ident.m_uFwVersion
      << " Naze32 with MPU-6050 IMU sensor";

    m_strIdent  = ss.str();
    strIdent    = m_strIdent;
  }

  return rc;
}

int LaeImuCleanFlight::readRawImu()
{
  int   rc;

  if( isBlackListed() )
  {
    rc = LAE_OK; 
  }

  else if( (rc = mspReadRawImu()) == LAE_OK )
  {
    rc = mspReadAttitude();
  }

  return rc;
}

int LaeImuCleanFlight::readRawInertia()
{
  int   rc;

  if( isBlackListed() )
  {
    rc = LAE_OK; 
  }

  else
  {
    rc = mspReadRawImu();
  }

  return rc;
}

int LaeImuCleanFlight::readRawRollPitchYaw()
{
  int   rc;

  if( isBlackListed() )
  {
    rc = LAE_OK; 
  }

  else
  {
    rc = mspReadAttitude();
  }

  return rc;
}

int LaeImuCleanFlight::convertRawToSI()
{
  for(int i = 0; i < NumOfAxes; ++i)
  {
    m_accel[i] = m_accelRaw[i] * MspMpu6050RawToG * MspGToMPerSec2;
    RtDb.m_imu.m_accel[i] = m_accel[i];

    m_gyro[i]  = degToRad(m_gyroRaw[i] * MspMpu6050RawToDegPerSec);
    RtDb.m_imu.m_gyro[i] = m_gyro[i];

    if( i != YAW )
    {
      m_rpy[i]  = degToRad(m_rpyRaw[i] * MspAttitudeRawToDeg);
      RtDb.m_imu.m_rpy[i] = m_rpy[i];
    }
  }
  m_rpy[YAW] = degToRad(m_rpyRaw[YAW]);
  RtDb.m_imu.m_rpy[YAW] = m_rpy[YAW];

  return LAE_OK;
}

int LaeImuCleanFlight::mspReadIdent(MspIdent &ident)
{
  static const uint_t CmdId       = MspCmdIdIdent;
  static const size_t RspDataLen = 7;

  byte_t  rspData[RspDataLen];
  int     rc;

  if( !isOpen() )
  {
    return -LAE_ECODE_IO;
  }

  lockIo();

  if( (rc = sendCmd(CmdId, NULL, 0)) == LAE_OK )
  {
    rc = receiveRsp(CmdId, rspData, RspDataLen);
  }

  if( rc == LAE_OK )
  {
    ident.m_uFwVersion  = (uint_t)rspData[0];
    ident.m_uMultiType  = (uint_t)rspData[1];
    ident.m_uMspVersion = (uint_t)rspData[2];
    unpack32(&rspData[3], ident.m_uCaps);
  }

  unlockIo();

  return rc;
}

int LaeImuCleanFlight::mspReadRawImu()
{
  static const uint_t CmdId       = MspCmdIdRawImu;
  static const size_t RspDataLen  = 18;

  byte_t  rspData[RspDataLen];
  int     i, n;
  int     rc;

  if( !isOpen() )
  {
    return -LAE_ECODE_IO;
  }

  lockIo();

  if( (rc = sendCmd(CmdId, NULL, 0)) == LAE_OK )
  {
    rc = receiveRsp(CmdId, rspData, RspDataLen);
  }

  if( rc == LAE_OK )
  {
    for(i = 0, n = 0; i < NumOfAxes; ++i, n += 2)
    {
      unpack16(&rspData[n], m_accelRaw[i]);
    }
    for(i = 0; i < NumOfAxes; ++i, n += 2)
    {
      unpack16(&rspData[n], m_gyroRaw[i]);
    }
    for(i = 0; i < NumOfAxes; ++i, n += 2)
    {
      unpack16(&rspData[n], m_magRaw[i]);
    }
  }

  unlockIo();

  return rc;
}

int LaeImuCleanFlight::mspReadAttitude()
{
  static const uint_t CmdId       = MspCmdIdAttitude;
  static const size_t RspDataLen = 6;

  byte_t  rspData[RspDataLen];
  int     i, n;
  int     rc;

  if( !isOpen() )
  {
    return -LAE_ECODE_IO;
  }

  lockIo();

  if( (rc = sendCmd(CmdId, NULL, 0)) == LAE_OK )
  {
    rc = receiveRsp(CmdId, rspData, RspDataLen);
  }

  if( rc == LAE_OK )
  {
    for(i = 0, n = 0; i < NumOfAxes; ++i, n += 2)
    {
      unpack16(&rspData[n], m_rpyRaw[i]);
    }
  }

  unlockIo();

  return rc;
}


int LaeImuCleanFlight::sendCmd(uint_t cmdId, byte_t cmdData[], size_t lenData)
{
  byte_t  cmd[MspCmdMaxLen];
  uint_t  chksum;
  size_t  n;

  for(n = 0; n<strlen(MspCmdPreamble); ++n)
  {
    cmd[n] = MspCmdPreamble[n];
  }

  chksum = 0;

  cmd[n++] = (byte_t)lenData;
  chksum  ^= (byte_t)lenData;

  cmd[n++] = (byte_t)cmdId;
  chksum  ^= (byte_t)cmdId;
  
  for(size_t i = 0; i < lenData; ++i)
  {
    cmd[n++] = (byte_t)cmdData[i];
    chksum  ^= (byte_t)cmdData[i];
  }

  cmd[n++] = (byte_t)chksum;

  if( SerDevWrite(m_fd, cmd, n, TCmdTimeout) == n )
  {
    return LAE_OK;
  }
  else
  {
    flush(TFlushDelay);
    LOGERROR("IMU: Cmd %d: Send failed.", cmdId);
    return -LAE_ECODE_IO;
  }
}

int LaeImuCleanFlight::receiveRsp(uint_t cmdId,
                                  byte_t rspData[],
                                  size_t lenData)
{
  byte_t  rsp[MspRspMaxLen];
  uint_t  chksum;
  size_t  lenRsp;
  size_t  n;
  size_t  fldSize;
  uint_t  fldCmdId;
  byte_t  fldChkSum;
  size_t  i;

  lenRsp = MspRspMinLen + lenData;

  if( (n = SerDevRead(m_fd, rsp, lenRsp, TRspTimeout)) < 0 )
  {
    LOGERROR("IMU: Cmd %d response: Receive failed.", cmdId);
    flush(TFlushDelay);
    return -LAE_ECODE_IO;
  }
  else if( n < lenRsp )
  {
    LOGERROR("IMU: Cmd %d response: Receive partial response: "
        "rcv'd %zu bytes, expected %zu bytes.", cmdId, n, lenRsp);
    flush(TFlushDelay);
    return -LAE_ECODE_IO;
  }

  fldSize = (size_t)rsp[MspFieldPosSize];

  if( fldSize != lenData )
  {
    LOGERROR("IMU: Cmd %d response: Data length mismatch: " \
          "Received %zu bytes, expected %zu bytes.", cmdId, fldSize, lenData);
    resyncComm();
    return -LAE_ECODE_IO;
  }

  fldCmdId = (uint_t)rsp[MspFieldPosCmdId];

  if( fldCmdId != cmdId )
  {
    LOGERROR("IMU: Cmd %d response: Command Id mismatch: Received %d.",
        cmdId, fldCmdId);
    resyncComm();
    return -LAE_ECODE_IO;
  }

  chksum  = rsp[MspFieldPosSize];
  chksum ^= rsp[MspFieldPosCmdId];

  for(i = 0, n = MspFieldPosDataStart; i < lenData; ++i, ++n)
  {
    chksum ^= rsp[n];
    rspData[i] = rsp[n];
  }

  fldChkSum = rsp[lenRsp-1];

  if( chksum != fldChkSum )
  {
    LOGERROR("IMU: Cmd %d response: Checksum mismatch: " \
          "Received 0x%02x, calculated 0x%02x.", cmdId, fldChkSum, chksum);
    resyncComm();
    return -LAE_ECODE_IO;
  }

  return LAE_OK;
}

void LaeImuCleanFlight::resyncComm()
{
  MspIdent  ident;
  int       nMaxTries = 3;
  int       nTries;
  int       rc;

  LOGDIAG3("IMU: Resynchronizing serial communication.");

  for(nTries = 0; nTries < nMaxTries; ++nTries)
  {
    flush(TFlushDelay);
    if( (rc = mspReadIdent(ident)) == LAE_OK )
    {
      return;
    }
  }

  LOGWARN("IMU: Failed to resynchronize communication in %d tries.", nMaxTries);
  LOGWARN("IMU: Blacklisting.", nMaxTries);

  blacklist();
}

void LaeImuCleanFlight::flush(uint_t t)
{
  if( t > 0 )
  {
    usleep(t);
  }

  SerDevFIFOOutputFlush(m_fd);
  SerDevFIFOInputFlush(m_fd);
}

int LaeImuCleanFlight::pack16(uint_t val, byte_t buf[])
{
  buf[0] = (byte_t)(val & 0xff);
  buf[1] = (byte_t)((val >> 8) & 0xff);
  return 2;
}

int LaeImuCleanFlight::unpack16(byte_t buf[], uint_t &val)
{
  val = ((uint_t)(buf[1]) << 8) | (uint_t)buf[0];
  return 2;
}

int LaeImuCleanFlight::unpack16(byte_t buf[], int &val)
{
  s16_t v;
  v = ((s16_t)(buf[1]) << 8) | (s16_t)buf[0];
  val = (int)v;
  return 2;
}

int LaeImuCleanFlight::pack32(uint_t val, byte_t buf[])
{
  buf[0] = (byte_t)(val & 0xff);
  buf[1] = (byte_t)((val >>  8) & 0xff);
  buf[2] = (byte_t)((val >> 16) & 0xff);
  buf[3] = (byte_t)((val >> 24) & 0xff);
  return 4;
}

int LaeImuCleanFlight::unpack32(byte_t buf[], uint_t &val)
{
  val = ((uint_t)(buf[3]) << 24) |
        ((uint_t)(buf[2]) << 16) |
        ((uint_t)(buf[1]) <<  8) |
        (uint_t)buf[0];
  return 4;
}

int LaeImuCleanFlight::unpack32(byte_t buf[], int &val)
{
  s32_t v;

  v = ((s32_t)(buf[3]) << 24) |
      ((s32_t)(buf[2]) << 16) |
      ((s32_t)(buf[1]) <<  8) |
      (s32_t)buf[0];
  val = (int)v;
  return 4;
}
