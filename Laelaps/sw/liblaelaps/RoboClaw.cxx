////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelasp
//
// Library:   liblaelaps
//
// File:      RoboClaw.cxx
//
/*! \file
 *
 * $LastChangedDate: 2016-04-13 11:47:47 -0600 (Wed, 13 Apr 2016) $
 * $Rev: 4386 $
 *
 * \brief RoboClaw motor controller class implementation.
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

#include <sys/types.h>
#include <unistd.h>
#include <stdio.h>

#include <string>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/serdev.h"

#include "Laelaps/RoboClaw.h"

using namespace std;
using namespace motor::roboclaw;

//-----------------------------------------------------------------------------
// Private 
//-----------------------------------------------------------------------------

/*!
 * \brief Test for valid motor index.
 *
 * This macro only within RoboClaw methods.
 *
 * \param motor Motor index.
 *
 * \return On failure, forces return from calling function with the appropriate
 * error code.
 */
#define ROBOCLAW_TRY_MOTOR(motor) \
  do \
  { \
    if( !isValidMotor(motor) ) \
    { \
      LOGERROR("Motor controller 0x%02x: Motor index %d invalid.", \
          m_address, motor); \
      return RC_ERROR; \
    } \
  } while(0)

#undef ROBOCLAW_DBG_ENABLE  ///< debug switch
#ifdef ROBOCLAW_DBG_ENABLE

/*!
 * \brief Debug print.
 *
 * \param addr    Motor controller address
 * \param cmdNum  Command number.
 * \param fmt     Format string.
 * \param ...     Optional variable arguments for fmt.
 */
#define ROBOCLAW_DBG(addr, cmdNum, fmt, ...) \
    fprintf(stderr, "DBG: %s[%d]: Motor controller=0x%02x: Cmd=%d: " fmt, \
        __FILE__, __LINE__, addr, cmdNum, ##__VA_ARGS__)

/*!
 * \brief Debug print message.
 *
 * \param addr    Motor controller address
 * \param cmdNum  Command number.
 * \param buf     Buffer to print.
 * \param len     Buffer length.
 * \param fmt     Format string.
 * \param ...     Optional variable arguments for fmt.
 */
#define ROBOCLAW_DBG_MSG(addr, cmdNum, buf, len, fmt, ...) \
  do \
  { \
    fprintf(stderr, "DBG: %s[%d]: Motor controller=0x%02x: Cmd=%d " fmt, \
        __FILE__, __LINE__, addr, cmdNum, ##__VA_ARGS__); \
    for(int i=0;i<len; ++i) { fprintf(stderr, "0x%02x ", buf[i]); } \
    fprintf(stderr, "\n"); \
  } while(0)

/*!
 * \brief Debug print buffer.
 *
 * \param buf     Buffer to print.
 * \param len     Buffer length.
 * \param fmt     Format string.
 * \param ...     Optional variable arguments for fmt.
 */
#define ROBOCLAW_DBG_BUF(buf, len, fmt, ...) \
  do \
  { \
    fprintf(stderr, "DBG: %s[%d]: " fmt, __FILE__, __LINE__, ##__VA_ARGS__); \
    for(int i=0;i<len; ++i) { fprintf(stderr, "0x%02x ", buf[i]); } \
    fprintf(stderr, "\n"); \
  } while(0)

#else   // Disabled

/*!
 * \brief Debug print disabled.
 */
#define ROBOCLAW_DBG(addr, cmdNum, fmt, ...)

/*!
 * \brief Debug print message disabled.
 */
#define ROBOCLAW_DBG_MSG(addr, cmdNum, buf, len, fmt, ...)

/*!
 * \brief Debug print buffer disabled.
 */
#define ROBOCLAW_DBG_BUF(buf, len, fmt, ...) \

#endif // ROBOCLAW_DBG_ENABLE

/*!
 * \brief Integer absolute value.
 *
 * \param a   Integer value.
 *
 * \return |a|
 */
inline s64_t iabs(s64_t a)
{
  return a>=0? a: -a;
}


//-----------------------------------------------------------------------------
// Class RoboClawChipSelect
//-----------------------------------------------------------------------------

RoboClawChipSelect::RoboClawChipSelect()
{
  m_addrLast = 0;
}

RoboClawChipSelect::~RoboClawChipSelect()
{
  m_addrLast = 0;
}

void RoboClawChipSelect::select(int fd, byte_t addrSel)
{
  m_addrLast = addrSel;
}

/*! Fixed no-op selection object. */
static RoboClawChipSelect csNoOp;


//-----------------------------------------------------------------------------
// Class RoboClawComm
//-----------------------------------------------------------------------------

RoboClawComm::RoboClawComm()
{
  m_nBaudRate   = 0;
  m_fd          = -1;
  m_pChipSelect = &csNoOp;
  m_bDbgEnable  = false;
}

RoboClawComm::RoboClawComm(std::string        &strDevName,
                           int                nBaudRate,
                           RoboClawChipSelect *pChipSelect)
{
  m_nBaudRate   = 0;
  m_fd          = -1;
  m_pChipSelect = &csNoOp;
  m_bDbgEnable  = false;

  open(strDevName, nBaudRate, pChipSelect);
}

RoboClawComm::~RoboClawComm()
{
  close();
}

int RoboClawComm::open(std::string        &strDevName,
                       int                 nBaudRate,
                       RoboClawChipSelect *pChipSelect)
{
  m_fd = SerDevOpen(strDevName.c_str(), nBaudRate, 8, 'N', 1, false, false);

  if( m_fd < 0 )
  {
    LOGERROR("Failed to open %s@%d.", strDevName.c_str(), nBaudRate);
    return RC_ERROR;
  }

  flushInput();

  m_strDevName  = strDevName;
  m_nBaudRate   = nBaudRate;
  m_pChipSelect = pChipSelect == NULL? &csNoOp: pChipSelect;

  LOGDIAG3("Opened interface to RoboClaw motor controller %s@%d.",
      strDevName.c_str(), nBaudRate);

  return OK;
}

int RoboClawComm::close()
{
  if( m_fd >= 0 )
  {
    SerDevClose(m_fd);
    LOGDIAG3("Closed %s interface to RoboClaw motor controller.", 
        m_strDevName.c_str());
  }

  m_strDevName.clear();
  m_nBaudRate   = 0;
  m_fd          = -1;
  m_pChipSelect = &csNoOp;
  m_bDbgEnable  = false;

  return OK;
}

int RoboClawComm::flushInput()
{
  SerDevFIFOInputFlush(m_fd);
}

int RoboClawComm::execCmd(byte_t cmd[], size_t lenCmd)
{
  int   rc;

  LOGWARN("Motor controller 0x%02x: execCmd() function is deprecated.");

  if( (rc = sendCmd(cmd, lenCmd)) != OK )
  {
    LOGERROR("Motor controller 0x%02x: Failed to send command %u.",
        cmd[FieldPosAddr], cmd[FieldPosCmd]);
    flushInput();
  }

  return rc;
}

int RoboClawComm::execCmdWithAckRsp(byte_t cmd[], size_t lenCmd, MsgFmt fmtCmd)
{
  int   rc;

  if( (rc = sendCmd(cmd, lenCmd, fmtCmd)) == OK )
  {
    if( (rc = recvAck()) != OK )
    {
      LOGERROR("Motor controller 0x%02x: Failed to receive ack for command %u.",
        cmd[FieldPosAddr], cmd[FieldPosCmd]);
      rc = RC_ERROR;
    }
  }
  else
  {
    LOGERROR("Motor controller 0x%02x: Failed to send command %u.",
        cmd[FieldPosAddr], cmd[FieldPosCmd]);
    rc = RC_ERROR;
  }

  if( rc == RC_ERROR )
  {
    flushInput();
  }

  return rc;
}

int RoboClawComm::execCmdWithDataRsp(byte_t cmd[],  size_t lenCmd,
                                     byte_t rsp[],  size_t lenRsp,
                                     MsgFmt fmtCmd, MsgFmt fmtRsp)
{
  uint_t  uCrcCalc;
  int     n;
  int     rc;

  if( (rc = sendCmd(cmd, lenCmd, fmtCmd)) == OK )
  {
    if( (n = recvDataRsp(rsp, lenRsp)) == (int)lenRsp )
    {
      if( fmtRsp == MsgWithCrc )
      {
        uCrcCalc = crc16(m_uCrcCmd, rsp, lenRsp-2);
        if( m_uCrcRsp != uCrcCalc )
        {
          LOGERROR("Motor controller 0x%02x: "
            "Command %u response CRC-16 mismatch: "
            "Expected 0x%04x, received 0x%04x.",
            cmd[FieldPosAddr], cmd[FieldPosCmd],
            uCrcCalc, m_uCrcRsp);
          rc = RC_ERROR;
        }
      }
    }
    else if( n > 0 )
    {
      LOGWARN("Motor controller 0x%02x: "
          "Command %u received partial response of %d/%zu bytes - ignoring.",
            cmd[FieldPosAddr], cmd[FieldPosCmd], n, lenRsp);
      rc = RC_ERROR;
    }
    else
    {
      LOGERROR("Motor controller 0x%02x: Command %u received no response.",
            cmd[FieldPosAddr], cmd[FieldPosCmd]);
      rc = RC_ERROR;
    }
  }
  else
  {
    LOGERROR("Motor controller 0x%02x: Failed to send command %u.",
            cmd[FieldPosAddr], cmd[FieldPosCmd]);
    rc = RC_ERROR;
  }

  if( rc == RC_ERROR )
  {
    flushInput();
  }

  return rc;
}

int RoboClawComm::sendCmd(byte_t cmd[], size_t lenCmd, MsgFmt fmtCmd)
{
  m_pChipSelect->select(m_fd, cmd[FieldPosAddr]);

  // command's CRC
  m_uCrcCmd = crc16(0, cmd, lenCmd);

  // append CRC
  if( fmtCmd == MsgWithCrc )
  {
    pack16(m_uCrcCmd, &cmd[lenCmd]);
    lenCmd += 2;
  }

  if( m_bDbgEnable )
  {
    ROBOCLAW_DBG_BUF(cmd, lenCmd,
        "Motor controller=0x%02x: Cmd=%d: CRC=0x%04x: sendCmd(): ",
        cmd[FieldPosAddr], cmd[FieldPosCmd], m_uCrcCmd);
  }

  if( SerDevWrite(m_fd, cmd, lenCmd, CmdTimeout) == lenCmd )
  {
    return OK;
  }
  else
  {
    return RC_ERROR;
  }
}

int RoboClawComm::recvDataRsp(byte_t rsp[], size_t lenRsp)
{
  ssize_t   n;

  if( (n = SerDevRead(m_fd, rsp, lenRsp, RspTimeout)) >= 0 )
  {
    // last two bytes are the 16-bit crc
    if( n >= 2 )
    {
      unpack16(&rsp[n-2], m_uCrcRsp);
    }

    if( m_bDbgEnable )
    {
      ROBOCLAW_DBG_BUF(rsp, n, "recvDataRsp(): ");
    }

    return (int)n;
  }
  else
  {
    return RC_ERROR;
  }
}

int RoboClawComm::recvAck()
{
  ssize_t n;
  byte_t  ack;

  n = SerDevRead(m_fd, &ack, 1, RspTimeout);

  if( n != 1 )
  {
    return RC_ERROR;
  }

  if( m_bDbgEnable )
  {
    ROBOCLAW_DBG_BUF((&ack), n, "recvAck(): ");
  }

  if( ack != RspAck )
  {
    return RC_ERROR;
  }
  else
  {
    return OK;
  }
}

int RoboClawComm::pack16(uint_t val, byte_t buf[])
{
  buf[0] = (byte_t)((val >> 8) & 0xff);
  buf[1] = (byte_t)(val & 0xff);
  return 2;
}

int RoboClawComm::unpack16(byte_t buf[], uint_t &val)
{
  val = ((uint_t)(buf[0]) << 8) | (uint_t)buf[1];
  return 2;
}

int RoboClawComm::unpack16(byte_t buf[], int &val)
{
  s16_t v;

  v = ((s16_t)(buf[0]) << 8) | (s16_t)buf[1];
  val = (int)v;

  return 2;
}

int RoboClawComm::pack32(uint_t val, byte_t buf[])
{
  buf[0] = (byte_t)((val >> 24) & 0xff);
  buf[1] = (byte_t)((val >> 16) & 0xff);
  buf[2] = (byte_t)((val >>  8) & 0xff);
  buf[3] = (byte_t)(val & 0xff);
  return 4;
}

int RoboClawComm::unpack32(byte_t buf[], uint_t &val)
{
  val = ((uint_t)(buf[0]) << 24) |
        ((uint_t)(buf[1]) << 16) |
        ((uint_t)(buf[2]) <<  8) |
        (uint_t)buf[3];
  return 4;
}

int RoboClawComm::unpack32(byte_t buf[], int &val)
{
  s32_t v;

  v = ((s32_t)(buf[0]) << 24) |
      ((s32_t)(buf[1]) << 16) |
      ((s32_t)(buf[2]) <<  8) |
      (s32_t)buf[3];
  val = (int)v;

  return 4;
}

byte_t RoboClawComm::checksum(byte_t buf[], size_t lenBuf, bool bAck)
{
  uint_t  sum;
  byte_t  chksum;
  size_t  i;

  for(i=0, sum=0; i<lenBuf; ++i)
  {
    sum += (uint_t)buf[i];
  }

  chksum = (byte_t)(sum) & CheckSumMask;
 
  if( bAck )
  {
    chksum |= AckReqBit;
  }

  return chksum;
}

uint_t RoboClawComm::crc16(uint_t crc, byte_t buf[], size_t lenBuf)
{
  size_t  i;
  size_t  bit;

  for(i = 0; i < lenBuf; ++i)
  {
    crc = crc ^ ((uint_t)buf[i] << 8);

    for(bit = 0; bit < 8; ++bit)
    {
      if( crc & 0x8000 )
      {
        crc = (crc << 1) ^ 0x1021;
      }
      else
      {
        crc = crc << 1;
      }
    }
  }
  return crc & 0xffff;
}


//-----------------------------------------------------------------------------
// Class RoboClaw
//-----------------------------------------------------------------------------

RoboClaw::RoboClaw(RoboClawComm &comm,
                   const byte_t address,
                   const string &strNameId) :
    m_comm(comm), m_address(address), m_strNameId(strNameId)
{
  for(int i=0; i<NumMotors; ++i)
  {
    m_nMotorDir[i]  = MotorDirNormal;
    m_nEncPrev[i]   = 0;
    m_nEncoder[i]   = 0;
  }
}

RoboClaw::~RoboClaw()
{
  if( m_comm.isOpen() )
  {
    cmdStop();
  }
}

bool RoboClaw::probe(byte_t address)
{
  byte_t  cmd[CmdMaxBufLen];
  byte_t  rsp[RspMaxBufLen];
  size_t  n = 0;
  int     rc;

  cmd[n++] = address;
  cmd[n++] = CmdReadMainBattVolt; // probe with a safe read command

  rc = m_comm.execCmdWithDataRsp(cmd, n, rsp, 4);

  return rc == OK;
}

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Identity Command
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

// Command 21
int RoboClaw::cmdReadFwVersion(string &strFwVer)
{
  byte_t  cmd[CmdMaxBufLen];
  byte_t  rsp[RspMaxBufLen];
  uint_t  uCrcCmd;
  uint_t  uCrcRsp;
  uint_t  uCrcCalc;
  size_t  n = 0;

  cmd[n++] = m_address;
  cmd[n++] = CmdReadFwVersion;

  if( m_comm.sendCmd(cmd, n) != OK )
  {
    return RC_ERROR;
  }

  n = m_comm.recvDataRsp(rsp, ParamVerLen);

  if( n > ParamVerLenMin )
  {
    //
    // Validate CRC 
    //
    uCrcCmd  = m_comm.getLastCmdCrc();
    uCrcRsp  = m_comm.getLastRspCrc();
    uCrcCalc = m_comm.crc16(uCrcCmd, rsp, n-2);

    if( uCrcRsp != uCrcCalc )
    {
      LOGERROR("Motor controller 0x%02x: "
            "Command %u response CRC-16 mismatch: "
            "Expected 0x%04x, received 0x%04x.",
            cmd[FieldPosAddr], cmd[FieldPosCmd],
            uCrcCalc, uCrcRsp);
      return RC_ERROR;
    }

    rsp[n-ParamVerLenMin] = 0;  // take off trailing junk
    strFwVer = (char *)rsp;     // convert to string

    LOGDIAG3("%s.", strFwVer.c_str());

    return OK;
  }
  else
  {
    LOGERROR("Motor controller 0x%02x: "
        "Failed to receive firmware version.",
            m_address);
    return RC_ERROR;
  }
}

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Main Battery Voltage Commands
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

// Command 24
int RoboClaw::cmdReadMainBattVoltage(double &volts)
{
  byte_t  cmd[CmdMaxBufLen];
  byte_t  rsp[RspMaxBufLen];
  size_t  n = 0;
  uint_t  val;
  int     rc;

  cmd[n++] = m_address;
  cmd[n++] = CmdReadMainBattVolt;

  if( (rc = m_comm.execCmdWithDataRsp(cmd, n, rsp, 4)) == OK )
  {
    m_comm.unpack16(rsp, val);
    volts = (double)val * ParamVoltScale;
  }

  return rc;
}

void RoboClaw::getMainBattCutoffRange(double &minmin, double &maxmax) const
{
  minmin = ParamVoltMainMin;
  maxmax = ParamVoltMax;
}

// Command 59
int RoboClaw::cmdReadMainBattCutoffs(double &min, double &max)
{
  byte_t  cmd[CmdMaxBufLen];
  byte_t  rsp[RspMaxBufLen];
  size_t  n = 0;
  uint_t  valMin, valMax;
  int     rc;

  cmd[n++] = m_address;
  cmd[n++] = CmdReadMainBattCutoffs;

  if( (rc = m_comm.execCmdWithDataRsp(cmd, n, rsp, 6)) == OK )
  {
    n   = m_comm.unpack16(rsp, valMin);
    n  += m_comm.unpack16(rsp+n, valMax);

    ROBOCLAW_DBG(cmd[FieldPosAddr], cmd[FieldPosCmd],
        "MainBatt Cutoffs [%u, %u]\n", valMin, valMax);

    min = (double)valMin * ParamVoltScale;
    max = (double)valMax * ParamVoltScale;
  }

  return rc;
}

// Command 57
int RoboClaw::cmdSetMainBattCutoffs(const double min, const double max)
{
  byte_t  cmd[CmdMaxBufLen];
  size_t  k, n = 0;
  double  fMin, fMax;
  uint_t  valMin, valMax;
  int     rc;

  fMin    = min / ParamVoltScale;
  valMin  = (uint_t)fcap(fMin, ParamVoltMainMin, ParamVoltMax);

  fMax    = max / ParamVoltScale;
  valMax  = (uint_t)fcap(fMax, fMin, ParamVoltMax);

  //
  // header
  //
  cmd[n++] = m_address;
  cmd[n++] = CmdSetMainBattCutoffs;

  //
  // values
  //
  k = m_comm.pack16(valMin, cmd+n);
  n += k;

  k = m_comm.pack16(valMax, cmd+n);
  n += k;

  return m_comm.execCmdWithAckRsp(cmd, n, MsgWithCrc);
}

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Motor Current Commands
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

// Command 49
//
// Note: RoboClaw User Manual v5 error. The data are signed 16-bit integers
//
int RoboClaw::cmdReadMotorCurrentDraw(double &amps1, double &amps2)
{
  byte_t  cmd[CmdMaxBufLen];
  byte_t  rsp[RspMaxBufLen];
  size_t  k, n = 0;
  int     val1, val2;
  int     rc;

  cmd[n++] = m_address;
  cmd[n++] = CmdReadMotorDraw;

  if( (rc = m_comm.execCmdWithDataRsp(cmd, n, rsp, 6)) == OK )
  {
    n = 0;

    k = m_comm.unpack16(rsp, val1);
    n += k;

    k  = m_comm.unpack16(rsp+n, val2);
    n += k;

    amps1 = (double)val1 * ParamAmpScale;
    amps2 = (double)val2 * ParamAmpScale;
  }

  return rc;
}

// Command 135, 136
//
// Note: RoboClaw User Manual v5 error. Each value is 32 bits.
//
int RoboClaw::cmdReadMotorMaxCurrentLimit(int motor, double &maxAmps)
{
  byte_t  cmd[CmdMaxBufLen];
  byte_t  rsp[RspMaxBufLen];
  size_t  k, n = 0;
  uint_t  val1, val2;
  double  minAmps;      // always zero - so ignore
  int     rc;

  cmd[n++] = m_address;
  cmd[n++] = motor == Motor1? CmdReadMaxCurrentMot1: CmdReadMaxCurrentMot2;

  m_comm.enableDbg(true);

  rc = m_comm.execCmdWithDataRsp(cmd, n, rsp, 10);

  if( rc == OK )
  {
    n = 0;

    k = m_comm.unpack32(rsp, val1);
    n += k;

    k  = m_comm.unpack32(rsp+n, val2);
    n += k;

    maxAmps = (double)val1 * ParamAmpScale;
    minAmps = (double)val2 * ParamAmpScale;
  }

  m_comm.enableDbg(false);

  return rc;
}

// Command 133, 134
//
// Note: RoboClaw User Manual v5 error. Each value is 32 bits.
//
int RoboClaw::cmdSetMotorMaxCurrentLimit(int motor, const double maxAmps)
{
  byte_t  cmd[CmdMaxBufLen];
  byte_t  rsp[RspMaxBufLen];
  size_t  k, n = 0;
  double  fMax;
  uint_t  valMin;   // always zero
  uint_t  valMax;
  int     rc;

  // convert interface values 
  // RDK fMax    = fcap(maxAmps, ParamAmpMinSane, ParamAmpMax);
  fMax    = fcap(maxAmps, ParamAmpMinSane, 100.0); // RDK
  valMax  = (uint_t)(fMax / ParamAmpScale);

  valMin  = 0;

  //
  // pack header
  //
  cmd[n++] = m_address;
  cmd[n++] = motor == Motor1? CmdSetMaxCurrentMot1: CmdSetMaxCurrentMot2;

  //
  // pack values
  //
  k = m_comm.pack32(valMax, cmd+n);
  n += k;

  k = m_comm.pack32(valMin, cmd+n);
  n += k;

  // execute
  return m_comm.execCmdWithAckRsp(cmd, n, MsgWithCrc);
}

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Logic Battery Voltage Commands
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

// Command 25
int RoboClaw::cmdReadLogicVoltage(double &volts)
{
  byte_t  cmd[CmdMaxBufLen];
  byte_t  rsp[RspMaxBufLen];
  size_t  n = 0;
  uint_t  val;
  int     rc;

  cmd[n++] = m_address;
  cmd[n++] = CmdReadLogicVolt;

  if( (rc = m_comm.execCmdWithDataRsp(cmd, n, rsp, 4)) == OK )
  {
    m_comm.unpack16(rsp, val);
    volts = (double)val * ParamVoltScale;
  }

  return rc;
}

void RoboClaw::getLogicCutoffRange(double &minmin, double &maxmax) const
{
  minmin = ParamVoltLogicMin;
  maxmax = ParamVoltMax;
}

// Command 60
int RoboClaw::cmdReadLogicCutoffs(double &min, double &max)
{
  byte_t  cmd[CmdMaxBufLen];
  byte_t  rsp[RspMaxBufLen];
  size_t  n = 0;
  uint_t  valMin, valMax;
  int     rc;

  cmd[n++] = m_address;
  cmd[n++] = CmdReadLogicCutoffs;

  if( (rc = m_comm.execCmdWithDataRsp(cmd, n, rsp, 6)) == OK )
  {
    n   = m_comm.unpack16(rsp, valMin);
    n  += m_comm.unpack16(rsp+n, valMax);

    ROBOCLAW_DBG(cmd[FieldPosAddr], cmd[FieldPosCmd],
        "Logic Cutoffs [%u, %u]\n", valMin, valMax);

    min = (double)valMin * ParamVoltScale;
    max = (double)valMax * ParamVoltScale;
  }

  return rc;
}

// Command 58
int RoboClaw::cmdSetLogicCutoffs(const double min, const double max)
{
  byte_t  cmd[CmdMaxBufLen];
  size_t  k, n = 0;
  double  fMin, fMax;
  uint_t  valMin, valMax;
  int     rc;

  fMin    = min / ParamVoltScale;
  valMin  = (uint_t)fcap(fMin, ParamVoltLogicMin, ParamVoltMax);

  fMax    = max / ParamVoltScale;
  valMax  = (uint_t)fcap(fMax, fMin, ParamVoltMax);

  cmd[n++] = m_address;
  cmd[n++] = CmdSetLogicCutoffs;

  k  = m_comm.pack16(valMin, cmd+n);
  n += k;

  k  = m_comm.pack16(valMax, cmd+n);
  n += k;

  return m_comm.execCmdWithAckRsp(cmd, n, MsgWithCrc);
}

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Motor PID Commands
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

// Command 55, 56
int RoboClaw::cmdReadVelocityPidConst(int   motor,
                                      u32_t &Kp,
                                      u32_t &Ki,
                                      u32_t &Kd,
                                      u32_t &qpps)
{
  byte_t  cmd[CmdMaxBufLen];
  byte_t  rsp[RspMaxBufLen];
  size_t  k, n = 0;
  int     rc;

  cmd[n++] = m_address;
  cmd[n++] = motor == Motor1? CmdReadVelPidMot1: CmdReadVelPidMot2;

  if( (rc = m_comm.execCmdWithDataRsp(cmd, n, rsp, 18)) == OK )
  {
    n = 0;

    k = m_comm.unpack32(rsp,   Kp);
    n += k;

    k = m_comm.unpack32(rsp+n, Ki);
    n += k;

    k = m_comm.unpack32(rsp+n, Kd);
    n += k;

    k = m_comm.unpack32(rsp+n, qpps);
    n += k;

    ROBOCLAW_DBG(cmd[FieldPosAddr], cmd[FieldPosCmd],
        "VelPid: motor=%d, P=0x%08x, I=0x%08x, D=0x%08x, Q=%u\n",
        motor, Kp, Ki, Kd, qpps);
  }

  return rc;
}

// Command 28,29
int RoboClaw::cmdSetVelocityPidConst(int         motor,
                                     const u32_t Kp,
                                     const u32_t Ki,
                                     const u32_t Kd,
                                     const u32_t qpps)
{
  byte_t  cmd[CmdMaxBufLen];
  byte_t  rsp[RspMaxBufLen];
  size_t  k, n = 0;
  int     rc;

  ROBOCLAW_TRY_MOTOR(motor);

  cmd[n++] = m_address;
  cmd[n++] = motor == Motor1? CmdSetVelPidMot1: CmdSetVelPidMot2;

  // order is strangly D,P,I,QPPS
  k  = m_comm.pack32(Kd, cmd+n);
  n += k;

  k  = m_comm.pack32(Kp, cmd+n);
  n += k;

  k  = m_comm.pack32(Ki, cmd+n);
  n += k;

  k  = m_comm.pack32(qpps, cmd+n);
  n += k;

  return m_comm.execCmdWithAckRsp(cmd, n, MsgWithCrc);
}

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Health and Status Commands
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

// Command 82
int RoboClaw::cmdReadBoardTemperature(double &temp)
{
  byte_t  cmd[CmdMaxBufLen];
  byte_t  rsp[RspMaxBufLen];
  size_t  n = 0;
  uint_t  val;
  int     rc;

  cmd[n++] = m_address;
  cmd[n++] = CmdReadTemp;

  if( (rc = m_comm.execCmdWithDataRsp(cmd, n, rsp, 4)) == OK )
  {
    m_comm.unpack16(rsp, val);
    temp = (double)val * ParamTempScale;
  }

  return rc;
}

// Command 90
int RoboClaw::cmdReadStatus(uint_t &status)
{
  byte_t  cmd[CmdMaxBufLen];
  byte_t  rsp[RspMaxBufLen];
  size_t  n = 0;
  uint_t  val;
  int     rc;

  cmd[n++] = m_address;
  cmd[n++] = CmdReadStatus;

  if( (rc = m_comm.execCmdWithDataRsp(cmd, n, rsp, 4)) == OK )
  {
    m_comm.unpack16(rsp, status);
  }

  return rc;
}

// Command 47
int RoboClaw::cmdReadCmdBufLengths(uint_t &len1, uint_t &len2)
{
  byte_t  cmd[CmdMaxBufLen];
  byte_t  rsp[RspMaxBufLen];
  size_t  n = 0;
  uint_t  val;
  int     rc;

  cmd[n++] = m_address;
  cmd[n++] = CmdReadBufLen;

  if( (rc = m_comm.execCmdWithDataRsp(cmd, n, rsp, 4)) == OK )
  {
    len1 = (uint_t)rsp[0];
    len2 = (uint_t)rsp[1];
  }

  return rc;
}

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Encoder Commands
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

// Command 91
int RoboClaw::cmdReadEncoderMode(byte_t &mode1, byte_t &mode2)
{
  byte_t  cmd[CmdMaxBufLen];
  byte_t  rsp[RspMaxBufLen];
  size_t  n = 0;
  uint_t  val;
  int     rc;

  cmd[n++] = m_address;
  cmd[n++] = CmdReadEncoderMode;

  if( (rc = m_comm.execCmdWithDataRsp(cmd, n, rsp, 4)) == OK )
  {
    mode1 = rsp[0];
    mode2 = rsp[1];
    //LOGDIAG3("Motor controller 0x%02x: mode1=0x%02x, mode2=0x%02x.",
    //    m_address, mode1, mode2);
  }

  return rc;
}

// Command 92, 93
int RoboClaw::cmdSetEncoderMode(int motor, byte_t mode)
{
  byte_t  cmd[CmdMaxBufLen];
  byte_t  mask = ParamEncModeRCAnalogBit | ParamEncModeQuadAbsBit;
  size_t  n = 0;

  ROBOCLAW_TRY_MOTOR(motor);

  cmd[n++] = m_address;
  cmd[n++] = motor == Motor1? CmdSetEncoderModeMot1: CmdSetEncoderModeMot2;
  cmd[n++] = (byte_t)(mask & mode);

  return m_comm.execCmdWithAckRsp(cmd, n, MsgWithCrc);
}

// Command 92, 93
int RoboClaw::cmdSetEncoderMode2(byte_t mode1, byte_t mode2)
{
  int rc;

  if( (rc = cmdSetEncoderMode(Motor1, mode1)) == OK )
  {
    rc = cmdSetEncoderMode(Motor2, mode2);
  }

  return rc;
}

// Command 20
int RoboClaw::cmdResetQEncoders()
{
  byte_t  cmd[CmdMaxBufLen];
  size_t  n = 0;
  int     rc;

  cmd[n++] = m_address;
  cmd[n++] = CmdResetEncoderCntrs;

  if( (rc = m_comm.execCmdWithAckRsp(cmd, n, MsgWithCrc)) == OK )
  {
    for(int i=0; i<NumMotors; ++i)
    {
      m_nEncPrev[i] = 0;
      m_nEncoder[i] = 0;
    }
  }

  return rc;
}

// Command 16, 17
int RoboClaw::cmdReadQEncoder(int motor, s64_t &encoder)
{
  static s64_t  MaxEncDelta = 2048; // maximum encoder delta on wrap

  byte_t  cmd[CmdMaxBufLen];
  byte_t  rsp[RspMaxBufLen];
  size_t  n = 0;
  uint_t  enc32;      // 32-bit encoder value (quad pulses).
  byte_t  status;     // status bits indicating sign, and over/under flow

  s64_t   enc64;      // 64-bit signed encoder value (quad pulses).
  s64_t   delta;      // encoder delta from previous value
  int     rc;

  ROBOCLAW_TRY_MOTOR(motor);

  cmd[n++] = m_address;
  cmd[n++] = motor == Motor1? CmdReadEncoderMot1: CmdReadEncoderMot2;

  if( (rc = m_comm.execCmdWithDataRsp(cmd, n, rsp, 7)) == OK )
  {
    ROBOCLAW_DBG_MSG(cmd[FieldPosAddr], cmd[FieldPosCmd], rsp, 7,
        "cmdReadQEncoder(motor=%d): rsp = ", motor);

    //
    // Get 32-bit unsigned encoder value plus its associated status.
    //
    // Note:  This 2-tuple should be atomically kept insync by the firmware but
    //        this is not always the case. During a sharp velocity change
    //        (e.g. stopping) the velocity pid will rightly try to back drive.
    //        The status byte will indicate the _desired_ direction, not the
    //        actual.
    //
    n = m_comm.unpack32(rsp, enc32);
    status = rsp[n];
      
    enc64 = (s64_t)enc32;

    //
    // Debug left_front encoder to reverse engineer and test encoder algorithm.
    //
    //if( m_address == 0x80 && motor == Motor1 && m_nEncPrev[motor] != enc64 )
    //{
    //  fprintf(stderr, "DBG %lld 0x%02x\n", enc64, status);
    //}

    //
    // Track underflow/overflow for now. Not too sure if any action or special
    // calculations are needed.
    //
    // Note: Condition is cleared after reading.
    //
    if( status & ParamEncStatusUnderFlow )
    {
      LOGDIAG2("Motor controller 0x%02x: Motor %d: Encoder underflow: "
                "Encoder32=%u.",
              m_address, motor, enc32);
    }
    if( status & ParamEncStatusOverFlow )
    {
      LOGDIAG2("Motor controller 0x%02x: Motor %d: Encoder overflow: "
                "Encoder32=%u",
              m_address, motor, enc32);
    }

    //
    // Moving backwards. There are two cases:
    //  Case 1: no wrap:  example: previous 1500, current: 1000
    //  Case 2: wrap:     example: previous 15,   current: 4294967200
    //
    if( status & ParamEncStatusDirBackward )
    {
      if( enc64 <= m_nEncPrev[motor] )  // no wrap
      {
        delta = enc64 - m_nEncPrev[motor];
      }
      else  // wrap
      {
        delta = enc64 - ParamEncQuadMax - m_nEncPrev[motor];
        if( iabs(delta) > MaxEncDelta ) // not really backwards
        {
          delta = enc64 - m_nEncPrev[motor];
        }
      }
    }

    //
    // Moving forwards. There are two cases:
    //  Case 1: no wrap:  example: previous 1000,       current: 1500
    //  Case 2: wrap:     example: previous 4294967200, current: 15
    //
    else
    {
      if( enc64 >= m_nEncPrev[motor] ) // no wrap
      {
        delta = enc64 - m_nEncPrev[motor];
      }
      else  // wrap
      {
        delta = enc64 + ParamEncQuadMax - m_nEncPrev[motor];
        if( iabs(delta) > MaxEncDelta ) // not really forwards
        {
          delta = enc64 - m_nEncPrev[motor];
        }
      }
    }

    //
    // Update accumulated value and save current as new previous.
    //
    m_nEncoder[motor] += (m_nMotorDir[motor] * delta);
    m_nEncPrev[motor] = enc64;
    encoder           = m_nEncoder[motor];
  }

  return rc;
}

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Duty Cycle Commands
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

// Command 34
int RoboClaw::cmdDutyDrive2(double duty1, double duty2)
{
  byte_t  cmd[CmdMaxBufLen];
  int     speed1, speed2;
  size_t  k, n = 0;

  speed1 = (int)((double)duty1 * ParamDutyCycleMax);
  speed2 = (int)((double)duty2 * ParamDutyCycleMax);

  cmd[n++] = m_address;
  cmd[n++] = CmdDriveDuty;

  k  = m_comm.pack16((int)speed1, cmd+n);
  n += k;

  k  = m_comm.pack16((int)speed2, cmd+n);
  n += k;

  return m_comm.execCmdWithAckRsp(cmd, n, MsgWithCrc);
}

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Speed and Position Commands
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

// Command 18, 19
int RoboClaw::cmdReadQSpeed(int motor, s32_t &speed)
{
  byte_t  cmd[CmdMaxBufLen];
  byte_t  rsp[RspMaxBufLen];
  size_t  n = 0;
  uint_t  val;
  byte_t  status;
  int     rc;

  ROBOCLAW_TRY_MOTOR(motor);

  cmd[n++] = m_address;
  cmd[n++] = motor == Motor1? CmdReadSpeedMot1: CmdReadSpeedMot2;

  if( (rc = m_comm.execCmdWithDataRsp(cmd, n, rsp, 7)) == OK )
  {
    ROBOCLAW_DBG_MSG(cmd[FieldPosAddr], cmd[FieldPosCmd], rsp, 7,
        "cmdReadQSpeed(motor=%d): rsp = ", motor);

    n = m_comm.unpack32(rsp, val);
    status = rsp[n];
    speed = (s32_t)val;

    if( status & ParamEncStatusDirBackward )
    {
      speed = -speed;
    }
  }

  return rc;
}

// Command 35, 36
int RoboClaw::cmdQDrive(int motor, s32_t speed)
{
  byte_t  cmd[CmdMaxBufLen];
  size_t  k, n = 0;

  ROBOCLAW_TRY_MOTOR(motor);

  speed *= m_nMotorDir[motor];

  cmd[n++] = m_address;
  cmd[n++] = motor == Motor1? CmdDriveQMot1: CmdDriveQMot2;

  k  = m_comm.pack32((int)speed, cmd+n);
  n += k;

  ROBOCLAW_DBG_MSG(cmd[FieldPosAddr], cmd[FieldPosCmd], cmd, n,
      "cmdQDrive(motor=%d, speed=%d): cmd = ", motor, speed);

  return m_comm.execCmdWithAckRsp(cmd, n, MsgWithCrc);
}

// Command 37
int RoboClaw::cmdQDrive2(s32_t speed1, s32_t speed2)
{
  byte_t  cmd[CmdMaxBufLen];
  size_t  k, n = 0;

  speed1 *= m_nMotorDir[Motor1];
  speed2 *= m_nMotorDir[Motor2];

  cmd[n++] = m_address;
  cmd[n++] = CmdDriveQ;

  k  = m_comm.pack32((int)speed1, cmd+n);
  n += k;

  k  = m_comm.pack32((int)speed2, cmd+n);
  n += k;

  return m_comm.execCmdWithAckRsp(cmd, n, MsgWithCrc);
}

// Command 38, 39
int RoboClaw::cmdQDriveWithAccel(int motor, s32_t speed, u32_t accel)
{
  byte_t  cmd[CmdMaxBufLen];
  size_t  k, n = 0;

  ROBOCLAW_TRY_MOTOR(motor);

  speed *= m_nMotorDir[motor];

  cmd[n++] = m_address;
  cmd[n++] = motor == Motor1? CmdDriveQAccelMot1: CmdDriveQAccelMot2;

  k  = m_comm.pack32((uint_t)accel, cmd+n);
  n += k;

  k  = m_comm.pack32((int)speed, cmd+n);
  n += k;

  return m_comm.execCmdWithAckRsp(cmd, n, MsgWithCrc);
}

// Command 50
int RoboClaw::cmdQDriveWithAccel(s32_t speed1, u32_t accel1,
                                 s32_t speed2, u32_t accel2)
{
  byte_t  cmd[CmdMaxBufLen];
  size_t  k, n = 0;

  speed1 *= m_nMotorDir[Motor1];
  speed2 *= m_nMotorDir[Motor2];

  cmd[n++] = m_address;
  cmd[n++] = CmdDriveQAccel2;

  k  = m_comm.pack32((uint_t)accel1, cmd+n);
  n += k;

  k  = m_comm.pack32((int)speed1, cmd+n);
  n += k;

  k  = m_comm.pack32((uint_t)accel2, cmd+n);
  n += k;

  k  = m_comm.pack32((int)speed2, cmd+n);
  n += k;

  return m_comm.execCmdWithAckRsp(cmd, n, MsgWithCrc);
}

// Command 41, 42
int RoboClaw::cmdQDriveForDist(int motor, s32_t speed, u32_t dist, bool bQueue)
{
  byte_t  cmd[CmdMaxBufLen];
  size_t  k, n = 0;

  ROBOCLAW_TRY_MOTOR(motor);

  speed *= m_nMotorDir[motor];

  cmd[n++] = m_address;
  cmd[n++] = motor == Motor1? CmdBufDriveQDistMot1: CmdBufDriveQDistMot2;

  k  = m_comm.pack32((int)speed, cmd+n);
  n += k;

  k  = m_comm.pack32((uint_t)dist, cmd+n);
  n += k;

  cmd[n++] = bQueue? ParamCmdBufQueue: ParamCmdBufPreempt;

  return m_comm.execCmdWithAckRsp(cmd, n, MsgWithCrc);
}

// Command 43
int RoboClaw::cmdQDriveForDist(s32_t speed1, u32_t dist1,
                               s32_t speed2, u32_t dist2,
                               bool  bQueue)
{
  byte_t  cmd[CmdMaxBufLen];
  size_t  k, n = 0;

  speed1 *= m_nMotorDir[Motor1];
  speed2 *= m_nMotorDir[Motor2];

  cmd[n++] = m_address;
  cmd[n++] = CmdBufDriveQDist;

  k  = m_comm.pack32((int)speed1, cmd+n);
  n += k;

  k  = m_comm.pack32((uint_t)dist1, cmd+n);
  n += k;

  k  = m_comm.pack32((int)speed2, cmd+n);
  n += k;

  k  = m_comm.pack32((uint_t)dist2, cmd+n);
  n += k;

  cmd[n++] = bQueue? ParamCmdBufQueue: ParamCmdBufPreempt;

  return m_comm.execCmdWithAckRsp(cmd, n, MsgWithCrc);
}

// Command 44, 45
int RoboClaw::cmdQDriveWithAccelForDist(int   motor,
                                        s32_t speed,
                                        u32_t accel,
                                        u32_t dist,
                                        bool  bQueue)
{
  byte_t  cmd[CmdMaxBufLen];
  size_t  k, n = 0;

  ROBOCLAW_TRY_MOTOR(motor);

  speed *= m_nMotorDir[motor];

  cmd[n++] = m_address;
  cmd[n++] = motor == Motor1? CmdBufDriveQAccelDistMot1:
                              CmdBufDriveQAccelDistMot2;

  k  = m_comm.pack32((uint_t)accel, cmd+n);
  n += k;

  k  = m_comm.pack32((int)speed, cmd+n);
  n += k;

  k  = m_comm.pack32((uint_t)dist, cmd+n);
  n += k;

  cmd[n++] = bQueue? ParamCmdBufQueue: ParamCmdBufPreempt;

  return m_comm.execCmdWithAckRsp(cmd, n, MsgWithCrc);
}

// Command 51
int RoboClaw::cmdQDriveWithAccelForDist(s32_t speed1,
                                        u32_t accel1,
                                        u32_t dist1,
                                        s32_t speed2,
                                        u32_t accel2,
                                        u32_t dist2,
                                        bool  bQueue)
{
  byte_t  cmd[CmdMaxBufLen];
  size_t  k, n = 0;

  speed1 *= m_nMotorDir[Motor1];
  speed2 *= m_nMotorDir[Motor2];

  cmd[n++] = m_address;
  cmd[n++] = CmdBufDriveQAccel2Dist;

  k  = m_comm.pack32((uint_t)accel1, cmd+n);
  n += k;

  k  = m_comm.pack32((int)speed1, cmd+n);
  n += k;

  k  = m_comm.pack32((uint_t)dist1, cmd+n);
  n += k;

  k  = m_comm.pack32((uint_t)accel2, cmd+n);
  n += k;

  k  = m_comm.pack32((int)speed2, cmd+n);
  n += k;

  k  = m_comm.pack32((uint_t)dist2, cmd+n);
  n += k;

  cmd[n++] = bQueue? ParamCmdBufQueue: ParamCmdBufPreempt;

  return m_comm.execCmdWithAckRsp(cmd, n, MsgWithCrc);
}

// Command 65, 66
int RoboClaw::cmdQDriveWithProfileToPos(int   motor,
                                        s32_t speed,
                                        u32_t accel,
                                        u32_t deccel,
                                        s32_t pos,
                                        bool  bQueue)
{
  byte_t  cmd[CmdMaxBufLen];
  size_t  k, n = 0;

  ROBOCLAW_TRY_MOTOR(motor);

  speed *= m_nMotorDir[motor];

  cmd[n++] = m_address;
  cmd[n++] = motor == Motor1? CmdBufDriveQProfPosMot1:
                              CmdBufDriveQProfPosMot2;

  k  = m_comm.pack32((uint_t)accel, cmd+n);
  n += k;

  k  = m_comm.pack32((int)speed, cmd+n);
  n += k;

  k  = m_comm.pack32((uint_t)deccel, cmd+n);
  n += k;

  k  = m_comm.pack32((int)pos, cmd+n);
  n += k;

  cmd[n++] = bQueue? ParamCmdBufQueue: ParamCmdBufPreempt;

  return m_comm.execCmdWithAckRsp(cmd, n, MsgWithCrc);
}

// Command 67
int RoboClaw::cmdQDriveWithProfileToPos(s32_t speed1,
                                        u32_t accel1,
                                        u32_t deccel1,
                                        s32_t pos1,
                                        s32_t speed2,
                                        u32_t accel2,
                                        u32_t deccel2,
                                        s32_t pos2,
                                        bool  bQueue)
{
  byte_t  cmd[CmdMaxBufLen];
  size_t  k, n = 0;

  speed1 *= m_nMotorDir[Motor1];
  speed2 *= m_nMotorDir[Motor2];

  cmd[n++] = m_address;
  cmd[n++] = CmdBufDriveQProfPos;

  k  = m_comm.pack32((uint_t)accel1, cmd+n);
  n += k;

  k  = m_comm.pack32((int)speed1, cmd+n);
  n += k;

  k  = m_comm.pack32((uint_t)deccel1, cmd+n);
  n += k;

  k  = m_comm.pack32((int)pos1, cmd+n);
  n += k;

  k  = m_comm.pack32((uint_t)accel2, cmd+n);
  n += k;

  k  = m_comm.pack32((int)speed2, cmd+n);
  n += k;

  k  = m_comm.pack32((uint_t)deccel2, cmd+n);
  n += k;

  k  = m_comm.pack32((int)pos2, cmd+n);
  n += k;

  cmd[n++] = bQueue? ParamCmdBufQueue: ParamCmdBufPreempt;

  return m_comm.execCmdWithAckRsp(cmd, n, MsgWithCrc);
}

// Command 0, 1, 4, 5
int RoboClaw::cmdSDrive(int motor, int speed)
{
  byte_t  cmd[CmdMaxBufLen];
  size_t  n = 0;

  ROBOCLAW_TRY_MOTOR(motor);

  speed  = RoboClaw::cap(speed, MotorSpeedMaxBackward, MotorSpeedMaxForward);
  speed *= m_nMotorDir[motor];

  if( speed >= 0 )
  {
    cmd[n++] = m_address;
    cmd[n++] = motor == Motor1? CmdDriveForwardMot1: CmdDriveForwardMot2;
    cmd[n++] = (byte_t)speed;
  }
  else
  {
    cmd[n++] = m_address;
    cmd[n++] = motor == Motor1? CmdDriveBackwardMot1: CmdDriveBackwardMot2;
    cmd[n++] = (byte_t)(-speed);
  }

  return m_comm.execCmdWithAckRsp(cmd, n, MsgWithCrc);
}

// Command 0, 1, 4, 5
int RoboClaw::cmdSDrive2(int speed1, int speed2)
{
  int rc;

  if( (rc = cmdSDrive(Motor1, speed1)) == OK )
  {
    rc = cmdSDrive(Motor2, speed2);
  }
  return rc;
}

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Stop Commands
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

// Command 35, 36
int RoboClaw::cmdStop(int motor)
{
  return cmdQDrive(motor, 0);
}

// Command 38, 39
int RoboClaw::cmdStopWithDecel(int motor, u32_t decel)
{
  return cmdQDriveWithAccel(motor, 0, decel);
}

// Command 37
int RoboClaw::cmdStop()
{
  return cmdQDrive2(0, 0);
}

// Command 50
int RoboClaw::cmdStopWithDecel(u32_t decel)
{
  return cmdQDriveWithAccel(0, decel, 0, decel);
}

// Command 37
int RoboClaw::cmdEStop()
{
  return cmdStop();
}

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Misc Commands
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

//
// Command 94
//
// Note: RoboClaw User Manual v5 errors.
//   The command appends a 4-btye magic number
//   The command includes a CRC checksum.
//
int RoboClaw::cmdWriteSettingsToEEPROM()
{
  byte_t  cmd[CmdMaxBufLen];
  size_t  n = 0;
  int     rc;

  cmd[n++] = m_address;
  cmd[n++] = CmdWriteEEPROM;

  // magic number
  cmd[n++] = 0xe2;
  cmd[n++] = 0x2e;
  cmd[n++] = 0xab;
  cmd[n++] = 0x7a;

  rc = m_comm.execCmdWithAckRsp(cmd, n, MsgWithCrc);

  // give a little time for the controller to write to its NVM
  if( rc == OK )
  {
    usleep(100000);   // 0.1 second to give controller time to write to nvm
  }

  return rc;
}
