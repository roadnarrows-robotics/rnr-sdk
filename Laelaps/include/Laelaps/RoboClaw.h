////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelasp
//
// Library:   liblaelaps
//
// File:      RoboClaw.h
//
/*! \file
 *
 * $LastChangedDate: 2016-02-19 11:48:14 -0700 (Fri, 19 Feb 2016) $
 * $Rev: 4324 $
 *
 * \brief RoboClaw motor controller class interface.
 *
 * RoboClaw is from Ion Motion Control.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2015-2016  RoadNarrows
 * (http://www.RoadNarrows.com)
 * \n All Rights Reserved
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

#ifndef _ROBO_CLAW_H
#define _ROBO_CLAW_H

#include <sys/types.h>

#ifndef SWIG
#include <string>
#endif // SWIG

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#ifndef SWIG
namespace motor
{
  namespace roboclaw
  {
#endif // SWIG
    // -------------------------------------------------------------------------
    // RoboClaw Data
    // -------------------------------------------------------------------------

    // ........................................................................
    // Serial Message Interface
    // ........................................................................
  
    //
    // Serial baud rates
    //
    static const int SerBaud2400    =    2400;  ///< 2400 serial baudrate 
    static const int SerBaud9600    =    9600;  ///< 9600 serial baudrate 
    static const int SerBaud19200   =   19200;  ///< 19200 serial baudrate 
    static const int SerBaud38400   =   38400;  ///< 38400 serial baudrate 
    static const int SerBaud115200  =  115200;  ///< 115200 serial baudrate 
    static const int SerBaud230400  =  230400;  ///< 230400 serial baudrate 
    static const int SerBaud460800  =  460800;  ///< 460800 serial baudrate 
    static const int USBBaud1Mbps   = 1000000;  ///< 1Mbps USB typical baudrate
  
    //
    // Message addresses.
    //
    static const byte_t AddrMin = 0x80; ///< minimum controller address
    static const byte_t AddrMax = 0x87; ///< maximum controller address
    static const byte_t AddrDft = 0x80; ///< default controller address
  
    //
    // Message Checksum (obsolete)
    //
    static const byte_t CheckSumMask = 0x7f; ///< checksum mask
    static const byte_t AckReqBit    = 0x80; ///< request ack to write commands
  
    /*!
     * \brief Command and response message formats.
     */
    enum MsgFmt
    {
      MsgWithNoCrc,   ///< message does not include a 16-bit CRC
      MsgWithCrc,     ///< message includes a 16-bit CRC
      MsgIgnoreCrc,   ///< response includes a CRC, ignore (fw work-around bug)
      MsgLenFixed,    ///< message is fixed length
      MsgLenVariable  ///< message is variable length
    };

    // write acknowledgment
    static const byte_t RspAck = 0xff;  ///< ack response to write commands
  
    /*!
     * \brief Commands Ids
     */
    enum Cmd
    {
      // compatibility mode commands
      CmdDriveForwardMot1     =  0, ///< drive motor 1 forward
      CmdDriveBackwardMot1    =  1, ///< drive motor 1 backward
      CmdSetMinMainVolt       =  2, ///< set main battery minimum voltage
      CmdSetMaxMainVolt       =  3, ///< set main battery maximum voltage
      CmdDriveForwardMot2     =  4, ///< drive motor 1 forward
      CmdDriveBackwardMot2    =  5, ///< drive motor 2 backward
      CmdDriveMot1            =  6, ///< drive motor 1 forward/back (7-bit)
      CmdDriveMot2            =  7, ///< drive motor 2 foward/back (7-bit)
      
      // mix mode commands
      CmdMixDriveForward      =  8, ///< drive motors forward
      CmdMixDriveBackward     =  9, ///< drive motors backward
      CmdMixTurnRight         = 10, ///< drive motors to turn right
      CmdMixTurnLeft          = 11, ///< drive motors to turn left
      CmdMixDrive             = 12, ///< drive motors foward/back (7-bit)
      CmdMixTurn              = 13, ///< drive motors to turn R/L (7-bit)
  
      // advance commands
      CmdReadEncoderMot1      = 16, ///< read motor 1 encoder
      CmdReadEncoderMot2      = 17, ///< read motor 2 encoder
      CmdReadSpeedMot1        = 18, ///< read motor 1 speed (qpps)
      CmdReadSpeedMot2        = 19, ///< read motor 2 speed (qpps)
      CmdResetEncoderCntrs    = 20, ///< reset encoder (quadrature )counters 
      CmdReadFwVersion        = 21, ///< read firmware version
      CmdSetEncoderReg1       = 22, ///< set encoder register 1 (quadrature)
      CmdSetEncoderReg2       = 23, ///< set encoder register 2 (quadrature)
      CmdReadMainBattVolt     = 24, ///< read main battery voltage
      CmdReadLogicVolt        = 25, ///< read logic battery voltage
      CmdSetMinLogicVolt      = 26, ///< set logic battery minimum voltage
      CmdSetMaxLogicVolt      = 27, ///< set logic battery maximum voltage
      CmdSetVelPidMot1        = 28, ///< set motor 1 velocity PID constants
      CmdSetVelPidMot2        = 29, ///< set motor 2 velocity PID constants
      CmdRead125SpeedMot1     = 30, ///< read motor 1 speed (pulses/125th sec)
      CmdRead125SpeedMot2     = 31, ///< read motor 2 speed (pulses/125th sec)
      CmdDriveDutyMot1        = 32, ///< drive motor 1 at duty cycle (no quad.)
      CmdDriveDutyMot2        = 33, ///< drive motor 2 at duty cycle (no quad.)
      CmdDriveDuty            = 34, ///< drive motors at duty cycle (no quad.)
      CmdDriveQMot1           = 35, ///< drive motor 1 at quad. pulses/second
      CmdDriveQMot2           = 36, ///< drive motor 2 at quad. pulses/second
      CmdDriveQ               = 37, ///< drive motors at quad. pulses/second
      CmdDriveQAccelMot1      = 38, ///< drive motor 1 at quad. pps with accel.
      CmdDriveQAccelMot2      = 39, ///< drive motor 2 at quad. pps with accel.
      CmdDriveQAccel          = 40, ///< drive motors at quad. pps with accel.
      CmdBufDriveQDistMot1    = 41, ///< buffered drive motor 1 to dist at qpps 
      CmdBufDriveQDistMot2    = 42, ///< buffered drive motor 2 to dist at qpps
      CmdBufDriveQDist        = 43, ///< buffered drive motors to dist at qpps
      CmdBufDriveQAccelDistMot1 = 44,
                         ///< buffered drive motor 1 to dist at qpps with accel.
      CmdBufDriveQAccelDistMot2 = 45,
                         ///< buffered drive motor 2 to dist at qpps with accel.
      CmdBufDriveQAccelDist   = 46,
                         ///< buffered drive motors to dist at qpps with accel.
      CmdReadBufLen           = 47, ///< read motors bufferd command length

      // more here...
     
      CmdReadMotorDraw        = 49, ///< read motors amp draw
      CmdDriveQAccel2         = 50, ///< drive motros at qpps with 2 accel vals
      CmdBufDriveQAccel2Dist  = 51,
                            /// buffered drive motors to dist at qpps w/ 2 accel

      // more here...

      CmdReadVelPidMot1       = 55, ///< read motor 1 velocity PID constants
      CmdReadVelPidMot2       = 56, ///< read motor 2 velocity PID constants
      CmdSetMainBattCutoffs   = 57, ///< set main battery voltage cutoffs
      CmdSetLogicCutoffs      = 58, ///< set logic voltage cutoffs
      CmdReadMainBattCutoffs  = 59, ///< read main battery cutoff settings
      CmdReadLogicCutoffs     = 60, ///< read logic voltage cutoff settings
      CmdSetPosPidMot1        = 61, ///< set motor 1 position PID constants
      CmdSetPosPidMot2        = 62, ///< set motor 2 position PID constants
      CmdReadPosPidMot1       = 63, ///< read motor 1 position PID constants
      CmdReadPosPidMot2       = 64, ///< read motor 2 position PID constants
      CmdBufDriveQProfPosMot1 = 65,
                ///< drive motor 1 with signed qpps, accel, deccel and position
      CmdBufDriveQProfPosMot2 = 66,
                ///< drive motor 2 with signed qpps, accel, deccel and position
      CmdBufDriveQProfPos     = 67,
                ///< drive motors with signed qpps, accel, deccel and position

      // more here...

      CmdReadTemp             = 82, ///< read board temperature
      CmdReadTemp2            = 83, ///< read board second temperature
      CmdReadStatus           = 90, ///< read current board status
      CmdReadEncoderMode      = 91, ///< read encoder mode for both motors
      CmdSetEncoderModeMot1   = 92, ///< set motor 1 encoder mode
      CmdSetEncoderModeMot2   = 93, ///< set motor 2 encoder mode
      CmdWriteEEPROM          = 94, ///< write settings to EEPROM

      // more here...

      // Note:  RoboClaw User Manual v5 error. The Cmd*MaxCurrentMotn commands
      //        numbers are shifted down by 1.
      CmdSetMaxCurrentMot1    = 133,  ///< set motor 1 maximum current limit
      CmdSetMaxCurrentMot2    = 134,  ///< set motor 2 maximum current limit
      CmdReadMaxCurrentMot1   = 135,  ///< read motor 1 maximum current limit
      CmdReadMaxCurrentMot2   = 136   ///< read motor 2 maximum current limit
    };
  
    //
    // Message motor speed parameters
    //

    // 8-bit speed with direction determined by command id
    static const byte_t ParamSpeedMin     = 0;    ///< minimum absolute speed
    static const byte_t ParamSpeedMax     = 127;  ///< maximum absolute speed
    static const byte_t ParamStop         = 0;    ///< stop
  
    // 7-bit speed
    static const byte_t ParamSpeed7MaxBwd = 0;   ///< minimum absolute speed
    static const byte_t ParamStop7        = 64;  ///< all stop
    static const byte_t ParamSpeed7MaxFwd = 127; ///< maximum absolute speed
  
    //
    // Message turn speed parameters
    //

    // 8-bit turn rate with rotation direction determined by command id
    static const byte_t ParamTurnMin      = 0;    ///< min absolute turn speed
    static const byte_t ParamTurnMax      = 127;  ///< max absolute turn speed
    static const byte_t ParamNoTurn       = 0;    ///< no turn
  
    // 7-bit turn rate
    static const byte_t ParamTurn7MaxLeft   = 0;    ///< full turn left
    static const byte_t ParamNoTurn7        = 64;   ///< no turn
    static const byte_t ParamTurn7MaxRight  = 127;  ///< full turn right
  
    //
    // Duty cycle parameters
    //
    static const int ParamDutyCycleMin  = -32767;   ///< -100% duty cycle
    static const int ParamDutyCycleStop =      0;   ///<    0% duty cycle
    static const int ParamDutyCycleMax  =  32767;   ///<  100% duty cycle
    
    //
    // Version: <string><NL>0<CRC16>
    //
    static const size_t ParamVerLen     = 48; ///< version string length.
    static const size_t ParamVerLenMin  =  4; ///< version minimum length.
  
    //
    // Message voltage and current parameters
    //

    // voltages
    static const double ParamVoltScale    =  0.1; ///< 10/th of a volt
    static const double ParamVoltMainMin  =  6.0; ///< min main battery voltage
    static const double ParamVoltLogicMin =  5.5; ///< minimum logic voltage
    static const double ParamVoltMax      = 34.0; ///< maximum voltage

    // motor ampere draw
    static const double ParamAmpScale   = 0.01; ///< sensed_value * S = A
    static const double ParamAmpMin     = 0.0;  ///< minimum amps
    static const double ParamAmpMinSane = 0.5;  ///< minimum amps for operation
    static const double ParamAmpMax     = 15.0; ///< maximum amps
  
    //
    // These complicated parameter conversion values are for commands
    // 26, 27. Don't use.
    //
    // main battery (powers motors) voltages
    static const byte_t ParamVoltMinMainMin =   0;  ///<   0 ==  6 volts
    static const byte_t ParamVoltMinMainMax = 120;  ///< 120 == 30 volts
    static const double ParamVoltMinMainOff = 6.0;  ///<   V @ 0 value
    static const double ParamVotlMinMainDft = 6.0;  ///< default on reset
    static const byte_t ParamVoltMaxMainMin =   0;  ///<   0 ==  0 volts
    static const byte_t ParamVoltMaxMainMax = 154;  ///< 154 == 30 volts
  
    // logic battery (powers circuitry) voltages
    static const byte_t ParamVoltMinLogicMin =   0;  ///<   0 ==  6 volts
    static const byte_t ParamVoltMinLogicMax = 140;  ///< 120 == 30 volts
    static const double ParamVoltMinLogicOff = 6.0;  ///<   V @ 0 value
    static const byte_t ParamVoltMaxLogicMin =  30;  ///<  30 ==  6 volts
    static const byte_t ParamVoltMaxLogicMax = 175;  ///< 175 == 34 volts
  
    // battery scaling factors
    static const double ParamVoltMinS        = 5.0;  ///< (V - Off) * S = value
    static const double ParamVoltMaxS        = 5.12; ///< V * S = value
    static const double ParamVoltSensedS     = 0.1;  ///< sensed_value * S = V
  
    //
    // Temperature parameters
    //
    static const double ParamTempScale  = 0.1; ///< sensed_value * S = C
  
    /*!
     * \brief Board status bits.
     */
    enum ParamStatus
    {
      ParamStatusNormal           = 0x0000, ///< normal
      ParamStatusWarnMot1OverCur  = 0x0001, ///< motor 1 over current warning
      ParamStatusWarnMot2OverCur  = 0x0002, ///< motor 2 over current warning
      ParamStatusEStopped         = 0x0004, ///< emergency stopped
      ParamStatusErrTemp          = 0x0008, ///< temperature out-of-range error
      ParamStatusErrTemp2         = 0x0010, ///< temperature2 out-of-range error
      ParamStatusErrMainBattHigh  = 0x0020, ///< main battery over voltage error
      ParamStatusErrLogicBattHigh = 0x0040, ///< logic battery over volt error
      ParamStatusErrLogicBattLow  = 0x0080, ///< logic battery under volt error
      ParamStatusErrMot1Fault     = 0x0100, ///< motor 1 drive fault
      ParamStatusErrMot2Fault     = 0x0200, ///< motor 2 drive fault
      ParamStatusWarnMainBattHigh = 0x0400, ///< main battery over volt warning
      ParamStatusWarnMainBattLow  = 0x0800, ///< main battery under volt warning
      ParamStatusWarnTemp         = 0x1000, ///< temp out-of-range warning
      ParamStatusWarnTemp2        = 0x2000, ///< temp2 out-of-range warning
      ParamStatusMot1Home         = 0x4000, ///< motor 1 home
      ParamStatusMot2Home         = 0x8000  ///< motor 2 home
    };
  
    //
    // Encoder parameters
    //
    static const byte_t ParamEncModeRCAnalogBit = 0x80; ///< RC/analog bit
    static const byte_t ParamEncModeRCAnalogDis = 0x00; ///< RC/analog disable
    static const byte_t ParamEncModeRCAnalogEn  = 0x80; ///< RC/analog enable
    static const byte_t ParamEncModeQuadAbsBit  = 0x01; ///< quad/absolute bit
    static const byte_t ParamEncModeQuad        = 0x00; ///< quadrature encoder
    static const byte_t ParamEncModeAbs         = 0x01; ///< absolute encoder
  
    static const long long ParamEncQuadMin = 0; ///< quadrature encoder min
    static const long long ParamEncQuadMax = 4294967295LL;
                                                    ///< quadrature encoder max
  
    /*!
     * \brief Encoder status on read.
     */
    enum ParamEncStatus
    {
      ParamEncStatusUnderFlow   = 0x01,  ///< quadrature encoder underflow
      ParamEncStatusDirBackward = 0x02,  ///< direction (sign)
      ParamEncStatusOverFlow    = 0x04   ///< quadrature encoder overflow
    };
  
    //
    // Velocity PID parameters
    //
    static const long ParamVelPidQppsDft = 44000;   ///< speed of encoder at max
    static const long ParamVelPidPDft = 0x00010000; ///< vel proportional const
    static const long ParamVelPidIDft = 0x00008000; ///< vel integrative const
    static const long ParamVelPidDDft = 0x00004000; ///< vel derivative const
    static const long ParamVelPidCvt  = 0x00010000; ///< vel const conversion

    //
    // Position PID parameters (TBD)
    //
    static const long ParamPosPidPDft         = 0; ///< pos proportional const
    static const long ParamPosPidIDft         = 0; ///< pos integrative const
    static const long ParamPosPidDDft         = 0; ///< pos derivative const
    static const long ParamPosPidMaxIDft      = 0; ///< max I windup 
    static const long ParamPosPidDeadzoneDft  = 0; ///< deadzone
    static const long ParamPosPidMinPos       = 0; ///< min position
    static const long ParamPosPidMaxPos       = 0; ///< max position 
  
    //
    // Command buffering
    //
    static const byte_t ParamCmdBufQueue    = 0;  ///< queue command
    static const byte_t ParamCmdBufPreempt  = 1;  ///< preempt all buffered cmds
    static const byte_t ParamCmdBufSize     = 31; ///< cmd buffer size (num q'd)
    static const byte_t ParamCmdBufExec     = 0;  ///< last command is executing
    static const byte_t ParamCmdBufEmpty    = 0x80; ///< buffer is empty

    //
    // Message response timeout
    //
    static const uint_t CmdTimeout = 10000;  ///< command timeout (usec)
    static const uint_t RspTimeout = 12000;  ///< response timeout (usec)

    //
    // Maximum buffers sizes
    //
    static const int  CmdMaxBufLen  = 64; ///< maximum command length (bytes)
    static const int  RspMaxBufLen  = 64; ///< maximum response length (bytes)
    static const int  CmdHdrLen     = 2;  ///< cmd header length [address,cmdid]

  
    //
    // Common parameter field starting byte positions
    //
    static const int  FieldPosAddr  = 0;  ///< controller address position
    static const int  FieldPosCmd   = 1;  ///< command id position


    // ........................................................................
    // Abstracted Interface
    // ........................................................................
  
    //
    // Motor 1 and 2 indices
    //
    static const int Motor1     = 0;  ///< motor 1
    static const int Motor2     = 1;  ///< motor 2 
    static const int NumMotors  = 2;  ///< number of motors/controller
  
    //
    // Motor forward/backward rotation sense.
    //
    static const int MotorDirUnknown  =  0; ///< unknown
    static const int MotorDirNormal   =  1; ///< normal
    static const int MotorDirReverse  = -1; ///< reverse (forward=backward, etc)
  
    //
    // Motor speeds (scaled)
    //
    static const int MotorSpeedMaxBackward  = -127; ///< full reverse
    static const int MotorSpeedStop         =    0; ///< stop
    static const int MotorSpeedMaxForward   =  127; ///< full forward
  

#ifndef SWIG
    // -------------------------------------------------------------------------
    // RoboClawChipSelect Class
    // -------------------------------------------------------------------------
  
    /*!
     * \brief RoboClaw motor controller chip select class.
     *
     * The RoboClawChipSelect class instance is used to select the motor
     * controller, given its address.
     */
    class RoboClawChipSelect
    {
    public:
      /*!
       * \brief Default constructor.
       */
      RoboClawChipSelect();

      /*!
       * \brief Destructor.
       */
      virtual ~RoboClawChipSelect();

      /*!
       * \brief Motor controller select function.
       *
       * The motor controllers are on a multi-drop serial bus. Serial does not
       * support multiple tx drivers on the same bus. Since there are two motor
       * controllers, each with a transmit line (odroid receive), the select
       * function disconnects one tx while connects the target tx.
       *
       * \param fd        Open serial file descriptor.
       * \param addrSel   Target motor controller to be selected. Controllers
       *                  are identified by their address.
       */
      virtual void select(int fd, byte_t addrSel);

    protected:
      byte_t  m_addrLast; ///< last selected motor controller, id'd by address
    };
#endif // SWIG


#ifndef SWIG
    // -------------------------------------------------------------------------
    // RoboClawComm Class
    // -------------------------------------------------------------------------

    /*!
     * \brief RoboClaw communication class.
     *
     * The RoboClaw class instances bind to this class.
     */
    class RoboClawComm
    {
    public:
      /*!
       * \brief Default constructor.
       */
      RoboClawComm();
  
      /*!
       * \brief Initialization constructor.
       *
       * \param strDevName    Serial device name.
       * \param nBaudRate     Serial device baud rate.
       * \param pChipSelect   (Derived) motor controller selection class object.
       *
       * \copydoc doc_return_mc_std
       */
      RoboClawComm(std::string        &strDevName,
                   int                 nBaudRate,
                   RoboClawChipSelect *pChipSelect = NULL);

      /*!
       * \brief Destructor.
       */
      virtual ~RoboClawComm();
  
      /*!
       * \brief Open connection to motor controller(s).
       *
       * \param strDevName    Serial device name.
       * \param nBaudRate     Serial device baud rate.
       * \param pChipSelect   (Derived) motor controller selection class object.
       *
       * \copydoc doc_return_mc_std
       */
      virtual int open(std::string        &strDevName,
                       int                 nBaudRate,
                       RoboClawChipSelect *pChipSelect = NULL);

      /*!
       * \brief Close connection to motor controller.
       *
       * \copydoc doc_return_mc_std
       */
      virtual int close();
  
      /*!
       * \brief Flush UART input FIFO.
       *
       * \copydoc doc_return_mc_std
       */
      virtual int flushInput();
  
      /*!
       * \brief Test if connection is open.
       *
       * \return Returns true or false.
       */
      virtual bool isOpen() const
      {
        return m_fd >= 0? true: false;
      }

      /*!
       * \brief Test if connection is open.
       *
       * \param bEnDis    Enable/disable debugging.
       */
      virtual void enableDbg(bool bEnDis)
      {
        m_bDbgEnable = bEnDis;
      }

      /*!
       * \brief Get last sent command's calculated 16-bit CRC.
       *
       * \return Returns CRC.
       */
      uint_t getLastCmdCrc()
      {
        return m_uCrcCmd;
      }

      /*!
       * \brief Get last received response's appended 16-bit CRC.
       *
       * \return Returns CRC.
       */
      uint_t getLastRspCrc()
      {
        return m_uCrcRsp;
      }

      /*!
       * \brief Execute a command with no response.
       *
       * \note Deprecated. 
       *
       * \param [in] cmd  Packed command buffer.
       * \param lenCmd    Length of command (bytes).
       *
       * \copydoc doc_return_mc_std
       */
      virtual int execCmd(byte_t cmd[], size_t lenCmd);

      /*!
       * \brief Execute a command with an acknowledgement response.
       *
       * \param [in] cmd  Packed command buffer.
       * \param lenCmd    Length of command (bytes).
       * \param fmtCmd    Command format. Do [not] append command with a 16-bit
       *                  CRC. If included, the command buffer must be of
       *                  length \h_ge lenCmd+2.
       *
       * \copydoc doc_return_mc_std
       */
      virtual int execCmdWithAckRsp(byte_t cmd[], size_t lenCmd,
                                    MsgFmt fmtCmd = MsgWithNoCrc);

      /*!
       * \brief Execute a command with an data response.
       *
       * \param [in] cmd  Packed command buffer.
       * \param lenCmd    Length of command (bytes).
       * \param [out] rsp Response buffer.
       * \param lenRsp    Expected length of packed response (bytes).
       * \param fmtCmd    Command format. Do [not] append command with a 16-bit
       *                  CRC. If included, the command buffer must be of
       *                  length \h_ge lenCmd+2.
       * \param fmtRsp    Response format. Response has with a 16-bit
       *                  CRC. If ignore, firmware bug.
       *
       * \copydoc doc_return_mc_std
       */
      virtual int execCmdWithDataRsp(byte_t cmd[], size_t lenCmd,
                                     byte_t rsp[], size_t lenRsp,
                                     MsgFmt fmtCmd = MsgWithNoCrc,
                                     MsgFmt fmtRsp = MsgWithCrc);

      /*!
       * \brief Send command over serial connection to motor controller.
       *
       * \param [in] cmd  Packed command buffer.
       * \param lenCmd    Length of command (bytes).
       * \param fmtCmd    Command format. Do [not] append command with a 16-bit
       *                  CRC. If included, the command buffer must be of
       *                  length \h_ge lenCmd+2.
       *
       * \copydoc doc_return_mc_std
       */
      virtual int sendCmd(byte_t cmd[], size_t lenCmd,
                          MsgFmt fmtCmd = MsgWithNoCrc);
  
      /*!
       * \brief Receive data response over serial connection from motor
       * controller.
       *
       * \param [out] rsp   Response buffer.
       * \param lenRsp      Expected length of packed response (bytes).
       *
       * \return Number of bytes received or -1 on error.
       */
      virtual int recvDataRsp(byte_t rsp[], size_t lenRsp);
  
      /*!
       * \brief Receive acknowledgement response over serial connection from
       * motor controller.
       *
       * \copydoc doc_return_mc_std
       */
      virtual int recvAck();

      /*!
       * \brief Pack 16-bit unsigned value into buffer.
       *
       * Order is big-endian (MSB first).
       *
       * \param [in] val   Value to pack.
       * \param [out] buf  Destination buffer.
       *
       * \return Number of bytes packed(2).
       */
      int pack16(uint_t val, byte_t buf[]);

      /*!
       * \brief Unpack 16-bit unsigned value from buffer.
       *
       * Buffer is big-endian (MSB first).
       *
       * \param [in] buf  Source buffer.
       * \param [out] val Unpacked value.
       *
       * \return Number of bytes unpacked(2).
       */
      int unpack16(byte_t buf[], uint_t &val);

      /*!
       * \brief Pack 16-bit signed value into buffer.
       *
       * Order is big-endian (MSB first).
       *
       * \param [in] val   Value to pack.
       * \param [out] buf  Destination buffer.
       *
       * \return Number of bytes packed(2).
       */
      int pack16(int val, byte_t buf[])
      {
        return pack16((uint_t)val, buf);
      }

      /*!
       * \brief Unpack 16-bit signed value from buffer.
       *
       * Buffer is big-endian (MSB first).
       *
       * \param [in] buf  Source buffer.
       * \param [out] val Unpacked value.
       *
       * \return Number of bytes unpacked(2).
       */
      int unpack16(byte_t buf[], int &val);

      /*!
       * \brief Pack 32-bit unsigned value into buffer.
       *
       * Order is big-endian (MSB first).
       *
       * \param [in] val   Value to pack.
       * \param [out] buf  Destination buffer.
       *
       * \return Number of bytes packed(4).
       */
      int pack32(uint_t val, byte_t buf[]);

      /*!
       * \brief Unpack 32-bit unsigned value from buffer.
       *
       * Buffer is big-endian (MSB first).
       *
       * \param [in] buf  Source buffer.
       * \param [out] val Unpacked value.
       *
       * \return Number of bytes unpacked(4).
       */
      int unpack32(byte_t buf[], uint_t &val);

      /*!
       * \brief Pack 32-bit signed value into buffer.
       *
       * Order is big-endian (MSB first).
       *
       * \param [in] val   Value to pack.
       * \param [out] buf  Destination buffer.
       *
       * \return Number of bytes packed(4).
       */
      int pack32(int val, byte_t buf[])
      {
        return pack32((uint_t)val, buf);
      }

      /*!
       * \brief Unpack 32-bit signed value from buffer.
       *
       * Buffer is big-endian (MSB first).
       *
       * \param [in] buf  Source buffer.
       * \param [out] val Unpacked value.
       *
       * \return Number of bytes unpacked(4).
       */
      int unpack32(byte_t buf[], int &val);

      /*!
       * \brief Calculate 7-bit checksum over buffer.
       *
       * \note For FW versions \h_lt 4.0
       *
       * \param [in] buf    Buffer holding command/response.
       * \param lenBuf      Length of data in buffer (bytes).
       * \param bAck        If true, or-in request ack bit to checksum
       *                    (commands only).
       *
       * \return Checksum byte.
       */
      virtual byte_t checksum(byte_t buf[], size_t lenBuf, bool bAck=false);
  
      /*!
       * \brief Calculate 16-bit CRC over buffer.
       *
       * \param crc         CRC starting seed value.
       * \param [in] buf    Buffer.
       * \param lenbuf      Length of data in buffer (bytes).
       *
       * \return Returns calculated 16-bit CRC over buffer.
       */
      virtual uint_t crc16(uint_t crc, byte_t buf[], size_t lenBuf);
  
    protected:
      std::string         m_strDevName;   ///< serial device name
      int                 m_nBaudRate;    ///< serial baud rate
      int                 m_fd;           ///< opened file descriptor
      RoboClawChipSelect *m_pChipSelect;  ///< motor ctlr select object
      uint_t              m_uCrcCmd;      ///< last sent command's CRC
      uint_t              m_uCrcRsp;      ///< last received response's CRC
      bool                m_bDbgEnable;   ///< do [not] enable debugging
    };
#endif // SWIG


#ifndef SWIG
    // -------------------------------------------------------------------------
    // RoboClaw Class
    // -------------------------------------------------------------------------

    /*!
     * \brief RoboClaw 2 motor controller class.
     *
     * RoboClaw by Ion Motion Control.
     */
    class RoboClaw
    {
    public:
      /*!
       * \brief Initialization constructor.
       *
       * \param comm      Bound controller communication object.
       * \param address   Motor controller address.
       * \param strNameId Motor controller name identifier.
       */
      RoboClaw(RoboClawComm      &comm,
               const byte_t      address=AddrDft,
               const std::string &strNameId="RoboClaw");
  
      /*!
       * \brief Destructor.
       */
      virtual ~RoboClaw();
  
      /*!
       * \brief Probe address of motor controller.
       *
       * A weakness of the RoboClaw controller firmware is that there is no
       * way to fetch its assigned address. The probe function uses a read-only
       * command to determine, by proxy, whether the address is the controller's
       * address. A false outcome is assumed if no response is received.
       *
       * \param address Motor controller address to test.
       *
       * \return Returns true if the motor controller has this address, false
       * otherwise.
       */
      virtual bool probe(byte_t address);


      //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
      // Attributes
      //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  
      /*!
       * \brief Get RoboClaw's bound communication object.
       *
       * \return Returns object.
       */
      RoboClawComm &getComm()
      {
        return m_comm;
      }

      /*!
       * \brief Test if connection is open.
       *
       * \return Returns true or false.
       */
      bool isOpen() const
      {
        return m_comm.isOpen();
      }

      /*!
       * \brief Get the controller address associated with this class instance.
       *
       * \return Returns address.
       */
      byte_t getAddress() const
      {
        return m_address;
      }

      /*!
       * \brief Set class instance address.
       *
       * \note The address may not match the controller's address. It is up to
       * the higher-level code to determine that.
       *
       * \param address   Motor controller address.
       */
      void setAddress(byte_t address)
      {
        m_address = address;
      }

      /*!
       * \brief Get class instance name identifier.
       *
       * \return Returns string id.
       */
      std::string getNameId() const
      {
        return m_strNameId;
      }

      /*!
       * \brief Set class instance name identifier.
       *
       * \param strNameId Motor controller name identifier.
       */
      void setNameId(const std::string &strNameId)
      {
        m_strNameId = strNameId;
      }

      /*!
       * \brief Get the direction of motor rotation.
       *
       * The motor direction is a software construct to apply a normal or
       * reverse the sense of forward/backward rotation.
       *
       * \param motor   Motor index Motor1(0) or Motor2(1).
       *
       * \return Returns motor direction MotorDirNormal(1) or
       * MotorDirReverse(-1).
       */
      virtual byte_t getMotorDir(int motor) const
      {
        if( isValidMotor(motor) )
        {
          return m_nMotorDir[motor];
        }
        else
        {
          return MotorDirNormal;
        }
      }

      /*!
       * \brief set the direction of motor rotation.
       *
       * The motor direction is a software construct to apply a normal or
       * reverse the sense of forward/backward rotation.
       *
       * \param motor     Motor index Motor1(0) or Motor2(1).
       * \param motorDir  Motor direction MotorDirNormal(1) or
       *                  MotorDirReverse(-1).
       */
      virtual void setMotorDir(int motor, int motorDir)
      {
        if( isValidMotor(motor) )
        {
          m_nMotorDir[motor] = motorDir == MotorDirReverse? MotorDirReverse:
                                                            MotorDirNormal;
        }
      }

      /*!
       * \brief Get the RoboClaw's main battery minimum and maximum voltage
       * cutoff settable range.
       *
       * \param [out] minmin  Minimum minimum voltage (V).
       * \param [out] maxmax  Maximum maximum voltage (V).
       */
      void getMainBattCutoffRange(double &minmin, double &maxmax) const;

      /*!
       * \brief Get the RoboClaw's logic minimum and maximum voltage
       * cutoff settable range.
       *
       * \param [out] minmin  Minimum minimum voltage (V).
       * \param [out] maxmax  Maximum maximum voltage (V).
       */
      void getLogicCutoffRange(double &minmin, double &maxmax) const;


      //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
      // Commands
      //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
      
      /*!
       * \brief Read the RoboClaw's firmware version.
       *
       * Command Numbers: 21
       *
       * \param [out] strFwVer  Read version string.
       *
       * \copydoc doc_return_mc_std
       */
      virtual int cmdReadFwVersion(std::string &strFwVer);

      /*!
       * \brief Read the RoboClaw's main battery input voltage.
       *
       * Command Numbers: 24
       *
       * \param [out] volts   Voltage (V).
       *
       * \copydoc doc_return_mc_std
       */
      virtual int cmdReadMainBattVoltage(double &volts);

      /*!
       * \brief Read the RoboClaw's main battery minimum and maximum voltage
       * cutoff settings.
       *
       * Command Numbers: 59
       *
       * \param [out] min   Minimum voltage (V).
       * \param [out] max   Maximum voltage (V).
       *
       * \copydoc doc_return_mc_std
       */
      virtual int cmdReadMainBattCutoffs(double &min, double &max);

      /*!
       * \brief Set the RoboClaw's main battery minimum and maximum voltage
       * cutoff settings.
       *
       * Note: The minimum value is always set to 0 (6.0V) on power-up. 
       *
       * The new values are set in volatile memory only. To save to non-volatile
       * memory, issue a write settings to EEPROM command.
       *
       * Command Numbers: 57
       *
       * \param [in] min  Minimum voltage (V).
       * \param [in] max  Maximum voltage (V).
       *
       * \copydoc doc_return_mc_std
       */
      virtual int cmdSetMainBattCutoffs(const double min, const double max);

      /*!
       * \brief Read the RoboClaw's LB-/LB+ terminals input voltage powered by
       * and optional logic dedicated battery.
       *
       * Command Numbers: 25
       *
       * \param [out] volts   Voltage (V).
       *
       * \copydoc doc_return_mc_std
       */
      virtual int cmdReadLogicVoltage(double &volts);

      /*!
       * \brief Read the RoboClaw's optional logic dedicated battery
       * minimum and maximum voltage cutoff settings. The battery is connected
       * to the LB-/LB+ terminals
       *
       * Command Numbers: 60
       *
       * \param [out] min   Minimum voltage (V).
       * \param [out] max   Maximum voltage (V).
       *
       * \copydoc doc_return_mc_std
       */
      virtual int cmdReadLogicCutoffs(double &min, double &max);

      /*!
       * \brief Set the RoboClaw's optional logic dedicated battery
       * minimum and maximum voltage cutoff settings. The battery is connected
       * to the LB-/LB+ terminals
       *
       * Note: The minimum value is always set to 0 (6.0V) on power-up.
       * RDK:   The documention is conflicted here. 3.0V or 6.0V?
       *
       * The new values are set in volatile memory only. To save to non-volatile
       * memory, issue a write settings to EEPROM command.
       *
       * Command Numbers: 58
       *
       * \param [in] min  Minimum voltage (V).
       * \param [in] max  Maximum voltage (V).
       *
       * \copydoc doc_return_mc_std
       */
      virtual int cmdSetLogicCutoffs(const double min, const double max);

      /*!
       * \brief Read motor's velocity PID constants.
       *
       * Command Numbers: 55, 56
       *
       * \param motor       Motor index Motor1(0) or Motor2(1).
       * \param [out] Kp    Velocity PID proportional constant.
       * \param [out] Ki    Velocity PID intergral constant.
       * \param [out] Kd    Velocity PID derivative constant.
       * \param [out] qpps  Maximum speed of motor.
       *
       * \copydoc doc_return_mc_std
       */
      virtual int cmdReadVelocityPidConst(int   motor,
                                          u32_t &Kp,
                                          u32_t &Ki,
                                          u32_t &Kd,
                                          u32_t &qpps);

      /*!
       * \brief Set motor's velocity PID constants.
       *
       * The new values are set in volatile memory only. To save to non-volatile
       * memory, issue a write settings to EEPROM command.
       *
       * Command Numbers: 28, 29
       *
       * \param motor     Motor index Motor1(0) or Motor2(1).
       * \param [in] Kp   Velocity PID proportional constant.
       * \param [in] Ki   Velocity PID intergral constant.
       * \param [in] Kd   Velocity PID derivative constant.
       * \param [in] qpps Maximum speed of motor.
       *
       * \copydoc doc_return_mc_std
       */
      virtual int cmdSetVelocityPidConst(int         motor,
                                         const u32_t Kp,
                                         const u32_t Ki,
                                         const u32_t Kd,
                                         const u32_t qpps);

      /*!
       * \brief Read the motors ampere draw.
       *
       * Command Numbers: 49
       *
       * \param [out] amps1     Motor1 current draw (amperes)
       * \param [out] amps2     Motor2 current draw (amperes)
       *
       * \copydoc doc_return_mc_std
       */
      virtual int cmdReadMotorCurrentDraw(double &amps1, double &amps2);

      /*!
       * \brief Read a motor's maximum current limit.
       *
       * Command Numbers: 136, 137
       *
       * \param motor           Motor index Motor1(0) or Motor2(1).
       * \param [out] maxAmps   Motor maximum current limit (amperes)
       *
       * \copydoc doc_return_mc_std
       */
      virtual int cmdReadMotorMaxCurrentLimit(int motor, double &maxAmps);

      /*!
       * \brief Set a motor's maximum current limit.
       *
       * Command Numbers: 134, 135
       *
       * \param motor     Motor index Motor1(0) or Motor2(1).
       * \param maxAmps   Motor maximum current limit (amperes)
       *
       * \copydoc doc_return_mc_std
       */
      virtual int cmdSetMotorMaxCurrentLimit(int motor, const double maxAmps);

      /*!
       * \brief Read RoboClaw board's temperature.
       *
       * Command Numbers: 82
       *
       * \param [out] temp  Board temperature (C)
       *
       * \copydoc doc_return_mc_std
       */
      virtual int cmdReadBoardTemperature(double &temp);

      /*!
       * \brief Read RoboClaw board's status bits.
       *
       * Command Numbers: 90
       *
       * \param [out] status  Board status bits.
       *
       * \copydoc doc_return_mc_std
       */
      virtual int cmdReadStatus(uint_t &status);

      /*!
       * \brief Read RoboClaw board's temperature.
       *
       * Command Numbers: 47
       *
       * \param [out] len1  Motor1 command buffer length.
       * \param [out] len2  Motor2 command buffer length.
       *
       * \copydoc doc_return_mc_std
       */
      virtual int cmdReadCmdBufLengths(uint_t &len1, uint_t &len2);

      /*!
       * \brief Read encoder mode.
       *
       * Command Numbers: 91
       *
       * \param [out] mode1 Motor1 encoder mode.
       * \param [out] mode2 Motor2 encoder mode.
       *
       * \copydoc doc_return_mc_std
       */
      virtual int cmdReadEncoderMode(byte_t &mode1, byte_t &mode2);

      /*!
       * \brief Set a motor's encoder mode.
       *
       * The new values are set in volatile memory only. To save to non-volatile
       * memory, issue a write settings to EEPROM command.
       *
       * Command Numbers: 92, 93
       *
       * \param motor   Motor index Motor1(0) or Motor2(1).
       * \param mode    Motor encoder mode.
       *
       * \copydoc doc_return_mc_std
       */
      virtual int cmdSetEncoderMode(int motor, byte_t mode);

      /*!
       * \brief Set both motors' encoder mode.
       *
       * The new values are set in volatile memory only. To save to non-volatile
       * memory, issue a write settings to EEPROM command.
       *
       * Command Numbers: 92, 93
       *
       * \param mode1   Motor1 encoder mode.
       * \param mode2   Motor2 encoder mode.
       *
       * \copydoc doc_return_mc_std
       */
      virtual int cmdSetEncoderMode2(byte_t mode1, byte_t mode2);

      /*!
       * \brief Reset both motors' encoders.
       *
       * Command Numbers: 20
       *
       * \copydoc doc_return_mc_std
       */
      virtual int cmdResetQEncoders();

      /*!
       * \brief Read motor's encoder.
       *
       * Command Numbers: 16, 17
       *
       * \param motor         Motor index Motor1(0) or Motor2(1).
       * \param [out] encoder Unsigned encoder value (quad pulses).
       * \param [out] status  Status bits indicating sign, and over/under flow.
       *
       * \copydoc doc_return_mc_std
       */
      virtual int cmdReadQEncoder(int motor, s64_t &encoder);

      /*!
       * \brief Drive both motors at the given duty cycle.
       *
       * Motor quadrature encoders are ignored.
       *
       * Command Numbers: 34
       *
       * \param duty1   Motor1 normalized duty cycle [-1.0, 1.0].
       *                Sign indicates direction.
       * \param duty2   Motor2 normalized duty cycle [-1.0, 1.0].
       *                Sign indicates direction.
       *
       * \copydoc doc_return_mc_std
       */
      virtual int cmdDutyDrive2(double duty1, double duty2);

      /*!
       * \brief Read motor's speed.
       *
       * Command Numbers: 18, 19
       *
       * \param motor         Motor index Motor1(0) or Motor2(1).
       * \param [out] speed   Signed speed value (qpps).
       *
       * \copydoc doc_return_mc_std
       */
      virtual int cmdReadQSpeed(int motor, s32_t &speed);

      /*!
       * \brief Drive a motor at the given speed.
       *
       * Movement is controlled using quadrature encoders.
       *
       * Command Numbers: 35, 36
       *
       * \param motor   Motor index Motor1(0) or Motor2(1).
       * \param speed   Motor speed in quad pulses per second.
       *                Sign indicates direction.
       *
       * \copydoc doc_return_mc_std
       */
      virtual int cmdQDrive(int motor, s32_t speed);

      /*!
       * \brief Drive both motors at the given speeds.
       *
       * Movement is controlled using quadrature encoders.
       *
       * Command Numbers: 37
       *
       * \param speed1  Motor1 speed in quad pulses per second.
       *                Sign indicates direction.
       * \param speed2  Motor2 speed in quad pulses per second.
       *                Sign indicates direction.
       *
       * \copydoc doc_return_mc_std
       */
      virtual int cmdQDrive2(s32_t speed1, s32_t speed2);

      /*!
       * \brief Drive a motor at the given speed and with the given
       * acceleration.
       *
       * Movement is controlled using quadrature encoders.
       *
       * Command Numbers: 38, 39
       *
       * \param motor   Motor index Motor1(0) or Motor2(1).
       * \param speed   Motor speed in quad pulses per second.
       *                Sign indicates direction.
       * \param accel   Acceleration (qpps/s).
       *
       * \copydoc doc_return_mc_std
       */
      virtual int cmdQDriveWithAccel(int motor, s32_t speed, u32_t accel);

      /*!
       * \brief Drive both motors at the given speeds and with the given
       * accelerations.
       *
       * Movement is controlled using quadrature encoders.
       *
       * Command Numbers: 50
       *
       * \param speed1  Motor1 speed in quad pulses per second.
       *                Sign indicates direction.
       * \param accel1  Motor1 acceleration (qpps/s).
       * \param speed2  Motor2 speed in quad pulses per second.
       *                Sign indicates direction.
       * \param accel2  Motor2 acceleration (qpps/s).
       *
       * \copydoc doc_return_mc_std
       */
      virtual int cmdQDriveWithAccel(s32_t speed1, u32_t accel1,
                                     s32_t speed2, u32_t accel2);

      /*!
       * \brief Drive a motor at the given speed for a given distance.
       *
       * Movement is controlled using quadrature encoders.
       *
       * This is a buffered command. Commands execute immediatly if no commands
       * are already in the motor drive queue or if the queue paramter is false.
       * Otherwise, the command is placed at end of the command buffer execution
       * queue.
       *
       * Command Numbers: 41, 42
       *
       * \param motor   Motor index Motor1(0) or Motor2(1).
       * \param speed   Motor speed in quad pulses per second.
       *                Sign indicates direction.
       * \param dist    Distance from current position (quad pulses).
       * \param bQueue  Do [not] queue the command. If false, the motor drive
       *                queue is flushed and any existing command is preempted
       *                by this command.
       *
       * \copydoc doc_return_mc_std
       */
      virtual int cmdQDriveForDist(int motor, s32_t speed, u32_t dist,
                                   bool bQueue=true);

      /*!
       * \brief Drive both motors at the given speeds for a given distances.
       *
       * Movement is controlled using quadrature encoders.
       *
       * This is a buffered command. Commands execute immediatly if no commands
       * are already in the motor drive queue or if the queue paramter is false.
       * Otherwise, the command is placed at end of the command buffer execution
       * queue.
       *
       * Command Numbers: 43
       *
       * \param speed1  Motor1 speed in quad pulses per second.
       *                Sign indicates direction.
       * \param dist1   Motor1 distance from current position (quad pulses).
       * \param speed2  Motor2 speed in quad pulses per second.
       *                Sign indicates direction.
       * \param dist2   Motor2 distance from current position (quad pulses).
       * \param bQueue  Do [not] queue the command. If false, the motor drive
       *                queue is flushed and any existing command is preempted
       *                by this command.
       *
       * \copydoc doc_return_mc_std
       */
      virtual int cmdQDriveForDist(s32_t speed1, u32_t dist1,
                                   s32_t speed2, u32_t dist2,
                                   bool  bQueue=true);

      /*!
       * \brief Drive a motor at the given speed with the given acceleration
       * for the given distance.
       *
       * Movement is controlled using quadrature encoders.
       *
       * This is a buffered command. Commands execute immediatly if no commands
       * are already in the motor drive queue or if the queue paramter is false.
       * Otherwise, the command is placed at end of the command buffer execution
       * queue.
       *
       * Command Numbers: 44, 45
       *
       * \param motor   Motor index Motor1(0) or Motor2(1).
       * \param speed   Speed in quad pulses per second.
       *                Sign indicates direction.
       * \param accel   Acceleration (qpps/s).
       * \param dist    Distance from current position (quad pulses).
       * \param bQueue  Do [not] queue the command. If false, the motor drive
       *                queue is flushed and any existing command is preempted
       *                by this command.
       *
       * \copydoc doc_return_mc_std
       */
      virtual int cmdQDriveWithAccelForDist(int   motor,
                                            s32_t speed,
                                            u32_t accel,
                                            u32_t dist,
                                            bool  bQueue=true);

      /*!
       * \brief Drive both motors at the given speeds with the given
       * accelerations for the given distances.
       *
       * Movement is controlled using quadrature encoders.
       *
       * This is a buffered command. Commands execute immediatly if no commands
       * are already in the motor drive queue or if the queue paramter is false.
       * Otherwise, the command is placed at end of the command buffer execution
       * queue.
       *
       * Command Numbers: 51
       *
       * \param speed1  Motor1 peed in quad pulses per second.
       *                Sign indicates direction.
       * \param accel1  Motor1 acceleration (qpps/s).
       * \param dist1   Motor1 distance from current position (quad pulses).
       * \param speed2  Motor2 peed in quad pulses per second.
       *                Sign indicates direction.
       * \param accel2  Motor2 acceleration (qpps/s).
       * \param dist2   Motor2 distance from current position (quad pulses).
       * \param bQueue  Do [not] queue the command. If false, the motor drive
       *                queue is flushed and any existing command is preempted
       *                by this command.
       *
       * \copydoc doc_return_mc_std
       */
      virtual int cmdQDriveWithAccelForDist(s32_t speed1,
                                            u32_t accel1,
                                            u32_t dist1,
                                            s32_t speed2,
                                            u32_t accel2,
                                            u32_t dist2,
                                            bool  bQueue=true);

      /*!
       * \brief Drive a motor at the given speed with the given acceleration/
       * decceleration profile to the given absolute position.
       *
       * Movement is controlled using quadrature encoders.
       *
       * This is a buffered command. Commands execute immediatly if no commands
       * are already in the motor drive queue or if the queue paramter is false.
       * Otherwise, the command is placed at end of the command buffer execution
       * queue.
       *
       * Command Numbers: 65, 66
       *
       * \param motor   Motor index Motor1(0) or Motor2(1).
       * \param speed   Speed in quad pulses per second.
       *                Sign indicates direction.
       * \param accel   Acceleration (qpps/s).
       * \param deccel  Decceleration (qpps/s).
       * \param pos     Goal absolute position (quad pulses).
       * \param bQueue  Do [not] queue the command. If false, the motor drive
       *                queue is flushed and any existing command is preempted
       *                by this command.
       *
       * \copydoc doc_return_mc_std
       */
      virtual int cmdQDriveWithProfileToPos(int   motor,
                                            s32_t speed,
                                            u32_t accel,
                                            u32_t deccel,
                                            s32_t pos,
                                            bool  bQueue=true);

      /*!
       * \brief Drive both motors at the given speeds with the given
       * acceleration/decceleration profile to the given absolute position.
       *
       * Movement is controlled using quadrature encoders.
       *
       * This is a buffered command. Commands execute immediatly if no commands
       * are already in the motor drive queue or if the queue paramter is false.
       * Otherwise, the command is placed at end of the command buffer execution
       * queue.
       *
       * Command Numbers: 67
       *
       * \param speed1  Motor1 speed in quad pulses per second.
       *                Sign indicates direction.
       * \param accel1  Motor1 acceleration (qpps/s).
       * \param deccel1 Motor1 decceleration (qpps/s).
       * \param pos1    Motor1 goal absolute position (quad pulses).
       * \param speed2  Motor2 speed in quad pulses per second.
       *                Sign indicates direction.
       * \param accel2  Motor2 acceleration (qpps/s).
       * \param deccel2 Motor2 decceleration (qpps/s).
       * \param pos2    Motor2 goal absolute position (quad pulses).
       * \param bQueue  Do [not] queue the command. If false, the motor drive
       *                queue is flushed and any existing command is preempted
       *                by this command.
       *
       * \copydoc doc_return_mc_std
       */
      virtual int cmdQDriveWithProfileToPos(s32_t speed1,
                                            u32_t accel1,
                                            u32_t deccel1,
                                            s32_t pos1,
                                            s32_t speed2,
                                            u32_t accel2,
                                            u32_t deccel2,
                                            s32_t pos2,
                                            bool  bQueue=true);

      /*!
       * \brief Stop both motors.
       *
       * Movement is controlled using quadrature encoders, with 0 being stop.
       *
       * Command Numbers: 37
       *
       * \copydoc doc_return_mc_std
       */
      virtual int cmdQStop()
      {
        return cmdQDrive2(0, 0);
      }

      /*!
       * \brief Drive a motor at the given speed.
       *
       * Speeds are scaled from [-max, max] with 0 being stop.
       *
       * Command Numbers: 0, 1, 4, 5
       *
       * \param motor   Motor index Motor1(0) or Motor2(1).
       * \param speed   Scaled speed
       *                [MotorSpeedMaxBackward(-127),MotorSpeedMaxForward(127)].
       *                Sign indicates direction.
       *
       * \copydoc doc_return_mc_std
       */
      virtual int cmdSDrive(int motor, int speed);
  
      /*!
       * \brief Drive both motors at the given speeds.
       *
       * Compatibility command. Speeds are scaled from [-max, max] with
       * \< 0, 0, \> 0 being backwards, stop, and forwards, respectively.
       *
       * Command Numbers: 0, 1, 4, 5
       *
       * \param speed1  Motor1 scaled speed
       *                [MotorSpeedMaxBackward(-127),MotorSpeedMaxForward(127)].
       *                Sign indicates direction.
       * \param speed2  Motor2 scaled speed
       *                [MotorSpeedMaxBackward(-127),MotorSpeedMaxForward(127)].
       *                Sign indicates direction.
       *
       * \copydoc doc_return_mc_std
       */
      virtual int cmdSDrive2(int speed1, int speed2);

      /*!
       * \brief Stop a motor.
       *
       * Compatibility command.
       *
       * Command Numbers: 35, 36
       *
       * \param motor   Motor index Motor1(0) or Motor2(1).
       *
       * \copydoc doc_return_mc_std
       */
      virtual int cmdStop(int motor);
  
      /*!
       * \brief Stop a motor with the given maximum decelerations.
       *
       * Stopping is controlled using velocity PID and quadrature encoders.
       *
       * Compatibility command.
       *
       * Command Numbers: 38, 39
       *
       * \param motor   Motor index Motor1(0) or Motor2(1).
       * \param decel   Deceleration (qpps/s).
       *
       * \copydoc doc_return_mc_std
       */
      virtual int cmdStopWithDecel(int motor, u32_t decel);
  
      /*!
       * \brief Stop all motors.
       *
       * Compatibility command.
       *
       * Command Numbers: 37
       *
       * \copydoc doc_return_mc_std
       */
      virtual int cmdStop();
  
      /*!
       * \brief Stop both motors with the given maximum decelerations.
       *
       * Stopping is controlled using velocity PID and quadrature encoders.
       *
       * Compatibility command.
       *
       * Command Numbers: 50
       *
       * \param decel   Deceleration (qpps/s).
       *
       * \copydoc doc_return_mc_std
       */
      virtual int cmdStopWithDecel(u32_t decel);
  
      /*!
       * \brief Emergency stop all motors.
       *
       * Command Numbers: 37
       *
       * \copydoc doc_return_mc_std
       */
      virtual int cmdEStop();

      /*!
       * \brief Write all settings to EEPROM.
       *
       * Command Numbers: 94
       *
       * \copydoc doc_return_mc_std
       */
      virtual int cmdWriteSettingsToEEPROM();

    protected:
      RoboClawComm &m_comm;                 ///< bound communication object
      byte_t        m_address;              ///< assigned controller address
      std::string   m_strNameId;            ///< controller name identifier
      int           m_nMotorDir[NumMotors]; ///< motor rotation direction sense
      s64_t         m_nEncPrev[NumMotors];  ///< encoder prevous values
      s64_t         m_nEncoder[NumMotors];  ///< 64-bit encoder shadow values
  
      /*!
       * \brief Test if motor index is a valid index.
       *
       * \param motor   Motor index Motor1(0) or Motor2(1).
       *
       * \return Returns true or false.
       */
      virtual bool isValidMotor(int motor) const
      {
        return (motor >= Motor1) && (motor < NumMotors);
      }

      /*!
       * \brief Integer absolute value.
       *
       * \param a   Integer value.
       *
       * \return |a|
       */
       static inline int abs(int a)
       {
         return a>=0? a: -a;
       }

       /*!
        * \brief Cap value within limits [min, max].
        *
        * \param a     Value.
        * \param min   Minimum.
        * \param max   Maximum.
        *
        * \return a: min \h_le a \h_le max
        */
       static inline int cap(int a, int min, int max)
       {
         return a<min? min: a>max? max: a;
       }

       /*!
        * \brief Cap FPN value within limits [min, max].
        *
        * \param a     Value.
        * \param min   Minimum.
        * \param max   Maximum.
        *
        * \return a: min \h_le a \h_le max
        */
       static inline double fcap(double a, double min, double max)
       {
         return a<min? min: a>max? max: a;
       }

    };
#endif // SWIG

#ifndef SWIG
  } // roboclaw namespace
} // motor namespace
#endif // SWIG

#endif // _ROBO_CLAW_H
