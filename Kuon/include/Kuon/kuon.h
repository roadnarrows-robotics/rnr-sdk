////////////////////////////////////////////////////////////////////////////////
//
// Package:   kuon
//
// Library:   libkuon
//
// File:      kuon.h
//
/*! \file
 *
 * $LastChangedDate: 2011-01-13 11:05:37 -0700 (Thu, 13 Jan 2011) $
 * $Rev: 657 $
 *
 * \brief RoadNarrows Kuon robot top-level header file.
 *
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 * \author Rob Shiely     (rob@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2010-2016.  RoadNarrows LLC.
 * (http://www.roadnarrows.com)
 * \n All Rights Reserved
 */
/*
 * @EulaBegin@
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
 * @EulaBegin@
 */
////////////////////////////////////////////////////////////////////////////////

#ifndef _KUON_H
#define _KUON_H

#include "rnr/rnrconfig.h"
#include "rnr/units.h"
#include "rnr/log.h"

/*!
 *  \brief The \h_kuon namespace encapsulates all \h_kuon related
 *  constructs.
 */
#ifndef SWIG
namespace kuon
{
#endif // SWIG

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*!
   * \ingroup kuon_h
   * \defgroup kuon_ecodes  Kuon Error Codes
   *
   * Kuon package-wide error codes.
   *
   * \{
   */
  static const int KUON_OK              =  0; ///< not an error, success

  static const int KUON_ECODE_GEN       =  1; ///< general, unspecified error
  static const int KUON_ECODE_SYS       =  2; ///< system (errno) error
  static const int KUON_ECODE_INTERNAL  =  3; ///< internal error (bug)
  static const int KUON_ECODE_BAD_VAL   =  4; ///< bad value general error
  static const int KUON_ECODE_TOO_BIG   =  5; ///< value/list/size too big
  static const int KUON_ECODE_TOO_SMALL =  6; ///< value/list/size too small
  static const int KUON_ECODE_RANGE     =  7; ///< value out-of-range
  static const int KUON_ECODE_BAD_OP    =  8; ///< invalid operation error
  static const int KUON_ECODE_TIMEDOUT  =  9; ///< operation timed out error
  static const int KUON_ECODE_NO_DEV    = 10; ///< device not found error
  static const int KUON_ECODE_NO_RSRC   = 11; ///< no resource available error
  static const int KUON_ECODE_BUSY      = 12; ///< resource busy error
  static const int KUON_ECODE_NO_EXEC   = 13; ///< cannot execute error
  static const int KUON_ECODE_PERM      = 14; ///< no permissions error
  static const int KUON_ECODE_MOTOR     = 15; ///< motor error
  static const int KUON_ECODE_MOT_CTLR  = 16; ///< motor controller error
  static const int KUON_ECODE_BATT      = 17; ///< battery error
  static const int KUON_ECODE_FORMAT    = 18; ///< bad format
  static const int KUON_ECODE_BOTSENSE  = 19; ///< botsense error
  static const int KUON_ECODE_NO_FILE   = 20; ///< file not found
  static const int KUON_ECODE_XML       = 21; ///< XML error
  static const int KUON_ECODE_ALARMED   = 22; ///< robot is alarmed
  static const int KUON_ECODE_INTR      = 23; ///< operation interrupted
  static const int KUON_ECODE_COLLISION = 24; ///< robot link(s) in collision
  static const int KUON_ECODE_ESTOP     = 25; ///< robot emergency stopped

  static const int KUON_ECODE_BADEC     = 26; ///< bad error code

  static const int KUON_ECODE_NUMOF     = 27; ///< number of error codes
  /*! \} */

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*!
   * \ingroup kuon_h
   * \defgroup kuon_alarms  Kuon Alarm Bits and Masks
   * \{
   */
  static const int KUON_ALARM_NONE      =  0x00;  ///< no alarms
  static const int KUON_ALARM_VOLTAGE   =  0x01;  ///< under/over voltage
  static const int KUON_ALARM_TEMP      =  0x02;  ///< under/over temperature
  static const int KUON_ALARM_COMM      =  0x04;  ///< communication

  /*! \} */

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*!
   *
   * \ingroup kuon_h
   * \defgroup kuon_prod  Kuon Product Identifiers
   *
   * \h_kuon product ids, names, and descriptions.
   *
   * \{
   */

  //
  // Product Ids and Names
  //
  const char* const KuonProdFamily      = "Kuon";     ///< product family name

  static const int  KuonProdIdUnknown   = 0;  ///< unknown/undefined product id
  static const int  KuonProdIdStd       = 1;  ///< standard Kuon product id
  static const int  KuonProdIdNarrow    = 2;  ///< narrow Kuon product id

  const char* const KuonProdModelStd    = "Standard"; ///< standard model
  const char* const KuonProdModelNarrow = "Narrow";   ///< narrow model

  /*!
   * \brief Convert version triplet to integer equivalent.
   *
   * \param major     Major version number.
   * \param minor     Minor version number.
   * \param revision  Revision number.
   */
  #define KUON_VERSION(major, minor, revision) \
    ((((major)&0xff)<<24) | (((minor)&0xff)<<16) | ((revision)&0xffff))

  /*!
   * \brief Get version major number from version.
   *
   * \param ver       Version number.
   *
   * \return Major number.
   */
  #define KUON_VER_MAJOR(ver)   (((ver)>>24) &0xff)

  /*!
   * \brief Get version minor number from version.
   *
   * \param ver       Version number.
   *
   * \return Minor number.
   */
  #define KUON_VER_MINOR(ver)   (((ver)>>16) &0xff)

  /*!
   * \brief Get revision number from version.
   *
   * \param ver       Version number.
   *
   * \return Revision number.
   */
  #define KUON_VER_REV(ver)     ((ver) & 0xffff)
  /*! \} */

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*!
   *
   * \ingroup kuon_h
   * \defgroup kuon_env  Kuon Environment
   *
   * Kuon directories and files.
   *
   * \{
   */

  //
  // Images directory.
  //
  #ifdef KUON_IMAGE_DIR
  const char* const KuonImageDir = KUON_IMAGE_DIR;  ///< image directory
  #else
  const char* const KuonImageDir = "/usr/local/share/Kuon/images";
                                                    ///< image directory
  #endif

  //
  // Icons directory.
  //
  #ifdef KUON_ICON_DIR
  const char* const KuonIconDir = KUON_ICON_DIR;    ///< icon directory
  #else
  const char* const KuonIconDir = "/usr/local/share/Kuon/images/icons";
                                                    ///< icon directory
  #endif

  //
  // Top-level configuration file.
  //
  #ifdef KUON_ETC_CFG
  const char* const KuonEtcCfg = KUON_ETC_CFG;      /// xml configuration file
  #else
  const char* const KuonEtcCfg = "/etc/kuon.conf";
                                                    /// xml configuration file
  #endif

  //
  // XML stylesheet URL
  //
  #ifdef KUON_XSL_URL
  const char* const KuonXslUrl = KUON_XSL_URL;      ///< xml stylesheet url
  #else
  const char* const KuonXslUrl =
    "http://roadnarrows.com/xml/Kuon/1.0/kuon.xsl";
                                                    ///< xml stylesheet url
  #endif

  //
  // XML schema instance URL
  //
  #ifdef KUON_XSI_URL
  const char* const KuonXsiUrl = KUON_XSI_URL;      ///< xml schema instance url
  #else
  const char* const KuonXsiUrl =
    "http://roadnarrows.com/xml/Kuon/1.0/kuon.xsd";
                                                    ///< xml schema instance url
  #endif

  //
  // Motor controller 0 USB serial default device name.
  //
  #ifdef KUON_DEV_MOTOR_CTLR_0
  const char* const KuonDevMotorCtlr0 = KUON_DEV_MOTOR_CTLR_0;
                                            ///< motor controller 0 device name
  #else
  const char* const KuonDevMotorCtlr0 = "/dev/kmot0";
                                            ///< motor controller 0 device name
  #endif

  //
  // Motor controller 1 USB serial default device name.
  //
  #ifdef KUON_DEV_MOTOR_CTLR_1
  const char* const KuonDevMotorCtlr1 = KUON_DEV_MOTOR_CTLR_1;
                                            ///< motor controller 1 device name
  #else
  const char* const KuonDevMotorCtlr1 = "/dev/kmot1";
                                            ///< motor controller 1 device name
  #endif

  //
  // Motor controllers default baud rate.
  //
  #ifdef KUON_BAUDRATE_MOTOR_CTLR
  const int KuonBaudRateMotorCtlr = KUON_BAUDRATE_MOTOR_CTLR;
                                            ///< motor controllers baud rate
  #else
  const int KuonBaudRateMotorCtlr = 38400;
                                            ///< motor controllers baud rate
  #endif

  /*! \} */

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*!
   * \ingroup kuon_spec
   *
   * \defgroup kuon_common_spec  Kuon Common Specs
   *
   * \h_kuon common specifications.
   *
   * \{
   */

  // motor/wheel ids
  static const int KuonMotorIdNone    = -1; ///< no motor id
  static const int KuonMotorIdLF      = 0;  ///< left front
  static const int KuonMotorIdRF      = 1;  ///< right front
  static const int KuonMotorIdLR      = 2;  ///< left rear
  static const int KuonMotorIdRR      = 3;  ///< right rear
 
  // motor controllers
  static const int KuonMotorCtlrIdNone  = -1; ///< no motor controller id
  static const int KuonMotorCtlrId0     = 0;  ///< motor controller id 0
  static const int KuonMotorCtlrId1     = 1;  ///< motor controller id 1

  // motor rotation
  static const int KuonMotorDirUnknown  = 0;  ///< unknown
  static const int KuonMotorDirCcw      = 1;  ///< counter-clockwise
  static const int KuonMotorDirCw       = -1; ///< clockwise
  /*! \} */


  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*!
   *
   * \ingroup kuon_h
   * \defgroup kuon_types  Kuon Common Simple Types
   *
   *  Common types used to control and report robot information.
   *  fields.
   *
   * \{
   */

  /*!
   * \brief \h_kuon tri-state type.
   *
   * Basically, a tri-state value is either unknown, 0, or 1.
   */
  enum KuonTriState
  {
    // unknown
    KuonTriStateUnknown   = -1,     ///< unknown state

    // low state synonyms
    KuonTriStateFalse     = 0,      ///< false
    KuonTriStateOff       = 0,      ///< off
    KuonTriStateDisabled  = 0,      ///< disabled
    KuonTriStateLow       = 0,      ///< low
    KuonTriStateOpen      = 0,      ///< open
    KuonTriStateDark      = 0,      ///< dark

    // high state synonyms
    KuonTriStateTrue      = 1,      ///< true
    KuonTriStateOn        = 1,      ///< on
    KuonTriStateEnabled   = 1,      ///< enabled
    KuonTriStateHigh      = 1,      ///< high
    KuonTriStateClosed    = 1,      ///< closed
    KuonTriStateLight     = 1       ///< light
  };

  /*!
   * \brief \h_kuon mode of operation.
   *
   * Robot mode of operation.
   */
  enum KuonRobotMode
  {
    KuonRobotModeUnknown  = -1, ///< unknown mode state
    KuonRobotModeManual   =  1, ///< can only be operated locally, not remotely
    KuonRobotModeAuto     =  2  ///< fully available
  };

  /*!
   * \brief Asynchronous task state.
   */
  enum KuonAsyncTaskState
  {
    KuonAsyncTaskStateIdle    = 0,    ///< idle, no async task running
    KuonAsyncTaskStateWorking = 1     ///< async task running
  };

  /*! \} */


#ifndef SWIG
} // namespace kuon
#endif // SWIG


#endif // _KUON_H
