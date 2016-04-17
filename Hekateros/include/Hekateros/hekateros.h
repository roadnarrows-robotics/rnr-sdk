////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// File:      hekateros.h
//
/*! \file
 *
 * $LastChangedDate: 2015-04-17 15:31:34 -0600 (Fri, 17 Apr 2015) $
 * $Rev: 3942 $
 *
 * \brief Top-level package include file.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2011-2015  RoadNarrows
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

#ifndef _HEKATEROS_H
#define _HEKATEROS_H

/*!
 *  \brief The \h_hekateros namespace encapsulates all \h_hekateros related
 *  constructs.
 */
#ifndef SWIG
namespace hekateros
{
#endif // SWIG

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*!
   * \ingroup hek_h
   * \defgroup hek_ecodes  Hekateros Error Codes
   *
   * Hekateros package-wide error codes.
   *
   * \{
   */
  static const int HEK_OK               =  0; ///< not an error, success

  static const int HEK_ECODE_GEN        =  1; ///< general, unspecified error
  static const int HEK_ECODE_SYS        =  2; ///< system (errno) error
  static const int HEK_ECODE_INTERNAL   =  3; ///< internal error (bug)
  static const int HEK_ECODE_BAD_VAL    =  4; ///< bad value general error
  static const int HEK_ECODE_TOO_BIG    =  5; ///< value/list/size too big
  static const int HEK_ECODE_TOO_SMALL  =  6; ///< value/list/size too small
  static const int HEK_ECODE_RANGE      =  7; ///< value out-of-range
  static const int HEK_ECODE_BAD_OP     =  8; ///< invalid operation error
  static const int HEK_ECODE_TIMEDOUT   =  9; ///< operation timed out error
  static const int HEK_ECODE_NO_DEV     = 10; ///< device not found error
  static const int HEK_ECODE_NO_RSRC    = 11; ///< no resource available error
  static const int HEK_ECODE_BUSY       = 12; ///< resource busy error
  static const int HEK_ECODE_NO_EXEC    = 13; ///< cannot execute error
  static const int HEK_ECODE_PERM       = 14; ///< no permissions error
  static const int HEK_ECODE_DYNA       = 15; ///< dynamixel error
  static const int HEK_ECODE_VIDEO      = 16; ///< video error
  static const int HEK_ECODE_FORMAT     = 17; ///< bad format
  static const int HEK_ECODE_BOTSENSE   = 18; ///< botsense error
  static const int HEK_ECODE_NO_FILE    = 19; ///< file not found
  static const int HEK_ECODE_XML        = 20; ///< XML error
  static const int HEK_ECODE_ALARMED    = 21; ///< robot is alarmed
  static const int HEK_ECODE_INTR       = 22; ///< operation interrupted
  static const int HEK_ECODE_COLLISION  = 23; ///< robot link(s) in collision
  static const int HEK_ECODE_ESTOP      = 24; ///< robot emergency stopped

  static const int HEK_ECODE_BADEC      = 25; ///< bad error code

  static const int HEK_ECODE_NUMOF      = 26; ///< number of error codes
  /*! \} */

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*!
   *
   * \ingroup hek_h
   * \defgroup hek_prod  Hekateros Product Identifiers
   *
   * \h_hek product ids, names, and descriptions.
   *
   *  Products ids are classified by class, family, size, dof, and special 
   *  fields.
   *
   * \{
   */

  //
  // Product release status
  //
  #define HEK_PROD_GA       0x000   ///< product general availability
  #define HEK_PROD_BETA     0x001   ///< product beta version
  #define HEK_PROD_ALPHA    0x002   ///< product alpha version
  #define HEK_PROD_PROTO    0x003   ///< product prototype version

  //
  // Product classes
  //
  #define HEK_CLASS_ARM         0xBA000000    ///< base arm
  #define HEK_CLASS_EE          0xEE000000    ///< end effector
  #define HEK_CLASS_EQUIP_DECK  0xED000000    ///< equipment deck effector
  #define HEK_CLASS_AUX         0xAE000000    ///< auxiliary effector

  /*!
   * \brief Convert robotic arm (base) product triplet to product id.
   *
   * \param sizecode  Product size code.
   * \param dof       Degrees of freedom.
   * \param special   Special product subfields. To be defined as needed.
   */
  #define HEK_PRODUCT_ID(sizecode, dof, special) \
    ( HEK_CLASS_ARM | \
     (((sizecode)&0xf)<<20) | (((dof)&0xff)<<12) | ((special)&0xfff) )

  /*!
   * \brief Convert end effector product 4-tuple to product id.
   *
   * \param family    End effector family
   * \param sizecode  Product size code.
   * \param dof       Degrees of freedom.
   * \param special   Special product subfields. To be defined as needed.
   */
  #define HEK_EE_PRODUCT_ID(family, sizecode, dof, special) \
    ( HEK_CLASS_EE | (((family)&0xff)<<16) | \
      (((sizecode)&0xf)<<12) | (((dof)&0xff)<<4) | ((special)&0xf) )

  /*!
   * \brief Convert equipment deck effector product triplet to product id.
   *
   * \param sizecode  Product size code.
   * \param dof       Degrees of freedom.
   * \param special   Special product subfields. To be defined as needed.
   */
  #define HEK_ED_PRODUCT_ID(sizecode, dof, special) \
    ( HEK_CLASS_EQUIP_DECK | \
     (((sizecode)&0xf)<<20) | (((dof)&0xff)<<12) | ((special)&0xfff) )

  /*!
   * \brief Convert auxiliary effector product triplet to product id.
   *
   * \param sizecode  Product size code.
   * \param dof       Degrees of freedom.
   * \param special   Special product subfields. To be defined as needed.
   */
  #define HEK_AE_PRODUCT_ID(sizecode, dof, special) \
    ( HEK_CLASS_AUX | \
     (((sizecode)&0xf)<<20) | (((dof)&0xff)<<12) | ((special)&0xfff) )

  /*!
   * \brief Convert version triplet to integer equivalent.
   *
   * \param major     Major version number.
   * \param minor     Minor version number.
   * \param revision  Revision number.
   */
  #define HEK_VERSION(major, minor, revision) \
    ((((major)&0xff)<<24) | (((minor)&0xff)<<16) | ((revision)&0xffff))

  /*!
   * \brief Get version major number from version.
   *
   * \param ver       Version number.
   *
   * \return Major number.
   */
  #define HEK_VER_MAJOR(ver)    (((ver)>>24) &0xff)

  /*!
   * \brief Get version minor number from version.
   *
   * \param ver       Version number.
   *
   * \return Minor number.
   */
  #define HEK_VER_MINOR(ver)    (((ver)>>16) &0xff)

  /*!
   * \brief Get revision number from version.
   *
   * \param ver       Version number.
   *
   * \return Revision number.
   */
  #define HEK_VER_REV(ver)    ((ver) & 0xffff)


  const char* const HekProdFamily = "Hekateros";  ///< product family name

  const int HekProdFamilyUnknown  = 0;  ///< unknown/undefined product family
  const int HekProdIdUnknown      = 0;  ///< unknown/undefined product id

  /*!
   * \brief \h_hek product size codes.
   *
   * Sizes are relavent to the product family. For example, a short base is
   * different from a short end effector.
   */
  enum HekProdSize
  {
    HekProdSizeUnknown  = 0,    ///< unknown size
    HekProdSizeStd      = 1,    ///< standard (normal) size
    HekProdSizeShort    = 2,    ///< short
    HekProdSizeLong     = 3,    ///< long
    HekProdSizeMini     = 4,    ///< mini

    HekProdSizeNumOf    = 5     ///< number of
  };

  /*!
   * \brief Common \h_hek product size codes as strings.
   */
  const char* const HekProdSizeStrUnknown = "?";    ///< unknown base size
  const char* const HekProdSizeStrStd     = "N";    ///< standard (normal) size
  const char* const HekProdSizeStrShort   = "S";    ///< short
  const char* const HekProdSizeStrLong    = "L";    ///< long
  const char* const HekProdSizeStrMini    = "Mini"; ///< mini
  /*! \} */

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*!
   * \ingroup hek_spec
   *
   * \defgroup hek_common_spec  Hekateros Common Specs
   *
   * \h_hek product family specifications.
   *
   * \{
   */
  // base servo ids
  static const int HekServoIdBase       = 1;  ///< continuous rotating base
  static const int HekServoIdShoulderL  = 2;  ///< left shoulder
  static const int HekServoIdShoulderR  = 3;  ///< right shoulder (linked slave)
  static const int HekServoIdElbow      = 4;  ///< elbow
  static const int HekServoIdWristPitch = 5;  ///< wrist pitch
  static const int HekServoIdWristRot   = 6;  ///< wrist continuous rotation
  static const int HekServoIdRsrv1      = 7;  ///< reserved for future bases
  static const int HekServoIdRsrv2      = 8;  ///< reserved for future bases
  static const int HekServoIdRsrv3      = 9;  ///< reserved for future bases

  // end effector servo ids
  static const int HekServoIdEEStart    = 10; ///< start id
  static const int HekServoIdGraboid    = 10; ///< graboid

  // equipment deck effector servo ids
  static const int HekServoIdEquipStart = 20; ///< start id
  static const int HekServoIdEquipPan   = 20; ///< pan
  static const int HekServoIdEquipTilt  = 21; ///< tilt

  // base auxiliary effector servo ids
  static const int HekServoIdAuxStart   = 30; ///< start id
  static const int HekServoIdAuxPan     = 30; ///< pan
  static const int HekServoIdAuxTilt    = 31; ///< tilt
  /*! \} */

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*!
   *
   * \ingroup hek_h
   * \defgroup hek_env  Hekateros Environment
   *
   * Hekateros directories and files.
   *
   * \{
   */

  //
  // Images directory.
  //
  #ifdef HEK_IMAGE_DIR
  const char* const HekImageDir = HEK_IMAGE_DIR;  ///< image directory
  #else
  const char* const HekImageDir = "/usr/local/share/Hekateros/images";
                                                  ///< image directory
  #endif

  //
  // Icons directory.
  //
  #ifdef HEK_ICON_DIR
  const char* const HekIconDir = HEK_ICON_DIR;    ///< icon directory
  #else
  const char* const HekIconDir = "/usr/local/share/Hekateros/images/icons";
                                                  ///< icon directory
  #endif

  //
  // Paths
  //
 
  /*! \brief System configuration search path. */
  const char* const HekSysCfgPath  = "/etc/hekateros:/etc";

  /*! \brief User configuration search path and inheritance order. */
  const char* const HekUserCfgPath = "/etc/hekateros:~/.roadnarrows";

  //
  // Top-level configuration XML file basename.
  //
  #ifdef HEK_ETC_CFG
  const char* const HekEtcCfg = HEK_ETC_CFG;      ///< xml configuration file
  #else
  const char* const HekEtcCfg = "hekateros.conf";
                                                  ///< xml configuration file
  #endif

  //
  // Tuning XML file basename.
  //
  #ifdef HEK_ETC_TUNE
  const char* const HekEtcTune = HEK_ETC_TUNE;    ///< xml tune file
  #else
  const char* const HekEtcTune = "hek_tune.conf";
                                                  ///< xml tune file
  #endif

  //
  // Specific Hekateros application configuration file basenames.
  //
  const char* const HekPanelXml = "hek_panel.xml";  ///< hek_panel configuration
  const char* const HekEECamXml = "hek_eecam.xml";  ///< end effector camera cfg
  const char* const HekXboxXml  = "hek_xbox.xml";   ///< Xbox teleop config

  //
  // XML stylesheet URL
  //
  #ifdef HEK_XSL_URL
  const char* const HekXslUrl = HEK_XSL_URL;      ///< xml stylesheet url
  #else
  const char* const HekXslUrl =
    "http://roadnarrows.com/xml/Hekateros/1.0/hekateros.xsl";
                                                  ///< xml stylesheet url
  #endif

  //
  // XML schema instance URL
  //
  #ifdef HEK_XSI_URL
  const char* const HekXsiUrl = HEK_XSI_URL;      ///< xml schema instance url
  #else
  const char* const HekXsiUrl =
    "http://roadnarrows.com/xml/Hekateros/1.0/hekateros.xsd";
                                                  ///< xml schema instance url
  #endif

  //
  // I2C default device name (deprecated)
  //
  #ifdef HEK_DEV_I2C
  const char* const HekI2CDevice = HEK_DEV_I2C;   ///< i2c device name
  #else
  const char* const HekI2CDevice = "/dev/i2c-3";  ///< i2c device name
  #endif

  //
  // Alarm LED GPIO number (deprecated)
  //
  // Direction: Output
  //
  #ifdef HEK_GPIO_ALARM
  const int HekGpioAlarmLED = HEK_GPIO_ALARM; ///< alarm led gpio number
  #else
  const int HekGpioAlarmLED = 10;             ///< alarm led gpio number
  #endif

  //
  // Halt GPIO number.
  //
  // Direction: Input
  //
  // Signal received from the monitor subprocessor to halt the system.
  //
  #ifdef HEK_GPIO_HALT
  const int HekGpioHalt = HEK_ALARM_HALT;     ///< halt gpio number
  #else
  const int HekGpioHalt = 174;                ///< halt gpio number
  #endif

  //
  // Halted GPIO number.
  //
  // Direction: Output
  //
  // Signal sent to the monitor subprocessor to indicate main processor is in
  // the safe-to-power-off state.
  //
  #ifdef HEK_GPIO_HALTED
  const int HekGpioHalted = HEK_ALARM_HALTED; ///< halt gpio number
  #else
  const int HekGpioHalted = 170;              ///< halt gpio number
  #endif

  //
  // Dynabus USB serial default device name.
  //
  #ifdef HEK_DEV_DYNABUS
  const char* const HekDevDynabus = HEK_DEV_DYNABUS;  ///< dynabus device name
  #else
  const char* const HekDevDynabus = "/dev/dynabus";   ///< dynabus device name
  #endif

  //
  // Dynabus default baud rate.
  //
  #ifdef HEK_BAUDRATE_DYNABUS
  const int HekBaudRateDynabus = HEK_BAUDRATE_DYNABUS;  ///< dynabus baudrate
  #else
  const int HekBaudRateDynabus = 1000000;               ///< dynabus baudrate
  #endif

  //
  // Arduino subprocessor USB serial default device name.
  //
  #ifdef HEK_DEV_ARDUINO
  const char* const HekDevArduino = HEK_DEV_ARDUINO;  ///< arduino device name
  #else
  const char* const HekDevArduino = "/dev/arduino";   ///< arduino device name
  #endif

  //
  // Arduino default baud rate.
  //
  #ifdef HEK_BAUDRATE_ARDUINO
  const int HekBaudRateArduino = HEK_BAUDRATE_ARDUINO;  ///< arduino baudrate
  #else
  const int HekBaudRateArduino = 115200;                ///< arduino baudrate
  #endif

  /*! \} */

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*!
   *
   * \ingroup hek_h
   * \defgroup hek_types  Hekateros Common Simple Types
   *
   *  Common types used to control and report robot information.
   *  fields.
   *
   * \{
   */

  /*!
   * \brief \h_hek tri-state type.
   *
   * Basically, a tri-state value is either unknown, 0, or 1.
   */
  enum HekTriState
  {
    // unknown
    HekTriStateUnknown  = -1,     ///< unknown state

    // low state synonyms
    HekTriStateFalse    = 0,      ///< false
    HekTriStateOff      = 0,      ///< off
    HekTriStateDisabled = 0,      ///< disabled
    HekTriStateLow      = 0,      ///< low
    HekTriStateOpen     = 0,      ///< open
    HekTriStateDark     = 0,      ///< dark

    // high state synonyms
    HekTriStateTrue     = 1,      ///< true
    HekTriStateOn       = 1,      ///< on
    HekTriStateEnabled  = 1,      ///< enabled
    HekTriStateHigh     = 1,      ///< high
    HekTriStateClosed   = 1,      ///< closed
    HekTriStateLight    = 1       ///< light
  };

  /*!
   * \brief \h_hek mode of operation.
   *
   * Robot mode of operation.
   */
  enum HekRobotMode
  {
    HekRobotModeUnknown = -1,   ///< unknown mode state
    HekRobotModeManual  =  1,   ///< can only be operated locally, not remotely
    HekRobotModeAuto    =  2    ///< fully available
  };

  /*!
   * \brief Robot or joint operational states.
   */
  enum HekOpState
  {
    HekOpStateUncalibrated  = 0,    ///< uncalibrated
    HekOpStateCalibrating   = 1,    ///< calibrating
    HekOpStateCalibrated    = 2     ///< calibrated
  };

  /*!
   * \brief Asynchronous task state.
   */
  enum HekAsyncTaskState
  {
    HekAsyncTaskStateIdle     = 0,    ///< idle, no async task running
    HekAsyncTaskStateWorking  = 1     ///< async task running
  };

  /*!
   * \brief Length/Distance Norm
   */
  enum HekNorm
  {
    HekNormL1   = 1,    ///< L1 norm (taxicab or manhattan norm)
    HekNormL2   = 2,    ///< L2 norm (Euclidean norm)
    HekNormLinf = 3     ///< Linf norm (maximum, infinity, or supremum norm)
  };

  /*! \} */

#ifndef SWIG
} // namespace hekateros
#endif // SWIG


#endif // _HEKATEROS_H
