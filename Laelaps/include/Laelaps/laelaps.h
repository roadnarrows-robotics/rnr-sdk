////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// File:      laelaps.h
//
/*! \file
 *
 * $LastChangedDate: 2016-01-21 16:50:25 -0700 (Thu, 21 Jan 2016) $
 * $Rev: 4268 $
 *
 * \brief Top-level package include file.
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

#ifndef _LAELAPS_H
#define _LAELAPS_H

#undef LAE_DEPRECATED   ///< use to delete old code

/*!
 *  \brief The \h_laelaps namespace encapsulates all \h_laelaps related
 *  constructs.
 */
#ifndef SWIG
namespace laelaps
{
#endif // SWIG

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*!
   * \ingroup laelaps_h
   * \defgroup lae_ecodes  Laelaps Error Codes
   *
   * Laelaps package-wide error codes.
   *
   * \{
   */
  static const int LAE_OK               =  0; ///< not an error, success

  static const int LAE_ECODE_GEN        =  1; ///< general, unspecified error
  static const int LAE_ECODE_SYS        =  2; ///< system (errno) error
  static const int LAE_ECODE_INTERNAL   =  3; ///< internal error (bug)
  static const int LAE_ECODE_BAD_VAL    =  4; ///< bad value general error
  static const int LAE_ECODE_TOO_BIG    =  5; ///< value/list/size too big
  static const int LAE_ECODE_TOO_SMALL  =  6; ///< value/list/size too small
  static const int LAE_ECODE_RANGE      =  7; ///< value out-of-range
  static const int LAE_ECODE_BAD_OP     =  8; ///< invalid operation error
  static const int LAE_ECODE_TIMEDOUT   =  9; ///< operation timed out error
  static const int LAE_ECODE_NO_DEV     = 10; ///< device not found error
  static const int LAE_ECODE_NO_RSRC    = 11; ///< no resource available error
  static const int LAE_ECODE_BUSY       = 12; ///< resource busy error
  static const int LAE_ECODE_NO_EXEC    = 13; ///< cannot execute error
  static const int LAE_ECODE_PERM       = 14; ///< no permissions error
  static const int LAE_ECODE_DYNA       = 15; ///< dynamixel error
  static const int LAE_ECODE_VIDEO      = 16; ///< video error
  static const int LAE_ECODE_FORMAT     = 17; ///< bad format
  static const int LAE_ECODE_BOTSENSE   = 18; ///< botsense error
  static const int LAE_ECODE_NO_FILE    = 19; ///< file not found
  static const int LAE_ECODE_XML        = 20; ///< XML error
  static const int LAE_ECODE_ALARMED    = 21; ///< robot is alarmed
  static const int LAE_ECODE_INTR       = 22; ///< operation interrupted
  static const int LAE_ECODE_COLLISION  = 23; ///< robot link(s) in collision
  static const int LAE_ECODE_ESTOP      = 24; ///< robot emergency stopped
  static const int LAE_ECODE_MOT_CTLR   = 25; ///< motor controller error
  static const int LAE_ECODE_IO         = 26; ///< I/O error

  static const int LAE_ECODE_BADEC      = 27; ///< bad error code

  static const int LAE_ECODE_NUMOF      = 28; ///< number of error codes
  /*! \} */

  /*! \} */

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*!
   *
   * \ingroup laelaps_h
   * \defgroup lae_prod  Laelaps Product Identifiers
   *
   * \h_laelaps product ids, names, and descriptions.
   *
   *  Products ids are classified by class, family, size, dof, and special 
   *  fields.
   *
   * \{
   */

  //
  // Product release status
  //
  #define LAE_PROD_GA       0x000   ///< product general availability
  #define LAE_PROD_BETA     0x001   ///< product beta version
  #define LAE_PROD_ALPHA    0x002   ///< product alpha version
  #define LAE_PROD_PROTO    0x003   ///< product prototype version

  //
  // Product classes
  //
  #define LAE_CLASS_MOBILE_BASE 0xBA000000    ///< base

  //
  // Product Ids and Names
  //
  // product family
  const char* const LaeProdFamilyUnknown  = "?"; ///< unknown product family
  const char* const LaeProdFamily         = "Laelaps"; ///< product family name

  // product ids
  static const int  LaeProdIdUnknown  = 0;  ///< unknown/undefined product id
  static const int  LaeProdIdStd      = 1;  ///< standard Laelaps product id
  static const int  LaeProdIdLarge    = 2;  ///< large Laelaps product id

  // product sizes
  const char* const LaeProdModelStd   = "Standard"; ///< standard model
  const char* const LaeProdModelLarge = "Large";    ///< future large model

  /*!
   * \brief Convert version triplet to integer equivalent.
   *
   * \param major     Major version number.
   * \param minor     Minor version number.
   * \param revision  Revision number.
   */
  #define LAE_VERSION(major, minor, revision) \
    ((((major)&0xff)<<24) | (((minor)&0xff)<<16) | ((revision)&0xffff))

  /*!
   * \brief Get version major number from version.
   *
   * \param ver       Version number.
   *
   * \return Major number.
   */
  #define LAE_VER_MAJOR(ver)    (((ver)>>24) & 0xff)

  /*!
   * \brief Get version minor number from version.
   *
   * \param ver       Version number.
   *
   * \return Minor number.
   */
  #define LAE_VER_MINOR(ver)    (((ver)>>16) & 0xff)

  /*!
   * \brief Get revision number from version.
   *
   * \param ver       Version number.
   *
   * \return Revision number.
   */
  #define LAE_VER_REV(ver)    ((ver) & 0xffff)
  /*! \} */

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*!
   *
   * \ingroup laelaps_h
   * \defgroup lae_env  Laelaps Environment
   *
   * Laelaps directories and files.
   *
   * \{
   */

  //
  // Images directory.
  //
  #ifdef LAE_IMAGE_DIR
  const char* const LaeImageDir = LAE_IMAGE_DIR;  ///< image directory
  #else
  const char* const LaeImageDir = "/usr/local/share/Laelaps/images";
                                                  ///< image directory
  #endif

  //
  // Icons directory.
  //
  #ifdef LAE_ICON_DIR
  const char* const LaeIconDir = LAE_ICON_DIR;    ///< icon directory
  #else
  const char* const LaeIconDir = "/usr/local/share/Laelaps/images/icons";
                                                  ///< icon directory
  #endif

  //
  // Paths
  //
 
    /*! \brief System configuration search path. */
  const char* const LaeSysCfgPath  = "/etc/laelaps:/etc";

  /*! \brief User configuration search path and inheritance order. */
  const char* const LaeUserCfgPath = "/etc/laelaps:~/.roadnarrows";

  //
  // Top-level configuration XML file basename.
  //
  #ifdef LAE_ETC_CFG
  const char* const LaeEtcCfg = LAE_ETC_CFG;      ///< xml configuration file
  #else
  const char* const LaeEtcCfg = "laelaps.conf";
                                                  ///< xml configuration file
  #endif

  //
  // Tuning XML file basename.
  //
  #ifdef LAE_ETC_TUNE
  const char* const LaeEtcTune = LAE_ETC_TUNE;    ///< xml tune file
  #else
  const char* const LaeEtcTune = "laelaps_tune.conf";
                                                  ///< xml tune file
  #endif

  //
  // Specific Laelaps application configuration file basenames.
  //
  const char* const LaePanelXml = "laelaps_panel.xml";    ///< control panel cfg
  const char* const LaeFrontCamXml = "laelaps_fcam.xml";  ///< front camera cfg
  const char* const LaeXboxXml  = "laelaps_xbox.xml";     ///< xbox teleop cfg

  //
  // XML stylesheet URL
  //
  #ifdef LAE_XSL_URL
  const char* const LaeXslUrl = LAE_XSL_URL;      ///< xml stylesheet url
  #else
  const char* const LaeXslUrl =
    "http://roadnarrows.com/xml/Laelaps/1.0/laelaps.xsl";
                                                  ///< xml stylesheet url
  #endif

  //
  // XML schema instance URL
  //
  #ifdef LAE_XSI_URL
  const char* const LaeXsiUrl = LAE_XSI_URL;      ///< xml schema instance url
  #else
  const char* const LaeXsiUrl =
    "http://roadnarrows.com/xml/Laelaps/1.0/laelaps.xsd";
                                                  ///< xml schema instance url
  #endif

  /*! \} */

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*!
   *
   * \ingroup laelaps_h
   * \defgroup lae_types  Laelaps Common Simple Types
   *
   *  Common types used to control and report robot information.
   *  fields.
   *
   * \{
   */

  /*!
   * \brief \h_laelaps tri-state type.
   *
   * Basically, a tri-state value is either unknown, 0, or 1.
   */
  enum LaeTriState
  {
    // unknown
    LaeTriStateUnknown  = -1,     ///< unknown state

    // low state synonyms
    LaeTriStateFalse    = 0,      ///< false
    LaeTriStateOff      = 0,      ///< off
    LaeTriStateDisabled = 0,      ///< disabled
    LaeTriStateLow      = 0,      ///< low
    LaeTriStateOpen     = 0,      ///< open
    LaeTriStateDark     = 0,      ///< dark

    // high state synonyms
    LaeTriStateTrue     = 1,      ///< true
    LaeTriStateOn       = 1,      ///< on
    LaeTriStateEnabled  = 1,      ///< enabled
    LaeTriStateHigh     = 1,      ///< high
    LaeTriStateClosed   = 1,      ///< closed
    LaeTriStateLight    = 1       ///< light
  };

  /*!
   * \brief \h_laelaps mode of operation.
   *
   * Robot mode of operation.
   */
  enum LaeRobotMode
  {
    LaeRobotModeUnknown = -1,   ///< unknown mode state
    LaeRobotModeManual  =  1,   ///< can only be operated locally, not remotely
    LaeRobotModeAuto    =  2    ///< fully available
  };

  /*!
   * \brief Robot or joint operational states.
   */
  enum LaeOpState
  {
    LaeOpStateUncalibrated  = 0,    ///< uncalibrated
    LaeOpStateCalibrating   = 1,    ///< calibrating
    LaeOpStateCalibrated    = 2     ///< calibrated
  };

  /*!
   * \brief Asynchronous task state.
   */
  enum LaeAsyncTaskState
  {
    LaeAsyncTaskStateIdle     = 0,    ///< idle, no async task running
    LaeAsyncTaskStateWorking  = 1     ///< async task running
  };

  /*!
   * \brief Length/Distance Norm
   */
  enum LaeNorm
  {
    LaeNormL1   = 1,    ///< L1 norm (taxicab or manhattan norm)
    LaeNormL2   = 2,    ///< L2 norm (Euclidean norm)
    LaeNormLinf = 3     ///< Linf norm (maximum, infinity, or supremum norm)
  };

  /*! \} */

#ifndef SWIG
} // namespace laelaps
#endif // SWIG


#endif // _LAELAPS_H
