////////////////////////////////////////////////////////////////////////////////
//
// Package:   CogniBoost
//
// File:      CogniBoost.h
//
/*! \file
 *
 *  $LastChangedDate$
 *  $Rev$
 *
 * \brief CogniBoost Package Common Declarations.
 *
 * The declarations unify common values and types shared between package 
 * components including software and firmware.
 *
 * \author Brent Wilkins  (brent@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 * \author Robin Knight   (robin@roadnarrows.com)
 *
 * \par Copyright
 *   \h_copy 2011-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
 *
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
///////////////////////////////////////////////////////////////////////////////

#ifndef _COGNIBOOST_H
#define _COGNIBOOST_H

#include "rnr/rnrconfig.h"

#include "CogniBoost/CogniMem.h"

#ifndef SWIG
C_DECLS_BEGIN
#endif

// ---------------------------------------------------------------------------
// CogniBoost Product Information
// ---------------------------------------------------------------------------

/*!
 * \ingroup cogniboost
 * \defgroup cb_prod_info CogniBoost Product Information
 *
 * \{
 */
#define CB_ID_STR_LEN_MAX     32                 ///< max length of an id string

#define CB_PROD_MFG           "RoadNarrows LLC" ///< manufacturer
#define CB_PROD_FAMILY        "CogniBoost"      ///< product family name
#define CB_PROD_VAR_STD       ""                ///< standard product variation

// reserved firmware application strings
#define CB_FWAPP_STD          "RNCogniBoostStd" ///< CogniBoost standard
/*! \} */


// ---------------------------------------------------------------------------
// CogniBoost Response Error Codes
// ---------------------------------------------------------------------------

/*!
 * \ingroup cb_syntax man_libcogniboost_ecodes
 * \defgroup cb_syntax_ecodes  CogniBoost Response Error Codes
 *
 * CogniBoost response error codes.
 *
 * \sa man_libcogniboost_ecodes
 *
 * \{
 */
#define CB_OK                       0 ///< not an error, success

#define CB_RSP_ECODE_GEN            1 ///< general, unspecified error
#define CB_RSP_ECODE_BAD_PKT        2 ///< bad packet
#define CB_RSP_ECODE_BAD_CHKSUM     3 ///< bad checksum
#define CB_RSP_ECODE_MSG_SMALL      4 ///< message too short
#define CB_RSP_ECODE_MSG_BIG        5 ///< message too long
#define CB_RSP_ECODE_BAD_MSG        6 ///< bad message
#define CB_RSP_ECODE_OVERFLOW       7 ///< receive overflow
#define CB_RSP_ECODE_ABORT          8 ///< upload aborted
#define CB_RSP_ECODE_CMD_ID_INVAL   9 ///< unknown or invalid command id
#define CB_RSP_ECODE_CMD_INVAL     10 ///< invalid command syntax
#define CB_RSP_ECODE_ARG_INVAL     11 ///< invalid argument sytanx
#define CB_RSP_ECODE_ARG_RANGE     12 ///< argument out of range
#define CB_RSP_ECODE_ARG_CNT       13 ///< wrong number of arguments
#define CB_RSP_ECODE_EXEC          14 ///< execution error
#define CB_RSP_ECODE_PERM          15 ///< operation not permitted
#define CB_RSP_ECODE_NUMOF         16 ///< number of response error codes

#define CB_RSP_ECODE_LAST_RSV      23
    ///< error codes through 23 are reserved for future CogniBoost response use
/*! \} */


// ---------------------------------------------------------------------------
// CogniBoost Message and XML Parameters
// ---------------------------------------------------------------------------

/*!
 * \brief Place double quotes around the constant helper macro.
 * \param x   Constant.
 */
#define QUOTEME_(x) #x

/*!
 * \brief Place double quotes around the expanded macro or constant
 * \param x   Macro or constant.
 */
#define QUOTEME(x) QUOTEME_(x)


/*!
 * \ingroup cb_params
 * \defgroup cb_params_baudrate Baud Rate
 *
 * The CogniBoost USB serial interface can operate at various supported
 * baud rates.
 *
 * \{
 */
#define CB_PARAM_BAUD_RATE_115200   115200                ///< 115.2 kbps
#define CB_PARAM_BAUD_RATE_230400   230400                ///< 230.4 kbps
#define CB_PARAM_BAUD_RATE_500000   500000                ///< 500 kbps
#define CB_PARAM_BAUD_RATE_1000000  1000000               ///< 1 mbps
#define CB_PARAM_BAUD_RATE_2000000  2000000               ///< 2 mbps
#define CB_PARAM_BAUD_RATE_3000000  3000000               ///< 3 mbps
#define CB_PARAM_BAUD_RATE_4000000  4000000               ///< 4 mbps
#define CB_PARAM_BAUD_RATE_DFT      CB_BAUD_RATE_115200   ///< default baud rate
/*! \} */

/*!
 * \ingroup cb_params
 * \defgroup cb_params_autorestore Auto-Restore
 *
 * On reboot,the CogniBoost can construct its working state by auto-loading
 * from the non-volatile memory both the operational and CogniMem parameters
 * and any saved neural network training set.
 *
 * \{
 */

#define CB_PARAM_AUTO_RESTORE_DISABLE false   ///< do not enable auto-restore
#define CB_PARAM_AUTO_RESTORE_ENABLE  true    ///< auto-restore
#define CB_PARAM_AUTO_RESTORE_DFT     CB_PARAM_AUTO_RESTORE_ENABLE
                                              ///< default auto-restore state

#define CB_XML_AUTO_RESTORE_DISABLE   QUOTEME(CB_PARAM_AUTO_RESTORE_DISABLE)
                                        ///< do not enable auto-restore string
#define CB_XML_AUTO_RESTORE_ENABLE    QUOTEME(CB_PARAM_AUTO_RESTORE_ENABLE)
                                        ///< auto-restore string
/*! \} */

/*!
 * \ingroup cb_params
 * \defgroup cb_params_autosleep Auto-Sleep
 *
 * The CogniBoost can automoatically enter power saving sleep mode after
 * <em>sec</em> seconds of idle time. In sleep mode, the LED is turned off, and
 * the CogniMem and \h_mu;Processor are placed in power-saving mode.
 *
 * Any received client message will wake up the CogniBoost, if asleep, and reset
 * the idle timer.
 *
 * \{
 */
#define CB_PARAM_AUTO_SLEEP_NEVER       0     ///< never sleep
#define CB_PARAM_AUTO_SLEEP_SEC_MIN     CB_PARAM_AUTO_SLEEP_NEVER
                                              ///< minimum idle value
#define CB_PARAM_AUTO_SLEEP_SEC_MAX     900   ///< maximum idle value (15 min)
#define CB_PARAM_AUTO_SLEEP_SEC_DFT      10   ///< default auto-sleep value
/*! \} */

/*!
 * \ingroup cb_params
 * \defgroup cb_params_ledbling LED Bling
 *
 * The CogniBoost supports one Red-Green-Blue (RGB) LED. The client can control
 * the color of this LED. Several built-in LED patterns are also supported.
 *
 * \{
 */

#define CB_PARAM_LED_BLING_ID_MIN       CB_PARAM_LED_BLING_FIXED_ID
                                                  ///< minimum bling rule id
#define CB_PARAM_LED_BLING_ID_MAX       CB_PARAM_LED_BLING_FADE_ID
                                                  ///< maximum bling rule id
#define CB_PARAM_LED_BLING_ID_DFT       CB_PARAM_LED_BLING_CM_ID
                                                  ///< default bling rule id
#define CB_PARAM_LED_BLING_PERIOD_DFT   200       ///< default LED period (msec)
#define CB_PARAM_LED_BLING_DWELL_DFT    50        ///< default LED dwell (msec)
#define CB_PARAM_LED_BLING_BOOT_RGB1    0xcc00cc  ///< boot up LED color
#define CB_PARAM_LED_BLING_ERROR_RGB1   0xff0000  ///< fatal error LED color

/*!
 * \ingroup cb_params
 * \defgroup cb_params_ledbling_fixed Fixed LED Bling
 *
 * The fixed LED bling illuminates the LED with the given RGB color. 
 *
 * \par Relevant Parameters:
 *  \termblock
 *  \term period \termdata N/A. \endterm
 *  \term dwell \termdata N/A. \endterm
 *  \term rgb1 \termdata LED color. \endterm
 *  \term rgb2 \termdata N/A. \endterm
 *  \endtermblock
 * \{
 */
#define CB_PARAM_LED_BLING_FIXED_ID   0         ///< fixed bling rule id

#define CB_XML_LED_BLING_FIXED_ID     "fixed"   ///< CM bling id string
/*! \} */

/*!
 * \ingroup cb_params_ledbling
 * \defgroup cb_params_ledbling_cm CogniMem LED Bling
 *
 * The CogniMem LED bling illuminates the LED when a pattern is presented to 
 * the CogniMem.
 *
 * \par Relevant Parameters:
 *  \termblock
 *  \term period \termdata N/A. \endterm
 *  \term dwell
 *    \termdata Illumination time (msec) unless another pattern is presented.
 *  \endterm
 *  \term rgb1 \termdata LED color indicating pattern not recognized. \endterm
 *  \term rgb2 \termdata LED color indicating pattern recognized. \endterm
 *  \endtermblock
 *
 * \{
 */
#define CB_PARAM_LED_BLING_CM_ID        1         ///< CogniMem bling rule id
#define CB_PARAM_LED_BLING_CM_DWELL_DFT 50        ///< default dwell time (msec)
#define CB_PARAM_LED_BLING_CM_RGB1_DFT  0xff8800  ///< dft not recognized color
#define CB_PARAM_LED_BLING_CM_RGB2_DFT  0x00ff88  ///< default recognized color

#define CB_XML_LED_BLING_CM_ID          "cm"      ///< CM bling id string
/*! \} */

/*!
 * \ingroup cb_params_ledbling
 * \defgroup cb_params_ledbling_blink Blink LED Bling
 *
 * The blink LED bling blinks the two colors at the given 1/period frequency.
 * The LED is off for <em>period - 2 * dwell</em> msec.
 *
 * \par Relevant Parameters:
 *  \termblock
 *  \term period \termdata Two color blink period (msec). \endterm
 *  \term dwell \termdata Illumination time (msec) of the current color.\endterm
 *  \term rgb1 \termdata First LED color. \endterm
 *  \term rgb2 \termdata Second LED color. \endterm
 *  \endtermblock
 *
 * \{
 */
#define CB_PARAM_LED_BLING_BLINK_ID         2         ///< blink bling rule id
#define CB_PARAM_LED_BLING_BLINK_PERIOD_DFT 200       ///< default period (msec)
#define CB_PARAM_LED_BLING_BLINK_DWELL_DFT  50        ///< default dwell (msec)
#define CB_PARAM_LED_BLING_BLINK_RGB1_DFT   0xff8800  ///< default 1st color
#define CB_PARAM_LED_BLING_BLINK_RGB2_DFT   0x00ff88  ///< default 2nd color

#define CB_XML_LED_BLING_BLINK_ID           "blink"   ///< blink bling id string
/*! \} */

/*!
 * \ingroup cb_params_ledbling
 * \defgroup cb_params_ledbling_fade Fade LED Bling
 *
 * The fade LED bling fades between the two colors at the given 1/period
 * frequency. The transition fade lasts <em>period - 2 * dwell</em> msec.
 *
 * \par Relevant Parameters:
 *  \termblock
 *  \term period \termdata Two color fade period (msec). \endterm
 *  \term dwell \termdata Illumination time (msec) of the current color.\endterm
 *  \term rgb1 \termdata First LED color. \endterm
 *  \term rgb2 \termdata Second LED color. \endterm
 *  \endtermblock
 *
 * \{
 */
#define CB_PARAM_LED_BLING_FADE_ID          3         ///< blink bling rule id
#define CB_PARAM_LED_BLING_FADE_PERIOD_DFT  300       ///< default period (msec)
#define CB_PARAM_LED_BLING_FADE_DWELL_DFT   50        ///< default dwell (msec)
#define CB_PARAM_LED_BLING_FADE_RGB1_DFT    0xff0000  ///< default 1st color
#define CB_PARAM_LED_BLING_FADE_RGB2_DFT    0x0000ff  ///< default 2nd color

#define CB_XML_LED_BLING_FADE_ID           "fade"     ///< blink bling id string
/*! \} */

/*! \} */

/*!
 * \ingroup cb_params
 * \defgroup cb_params_cm_classifier CogniMem Classifier
 *
 * \{
 */
#define CB_PARAM_CLASSIFIER_RBF   0           ///< radial basis function
#define CB_PARAM_CLASSIFIER_KNN   1           ///< k-nearest neighbors
#define CB_PARAM_CLASSIFIER_DFT   CB_PARAM_CLASSIFIER_RBF
                                              ///< default classifier

#define CB_XML_CLASSIFIER_RBF     "RBF"       ///< xml rbf string
#define CB_XML_CLASSIFIER_KNN     "KNN"       ///< xml knn string
/*! \} */

/*!
 * \ingroup cb_params
 * \defgroup cb_params_cm_norm CogniMem Norm
 *
 * \{
 */
#define CB_PARAM_NORM_L1        0             ///< L<sup>1</sup> norm
#define CB_PARAM_NORM_LSUP      1             ///< L<sup>\h_inf</sup> norm
#define CB_PARAM_NORM_DFT       CB_PARAM_NORM_L1
                                              ///< default norm

#define CB_XML_NORM_L1          "L1"          ///< xml L1 string
#define CB_XML_NORM_LSUP        "Lsup"        ///< xml Lsup string
/*! \} */

/*!
 * \ingroup cb_params
 * \defgroup cb_params_cm_minif CogniMem Minimum Influence Field
 *
 * \{
 */
#define CB_PARAM_MINIF_MIN      CM_REG_MINIF_MIN    ///< minif minimum
#define CB_PARAM_MINIF_MAX      CM_REG_MINIF_MAX    ///< minif maximum
#define CB_PARAM_MINIF_DFT      CM_REG_MINIF_DFT    ///< minif default
/*! \} */

/*!
 * \ingroup cb_params
 * \defgroup cb_params_cm_maxif CogniMem Maximum Influence Field
 *
 * \{
 */
#define CB_PARAM_MAXIF_MIN      CM_REG_MAXIF_MIN    ///< maxif minimum
#define CB_PARAM_MAXIF_MAX      CM_REG_MAXIF_MAX    ///< maxif maximum
#define CB_PARAM_MAXIF_DFT      CM_REG_MAXIF_DFT    ///< maxif default
/*! \} */

/*!
 * \ingroup cb_params
 * \defgroup cb_params_cm_maxclass CogniMem Maximum Recognized Classifications 
 *
 * \{
 */
#define CB_PARAM_MAX_CLASSIFIED_MIN CM_CLASSIFY_CNT_MIN ///< min max classified
#define CB_PARAM_MAX_CLASSIFIED_MAX CM_CLASSIFY_CNT_MAX ///< max max classified
#define CB_PARAM_MAX_CLASSIFIED_DFT 4                   ///< dft max classified
/*! \} */

/*!
 * \ingroup cb_params
 * \defgroup cb_params_nn_label Neural Network Training Set Label
 *
 * \{
 */
#define CB_PARAM_NN_LABEL_LEN_MAX  256  ///< max length of label string
/*! \} */

/*!
 * \ingroup cb_params
 * \defgroup cb_params_mem CogniBoost Bulk Transfer Source/Destination 
 *
 * \{
 */
#define CB_PARAM_MEM_WORKING  0  ///< working memory (volatile)
#define CB_PARAM_MEM_NV       1  ///< non-volatile memory
/*! \} */


#ifndef SWIG
C_DECLS_END
#endif


#endif // _COGNIBOOST_H
