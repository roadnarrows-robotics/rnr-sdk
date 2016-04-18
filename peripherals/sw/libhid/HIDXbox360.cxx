////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Peripherals
//
// Library:   libhid
//
// File:      HIDXbox360.cxx
//
/*! \file
 *
 * $LastChangedDate: 2014-03-05 16:20:41 -0700 (Wed, 05 Mar 2014) $
 * $Rev: 3591 $
 *
 * \brief Xbox360 Controller C implementation.
 *
 * \author Rob Shiely (rob.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2012-2014.  RoadNarrows LLC.
 * (http://www.roadnarrows.com)
 * \n All Rights Reserved
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


#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>

#include <limits>
#include <string>
#include <map>

#include <libusb-1.0/libusb.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/usbext.h"
#include "rnr/hid/HIDXbox360.h"

using namespace std;
using namespace rnr;


// -----------------------------------------------------------------------------
// Public Interface
// -----------------------------------------------------------------------------

HIDXbox360::HIDXbox360(int usbDebugLevel) : HIDInput(HIDClassXbox360)
{ 
  int   i;

  //
  // HID state
  //
  m_nProdId         = XBOX360_NO_PROD_ID;
  m_nBatteryStatus  = 0;
  m_nStatus         = 0;
  m_nOffset         = 0;

  calibrateJoySticks(XBOX360_LEFT_DEAD_ZONE, XBOX360_RIGHT_DEAD_ZONE);

  zeroState();

  m_nLEDPattern   = XBOX360_LED_PAT_ALL_BLINK;
  m_nLeftRumble   = 0;
  m_nRightRumble  = 0;

  for(i=Xbox360FeatIdPadUp; i<Xbox360FeatIdNumOf; ++i)
  {
    m_featMap[i] = i;
  }

  // set error thresholds defaults and clear error counts
  setErrorThresholds();
  clearErrorCnts();

  //
  // USB data
  //
  m_usbContext  = NULL;
  m_usbHandle   = NULL;

  for(i=0; i<UsbPktIdNumOf; ++i)
  {
    m_usbPkt[i].m_usbTransfer   = NULL;
    m_usbPkt[i].m_bHasSubmitted = false;
    m_usbPkt[i].m_bCancel       = false;
  }

  pthread_mutex_init(&m_mutexUpdate, NULL);
  pthread_cond_init(&m_condUpdate,   NULL);
  m_eThreadState = ThreadStateNone;

  // initialize the libusb library, setting context
  if( m_nError = libusb_init(&m_usbContext) != LIBUSB_SUCCESS )
  {
    LOGERROR("libusb_init() failed: %s.", libusb_error_name(m_nError));
    m_usbContext = NULL;
    m_bFatal = true;
  }

  // set debug level
  else
  {
    libusb_set_debug(m_usbContext, usbDebugLevel);
  }
}

HIDXbox360::~HIDXbox360()
{
  if( m_bIsConnected )
  {
    close();
  }

  pthread_cond_destroy(&m_condUpdate);
  pthread_mutex_destroy(&m_mutexUpdate);

  // exit libusb
  if( m_usbContext != NULL )
  {
    libusb_exit(m_usbContext);
    m_usbContext = NULL;
  }
}

int HIDXbox360::open()
{
  // in fatal condition and/or libusb not initialzed
  if( m_bFatal || (m_usbContext == NULL) )
  {
    LOGERROR("libusb not initialized.");
  }

  // Xbox USB device alread open
  else if( m_usbHandle != NULL )
  {
    LOGERROR("Xbox360 controller 0x%04x already open.", m_nProdId);
    m_nError = LIBUSB_ERROR_IO;
  }

  // failed to open Xbox
  else if( (m_nError = openXbox()) < 0 )
  {
    LOGERROR("open() failed: %s.", libusb_error_name(m_nError));
  }

  // failed to allocate and initialize USB transfer packets
  else if( (m_nError = initTransferPkts()) < 0 )
  {
    LOGERROR("initTransferPkts() failed: %s.", libusb_error_name(m_nError));
    close();
  }

  // failed to start async read transfer
  else if( (m_nError = submitReadTransfer()) < 0 )
  {
    LOGERROR("submitReadTransfer() failed: %s.", libusb_error_name(m_nError));
    close();
  }

  // success
  else
  {
    clearErrorCnts();
    zeroState();

    m_nLEDPattern   = XBOX360_LED_PAT_ALL_BLINK;
    m_nLeftRumble   = 0;
    m_nRightRumble  = 0;

    setConnectionState(true);

    flushInput();

    switch( m_nProdId )
    {
      case XBOX360_WIRED_PROD_ID:
        m_strProdName = "Xbox360 Wired Controller";
        setLinkState(true);
        break;
      case XBOX360_WIRELESS_PROD_ID:
      case XBOX360_WIRELESS_WIN_PROD_ID:
        m_strProdName = "Xbox360 Wireless Controller";
        break;
      default:
        m_strProdName = "Xbox360 Controller";
        break;
    }

    LOGDIAG3("Opened %s (product id=0x%04x).",
        m_strProdName.c_str(), m_nProdId);
  }

  return m_nError;
}

int HIDXbox360::close()
{
  // in fatal condition and/or libusb not initialzed
  if( m_bFatal || (m_usbContext == NULL) )
  {
    LOGERROR("libusb not initialized.");
    return m_nError;
  }

  //
  // Close
  //
  if( m_usbHandle != NULL )
  {
    // put Xbox in a good state
    if( (m_state[Xbox360FeatIdLeftRumble] != 0) ||
        (m_state[Xbox360FeatIdRightRumble] != 0) )
    {
      //setRumble(0, 0);
      //usleep(10000);
    }

    // stop update thread
    stop();

    // cancel any pending transfers
    cancelTransfers();

    // free all transfer packet memory
    freeTransferPkts();

    libusb_reset_device(m_usbHandle); 	

    // release claimed interfaces
    for(int i=0; i<XBOX360_NUM_OF_INTERFACES; ++i)
    {
      if( (m_nError = libusb_release_interface(m_usbHandle, i)) < 0 )
      {
        LOGWARN("libusb_release_interface() failed: %s.",
            libusb_error_name(m_nError));
      }
    }

    // close USB opened device
    libusb_close(m_usbHandle);

    LOGDIAG3("Closed Xbox360 controller (product id=0x%04x).", m_nProdId);
  }

  // zap state
  zeroState();
  m_usbHandle     = NULL;
  setConnectionState(false);
  setLinkState(false);
  m_nProdId       = XBOX360_NO_PROD_ID;
  m_strProdName.clear();
  m_strSerialNum.clear();
  m_nStatus       = 0;

  return LIBUSB_SUCCESS;
}

int HIDXbox360::run(float hz)
{
  if( hz <= 0.0 )
  {
    hz = 1.0;
  }

  float t = 1.0/hz;

  m_tsThread.tv_sec = (long)t;
  m_tsThread.tv_nsec = (long)(t - m_tsThread.tv_sec);

  if( m_eThreadState != ThreadStateNone )
  {
    LOGERROR("Update thread already running.");
    return -1;
  }

  else if( createUpdateThread() < 0 )
  {
    LOGERROR("Failed to create update thread.");
    return -1;
  }
 
  else
  {
    return 0;
  }
}

int HIDXbox360::stop()
{
  cancelUpdateThread();
  unlock();
  m_eThreadState = ThreadStateNone;

  return 0;
}

int HIDXbox360::setFeatureVal(int iMnem, int nVal)
{
  FeatMap_T::iterator pos;

  if( (pos = m_featMap.find(iMnem)) == m_featMap.end() )
  {
    return LIBUSB_ERROR_INVALID_PARAM;
  }

  switch( pos->second )
  {
    case Xbox360FeatIdLeftRumble:
      return setRumble(nVal, m_state[Xbox360FeatIdRightRumble]);
    case Xbox360FeatIdRightRumble:
      return setRumble(m_state[Xbox360FeatIdLeftRumble], nVal);
    case Xbox360FeatIdLEDPat:
      return setLED(nVal);
    default:
      return LIBUSB_ERROR_INVALID_PARAM;
  }
}

int HIDXbox360::getFeatureProp(int          iMnem,
                               HIDFeatType &eFeatType,
                               int         &nDir,
                               int         &nMin,
                               int         &nMax,
                               int         &nStep)
{
  FeatMap_T::iterator pos;

  if( (pos = m_featMap.find(iMnem)) == m_featMap.end() )
  {
    return LIBUSB_ERROR_INVALID_PARAM;
  }

  switch( pos->second )
  {
    case Xbox360FeatIdLeftJoyX:
    case Xbox360FeatIdLeftJoyY:
    case Xbox360FeatIdRightJoyX:
    case Xbox360FeatIdRightJoyY:
      eFeatType = HIDFeatTypeRange;
      nDir      = HID_FEAT_INPUT;
      nMin      = numeric_limits<int16_t>::min();
      nMax      = numeric_limits<int16_t>::max();
      nStep     = 1;
      break;
    case Xbox360FeatIdLeftTrigger:
    case Xbox360FeatIdRightTrigger:
      eFeatType = HIDFeatTypeRange;
      nDir      = HID_FEAT_INPUT;
      nMin      = 0;
      nMax      = numeric_limits<int8_t>::max();
      nStep     = 1;
      break;
    case Xbox360FeatIdLeftRumble:
    case Xbox360FeatIdRightRumble:
      eFeatType = HIDFeatTypeRange;
      nDir      = HID_FEAT_OUTPUT;
      nMin      = 0;
      nMax      = numeric_limits<int8_t>::max();
      nStep     = 1;
      break;
    case Xbox360FeatIdLEDPat:
      eFeatType = HIDFeatTypeEnum;
      nDir      = HID_FEAT_INPUT | HID_FEAT_OUTPUT;
      nMin      = XBOX360_LED_PAT_ALL_OFF;
      nMax      = XBOX360_LED_PAT_ALL_BLINK;
      nStep     = 1;
      break;
    default:
      eFeatType = HIDFeatTypeBiState;
      nDir      = HID_FEAT_INPUT;
      nMin      = HID_BTTN_UP;
      nMax      = HID_BTTN_DOWN;
      nStep     = 1;
      break;
  }

  return LIBUSB_SUCCESS;
}

bool HIDXbox360::ping()
{
  // N.B. ping is unreliable - simply return link state
  return m_bIsLinked;
}

void HIDXbox360::goad()
{
  byte_t *p;
  size_t  n;
  int     rc;

  //
  // Request already pending.
  //
  if( m_usbPkt[UsbPktIdGoad].m_bHasSubmitted )
  {
    return;
  }

  //
  // Transfer is being canceled.
  //
  else if( m_usbPkt[UsbPktIdGoad].m_bCancel )
  {
    return;
  }

  p = m_usbPkt[UsbPktIdGoad].m_bufData;

  //
  // Use LED to prompt an Xbox360 response.
  //
  switch( m_nProdId )
  {
    case XBOX360_WIRELESS_PROD_ID:
    case XBOX360_WIRELESS_WIN_PROD_ID:
      n = XBOX360_WL_PKT_HDR_LEN + XBOX360_WL_LED_MSG_LEN;

      memset(p, 0, n);

      // header
      p[XBOX360_WL_PKT_TYPE_POS] = XBOX360_WL_PKT_TYPE_NORM;
      p[1] = XBOX360_WL_LED_PAT_1;
      p[2] = XBOX360_WL_LED_PAT_2;
      p[3] = XBOX360_WL_LED_PAT_3 | m_nLEDPattern;

      m_usbPkt[UsbPktIdGoad].m_usbTransfer->length = (int)n;
      break;

    case XBOX360_WIRED_PROD_ID:
    default:
      n = XBOX360_LED_MSG_LEN;

      p[XBOX360_MSG_TYPE_POS] = XBOX360_LED_MSG_TYPE;
      p[XBOX360_MSG_LEN_POS]  = (byte_t)n;
      p[XBOX360_LED_PAT_POS]  = (byte_t)m_nLEDPattern & XBOX360_LED_PAT_MASK;

      m_usbPkt[UsbPktIdGoad].m_usbTransfer->length = (int)n;
      break;
  }

  m_usbPkt[UsbPktIdGoad].m_bHasSubmitted = true;

  rc = libusb_submit_transfer(m_usbPkt[UsbPktIdGoad].m_usbTransfer);
 
  switch( rc )
  {
    case LIBUSB_SUCCESS:
      break;
    default:
      m_usbPkt[UsbPktIdGoad].m_bHasSubmitted = false;
      LOGERROR("Goad libusb_submit_transfer() failed: %s.",
        libusb_error_name(rc));
      break;
  }
}

int HIDXbox360::setRumble(int nLeftMot, int nRightMot)
{
  byte_t *p;
  size_t  n;
  int     nMaxTries = 3;
  int     nTries;
  int     rc;

  // targets
  m_nLeftRumble   = nLeftMot & XBOX360_RUMBLE_LEFT_MASK;
  m_nRightRumble  = nRightMot & XBOX360_RUMBLE_RIGHT_MASK;

  //
  // No link.
  //
  if( !m_bIsLinked )
  {
    return LIBUSB_SUCCESS;
  }

  //
  // Transfer is being canceled.
  //
  else if( m_usbPkt[UsbPktIdRumble].m_bCancel )
  {
    return LIBUSB_SUCCESS;
  }

  LOGDIAG4("Set rumble moters %d, %d.", m_nLeftRumble, m_nRightRumble);

  //
  // Previous event to rumble change is pending.
  //
  for(nTries=0, rc=LIBUSB_ERROR_BUSY;
      (nTries<nMaxTries) && (rc==LIBUSB_ERROR_BUSY);
      ++nTries)
  {
    if( m_usbPkt[UsbPktIdRumble].m_bHasSubmitted )
    {
      usleep(10);
    }
    else
    {
      rc = LIBUSB_SUCCESS;
    }
  }

  //
  // Last request failed. It may have been lost. Fail this request, but
  // reset to re-enable.
  //
  if( rc != LIBUSB_SUCCESS )
  {
    LOGERROR("Rumble libusb_submit_transfer() failed in %d tries: %s.",
          nTries, libusb_error_name(rc));
    m_usbPkt[UsbPktIdRumble].m_bHasSubmitted = false;
    m_nErrorSend++;
    m_nErrorTotal++;
    m_nError = rc;
    return rc;
  }

  //
  // Write new rumble request.
  //
  p = m_usbPkt[UsbPktIdRumble].m_bufData;

  switch( m_nProdId )
  {
    //
    // From xboxdrv-linux-0.8.5, rumble command =
    // 0x00, 0x01, 0x0f, 0xc0, 0x00, left, right, 0x00, 0x00, 0x00, 0x00, 0x00
    //
    case XBOX360_WIRELESS_PROD_ID:
    case XBOX360_WIRELESS_WIN_PROD_ID:
      n = XBOX360_WL_PKT_HDR_LEN + XBOX360_RUMBLE_MSG_LEN;

      memset(p, 0, n);

      // header
      p[XBOX360_WL_PKT_TYPE_POS] = XBOX360_WL_PKT_TYPE_NORM;
      p[1] = XBOX360_WL_RUMBLE_PAT_1;
      p[2] = XBOX360_WL_RUMBLE_PAT_2;
      p[3] = XBOX360_WL_RUMBLE_PAT_3;

      
      // message
      p[4] = XBOX360_RUMBLE_MSG_TYPE;
      p[5] = (byte_t)m_nLeftRumble;
      p[6] = (byte_t)m_nRightRumble;

      m_usbPkt[UsbPktIdRumble].m_usbTransfer->length = (int)n;
      break;

    case XBOX360_WIRED_PROD_ID:
    default:
      n = XBOX360_RUMBLE_MSG_LEN;

      memset(p, 0, n);

      // message
      p[XBOX360_MSG_TYPE_POS]     = XBOX360_RUMBLE_MSG_TYPE;
      p[XBOX360_MSG_LEN_POS]      = (byte_t)n;
      p[XBOX360_RUMBLE_LEFT_POS]  = (byte_t)nLeftMot;
      p[XBOX360_RUMBLE_RIGHT_POS] = (byte_t)nRightMot;

      m_usbPkt[UsbPktIdRumble].m_usbTransfer->length = (int)n;
      break;
  }

  m_usbPkt[UsbPktIdRumble].m_bHasSubmitted = true;

  rc = libusb_submit_transfer(m_usbPkt[UsbPktIdRumble].m_usbTransfer);
 
  switch( rc )
  {
    case LIBUSB_SUCCESS:
      break;
    default:
      LOGERROR("Rumble libusb_submit_transfer() failed: %s.",
                libusb_error_name(rc));
      m_usbPkt[UsbPktIdRumble].m_bHasSubmitted = false;
      m_nErrorSend++;
      m_nErrorTotal++;
      m_nError = rc;
      break;
  }

  return rc;
}

int HIDXbox360::setLED(int nPattern)
{
  byte_t *p;
  size_t  n;
  int     nMaxTries = 3;
  int     nTries;
  int     rc;

  // target
  m_nLEDPattern = nPattern & XBOX360_LED_PAT_MASK;

  //
  // No link.
  //
  if( !m_bIsLinked )
  {
    return LIBUSB_SUCCESS;
  }

  //
  // Transfer is being canceled.
  //
  else if( m_usbPkt[UsbPktIdLED].m_bCancel )
  {
    return LIBUSB_SUCCESS;
  }

  LOGDIAG4("Set LED pattern %d.", m_nLEDPattern);

  //
  // Previous event to LED state change is pending.
  //
  for(nTries=0, rc=LIBUSB_ERROR_BUSY;
      (nTries<nMaxTries) && (rc==LIBUSB_ERROR_BUSY);
      ++nTries)
  {
    if( m_usbPkt[UsbPktIdLED].m_bHasSubmitted )
    {
      usleep(10);
    }
    else
    {
      rc = LIBUSB_SUCCESS;
    }
  }


  //
  // Last request failed. It may have been lost. Fail this request, but
  // reset to re-enable.
  //
  if( rc != LIBUSB_SUCCESS )
  {
    LOGERROR("LED libusb_submit_transfer() failed in %d tries: %s.",
          nTries, libusb_error_name(rc));
    m_usbPkt[UsbPktIdLED].m_bHasSubmitted = false;
    m_nErrorSend++;
    m_nErrorTotal++;
    m_nError = rc;
    return rc;
  }


  //
  // Write new LED pattern request.
  //
  p = m_usbPkt[UsbPktIdLED].m_bufData;

  switch( m_nProdId )
  {
    case XBOX360_WIRELESS_PROD_ID:
    case XBOX360_WIRELESS_WIN_PROD_ID:
      n = XBOX360_WL_PKT_HDR_LEN + XBOX360_WL_LED_MSG_LEN;

      memset(p, 0, n);

      // header
      p[XBOX360_WL_PKT_TYPE_POS] = XBOX360_WL_PKT_TYPE_NORM;
      p[1] = XBOX360_WL_LED_PAT_1;
      p[2] = XBOX360_WL_LED_PAT_2;
      p[3] = XBOX360_WL_LED_PAT_3 | m_nLEDPattern;

      m_usbPkt[UsbPktIdLED].m_usbTransfer->length = (int)n;
      break;

    case XBOX360_WIRED_PROD_ID:
    default:
      n = XBOX360_LED_MSG_LEN;

      p[XBOX360_MSG_TYPE_POS] = XBOX360_LED_MSG_TYPE;
      p[XBOX360_MSG_LEN_POS]  = (byte_t)n;
      p[XBOX360_LED_PAT_POS]  = (byte_t)m_nLEDPattern & XBOX360_LED_PAT_MASK;

      m_usbPkt[UsbPktIdLED].m_usbTransfer->length = (int)n;
      break;
  }

  m_usbPkt[UsbPktIdLED].m_bHasSubmitted = true;

  rc = libusb_submit_transfer(m_usbPkt[UsbPktIdLED].m_usbTransfer);
 
  switch( rc )
  {
    case LIBUSB_SUCCESS:
      break;
    default:
      LOGERROR("LED libusb_submit_transfer() failed: %s.",
        libusb_error_name(rc));
      m_usbPkt[UsbPktIdLED].m_bHasSubmitted = false;
      m_nErrorSend++;
      m_nErrorTotal++;
      m_nError = rc;
      break;
  }

  return rc;
}

void HIDXbox360::calibrateJoySticks(int nLeftDeadZone, int nRightDeadZone)
{
  float fMax = (float)numeric_limits<int16_t>::max();

  m_nLeftJoyDeadZone  = nLeftDeadZone;
  m_fLeftJoyM         = fMax / (float)(fMax - m_nLeftJoyDeadZone);
  m_fLeftJoyB         = (float)(-m_nLeftJoyDeadZone) * m_fLeftJoyM;

  m_nRightJoyDeadZone = nRightDeadZone;
  m_fRightJoyM        = (float)fMax / (float)(fMax - m_nRightJoyDeadZone);
  m_fRightJoyB        = (float)(-m_nRightJoyDeadZone) * m_fRightJoyM;
}


// -----------------------------------------------------------------------------
// Protected Interface
// -----------------------------------------------------------------------------

void HIDXbox360::setLinkState(bool bNewState)
{
  bool bOldState = m_bIsLinked;

  HIDInput::setLinkState(bNewState);

  //
  // Transition from not linked to linked.
  //
  if( !bOldState && bNewState )
  {
    // reset LED pattern since the xbox may have gone to a 'not linked' pattern
    setLED(m_nLEDPattern);
  }

  //
  // Transition from linked to not linked.
  //
  else if( bOldState && !bNewState )
  {
    zeroState();
  }
}

int HIDXbox360::openXbox()
{
  // Xbox product id's in priority search order
  static ushort_t xbox_prod_id[] =
  {
    XBOX360_WIRED_PROD_ID,
    XBOX360_TETHERED_PROD_ID,
    XBOX360_WIRELESS_PROD_ID,
    XBOX360_WIRELESS_WIN_PROD_ID,
  };

  size_t  i;
  int     rc;

  //
  // Find an Xbox360 controller and claim its interfaces.
  //
  for(i=0; i<arraysize(xbox_prod_id); ++i)
  {
    // open by vendor id, product id
    m_usbHandle = libusb_open_device_with_vid_pid(m_usbContext,
                                          MICROSOFT_VENDOR_ID,
                                          xbox_prod_id[i]);

    if( m_usbHandle != NULL )
    {
      m_nProdId = xbox_prod_id[i];

      if( (rc = claimXboxInterfaces()) < 0 )
      {
        close();
      }
      else
      {
        break;
      }
    }
  }

  //
  // Opened successfully
  //
  if( m_usbHandle != NULL )
  {
    // set specific Xbox360 product values
    switch( m_nProdId )
    {
      case XBOX360_WIRELESS_PROD_ID:
      case XBOX360_WIRELESS_WIN_PROD_ID:
        m_nOffset = XBOX360_MSG_OFFSET_WL;
        break;
      case XBOX360_WIRED_PROD_ID:
        m_nOffset = XBOX360_MSG_OFFSET_W;
        break;
      case XBOX360_TETHERED_PROD_ID:
      default:
        m_nOffset = XBOX360_MSG_OFFSET_DFT;
        break;
    }

    // reset the device to default state
    libusb_reset_device(m_usbHandle);

    return LIBUSB_SUCCESS;
  }

  //
  // Failure
  //
  else
  {
    m_nProdId = XBOX360_NO_PROD_ID;
    m_nError  = LIBUSB_ERROR_NO_DEVICE;
  }

  return m_nError;
}

int HIDXbox360::claimXboxInterfaces()
{
  int   i;
  int   status;
  int   interfaces = 0;

  interfaces = XBOX360_NUM_OF_INTERFACES;

  //
  // RAS: Set all other interface values here
  //

  //
  // Detach any kernel drivers already attached to Xbox.
  //
  for(i=0; i<interfaces; ++i)
  {
    status = libusb_kernel_driver_active(m_usbHandle, i);
    
    // attached, so detach
    if( status > 0 )
    {
      m_nError = libusb_detach_kernel_driver(m_usbHandle, i);
      if( m_nError < 0 )
      {
        LOGERROR("libusb_detach_kernel_driver() failed: %s.",
            libusb_error_name(m_nError));
        return m_nError;
      }
    }
    else if( status < 0 )
    {
      m_nError = status;
      LOGERROR("libusb_kernel_driver_active() failed: %s.",
            libusb_error_name(m_nError));
      return m_nError;
    }
  }

  //
  // Set active configuration for Xbox, performing light-weight reset if
  // necessary.
  //
  if( (m_nProdId == XBOX360_WIRED_PROD_ID) ||
      (m_nProdId == XBOX360_TETHERED_PROD_ID) )
  {
    m_nError = libusb_set_configuration(m_usbHandle, 1);
    if( m_nError < 0 )
    {
      LOGERROR("libusb_set_configuration() failed: %s.",
          libusb_error_name(m_nError));
      return m_nError;
    }
  }

  //
  // Claim the Xbox interfaces.
  //
  for(i=0; i<interfaces; ++i)
  {
    m_nError = libusb_claim_interface(m_usbHandle, i);
    if( m_nError < 0 )
    {
      LOGERROR("libusb_claim_interface() failed: %s.",
          libusb_error_name(m_nError));
      return m_nError;
    }
  }

  return LIBUSB_SUCCESS;
}

int HIDXbox360::initTransferPkts()
{
  for(int i=0; i<UsbPktIdNumOf; ++i)
  {
    // allocate a libusb transfer
    m_usbPkt[i].m_usbTransfer = libusb_alloc_transfer(0);

    if( m_usbPkt[i].m_usbTransfer == NULL )
    {
      LOGERROR("libusb_alloc_transfer() failed.");
      m_nError = LIBUSB_ERROR_OTHER;
      return m_nError;
    }

    //
    // Populate transfer fields.
    //
    switch( i )
    {
      case UsbPktIdInput:
        libusb_fill_interrupt_transfer(m_usbPkt[i].m_usbTransfer,
                                       m_usbHandle,
                                       XBOX360_READ_ENDPOINT, 
                                       m_usbPkt[i].m_bufData,
                                       (int)sizeof(m_usbPkt[i].m_bufData),
                                       transferCallbackInput,
                                       this,
                                       100); // timeout ms
        break;
      case UsbPktIdRumble:
        libusb_fill_interrupt_transfer(m_usbPkt[i].m_usbTransfer,
                                       m_usbHandle,
                                       XBOX360_WRITE_ENDPOINT, 
                                       m_usbPkt[i].m_bufData,
                                       (int)sizeof(m_usbPkt[i].m_bufData),
                                       transferCallbackRumble,
                                       this,
                                       0); // timeout ms
        break;
      case UsbPktIdLED:
        libusb_fill_interrupt_transfer(m_usbPkt[i].m_usbTransfer,
                                       m_usbHandle,
                                       XBOX360_WRITE_ENDPOINT, 
                                       m_usbPkt[i].m_bufData,
                                       (int)sizeof(m_usbPkt[i].m_bufData),
                                       transferCallbackLED,
                                       this,
                                       0); // timeout ms
        break;
      case UsbPktIdGoad:
        libusb_fill_interrupt_transfer(m_usbPkt[i].m_usbTransfer,
                                       m_usbHandle,
                                       XBOX360_WRITE_ENDPOINT, 
                                       m_usbPkt[i].m_bufData,
                                       (int)sizeof(m_usbPkt[i].m_bufData),
                                       transferCallbackGoad,
                                       this,
                                       0); // timeout ms
        break;
    }

    m_usbPkt[i].m_bHasSubmitted = false;
    m_usbPkt[i].m_bCancel       = false;
  }
}

void HIDXbox360::freeTransferPkts()
{
  for(int i=0; i<UsbPktIdNumOf; ++i)
  {
    if( m_usbPkt[i].m_usbTransfer == NULL )
    {
      continue;
    }
    else if( m_usbPkt[i].m_bHasSubmitted )
    {
      LOGWARN("USB transfer packet %d has been submitted - cannot free.", i);
    }
    else
    {
      libusb_free_transfer(m_usbPkt[i].m_usbTransfer);
      m_usbPkt[i].m_usbTransfer = NULL;
    }
  }
}

int HIDXbox360::submitReadTransfer()
{
  int   rc;

  m_usbPkt[UsbPktIdInput].m_bHasSubmitted = false;
  
  if( m_usbPkt[UsbPktIdInput].m_bCancel )
  {
    return LIBUSB_SUCCESS;
  }

  // set up transfer
  rc = libusb_submit_transfer(m_usbPkt[UsbPktIdInput].m_usbTransfer);

  if( rc == LIBUSB_SUCCESS )
  {
    m_usbPkt[UsbPktIdInput].m_bHasSubmitted = true;
  }
  else
  {
    LOGERROR("Input libusb_submit_transfer() failed: %s.",
        libusb_error_name(rc));
    m_nError = rc;
  }

  return rc;
}

void HIDXbox360::transferCallbackInput(struct libusb_transfer *transfer)
{
  HIDXbox360 *pThis = (HIDXbox360*)transfer->user_data;
  bool        bResubmit = true;
  bool        bReceived = false;

  //
  // USB transfer status.
  //
  switch( transfer->status )
  {
    //
    // Read transfer completed. Parse, validate, and set state according to the
    // received packet.
    //
    case LIBUSB_TRANSFER_COMPLETED:
      switch( pThis->m_nProdId )
      {
        case XBOX360_WIRELESS_PROD_ID:
        case XBOX360_WIRELESS_WIN_PROD_ID:
          bReceived = pThis->parseWireless(transfer->buffer,
                                          transfer->actual_length);
          break;
        case XBOX360_WIRED_PROD_ID:
        default:
          bReceived = pThis->parseWired(transfer->buffer,
                                       transfer->actual_length);
          break;
      }
      break;

    //
    // USB transfer error.
    //
    case LIBUSB_TRANSFER_ERROR:
      LOGERROR("Transfer error.");
      pThis->m_nErrorRcv++;
      pThis->m_nErrorTotal++;
      break;

    //
    // No USB device. Probably unplugged.
    //
    case LIBUSB_TRANSFER_NO_DEVICE:
      LOGERROR("Disconnected!");
      pThis->m_nErrorRcv++;
      pThis->m_nErrorTotal++;
      pThis->setConnectionState(false);
      pThis->setLinkState(false);
      bResubmit = false;
      break;

    //
    // USB transfer timed out. This will happen when there are no new Xbox
    // events to report or when there is no link for the wireless Xbox.
    // Critical to differentiate these two conditions.
    //
    case LIBUSB_TRANSFER_TIMED_OUT:
      //LOGERROR("Receive Timeout!");
      if( pThis->isWireless() && pThis->m_bIsLinked )
      {
        pThis->m_nErrorRcvTimeout++;
        if( (pThis->m_nErrorRcvTimeout % 15) == 0 ) // too quiet - goad xbox
        {
          // RDK can't get this to work
          //pThis->goad();
        }
      }
      break;

    //
    // USB transfer is being canceled. Typical when closing the interface.
    //
    case LIBUSB_TRANSFER_CANCELLED:
      LOGDIAG3("Tranfer cancelled.");
      bResubmit = false;
      break;

    //
    // USB transfer stalled.
    //
    case LIBUSB_TRANSFER_STALL:
      LOGERROR("Transfer stalled.");
      pThis->m_nErrorRcv++;
      pThis->m_nErrorTotal++;
      break;

    //
    // USB transfer overflow. Typically the OS cannot handle the events soon
    // enough.
    //
    case LIBUSB_TRANSFER_OVERFLOW:
      LOGERROR("Overflow: Amount transferred: %d", transfer->actual_length);
      pThis->m_nErrorRcv++;
      pThis->m_nErrorTotal++;
      break;

    //
    // Should never be here.
    //
    default:
      LOGERROR("HIDXbox360 Controller: Unknown error, USB status: %d", 
               transfer->status);
      pThis->m_nErrorRcv++;
      pThis->m_nErrorTotal++;
      break; 
  }

  //
  // Received an input packet.
  //
  if( bReceived )
  {
    pThis->m_nErrorRcv         = 0;
    pThis->m_nErrorRcvTimeout  = 0;
  }

  //
  // No input packet received. Check error thresholds, which could result in
  // taking down the link.
  //
  else
  {
    pThis->checkErrorThresholds();
  }

  //
  // Resubmit new read transfer request.
  //
  if( bResubmit )
  {
    pThis->submitReadTransfer();
  }

  //
  // Don't resubmit new read transfer request.
  //
  else
  {
    pThis->m_usbPkt[UsbPktIdInput].m_bHasSubmitted = false;
  }
}

void HIDXbox360::transferCallbackRumble(struct libusb_transfer *transfer)
{
  HIDXbox360 *pThis = (HIDXbox360*)transfer->user_data;

  pThis->m_usbPkt[UsbPktIdRumble].m_bHasSubmitted = false;

  //
  // USB transfer status.
  //
  switch( transfer->status )
  {
    //
    // Write transfer completed. The buffer just contains the data sent in the
    // write tranfer buffer. Not a true reply, but accept as successful.
    // 
    case LIBUSB_TRANSFER_COMPLETED:
      pThis->m_state[Xbox360FeatIdLeftRumble]  = pThis->m_nLeftRumble;
      pThis->m_state[Xbox360FeatIdRightRumble] = pThis->m_nRightRumble;
      pThis->m_nErrorSend = 0;
      //pThis->debugPrintTransferBuf("DBG: rumble cb", transfer);
      break;

    //
    // No USB device. Probably unplugged.
    //
    case LIBUSB_TRANSFER_NO_DEVICE:
      LOGERROR("Disconnected!");
      pThis->m_nErrorSend++;
      pThis->m_nErrorTotal++;
      pThis->setConnectionState(false);
      pThis->setLinkState(false);
      break;

    //
    // USB transfer is being canceled. Typical when closing the interface.
    //
    case LIBUSB_TRANSFER_CANCELLED:
      LOGDIAG3("Tranfer cancelled.");
      break;

    //
    // USB transfer timed out. Write did not succeed in this case.
    //
    case LIBUSB_TRANSFER_TIMED_OUT:
      LOGERROR("USB write rumble motors timed out.");
      pThis->m_nErrorRcvTimeout++;
      break;

    //
    // USB transfer errors.
    //
    case LIBUSB_TRANSFER_ERROR:
    case LIBUSB_TRANSFER_STALL:
    case LIBUSB_TRANSFER_OVERFLOW:
    default:
      LOGERROR("USB write rumble motors failed (length=%d): USB status=%d",
        transfer->length, transfer->status);
      pThis->m_nErrorRcv++;
      pThis->m_nErrorTotal++;
      break;
  }
}

void HIDXbox360::transferCallbackLED(struct libusb_transfer *transfer)
{
  HIDXbox360 *pThis = (HIDXbox360*)transfer->user_data;

  pThis->m_usbPkt[UsbPktIdLED].m_bHasSubmitted = false;

  //
  // USB transfer status.
  //
  switch( transfer->status )
  {
    //
    // Write transfer completed. The buffer just contains the data sent in the
    // write tranfer buffer. Not a true reply, but accept as successful.
    // 
    case LIBUSB_TRANSFER_COMPLETED:
      pThis->m_state[Xbox360FeatIdLEDPat] = pThis->m_nLEDPattern;
      pThis->m_nErrorSend = 0;
      //pThis->debugPrintTransferBuf("DBG: led cb", transfer);
      break;

    //
    // No USB device. Probably unplugged.
    //
    case LIBUSB_TRANSFER_NO_DEVICE:
      LOGERROR("Disconnected!");
      pThis->m_nErrorSend++;
      pThis->m_nErrorTotal++;
      pThis->setConnectionState(false);
      pThis->setLinkState(false);
      break;

    //
    // USB transfer is being canceled. Typical when closing the interface.
    //
    case LIBUSB_TRANSFER_CANCELLED:
      LOGDIAG3("Tranfer cancelled.");
      break;

    //
    // USB transfer timed out. Write did not succeed in this case.
    //
    case LIBUSB_TRANSFER_TIMED_OUT:
      LOGERROR("USB write LED pattern timed out.");
      pThis->m_nErrorRcvTimeout++;
      break;

    //
    // USB transfer errors.
    //
    case LIBUSB_TRANSFER_ERROR:
    case LIBUSB_TRANSFER_STALL:
    case LIBUSB_TRANSFER_OVERFLOW:
    default:
      LOGERROR("USB write LED pattern (length=%d): USB status=%d.",
        transfer->length, transfer->status);
      pThis->m_nErrorRcv++;
      pThis->m_nErrorTotal++;
      break;
  }
}

void HIDXbox360::transferCallbackGoad(struct libusb_transfer *transfer)
{
  HIDXbox360 *pThis = (HIDXbox360*)transfer->user_data;

  pThis->m_usbPkt[UsbPktIdGoad].m_bHasSubmitted = false;

  //
  // USB transfer status. Not too concern about the status.
  //
  switch( transfer->status )
  {
    case LIBUSB_TRANSFER_COMPLETED:
      //pThis->debugPrintTransferBuf("DBG: goad cb", transfer);
      break;
    case LIBUSB_TRANSFER_NO_DEVICE:
    case LIBUSB_TRANSFER_TIMED_OUT:
    case LIBUSB_TRANSFER_CANCELLED:
    case LIBUSB_TRANSFER_ERROR:
    case LIBUSB_TRANSFER_STALL:
    case LIBUSB_TRANSFER_OVERFLOW:
    default:
      break;
  }
}

void HIDXbox360::cancelTransfers()
{
  int   nMaxTries = 5;
  int   nTries;
  int   nTrans;
  int   i;

  for(i=0; i<UsbPktIdNumOf; ++i)
  {
    m_usbPkt[i].m_bCancel = true;
  }

  for(nTries=0; nTries<nMaxTries; ++nTries)
  {
    update();

    for(i=0, nTrans=0; i<UsbPktIdNumOf; ++i)
    {
      if( !m_usbPkt[i].m_bHasSubmitted )
      {
        ++nTrans;
      }
    }

    if( nTrans == UsbPktIdNumOf )
    {
      return;
    }
  }
}

void HIDXbox360::flushInput()
{
  static int    nMaxTries = 5;

  for(int i=0; i<nMaxTries; ++i)
  {
    update();
  }
}

bool HIDXbox360::parseWireless(byte_t buf[], ssize_t n)
{
  bool  bGoodPkt    = true;   // recevied good (bad) packet
  bool  bNewState   = true;   // new link state

  //
  // Link Status Change Packet. Not reliable.
  //
  if( (n == XBOX360_WL_LINK_PKT_LEN) &&
      (buf[XBOX360_WL_PKT_TYPE_POS] == XBOX360_WL_PKT_TYPE_CHG) )
  {
    m_nStatus = buf[XBOX360_WL_LINK_STATUS_POS];
    switch( m_nStatus )
    {
      case XBOX360_WL_LINK_STATUS_CTLR:
        LOGDIAG3("Wireless status: Contoller linked.");
        break;
      case XBOX360_WL_LINK_STATUS_HEADSET:
        LOGDIAG3("Wireless status: Headset linked.");
        break;
      case XBOX360_WL_LINK_STATUS_CTLR_HEADSET:
        LOGDIAG3("Wireless status: Contoller and headset linked.");
        break;
      case XBOX360_WL_LINK_STATUS_NO_LINK:
        LOGERROR("Wireless link: No link.");
        bNewState = false;
        break;
      default:
        break;
    }
  }

  //
  // Packet is too short.
  //
  else if( n < XBOX360_WL_NORM_PKT_LEN )
  {
    LOGERROR("Short packet.");
    bGoodPkt  = false;
    bNewState = m_bIsLinked;
    //debugPrintTransferBuf("DBG: short pkt", buf, n);
  }

  //
  // Unknown packet type.
  //
  // Note: These could be valid packets, simply undocumented.
  //
  else if( buf[XBOX360_WL_PKT_TYPE_POS] != XBOX360_WL_PKT_TYPE_NORM )
  {
    LOGERROR("Unknown packet type = 0x%02x.", buf[XBOX360_WL_PKT_TYPE_POS]);
    bGoodPkt  = false;
    bNewState = m_bIsLinked;
    //debugPrintTransferBuf("DBG: unknown pkt type", buf, n);
  }

  //
  // Button state update message.
  //
  else if(  (buf[1] == XBOX360_WL_BTTN_PAT_1) &&
            (buf[2] == XBOX360_WL_BTTN_PAT_2) &&
            (buf[3] == XBOX360_WL_BTTN_PAT_3) &&
            (buf[m_nOffset+XBOX360_MSG_TYPE_POS] == XBOX360_BTTN_MSG_TYPE) )
  {
    LOGDIAG4("Button state update packet.");
    updateButtonState(buf+m_nOffset, n-m_nOffset);
  }

  //
  // Announcement message.
  // 
  // It is sent when controller transitions from no-link to linked. Not so
  // often sent by dongle though.
  //
  else if(  (buf[1] == XBOX360_WL_ANN_PAT_1) &&
            (buf[2] == XBOX360_WL_ANN_PAT_2) &&
            (buf[3] == XBOX360_WL_ANN_PAT_3) )
  {
    char        bufsn[16];
    const char *sep = "";

    m_strSerialNum.clear();

    for(int i=0; i<XBOX360_WL_ANN_SN_LEN; ++i)
    {
      sprintf(bufsn, "%s%02x", sep, (uint_t)buf[XBOX360_WL_ANN_SN_POS+i]&0xff);
      m_strSerialNum += bufsn;
      sep = ":";
    }

    m_nBatteryStatus = buf[XBOX360_WL_ANN_BATT_POS];

    LOGDIAG3("Announcement packet: S/N %s, battery status: %d",
          m_strSerialNum.c_str(), m_nBatteryStatus);
  }

  //
  // Battery status message.
  //
  else if(  (buf[1] == XBOX360_WL_BATT_PAT_1) &&
            (buf[2] == XBOX360_WL_BATT_PAT_2) &&
            (buf[3] == XBOX360_WL_BATT_PAT_3) )
  {
    m_nBatteryStatus = buf[XBOX360_WL_BATT_BATT_POS];
    LOGDIAG3("Battery status packet: battery status: %d", m_nBatteryStatus);
  }

  //
  // Null message.
  //
  else if(  (buf[1] == XBOX360_WL_NULL_PAT_1) &&
            (buf[2] == XBOX360_WL_NULL_PAT_2) )
  {
    LOGDIAG4("00 messsage: val=0x%02x.", buf[3]);
    if( buf[3] != XBOX360_WL_NULL_PAT_3_ACK )
    {
      bNewState = m_bIsLinked;
    }
    //debugPrintTransferBuf("00 msg:", buf, 4);
  }

  //
  // F8 message. Ignore.
  //
  else if(  (buf[1] == XBOX360_WL_F8_PAT_1) &&
            (buf[3] == XBOX360_WL_F8_PAT_3) )
  {
    LOGDIAG4("F8 messsage: val=0x%02x.", buf[2]);
    bNewState = m_bIsLinked;
    //debugPrintTransferBuf("F8 msg:", buf, 4);
  }

  //
  // Don't know, but assume valid. Ignore.
  //
  else
  {
    LOGDIAG4("%02x messsage: val=0x%02x.", buf[1]);
    bNewState = m_bIsLinked;
    //debugPrintTransferBuf("unknown msg:", buf, 4);
  }

  if( bGoodPkt )
  {
    m_nErrorRcv         = 0;
    m_nErrorRcvTimeout  = 0;
  }

  else
  {
    ++m_nErrorRcv;
    ++m_nErrorTotal;
  }

  if( bNewState != m_bIsLinked )
  {
    setLinkState(bNewState);
  }

  return bGoodPkt;
}

bool HIDXbox360::parseWired(byte_t buf[], ssize_t n)
{
  bool bGoodPkt;

  switch( buf[XBOX360_MSG_TYPE_POS] )
  {
    case XBOX360_BTTN_MSG_TYPE:
      if( buf[XBOX360_MSG_LEN_POS] == XBOX360_BTTN_MSG_LEN )
      {
        updateButtonState(buf, n);
        //LOGDIAG3("updated button state.");
        bGoodPkt = true;
      }
      break;
    case XBOX360_LED_MSG_TYPE:
      if( buf[XBOX360_MSG_LEN_POS] == XBOX360_LED_MSG_LEN )
      {
        updateLEDState(buf, n);
        //LOGDIAG3("updated led state.");
        bGoodPkt = true;
      }
      break;
    default:
      break;
  }

  if( !bGoodPkt )
  {
    //debugPrintTransferBuf("DBG: wired input", buf, n);
  }

  return bGoodPkt;
}

void HIDXbox360::updateButtonState(byte_t msg[], ssize_t n)
{
  int   val;

  // 19 or 20 byte message
  if( msg[XBOX360_MSG_LEN_POS] < XBOX360_BTTN_MSG_MIN_LEN )
  {
    return;
  }

  // --
  // Update state
  // --
  
  // 
  // D Pad
  //
  m_state[Xbox360FeatIdPadUp] =
        updown(msg[XBOX360_BTTN_PAD_UP_POS] & XBOX360_BTTN_PAD_UP_MASK);
  m_state[Xbox360FeatIdPadDown] =
        updown(msg[XBOX360_BTTN_PAD_DOWN_POS] & XBOX360_BTTN_PAD_DOWN_MASK);
  m_state[Xbox360FeatIdPadLeft] =
        updown(msg[XBOX360_BTTN_PAD_LEFT_POS] & XBOX360_BTTN_PAD_LEFT_MASK);
  m_state[Xbox360FeatIdPadRight] =
        updown(msg[XBOX360_BTTN_PAD_RIGHT_POS] & XBOX360_BTTN_PAD_RIGHT_MASK);

  //
  // Start/Back buttons
  //
  m_state[Xbox360FeatIdStart] =
        updown(msg[XBOX360_BTTN_START_POS] & XBOX360_BTTN_START_MASK);
  m_state[Xbox360FeatIdBack] =
        updown(msg[XBOX360_BTTN_BACK_POS] & XBOX360_BTTN_BACK_MASK);

  //
  // Stick clicks
  //
  m_state[Xbox360FeatIdLeftStickClick] =
    updown(msg[XBOX360_BTTN_LEFT_STICK_CLICK_POS] &
        XBOX360_BTTN_LEFT_STICK_CLICK_MASK);
  m_state[Xbox360FeatIdRightStickClick] =
    updown(msg[XBOX360_BTTN_RIGHT_STICK_CLICK_POS] &
        XBOX360_BTTN_RIGHT_STICK_CLICK_MASK);

  //
  // Bump buttons
  //
  m_state[Xbox360FeatIdLeftBump] =
      updown(msg[XBOX360_BTTN_LEFT_BUMP_POS] & XBOX360_BTTN_LEFT_BUMP_MASK);
  m_state[Xbox360FeatIdRightBump] =
      updown(msg[XBOX360_BTTN_RIGHT_BUMP_POS] & XBOX360_BTTN_RIGHT_BUMP_MASK);

  //
  // Center LED button
  //
  m_state[Xbox360FeatIdCenterX] =
        updown(msg[XBOX360_BTTN_CENTER_X_POS] & XBOX360_BTTN_CENTER_X_MASK);

  //
  // Function buttons
  //
  m_state[Xbox360FeatIdAButton] =
        updown(msg[XBOX360_BTTN_A_BUTTON_POS] & XBOX360_BTTN_A_BUTTON_MASK);
  m_state[Xbox360FeatIdBButton] =
        updown(msg[XBOX360_BTTN_B_BUTTON_POS] & XBOX360_BTTN_B_BUTTON_MASK);
  m_state[Xbox360FeatIdXButton] =
        updown(msg[XBOX360_BTTN_X_BUTTON_POS] & XBOX360_BTTN_X_BUTTON_MASK);
  m_state[Xbox360FeatIdYButton] =
        updown(msg[XBOX360_BTTN_Y_BUTTON_POS] & XBOX360_BTTN_Y_BUTTON_MASK);

  //
  // Triggers
  //
  m_state[Xbox360FeatIdLeftTrigger] =
            convertToInt(msg[XBOX360_BTTN_LEFT_TRIGGER_POS], 0);
  m_state[Xbox360FeatIdRightTrigger] =
            convertToInt(msg[XBOX360_BTTN_RIGHT_TRIGGER_POS], 0);

  //
  // Left Joystick
  //
  val = convertToInt( msg[XBOX360_BTTN_LEFT_X_POS_L],
                      msg[XBOX360_BTTN_LEFT_X_POS_H] );

  if( val > m_nLeftJoyDeadZone )
  {
    m_state[Xbox360FeatIdLeftJoyX] = (int)(m_fLeftJoyM * val + m_fLeftJoyB);
  }
  else if( val < -m_nLeftJoyDeadZone )
  {
    m_state[Xbox360FeatIdLeftJoyX] = (int)(m_fLeftJoyM * val - m_fLeftJoyB);
  }
  else
  {
    m_state[Xbox360FeatIdLeftJoyX] = 0;
  }

  val = convertToInt( msg[XBOX360_BTTN_LEFT_Y_POS_L],
                      msg[XBOX360_BTTN_LEFT_Y_POS_H] );

  if( val > m_nLeftJoyDeadZone )
  {
    m_state[Xbox360FeatIdLeftJoyY] = (int)(m_fLeftJoyM * val + m_fLeftJoyB);
  }
  else if( val < -m_nLeftJoyDeadZone )
  {
    m_state[Xbox360FeatIdLeftJoyY] = (int)(m_fLeftJoyM * val - m_fLeftJoyB);
  }
  else
  {
    m_state[Xbox360FeatIdLeftJoyY] = 0;
  }

  //
  // Right Joystick
  //
  val = convertToInt( msg[XBOX360_BTTN_RIGHT_X_POS_L],
                      msg[XBOX360_BTTN_RIGHT_X_POS_H] );

  if( val > m_nRightJoyDeadZone )
  {
    m_state[Xbox360FeatIdRightJoyX] = (int)(m_fRightJoyM * val + m_fRightJoyB);
  }
  else if( val < -m_nRightJoyDeadZone )
  {
    m_state[Xbox360FeatIdRightJoyX] = (int)(m_fRightJoyM * val - m_fRightJoyB);
  }
  else
  {
    m_state[Xbox360FeatIdRightJoyX] = 0;
  }

  val = convertToInt( msg[XBOX360_BTTN_RIGHT_Y_POS_L],
                      msg[XBOX360_BTTN_RIGHT_Y_POS_H] );

  if( val > m_nRightJoyDeadZone )
  {
    m_state[Xbox360FeatIdRightJoyY] = (int)(m_fRightJoyM * val + m_fRightJoyB);
  }
  else if( val < -m_nRightJoyDeadZone )
  {
    m_state[Xbox360FeatIdRightJoyY] = (int)(m_fRightJoyM * val - m_fRightJoyB);
  }
  else
  {
    m_state[Xbox360FeatIdRightJoyY] = 0;
  }
}

void HIDXbox360::updateRumbleState(byte_t msg[], ssize_t n)
{
  switch( m_nProdId )
  {
    case XBOX360_WIRELESS_PROD_ID:
    case XBOX360_WIRELESS_WIN_PROD_ID:
      m_state[Xbox360FeatIdLeftRumble]  = msg[5];
      m_state[Xbox360FeatIdRightRumble] = msg[6];
      break;

    case XBOX360_WIRED_PROD_ID:
    default:
      if( n >= XBOX360_RUMBLE_MSG_LEN )
      {
        m_state[Xbox360FeatIdLeftRumble] = 
                            convertToInt(msg[XBOX360_RUMBLE_LEFT_POS], 0);
        m_state[Xbox360FeatIdRightRumble] =
                            convertToInt(msg[XBOX360_RUMBLE_RIGHT_POS], 0);
      }
      break;
  }
}

void HIDXbox360::updateLEDState(byte_t buf[], ssize_t n)
{
  switch( m_nProdId )
  {
    case XBOX360_WIRELESS_PROD_ID:
    case XBOX360_WIRELESS_WIN_PROD_ID:
      if( n >= XBOX360_MSG_OFFSET_WL )
      {
        m_state[Xbox360FeatIdLEDPat] = buf[XBOX360_WL_LED_PAT_POS] &
                                            XBOX360_LED_PAT_MASK;
      }
      break;

    case XBOX360_WIRED_PROD_ID:
    default:
      if( n >= XBOX360_LED_MSG_LEN )
      {
        m_state[Xbox360FeatIdLEDPat] = buf[XBOX360_LED_PAT_POS] &
                                            XBOX360_LED_PAT_MASK;
      }
      break;
  }
}

int HIDXbox360::convertToInt(byte_t lsb, byte_t msb)
{
  int output;

  output = (int)( (((uint_t)msb) << 8) | (uint_t)lsb);

  if( output & 0x8000 )
  {
    output -= 0xffff;
  }

  return output;
}

bool HIDXbox360::checkErrorThresholds()
{
  if( !m_bIsLinked )
  {
    return m_bIsLinked;
  }

  //
  // Receive timeout threshold.
  //
  if((m_nErrorRcvTimeoutTh > 0) && (m_nErrorRcvTimeout > m_nErrorRcvTimeoutTh))
  {
    LOGERROR("Consecutive received timeout error count %d "
             "exceeds threshold %d.", m_nErrorRcvTimeout, m_nErrorRcvTimeoutTh);
    ++m_nErrorRcv;
    ++m_nErrorTotal;
    setLinkState(false);
  }

  //
  // Receive error threshold.
  //
  if( (m_nErrorRcvTh > 0) && (m_nErrorRcv > m_nErrorRcvTh) )
  {
    LOGERROR("Consecutive received error count %d exceeds threshold %d.",
        m_nErrorRcv, m_nErrorRcvTh);
    setLinkState(false);
  }

  //
  // Total receive error threshold.
  //
  if( (m_nErrorTotalTh > 0) && (m_nErrorTotal > m_nErrorTotalTh) )
  {
    LOGERROR("Total received error count %d exceeds threshold %d.",
        m_nErrorTotal, m_nErrorTotalTh);
    setLinkState(false);
  }

  return m_bIsLinked;
}

int HIDXbox360::createUpdateThread()
{
  int   rc;

  m_eThreadState = ThreadStateRunning;

  rc = pthread_create(&m_threadId, NULL, HIDXbox360::updateThread, (void*)this);
 
  if( rc < 0 )
  {
    LOGSYSERROR("pthread_create()");
  }

  return rc;
}

int HIDXbox360::cancelUpdateThread()
{
  m_eThreadState = ThreadStateExit;
  signalUpdateThread();
  usleep(1000);
  pthread_cancel(m_threadId);
  pthread_join(m_threadId, NULL);
  return 0;
}

void HIDXbox360::signalUpdateThread()
{
  pthread_cond_signal(&m_condUpdate);
}

void HIDXbox360::blockWait()
{
  lock();
  pthread_cond_timedwait(&m_condUpdate, &m_mutexUpdate, &m_tsThread);
  unlock();
}

int HIDXbox360::update(uint_t uMSec)
{
  struct timeval  tvWait;

  // no connected
  if( !m_bIsConnected )
  {
    m_nError = LIBUSB_ERROR_NO_DEVICE;
    return m_nError;
  }

  tvWait.tv_sec   = 0;
  tvWait.tv_usec  = uMSec * 1000;

  lock();

  //
  // N.B. Cannot make libusb_handle_events_*() call from a callback since the
  // callback was called within this call. Causes deadlock.
  //
  m_nError = libusb_handle_events_timeout_completed(m_usbContext,
                                                      &tvWait, NULL);

  unlock();
 
  return m_nError;
}

void *HIDXbox360::updateThread(void *pArg)
{
  HIDXbox360 *pThis = (HIDXbox360 *)pArg;
  int         oldstate;
  int         rc;

  pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, &oldstate);
  pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, &oldstate);

  LOGDIAG3("Update thread created.");

  while( (pThis->m_eThreadState == ThreadStateRunning) && pThis->isConnected() )
  {
    pThis->blockWait();

    switch( pThis->m_eThreadState )
    {
      case ThreadStateRunning:
        pThis->update(0);
        break;
      case ThreadStateExit:
      default:
        break;
    }
  }

  pThis->m_eThreadState = ThreadStateNone;

  LOGDIAG3("Update thread exited.");

  return NULL;
}

void HIDXbox360::debugPrintHdr()
{
  fprintf(stderr,
"                            %s (Product Id 0x%04x)"
"\n"
" DPad     Bump  Bk Cx St  A B X Y    Trigger   LeftJoyStick     RightJoytStick"
"    LED  Rumble   Link Status Batt  ERTO  ER  ES  ETOT"
"\n", m_strProdName.c_str(), m_nProdId);
//"1 1 1 1   0 0   1  1  1   1 1 1 1   255 255  1 -32333 -32333  1 -32333 -32333
//    0xhh false  127  10 100
}

void HIDXbox360::debugPrintState()
{
  fprintf(stderr,
    "%1d %1d %1d %1d   %1d %1d   %1d  %1d  %1d   %1d %1d %1d %1d   %3d %3d   "
    "%1d %6d %6d   %1d %6d %6d    %2d %3d %3d  %5s   0x%02x %4d %5d %3d %3d %5d\r",
        m_state[Xbox360FeatIdPadUp],
        m_state[Xbox360FeatIdPadDown],
        m_state[Xbox360FeatIdPadLeft],
        m_state[Xbox360FeatIdPadRight],

        m_state[Xbox360FeatIdLeftBump],
        m_state[Xbox360FeatIdRightBump],

        m_state[Xbox360FeatIdBack],
        m_state[Xbox360FeatIdCenterX],
        m_state[Xbox360FeatIdStart],

        m_state[Xbox360FeatIdAButton],
        m_state[Xbox360FeatIdBButton],
        m_state[Xbox360FeatIdXButton],
        m_state[Xbox360FeatIdYButton],

        m_state[Xbox360FeatIdLeftTrigger],
        m_state[Xbox360FeatIdRightTrigger],

        m_state[Xbox360FeatIdLeftStickClick],
        m_state[Xbox360FeatIdLeftJoyX],
        m_state[Xbox360FeatIdLeftJoyY],
        m_state[Xbox360FeatIdRightStickClick],
        m_state[Xbox360FeatIdRightJoyX],
        m_state[Xbox360FeatIdRightJoyY],
          
        m_state[Xbox360FeatIdLEDPat],
        m_state[Xbox360FeatIdLeftRumble],
        m_state[Xbox360FeatIdRightRumble],

        (m_bIsLinked? "true": "false"),
        m_nStatus,
        m_nBatteryStatus,
        m_nErrorRcvTimeout,
        m_nErrorRcv,
        m_nErrorSend,
        m_nErrorTotal);

  fflush(stderr);
}

void HIDXbox360::debugPrintTransferBuf(const std::string &strPreface,
                                       byte_t             buf[],
                                       ssize_t            n,
                                       const std::string &strEoR)
{
  struct timeval  tvNow;
  double          t;

  gettimeofday(&tvNow, NULL);

  t = (double)tvNow.tv_sec + (double)tvNow.tv_usec / 1000000.0;

  fprintf(stderr, "%.3lf ", t);

  if( !strPreface.empty() )
  {
    fprintf(stderr, "%s: ", strPreface.c_str());
  }

  fprintf(stderr, "buf_len=%zd: ", n);

  for(int i=0; i<n; ++i)
  {
    fprintf(stderr, "%02x ", buf[i]);
  }

  fprintf(stderr, "%s", strEoR.c_str());
  fflush(stderr);
}
