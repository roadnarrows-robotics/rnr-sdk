////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Peripherals
//
// Library:   libhid
//
// File:      HIDXbox360.h
//
/*! \file
 *
 * $LastChangedDate: 2014-07-10 15:03:23 -0600 (Thu, 10 Jul 2014) $
 * $Rev: 3696 $
 *
 * \brief Xbox360 Controller C interface.
 *
 * \author Rob Shiely (rob.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2012-2013.  RoadNarrows LLC.
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


#ifndef _HIDXBOX360_H
#define _HIDXBOX360_H

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <limits.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdarg.h>
#include <time.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>

#include <map>
#include <string>

#include <libusb-1.0/libusb.h>

#include "rnr/rnrconfig.h"

#include "rnr/usbext.h"
#include "rnr/hid/HID.h"

/*!
 * \ingroup periph_hid
 * \defgroup periph_hid_xbox Xbox360
 * \{
 *
 * Xbox360 Human Interface Device interface.
 *
 * \}
 */

/*!
 * \ingroup periph_hid_xbox
 * \defgroup periph_hid_xbox_intro Intro
 * \{

The Xbox 360 game controller is a RoadNarrows supported HID.
It provides an easy to use interface to manually 
control RoadNarrows robotic platforms such as the Kuon\h_reg and the 
Hekateros\h_reg. 
\htmlonly
<div style="float:left; border:0 solid; padding:0; width: 300px; margin:10px;">
\endhtmlonly
\image html xbox360.jpg "Mircrosoft&reg; Xbox 360&reg; controller."
\htmlonly
</div>
\endhtmlonly
\htmlonly
<div style="float:left; border:0 solid; padding:0; width: 300px; margin:10px;">
\endhtmlonly
\section xbox_intro_in Input Features
\termblock
\term \b Feature
  \termdata \b Type
  \termdata \b Min
  \termdata \b Max
\endterm
\term D-Pad Up, Down, Left, Right
  \termdata bi-state
  \termdata 0
  \termdata 1
\endterm
\term Start Button
  \termdata bi-state
  \termdata 0
  \termdata 1
\endterm
\term Center X (LED) Button
  \termdata bi-state
  \termdata 0
  \termdata 1
\endterm
\term Back Button
  \termdata bi-state
  \termdata 0
  \termdata 1
\endterm
\term Left Joystick X Y
  \termdata range
  \termdata -32767
  \termdata 32767
\endterm
\term Left Joystick Click
  \termdata bi-state
  \termdata 0
  \termdata 1
\term Right Joystick X Y
  \termdata range
  \termdata -32767
  \termdata 32767
\endterm
\term Right Joystick Click
  \termdata bi-state
  \termdata 0
  \termdata 1
\endterm
\term Left Trigger
  \termdata range
  \termdata 0
  \termdata 255
\endterm
\term Right Trigger
  \termdata range
  \termdata 0
  \termdata 255
\endterm
\term Left Bump
  \termdata bi-state
  \termdata 0
  \termdata 1
\endterm
\term Right Bump
  \termdata bi-state
  \termdata 0
  \termdata 1
\endterm
\endtermblock

\section xbox_intro_out Output Features
\termblock
\term \b Feature
  \termdata \b Type
  \termdata \b Min
  \termdata \b Max
\endterm
\term Left Low-Frequency Rumble Motor
  \termdata range
  \termdata 0
  \termdata 255
\endterm
\term Right High-Frequency Rumble Motor
  \termdata range
  \termdata 0
  \termdata 255
\endterm
\term LED Pattern
  \termdata enum
  \termdata 0
  \termdata 14
\endterm
\endtermblock
\htmlonly
</div>
\endhtmlonly
 * 
 * \}
 */

/*!
 * \ingroup periph_hid_xbox
 * \defgroup periph_hid_xbox_usb USB Protocol
 * \{
 *
 * Xbox USB protocol interface.
 */ 

/*! USB Microsoft Vendor ID */
#define MICROSOFT_VENDOR_ID       0x045e

/*! USB product ID for USB only Xbox controller */
#define XBOX360_WIRED_PROD_ID     0x028e

/*! USB product ID for wireless Xbox controoler tethered via a charging cable */
#define XBOX360_TETHERED_PROD_ID  0x028f

/*! USB product ID for Microsoft wireless dongle */
#define XBOX360_WIRELESS_PROD_ID      0x0719
#define XBOX360_WIRELESS_WIN_PROD_ID  0x0291

/*! no product id */
#define XBOX360_NO_PROD_ID        0x0000

/*! USB endpoint where current controller state is read from */
#define XBOX360_READ_ENDPOINT     0x81

/*! USB endpoint where controller state is modified */
#define XBOX360_WRITE_ENDPOINT    0x01

/*! USB typical number of interfaces for a microsoft device intended for Xbox */
#define XBOX360_NUM_OF_INTERFACES 4

/*! USB debug level for libusb (values: 0:off, 1:error, 2:warning, 3:info) */
#define XBOX_LIBUSB_DEBUG_OFF     0   ///< libusb logging off
#define XBOX_LIBUSB_DEBUG_ERROR   1   ///< log libusb errors
#define XBOX_LIBUSB_DEBUG_WARN    2   ///< log libusb warnings and errors
#define XBOX_LIBUSB_DEBUG_INFO    3   ///< log libusb info, warnings and errors
#define XBOX_LIBUSB_DEBUG_DFT     XBOX_LIBUSB_DEBUG_OFF
                                      ///< default libusb logging

/*!
 * \}
 */

/*!
 * \ingroup periph_hid_xbox
 * \defgroup periph_hid_xbox_msg Messages
 * \{
 *
 * Xbox360 application message interface.
 */ 

//
// Message offsets in packet.
//
#define XBOX360_MSG_OFFSET_DFT      0   ///< default message start byte offset
#define XBOX360_MSG_OFFSET_W        0   ///< wired message start byte offset
#define XBOX360_MSG_OFFSET_WL       4   ///< wireless message start byte offset
#define XBOX360_WL_PKT_HDR_LEN      XBOX360_MSG_OFFSET_WL
                                        ///< wireless packet header length


// .............................................................................
// Wireless controller packet header and specific packets
// .............................................................................

//
// Packet lengths.
//
#define XBOX360_WL_LINK_PKT_LEN     2   ///< link status change packet length
#define XBOX360_WL_NORM_PKT_LEN     29  ///< normal packet length

//
// Packet type header byte.
//
#define XBOX360_WL_PKT_TYPE_POS     0     ///< byte position
#define XBOX360_WL_PKT_TYPE_NORM    0x00  ///< normal link message 
#define XBOX360_WL_PKT_TYPE_CHG     0x08  ///< link status changed message 
#define XBOX360_WL_PKT_TYPE_20      0x20  ///< ? 20 d7 67 fc 01 88 ff ff 00...
#define XBOX360_WL_PKT_TYPE_40      0x40  ///< ? 40 7c 78 a7 01 88 ff ff 00...
#define XBOX360_WL_PKT_TYPE_60      0x40  ///< ? 60 4e 23 ff 01 88 ff ff 00...
#define XBOX360_WL_PKT_TYPE_80      0x80  ///< ? 80 dc 67 fc 01 88 ff ff 68
                                          ///<   c4 63 ac 01 88 ff ff c4 63
                                          ///<   ac 01 88 ff ff 00...
#define XBOX360_WL_PKT_TYPE_C0      0xc0  ///< ? c0 46 40 ac 01 88 ff ff 68
                                          ///<   2c 39 e0 10 88 ff ff d0 2c
                                          ///<   e9 e0 01 88 ff ff 00...

//
// Link status header byte.
//
// T  body
// 08 status
//
#define XBOX360_WL_LINK_STATUS_POS          1     ///< byte position
#define XBOX360_WL_LINK_STATUS_NO_LINK      0x00  ///< no link
#define XBOX360_WL_LINK_STATUS_HEADSET      0x40  ///< linked w/ headset
#define XBOX360_WL_LINK_STATUS_CTLR         0x80  ///< linked w/ controller
#define XBOX360_WL_LINK_STATUS_CTLR_HEADSET 0xc0  ///< linked w/ ctlr & headset

//
// Announcement header and message packet.
// 
// T  1  2  3  body
// 00 0f 00 f0 s/n battery
//
#define XBOX360_WL_ANN_PAT_1        0x0f    ///< header pattern at byte 1
#define XBOX360_WL_ANN_PAT_2        0x00    ///< header pattern at byte 2 
#define XBOX360_WL_ANN_PAT_3        0xf0    ///< header pattern at byte 3
#define XBOX360_WL_ANN_SN_POS       7       ///< serial number field position
#define XBOX360_WL_ANN_SN_LEN       7       ///< serial number field length
#define XBOX360_WL_ANN_BATT_POS     17      ///< battery status field position
#define XBOX360_WL_ANN_BATT_LEN     1       ///< battery status field length

//
// Battery status header and message packet.
//
// T  1  2  3  body
// 00 00 00 13 battery
//
#define XBOX360_WL_BATT_PAT_1       0x00    ///< header pattern at byte 1
#define XBOX360_WL_BATT_PAT_2       0x00    ///< header pattern at byte 2 
#define XBOX360_WL_BATT_PAT_3       0x13    ///< header pattern at byte 3
#define XBOX360_WL_BATT_BATT_POS    4       ///< battery status field position
#define XBOX360_WL_BATT_BATT_LEN    1       ///< battery status field length

//
// 'Null' status header and message packet. Most are uknown. Byte 3 patterns
// 10, 20, 00, 40, in that order, seem to be ascciated with (re)initial state.
//
// T  1  2  3  body
// 00 00 00 00 62 00 00...
// 00 00 00 10 e2 00 00...
// 00 00 00 20 1d 00 00...
// 00 00 00 40 01 00 00...
// 00 00 00 f0 00 00 00...    set led acknowledge
//
#define XBOX360_WL_NULL_PAT_1       0x00    ///< header pattern at byte 1
#define XBOX360_WL_NULL_PAT_2       0x00    ///< header pattern at byte 2 
#define XBOX360_WL_NULL_PAT_3_00    0x00    ///< header pattern at byte 3
#define XBOX360_WL_NULL_PAT_3_10    0x10    ///< header pattern at byte 3
#define XBOX360_WL_NULL_PAT_3_20    0x20    ///< header pattern at byte 3
#define XBOX360_WL_NULL_PAT_3_40    0x40    ///< header pattern at byte 3
#define XBOX360_WL_NULL_PAT_3_ACK   0xf0    ///< header pattern at byte 3

//
// Button state update header packet.
//
// T  1  2  3  body
// 00 01 00 f0 buttons...
//
#define XBOX360_WL_BTTN_PAT_1       0x01    ///< header pattern at byte 1
#define XBOX360_WL_BTTN_PAT_2       0x00    ///< header pattern at byte 2 
#define XBOX360_WL_BTTN_PAT_3       0xf0    ///< header pattern at byte 3

//
// Rumble state request/response header packet.
//
// T  1  2  3  body 
// 00 01 0f c0 00 left right
//
#define XBOX360_WL_RUMBLE_PAT_1     0x01    ///< header pattern at byte 1
#define XBOX360_WL_RUMBLE_PAT_2     0x0f    ///< header pattern at byte 2 
#define XBOX360_WL_RUMBLE_PAT_3     0xc0    ///< header pattern at byte 3

//
// LED state request/response/update header packet.
//
// T  1  2  3  
// 00 00 08 led
//
#define XBOX360_WL_LED_PAT_1        0x00    ///< header pattern at byte 1
#define XBOX360_WL_LED_PAT_2        0x08    ///< header pattern at byte 2 
#define XBOX360_WL_LED_PAT_3        0x40    ///< header pattern at byte 3

//
// F8 state request/response/update header packet. Appears to be tied to link
// or controller operational state.
//
// T  1  2  3  body
// 00 f8 02 00 00...
//
#define XBOX360_WL_F8_PAT_1         0xf8    ///< header pattern at byte 1
#define XBOX360_WL_F8_PAT_2_1       0x01    ///< header pattern at byte 2 
#define XBOX360_WL_F8_PAT_2_2       0x02    ///< header pattern at byte 2 
#define XBOX360_WL_F8_PAT_2_3       0x03    ///< header pattern at byte 2 
#define XBOX360_WL_F8_PAT_3         0x00    ///< header pattern at byte 3

//
// 32 state request/response/update header packet. Unknown.
//
// T  1  2  3  body
// 00 32 5f fa 01 88 ff ff 50 c8 19 81 ff ff ff ff 30 c8 19 81 ff ff ff ff 40
// f0 0d 81 ff
//
#define XBOX360_WL_32_PAT_1         0x32    ///< header pattern at byte 1
#define XBOX360_WL_32_PAT_2         0x00    ///< header pattern at byte 2?
#define XBOX360_WL_32_PAT_3         0x00    ///< header pattern at byte 3?


// .............................................................................
// Wired controller specific packets
// .............................................................................

// no specifics yet


// .............................................................................
// Common controller messages
// .............................................................................

//
// Message header.
//
#define XBOX360_MSG_TYPE_POS        0       ///< message type byte position
#define XBOX360_MSG_LEN_POS         1       ///< message length byte position

// --
// The messages.  All positions are relative to message start within packet.
// --

// --
// Button state update message.
//
// Button state update message length are different for the wired and 
// wireles interface. However both have trailing padding bytes, so take the
// smallest of the two as the minumum
// --
#define XBOX360_BTTN_MSG_TYPE               0x00    ///< button update msg type
#define XBOX360_BTTN_MSG_LEN                20      ///< wired button msg length
#define XBOX360_WL_BTTN_MSG_LEN             19      ///< wireless button msg len
#define XBOX360_BTTN_MSG_MIN_LEN            XBOX360_WL_BTTN_MSG_LEN // length

// D pad
#define XBOX360_BTTN_PAD_UP_POS             2       ///< byte position
#define XBOX360_BTTN_PAD_UP_MASK            0x01    ///< bit mask

#define XBOX360_BTTN_PAD_DOWN_POS           2       ///< byte position
#define XBOX360_BTTN_PAD_DOWN_MASK          0x02    ///< bit mask

#define XBOX360_BTTN_PAD_LEFT_POS           2       ///< byte position
#define XBOX360_BTTN_PAD_LEFT_MASK          0x04    ///< bit mask

#define XBOX360_BTTN_PAD_RIGHT_POS          2       ///< byte position
#define XBOX360_BTTN_PAD_RIGHT_MASK         0x08    ///< bit mask

// start button
#define XBOX360_BTTN_START_POS              2       ///< byte position
#define XBOX360_BTTN_START_MASK             0x10    ///< bit mask

// back button
#define XBOX360_BTTN_BACK_POS               2       ///< byte position
#define XBOX360_BTTN_BACK_MASK              0x20    ///< bit mask

// left stick click
#define XBOX360_BTTN_LEFT_STICK_CLICK_POS   2       ///< byte position
#define XBOX360_BTTN_LEFT_STICK_CLICK_MASK  0x40    ///< bit mask

// right stick click
#define XBOX360_BTTN_RIGHT_STICK_CLICK_POS  2       ///< byte position
#define XBOX360_BTTN_RIGHT_STICK_CLICK_MASK 0x80    ///< bit mask

// left bump
#define XBOX360_BTTN_LEFT_BUMP_POS          3       ///< byte position
#define XBOX360_BTTN_LEFT_BUMP_MASK         0x01    ///< bit mask

// right bump
#define XBOX360_BTTN_RIGHT_BUMP_POS         3       ///< byte position
#define XBOX360_BTTN_RIGHT_BUMP_MASK        0x02    ///< bit mask

// center X button
#define XBOX360_BTTN_CENTER_X_POS           3       ///< byte position
#define XBOX360_BTTN_CENTER_X_MASK          0x04    ///< bit mask

// A button
#define XBOX360_BTTN_A_BUTTON_POS           3       ///< byte position
#define XBOX360_BTTN_A_BUTTON_MASK          0x10    ///< bit mask

// B button
#define XBOX360_BTTN_B_BUTTON_POS           3       ///< byte position
#define XBOX360_BTTN_B_BUTTON_MASK          0x20    ///< bit mask

// X button
#define XBOX360_BTTN_X_BUTTON_POS           3       ///< byte position
#define XBOX360_BTTN_X_BUTTON_MASK          0x40    ///< bit mask

// Y button
#define XBOX360_BTTN_Y_BUTTON_POS           3       ///< byte position
#define XBOX360_BTTN_Y_BUTTON_MASK          0x80    ///< bit mask

// left trigger
#define XBOX360_BTTN_LEFT_TRIGGER_POS       4       ///< byte position
#define XBOX360_BTTN_LEFT_TRIGGER_MASK      0xff    ///< bit mask

// right trigger
#define XBOX360_BTTN_RIGHT_TRIGGER_POS      5       ///< byte position
#define XBOX360_BTTN_RIGHT_TRIGGER_MASK     0xff    ///< bit mask

#define XBOX360_TRIGGER_MIN                 0       ///< minimum trigger value
#define XBOX360_TRIGGER_MAX                 255     ///< maximum trigger value

// left stick
#define XBOX360_BTTN_LEFT_X_POS_L           6       ///< byte position
#define XBOX360_BTTN_LEFT_X_POS_H           7       ///< byte position
#define XBOX360_BTTN_LEFT_X_MASK            0xffff  ///< bit mask

#define XBOX360_BTTN_LEFT_Y_POS_L           8       ///< byte position
#define XBOX360_BTTN_LEFT_Y_POS_H           9       ///< byte position
#define XBOX360_BTTN_LEFT_Y_MASK            0xffff  ///< bit mask

// right stick
#define XBOX360_BTTN_RIGHT_X_POS_L          10      ///< byte position
#define XBOX360_BTTN_RIGHT_X_POS_H          11      ///< byte position
#define XBOX360_BTTN_RIGHT_X_MASK           0xffff  ///< bit mask

#define XBOX360_BTTN_RIGHT_Y_POS_L          12      ///< byte position
#define XBOX360_BTTN_RIGHT_Y_POS_H          13      ///< byte position
#define XBOX360_BTTN_RIGHT_Y_MASK           0xffff  ///< bit mask

#define XBOX360_JOY_MIN                    -32767   ///< minimum joystick value
#define XBOX360_JOY_MAX                     32767   ///< maximum joystick value

// --
// Rumble motors request/response message
// --
#define XBOX360_RUMBLE_MSG_TYPE     0x00  ///< rumble message type
#define XBOX360_RUMBLE_MSG_LEN      8     ///< rumble read/write message length

#define XBOX360_RUMBLE_ZERO_2       2     ///< byte position with 0 value

// low-frequency left rumble motor
#define XBOX360_RUMBLE_LEFT_POS     3     ///< byte position
#define XBOX360_RUMBLE_LEFT_MASK    0xff  ///< bit mask
#define XBOX360_RUMBLE_LEFT_MAX     255   ///< left rumble motor maximum

// high-frequency right rumble motor
#define XBOX360_RUMBLE_RIGHT_POS    4     ///< byte position
#define XBOX360_RUMBLE_RIGHT_MASK   0xff  ///< bit mask
#define XBOX360_RUMBLE_RIGHT_MAX    255   ///< right rumble motor maximum

// pad
#define XBOX360_RUMBLE_ZERO_5       5     ///< byte position with 0 value
#define XBOX360_RUMBLE_ZERO_6       6     ///< byte position with 0 value
#define XBOX360_RUMBLE_ZERO_7       7     ///< byte position with 0 value


// --
// LED request/response/update pattern message
// --

// wired
#define XBOX360_LED_MSG_TYPE        0x01  ///< LED message type
#define XBOX360_LED_MSG_LEN         3     ///< LED message length
#define XBOX360_LED_PAT_POS         2     ///< LED messge byte position

// wireless
#define XBOX360_WL_LED_MSG_TYPE     0x00  ///< LED message type
#define XBOX360_WL_LED_MSG_LEN      8     ///< LED message length
#define XBOX360_WL_LED_PAT_POS      3     ///< LED position is in PACKET header

// patterns
#define XBOX360_LED_PAT_MASK           0x0f ///< bit mask
#define XBOX360_LED_PAT_ALL_OFF        0    ///< all 4 LEDs off
#define XBOX360_LED_PAT_ALL_BLINK_ON_1 1    ///< all 4 LEDs blink, then #1 on
#define XBOX360_LED_PAT_1_BLINK_ON     2    ///< #1 LED blinks, then #1 on
#define XBOX360_LED_PAT_2_BLINK_ON     3    ///< #2 LED blinks, then #2 on
#define XBOX360_LED_PAT_3_BLINK_ON     4    ///< #3 LED blinks, then #3 on
#define XBOX360_LED_PAT_4_BLINK_ON     5    ///< #4 LED blinks, then #4 on
#define XBOX360_LED_PAT_1_ON           6    ///< #1 LED on
#define XBOX360_LED_PAT_2_ON           7    ///< #2 LED on
#define XBOX360_LED_PAT_3_ON           8    ///< #3 LED on
#define XBOX360_LED_PAT_4_ON           9    ///< #4 LED on
#define XBOX360_LED_PAT_ALL_SPIN       10   ///< spinnig LEDs one at a time
#define XBOX360_LED_PAT_4_BLINK_ON_L   11   ///< #4 LED blinks long, then #4 on
#define XBOX360_LED_PAT_4_BLINK        12   ///< #4 LED blinks slowly
#define XBOX360_LED_PAT_ALL_SPIN_2     13   ///< spinnig LEDs two at a time
                                            ///< auto-transitions to previous
#define XBOX360_LED_PAT_ALL_BLINK      14   ///< all 4 LEDs blink continuously
#define XBOX360_LED_PAT_NUMOF          15   ///< number of patterns

/*!
 * \}
 */

/*!
 * \ingroup periph_hid_xbox
 * \defgroup periph_hid_xbox_tune Tuning
 * \{
 *
 * Xbox tuning heuristics.
 */ 

/*! left joystick deadzone */
#define XBOX360_LEFT_DEAD_ZONE      ((int)(0.25 * XBOX360_JOY_MAX))

/*! right joystick deadzone */
#define XBOX360_RIGHT_DEAD_ZONE     ((int)(0.25 * XBOX360_JOY_MAX))

/*!
 * \}
 */


namespace rnr
{
  /*!
   * \ingroup periph_hid_xbox
   * \brief Xbox360 Feature IDs.
   */
  typedef enum
  {
    Xbox360FeatIdPadUp = 0,         ///< dpad up [0,1]
    Xbox360FeatIdPadDown,           ///< dpad down [0,1]
    Xbox360FeatIdPadLeft,           ///< dpad left [0,1]
    Xbox360FeatIdPadRight,          ///< dpad right [0,1]
    Xbox360FeatIdStart,             ///< start button [0,1]
    Xbox360FeatIdBack,              ///< back button [0,1]
    Xbox360FeatIdLeftStickClick,    ///< left joystick click(left thumb) [0,1]
    Xbox360FeatIdRightStickClick,   ///< right joystick click(right thumb) [0,1]
    Xbox360FeatIdLeftBump,          ///< left bump (left shoulder) [0,1]
    Xbox360FeatIdRightBump,         ///< right bump (right shoulder) [0,1]
    Xbox360FeatIdCenterX,           ///< center x (big button) [0,1]
    Xbox360FeatIdAButton,           ///< A button [0,1]
    Xbox360FeatIdBButton,           ///< B button [0,1]
    Xbox360FeatIdXButton,           ///< X button [0,1]
    Xbox360FeatIdYButton,           ///< Y button [0,1]
    Xbox360FeatIdLeftTrigger,       ///< left trigger [0-255]
    Xbox360FeatIdRightTrigger,      ///< right trigger [0-255]
    Xbox360FeatIdLeftJoyX,          ///< left joystick x value [-32768-32767]
    Xbox360FeatIdLeftJoyY,          ///< left joystick y value [-32768-32767]
    Xbox360FeatIdRightJoyX,         ///< right joystick x value [-32768-32767]
    Xbox360FeatIdRightJoyY,         ///< right joystick x value [-32768-32767]
    Xbox360FeatIdLeftRumble,        ///< left low-freq rumble motor [0-255]
    Xbox360FeatIdRightRumble,       ///< right hi-freq rumble motor [0-255]
    Xbox360FeatIdLEDPat,            ///< LED pattern feature

    Xbox360FeatIdNumOf,             ///< number of features (keep last)
  } Xbox360FeatId;

  /*!
   * \ingroup periph_hid_xbox
   * \brief Xbox360 controller HID input class.
   */
  class HIDXbox360 : public HIDInput
  {
  public:
    /*! \brief Transfer buffer maximum byte size. */
    static const int  UsbTransferBufSize    = 32;

    /*! \brief Consecutive receive timeouts threshold default. */
    static const int  NErrorRcvTimeoutThDft = 0;

    /*! \brief Consecutive receive errors threshold default. */
    static const int  NErrorRcvThDft        = 5;

    /*! \brief Total receive errors threshold default. */
    static const int  NErrorTotalThDft      = 0;

    /*!
     * \brief Update thread states.
     */
    enum ThreadState
    {
      ThreadStateNone,      ///< no thread
      ThreadStateRunning,   ///< thread running
      ThreadStateExit       ///< thread to exit
    };

    /*!
     * \brief USB transfer packet purpose.
     */
    typedef enum
    {
      UsbPktIdInput = 0,    ///< normal input (read) transfer packet
      UsbPktIdRumble,       ///< rumble (write) transfer packet
      UsbPktIdLED,          ///< LED (write) transfer packet
      UsbPktIdGoad,         ///< Goad (write) transfer packet
      UsbPktIdNumOf         ///< number of transfer packet ids
    } UsbPktId;

    /*!
     * \brief USB application packet transfer structure.
     */
    typedef struct
    {
      struct libusb_transfer *m_usbTransfer;  ///< allocated transfer packet
      byte_t  m_bufData[UsbTransferBufSize];  ///< transfer data buffer
      bool    m_bHasSubmitted;                ///< has [not] been submitted 
      bool    m_bCancel;                      ///< do [not] cancel submissions
    } UsbPkt_T;

    /*!
     * \brief Default initialization constructor.
     */
    HIDXbox360(int usbDebugLevel=XBOX_LIBUSB_DEBUG_DFT);

    /*!
     * \brief Destructor.
     */
    ~HIDXbox360();

    // .........................................................................
    // Common HID interface
    // .........................................................................

    /*!
     * \brief Get the value associated with the mapped user mnemonic.
     *
     * \param iMnem       User-specific mnemonic.
     *
     * \return Feature current value.
     */
    virtual int getFeatureVal(int iMnem)
    {
      FeatMap_T::iterator pos;

      if( (pos = m_featMap.find(iMnem)) != m_featMap.end() )
      {
        return m_state[pos->second];
      }
      else
      {
        return -1;
      }
    }

    /*!
     * \brief Set the value associated with the mapped user mnemonic.
     *
     * \param iMnem       User-specific mnemonic.
     * \param [in] nVal   New input HID output value (e.g.LED).
     *                    Meaning is specific to input.
     *
     * \copydoc doc_return_usb
     */
    virtual int setFeatureVal(int iMnem, int nVal);

    /*!
     * \brief Get the feature properties.
     *
     * \param iMnem             User-specific mnemonic.
     * \param [out] eFeatType   Feature property type.
     * \param [out] nDir        Feature is an input (HID_FEAT_INPUT) to host
     *                          and/or an output (HID_FEAT_OUTPUT) to device.
     * \param [out] nMin        Feature minimum value.
     * \param [out] nMax        Feature maximum value.
     * \param [out] nStep       Feature step size between [min,max]
     *
     * \copydoc doc_return_usb
     */
    virtual int getFeatureProp(int          iMnem,
                               HIDFeatType &eFeatType,
                               int         &nDir,
                               int         &nMin,
                               int         &nMax,
                               int         &nStep);

    /*!
     * \brief Get the current full state of the device.
     *
     * \return Return (int *) pointer to the device-specific input state.
     */
    virtual void *getCurrentState() { return m_state; }
  
    /*!
     * \brief Ping device if it is connected and is responding.
     *
     * \note Ping does not work for wireless controllers unless link is up which
     * kind of defeats the purpose of ping.
     *
     * \not Ping is not reliable. Always return true.
     *
     * \return Returns true if the HID is responding, else false.
     */
    virtual bool ping();
  
    /*!
     * \brief Get the string associated with the error number.
     *
     * \param nError    HID-specific error number.
     *
     * \return Null-terminated error string.
     */
    virtual const char *getStrError(int nError) const
    {
      return nError < 0? libusb_error_name(nError): "Ok";
    }
    
    /*!
     * \brief Open connection to an USB Xbox360 controller.
     *
     * The first Xbox found will be opened. All intefaces will be bound to this
     * class instance.
     *
     * \copydoc doc_return_usb
     */
    virtual int open();

    /*!
     * \brief Close connection to an opened USB Xbox360 controller.
     *
     * All interface will be released.
     *
     * \copydoc doc_return_usb
     */
    virtual int close();

    /*!
     * \brief Create and run USB update in thread.
     *
     * \param hz  Update Hertz.
     *
     * \return Returns 0 on success, \h_lt 0 on failure.
     */
    virtual int run(float hz=30.0);

    /*!
     * \brief Stop and destroy USB update thread.
     *
     * \return Returns 0 on success, \h_lt 0 on failure.
     */
    virtual int stop();

    /*!
     * \brief Read device and update HID state.
     *
     * \note The application must call this function at regular intervals to 
     * insure that the USB stream does not stall.
     *
     * \param uMSec   Block wait at most the given milliseconds for events.\n
     *                If zero, then update() will handle any already-pending
     *                events and then immediately return (non-blocking).\n
     *                Otherwise, if no events are currently pending, update()
     *                will block waiting for events for up to the specified
     *                timeout. If an event arrives update() will return early.
     *
     * \copydoc doc_return_std
     */
    virtual int update(uint_t uMSec=T_UPDATE_DFT);

    // .........................................................................
    // Device-specific interface
    // .........................................................................

    /*!
     * \brief Set the rumble instensity.
     *
     * \note The left motor only really works above 255.
     * \note The right motor only really works below 256.
     *
     * \todo Does not work for wireless xbox controller yet.
     *
     * \param nLeftMot    Left, low-frequency motor [0-255].
     * \param nRightMot   Right, high-frequency motor [0-255].
     *
     * \copydoc doc_return_usb
     */
    int setRumble(int nLeftMot, int nRightMot);

    /*!
     * \brief Set the LED pattern.
     *
     * \todo Does not work for wireless xbox controller yet.
     *
     * \param nPattern  LED pattern. See \ref XBOX_LED_PAT_ALL_OFF ...
     *
     * \copydoc doc_return_usb
     */
    int setLED(int nPattern);

    /*!
     * \brief Set joysticks calibration.
     *
     * For each joystick, there exist a deadzone near the center that should 
     * be treated as zero due to hardware induced noise. Each stick output is
     * calibrated as illustrated:
     * \verbatim
     *
     *      Uncalibrated                      DeadZone Calibrated
     *     |                                 |
     * max -          .                  max -          .
     *     |        .                        |         .
     *     |      .                          |       .
     *     |    .             ==>            |      .
     *     |  .                              |     .
     *     |.                                |    .
     *   0 +----------|--                  0 +....------|--
     *               max                        dz     max
     *
     *
     * \endverbatim
     */
    void calibrateJoySticks(int nLeftDeadZone, int nRightDeadZone);

    /*!
     * \brief Set error count thresholds.
     *
     * \param nErrorRcvTimeoutTh  Consecutive receive timeout error count
     *                            threshold. Set to 0 to ignore.
     * \param nErrorRcvTh         Consecutive receive error count threshold.
     *                            Set to 0 to ignore.
     * \param nErrorTotalTh       Total receive error count threshold.
     *                            Set to 0 to ignore.
     */
    void setErrorThresholds(int nErrorRcvTimeoutTh  = NErrorRcvTimeoutThDft,
                            int nErrorRcvTh         = NErrorRcvThDft,
                            int nErrorTotalTh       = NErrorTotalThDft)
    {
      m_nErrorRcvTimeoutTh  = nErrorRcvTimeoutTh;
      m_nErrorRcvTh         = nErrorRcvTh;
      m_nErrorTotalTh       = nErrorTotalTh;
    }

    /*!
     * \brief Check if Xbox360 hardware is wireless.
     *
     * \return Returns true or false.
     */
    bool isWireless()
    {
      return  m_nProdId == XBOX360_WIRELESS_PROD_ID ||
              m_nProdId == XBOX360_WIRELESS_WIN_PROD_ID;
    }

    /*!
     * \brief Simple debug print header.
     */
    void debugPrintHdr();

    /*!
     * \brief Simple debug print state.
     */
    void debugPrintState();

  protected:
    // XBox360 specifics
    int         m_nProdId;                    ///< Xbox USB product id
    std::string m_strProdName;                ///< product name
    std::string m_strSerialNum;               ///< serial number
    int         m_nBatteryStatus;             ///< batter status
    int         m_nStatus;                    ///< raw status byte
    int         m_nOffset;                    ///< start offset in message

    // calibration
    int         m_nLeftJoyDeadZone;           ///< left joystick deadzone
    float       m_fLeftJoyM;                  ///< left joystick slope
    float       m_fLeftJoyB;                  ///< left joystick y intercept
    int         m_nRightJoyDeadZone;          ///< right joystick deadzone
    float       m_fRightJoyM;                 ///< right joystick slope
    float       m_fRightJoyB;                 ///< right joystick y intercept

    // state
    int         m_state[Xbox360FeatIdNumOf];  ///< current state
    int         m_nLEDPattern;                ///< target LED pattern
    int         m_nLeftRumble;                ///< target left rumble intensity
    int         m_nRightRumble;               ///< target right rumble intensity

    // Errors
    int         m_nErrorRcv;                  ///< consecutive rcv error count
    int         m_nErrorRcvTh;                ///< consec rcv error threshold
    int         m_nErrorRcvTimeout;           ///< consec rcv timeout error cnt
    int         m_nErrorRcvTimeoutTh;         ///< consec rcv timeout error th
    int         m_nErrorSend;                 ///< consec send error count
    int         m_nErrorTotal;                ///< total error count
    int         m_nErrorTotalTh;              ///< total error threshold

    // USB
    struct libusb_context        *m_usbContext; ///< libusb session context
    struct libusb_device_handle  *m_usbHandle;  ///< USB open handle
    UsbPkt_T                      m_usbPkt[UsbPktIdNumOf];
                                                ///< USB transfer packets
    // thread
    ThreadState             m_eThreadState; ///< update thread state
    struct timespec         m_tsThread;     ///< update thread period
    pthread_mutex_t         m_mutexUpdate;  ///< mutex
    pthread_cond_t          m_condUpdate;   ///< condition
    pthread_t               m_threadId;     ///< update pthread identifier                                                 
    /*!
     * \brief Set new link state.
     *
     * \param bNewState   New link state.
     */
    virtual void setLinkState(bool bNewState);

    /*!
     * \brief Open Xbox360 connection.
     *
     * Checks for all supported Xbox Controllers and opens the first for
     * communication All interfaces are claimed.
     *
     * \copydoc doc_return_usb
     */
    int openXbox();
    
    /*!
     * \brief Claim all Xbox USB interfaces.
     *
     * Any any kernel claimed drivers will be detached.
     *
     * \copydoc doc_return_usb
     */
    int claimXboxInterfaces();

    /*!
     * \brief Allocate and initialize all USB transfer packets.
     *
     * \copydoc doc_return_usb
     */
    int initTransferPkts();

    /*!
     * \brief Free all USB transfer packets.
     *
     * \copydoc doc_return_usb
     */
    void freeTransferPkts();

    /*!
     * \brief Submit read Xbox input asynchronous transfer.
     *
     * \copydoc doc_return_usb
     */
    int submitReadTransfer();

    /*!
     * \brief Flush any pending input transfer buffers.
     */
    void flushInput();

    /*!
     * \brief Input transfer function callback.
     *
     * \param transfer    USB transfer block structure.
     */
    static void transferCallbackInput(struct libusb_transfer *transfer);
    
    /*!
     * \brief Write rumble motor values transfer function callback.
     *
     * \param transfer    USB transfer block structure.
     */
    static void transferCallbackRumble(struct libusb_transfer *transfer);

    /*!
     * \brief Write LED pattern transfer function callback.
     *
     * \param transfer    USB transfer block structure.
     */
    static void transferCallbackLED(struct libusb_transfer *transfer);

    /*!
     * \brief Write packet to force a response packet from the Xbox360.
     *
     * Note that the response is captured in the input transfer.
     *
     * \param transfer    USB transfer block structure.
     */
    static void transferCallbackGoad(struct libusb_transfer *transfer);

    /*!
     * \brief Cancel all pending USB transfers.
     */
    void cancelTransfers();

    /*!
     * \brief Goad the Xbox360 to send a message.
     *
     * Are you still linked?
     */
    void goad();

    /*!
     * \brief Parse wireless Xbox360 input events.
     *
     * While executing, the Xbox controller can be unplugged. For a wireless
     * connection, the dongle - Xbox controller can become unpaired,
     * out-of-range, or wonking connection.
     *
     * \param buf   Raw USB input buffer.
     * \param n     Length of data in buffer.
     *
     * \return Returns true if good parse, else false.
     */
    bool parseWireless(byte_t buf[], ssize_t n);

    /*!
     * \brief Parse wired Xbox360 input events.
     *
     * \param buf   Raw USB input buffer.
     * \param n     Length of data in buffer.
     *
     * \return Returns true if good parse, else false.
     */
    bool parseWired(byte_t buf[], ssize_t n);

    /*!
     * \brief Update the controller input button state.
     *
     * \param msg   Message containing raw controller state data.
     * \param n     Length of data in message.
     */
    void updateButtonState(byte_t msg[], ssize_t n);

    /*!
     * \brief Update the rumble motors state.
     *
     * \param msg   Message containing raw controller status data.
     * \param n     Length of data in message.
     */
    void updateRumbleState(byte_t msg[], ssize_t n);

    /*!
     * \brief Update the LED pattern state.
     *
     * \note Wireless controller has LED state in the packet header for
     * some idiotic reason.
     *
     * \param buf   Raw USB input buffer.
     * \param n     Length of data in buffer.
     */
    void updateLEDState(byte_t buf[], ssize_t n);

    /*!
     * \brief Clear error counts.
     */
    void clearErrorCnts()
    {
      m_nErrorRcvTimeout  = 0;
      m_nErrorRcv         = 0;
      m_nErrorSend        = 0;
      m_nErrorTotal       = 0;
    }

    /*!
     * \brief Check errors against thresholds.
     *
     * If any of the error counts exceed their respective thresholds, then
     * the connection state is automatically transitioned to the 'no link'
     * state.
     *
     * \return Returns true if link is good, else false.
     */
    bool checkErrorThresholds();

    /*!
     * \brief Zero controller shadowed state.
     */
    void zeroState()
    {
      memset(m_state, 0, sizeof(m_state));
    }

    /*!
     * \brief Convert bit value to button up/down state.
     *
     * \param nBit  Bit value.
     *
     * \return HID_BTTN_UP or HID_BTTN_DOWN
     */
    int updown(int nBit)
    {
      return nBit != 0? HID_BTTN_DOWN: HID_BTTN_UP;
    }

    /*!
     * \brief Convert two bytes to signed integer.
     *
     * \param lsb Least significant byte.
     * \param msb Most significant byte.
     *
     * \return Converted integer value.
     */
    int convertToInt(byte_t lsb, byte_t msb);

    /*!
     * \brief Create USB update thread.
     *
     * \return Returns 0 on success, \h_lt 0 on failure.
     */
    int createUpdateThread();

    /*!
     * \brief Cancel (destroy) USB update thread.
     *
     * \return Returns 0 on success, \h_lt 0 on failure.
     */
    int cancelUpdateThread();

    /*!
     * \brief Lock update thread.
     *
     * The calling thread will block while waiting for the mutex to become 
     * available. Once locked, the update thread will block.
     *
     * The lock()/unlock() primitives provide a safe mechanism to modify state. 
     *
     * \par Context:
     * Any.
     */
    void lock()
    {
      pthread_mutex_lock(&m_mutexUpdate);
    }

    /*!
     * \brief Unlock the update thread.
     *
     * The update() will be available to execute.
     *
     * \par Context:
     * Any.
     */
    void unlock()
    {
      pthread_mutex_unlock(&m_mutexUpdate);
    }
    
    /*!
     * \brief Signal update thread of change of state.
     *
     * \par Context:
     * Calling thread or Update thread.
     */
    void signalUpdateThread();

    /*!
     * \brief Blocked timed wait.
     *
     * \par Context:
     * Update thread.
     */
    void blockWait();
    
    /*!
     * \brief The update thread.
     *
     * \param pArg    Point to this class instance.
     *
     * \return NULL
     */
    static void *updateThread(void *pArg);
    
    /*!
     * \brief Debug print USB transfer buffer contents.
     *
     * \param strPreface    User identifying preface string.
     * \param tranfer       Pointer to USB transfer buffer.
     */
    void debugPrintTransferBuf(const std::string      &strPreface,
                               struct libusb_transfer *transfer)
    {
      debugPrintTransferBuf(strPreface, transfer->buffer,
                                        transfer->actual_length);
    }

    /*!
     * \brief Debug print USB transfer buffer contents.
     *
     * \param strPreface    User identifying preface string.
     * \param buf           USB data buffer.
     * \param n             USB data buffer length
     */
    void debugPrintTransferBuf(const std::string  &strPreface,
                               byte_t              buf[],
                               ssize_t             n,
                               const std::string  &strEoR="\n");

  };
} // namespace


#endif // _HIDXBOX360_H
