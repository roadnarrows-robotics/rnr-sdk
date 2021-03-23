////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Library:   libhekateros
//
// File:      hekOptical.h
//
/*! \file
 *
 * $LastChangedDate: 2014-09-18 16:53:49 -0600 (Thu, 18 Sep 2014) $
 * $Rev: 3748 $
 *
 * \brief \h_hek optical limit switches.
 *
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2013-2017. RoadNarrows LLC.\n
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

#ifndef _HEK_OPTICAL_H
#define _HEK_OPTICAL_H

#include "rnr/rnrconfig.h"

#include "Dynamixel/Dynamixel.h"

#include "Hekateros/hekateros.h"

namespace hekateros
{
  /*!
   * \ingroup hek_spec
   * \defgroup hek_optical_spec  Hekateros Optical Limits
   *
   * All optical joint limits are feed into an \h_i2c I/O expander. In addition
   * two input/ouput user-defined values are available at the end effector
   * tool zero point.
   * \{
   */
  static const byte_t HekIOExpI2CAddr = 0x20;         ///< i2c 7-bit address

  // I/O expander commands
  static const byte_t HekIOExpCmdInput0     = 0;  ///< read port 0 command
  static const byte_t HekIOExpCmdInput1     = 1;  ///< read port 1 command
  static const byte_t HekIOExpCmdOutput0    = 2;  ///< write port 0 command
  static const byte_t HekIOExpCmdOutput1    = 3;  ///< write port 1 command
  static const byte_t HekIOExpCmdPolarity0  = 4;  ///< polarity inversion port 0
  static const byte_t HekIOExpCmdPolarity1  = 5;  ///< polarity inversion port 1
  static const byte_t HekIOExpCmdConfig0    = 6;  ///< configuration port 0 cmd
  static const byte_t HekIOExpCmdConfig1    = 7;  ///< configuration port 1 cmd
  
  // I/O expander special values
  static const byte_t HekIOExpUnassigned      = 0;    ///< unassigned port bit
  static const byte_t HekIOExpDark            = 0;    ///< optic switch blocked
  static const byte_t HekIOExpLight           = 0xff; ///< optical switch is lit
  static const byte_t HekIOExpConstPolarity0  = 0x00; ///< port 0 not inverted
  static const byte_t HekIOExpConstConfig0    = 0xff; ///< port 0 all input

  // I/O expander port 0 bits
#if 0 //DHP - hack
  static const byte_t HekIOExpPort0Base0      = 0x20; ///< base 0\h_deg limit
  static const byte_t HekIOExpPort0Base180    = 0x10; ///< base 180\h_deg lim.
  static const byte_t HekIOExpPort0Shoulder   = 0x08; ///< shoulder limit
  static const byte_t HekIOExpPort0Elbow      = 0x04; ///< elbow limit
  static const byte_t HekIOExpPort0WristPitch = 0x02; ///< wrist pitch limit
  static const byte_t HekIOExpPort0WristRot0  = 0x01; ///< wrist rot 0\h_deg
  static const byte_t HekIOExpPort0Rsrv1      = 0x40; ///< reserved
  static const byte_t HekIOExpPort0Rsrv2      = 0x80; ///< reserved
#endif
  //DHP - back to normal?
  static const byte_t HekIOExpPort0Base0      = 0x01; ///< base 0\h_deg limit
  static const byte_t HekIOExpPort0Base180    = 0x02; ///< base 180\h_deg lim.
  static const byte_t HekIOExpPort0Shoulder   = 0x04; ///< shoulder limit
  static const byte_t HekIOExpPort0Elbow      = 0x08; ///< elbow limit
  static const byte_t HekIOExpPort0WristPitch = 0x10; ///< wrist pitch limit
  static const byte_t HekIOExpPort0WristRot0  = 0x20; ///< wrist rot 0\h_deg
  static const byte_t HekIOExpPort0Rsrv1      = 0x40; ///< reserved
  static const byte_t HekIOExpPort0Rsrv2      = 0x80; ///< reserved

  // I/O expander port 1 bits
  static const byte_t HekIOExpPort1EEUser1    = 0x01; ///< user defined 1
  static const byte_t HekIOExpPort1EEUser2    = 0x02; ///< user defined 2
  static const byte_t HekIOExpPort1Rsrv1      = 0x04; ///< reserved
  static const byte_t HekIOExpPort1Rsrv2      = 0x08; ///< reserved
  static const byte_t HekIOExpPort1Rsrv3      = 0x10; ///< reserved
  static const byte_t HekIOExpPort1Rsrv4      = 0x20; ///< reserved
  static const byte_t HekIOExpPort1Rsrv5      = 0x40; ///< reserved
  static const byte_t HekIOExpPort1Rsrv6      = 0x80; ///< reserved

  // Sizes
  static const int HekOptLimitMaxPerJoint     = 2;    ///< max limits/joint
  static const int HekOptLimitMaxEdges        = 2;    ///< max edges/limit


  // ---------------------------------------------------------------------------
  // Struct HekOpticalLimit_T
  // ---------------------------------------------------------------------------

  /*!
   * \brief Optical limit switch.
   *
   * An optical limit switch is a binary valued detector that is either
   * completely on ("closed") or completely off ("open").
   * The switch is composed of an IR transmitter and a photodiode. When the 
   * photodiode detects the IR beam, the circuit is closed. When the beam is
   * blocked (occluded)the circuit is open. For \h_hek, the occluson bands serve
   * one of two purposes:
   * \li For joints with limited rotation, the band marks the arc of valid
   * rotation.
   * \li For continously rotating joints, the band(s) mark key joint reference
   * points such as top-dead-center (0\h_deg).
   *
   * For each \h_hek joint with optical limits, a physical occlusion band is
   * integrated into robotic structure.
   *
   * To read the state of an optical switch, the signal is feed to an \h_i2c
   * enabled I/O expander. \h_hek software can read the values via \h_i2c.
   */
  struct HekOpticalLimit_T
  {
    /*!
     * \brief Assignment operator.
     *
     * \param rhs   Right hand side object.
     *
     * \return this.
     */
    HekOpticalLimit_T &operator=(const HekOpticalLimit_T &rhs)
    {
      m_uBit          = rhs.m_uBit;
      m_fMinEdgePos   = rhs.m_fMinEdgePos;
      m_fMinBlackPos  = rhs.m_fMinBlackPos;
      m_fCenterPos    = rhs.m_fCenterPos;
      m_fMaxBlackPos  = rhs.m_fMaxBlackPos;
      m_fMaxEdgePos   = rhs.m_fMaxEdgePos;

      return *this;
    }

    void clear()
    {
      m_uBit          = 0x00;
      m_fMinEdgePos   = 0.0;
      m_fMinBlackPos  = 0.0;
      m_fCenterPos    = 0.0;
      m_fMaxBlackPos  = 0.0;
      m_fMaxEdgePos   = 0.0;
    }

    byte_t  m_uBit;           ///< i/o expander bit position
    double  m_fMinEdgePos;    ///< mininum edge position of occlusion band
    double  m_fMinBlackPos;   ///< minimum complete occlusion position
    double  m_fCenterPos;     ///< center of operation position
    double  m_fMaxBlackPos;   ///< maximum complete occlusion position
    double  m_fMaxEdgePos;    ///< maxinum edge position of occlusion band
  };

  /*!
   * \brief Test if any of the optical limits have been triggered (occluded).
   *
   * \param byBits    I/O expander mapped optical limit switches.
   * \param byMask    Mask of limits to check.
   *
   * \return Returns bit map where 1 at a given mapped position indicates
   * the optical limit has been triggered, 0 otherwise.
   */
  static byte_t getDarkOpticalLimits(byte_t byBits, byte_t byMask)
  {
    //fprintf(stderr, "dhp: dark: raw bits, mask = %02x, %02x\n", byBits, byMask);
    return ~byBits & byMask;
  }

  /*!
   * \brief Test if any of the optical limits are lit (not occluded).
   *
   * \param byBits    I/O expander mapped optical limit switches.
   * \param byMask    Mask of limits to check.
   *
   * \return Returns bit map where 1 at a given mapped position indicates
   * the optical limit is lit, 0 otherwise.
   */
  static byte_t getLitOpticalLimits(byte_t byBits, byte_t byMask)
  {
    //fprintf(stderr, "dhp: lit: raw bits, mask = %02x, %02x\n", byBits, byMask);
    return byBits & byMask;
  }

  /*! \} */

} // namespace hekateros


#endif // _HEK_OPTICAL_H
