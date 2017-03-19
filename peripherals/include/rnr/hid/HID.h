////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Peripherals
//
// Library:   libhid
//
// File:      HID.h
//
/*! \file
 *
 * $LastChangedDate: 2015-02-25 14:46:44 -0700 (Wed, 25 Feb 2015) $
 * $Rev: 3872 $
 *
 * \brief Common Human Interface Device Interface.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2012-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
 */
/*
 * @EulaBegin@
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////

#ifndef _HID_H
#define _HID_H

#include <string>
#include <map>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"


/*!
 * \ingroup periph_hid
 * \defgroup periph_hid_base HID Abstract Interfaces
 *
 * \{
 *
 * Human Interface Device abstract interfaces.
 */

//
// Common HID feature states
//
#define HID_BTTN_UP     0     ///< button/key state is up or unpressed
#define HID_BTTN_DOWN   1     ///< button/key state is down or pressed

#define HID_FEAT_INPUT  0x01  ///< input to host feature
#define HID_FEAT_OUTPUT 0x02  ///< output to device feature
/*!
 * \}
 */


/*!
 * \brief RoadNarrows Robotics standard namespace.
 */
namespace rnr
{
  /*!
   * \ingroup periph_hid_base
   * \brief Supported HID classes.
   */
  typedef enum
  {
    HIDClassUnknown = 0,      ///< class is unknown or not initialized

    HIDClassKeyboardMouse,    ///< keyboards/mice class
    HIDClassXbox360,          ///< Microsoft Xbox360 game console family

    HIDClassNumOf             ///< number of supported classes (keep last)
  } HIDClass;

  /*!
   * \ingroup periph_hid_base
   * \brief Feature property types.
   */
  typedef enum
  {
    HIDFeatTypeUnknown = 0,     ///< unknown feature property type

    HIDFeatTypeBiState,         ///< binary state property type
    HIDFeatTypeRange,           ///< range between [min,max] property type
    HIDFeatTypeEnum,            ///< discrete enumeration feature property type

    HIDFeatTypeNumOf            ///< number of feature property types(keep last)
  } HIDFeatType;

  /*!
   * \ingroup periph_hid_base
   * \brief Input Human Interface Device Abstract Base Class.
   */
  class HIDInput
  {
  public:
    static const int MNEM_START = 1024; ///< user mapped mnemonic starting index
    static const int T_UPDATE_DFT = 33; ///< default update timeout (msec)

    typedef std::map<int, int> FeatMap_T;  ///< device feature id - mnemonic map

    /*!
     * \brief Default initialization constructor.
     *
     * \param eHIDClass   Actual input HID class.
     */
    HIDInput(HIDClass eHIDClass=HIDClassUnknown)
      : m_eHIDClass(eHIDClass),
        m_bIsConnected(false),
        m_bIsLinked(false),
        m_nError(0),
        m_bFatal(false)
    {
    }

    /*!
     * \brief Destructor.
     */
    virtual ~HIDInput() { }

    /*!
     * \brief Open connection to HID.
     * 
     * \copydoc doc_return_std
     */
    virtual int open() = 0;

    /*!
     * \brief Close connection to HID.
     *
     * \copydoc doc_return_std
     */
    virtual int close() = 0;

    /*!
     * \brief Read device and update HID state.
     *
     * \param uMSec   Block wait at most the given milliseconds for events.\n
     *                If zero, then update() will handle any already-pending
     *                events and then immediately return (non-blocking).\n
     *                Otherwise, if no events are currently pending, update()
     *                will block waiting for events for up specified timeout.
     *                If an event arrives update() will return early.
     *
     * \copydoc doc_return_std
     */
    virtual int update(uint_t uMSec=T_UPDATE_DFT) = 0;

    /*!
     * \brief Get the HID class.
     *
     * \return Returns \ref HIDClass
     */
    virtual HIDClass getHIDClass()  { return m_eHIDClass; }

    /*!
     * \brief Query if HID is connected.
     *
     * \return Returns true if (physically) connected, else false.
     */
    virtual bool isConnected()  { return m_bIsConnected; }

    /*!
     * \brief Query if HID is linked.
     *
     * \return Returns true if linked and responding, else false.
     */
    virtual bool isLinked()  { return m_bIsLinked; }

    /*!
     * \brief Associate user mnemonic to the device feature (e.g. button) id.
     *
     * \param iFeatId   Device-specific feature id (or previous mnemonic).
     * \param iMnem     User-specific mnemonic.
     *
     * \copydoc doc_return_std
     */
    virtual int assocFeature(int iFeatId, int iMnem)
    {
      FeatMap_T::iterator pos;
      int                 iFeatIdFixed;

      if( (pos = m_featMap.find(iFeatId)) != m_featMap.end() )
      {
        iFeatIdFixed = pos->second;
        m_featMap.erase(pos);
        m_featMap[iMnem] = iFeatIdFixed;
        return OK;
      }
      else
      {
        return RC_ERROR;
      }
    }

    /*!
     * \brief Get the value associated with the mapped user mnemonic.
     *
     * \param iMnem       User-specific mnemonic.
     *
     * \return Feature current value.
     */
    virtual int getFeatureVal(int iMnem) = 0;

    /*!
     * \brief Set the value associated with the mapped user mnemonic.
     *
     * \param iMnem       User-specific mnemonic.
     * \param [in] nVal   New input HID output value (e.g.LED).
     *                    Meaning is specific to input.
     *
     * \copydoc doc_return_std
     */
    virtual int setFeatureVal(int iMnem, int nVal) = 0;

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
     * \copydoc doc_return_std
     */
    virtual int getFeatureProp(int          iMnem,
                               HIDFeatType &eFeatType,
                               int         &nDir,
                               int         &nMin,
                               int         &nMax,
                               int         &nStep) = 0;

    /*!
     * \brief Get the current full state of the device.
     *
     * \return Return pointer to the device-specific input state.
     */
    virtual void *getCurrentState() { return NULL; }
  
    /*!
     * \brief Ping device if it is connected and is responding.
     *
     * \return Returns true if the HID is responding, else false.
     */
    virtual bool ping() { return false; }
  
    /*!
     * \brief Test to see if HID is in a fatal errored state.
     *
     * \return Returns true if fatal, else false.
     */
    virtual bool isFatal() const { return m_bFatal; }

    /*!
     * \brief Get the last HID-specific error number.
     *
     * Errors are \h_lt 0, no error is 0.
     *
     * \return Returns HID-specific error number.
     */
    virtual int getError() const { return m_nError; } 

    /*!
     * \brief Get the string associated with the error number.
     *
     * \param nError    HID-specific error number.
     *
     * \return Null-terminated error string.
     */
    virtual const char *getStrError(int nError) const
    {
      return nError < 0? "Error": "";
    }

    /*!
     * \brief Get HID product name.
     *
     * \return String.
     */
    virtual std::string getProdName()
    {
      return m_strProdName;
    }

  protected:
    HIDClass    m_eHIDClass;    ///< actual HID class
    std::string m_strProdName;  ///< product name
    bool        m_bIsConnected; ///< HID is [not] physically connected
    bool        m_bIsLinked;    ///< HID is [not] linked and communicating
    int         m_nError;       ///< last error number (HID specific)
    bool        m_bFatal;       ///< HID is [not] in a fatal condition
    FeatMap_T   m_featMap;      ///< feature-userid association map

    /*!
     * \brief Set connection state.
     *
     * \param bNewState   New connection state.
     */
    virtual void setConnectionState(bool bNewState)
    {
      if( bNewState != m_bIsConnected )
      {
        LOGDIAG3("HID is %s.", (bNewState? "connected": "not connected"));
      }
      m_bIsConnected = bNewState;
      if( !m_bIsConnected )
      {
        setLinkState(false);
      }
    }

    /*!
     * \brief Set new link state.
     *
     * \param bNewState   New link state.
     */
    virtual void setLinkState(bool bNewState)
    {
      if( bNewState != m_bIsLinked )
      {
        LOGDIAG3("HID is %s.", (bNewState? "linked": "not linked"));
      }
      m_bIsLinked = bNewState;
    }
  };
} // namespace rnr


#endif // _HID_H
