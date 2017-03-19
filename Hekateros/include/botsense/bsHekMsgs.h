///////////////////////////////////////////////////////////////////////////////
//
// File: bsHekMsgs.h
//
/*!
 * \file
 *
 * \brief \h_botsense Server/Client Hekateros NetMsgs XML Definitions.
 *
 * \warning This file was auto-generated on 2012.11.10 09:12:10 from the NetMsgs
 * XML specification bsHekMsgs.xml.
 *
 * \copyright
 *   \h_copy 2012-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
 */
///////////////////////////////////////////////////////////////////////////////


#ifndef _BSHEKMSGS_H
#define _BSHEKMSGS_H

#include "rnr/rnrconfig.h"
#include "rnr/netmsgs.h"

#include "botsense/BotSense.h"
#include "Dynamixel/Dynamixel.h"


C_DECLS_BEGIN

/*!
 * BsHek Message Id Enumeration
 */
typedef enum
{
  BsHekMsgIdNone                        = 0,    ///< no message
  BsHekMsgIdReqOpenArgs                 = 1,    ///< ReqOpenArgs
  BsHekMsgIdReqGetVersion               = 2,    ///< ReqGetVersion
  BsHekMsgIdRspGetVersion               = 3,    ///< RspGetVersion
  BsHekMsgIdReqMoveAtSpeedTo            = 4,    ///< ReqMoveAtSpeedTo
  BsHekMsgIdReqGetState                 = 5,    ///< ReqGetState
  BsHekMsgIdReqFreeze                   = 6,    ///< ReqFreeze
  BsHekMsgIdReqEStop                    = 7,    ///< ReqEStop
  BsHekMsgIdReqCalibrate                = 8,    ///< ReqCalibrate
  BsHekMsgIdRspState                    = 9,    ///< RspState
  BsHekMsgIdReqGetHealth                = 10,   ///< ReqGetHealth
  BsHekMsgIdRspGetHealth                = 11,   ///< RspGetHealth
  BsHekMsgIdReqClearAlarms              = 12,   ///< ReqClearAlarms
  BsHekMsgIdNumOf                       = 13    ///< number of message ids
} BsHekMsgId_T;


//-----------------------------------------------------------------------------
// Extended Field Types
//-----------------------------------------------------------------------------


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Field Type BsHekServoState Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * ServoState Field Type
 */
typedef struct
{
  byte_t                  m_servo_id;         ///< servo_id
  short                   m_goal_speed;       ///< goal_speed
  short                   m_goal_pos;         ///< goal_pos
} BsHekServoState_T;                          ///< ServoState structure


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Field Type BsHekServoStateVec Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * ServoStateVec Field Type
 */
typedef struct
{
  size_t                  m_count;            ///< vector item count
  union
  {
    void                  *m_pAlign;          ///< force alignment
    BsHekServoState_T     m_buf[DYNA_ID_NUMOF];
                                              ///< the item vector
  } u; ///< aligned vector items
} BsHekServoStateVec_T;                       ///< vector


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Field Type BsHekServoIdVec Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * ServoIdVec Field Type
 */
typedef struct
{
  size_t                  m_count;            ///< vector item count
  union
  {
    void                  *m_pAlign;          ///< force alignment
    byte_t                m_buf[DYNA_ID_NUMOF];
                                              ///< the item vector
  } u; ///< aligned vector items
} BsHekServoIdVec_T;                          ///< vector


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Field Type BsHekServoHealth Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * ServoHealth Field Type
 */
typedef struct
{
  byte_t                  m_servo_id;         ///< servo_id
  byte_t                  m_alarms;           ///< alarms
  short                   m_load;             ///< load
  ushort_t                m_volts;            ///< volts
  ushort_t                m_temp;             ///< temp
} BsHekServoHealth_T;                         ///< ServoHealth structure


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Field Type BsHekServoHealthVec Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * ServoHealthVec Field Type
 */
typedef struct
{
  size_t                  m_count;            ///< vector item count
  union
  {
    void                  *m_pAlign;          ///< force alignment
    BsHekServoHealth_T    m_buf[DYNA_ID_NUMOF];
                                              ///< the item vector
  } u; ///< aligned vector items
} BsHekServoHealthVec_T;                      ///< vector


//-----------------------------------------------------------------------------
// Message Types
//-----------------------------------------------------------------------------


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message BsHekReqOpenArgs Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * Message ReqOpenArgs Structure
 */
typedef struct
{
  uint_t                  m_baudrate;         ///< baudrate
} BsHekReqOpenArgs_T;                         ///< ReqOpenArgs structure


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message BsHekRspGetVersion Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*! RspGetVersion version maximum string length */
#define BSHEK_RSPGETVERSION_VERSION_LEN (NMFVAL_LEN_MAX_STRING)

/*!
 * Message RspGetVersion Structure
 */
typedef struct
{
  char                    m_version[BSHEK_RSPGETVERSION_VERSION_LEN+1];
                                              ///< version
} BsHekRspGetVersion_T;                       ///< RspGetVersion structure


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message BsHekReqMoveAtSpeedTo Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * Message ReqMoveAtSpeedTo Structure
 */
typedef struct
{
  BsHekServoStateVec_T    m_move;             ///< move
} BsHekReqMoveAtSpeedTo_T;                    ///< ReqMoveAtSpeedTo structure


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message BsHekRspState Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * Message RspState Structure
 */
typedef struct
{
  BsHekServoStateVec_T    m_state;            ///< state
} BsHekRspState_T;                            ///< RspState structure


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message BsHekReqGetHealth Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * Message ReqGetHealth Structure
 */
typedef struct
{
  BsHekServoIdVec_T       m_servo_id;         ///< servo_id
} BsHekReqGetHealth_T;                        ///< ReqGetHealth structure


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message BsHekRspGetHealth Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * Message RspGetHealth Structure
 */
typedef struct
{
  BsHekServoHealthVec_T   m_health;           ///< health
} BsHekRspGetHealth_T;                        ///< RspGetHealth structure


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message BsHekReqClearAlarms Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * Message ReqClearAlarms Structure
 */
typedef struct
{
  BsHekServoIdVec_T       m_servo_id;         ///< servo_id
} BsHekReqClearAlarms_T;                      ///< ReqClearAlarms structure


//-----------------------------------------------------------------------------
// External Data
//-----------------------------------------------------------------------------

//
// BsHek Message Definition Look-Up Table.
// (indexed by BsHekMsgId_T enum)
// 
extern const NMMsgDef_T *BsHekMsgDefLookupTbl[];

//
// BsHek Maximum Message Body Length (bytes) Look-Up Table.
// (indexed by BsHekMsgId_T enum)
// 
extern size_t BsHekMsgMaxLenLookupTbl[];


//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------

extern const NMMsgDef_T * BsHekLookupMsgDef( BsHekMsgId_T eMsgId );

extern size_t BsHekLookupMsgMaxLen( BsHekMsgId_T eMsgId );

extern int BsHekPackMsg( BsHekMsgId_T eMsgId,
                         void * pStruct,
                         byte_t buf[],
                         size_t bufSize,
                         bool_t bTrace );

extern int BsHekUnpackMsg( BsHekMsgId_T eMsgId,
                           byte_t buf[],
                           size_t uMsgLen,
                           void * pStruct,
                           bool_t bTrace );

/*!
 * \brief Pack a BsHekReqOpenArgs ITV message in big-endian byte order
 *        into the output buffer.
 * 
 * \param [in] pStruct    Pointer to the associated, populated message
 *                        structure.
 * \param [out] buf       Output message buffer.
 * \param bufSize         Size of output buffer.
 * \param bTrace          Do [not] trace packing.
 * 
 * \return
 * On success, returns the number of bytes packed.
 * On error, returns the appropriate \h_lt 0 negated NM_ECODE.
 */
INLINE_IN_H int BsHekPackReqOpenArgs( BsHekReqOpenArgs_T * pStruct,
                                      byte_t buf[],
                                      size_t bufSize,
                                      bool_t bTrace )
{
  return BsHekPackMsg(BsHekMsgIdReqOpenArgs, pStruct, buf, bufSize, bTrace);
}

/*!
 * \brief Unpack a BsHekReqOpenArgs ITV message in big-endian byte order
 *        from the input buffer.
 * 
 * \param [in] buf        Output message buffer.
 * \param uMsgLen         Length of message (bytes) in input buffer.
 * \param [out] pStruct   Pointer to the associated message structure.
 * \param bTrace          Do [not] trace packing.
 * 
 * \return
 * On success, returns the number of bytes unpacked.
 * On error, returns the appropriate \h_lt 0 negated NM_ECODE.
 */
INLINE_IN_H int BsHekUnpackReqOpenArgs( byte_t buf[],
                                        size_t uMsgLen,
                                        BsHekReqOpenArgs_T * pStruct,
                                        bool_t bTrace )
{
  return BsHekUnpackMsg(BsHekMsgIdReqOpenArgs, buf, uMsgLen, pStruct, bTrace);
}

/*!
 * \brief Pack a BsHekRspGetVersion ITV message in big-endian byte order
 *        into the output buffer.
 * 
 * \param [in] pStruct    Pointer to the associated, populated message
 *                        structure.
 * \param [out] buf       Output message buffer.
 * \param bufSize         Size of output buffer.
 * \param bTrace          Do [not] trace packing.
 * 
 * \return
 * On success, returns the number of bytes packed.
 * On error, returns the appropriate \h_lt 0 negated NM_ECODE.
 */
INLINE_IN_H int BsHekPackRspGetVersion( BsHekRspGetVersion_T * pStruct,
                                        byte_t buf[],
                                        size_t bufSize,
                                        bool_t bTrace )
{
  return BsHekPackMsg(BsHekMsgIdRspGetVersion, pStruct, buf, bufSize, bTrace);
}

/*!
 * \brief Unpack a BsHekRspGetVersion ITV message in big-endian byte order
 *        from the input buffer.
 * 
 * \param [in] buf        Output message buffer.
 * \param uMsgLen         Length of message (bytes) in input buffer.
 * \param [out] pStruct   Pointer to the associated message structure.
 * \param bTrace          Do [not] trace packing.
 * 
 * \return
 * On success, returns the number of bytes unpacked.
 * On error, returns the appropriate \h_lt 0 negated NM_ECODE.
 */
INLINE_IN_H int BsHekUnpackRspGetVersion( byte_t buf[],
                                          size_t uMsgLen,
                                          BsHekRspGetVersion_T * pStruct,
                                          bool_t bTrace )
{
  return BsHekUnpackMsg(BsHekMsgIdRspGetVersion, buf, uMsgLen, pStruct, bTrace);
}

/*!
 * \brief Pack a BsHekReqMoveAtSpeedTo ITV message in big-endian byte order
 *        into the output buffer.
 * 
 * \param [in] pStruct    Pointer to the associated, populated message
 *                        structure.
 * \param [out] buf       Output message buffer.
 * \param bufSize         Size of output buffer.
 * \param bTrace          Do [not] trace packing.
 * 
 * \return
 * On success, returns the number of bytes packed.
 * On error, returns the appropriate \h_lt 0 negated NM_ECODE.
 */
INLINE_IN_H int BsHekPackReqMoveAtSpeedTo( BsHekReqMoveAtSpeedTo_T * pStruct,
                                           byte_t buf[],
                                           size_t bufSize,
                                           bool_t bTrace )
{
  return BsHekPackMsg(BsHekMsgIdReqMoveAtSpeedTo, pStruct, buf, bufSize, bTrace);
}

/*!
 * \brief Unpack a BsHekReqMoveAtSpeedTo ITV message in big-endian byte order
 *        from the input buffer.
 * 
 * \param [in] buf        Output message buffer.
 * \param uMsgLen         Length of message (bytes) in input buffer.
 * \param [out] pStruct   Pointer to the associated message structure.
 * \param bTrace          Do [not] trace packing.
 * 
 * \return
 * On success, returns the number of bytes unpacked.
 * On error, returns the appropriate \h_lt 0 negated NM_ECODE.
 */
INLINE_IN_H int BsHekUnpackReqMoveAtSpeedTo( byte_t buf[],
                                             size_t uMsgLen,
                                             BsHekReqMoveAtSpeedTo_T * pStruct,
                                             bool_t bTrace )
{
  return BsHekUnpackMsg(BsHekMsgIdReqMoveAtSpeedTo, buf, uMsgLen, pStruct, bTrace);
}

/*!
 * \brief Pack a BsHekRspState ITV message in big-endian byte order
 *        into the output buffer.
 * 
 * \param [in] pStruct    Pointer to the associated, populated message
 *                        structure.
 * \param [out] buf       Output message buffer.
 * \param bufSize         Size of output buffer.
 * \param bTrace          Do [not] trace packing.
 * 
 * \return
 * On success, returns the number of bytes packed.
 * On error, returns the appropriate \h_lt 0 negated NM_ECODE.
 */
INLINE_IN_H int BsHekPackRspState( BsHekRspState_T * pStruct,
                                   byte_t buf[],
                                   size_t bufSize,
                                   bool_t bTrace )
{
  return BsHekPackMsg(BsHekMsgIdRspState, pStruct, buf, bufSize, bTrace);
}

/*!
 * \brief Unpack a BsHekRspState ITV message in big-endian byte order
 *        from the input buffer.
 * 
 * \param [in] buf        Output message buffer.
 * \param uMsgLen         Length of message (bytes) in input buffer.
 * \param [out] pStruct   Pointer to the associated message structure.
 * \param bTrace          Do [not] trace packing.
 * 
 * \return
 * On success, returns the number of bytes unpacked.
 * On error, returns the appropriate \h_lt 0 negated NM_ECODE.
 */
INLINE_IN_H int BsHekUnpackRspState( byte_t buf[],
                                     size_t uMsgLen,
                                     BsHekRspState_T * pStruct,
                                     bool_t bTrace )
{
  return BsHekUnpackMsg(BsHekMsgIdRspState, buf, uMsgLen, pStruct, bTrace);
}

/*!
 * \brief Pack a BsHekReqGetHealth ITV message in big-endian byte order
 *        into the output buffer.
 * 
 * \param [in] pStruct    Pointer to the associated, populated message
 *                        structure.
 * \param [out] buf       Output message buffer.
 * \param bufSize         Size of output buffer.
 * \param bTrace          Do [not] trace packing.
 * 
 * \return
 * On success, returns the number of bytes packed.
 * On error, returns the appropriate \h_lt 0 negated NM_ECODE.
 */
INLINE_IN_H int BsHekPackReqGetHealth( BsHekReqGetHealth_T * pStruct,
                                       byte_t buf[],
                                       size_t bufSize,
                                       bool_t bTrace )
{
  return BsHekPackMsg(BsHekMsgIdReqGetHealth, pStruct, buf, bufSize, bTrace);
}

/*!
 * \brief Unpack a BsHekReqGetHealth ITV message in big-endian byte order
 *        from the input buffer.
 * 
 * \param [in] buf        Output message buffer.
 * \param uMsgLen         Length of message (bytes) in input buffer.
 * \param [out] pStruct   Pointer to the associated message structure.
 * \param bTrace          Do [not] trace packing.
 * 
 * \return
 * On success, returns the number of bytes unpacked.
 * On error, returns the appropriate \h_lt 0 negated NM_ECODE.
 */
INLINE_IN_H int BsHekUnpackReqGetHealth( byte_t buf[],
                                         size_t uMsgLen,
                                         BsHekReqGetHealth_T * pStruct,
                                         bool_t bTrace )
{
  return BsHekUnpackMsg(BsHekMsgIdReqGetHealth, buf, uMsgLen, pStruct, bTrace);
}

/*!
 * \brief Pack a BsHekRspGetHealth ITV message in big-endian byte order
 *        into the output buffer.
 * 
 * \param [in] pStruct    Pointer to the associated, populated message
 *                        structure.
 * \param [out] buf       Output message buffer.
 * \param bufSize         Size of output buffer.
 * \param bTrace          Do [not] trace packing.
 * 
 * \return
 * On success, returns the number of bytes packed.
 * On error, returns the appropriate \h_lt 0 negated NM_ECODE.
 */
INLINE_IN_H int BsHekPackRspGetHealth( BsHekRspGetHealth_T * pStruct,
                                       byte_t buf[],
                                       size_t bufSize,
                                       bool_t bTrace )
{
  return BsHekPackMsg(BsHekMsgIdRspGetHealth, pStruct, buf, bufSize, bTrace);
}

/*!
 * \brief Unpack a BsHekRspGetHealth ITV message in big-endian byte order
 *        from the input buffer.
 * 
 * \param [in] buf        Output message buffer.
 * \param uMsgLen         Length of message (bytes) in input buffer.
 * \param [out] pStruct   Pointer to the associated message structure.
 * \param bTrace          Do [not] trace packing.
 * 
 * \return
 * On success, returns the number of bytes unpacked.
 * On error, returns the appropriate \h_lt 0 negated NM_ECODE.
 */
INLINE_IN_H int BsHekUnpackRspGetHealth( byte_t buf[],
                                         size_t uMsgLen,
                                         BsHekRspGetHealth_T * pStruct,
                                         bool_t bTrace )
{
  return BsHekUnpackMsg(BsHekMsgIdRspGetHealth, buf, uMsgLen, pStruct, bTrace);
}

/*!
 * \brief Pack a BsHekReqClearAlarms ITV message in big-endian byte order
 *        into the output buffer.
 * 
 * \param [in] pStruct    Pointer to the associated, populated message
 *                        structure.
 * \param [out] buf       Output message buffer.
 * \param bufSize         Size of output buffer.
 * \param bTrace          Do [not] trace packing.
 * 
 * \return
 * On success, returns the number of bytes packed.
 * On error, returns the appropriate \h_lt 0 negated NM_ECODE.
 */
INLINE_IN_H int BsHekPackReqClearAlarms( BsHekReqClearAlarms_T * pStruct,
                                         byte_t buf[],
                                         size_t bufSize,
                                         bool_t bTrace )
{
  return BsHekPackMsg(BsHekMsgIdReqClearAlarms, pStruct, buf, bufSize, bTrace);
}

/*!
 * \brief Unpack a BsHekReqClearAlarms ITV message in big-endian byte order
 *        from the input buffer.
 * 
 * \param [in] buf        Output message buffer.
 * \param uMsgLen         Length of message (bytes) in input buffer.
 * \param [out] pStruct   Pointer to the associated message structure.
 * \param bTrace          Do [not] trace packing.
 * 
 * \return
 * On success, returns the number of bytes unpacked.
 * On error, returns the appropriate \h_lt 0 negated NM_ECODE.
 */
INLINE_IN_H int BsHekUnpackReqClearAlarms( byte_t buf[],
                                           size_t uMsgLen,
                                           BsHekReqClearAlarms_T * pStruct,
                                           bool_t bTrace )
{
  return BsHekUnpackMsg(BsHekMsgIdReqClearAlarms, buf, uMsgLen, pStruct, bTrace);
}


C_DECLS_END


#endif // _BSHEKMSGS_H
