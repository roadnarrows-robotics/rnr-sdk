///////////////////////////////////////////////////////////////////////////////
//
// File: bsDynaMsgs.c
//
/*!
 * \file
 *
 * \brief \h_botsense Server/Client Dynamixel NetMsgs XML Definitions.
 *
 * \warning This file was auto-generated on 2017.03.17 15:39:48 from the NetMsgs
 * XML specification bsDynaMsgs.xml.
 *
 * \copyright
 *   \h_copy 2017-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
 */
///////////////////////////////////////////////////////////////////////////////


#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/netmsgs.h"

#include "botsense/bsDynaMsgs.h"

#ifndef EOFDEF
/*! End of Field Definition entry. */
#define EOFDEF {NULL, 0, NMFTypeNone, 0, }
#endif // EOFDEF


//-----------------------------------------------------------------------------
// Private Interface
//-----------------------------------------------------------------------------


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Field Type BsDynaWriteTuple Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * BsDynaWriteTuple Field Id Enumeration
 */
typedef enum
{
  BsDynaWriteTupleFIdReserved           = 0,    ///< reserved field id
  BsDynaWriteTupleFIdservo_id           = 1,    ///< servo_id field id
  BsDynaWriteTupleFIdval                = 2,    ///< val field id
  BsDynaWriteTupleFIdNumOf              = 3     ///< number of fields
} BsDynaWriteTupleFId_T;

/*!
 * BsDynaWriteTuple Field Definitions
 */
static const NMFieldDef_T BsDynaWriteTupleFieldDefs[] =
{
  {
    .m_sFName                 = "servo_id",
    .m_eFId                   = BsDynaWriteTupleFIdservo_id,
    .m_eFType                 = NMFTypeU8,
    .m_uOffset                = memberoffset(BsDynaWriteTuple_T, m_servo_id),
    .m_this.m_u8.m_bits       = (byte_t)(0),
    .m_this.m_u8.m_valMin     = (byte_t)(0),
    .m_this.m_u8.m_valMax     = (byte_t)(0),
    .m_this.m_u8.m_valConst   = (byte_t)(0),
  },
  {
    .m_sFName                 = "val",
    .m_eFId                   = BsDynaWriteTupleFIdval,
    .m_eFType                 = NMFTypeU16,
    .m_uOffset                = memberoffset(BsDynaWriteTuple_T, m_val),
    .m_this.m_u16.m_bits      = (byte_t)(0),
    .m_this.m_u16.m_valMin    = (ushort_t)(0),
    .m_this.m_u16.m_valMax    = (ushort_t)(0),
    .m_this.m_u16.m_valConst  = (ushort_t)(0),
  },
  EOFDEF
};

/*!
 * BsDynaWriteTuple Message Definition
 */
static const NMMsgDef_T BsDynaWriteTupleMsgDef =
{
  .m_sMsgName         = "BsDynaWriteTuple",
  .m_eMsgId           = BsDynaMsgIdNone,
  .m_uCount           = (size_t)(2),
  .m_pFields          = BsDynaWriteTupleFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message BsDynaReqOpenArgs Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * BsDynaReqOpenArgs Field Id Enumeration
 */
typedef enum
{
  BsDynaReqOpenArgsFIdReserved          = 0,    ///< reserved field id
  BsDynaReqOpenArgsFIdbaudrate          = 1,    ///< baudrate field id
  BsDynaReqOpenArgsFIdNumOf             = 2     ///< number of fields
} BsDynaReqOpenArgsFId_T;

/*!
 * BsDynaReqOpenArgs Field Definitions
 */
static const NMFieldDef_T BsDynaReqOpenArgsFieldDefs[] =
{
  {
    .m_sFName                 = "baudrate",
    .m_eFId                   = BsDynaReqOpenArgsFIdbaudrate,
    .m_eFType                 = NMFTypeU32,
    .m_uOffset                = memberoffset(BsDynaReqOpenArgs_T, m_baudrate),
    .m_this.m_u32.m_bits      = (byte_t)(0),
    .m_this.m_u32.m_valMin    = (uint_t)(0),
    .m_this.m_u32.m_valMax    = (uint_t)(0),
    .m_this.m_u32.m_valConst  = (uint_t)(0),
  },
  EOFDEF
};

/*!
 * BsDynaReqOpenArgs Message Definition
 */
static const NMMsgDef_T BsDynaReqOpenArgsMsgDef =
{
  .m_sMsgName         = "BsDynaReqOpenArgs",
  .m_eMsgId           = BsDynaMsgIdReqOpenArgs,
  .m_uCount           = (size_t)(1),
  .m_pFields          = BsDynaReqOpenArgsFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message BsDynaReqSetBaudRate Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * BsDynaReqSetBaudRate Field Id Enumeration
 */
typedef enum
{
  BsDynaReqSetBaudRateFIdReserved       = 0,    ///< reserved field id
  BsDynaReqSetBaudRateFIdbaudrate       = 1,    ///< baudrate field id
  BsDynaReqSetBaudRateFIdNumOf          = 2     ///< number of fields
} BsDynaReqSetBaudRateFId_T;

/*!
 * BsDynaReqSetBaudRate Field Definitions
 */
static const NMFieldDef_T BsDynaReqSetBaudRateFieldDefs[] =
{
  {
    .m_sFName                 = "baudrate",
    .m_eFId                   = BsDynaReqSetBaudRateFIdbaudrate,
    .m_eFType                 = NMFTypeU32,
    .m_uOffset                = memberoffset(BsDynaReqSetBaudRate_T, m_baudrate),
    .m_this.m_u32.m_bits      = (byte_t)(0),
    .m_this.m_u32.m_valMin    = (uint_t)(0),
    .m_this.m_u32.m_valMax    = (uint_t)(0),
    .m_this.m_u32.m_valConst  = (uint_t)(0),
  },
  EOFDEF
};

/*!
 * BsDynaReqSetBaudRate Message Definition
 */
static const NMMsgDef_T BsDynaReqSetBaudRateMsgDef =
{
  .m_sMsgName         = "BsDynaReqSetBaudRate",
  .m_eMsgId           = BsDynaMsgIdReqSetBaudRate,
  .m_uCount           = (size_t)(1),
  .m_pFields          = BsDynaReqSetBaudRateFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message BsDynaReqRead8 Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * BsDynaReqRead8 Field Id Enumeration
 */
typedef enum
{
  BsDynaReqRead8FIdReserved             = 0,    ///< reserved field id
  BsDynaReqRead8FIdservo_id             = 1,    ///< servo_id field id
  BsDynaReqRead8FIdaddr                 = 2,    ///< addr field id
  BsDynaReqRead8FIdNumOf                = 3     ///< number of fields
} BsDynaReqRead8FId_T;

/*!
 * BsDynaReqRead8 Field Definitions
 */
static const NMFieldDef_T BsDynaReqRead8FieldDefs[] =
{
  {
    .m_sFName                 = "servo_id",
    .m_eFId                   = BsDynaReqRead8FIdservo_id,
    .m_eFType                 = NMFTypeU8,
    .m_uOffset                = memberoffset(BsDynaReqRead8_T, m_servo_id),
    .m_this.m_u8.m_bits       = (byte_t)(0),
    .m_this.m_u8.m_valMin     = (byte_t)(0),
    .m_this.m_u8.m_valMax     = (byte_t)(0),
    .m_this.m_u8.m_valConst   = (byte_t)(0),
  },
  {
    .m_sFName                 = "addr",
    .m_eFId                   = BsDynaReqRead8FIdaddr,
    .m_eFType                 = NMFTypeU8,
    .m_uOffset                = memberoffset(BsDynaReqRead8_T, m_addr),
    .m_this.m_u8.m_bits       = (byte_t)(0),
    .m_this.m_u8.m_valMin     = (byte_t)(0),
    .m_this.m_u8.m_valMax     = (byte_t)(0),
    .m_this.m_u8.m_valConst   = (byte_t)(0),
  },
  EOFDEF
};

/*!
 * BsDynaReqRead8 Message Definition
 */
static const NMMsgDef_T BsDynaReqRead8MsgDef =
{
  .m_sMsgName         = "BsDynaReqRead8",
  .m_eMsgId           = BsDynaMsgIdReqRead8,
  .m_uCount           = (size_t)(2),
  .m_pFields          = BsDynaReqRead8FieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message BsDynaRspRead8 Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * BsDynaRspRead8 Field Id Enumeration
 */
typedef enum
{
  BsDynaRspRead8FIdReserved             = 0,    ///< reserved field id
  BsDynaRspRead8FIdalarms               = 1,    ///< alarms field id
  BsDynaRspRead8FIdval                  = 2,    ///< val field id
  BsDynaRspRead8FIdNumOf                = 3     ///< number of fields
} BsDynaRspRead8FId_T;

/*!
 * BsDynaRspRead8 Field Definitions
 */
static const NMFieldDef_T BsDynaRspRead8FieldDefs[] =
{
  {
    .m_sFName                 = "alarms",
    .m_eFId                   = BsDynaRspRead8FIdalarms,
    .m_eFType                 = NMFTypeU8,
    .m_uOffset                = memberoffset(BsDynaRspRead8_T, m_alarms),
    .m_this.m_u8.m_bits       = (byte_t)(0),
    .m_this.m_u8.m_valMin     = (byte_t)(0),
    .m_this.m_u8.m_valMax     = (byte_t)(0),
    .m_this.m_u8.m_valConst   = (byte_t)(0),
  },
  {
    .m_sFName                 = "val",
    .m_eFId                   = BsDynaRspRead8FIdval,
    .m_eFType                 = NMFTypeU8,
    .m_uOffset                = memberoffset(BsDynaRspRead8_T, m_val),
    .m_this.m_u8.m_bits       = (byte_t)(0),
    .m_this.m_u8.m_valMin     = (byte_t)(0),
    .m_this.m_u8.m_valMax     = (byte_t)(0),
    .m_this.m_u8.m_valConst   = (byte_t)(0),
  },
  EOFDEF
};

/*!
 * BsDynaRspRead8 Message Definition
 */
static const NMMsgDef_T BsDynaRspRead8MsgDef =
{
  .m_sMsgName         = "BsDynaRspRead8",
  .m_eMsgId           = BsDynaMsgIdRspRead8,
  .m_uCount           = (size_t)(2),
  .m_pFields          = BsDynaRspRead8FieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message BsDynaReqRead16 Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * BsDynaReqRead16 Field Id Enumeration
 */
typedef enum
{
  BsDynaReqRead16FIdReserved            = 0,    ///< reserved field id
  BsDynaReqRead16FIdservo_id            = 1,    ///< servo_id field id
  BsDynaReqRead16FIdaddr                = 2,    ///< addr field id
  BsDynaReqRead16FIdNumOf               = 3     ///< number of fields
} BsDynaReqRead16FId_T;

/*!
 * BsDynaReqRead16 Field Definitions
 */
static const NMFieldDef_T BsDynaReqRead16FieldDefs[] =
{
  {
    .m_sFName                 = "servo_id",
    .m_eFId                   = BsDynaReqRead16FIdservo_id,
    .m_eFType                 = NMFTypeU8,
    .m_uOffset                = memberoffset(BsDynaReqRead16_T, m_servo_id),
    .m_this.m_u8.m_bits       = (byte_t)(0),
    .m_this.m_u8.m_valMin     = (byte_t)(0),
    .m_this.m_u8.m_valMax     = (byte_t)(0),
    .m_this.m_u8.m_valConst   = (byte_t)(0),
  },
  {
    .m_sFName                 = "addr",
    .m_eFId                   = BsDynaReqRead16FIdaddr,
    .m_eFType                 = NMFTypeU8,
    .m_uOffset                = memberoffset(BsDynaReqRead16_T, m_addr),
    .m_this.m_u8.m_bits       = (byte_t)(0),
    .m_this.m_u8.m_valMin     = (byte_t)(0),
    .m_this.m_u8.m_valMax     = (byte_t)(0),
    .m_this.m_u8.m_valConst   = (byte_t)(0),
  },
  EOFDEF
};

/*!
 * BsDynaReqRead16 Message Definition
 */
static const NMMsgDef_T BsDynaReqRead16MsgDef =
{
  .m_sMsgName         = "BsDynaReqRead16",
  .m_eMsgId           = BsDynaMsgIdReqRead16,
  .m_uCount           = (size_t)(2),
  .m_pFields          = BsDynaReqRead16FieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message BsDynaRspRead16 Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * BsDynaRspRead16 Field Id Enumeration
 */
typedef enum
{
  BsDynaRspRead16FIdReserved            = 0,    ///< reserved field id
  BsDynaRspRead16FIdalarms              = 1,    ///< alarms field id
  BsDynaRspRead16FIdval                 = 2,    ///< val field id
  BsDynaRspRead16FIdNumOf               = 3     ///< number of fields
} BsDynaRspRead16FId_T;

/*!
 * BsDynaRspRead16 Field Definitions
 */
static const NMFieldDef_T BsDynaRspRead16FieldDefs[] =
{
  {
    .m_sFName                 = "alarms",
    .m_eFId                   = BsDynaRspRead16FIdalarms,
    .m_eFType                 = NMFTypeU8,
    .m_uOffset                = memberoffset(BsDynaRspRead16_T, m_alarms),
    .m_this.m_u8.m_bits       = (byte_t)(0),
    .m_this.m_u8.m_valMin     = (byte_t)(0),
    .m_this.m_u8.m_valMax     = (byte_t)(0),
    .m_this.m_u8.m_valConst   = (byte_t)(0),
  },
  {
    .m_sFName                 = "val",
    .m_eFId                   = BsDynaRspRead16FIdval,
    .m_eFType                 = NMFTypeU16,
    .m_uOffset                = memberoffset(BsDynaRspRead16_T, m_val),
    .m_this.m_u16.m_bits      = (byte_t)(0),
    .m_this.m_u16.m_valMin    = (ushort_t)(0),
    .m_this.m_u16.m_valMax    = (ushort_t)(0),
    .m_this.m_u16.m_valConst  = (ushort_t)(0),
  },
  EOFDEF
};

/*!
 * BsDynaRspRead16 Message Definition
 */
static const NMMsgDef_T BsDynaRspRead16MsgDef =
{
  .m_sMsgName         = "BsDynaRspRead16",
  .m_eMsgId           = BsDynaMsgIdRspRead16,
  .m_uCount           = (size_t)(2),
  .m_pFields          = BsDynaRspRead16FieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message BsDynaReqWrite8 Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * BsDynaReqWrite8 Field Id Enumeration
 */
typedef enum
{
  BsDynaReqWrite8FIdReserved            = 0,    ///< reserved field id
  BsDynaReqWrite8FIdservo_id            = 1,    ///< servo_id field id
  BsDynaReqWrite8FIdaddr                = 2,    ///< addr field id
  BsDynaReqWrite8FIdval                 = 3,    ///< val field id
  BsDynaReqWrite8FIdNumOf               = 4     ///< number of fields
} BsDynaReqWrite8FId_T;

/*!
 * BsDynaReqWrite8 Field Definitions
 */
static const NMFieldDef_T BsDynaReqWrite8FieldDefs[] =
{
  {
    .m_sFName                 = "servo_id",
    .m_eFId                   = BsDynaReqWrite8FIdservo_id,
    .m_eFType                 = NMFTypeU8,
    .m_uOffset                = memberoffset(BsDynaReqWrite8_T, m_servo_id),
    .m_this.m_u8.m_bits       = (byte_t)(0),
    .m_this.m_u8.m_valMin     = (byte_t)(0),
    .m_this.m_u8.m_valMax     = (byte_t)(0),
    .m_this.m_u8.m_valConst   = (byte_t)(0),
  },
  {
    .m_sFName                 = "addr",
    .m_eFId                   = BsDynaReqWrite8FIdaddr,
    .m_eFType                 = NMFTypeU8,
    .m_uOffset                = memberoffset(BsDynaReqWrite8_T, m_addr),
    .m_this.m_u8.m_bits       = (byte_t)(0),
    .m_this.m_u8.m_valMin     = (byte_t)(0),
    .m_this.m_u8.m_valMax     = (byte_t)(0),
    .m_this.m_u8.m_valConst   = (byte_t)(0),
  },
  {
    .m_sFName                 = "val",
    .m_eFId                   = BsDynaReqWrite8FIdval,
    .m_eFType                 = NMFTypeU8,
    .m_uOffset                = memberoffset(BsDynaReqWrite8_T, m_val),
    .m_this.m_u8.m_bits       = (byte_t)(0),
    .m_this.m_u8.m_valMin     = (byte_t)(0),
    .m_this.m_u8.m_valMax     = (byte_t)(0),
    .m_this.m_u8.m_valConst   = (byte_t)(0),
  },
  EOFDEF
};

/*!
 * BsDynaReqWrite8 Message Definition
 */
static const NMMsgDef_T BsDynaReqWrite8MsgDef =
{
  .m_sMsgName         = "BsDynaReqWrite8",
  .m_eMsgId           = BsDynaMsgIdReqWrite8,
  .m_uCount           = (size_t)(3),
  .m_pFields          = BsDynaReqWrite8FieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message BsDynaRspWrite8 Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * BsDynaRspWrite8 Field Id Enumeration
 */
typedef enum
{
  BsDynaRspWrite8FIdReserved            = 0,    ///< reserved field id
  BsDynaRspWrite8FIdalarms              = 1,    ///< alarms field id
  BsDynaRspWrite8FIdNumOf               = 2     ///< number of fields
} BsDynaRspWrite8FId_T;

/*!
 * BsDynaRspWrite8 Field Definitions
 */
static const NMFieldDef_T BsDynaRspWrite8FieldDefs[] =
{
  {
    .m_sFName                 = "alarms",
    .m_eFId                   = BsDynaRspWrite8FIdalarms,
    .m_eFType                 = NMFTypeU8,
    .m_uOffset                = memberoffset(BsDynaRspWrite8_T, m_alarms),
    .m_this.m_u8.m_bits       = (byte_t)(0),
    .m_this.m_u8.m_valMin     = (byte_t)(0),
    .m_this.m_u8.m_valMax     = (byte_t)(0),
    .m_this.m_u8.m_valConst   = (byte_t)(0),
  },
  EOFDEF
};

/*!
 * BsDynaRspWrite8 Message Definition
 */
static const NMMsgDef_T BsDynaRspWrite8MsgDef =
{
  .m_sMsgName         = "BsDynaRspWrite8",
  .m_eMsgId           = BsDynaMsgIdRspWrite8,
  .m_uCount           = (size_t)(1),
  .m_pFields          = BsDynaRspWrite8FieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message BsDynaReqWrite16 Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * BsDynaReqWrite16 Field Id Enumeration
 */
typedef enum
{
  BsDynaReqWrite16FIdReserved           = 0,    ///< reserved field id
  BsDynaReqWrite16FIdservo_id           = 1,    ///< servo_id field id
  BsDynaReqWrite16FIdaddr               = 2,    ///< addr field id
  BsDynaReqWrite16FIdval                = 3,    ///< val field id
  BsDynaReqWrite16FIdNumOf              = 4     ///< number of fields
} BsDynaReqWrite16FId_T;

/*!
 * BsDynaReqWrite16 Field Definitions
 */
static const NMFieldDef_T BsDynaReqWrite16FieldDefs[] =
{
  {
    .m_sFName                 = "servo_id",
    .m_eFId                   = BsDynaReqWrite16FIdservo_id,
    .m_eFType                 = NMFTypeU8,
    .m_uOffset                = memberoffset(BsDynaReqWrite16_T, m_servo_id),
    .m_this.m_u8.m_bits       = (byte_t)(0),
    .m_this.m_u8.m_valMin     = (byte_t)(0),
    .m_this.m_u8.m_valMax     = (byte_t)(0),
    .m_this.m_u8.m_valConst   = (byte_t)(0),
  },
  {
    .m_sFName                 = "addr",
    .m_eFId                   = BsDynaReqWrite16FIdaddr,
    .m_eFType                 = NMFTypeU8,
    .m_uOffset                = memberoffset(BsDynaReqWrite16_T, m_addr),
    .m_this.m_u8.m_bits       = (byte_t)(0),
    .m_this.m_u8.m_valMin     = (byte_t)(0),
    .m_this.m_u8.m_valMax     = (byte_t)(0),
    .m_this.m_u8.m_valConst   = (byte_t)(0),
  },
  {
    .m_sFName                 = "val",
    .m_eFId                   = BsDynaReqWrite16FIdval,
    .m_eFType                 = NMFTypeU16,
    .m_uOffset                = memberoffset(BsDynaReqWrite16_T, m_val),
    .m_this.m_u16.m_bits      = (byte_t)(0),
    .m_this.m_u16.m_valMin    = (ushort_t)(0),
    .m_this.m_u16.m_valMax    = (ushort_t)(0),
    .m_this.m_u16.m_valConst  = (ushort_t)(0),
  },
  EOFDEF
};

/*!
 * BsDynaReqWrite16 Message Definition
 */
static const NMMsgDef_T BsDynaReqWrite16MsgDef =
{
  .m_sMsgName         = "BsDynaReqWrite16",
  .m_eMsgId           = BsDynaMsgIdReqWrite16,
  .m_uCount           = (size_t)(3),
  .m_pFields          = BsDynaReqWrite16FieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message BsDynaRspWrite16 Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * BsDynaRspWrite16 Field Id Enumeration
 */
typedef enum
{
  BsDynaRspWrite16FIdReserved           = 0,    ///< reserved field id
  BsDynaRspWrite16FIdalarms             = 1,    ///< alarms field id
  BsDynaRspWrite16FIdNumOf              = 2     ///< number of fields
} BsDynaRspWrite16FId_T;

/*!
 * BsDynaRspWrite16 Field Definitions
 */
static const NMFieldDef_T BsDynaRspWrite16FieldDefs[] =
{
  {
    .m_sFName                 = "alarms",
    .m_eFId                   = BsDynaRspWrite16FIdalarms,
    .m_eFType                 = NMFTypeU8,
    .m_uOffset                = memberoffset(BsDynaRspWrite16_T, m_alarms),
    .m_this.m_u8.m_bits       = (byte_t)(0),
    .m_this.m_u8.m_valMin     = (byte_t)(0),
    .m_this.m_u8.m_valMax     = (byte_t)(0),
    .m_this.m_u8.m_valConst   = (byte_t)(0),
  },
  EOFDEF
};

/*!
 * BsDynaRspWrite16 Message Definition
 */
static const NMMsgDef_T BsDynaRspWrite16MsgDef =
{
  .m_sMsgName         = "BsDynaRspWrite16",
  .m_eMsgId           = BsDynaMsgIdRspWrite16,
  .m_uCount           = (size_t)(1),
  .m_pFields          = BsDynaRspWrite16FieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message BsDynaReqSyncWrite Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * BsDynaReqSyncWritetuples Field Definitions
 */
static const NMFieldDef_T BsDynaReqSyncWritetuplesFieldDef[] =
{
  {
    .m_sFName                 = "tuples",
    .m_eFId                   = 0,
    .m_eFType                 = NMFTypeStruct,
    .m_uOffset                = (size_t)0,
    .m_this.m_struct          = &BsDynaWriteTupleMsgDef,
  },
  EOFDEF
};

/*!
 * BsDynaReqSyncWrite Field Id Enumeration
 */
typedef enum
{
  BsDynaReqSyncWriteFIdReserved         = 0,    ///< reserved field id
  BsDynaReqSyncWriteFIdaddr             = 1,    ///< addr field id
  BsDynaReqSyncWriteFIddata_size        = 2,    ///< data_size field id
  BsDynaReqSyncWriteFIdtuples           = 3,    ///< tuples field id
  BsDynaReqSyncWriteFIdNumOf            = 4     ///< number of fields
} BsDynaReqSyncWriteFId_T;

/*!
 * BsDynaReqSyncWrite Field Definitions
 */
static const NMFieldDef_T BsDynaReqSyncWriteFieldDefs[] =
{
  {
    .m_sFName                 = "addr",
    .m_eFId                   = BsDynaReqSyncWriteFIdaddr,
    .m_eFType                 = NMFTypeU8,
    .m_uOffset                = memberoffset(BsDynaReqSyncWrite_T, m_addr),
    .m_this.m_u8.m_bits       = (byte_t)(0),
    .m_this.m_u8.m_valMin     = (byte_t)(0),
    .m_this.m_u8.m_valMax     = (byte_t)(0),
    .m_this.m_u8.m_valConst   = (byte_t)(0),
  },
  {
    .m_sFName                 = "data_size",
    .m_eFId                   = BsDynaReqSyncWriteFIddata_size,
    .m_eFType                 = NMFTypeU8,
    .m_uOffset                = memberoffset(BsDynaReqSyncWrite_T, m_data_size),
    .m_this.m_u8.m_bits       = (byte_t)(0),
    .m_this.m_u8.m_valMin     = (byte_t)(0),
    .m_this.m_u8.m_valMax     = (byte_t)(0),
    .m_this.m_u8.m_valConst   = (byte_t)(0),
  },
  {
    .m_sFName                 = "tuples",
    .m_eFId                   = BsDynaReqSyncWriteFIdtuples,
    .m_eFType                 = NMFTypeVector,
    .m_uOffset                = memberoffset(BsDynaReqSyncWrite_T, m_tuples),
    .m_this.m_vector.m_uMaxCount
                              = (size_t)DYNA_ID_NUMOF,
    .m_this.m_vector.m_uElemSize
                              = sizeof(BsDynaWriteTuple_T),
    .m_this.m_vector.m_pThisElem
                              = BsDynaReqSyncWritetuplesFieldDef,
  },
  EOFDEF
};

/*!
 * BsDynaReqSyncWrite Message Definition
 */
static const NMMsgDef_T BsDynaReqSyncWriteMsgDef =
{
  .m_sMsgName         = "BsDynaReqSyncWrite",
  .m_eMsgId           = BsDynaMsgIdReqSyncWrite,
  .m_uCount           = (size_t)(3),
  .m_pFields          = BsDynaReqSyncWriteFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message BsDynaReqPing Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * BsDynaReqPing Field Id Enumeration
 */
typedef enum
{
  BsDynaReqPingFIdReserved              = 0,    ///< reserved field id
  BsDynaReqPingFIdservo_id              = 1,    ///< servo_id field id
  BsDynaReqPingFIdNumOf                 = 2     ///< number of fields
} BsDynaReqPingFId_T;

/*!
 * BsDynaReqPing Field Definitions
 */
static const NMFieldDef_T BsDynaReqPingFieldDefs[] =
{
  {
    .m_sFName                 = "servo_id",
    .m_eFId                   = BsDynaReqPingFIdservo_id,
    .m_eFType                 = NMFTypeU8,
    .m_uOffset                = memberoffset(BsDynaReqPing_T, m_servo_id),
    .m_this.m_u8.m_bits       = (byte_t)(0),
    .m_this.m_u8.m_valMin     = (byte_t)(0),
    .m_this.m_u8.m_valMax     = (byte_t)(0),
    .m_this.m_u8.m_valConst   = (byte_t)(0),
  },
  EOFDEF
};

/*!
 * BsDynaReqPing Message Definition
 */
static const NMMsgDef_T BsDynaReqPingMsgDef =
{
  .m_sMsgName         = "BsDynaReqPing",
  .m_eMsgId           = BsDynaMsgIdReqPing,
  .m_uCount           = (size_t)(1),
  .m_pFields          = BsDynaReqPingFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message BsDynaRspPing Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * BsDynaRspPing Field Id Enumeration
 */
typedef enum
{
  BsDynaRspPingFIdReserved              = 0,    ///< reserved field id
  BsDynaRspPingFIdpong                  = 1,    ///< pong field id
  BsDynaRspPingFIdNumOf                 = 2     ///< number of fields
} BsDynaRspPingFId_T;

/*!
 * BsDynaRspPing Field Definitions
 */
static const NMFieldDef_T BsDynaRspPingFieldDefs[] =
{
  {
    .m_sFName                 = "pong",
    .m_eFId                   = BsDynaRspPingFIdpong,
    .m_eFType                 = NMFTypeBool,
    .m_uOffset                = memberoffset(BsDynaRspPing_T, m_pong),
  },
  EOFDEF
};

/*!
 * BsDynaRspPing Message Definition
 */
static const NMMsgDef_T BsDynaRspPingMsgDef =
{
  .m_sMsgName         = "BsDynaRspPing",
  .m_eMsgId           = BsDynaMsgIdRspPing,
  .m_uCount           = (size_t)(1),
  .m_pFields          = BsDynaRspPingFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message BsDynaReqReset Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * BsDynaReqReset Field Id Enumeration
 */
typedef enum
{
  BsDynaReqResetFIdReserved             = 0,    ///< reserved field id
  BsDynaReqResetFIdservo_id             = 1,    ///< servo_id field id
  BsDynaReqResetFIdNumOf                = 2     ///< number of fields
} BsDynaReqResetFId_T;

/*!
 * BsDynaReqReset Field Definitions
 */
static const NMFieldDef_T BsDynaReqResetFieldDefs[] =
{
  {
    .m_sFName                 = "servo_id",
    .m_eFId                   = BsDynaReqResetFIdservo_id,
    .m_eFType                 = NMFTypeU8,
    .m_uOffset                = memberoffset(BsDynaReqReset_T, m_servo_id),
    .m_this.m_u8.m_bits       = (byte_t)(0),
    .m_this.m_u8.m_valMin     = (byte_t)(0),
    .m_this.m_u8.m_valMax     = (byte_t)(0),
    .m_this.m_u8.m_valConst   = (byte_t)(0),
  },
  EOFDEF
};

/*!
 * BsDynaReqReset Message Definition
 */
static const NMMsgDef_T BsDynaReqResetMsgDef =
{
  .m_sMsgName         = "BsDynaReqReset",
  .m_eMsgId           = BsDynaMsgIdReqReset,
  .m_uCount           = (size_t)(1),
  .m_pFields          = BsDynaReqResetFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message BsDynaReqSetHalfDuplexCtl Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * BsDynaReqSetHalfDuplexCtl Field Id Enumeration
 */
typedef enum
{
  BsDynaReqSetHalfDuplexCtlFIdReserved  = 0,    ///< reserved field id
  BsDynaReqSetHalfDuplexCtlFIdsignal    = 1,    ///< signal field id
  BsDynaReqSetHalfDuplexCtlFIdNumOf     = 2     ///< number of fields
} BsDynaReqSetHalfDuplexCtlFId_T;

/*!
 * BsDynaReqSetHalfDuplexCtl Field Definitions
 */
static const NMFieldDef_T BsDynaReqSetHalfDuplexCtlFieldDefs[] =
{
  {
    .m_sFName                 = "signal",
    .m_eFId                   = BsDynaReqSetHalfDuplexCtlFIdsignal,
    .m_eFType                 = NMFTypeU32,
    .m_uOffset                = memberoffset(BsDynaReqSetHalfDuplexCtl_T, m_signal),
    .m_this.m_u32.m_bits      = (byte_t)(0),
    .m_this.m_u32.m_valMin    = (uint_t)(0),
    .m_this.m_u32.m_valMax    = (uint_t)(0),
    .m_this.m_u32.m_valConst  = (uint_t)(0),
  },
  EOFDEF
};

/*!
 * BsDynaReqSetHalfDuplexCtl Message Definition
 */
static const NMMsgDef_T BsDynaReqSetHalfDuplexCtlMsgDef =
{
  .m_sMsgName         = "BsDynaReqSetHalfDuplexCtl",
  .m_eMsgId           = BsDynaMsgIdReqSetHalfDuplexCtl,
  .m_uCount           = (size_t)(1),
  .m_pFields          = BsDynaReqSetHalfDuplexCtlFieldDefs
};


//-----------------------------------------------------------------------------
// Public Interface
//-----------------------------------------------------------------------------

/*!
 * BsDyna Message Definition Look-Up Table. Indexed by BsDynaMsgId_T enum.
 */
const NMMsgDef_T *BsDynaMsgDefLookupTbl[] =
{
  NULL,                                 ///< [BsDynaMsgIdNone]
  &BsDynaReqOpenArgsMsgDef,             ///< [BsDynaMsgIdReqOpenArgs]
  &BsDynaReqSetBaudRateMsgDef,          ///< [BsDynaMsgIdReqSetBaudRate]
  &BsDynaReqRead8MsgDef,                ///< [BsDynaMsgIdReqRead8]
  &BsDynaRspRead8MsgDef,                ///< [BsDynaMsgIdRspRead8]
  &BsDynaReqRead16MsgDef,               ///< [BsDynaMsgIdReqRead16]
  &BsDynaRspRead16MsgDef,               ///< [BsDynaMsgIdRspRead16]
  &BsDynaReqWrite8MsgDef,               ///< [BsDynaMsgIdReqWrite8]
  &BsDynaRspWrite8MsgDef,               ///< [BsDynaMsgIdRspWrite8]
  &BsDynaReqWrite16MsgDef,              ///< [BsDynaMsgIdReqWrite16]
  &BsDynaRspWrite16MsgDef,              ///< [BsDynaMsgIdRspWrite16]
  &BsDynaReqSyncWriteMsgDef,            ///< [BsDynaMsgIdReqSyncWrite]
  &BsDynaReqPingMsgDef,                 ///< [BsDynaMsgIdReqPing]
  &BsDynaRspPingMsgDef,                 ///< [BsDynaMsgIdRspPing]
  &BsDynaReqResetMsgDef,                ///< [BsDynaMsgIdReqReset]
  &BsDynaReqSetHalfDuplexCtlMsgDef,     ///< [BsDynaMsgIdReqSetHalfDuplexCtl]
  NULL                                  ///< [BsDynaMsgIdNumOf]
};

/*!
 * BsDyna Message Maximum Size Look-Up Table. Indexed by BsDynaMsgId_T enum.
 */
size_t BsDynaMsgMaxLenLookupTbl[] =
{
  (size_t)(0),                          ///< [BsDynaMsgIdNone]
  (size_t)(9),                          ///< [BsDynaMsgIdReqOpenArgs]
  (size_t)(9),                          ///< [BsDynaMsgIdReqSetBaudRate]
  (size_t)(9),                          ///< [BsDynaMsgIdReqRead8]
  (size_t)(9),                          ///< [BsDynaMsgIdRspRead8]
  (size_t)(9),                          ///< [BsDynaMsgIdReqRead16]
  (size_t)(10),                         ///< [BsDynaMsgIdRspRead16]
  (size_t)(12),                         ///< [BsDynaMsgIdReqWrite8]
  (size_t)(6),                          ///< [BsDynaMsgIdRspWrite8]
  (size_t)(13),                         ///< [BsDynaMsgIdReqWrite16]
  (size_t)(6),                          ///< [BsDynaMsgIdRspWrite16]
  (size_t)((3+(3+(3+(4+(10*DYNA_ID_NUMOF)))))),
                                        ///< [BsDynaMsgIdReqSyncWrite]
  (size_t)(6),                          ///< [BsDynaMsgIdReqPing]
  (size_t)(6),                          ///< [BsDynaMsgIdRspPing]
  (size_t)(6),                          ///< [BsDynaMsgIdReqReset]
  (size_t)(9),                          ///< [BsDynaMsgIdReqSetHalfDuplexCtl]
  (size_t)(0)                           ///< [BsDynaMsgIdNumOf]
};


//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

/*!
 * \brief Look up the message definition associated with the message id.
 * 
 * \param eMsgId          Message Id.
 * 
 * \return
 * On success, returns the pointer to the NMMsgDef_T.
 * On error, NULL is returned.
 */
const NMMsgDef_T * BsDynaLookupMsgDef( BsDynaMsgId_T eMsgId )
{
  if( (uint_t)eMsgId >= (uint_t)BsDynaMsgIdNumOf )
  {
    return NULL;
  }
  else
  {
    return BsDynaMsgDefLookupTbl[(uint_t)eMsgId];
  }
}

/*!
 * \brief Look up the message maximum length associated with the message id.
 * 
 * The maximum length is the total number of packed bytes possible for the
 * given message. The message may be much shorter.
 * 
 * \param eMsgId          Message Id.
 * 
 * \return
 * On success, returns the number of bytes.
 * On error, 0 is returned.
 */
size_t BsDynaLookupMsgMaxLen( BsDynaMsgId_T eMsgId )
{
  if( (uint_t)eMsgId >= (uint_t)BsDynaMsgIdNumOf )
  {
    return (size_t)0;
  }
  else
  {
    return BsDynaMsgMaxLenLookupTbl[(uint_t)eMsgId];
  }
}

/*!
 * \brief Pack a ITV message in big-endian byte order.
 * 
 * \param eMsgId          Message Id.
 * \param [in] pStruct    Pointer to the associated, populated message
 *                        structure.
 * \param [out] buf       Output message buffer.
 * \param bufSize         Size of output buffer.
 * \param bTrace          Do [not] trace packing.
 * 
 * \return
 * On success, returns the number of bytes packed.
 * On error, returns the appropriate \< 0 negated NM_ECODE.
 */
int BsDynaPackMsg( BsDynaMsgId_T eMsgId,
                   void * pStruct,
                   byte_t buf[],
                   size_t bufSize,
                   bool_t bTrace )
{
  const  NMMsgDef_T  *pMsgDef;
  
  if( (pMsgDef = BsDynaLookupMsgDef(eMsgId)) == NULL )
  {
    LOGERROR("%s(ecode=%d): msgid=%u.", 
              nmStrError(NM_ECODE_MSGID), NM_ECODE_MSGID, eMsgId);
    return -NM_ECODE_MSGID;
  }
  
  if( bTrace )
  {
    return nmPackITVMsgDebug(pMsgDef, pStruct, buf, bufSize, NMEndianBig);
  }
  else
  {
    return nmPackITVMsg(pMsgDef, pStruct, buf, bufSize, NMEndianBig);
  }
}

/*!
 * \brief Unpack a ITV message in big-endian byte order.
 * 
 * \param eMsgId          Message Id.
 * \param [in] buf        Input message buffer.
 * \param uMsgLen         Length of message (bytes) in input buffer.
 * \param [out] pStruct   Pointer to the associated message structure.
 * \param bTrace          Do [not] trace packing.
 * 
 * \return
 * On success, returns the number of bytes unpacked.
 * On error, returns the appropriate \< 0 negated NM_ECODE.
 */
int BsDynaUnpackMsg( BsDynaMsgId_T eMsgId,
                     byte_t buf[],
                     size_t uMsgLen,
                     void * pStruct,
                     bool_t bTrace )
{
  const  NMMsgDef_T  *pMsgDef;
  
  if( (pMsgDef = BsDynaLookupMsgDef(eMsgId)) == NULL )
  {
    LOGERROR("%s(ecode=%d): msgid=%u.", 
              nmStrError(NM_ECODE_MSGID), NM_ECODE_MSGID, eMsgId);
    return -NM_ECODE_MSGID;
  }
  
  if( bTrace )
  {
    return nmUnpackITVMsgDebug(pMsgDef, buf, uMsgLen, pStruct, NMEndianBig);
  }
  else
  {
    return nmUnpackITVMsg(pMsgDef, buf, uMsgLen, pStruct, NMEndianBig);
  }
}

