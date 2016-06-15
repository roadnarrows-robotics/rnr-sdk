///////////////////////////////////////////////////////////////////////////////
//
// File: CogniBoostMsgs.c
//
/*!
 * \file
 *
 * \brief CogniBoost host client - CogniBoost device server message set.
 *
 * \warning This file was auto-generated on 2016.06.15 14:31:53 from the NetMsgs
 * XML specification CogniBoostMsgs.xml.
 *
 * \par Copyright:
 * (C) 2016. RoadNarrows LLC
 * (http://www.roadnarrows.com)
 * All Rights Reserved
 */
///////////////////////////////////////////////////////////////////////////////


#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/netmsgs.h"

#include "CogniBoost/CogniBoostMsgs.h"

#ifndef EOFDEF
/*! End of Field Definition entry. */
#define EOFDEF {NULL, 0, NMFTypeNone, 0, }
#endif // EOFDEF


//-----------------------------------------------------------------------------
// Private Interface
//-----------------------------------------------------------------------------


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Field Type cbLedBlingParams Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbLedBlingParams Field Id Enumeration
 */
typedef enum
{
  cbLedBlingParamsFIdReserved           = 0,    ///< reserved field id
  cbLedBlingParamsFIdid                 = 1,    ///< id field id
  cbLedBlingParamsFIdmsecPeriod         = 2,    ///< msecPeriod field id
  cbLedBlingParamsFIdmsecDwell          = 3,    ///< msecDwell field id
  cbLedBlingParamsFIdrgb1               = 4,    ///< rgb1 field id
  cbLedBlingParamsFIdrgb2               = 5,    ///< rgb2 field id
  cbLedBlingParamsFIdNumOf              = 6     ///< number of fields
} cbLedBlingParamsFId_T;

/*!
 * cbLedBlingParams Field Definitions
 */
static const NMFieldDef_T cbLedBlingParamsFieldDefs[] =
{
  {
    .m_sFName                 = "id",
    .m_eFId                   = cbLedBlingParamsFIdid,
    .m_eFType                 = NMFTypeU8,
    .m_uOffset                = memberoffset(cbLedBlingParams_T, m_id),
    .m_this.m_u8.m_bits       = (byte_t)(0),
    .m_this.m_u8.m_valMin     = (byte_t)(0),
    .m_this.m_u8.m_valMax     = (byte_t)(0),
    .m_this.m_u8.m_valConst   = (byte_t)(0),
  },
  {
    .m_sFName                 = "msecPeriod",
    .m_eFId                   = cbLedBlingParamsFIdmsecPeriod,
    .m_eFType                 = NMFTypeU16,
    .m_uOffset                = memberoffset(cbLedBlingParams_T, m_msecPeriod),
    .m_this.m_u16.m_bits      = (byte_t)(0),
    .m_this.m_u16.m_valMin    = (ushort_t)(0),
    .m_this.m_u16.m_valMax    = (ushort_t)(0),
    .m_this.m_u16.m_valConst  = (ushort_t)(0),
  },
  {
    .m_sFName                 = "msecDwell",
    .m_eFId                   = cbLedBlingParamsFIdmsecDwell,
    .m_eFType                 = NMFTypeU16,
    .m_uOffset                = memberoffset(cbLedBlingParams_T, m_msecDwell),
    .m_this.m_u16.m_bits      = (byte_t)(0),
    .m_this.m_u16.m_valMin    = (ushort_t)(0),
    .m_this.m_u16.m_valMax    = (ushort_t)(0),
    .m_this.m_u16.m_valConst  = (ushort_t)(0),
  },
  {
    .m_sFName                 = "rgb1",
    .m_eFId                   = cbLedBlingParamsFIdrgb1,
    .m_eFType                 = NMFTypeU32,
    .m_uOffset                = memberoffset(cbLedBlingParams_T, m_rgb1),
    .m_this.m_u32.m_bits      = (byte_t)(0),
    .m_this.m_u32.m_valMin    = (uint_t)(0),
    .m_this.m_u32.m_valMax    = (uint_t)(0),
    .m_this.m_u32.m_valConst  = (uint_t)(0),
  },
  {
    .m_sFName                 = "rgb2",
    .m_eFId                   = cbLedBlingParamsFIdrgb2,
    .m_eFType                 = NMFTypeU32,
    .m_uOffset                = memberoffset(cbLedBlingParams_T, m_rgb2),
    .m_this.m_u32.m_bits      = (byte_t)(0),
    .m_this.m_u32.m_valMin    = (uint_t)(0),
    .m_this.m_u32.m_valMax    = (uint_t)(0),
    .m_this.m_u32.m_valConst  = (uint_t)(0),
  },
  EOFDEF
};

/*!
 * cbLedBlingParams Message Definition
 */
static const NMMsgDef_T cbLedBlingParamsMsgDef =
{
  .m_sMsgName         = "cbLedBlingParams",
  .m_eMsgId           = cbMsgIdNone,
  .m_uCount           = (size_t)(5),
  .m_pFields          = cbLedBlingParamsFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Field Type cbCogniBoostParams Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbCogniBoostParams Field Id Enumeration
 */
typedef enum
{
  cbCogniBoostParamsFIdReserved         = 0,    ///< reserved field id
  cbCogniBoostParamsFIdopBaudRate       = 1,    ///< opBaudRate field id
  cbCogniBoostParamsFIdopAutoRestore    = 2,    ///< opAutoRestore field id
  cbCogniBoostParamsFIdopAutoSleep      = 3,    ///< opAutoSleep field id
  cbCogniBoostParamsFIdopLedBling       = 4,    ///< opLedBling field id
  cbCogniBoostParamsFIdcmClassifier     = 5,    ///< cmClassifier field id
  cbCogniBoostParamsFIdcmContext        = 6,    ///< cmContext field id
  cbCogniBoostParamsFIdcmNorm           = 7,    ///< cmNorm field id
  cbCogniBoostParamsFIdcmMinif          = 8,    ///< cmMinif field id
  cbCogniBoostParamsFIdcmMaxif          = 9,    ///< cmMaxif field id
  cbCogniBoostParamsFIdcmMaxClassified  = 10,   ///< cmMaxClassified field id
  cbCogniBoostParamsFIdnnLabel          = 11,   ///< nnLabel field id
  cbCogniBoostParamsFIdNumOf            = 12    ///< number of fields
} cbCogniBoostParamsFId_T;

/*!
 * cbCogniBoostParams Field Definitions
 */
static const NMFieldDef_T cbCogniBoostParamsFieldDefs[] =
{
  {
    .m_sFName                 = "opBaudRate",
    .m_eFId                   = cbCogniBoostParamsFIdopBaudRate,
    .m_eFType                 = NMFTypeU32,
    .m_uOffset                = memberoffset(cbCogniBoostParams_T, m_opBaudRate),
    .m_this.m_u32.m_bits      = (byte_t)(0),
    .m_this.m_u32.m_valMin    = (uint_t)(0),
    .m_this.m_u32.m_valMax    = (uint_t)(0),
    .m_this.m_u32.m_valConst  = (uint_t)(0),
  },
  {
    .m_sFName                 = "opAutoRestore",
    .m_eFId                   = cbCogniBoostParamsFIdopAutoRestore,
    .m_eFType                 = NMFTypeBool,
    .m_uOffset                = memberoffset(cbCogniBoostParams_T, m_opAutoRestore),
  },
  {
    .m_sFName                 = "opAutoSleep",
    .m_eFId                   = cbCogniBoostParamsFIdopAutoSleep,
    .m_eFType                 = NMFTypeU16,
    .m_uOffset                = memberoffset(cbCogniBoostParams_T, m_opAutoSleep),
    .m_this.m_u16.m_bits      = (byte_t)(0),
    .m_this.m_u16.m_valMin    = (ushort_t)(0),
    .m_this.m_u16.m_valMax    = (ushort_t)(0),
    .m_this.m_u16.m_valConst  = (ushort_t)(0),
  },
  {
    .m_sFName                 = "opLedBling",
    .m_eFId                   = cbCogniBoostParamsFIdopLedBling,
    .m_eFType                 = NMFTypeStruct,
    .m_uOffset                = memberoffset(cbCogniBoostParams_T, m_opLedBling),
    .m_this.m_struct          = &cbLedBlingParamsMsgDef,
  },
  {
    .m_sFName                 = "cmClassifier",
    .m_eFId                   = cbCogniBoostParamsFIdcmClassifier,
    .m_eFType                 = NMFTypeU8,
    .m_uOffset                = memberoffset(cbCogniBoostParams_T, m_cmClassifier),
    .m_this.m_u8.m_bits       = (byte_t)(0),
    .m_this.m_u8.m_valMin     = (byte_t)(0),
    .m_this.m_u8.m_valMax     = (byte_t)(0),
    .m_this.m_u8.m_valConst   = (byte_t)(0),
  },
  {
    .m_sFName                 = "cmContext",
    .m_eFId                   = cbCogniBoostParamsFIdcmContext,
    .m_eFType                 = NMFTypeU8,
    .m_uOffset                = memberoffset(cbCogniBoostParams_T, m_cmContext),
    .m_this.m_u8.m_bits       = (byte_t)(0),
    .m_this.m_u8.m_valMin     = (byte_t)(0),
    .m_this.m_u8.m_valMax     = (byte_t)(0),
    .m_this.m_u8.m_valConst   = (byte_t)(0),
  },
  {
    .m_sFName                 = "cmNorm",
    .m_eFId                   = cbCogniBoostParamsFIdcmNorm,
    .m_eFType                 = NMFTypeU8,
    .m_uOffset                = memberoffset(cbCogniBoostParams_T, m_cmNorm),
    .m_this.m_u8.m_bits       = (byte_t)(0),
    .m_this.m_u8.m_valMin     = (byte_t)(0),
    .m_this.m_u8.m_valMax     = (byte_t)(0),
    .m_this.m_u8.m_valConst   = (byte_t)(0),
  },
  {
    .m_sFName                 = "cmMinif",
    .m_eFId                   = cbCogniBoostParamsFIdcmMinif,
    .m_eFType                 = NMFTypeU16,
    .m_uOffset                = memberoffset(cbCogniBoostParams_T, m_cmMinif),
    .m_this.m_u16.m_bits      = (byte_t)(0),
    .m_this.m_u16.m_valMin    = (ushort_t)(0),
    .m_this.m_u16.m_valMax    = (ushort_t)(0),
    .m_this.m_u16.m_valConst  = (ushort_t)(0),
  },
  {
    .m_sFName                 = "cmMaxif",
    .m_eFId                   = cbCogniBoostParamsFIdcmMaxif,
    .m_eFType                 = NMFTypeU16,
    .m_uOffset                = memberoffset(cbCogniBoostParams_T, m_cmMaxif),
    .m_this.m_u16.m_bits      = (byte_t)(0),
    .m_this.m_u16.m_valMin    = (ushort_t)(0),
    .m_this.m_u16.m_valMax    = (ushort_t)(0),
    .m_this.m_u16.m_valConst  = (ushort_t)(0),
  },
  {
    .m_sFName                 = "cmMaxClassified",
    .m_eFId                   = cbCogniBoostParamsFIdcmMaxClassified,
    .m_eFType                 = NMFTypeU16,
    .m_uOffset                = memberoffset(cbCogniBoostParams_T, m_cmMaxClassified),
    .m_this.m_u16.m_bits      = (byte_t)(0),
    .m_this.m_u16.m_valMin    = (ushort_t)(0),
    .m_this.m_u16.m_valMax    = (ushort_t)(0),
    .m_this.m_u16.m_valConst  = (ushort_t)(0),
  },
  {
    .m_sFName                 = "nnLabel",
    .m_eFId                   = cbCogniBoostParamsFIdnnLabel,
    .m_eFType                 = NMFTypeString,
    .m_uOffset                = memberoffset(cbCogniBoostParams_T, m_nnLabel),
    .m_this.m_string.m_uMaxCount
                              = (size_t)CB_PARAM_NN_LABEL_LEN_MAX+1,
    .m_this.m_string.m_sConst = (char *)(NULL),
  },
  EOFDEF
};

/*!
 * cbCogniBoostParams Message Definition
 */
static const NMMsgDef_T cbCogniBoostParamsMsgDef =
{
  .m_sMsgName         = "cbCogniBoostParams",
  .m_eMsgId           = cbMsgIdNone,
  .m_uCount           = (size_t)(11),
  .m_pFields          = cbCogniBoostParamsFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Field Type cbNeuron Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbNeuronpattern Field Definitions
 */
static const NMFieldDef_T cbNeuronpatternFieldDef[] =
{
  {
    .m_sFName                 = "pattern",
    .m_eFId                   = 0,
    .m_eFType                 = NMFTypeU8,
    .m_uOffset                = (size_t)0,
    .m_this.m_u8.m_bits       = (byte_t)(0),
    .m_this.m_u8.m_valMin     = (byte_t)(0),
    .m_this.m_u8.m_valMax     = (byte_t)(0),
    .m_this.m_u8.m_valConst   = (byte_t)(0),
  },
  EOFDEF
};

/*!
 * cbNeuron Field Id Enumeration
 */
typedef enum
{
  cbNeuronFIdReserved                   = 0,    ///< reserved field id
  cbNeuronFIdneuronId                   = 1,    ///< neuronId field id
  cbNeuronFIdcontext                    = 2,    ///< context field id
  cbNeuronFIdnorm                       = 3,    ///< norm field id
  cbNeuronFIdaif                        = 4,    ///< aif field id
  cbNeuronFIdcategory                   = 5,    ///< category field id
  cbNeuronFIdpattern                    = 6,    ///< pattern field id
  cbNeuronFIdNumOf                      = 7     ///< number of fields
} cbNeuronFId_T;

/*!
 * cbNeuron Field Definitions
 */
static const NMFieldDef_T cbNeuronFieldDefs[] =
{
  {
    .m_sFName                 = "neuronId",
    .m_eFId                   = cbNeuronFIdneuronId,
    .m_eFType                 = NMFTypeU16,
    .m_uOffset                = memberoffset(cbNeuron_T, m_neuronId),
    .m_this.m_u16.m_bits      = (byte_t)(0),
    .m_this.m_u16.m_valMin    = (ushort_t)(0),
    .m_this.m_u16.m_valMax    = (ushort_t)(0),
    .m_this.m_u16.m_valConst  = (ushort_t)(0),
  },
  {
    .m_sFName                 = "context",
    .m_eFId                   = cbNeuronFIdcontext,
    .m_eFType                 = NMFTypeU8,
    .m_uOffset                = memberoffset(cbNeuron_T, m_context),
    .m_this.m_u8.m_bits       = (byte_t)(0),
    .m_this.m_u8.m_valMin     = (byte_t)(0),
    .m_this.m_u8.m_valMax     = (byte_t)(0),
    .m_this.m_u8.m_valConst   = (byte_t)(0),
  },
  {
    .m_sFName                 = "norm",
    .m_eFId                   = cbNeuronFIdnorm,
    .m_eFType                 = NMFTypeU8,
    .m_uOffset                = memberoffset(cbNeuron_T, m_norm),
    .m_this.m_u8.m_bits       = (byte_t)(0),
    .m_this.m_u8.m_valMin     = (byte_t)(0),
    .m_this.m_u8.m_valMax     = (byte_t)(0),
    .m_this.m_u8.m_valConst   = (byte_t)(0),
  },
  {
    .m_sFName                 = "aif",
    .m_eFId                   = cbNeuronFIdaif,
    .m_eFType                 = NMFTypeU16,
    .m_uOffset                = memberoffset(cbNeuron_T, m_aif),
    .m_this.m_u16.m_bits      = (byte_t)(0),
    .m_this.m_u16.m_valMin    = (ushort_t)(0),
    .m_this.m_u16.m_valMax    = (ushort_t)(0),
    .m_this.m_u16.m_valConst  = (ushort_t)(0),
  },
  {
    .m_sFName                 = "category",
    .m_eFId                   = cbNeuronFIdcategory,
    .m_eFType                 = NMFTypeU16,
    .m_uOffset                = memberoffset(cbNeuron_T, m_category),
    .m_this.m_u16.m_bits      = (byte_t)(0),
    .m_this.m_u16.m_valMin    = (ushort_t)(0),
    .m_this.m_u16.m_valMax    = (ushort_t)(0),
    .m_this.m_u16.m_valConst  = (ushort_t)(0),
  },
  {
    .m_sFName                 = "pattern",
    .m_eFId                   = cbNeuronFIdpattern,
    .m_eFType                 = NMFTypeVector,
    .m_uOffset                = memberoffset(cbNeuron_T, m_pattern),
    .m_this.m_vector.m_uMaxCount
                              = (size_t)CM_PATTERN_COMP_NUMOF,
    .m_this.m_vector.m_uElemSize
                              = sizeof(byte_t),
    .m_this.m_vector.m_pThisElem
                              = cbNeuronpatternFieldDef,
  },
  EOFDEF
};

/*!
 * cbNeuron Message Definition
 */
static const NMMsgDef_T cbNeuronMsgDef =
{
  .m_sMsgName         = "cbNeuron",
  .m_eMsgId           = cbMsgIdNone,
  .m_uCount           = (size_t)(6),
  .m_pFields          = cbNeuronFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Field Type cbClassification Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbClassification Field Id Enumeration
 */
typedef enum
{
  cbClassificationFIdReserved           = 0,    ///< reserved field id
  cbClassificationFIdcategory           = 1,    ///< category field id
  cbClassificationFIddist               = 2,    ///< dist field id
  cbClassificationFIdnid                = 3,    ///< nid field id
  cbClassificationFIdNumOf              = 4     ///< number of fields
} cbClassificationFId_T;

/*!
 * cbClassification Field Definitions
 */
static const NMFieldDef_T cbClassificationFieldDefs[] =
{
  {
    .m_sFName                 = "category",
    .m_eFId                   = cbClassificationFIdcategory,
    .m_eFType                 = NMFTypeU16,
    .m_uOffset                = memberoffset(cbClassification_T, m_category),
    .m_this.m_u16.m_bits      = (byte_t)(0),
    .m_this.m_u16.m_valMin    = (ushort_t)(0),
    .m_this.m_u16.m_valMax    = (ushort_t)(0),
    .m_this.m_u16.m_valConst  = (ushort_t)(0),
  },
  {
    .m_sFName                 = "dist",
    .m_eFId                   = cbClassificationFIddist,
    .m_eFType                 = NMFTypeU16,
    .m_uOffset                = memberoffset(cbClassification_T, m_dist),
    .m_this.m_u16.m_bits      = (byte_t)(0),
    .m_this.m_u16.m_valMin    = (ushort_t)(0),
    .m_this.m_u16.m_valMax    = (ushort_t)(0),
    .m_this.m_u16.m_valConst  = (ushort_t)(0),
  },
  {
    .m_sFName                 = "nid",
    .m_eFId                   = cbClassificationFIdnid,
    .m_eFType                 = NMFTypeU16,
    .m_uOffset                = memberoffset(cbClassification_T, m_nid),
    .m_this.m_u16.m_bits      = (byte_t)(0),
    .m_this.m_u16.m_valMin    = (ushort_t)(0),
    .m_this.m_u16.m_valMax    = (ushort_t)(0),
    .m_this.m_u16.m_valConst  = (ushort_t)(0),
  },
  EOFDEF
};

/*!
 * cbClassification Message Definition
 */
static const NMMsgDef_T cbClassificationMsgDef =
{
  .m_sMsgName         = "cbClassification",
  .m_eMsgId           = cbMsgIdNone,
  .m_uCount           = (size_t)(3),
  .m_pFields          = cbClassificationFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbRspOk Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbRspOk Field Id Enumeration
 */
typedef enum
{
  cbRspOkFIdReserved                    = 0,    ///< reserved field id
  cbRspOkFIdNumOf                       = 1     ///< number of fields
} cbRspOkFId_T;

/*!
 * cbRspOk Field Definitions
 */
static const NMFieldDef_T cbRspOkFieldDefs[] =
{
  EOFDEF
};

/*!
 * cbRspOk Message Definition
 */
static const NMMsgDef_T cbRspOkMsgDef =
{
  .m_sMsgName         = "cbRspOk",
  .m_eMsgId           = cbMsgIdRspOk,
  .m_uCount           = (size_t)(0),
  .m_pFields          = cbRspOkFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbRspErr Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbRspErr Field Id Enumeration
 */
typedef enum
{
  cbRspErrFIdReserved                   = 0,    ///< reserved field id
  cbRspErrFIdecode                      = 1,    ///< ecode field id
  cbRspErrFIdemsg                       = 2,    ///< emsg field id
  cbRspErrFIdNumOf                      = 3     ///< number of fields
} cbRspErrFId_T;

/*!
 * cbRspErr Field Definitions
 */
static const NMFieldDef_T cbRspErrFieldDefs[] =
{
  {
    .m_sFName                 = "ecode",
    .m_eFId                   = cbRspErrFIdecode,
    .m_eFType                 = NMFTypeU8,
    .m_uOffset                = memberoffset(cbRspErr_T, m_ecode),
    .m_this.m_u8.m_bits       = (byte_t)(0),
    .m_this.m_u8.m_valMin     = (byte_t)(0),
    .m_this.m_u8.m_valMax     = (byte_t)(0),
    .m_this.m_u8.m_valConst   = (byte_t)(0),
  },
  {
    .m_sFName                 = "emsg",
    .m_eFId                   = cbRspErrFIdemsg,
    .m_eFType                 = NMFTypeString,
    .m_uOffset                = memberoffset(cbRspErr_T, m_emsg),
    .m_this.m_string.m_uMaxCount
                              = (size_t)CB_RSPERR_EMSG_LEN+1,
    .m_this.m_string.m_sConst = (char *)(NULL),
  },
  EOFDEF
};

/*!
 * cbRspErr Message Definition
 */
static const NMMsgDef_T cbRspErrMsgDef =
{
  .m_sMsgName         = "cbRspErr",
  .m_eMsgId           = cbMsgIdRspErr,
  .m_uCount           = (size_t)(2),
  .m_pFields          = cbRspErrFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqVersion Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbReqVersion Field Id Enumeration
 */
typedef enum
{
  cbReqVersionFIdReserved               = 0,    ///< reserved field id
  cbReqVersionFIdNumOf                  = 1     ///< number of fields
} cbReqVersionFId_T;

/*!
 * cbReqVersion Field Definitions
 */
static const NMFieldDef_T cbReqVersionFieldDefs[] =
{
  EOFDEF
};

/*!
 * cbReqVersion Message Definition
 */
static const NMMsgDef_T cbReqVersionMsgDef =
{
  .m_sMsgName         = "cbReqVersion",
  .m_eMsgId           = cbMsgIdReqVersion,
  .m_uCount           = (size_t)(0),
  .m_pFields          = cbReqVersionFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbRspVersion Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbRspVersion Field Id Enumeration
 */
typedef enum
{
  cbRspVersionFIdReserved               = 0,    ///< reserved field id
  cbRspVersionFIdmfgName                = 1,    ///< mfgName field id
  cbRspVersionFIdprodName               = 2,    ///< prodName field id
  cbRspVersionFIdhwSN                   = 3,    ///< hwSN field id
  cbRspVersionFIdhwVer                  = 4,    ///< hwVer field id
  cbRspVersionFIdfwApp                  = 5,    ///< fwApp field id
  cbRspVersionFIdfwVer                  = 6,    ///< fwVer field id
  cbRspVersionFIdfwDate                 = 7,    ///< fwDate field id
  cbRspVersionFIdNumOf                  = 8     ///< number of fields
} cbRspVersionFId_T;

/*!
 * cbRspVersion Field Definitions
 */
static const NMFieldDef_T cbRspVersionFieldDefs[] =
{
  {
    .m_sFName                 = "mfgName",
    .m_eFId                   = cbRspVersionFIdmfgName,
    .m_eFType                 = NMFTypeString,
    .m_uOffset                = memberoffset(cbRspVersion_T, m_mfgName),
    .m_this.m_string.m_uMaxCount
                              = (size_t)CB_ID_STR_LEN_MAX+1,
    .m_this.m_string.m_sConst = (char *)(NULL),
  },
  {
    .m_sFName                 = "prodName",
    .m_eFId                   = cbRspVersionFIdprodName,
    .m_eFType                 = NMFTypeString,
    .m_uOffset                = memberoffset(cbRspVersion_T, m_prodName),
    .m_this.m_string.m_uMaxCount
                              = (size_t)CB_ID_STR_LEN_MAX+1,
    .m_this.m_string.m_sConst = (char *)(NULL),
  },
  {
    .m_sFName                 = "hwSN",
    .m_eFId                   = cbRspVersionFIdhwSN,
    .m_eFType                 = NMFTypeString,
    .m_uOffset                = memberoffset(cbRspVersion_T, m_hwSN),
    .m_this.m_string.m_uMaxCount
                              = (size_t)CB_ID_STR_LEN_MAX+1,
    .m_this.m_string.m_sConst = (char *)(NULL),
  },
  {
    .m_sFName                 = "hwVer",
    .m_eFId                   = cbRspVersionFIdhwVer,
    .m_eFType                 = NMFTypeString,
    .m_uOffset                = memberoffset(cbRspVersion_T, m_hwVer),
    .m_this.m_string.m_uMaxCount
                              = (size_t)CB_ID_STR_LEN_MAX+1,
    .m_this.m_string.m_sConst = (char *)(NULL),
  },
  {
    .m_sFName                 = "fwApp",
    .m_eFId                   = cbRspVersionFIdfwApp,
    .m_eFType                 = NMFTypeString,
    .m_uOffset                = memberoffset(cbRspVersion_T, m_fwApp),
    .m_this.m_string.m_uMaxCount
                              = (size_t)CB_ID_STR_LEN_MAX+1,
    .m_this.m_string.m_sConst = (char *)(NULL),
  },
  {
    .m_sFName                 = "fwVer",
    .m_eFId                   = cbRspVersionFIdfwVer,
    .m_eFType                 = NMFTypeString,
    .m_uOffset                = memberoffset(cbRspVersion_T, m_fwVer),
    .m_this.m_string.m_uMaxCount
                              = (size_t)CB_ID_STR_LEN_MAX+1,
    .m_this.m_string.m_sConst = (char *)(NULL),
  },
  {
    .m_sFName                 = "fwDate",
    .m_eFId                   = cbRspVersionFIdfwDate,
    .m_eFType                 = NMFTypeString,
    .m_uOffset                = memberoffset(cbRspVersion_T, m_fwDate),
    .m_this.m_string.m_uMaxCount
                              = (size_t)CB_ID_STR_LEN_MAX+1,
    .m_this.m_string.m_sConst = (char *)(NULL),
  },
  EOFDEF
};

/*!
 * cbRspVersion Message Definition
 */
static const NMMsgDef_T cbRspVersionMsgDef =
{
  .m_sMsgName         = "cbRspVersion",
  .m_eMsgId           = cbMsgIdRspVersion,
  .m_uCount           = (size_t)(7),
  .m_pFields          = cbRspVersionFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqPing Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbReqPing Field Id Enumeration
 */
typedef enum
{
  cbReqPingFIdReserved                  = 0,    ///< reserved field id
  cbReqPingFIdNumOf                     = 1     ///< number of fields
} cbReqPingFId_T;

/*!
 * cbReqPing Field Definitions
 */
static const NMFieldDef_T cbReqPingFieldDefs[] =
{
  EOFDEF
};

/*!
 * cbReqPing Message Definition
 */
static const NMMsgDef_T cbReqPingMsgDef =
{
  .m_sMsgName         = "cbReqPing",
  .m_eMsgId           = cbMsgIdReqPing,
  .m_uCount           = (size_t)(0),
  .m_pFields          = cbReqPingFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqLoopback Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbReqLoopbackcdata Field Definitions
 */
static const NMFieldDef_T cbReqLoopbackcdataFieldDef[] =
{
  {
    .m_sFName                 = "cdata",
    .m_eFId                   = 0,
    .m_eFType                 = NMFTypeU8,
    .m_uOffset                = (size_t)0,
    .m_this.m_u8.m_bits       = (byte_t)(0),
    .m_this.m_u8.m_valMin     = (byte_t)(0),
    .m_this.m_u8.m_valMax     = (byte_t)(0),
    .m_this.m_u8.m_valConst   = (byte_t)(0),
  },
  EOFDEF
};

/*!
 * cbReqLoopback Field Id Enumeration
 */
typedef enum
{
  cbReqLoopbackFIdReserved              = 0,    ///< reserved field id
  cbReqLoopbackFIdcdata                 = 1,    ///< cdata field id
  cbReqLoopbackFIdNumOf                 = 2     ///< number of fields
} cbReqLoopbackFId_T;

/*!
 * cbReqLoopback Field Definitions
 */
static const NMFieldDef_T cbReqLoopbackFieldDefs[] =
{
  {
    .m_sFName                 = "cdata",
    .m_eFId                   = cbReqLoopbackFIdcdata,
    .m_eFType                 = NMFTypeVector,
    .m_uOffset                = memberoffset(cbReqLoopback_T, m_cdata),
    .m_this.m_vector.m_uMaxCount
                              = (size_t)CB_REQLOOPBACK_CDATA_LEN,
    .m_this.m_vector.m_uElemSize
                              = sizeof(byte_t),
    .m_this.m_vector.m_pThisElem
                              = cbReqLoopbackcdataFieldDef,
  },
  EOFDEF
};

/*!
 * cbReqLoopback Message Definition
 */
static const NMMsgDef_T cbReqLoopbackMsgDef =
{
  .m_sMsgName         = "cbReqLoopback",
  .m_eMsgId           = cbMsgIdReqLoopback,
  .m_uCount           = (size_t)(1),
  .m_pFields          = cbReqLoopbackFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbRspLoopback Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbRspLoopbackcdata Field Definitions
 */
static const NMFieldDef_T cbRspLoopbackcdataFieldDef[] =
{
  {
    .m_sFName                 = "cdata",
    .m_eFId                   = 0,
    .m_eFType                 = NMFTypeU8,
    .m_uOffset                = (size_t)0,
    .m_this.m_u8.m_bits       = (byte_t)(0),
    .m_this.m_u8.m_valMin     = (byte_t)(0),
    .m_this.m_u8.m_valMax     = (byte_t)(0),
    .m_this.m_u8.m_valConst   = (byte_t)(0),
  },
  EOFDEF
};

/*!
 * cbRspLoopback Field Id Enumeration
 */
typedef enum
{
  cbRspLoopbackFIdReserved              = 0,    ///< reserved field id
  cbRspLoopbackFIdcdata                 = 1,    ///< cdata field id
  cbRspLoopbackFIdNumOf                 = 2     ///< number of fields
} cbRspLoopbackFId_T;

/*!
 * cbRspLoopback Field Definitions
 */
static const NMFieldDef_T cbRspLoopbackFieldDefs[] =
{
  {
    .m_sFName                 = "cdata",
    .m_eFId                   = cbRspLoopbackFIdcdata,
    .m_eFType                 = NMFTypeVector,
    .m_uOffset                = memberoffset(cbRspLoopback_T, m_cdata),
    .m_this.m_vector.m_uMaxCount
                              = (size_t)CB_RSPLOOPBACK_CDATA_LEN,
    .m_this.m_vector.m_uElemSize
                              = sizeof(byte_t),
    .m_this.m_vector.m_pThisElem
                              = cbRspLoopbackcdataFieldDef,
  },
  EOFDEF
};

/*!
 * cbRspLoopback Message Definition
 */
static const NMMsgDef_T cbRspLoopbackMsgDef =
{
  .m_sMsgName         = "cbRspLoopback",
  .m_eMsgId           = cbMsgIdRspLoopback,
  .m_uCount           = (size_t)(1),
  .m_pFields          = cbRspLoopbackFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqReboot Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbReqReboot Field Id Enumeration
 */
typedef enum
{
  cbReqRebootFIdReserved                = 0,    ///< reserved field id
  cbReqRebootFIdbootloader              = 1,    ///< bootloader field id
  cbReqRebootFIdNumOf                   = 2     ///< number of fields
} cbReqRebootFId_T;

/*!
 * cbReqReboot Field Definitions
 */
static const NMFieldDef_T cbReqRebootFieldDefs[] =
{
  {
    .m_sFName                 = "bootloader",
    .m_eFId                   = cbReqRebootFIdbootloader,
    .m_eFType                 = NMFTypeBool,
    .m_uOffset                = memberoffset(cbReqReboot_T, m_bootloader),
  },
  EOFDEF
};

/*!
 * cbReqReboot Message Definition
 */
static const NMMsgDef_T cbReqRebootMsgDef =
{
  .m_sMsgName         = "cbReqReboot",
  .m_eMsgId           = cbMsgIdReqReboot,
  .m_uCount           = (size_t)(1),
  .m_pFields          = cbReqRebootFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqSleep Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbReqSleep Field Id Enumeration
 */
typedef enum
{
  cbReqSleepFIdReserved                 = 0,    ///< reserved field id
  cbReqSleepFIdNumOf                    = 1     ///< number of fields
} cbReqSleepFId_T;

/*!
 * cbReqSleep Field Definitions
 */
static const NMFieldDef_T cbReqSleepFieldDefs[] =
{
  EOFDEF
};

/*!
 * cbReqSleep Message Definition
 */
static const NMMsgDef_T cbReqSleepMsgDef =
{
  .m_sMsgName         = "cbReqSleep",
  .m_eMsgId           = cbMsgIdReqSleep,
  .m_uCount           = (size_t)(0),
  .m_pFields          = cbReqSleepFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqWakeUp Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbReqWakeUp Field Id Enumeration
 */
typedef enum
{
  cbReqWakeUpFIdReserved                = 0,    ///< reserved field id
  cbReqWakeUpFIdNumOf                   = 1     ///< number of fields
} cbReqWakeUpFId_T;

/*!
 * cbReqWakeUp Field Definitions
 */
static const NMFieldDef_T cbReqWakeUpFieldDefs[] =
{
  EOFDEF
};

/*!
 * cbReqWakeUp Message Definition
 */
static const NMMsgDef_T cbReqWakeUpMsgDef =
{
  .m_sMsgName         = "cbReqWakeUp",
  .m_eMsgId           = cbMsgIdReqWakeUp,
  .m_uCount           = (size_t)(0),
  .m_pFields          = cbReqWakeUpFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqSetOpParamBaudRate Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbReqSetOpParamBaudRate Field Id Enumeration
 */
typedef enum
{
  cbReqSetOpParamBaudRateFIdReserved    = 0,    ///< reserved field id
  cbReqSetOpParamBaudRateFIdbaudrate    = 1,    ///< baudrate field id
  cbReqSetOpParamBaudRateFIdNumOf       = 2     ///< number of fields
} cbReqSetOpParamBaudRateFId_T;

/*!
 * cbReqSetOpParamBaudRate Field Definitions
 */
static const NMFieldDef_T cbReqSetOpParamBaudRateFieldDefs[] =
{
  {
    .m_sFName                 = "baudrate",
    .m_eFId                   = cbReqSetOpParamBaudRateFIdbaudrate,
    .m_eFType                 = NMFTypeU32,
    .m_uOffset                = memberoffset(cbReqSetOpParamBaudRate_T, m_baudrate),
    .m_this.m_u32.m_bits      = (byte_t)(0),
    .m_this.m_u32.m_valMin    = (uint_t)(0),
    .m_this.m_u32.m_valMax    = (uint_t)(0),
    .m_this.m_u32.m_valConst  = (uint_t)(0),
  },
  EOFDEF
};

/*!
 * cbReqSetOpParamBaudRate Message Definition
 */
static const NMMsgDef_T cbReqSetOpParamBaudRateMsgDef =
{
  .m_sMsgName         = "cbReqSetOpParamBaudRate",
  .m_eMsgId           = cbMsgIdReqSetOpParamBaudRate,
  .m_uCount           = (size_t)(1),
  .m_pFields          = cbReqSetOpParamBaudRateFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqGetOpParamBaudRate Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbReqGetOpParamBaudRate Field Id Enumeration
 */
typedef enum
{
  cbReqGetOpParamBaudRateFIdReserved    = 0,    ///< reserved field id
  cbReqGetOpParamBaudRateFIdNumOf       = 1     ///< number of fields
} cbReqGetOpParamBaudRateFId_T;

/*!
 * cbReqGetOpParamBaudRate Field Definitions
 */
static const NMFieldDef_T cbReqGetOpParamBaudRateFieldDefs[] =
{
  EOFDEF
};

/*!
 * cbReqGetOpParamBaudRate Message Definition
 */
static const NMMsgDef_T cbReqGetOpParamBaudRateMsgDef =
{
  .m_sMsgName         = "cbReqGetOpParamBaudRate",
  .m_eMsgId           = cbMsgIdReqGetOpParamBaudRate,
  .m_uCount           = (size_t)(0),
  .m_pFields          = cbReqGetOpParamBaudRateFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbRspGetOpParamBaudRate Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbRspGetOpParamBaudRate Field Id Enumeration
 */
typedef enum
{
  cbRspGetOpParamBaudRateFIdReserved    = 0,    ///< reserved field id
  cbRspGetOpParamBaudRateFIdbaudrate    = 1,    ///< baudrate field id
  cbRspGetOpParamBaudRateFIdNumOf       = 2     ///< number of fields
} cbRspGetOpParamBaudRateFId_T;

/*!
 * cbRspGetOpParamBaudRate Field Definitions
 */
static const NMFieldDef_T cbRspGetOpParamBaudRateFieldDefs[] =
{
  {
    .m_sFName                 = "baudrate",
    .m_eFId                   = cbRspGetOpParamBaudRateFIdbaudrate,
    .m_eFType                 = NMFTypeU32,
    .m_uOffset                = memberoffset(cbRspGetOpParamBaudRate_T, m_baudrate),
    .m_this.m_u32.m_bits      = (byte_t)(0),
    .m_this.m_u32.m_valMin    = (uint_t)(0),
    .m_this.m_u32.m_valMax    = (uint_t)(0),
    .m_this.m_u32.m_valConst  = (uint_t)(0),
  },
  EOFDEF
};

/*!
 * cbRspGetOpParamBaudRate Message Definition
 */
static const NMMsgDef_T cbRspGetOpParamBaudRateMsgDef =
{
  .m_sMsgName         = "cbRspGetOpParamBaudRate",
  .m_eMsgId           = cbMsgIdRspGetOpParamBaudRate,
  .m_uCount           = (size_t)(1),
  .m_pFields          = cbRspGetOpParamBaudRateFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqSetOpParamAutoRestore Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbReqSetOpParamAutoRestore Field Id Enumeration
 */
typedef enum
{
  cbReqSetOpParamAutoRestoreFIdReserved = 0,    ///< reserved field id
  cbReqSetOpParamAutoRestoreFIdenable   = 1,    ///< enable field id
  cbReqSetOpParamAutoRestoreFIdNumOf    = 2     ///< number of fields
} cbReqSetOpParamAutoRestoreFId_T;

/*!
 * cbReqSetOpParamAutoRestore Field Definitions
 */
static const NMFieldDef_T cbReqSetOpParamAutoRestoreFieldDefs[] =
{
  {
    .m_sFName                 = "enable",
    .m_eFId                   = cbReqSetOpParamAutoRestoreFIdenable,
    .m_eFType                 = NMFTypeBool,
    .m_uOffset                = memberoffset(cbReqSetOpParamAutoRestore_T, m_enable),
  },
  EOFDEF
};

/*!
 * cbReqSetOpParamAutoRestore Message Definition
 */
static const NMMsgDef_T cbReqSetOpParamAutoRestoreMsgDef =
{
  .m_sMsgName         = "cbReqSetOpParamAutoRestore",
  .m_eMsgId           = cbMsgIdReqSetOpParamAutoRestore,
  .m_uCount           = (size_t)(1),
  .m_pFields          = cbReqSetOpParamAutoRestoreFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqGetOpParamAutoRestore Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbReqGetOpParamAutoRestore Field Id Enumeration
 */
typedef enum
{
  cbReqGetOpParamAutoRestoreFIdReserved = 0,    ///< reserved field id
  cbReqGetOpParamAutoRestoreFIdNumOf    = 1     ///< number of fields
} cbReqGetOpParamAutoRestoreFId_T;

/*!
 * cbReqGetOpParamAutoRestore Field Definitions
 */
static const NMFieldDef_T cbReqGetOpParamAutoRestoreFieldDefs[] =
{
  EOFDEF
};

/*!
 * cbReqGetOpParamAutoRestore Message Definition
 */
static const NMMsgDef_T cbReqGetOpParamAutoRestoreMsgDef =
{
  .m_sMsgName         = "cbReqGetOpParamAutoRestore",
  .m_eMsgId           = cbMsgIdReqGetOpParamAutoRestore,
  .m_uCount           = (size_t)(0),
  .m_pFields          = cbReqGetOpParamAutoRestoreFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbRspGetOpParamAutoRestore Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbRspGetOpParamAutoRestore Field Id Enumeration
 */
typedef enum
{
  cbRspGetOpParamAutoRestoreFIdReserved = 0,    ///< reserved field id
  cbRspGetOpParamAutoRestoreFIdenable   = 1,    ///< enable field id
  cbRspGetOpParamAutoRestoreFIdNumOf    = 2     ///< number of fields
} cbRspGetOpParamAutoRestoreFId_T;

/*!
 * cbRspGetOpParamAutoRestore Field Definitions
 */
static const NMFieldDef_T cbRspGetOpParamAutoRestoreFieldDefs[] =
{
  {
    .m_sFName                 = "enable",
    .m_eFId                   = cbRspGetOpParamAutoRestoreFIdenable,
    .m_eFType                 = NMFTypeBool,
    .m_uOffset                = memberoffset(cbRspGetOpParamAutoRestore_T, m_enable),
  },
  EOFDEF
};

/*!
 * cbRspGetOpParamAutoRestore Message Definition
 */
static const NMMsgDef_T cbRspGetOpParamAutoRestoreMsgDef =
{
  .m_sMsgName         = "cbRspGetOpParamAutoRestore",
  .m_eMsgId           = cbMsgIdRspGetOpParamAutoRestore,
  .m_uCount           = (size_t)(1),
  .m_pFields          = cbRspGetOpParamAutoRestoreFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqSetOpParamAutoSleep Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbReqSetOpParamAutoSleep Field Id Enumeration
 */
typedef enum
{
  cbReqSetOpParamAutoSleepFIdReserved   = 0,    ///< reserved field id
  cbReqSetOpParamAutoSleepFIdsec        = 1,    ///< sec field id
  cbReqSetOpParamAutoSleepFIdNumOf      = 2     ///< number of fields
} cbReqSetOpParamAutoSleepFId_T;

/*!
 * cbReqSetOpParamAutoSleep Field Definitions
 */
static const NMFieldDef_T cbReqSetOpParamAutoSleepFieldDefs[] =
{
  {
    .m_sFName                 = "sec",
    .m_eFId                   = cbReqSetOpParamAutoSleepFIdsec,
    .m_eFType                 = NMFTypeU16,
    .m_uOffset                = memberoffset(cbReqSetOpParamAutoSleep_T, m_sec),
    .m_this.m_u16.m_bits      = (byte_t)(0),
    .m_this.m_u16.m_valMin    = (ushort_t)(0),
    .m_this.m_u16.m_valMax    = (ushort_t)(0),
    .m_this.m_u16.m_valConst  = (ushort_t)(0),
  },
  EOFDEF
};

/*!
 * cbReqSetOpParamAutoSleep Message Definition
 */
static const NMMsgDef_T cbReqSetOpParamAutoSleepMsgDef =
{
  .m_sMsgName         = "cbReqSetOpParamAutoSleep",
  .m_eMsgId           = cbMsgIdReqSetOpParamAutoSleep,
  .m_uCount           = (size_t)(1),
  .m_pFields          = cbReqSetOpParamAutoSleepFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqGetOpParamAutoSleep Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbReqGetOpParamAutoSleep Field Id Enumeration
 */
typedef enum
{
  cbReqGetOpParamAutoSleepFIdReserved   = 0,    ///< reserved field id
  cbReqGetOpParamAutoSleepFIdNumOf      = 1     ///< number of fields
} cbReqGetOpParamAutoSleepFId_T;

/*!
 * cbReqGetOpParamAutoSleep Field Definitions
 */
static const NMFieldDef_T cbReqGetOpParamAutoSleepFieldDefs[] =
{
  EOFDEF
};

/*!
 * cbReqGetOpParamAutoSleep Message Definition
 */
static const NMMsgDef_T cbReqGetOpParamAutoSleepMsgDef =
{
  .m_sMsgName         = "cbReqGetOpParamAutoSleep",
  .m_eMsgId           = cbMsgIdReqGetOpParamAutoSleep,
  .m_uCount           = (size_t)(0),
  .m_pFields          = cbReqGetOpParamAutoSleepFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbRspGetOpParamAutoSleep Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbRspGetOpParamAutoSleep Field Id Enumeration
 */
typedef enum
{
  cbRspGetOpParamAutoSleepFIdReserved   = 0,    ///< reserved field id
  cbRspGetOpParamAutoSleepFIdsec        = 1,    ///< sec field id
  cbRspGetOpParamAutoSleepFIdNumOf      = 2     ///< number of fields
} cbRspGetOpParamAutoSleepFId_T;

/*!
 * cbRspGetOpParamAutoSleep Field Definitions
 */
static const NMFieldDef_T cbRspGetOpParamAutoSleepFieldDefs[] =
{
  {
    .m_sFName                 = "sec",
    .m_eFId                   = cbRspGetOpParamAutoSleepFIdsec,
    .m_eFType                 = NMFTypeU16,
    .m_uOffset                = memberoffset(cbRspGetOpParamAutoSleep_T, m_sec),
    .m_this.m_u16.m_bits      = (byte_t)(0),
    .m_this.m_u16.m_valMin    = (ushort_t)(0),
    .m_this.m_u16.m_valMax    = (ushort_t)(0),
    .m_this.m_u16.m_valConst  = (ushort_t)(0),
  },
  EOFDEF
};

/*!
 * cbRspGetOpParamAutoSleep Message Definition
 */
static const NMMsgDef_T cbRspGetOpParamAutoSleepMsgDef =
{
  .m_sMsgName         = "cbRspGetOpParamAutoSleep",
  .m_eMsgId           = cbMsgIdRspGetOpParamAutoSleep,
  .m_uCount           = (size_t)(1),
  .m_pFields          = cbRspGetOpParamAutoSleepFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqSetOpParamLedBling Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbReqSetOpParamLedBling Field Id Enumeration
 */
typedef enum
{
  cbReqSetOpParamLedBlingFIdReserved    = 0,    ///< reserved field id
  cbReqSetOpParamLedBlingFIdparams      = 1,    ///< params field id
  cbReqSetOpParamLedBlingFIdNumOf       = 2     ///< number of fields
} cbReqSetOpParamLedBlingFId_T;

/*!
 * cbReqSetOpParamLedBling Field Definitions
 */
static const NMFieldDef_T cbReqSetOpParamLedBlingFieldDefs[] =
{
  {
    .m_sFName                 = "params",
    .m_eFId                   = cbReqSetOpParamLedBlingFIdparams,
    .m_eFType                 = NMFTypeStruct,
    .m_uOffset                = memberoffset(cbReqSetOpParamLedBling_T, m_params),
    .m_this.m_struct          = &cbLedBlingParamsMsgDef,
  },
  EOFDEF
};

/*!
 * cbReqSetOpParamLedBling Message Definition
 */
static const NMMsgDef_T cbReqSetOpParamLedBlingMsgDef =
{
  .m_sMsgName         = "cbReqSetOpParamLedBling",
  .m_eMsgId           = cbMsgIdReqSetOpParamLedBling,
  .m_uCount           = (size_t)(1),
  .m_pFields          = cbReqSetOpParamLedBlingFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqGetOpParamLedBling Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbReqGetOpParamLedBling Field Id Enumeration
 */
typedef enum
{
  cbReqGetOpParamLedBlingFIdReserved    = 0,    ///< reserved field id
  cbReqGetOpParamLedBlingFIdNumOf       = 1     ///< number of fields
} cbReqGetOpParamLedBlingFId_T;

/*!
 * cbReqGetOpParamLedBling Field Definitions
 */
static const NMFieldDef_T cbReqGetOpParamLedBlingFieldDefs[] =
{
  EOFDEF
};

/*!
 * cbReqGetOpParamLedBling Message Definition
 */
static const NMMsgDef_T cbReqGetOpParamLedBlingMsgDef =
{
  .m_sMsgName         = "cbReqGetOpParamLedBling",
  .m_eMsgId           = cbMsgIdReqGetOpParamLedBling,
  .m_uCount           = (size_t)(0),
  .m_pFields          = cbReqGetOpParamLedBlingFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbRspGetOpParamLedBling Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbRspGetOpParamLedBling Field Id Enumeration
 */
typedef enum
{
  cbRspGetOpParamLedBlingFIdReserved    = 0,    ///< reserved field id
  cbRspGetOpParamLedBlingFIdparams      = 1,    ///< params field id
  cbRspGetOpParamLedBlingFIdNumOf       = 2     ///< number of fields
} cbRspGetOpParamLedBlingFId_T;

/*!
 * cbRspGetOpParamLedBling Field Definitions
 */
static const NMFieldDef_T cbRspGetOpParamLedBlingFieldDefs[] =
{
  {
    .m_sFName                 = "params",
    .m_eFId                   = cbRspGetOpParamLedBlingFIdparams,
    .m_eFType                 = NMFTypeStruct,
    .m_uOffset                = memberoffset(cbRspGetOpParamLedBling_T, m_params),
    .m_this.m_struct          = &cbLedBlingParamsMsgDef,
  },
  EOFDEF
};

/*!
 * cbRspGetOpParamLedBling Message Definition
 */
static const NMMsgDef_T cbRspGetOpParamLedBlingMsgDef =
{
  .m_sMsgName         = "cbRspGetOpParamLedBling",
  .m_eMsgId           = cbMsgIdRspGetOpParamLedBling,
  .m_uCount           = (size_t)(1),
  .m_pFields          = cbRspGetOpParamLedBlingFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqSetCMParamClassifier Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbReqSetCMParamClassifier Field Id Enumeration
 */
typedef enum
{
  cbReqSetCMParamClassifierFIdReserved  = 0,    ///< reserved field id
  cbReqSetCMParamClassifierFIdclassifier = 1,   ///< classifier field id
  cbReqSetCMParamClassifierFIdNumOf     = 2     ///< number of fields
} cbReqSetCMParamClassifierFId_T;

/*!
 * cbReqSetCMParamClassifier Field Definitions
 */
static const NMFieldDef_T cbReqSetCMParamClassifierFieldDefs[] =
{
  {
    .m_sFName                 = "classifier",
    .m_eFId                   = cbReqSetCMParamClassifierFIdclassifier,
    .m_eFType                 = NMFTypeU8,
    .m_uOffset                = memberoffset(cbReqSetCMParamClassifier_T, m_classifier),
    .m_this.m_u8.m_bits       = (byte_t)(0),
    .m_this.m_u8.m_valMin     = (byte_t)(0),
    .m_this.m_u8.m_valMax     = (byte_t)(0),
    .m_this.m_u8.m_valConst   = (byte_t)(0),
  },
  EOFDEF
};

/*!
 * cbReqSetCMParamClassifier Message Definition
 */
static const NMMsgDef_T cbReqSetCMParamClassifierMsgDef =
{
  .m_sMsgName         = "cbReqSetCMParamClassifier",
  .m_eMsgId           = cbMsgIdReqSetCMParamClassifier,
  .m_uCount           = (size_t)(1),
  .m_pFields          = cbReqSetCMParamClassifierFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqGetCMParamClassifier Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbReqGetCMParamClassifier Field Id Enumeration
 */
typedef enum
{
  cbReqGetCMParamClassifierFIdReserved  = 0,    ///< reserved field id
  cbReqGetCMParamClassifierFIdNumOf     = 1     ///< number of fields
} cbReqGetCMParamClassifierFId_T;

/*!
 * cbReqGetCMParamClassifier Field Definitions
 */
static const NMFieldDef_T cbReqGetCMParamClassifierFieldDefs[] =
{
  EOFDEF
};

/*!
 * cbReqGetCMParamClassifier Message Definition
 */
static const NMMsgDef_T cbReqGetCMParamClassifierMsgDef =
{
  .m_sMsgName         = "cbReqGetCMParamClassifier",
  .m_eMsgId           = cbMsgIdReqGetCMParamClassifier,
  .m_uCount           = (size_t)(0),
  .m_pFields          = cbReqGetCMParamClassifierFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbRspGetCMParamClassifier Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbRspGetCMParamClassifier Field Id Enumeration
 */
typedef enum
{
  cbRspGetCMParamClassifierFIdReserved  = 0,    ///< reserved field id
  cbRspGetCMParamClassifierFIdclassifier = 1,   ///< classifier field id
  cbRspGetCMParamClassifierFIdNumOf     = 2     ///< number of fields
} cbRspGetCMParamClassifierFId_T;

/*!
 * cbRspGetCMParamClassifier Field Definitions
 */
static const NMFieldDef_T cbRspGetCMParamClassifierFieldDefs[] =
{
  {
    .m_sFName                 = "classifier",
    .m_eFId                   = cbRspGetCMParamClassifierFIdclassifier,
    .m_eFType                 = NMFTypeU8,
    .m_uOffset                = memberoffset(cbRspGetCMParamClassifier_T, m_classifier),
    .m_this.m_u8.m_bits       = (byte_t)(0),
    .m_this.m_u8.m_valMin     = (byte_t)(0),
    .m_this.m_u8.m_valMax     = (byte_t)(0),
    .m_this.m_u8.m_valConst   = (byte_t)(0),
  },
  EOFDEF
};

/*!
 * cbRspGetCMParamClassifier Message Definition
 */
static const NMMsgDef_T cbRspGetCMParamClassifierMsgDef =
{
  .m_sMsgName         = "cbRspGetCMParamClassifier",
  .m_eMsgId           = cbMsgIdRspGetCMParamClassifier,
  .m_uCount           = (size_t)(1),
  .m_pFields          = cbRspGetCMParamClassifierFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqSetCMParamContext Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbReqSetCMParamContext Field Id Enumeration
 */
typedef enum
{
  cbReqSetCMParamContextFIdReserved     = 0,    ///< reserved field id
  cbReqSetCMParamContextFIdcontext      = 1,    ///< context field id
  cbReqSetCMParamContextFIdNumOf        = 2     ///< number of fields
} cbReqSetCMParamContextFId_T;

/*!
 * cbReqSetCMParamContext Field Definitions
 */
static const NMFieldDef_T cbReqSetCMParamContextFieldDefs[] =
{
  {
    .m_sFName                 = "context",
    .m_eFId                   = cbReqSetCMParamContextFIdcontext,
    .m_eFType                 = NMFTypeU8,
    .m_uOffset                = memberoffset(cbReqSetCMParamContext_T, m_context),
    .m_this.m_u8.m_bits       = (byte_t)(0),
    .m_this.m_u8.m_valMin     = (byte_t)(0),
    .m_this.m_u8.m_valMax     = (byte_t)(0),
    .m_this.m_u8.m_valConst   = (byte_t)(0),
  },
  EOFDEF
};

/*!
 * cbReqSetCMParamContext Message Definition
 */
static const NMMsgDef_T cbReqSetCMParamContextMsgDef =
{
  .m_sMsgName         = "cbReqSetCMParamContext",
  .m_eMsgId           = cbMsgIdReqSetCMParamContext,
  .m_uCount           = (size_t)(1),
  .m_pFields          = cbReqSetCMParamContextFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqGetCMParamContext Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbReqGetCMParamContext Field Id Enumeration
 */
typedef enum
{
  cbReqGetCMParamContextFIdReserved     = 0,    ///< reserved field id
  cbReqGetCMParamContextFIdNumOf        = 1     ///< number of fields
} cbReqGetCMParamContextFId_T;

/*!
 * cbReqGetCMParamContext Field Definitions
 */
static const NMFieldDef_T cbReqGetCMParamContextFieldDefs[] =
{
  EOFDEF
};

/*!
 * cbReqGetCMParamContext Message Definition
 */
static const NMMsgDef_T cbReqGetCMParamContextMsgDef =
{
  .m_sMsgName         = "cbReqGetCMParamContext",
  .m_eMsgId           = cbMsgIdReqGetCMParamContext,
  .m_uCount           = (size_t)(0),
  .m_pFields          = cbReqGetCMParamContextFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbRspGetCMParamContext Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbRspGetCMParamContext Field Id Enumeration
 */
typedef enum
{
  cbRspGetCMParamContextFIdReserved     = 0,    ///< reserved field id
  cbRspGetCMParamContextFIdcontext      = 1,    ///< context field id
  cbRspGetCMParamContextFIdNumOf        = 2     ///< number of fields
} cbRspGetCMParamContextFId_T;

/*!
 * cbRspGetCMParamContext Field Definitions
 */
static const NMFieldDef_T cbRspGetCMParamContextFieldDefs[] =
{
  {
    .m_sFName                 = "context",
    .m_eFId                   = cbRspGetCMParamContextFIdcontext,
    .m_eFType                 = NMFTypeU8,
    .m_uOffset                = memberoffset(cbRspGetCMParamContext_T, m_context),
    .m_this.m_u8.m_bits       = (byte_t)(0),
    .m_this.m_u8.m_valMin     = (byte_t)(0),
    .m_this.m_u8.m_valMax     = (byte_t)(0),
    .m_this.m_u8.m_valConst   = (byte_t)(0),
  },
  EOFDEF
};

/*!
 * cbRspGetCMParamContext Message Definition
 */
static const NMMsgDef_T cbRspGetCMParamContextMsgDef =
{
  .m_sMsgName         = "cbRspGetCMParamContext",
  .m_eMsgId           = cbMsgIdRspGetCMParamContext,
  .m_uCount           = (size_t)(1),
  .m_pFields          = cbRspGetCMParamContextFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqSetCMParamNorm Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbReqSetCMParamNorm Field Id Enumeration
 */
typedef enum
{
  cbReqSetCMParamNormFIdReserved        = 0,    ///< reserved field id
  cbReqSetCMParamNormFIdnorm            = 1,    ///< norm field id
  cbReqSetCMParamNormFIdNumOf           = 2     ///< number of fields
} cbReqSetCMParamNormFId_T;

/*!
 * cbReqSetCMParamNorm Field Definitions
 */
static const NMFieldDef_T cbReqSetCMParamNormFieldDefs[] =
{
  {
    .m_sFName                 = "norm",
    .m_eFId                   = cbReqSetCMParamNormFIdnorm,
    .m_eFType                 = NMFTypeU8,
    .m_uOffset                = memberoffset(cbReqSetCMParamNorm_T, m_norm),
    .m_this.m_u8.m_bits       = (byte_t)(0),
    .m_this.m_u8.m_valMin     = (byte_t)(0),
    .m_this.m_u8.m_valMax     = (byte_t)(0),
    .m_this.m_u8.m_valConst   = (byte_t)(0),
  },
  EOFDEF
};

/*!
 * cbReqSetCMParamNorm Message Definition
 */
static const NMMsgDef_T cbReqSetCMParamNormMsgDef =
{
  .m_sMsgName         = "cbReqSetCMParamNorm",
  .m_eMsgId           = cbMsgIdReqSetCMParamNorm,
  .m_uCount           = (size_t)(1),
  .m_pFields          = cbReqSetCMParamNormFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqGetCMParamNorm Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbReqGetCMParamNorm Field Id Enumeration
 */
typedef enum
{
  cbReqGetCMParamNormFIdReserved        = 0,    ///< reserved field id
  cbReqGetCMParamNormFIdNumOf           = 1     ///< number of fields
} cbReqGetCMParamNormFId_T;

/*!
 * cbReqGetCMParamNorm Field Definitions
 */
static const NMFieldDef_T cbReqGetCMParamNormFieldDefs[] =
{
  EOFDEF
};

/*!
 * cbReqGetCMParamNorm Message Definition
 */
static const NMMsgDef_T cbReqGetCMParamNormMsgDef =
{
  .m_sMsgName         = "cbReqGetCMParamNorm",
  .m_eMsgId           = cbMsgIdReqGetCMParamNorm,
  .m_uCount           = (size_t)(0),
  .m_pFields          = cbReqGetCMParamNormFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbRspGetCMParamNorm Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbRspGetCMParamNorm Field Id Enumeration
 */
typedef enum
{
  cbRspGetCMParamNormFIdReserved        = 0,    ///< reserved field id
  cbRspGetCMParamNormFIdnorm            = 1,    ///< norm field id
  cbRspGetCMParamNormFIdNumOf           = 2     ///< number of fields
} cbRspGetCMParamNormFId_T;

/*!
 * cbRspGetCMParamNorm Field Definitions
 */
static const NMFieldDef_T cbRspGetCMParamNormFieldDefs[] =
{
  {
    .m_sFName                 = "norm",
    .m_eFId                   = cbRspGetCMParamNormFIdnorm,
    .m_eFType                 = NMFTypeU8,
    .m_uOffset                = memberoffset(cbRspGetCMParamNorm_T, m_norm),
    .m_this.m_u8.m_bits       = (byte_t)(0),
    .m_this.m_u8.m_valMin     = (byte_t)(0),
    .m_this.m_u8.m_valMax     = (byte_t)(0),
    .m_this.m_u8.m_valConst   = (byte_t)(0),
  },
  EOFDEF
};

/*!
 * cbRspGetCMParamNorm Message Definition
 */
static const NMMsgDef_T cbRspGetCMParamNormMsgDef =
{
  .m_sMsgName         = "cbRspGetCMParamNorm",
  .m_eMsgId           = cbMsgIdRspGetCMParamNorm,
  .m_uCount           = (size_t)(1),
  .m_pFields          = cbRspGetCMParamNormFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqSetCMParamMinIF Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbReqSetCMParamMinIF Field Id Enumeration
 */
typedef enum
{
  cbReqSetCMParamMinIFFIdReserved       = 0,    ///< reserved field id
  cbReqSetCMParamMinIFFIdminif          = 1,    ///< minif field id
  cbReqSetCMParamMinIFFIdNumOf          = 2     ///< number of fields
} cbReqSetCMParamMinIFFId_T;

/*!
 * cbReqSetCMParamMinIF Field Definitions
 */
static const NMFieldDef_T cbReqSetCMParamMinIFFieldDefs[] =
{
  {
    .m_sFName                 = "minif",
    .m_eFId                   = cbReqSetCMParamMinIFFIdminif,
    .m_eFType                 = NMFTypeU16,
    .m_uOffset                = memberoffset(cbReqSetCMParamMinIF_T, m_minif),
    .m_this.m_u16.m_bits      = (byte_t)(0),
    .m_this.m_u16.m_valMin    = (ushort_t)(0),
    .m_this.m_u16.m_valMax    = (ushort_t)(0),
    .m_this.m_u16.m_valConst  = (ushort_t)(0),
  },
  EOFDEF
};

/*!
 * cbReqSetCMParamMinIF Message Definition
 */
static const NMMsgDef_T cbReqSetCMParamMinIFMsgDef =
{
  .m_sMsgName         = "cbReqSetCMParamMinIF",
  .m_eMsgId           = cbMsgIdReqSetCMParamMinIF,
  .m_uCount           = (size_t)(1),
  .m_pFields          = cbReqSetCMParamMinIFFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqGetCMParamMinIF Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbReqGetCMParamMinIF Field Id Enumeration
 */
typedef enum
{
  cbReqGetCMParamMinIFFIdReserved       = 0,    ///< reserved field id
  cbReqGetCMParamMinIFFIdNumOf          = 1     ///< number of fields
} cbReqGetCMParamMinIFFId_T;

/*!
 * cbReqGetCMParamMinIF Field Definitions
 */
static const NMFieldDef_T cbReqGetCMParamMinIFFieldDefs[] =
{
  EOFDEF
};

/*!
 * cbReqGetCMParamMinIF Message Definition
 */
static const NMMsgDef_T cbReqGetCMParamMinIFMsgDef =
{
  .m_sMsgName         = "cbReqGetCMParamMinIF",
  .m_eMsgId           = cbMsgIdReqGetCMParamMinIF,
  .m_uCount           = (size_t)(0),
  .m_pFields          = cbReqGetCMParamMinIFFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbRspGetCMParamMinIF Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbRspGetCMParamMinIF Field Id Enumeration
 */
typedef enum
{
  cbRspGetCMParamMinIFFIdReserved       = 0,    ///< reserved field id
  cbRspGetCMParamMinIFFIdminif          = 1,    ///< minif field id
  cbRspGetCMParamMinIFFIdNumOf          = 2     ///< number of fields
} cbRspGetCMParamMinIFFId_T;

/*!
 * cbRspGetCMParamMinIF Field Definitions
 */
static const NMFieldDef_T cbRspGetCMParamMinIFFieldDefs[] =
{
  {
    .m_sFName                 = "minif",
    .m_eFId                   = cbRspGetCMParamMinIFFIdminif,
    .m_eFType                 = NMFTypeU16,
    .m_uOffset                = memberoffset(cbRspGetCMParamMinIF_T, m_minif),
    .m_this.m_u16.m_bits      = (byte_t)(0),
    .m_this.m_u16.m_valMin    = (ushort_t)(0),
    .m_this.m_u16.m_valMax    = (ushort_t)(0),
    .m_this.m_u16.m_valConst  = (ushort_t)(0),
  },
  EOFDEF
};

/*!
 * cbRspGetCMParamMinIF Message Definition
 */
static const NMMsgDef_T cbRspGetCMParamMinIFMsgDef =
{
  .m_sMsgName         = "cbRspGetCMParamMinIF",
  .m_eMsgId           = cbMsgIdRspGetCMParamMinIF,
  .m_uCount           = (size_t)(1),
  .m_pFields          = cbRspGetCMParamMinIFFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqSetCMParamMaxIF Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbReqSetCMParamMaxIF Field Id Enumeration
 */
typedef enum
{
  cbReqSetCMParamMaxIFFIdReserved       = 0,    ///< reserved field id
  cbReqSetCMParamMaxIFFIdmaxif          = 1,    ///< maxif field id
  cbReqSetCMParamMaxIFFIdNumOf          = 2     ///< number of fields
} cbReqSetCMParamMaxIFFId_T;

/*!
 * cbReqSetCMParamMaxIF Field Definitions
 */
static const NMFieldDef_T cbReqSetCMParamMaxIFFieldDefs[] =
{
  {
    .m_sFName                 = "maxif",
    .m_eFId                   = cbReqSetCMParamMaxIFFIdmaxif,
    .m_eFType                 = NMFTypeU16,
    .m_uOffset                = memberoffset(cbReqSetCMParamMaxIF_T, m_maxif),
    .m_this.m_u16.m_bits      = (byte_t)(0),
    .m_this.m_u16.m_valMin    = (ushort_t)(0),
    .m_this.m_u16.m_valMax    = (ushort_t)(0),
    .m_this.m_u16.m_valConst  = (ushort_t)(0),
  },
  EOFDEF
};

/*!
 * cbReqSetCMParamMaxIF Message Definition
 */
static const NMMsgDef_T cbReqSetCMParamMaxIFMsgDef =
{
  .m_sMsgName         = "cbReqSetCMParamMaxIF",
  .m_eMsgId           = cbMsgIdReqSetCMParamMaxIF,
  .m_uCount           = (size_t)(1),
  .m_pFields          = cbReqSetCMParamMaxIFFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqGetCMParamMaxIF Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbReqGetCMParamMaxIF Field Id Enumeration
 */
typedef enum
{
  cbReqGetCMParamMaxIFFIdReserved       = 0,    ///< reserved field id
  cbReqGetCMParamMaxIFFIdNumOf          = 1     ///< number of fields
} cbReqGetCMParamMaxIFFId_T;

/*!
 * cbReqGetCMParamMaxIF Field Definitions
 */
static const NMFieldDef_T cbReqGetCMParamMaxIFFieldDefs[] =
{
  EOFDEF
};

/*!
 * cbReqGetCMParamMaxIF Message Definition
 */
static const NMMsgDef_T cbReqGetCMParamMaxIFMsgDef =
{
  .m_sMsgName         = "cbReqGetCMParamMaxIF",
  .m_eMsgId           = cbMsgIdReqGetCMParamMaxIF,
  .m_uCount           = (size_t)(0),
  .m_pFields          = cbReqGetCMParamMaxIFFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbRspGetCMParamMaxIF Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbRspGetCMParamMaxIF Field Id Enumeration
 */
typedef enum
{
  cbRspGetCMParamMaxIFFIdReserved       = 0,    ///< reserved field id
  cbRspGetCMParamMaxIFFIdmaxif          = 1,    ///< maxif field id
  cbRspGetCMParamMaxIFFIdNumOf          = 2     ///< number of fields
} cbRspGetCMParamMaxIFFId_T;

/*!
 * cbRspGetCMParamMaxIF Field Definitions
 */
static const NMFieldDef_T cbRspGetCMParamMaxIFFieldDefs[] =
{
  {
    .m_sFName                 = "maxif",
    .m_eFId                   = cbRspGetCMParamMaxIFFIdmaxif,
    .m_eFType                 = NMFTypeU16,
    .m_uOffset                = memberoffset(cbRspGetCMParamMaxIF_T, m_maxif),
    .m_this.m_u16.m_bits      = (byte_t)(0),
    .m_this.m_u16.m_valMin    = (ushort_t)(0),
    .m_this.m_u16.m_valMax    = (ushort_t)(0),
    .m_this.m_u16.m_valConst  = (ushort_t)(0),
  },
  EOFDEF
};

/*!
 * cbRspGetCMParamMaxIF Message Definition
 */
static const NMMsgDef_T cbRspGetCMParamMaxIFMsgDef =
{
  .m_sMsgName         = "cbRspGetCMParamMaxIF",
  .m_eMsgId           = cbMsgIdRspGetCMParamMaxIF,
  .m_uCount           = (size_t)(1),
  .m_pFields          = cbRspGetCMParamMaxIFFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqSetCMParamMaxClassified Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbReqSetCMParamMaxClassified Field Id Enumeration
 */
typedef enum
{
  cbReqSetCMParamMaxClassifiedFIdReserved = 0,  ///< reserved field id
  cbReqSetCMParamMaxClassifiedFIdmax    = 1,    ///< max field id
  cbReqSetCMParamMaxClassifiedFIdNumOf  = 2     ///< number of fields
} cbReqSetCMParamMaxClassifiedFId_T;

/*!
 * cbReqSetCMParamMaxClassified Field Definitions
 */
static const NMFieldDef_T cbReqSetCMParamMaxClassifiedFieldDefs[] =
{
  {
    .m_sFName                 = "max",
    .m_eFId                   = cbReqSetCMParamMaxClassifiedFIdmax,
    .m_eFType                 = NMFTypeU8,
    .m_uOffset                = memberoffset(cbReqSetCMParamMaxClassified_T, m_max),
    .m_this.m_u8.m_bits       = (byte_t)(0),
    .m_this.m_u8.m_valMin     = (byte_t)(0),
    .m_this.m_u8.m_valMax     = (byte_t)(0),
    .m_this.m_u8.m_valConst   = (byte_t)(0),
  },
  EOFDEF
};

/*!
 * cbReqSetCMParamMaxClassified Message Definition
 */
static const NMMsgDef_T cbReqSetCMParamMaxClassifiedMsgDef =
{
  .m_sMsgName         = "cbReqSetCMParamMaxClassified",
  .m_eMsgId           = cbMsgIdReqSetCMParamMaxClassified,
  .m_uCount           = (size_t)(1),
  .m_pFields          = cbReqSetCMParamMaxClassifiedFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqGetCMParamMaxClassified Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbReqGetCMParamMaxClassified Field Id Enumeration
 */
typedef enum
{
  cbReqGetCMParamMaxClassifiedFIdReserved = 0,  ///< reserved field id
  cbReqGetCMParamMaxClassifiedFIdNumOf  = 1     ///< number of fields
} cbReqGetCMParamMaxClassifiedFId_T;

/*!
 * cbReqGetCMParamMaxClassified Field Definitions
 */
static const NMFieldDef_T cbReqGetCMParamMaxClassifiedFieldDefs[] =
{
  EOFDEF
};

/*!
 * cbReqGetCMParamMaxClassified Message Definition
 */
static const NMMsgDef_T cbReqGetCMParamMaxClassifiedMsgDef =
{
  .m_sMsgName         = "cbReqGetCMParamMaxClassified",
  .m_eMsgId           = cbMsgIdReqGetCMParamMaxClassified,
  .m_uCount           = (size_t)(0),
  .m_pFields          = cbReqGetCMParamMaxClassifiedFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbRspGetCMParamMaxClassified Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbRspGetCMParamMaxClassified Field Id Enumeration
 */
typedef enum
{
  cbRspGetCMParamMaxClassifiedFIdReserved = 0,  ///< reserved field id
  cbRspGetCMParamMaxClassifiedFIdmax    = 1,    ///< max field id
  cbRspGetCMParamMaxClassifiedFIdNumOf  = 2     ///< number of fields
} cbRspGetCMParamMaxClassifiedFId_T;

/*!
 * cbRspGetCMParamMaxClassified Field Definitions
 */
static const NMFieldDef_T cbRspGetCMParamMaxClassifiedFieldDefs[] =
{
  {
    .m_sFName                 = "max",
    .m_eFId                   = cbRspGetCMParamMaxClassifiedFIdmax,
    .m_eFType                 = NMFTypeU8,
    .m_uOffset                = memberoffset(cbRspGetCMParamMaxClassified_T, m_max),
    .m_this.m_u8.m_bits       = (byte_t)(0),
    .m_this.m_u8.m_valMin     = (byte_t)(0),
    .m_this.m_u8.m_valMax     = (byte_t)(0),
    .m_this.m_u8.m_valConst   = (byte_t)(0),
  },
  EOFDEF
};

/*!
 * cbRspGetCMParamMaxClassified Message Definition
 */
static const NMMsgDef_T cbRspGetCMParamMaxClassifiedMsgDef =
{
  .m_sMsgName         = "cbRspGetCMParamMaxClassified",
  .m_eMsgId           = cbMsgIdRspGetCMParamMaxClassified,
  .m_uCount           = (size_t)(1),
  .m_pFields          = cbRspGetCMParamMaxClassifiedFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqSetNNParamLabel Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbReqSetNNParamLabel Field Id Enumeration
 */
typedef enum
{
  cbReqSetNNParamLabelFIdReserved       = 0,    ///< reserved field id
  cbReqSetNNParamLabelFIdlabel          = 1,    ///< label field id
  cbReqSetNNParamLabelFIdNumOf          = 2     ///< number of fields
} cbReqSetNNParamLabelFId_T;

/*!
 * cbReqSetNNParamLabel Field Definitions
 */
static const NMFieldDef_T cbReqSetNNParamLabelFieldDefs[] =
{
  {
    .m_sFName                 = "label",
    .m_eFId                   = cbReqSetNNParamLabelFIdlabel,
    .m_eFType                 = NMFTypeString,
    .m_uOffset                = memberoffset(cbReqSetNNParamLabel_T, m_label),
    .m_this.m_string.m_uMaxCount
                              = (size_t)CB_PARAM_NN_LABEL_LEN_MAX+1,
    .m_this.m_string.m_sConst = (char *)(NULL),
  },
  EOFDEF
};

/*!
 * cbReqSetNNParamLabel Message Definition
 */
static const NMMsgDef_T cbReqSetNNParamLabelMsgDef =
{
  .m_sMsgName         = "cbReqSetNNParamLabel",
  .m_eMsgId           = cbMsgIdReqSetNNParamLabel,
  .m_uCount           = (size_t)(1),
  .m_pFields          = cbReqSetNNParamLabelFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqGetNNParamLabel Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbReqGetNNParamLabel Field Id Enumeration
 */
typedef enum
{
  cbReqGetNNParamLabelFIdReserved       = 0,    ///< reserved field id
  cbReqGetNNParamLabelFIdNumOf          = 1     ///< number of fields
} cbReqGetNNParamLabelFId_T;

/*!
 * cbReqGetNNParamLabel Field Definitions
 */
static const NMFieldDef_T cbReqGetNNParamLabelFieldDefs[] =
{
  EOFDEF
};

/*!
 * cbReqGetNNParamLabel Message Definition
 */
static const NMMsgDef_T cbReqGetNNParamLabelMsgDef =
{
  .m_sMsgName         = "cbReqGetNNParamLabel",
  .m_eMsgId           = cbMsgIdReqGetNNParamLabel,
  .m_uCount           = (size_t)(0),
  .m_pFields          = cbReqGetNNParamLabelFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbRspGetNNParamLabel Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbRspGetNNParamLabel Field Id Enumeration
 */
typedef enum
{
  cbRspGetNNParamLabelFIdReserved       = 0,    ///< reserved field id
  cbRspGetNNParamLabelFIdlabel          = 1,    ///< label field id
  cbRspGetNNParamLabelFIdNumOf          = 2     ///< number of fields
} cbRspGetNNParamLabelFId_T;

/*!
 * cbRspGetNNParamLabel Field Definitions
 */
static const NMFieldDef_T cbRspGetNNParamLabelFieldDefs[] =
{
  {
    .m_sFName                 = "label",
    .m_eFId                   = cbRspGetNNParamLabelFIdlabel,
    .m_eFType                 = NMFTypeString,
    .m_uOffset                = memberoffset(cbRspGetNNParamLabel_T, m_label),
    .m_this.m_string.m_uMaxCount
                              = (size_t)CB_PARAM_NN_LABEL_LEN_MAX+1,
    .m_this.m_string.m_sConst = (char *)(NULL),
  },
  EOFDEF
};

/*!
 * cbRspGetNNParamLabel Message Definition
 */
static const NMMsgDef_T cbRspGetNNParamLabelMsgDef =
{
  .m_sMsgName         = "cbRspGetNNParamLabel",
  .m_eMsgId           = cbMsgIdRspGetNNParamLabel,
  .m_uCount           = (size_t)(1),
  .m_pFields          = cbRspGetNNParamLabelFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqNVMemReset Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbReqNVMemReset Field Id Enumeration
 */
typedef enum
{
  cbReqNVMemResetFIdReserved            = 0,    ///< reserved field id
  cbReqNVMemResetFIdNumOf               = 1     ///< number of fields
} cbReqNVMemResetFId_T;

/*!
 * cbReqNVMemReset Field Definitions
 */
static const NMFieldDef_T cbReqNVMemResetFieldDefs[] =
{
  EOFDEF
};

/*!
 * cbReqNVMemReset Message Definition
 */
static const NMMsgDef_T cbReqNVMemResetMsgDef =
{
  .m_sMsgName         = "cbReqNVMemReset",
  .m_eMsgId           = cbMsgIdReqNVMemReset,
  .m_uCount           = (size_t)(0),
  .m_pFields          = cbReqNVMemResetFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqNVMemSave Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbReqNVMemSave Field Id Enumeration
 */
typedef enum
{
  cbReqNVMemSaveFIdReserved             = 0,    ///< reserved field id
  cbReqNVMemSaveFIdNumOf                = 1     ///< number of fields
} cbReqNVMemSaveFId_T;

/*!
 * cbReqNVMemSave Field Definitions
 */
static const NMFieldDef_T cbReqNVMemSaveFieldDefs[] =
{
  EOFDEF
};

/*!
 * cbReqNVMemSave Message Definition
 */
static const NMMsgDef_T cbReqNVMemSaveMsgDef =
{
  .m_sMsgName         = "cbReqNVMemSave",
  .m_eMsgId           = cbMsgIdReqNVMemSave,
  .m_uCount           = (size_t)(0),
  .m_pFields          = cbReqNVMemSaveFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqNVMemRestore Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbReqNVMemRestore Field Id Enumeration
 */
typedef enum
{
  cbReqNVMemRestoreFIdReserved          = 0,    ///< reserved field id
  cbReqNVMemRestoreFIdNumOf             = 1     ///< number of fields
} cbReqNVMemRestoreFId_T;

/*!
 * cbReqNVMemRestore Field Definitions
 */
static const NMFieldDef_T cbReqNVMemRestoreFieldDefs[] =
{
  EOFDEF
};

/*!
 * cbReqNVMemRestore Message Definition
 */
static const NMMsgDef_T cbReqNVMemRestoreMsgDef =
{
  .m_sMsgName         = "cbReqNVMemRestore",
  .m_eMsgId           = cbMsgIdReqNVMemRestore,
  .m_uCount           = (size_t)(0),
  .m_pFields          = cbReqNVMemRestoreFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqNVMemGetNNCount Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbReqNVMemGetNNCount Field Id Enumeration
 */
typedef enum
{
  cbReqNVMemGetNNCountFIdReserved       = 0,    ///< reserved field id
  cbReqNVMemGetNNCountFIdNumOf          = 1     ///< number of fields
} cbReqNVMemGetNNCountFId_T;

/*!
 * cbReqNVMemGetNNCount Field Definitions
 */
static const NMFieldDef_T cbReqNVMemGetNNCountFieldDefs[] =
{
  EOFDEF
};

/*!
 * cbReqNVMemGetNNCount Message Definition
 */
static const NMMsgDef_T cbReqNVMemGetNNCountMsgDef =
{
  .m_sMsgName         = "cbReqNVMemGetNNCount",
  .m_eMsgId           = cbMsgIdReqNVMemGetNNCount,
  .m_uCount           = (size_t)(0),
  .m_pFields          = cbReqNVMemGetNNCountFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbRspNVMemGetNNCount Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbRspNVMemGetNNCount Field Id Enumeration
 */
typedef enum
{
  cbRspNVMemGetNNCountFIdReserved       = 0,    ///< reserved field id
  cbRspNVMemGetNNCountFIdcount          = 1,    ///< count field id
  cbRspNVMemGetNNCountFIdNumOf          = 2     ///< number of fields
} cbRspNVMemGetNNCountFId_T;

/*!
 * cbRspNVMemGetNNCount Field Definitions
 */
static const NMFieldDef_T cbRspNVMemGetNNCountFieldDefs[] =
{
  {
    .m_sFName                 = "count",
    .m_eFId                   = cbRspNVMemGetNNCountFIdcount,
    .m_eFType                 = NMFTypeU16,
    .m_uOffset                = memberoffset(cbRspNVMemGetNNCount_T, m_count),
    .m_this.m_u16.m_bits      = (byte_t)(0),
    .m_this.m_u16.m_valMin    = (ushort_t)(0),
    .m_this.m_u16.m_valMax    = (ushort_t)(0),
    .m_this.m_u16.m_valConst  = (ushort_t)(0),
  },
  EOFDEF
};

/*!
 * cbRspNVMemGetNNCount Message Definition
 */
static const NMMsgDef_T cbRspNVMemGetNNCountMsgDef =
{
  .m_sMsgName         = "cbRspNVMemGetNNCount",
  .m_eMsgId           = cbMsgIdRspNVMemGetNNCount,
  .m_uCount           = (size_t)(1),
  .m_pFields          = cbRspNVMemGetNNCountFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqNNTrain Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbReqNNTrainpattern Field Definitions
 */
static const NMFieldDef_T cbReqNNTrainpatternFieldDef[] =
{
  {
    .m_sFName                 = "pattern",
    .m_eFId                   = 0,
    .m_eFType                 = NMFTypeU8,
    .m_uOffset                = (size_t)0,
    .m_this.m_u8.m_bits       = (byte_t)(0),
    .m_this.m_u8.m_valMin     = (byte_t)(0),
    .m_this.m_u8.m_valMax     = (byte_t)(0),
    .m_this.m_u8.m_valConst   = (byte_t)(0),
  },
  EOFDEF
};

/*!
 * cbReqNNTrain Field Id Enumeration
 */
typedef enum
{
  cbReqNNTrainFIdReserved               = 0,    ///< reserved field id
  cbReqNNTrainFIdcategory               = 1,    ///< category field id
  cbReqNNTrainFIdpattern                = 2,    ///< pattern field id
  cbReqNNTrainFIdNumOf                  = 3     ///< number of fields
} cbReqNNTrainFId_T;

/*!
 * cbReqNNTrain Field Definitions
 */
static const NMFieldDef_T cbReqNNTrainFieldDefs[] =
{
  {
    .m_sFName                 = "category",
    .m_eFId                   = cbReqNNTrainFIdcategory,
    .m_eFType                 = NMFTypeU16,
    .m_uOffset                = memberoffset(cbReqNNTrain_T, m_category),
    .m_this.m_u16.m_bits      = (byte_t)(0),
    .m_this.m_u16.m_valMin    = (ushort_t)(0),
    .m_this.m_u16.m_valMax    = (ushort_t)(0),
    .m_this.m_u16.m_valConst  = (ushort_t)(0),
  },
  {
    .m_sFName                 = "pattern",
    .m_eFId                   = cbReqNNTrainFIdpattern,
    .m_eFType                 = NMFTypeVector,
    .m_uOffset                = memberoffset(cbReqNNTrain_T, m_pattern),
    .m_this.m_vector.m_uMaxCount
                              = (size_t)CM_PATTERN_COMP_NUMOF,
    .m_this.m_vector.m_uElemSize
                              = sizeof(byte_t),
    .m_this.m_vector.m_pThisElem
                              = cbReqNNTrainpatternFieldDef,
  },
  EOFDEF
};

/*!
 * cbReqNNTrain Message Definition
 */
static const NMMsgDef_T cbReqNNTrainMsgDef =
{
  .m_sMsgName         = "cbReqNNTrain",
  .m_eMsgId           = cbMsgIdReqNNTrain,
  .m_uCount           = (size_t)(2),
  .m_pFields          = cbReqNNTrainFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbRspNNTrain Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbRspNNTrain Field Id Enumeration
 */
typedef enum
{
  cbRspNNTrainFIdReserved               = 0,    ///< reserved field id
  cbRspNNTrainFIdneuronId               = 1,    ///< neuronId field id
  cbRspNNTrainFIdNumOf                  = 2     ///< number of fields
} cbRspNNTrainFId_T;

/*!
 * cbRspNNTrain Field Definitions
 */
static const NMFieldDef_T cbRspNNTrainFieldDefs[] =
{
  {
    .m_sFName                 = "neuronId",
    .m_eFId                   = cbRspNNTrainFIdneuronId,
    .m_eFType                 = NMFTypeU16,
    .m_uOffset                = memberoffset(cbRspNNTrain_T, m_neuronId),
    .m_this.m_u16.m_bits      = (byte_t)(0),
    .m_this.m_u16.m_valMin    = (ushort_t)(0),
    .m_this.m_u16.m_valMax    = (ushort_t)(0),
    .m_this.m_u16.m_valConst  = (ushort_t)(0),
  },
  EOFDEF
};

/*!
 * cbRspNNTrain Message Definition
 */
static const NMMsgDef_T cbRspNNTrainMsgDef =
{
  .m_sMsgName         = "cbRspNNTrain",
  .m_eMsgId           = cbMsgIdRspNNTrain,
  .m_uCount           = (size_t)(1),
  .m_pFields          = cbRspNNTrainFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqNNCatergorize Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbReqNNCatergorizepattern Field Definitions
 */
static const NMFieldDef_T cbReqNNCatergorizepatternFieldDef[] =
{
  {
    .m_sFName                 = "pattern",
    .m_eFId                   = 0,
    .m_eFType                 = NMFTypeU8,
    .m_uOffset                = (size_t)0,
    .m_this.m_u8.m_bits       = (byte_t)(0),
    .m_this.m_u8.m_valMin     = (byte_t)(0),
    .m_this.m_u8.m_valMax     = (byte_t)(0),
    .m_this.m_u8.m_valConst   = (byte_t)(0),
  },
  EOFDEF
};

/*!
 * cbReqNNCatergorize Field Id Enumeration
 */
typedef enum
{
  cbReqNNCatergorizeFIdReserved         = 0,    ///< reserved field id
  cbReqNNCatergorizeFIdpattern          = 1,    ///< pattern field id
  cbReqNNCatergorizeFIdNumOf            = 2     ///< number of fields
} cbReqNNCatergorizeFId_T;

/*!
 * cbReqNNCatergorize Field Definitions
 */
static const NMFieldDef_T cbReqNNCatergorizeFieldDefs[] =
{
  {
    .m_sFName                 = "pattern",
    .m_eFId                   = cbReqNNCatergorizeFIdpattern,
    .m_eFType                 = NMFTypeVector,
    .m_uOffset                = memberoffset(cbReqNNCatergorize_T, m_pattern),
    .m_this.m_vector.m_uMaxCount
                              = (size_t)CM_PATTERN_COMP_NUMOF,
    .m_this.m_vector.m_uElemSize
                              = sizeof(byte_t),
    .m_this.m_vector.m_pThisElem
                              = cbReqNNCatergorizepatternFieldDef,
  },
  EOFDEF
};

/*!
 * cbReqNNCatergorize Message Definition
 */
static const NMMsgDef_T cbReqNNCatergorizeMsgDef =
{
  .m_sMsgName         = "cbReqNNCatergorize",
  .m_eMsgId           = cbMsgIdReqNNCatergorize,
  .m_uCount           = (size_t)(1),
  .m_pFields          = cbReqNNCatergorizeFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbRspNNCatergorize Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbRspNNCatergorizeclassified Field Definitions
 */
static const NMFieldDef_T cbRspNNCatergorizeclassifiedFieldDef[] =
{
  {
    .m_sFName                 = "classified",
    .m_eFId                   = 0,
    .m_eFType                 = NMFTypeStruct,
    .m_uOffset                = (size_t)0,
    .m_this.m_struct          = &cbClassificationMsgDef,
  },
  EOFDEF
};

/*!
 * cbRspNNCatergorize Field Id Enumeration
 */
typedef enum
{
  cbRspNNCatergorizeFIdReserved         = 0,    ///< reserved field id
  cbRspNNCatergorizeFIdclassified       = 1,    ///< classified field id
  cbRspNNCatergorizeFIdNumOf            = 2     ///< number of fields
} cbRspNNCatergorizeFId_T;

/*!
 * cbRspNNCatergorize Field Definitions
 */
static const NMFieldDef_T cbRspNNCatergorizeFieldDefs[] =
{
  {
    .m_sFName                 = "classified",
    .m_eFId                   = cbRspNNCatergorizeFIdclassified,
    .m_eFType                 = NMFTypeVector,
    .m_uOffset                = memberoffset(cbRspNNCatergorize_T, m_classified),
    .m_this.m_vector.m_uMaxCount
                              = (size_t)CB_PARAM_MAX_CLASSIFIED_MAX,
    .m_this.m_vector.m_uElemSize
                              = sizeof(cbClassification_T),
    .m_this.m_vector.m_pThisElem
                              = cbRspNNCatergorizeclassifiedFieldDef,
  },
  EOFDEF
};

/*!
 * cbRspNNCatergorize Message Definition
 */
static const NMMsgDef_T cbRspNNCatergorizeMsgDef =
{
  .m_sMsgName         = "cbRspNNCatergorize",
  .m_eMsgId           = cbMsgIdRspNNCatergorize,
  .m_uCount           = (size_t)(1),
  .m_pFields          = cbRspNNCatergorizeFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqNNForget Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbReqNNForget Field Id Enumeration
 */
typedef enum
{
  cbReqNNForgetFIdReserved              = 0,    ///< reserved field id
  cbReqNNForgetFIdNumOf                 = 1     ///< number of fields
} cbReqNNForgetFId_T;

/*!
 * cbReqNNForget Field Definitions
 */
static const NMFieldDef_T cbReqNNForgetFieldDefs[] =
{
  EOFDEF
};

/*!
 * cbReqNNForget Message Definition
 */
static const NMMsgDef_T cbReqNNForgetMsgDef =
{
  .m_sMsgName         = "cbReqNNForget",
  .m_eMsgId           = cbMsgIdReqNNForget,
  .m_uCount           = (size_t)(0),
  .m_pFields          = cbReqNNForgetFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqNNGetCount Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbReqNNGetCount Field Id Enumeration
 */
typedef enum
{
  cbReqNNGetCountFIdReserved            = 0,    ///< reserved field id
  cbReqNNGetCountFIdNumOf               = 1     ///< number of fields
} cbReqNNGetCountFId_T;

/*!
 * cbReqNNGetCount Field Definitions
 */
static const NMFieldDef_T cbReqNNGetCountFieldDefs[] =
{
  EOFDEF
};

/*!
 * cbReqNNGetCount Message Definition
 */
static const NMMsgDef_T cbReqNNGetCountMsgDef =
{
  .m_sMsgName         = "cbReqNNGetCount",
  .m_eMsgId           = cbMsgIdReqNNGetCount,
  .m_uCount           = (size_t)(0),
  .m_pFields          = cbReqNNGetCountFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbRspNNGetCount Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbRspNNGetCount Field Id Enumeration
 */
typedef enum
{
  cbRspNNGetCountFIdReserved            = 0,    ///< reserved field id
  cbRspNNGetCountFIdcount               = 1,    ///< count field id
  cbRspNNGetCountFIdNumOf               = 2     ///< number of fields
} cbRspNNGetCountFId_T;

/*!
 * cbRspNNGetCount Field Definitions
 */
static const NMFieldDef_T cbRspNNGetCountFieldDefs[] =
{
  {
    .m_sFName                 = "count",
    .m_eFId                   = cbRspNNGetCountFIdcount,
    .m_eFType                 = NMFTypeU16,
    .m_uOffset                = memberoffset(cbRspNNGetCount_T, m_count),
    .m_this.m_u16.m_bits      = (byte_t)(0),
    .m_this.m_u16.m_valMin    = (ushort_t)(0),
    .m_this.m_u16.m_valMax    = (ushort_t)(0),
    .m_this.m_u16.m_valConst  = (ushort_t)(0),
  },
  EOFDEF
};

/*!
 * cbRspNNGetCount Message Definition
 */
static const NMMsgDef_T cbRspNNGetCountMsgDef =
{
  .m_sMsgName         = "cbRspNNGetCount",
  .m_eMsgId           = cbMsgIdRspNNGetCount,
  .m_uCount           = (size_t)(1),
  .m_pFields          = cbRspNNGetCountFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqCMReadReg Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbReqCMReadReg Field Id Enumeration
 */
typedef enum
{
  cbReqCMReadRegFIdReserved             = 0,    ///< reserved field id
  cbReqCMReadRegFIdaddr                 = 1,    ///< addr field id
  cbReqCMReadRegFIdNumOf                = 2     ///< number of fields
} cbReqCMReadRegFId_T;

/*!
 * cbReqCMReadReg Field Definitions
 */
static const NMFieldDef_T cbReqCMReadRegFieldDefs[] =
{
  {
    .m_sFName                 = "addr",
    .m_eFId                   = cbReqCMReadRegFIdaddr,
    .m_eFType                 = NMFTypeU8,
    .m_uOffset                = memberoffset(cbReqCMReadReg_T, m_addr),
    .m_this.m_u8.m_bits       = (byte_t)(0),
    .m_this.m_u8.m_valMin     = (byte_t)(0),
    .m_this.m_u8.m_valMax     = (byte_t)(0),
    .m_this.m_u8.m_valConst   = (byte_t)(0),
  },
  EOFDEF
};

/*!
 * cbReqCMReadReg Message Definition
 */
static const NMMsgDef_T cbReqCMReadRegMsgDef =
{
  .m_sMsgName         = "cbReqCMReadReg",
  .m_eMsgId           = cbMsgIdReqCMReadReg,
  .m_uCount           = (size_t)(1),
  .m_pFields          = cbReqCMReadRegFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbRspCMReadReg Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbRspCMReadReg Field Id Enumeration
 */
typedef enum
{
  cbRspCMReadRegFIdReserved             = 0,    ///< reserved field id
  cbRspCMReadRegFIdaddr                 = 1,    ///< addr field id
  cbRspCMReadRegFIdvalue                = 2,    ///< value field id
  cbRspCMReadRegFIdNumOf                = 3     ///< number of fields
} cbRspCMReadRegFId_T;

/*!
 * cbRspCMReadReg Field Definitions
 */
static const NMFieldDef_T cbRspCMReadRegFieldDefs[] =
{
  {
    .m_sFName                 = "addr",
    .m_eFId                   = cbRspCMReadRegFIdaddr,
    .m_eFType                 = NMFTypeU8,
    .m_uOffset                = memberoffset(cbRspCMReadReg_T, m_addr),
    .m_this.m_u8.m_bits       = (byte_t)(0),
    .m_this.m_u8.m_valMin     = (byte_t)(0),
    .m_this.m_u8.m_valMax     = (byte_t)(0),
    .m_this.m_u8.m_valConst   = (byte_t)(0),
  },
  {
    .m_sFName                 = "value",
    .m_eFId                   = cbRspCMReadRegFIdvalue,
    .m_eFType                 = NMFTypeU16,
    .m_uOffset                = memberoffset(cbRspCMReadReg_T, m_value),
    .m_this.m_u16.m_bits      = (byte_t)(0),
    .m_this.m_u16.m_valMin    = (ushort_t)(0),
    .m_this.m_u16.m_valMax    = (ushort_t)(0),
    .m_this.m_u16.m_valConst  = (ushort_t)(0),
  },
  EOFDEF
};

/*!
 * cbRspCMReadReg Message Definition
 */
static const NMMsgDef_T cbRspCMReadRegMsgDef =
{
  .m_sMsgName         = "cbRspCMReadReg",
  .m_eMsgId           = cbMsgIdRspCMReadReg,
  .m_uCount           = (size_t)(2),
  .m_pFields          = cbRspCMReadRegFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqCMWriteReg Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbReqCMWriteReg Field Id Enumeration
 */
typedef enum
{
  cbReqCMWriteRegFIdReserved            = 0,    ///< reserved field id
  cbReqCMWriteRegFIdaddr                = 1,    ///< addr field id
  cbReqCMWriteRegFIdvalue               = 2,    ///< value field id
  cbReqCMWriteRegFIdNumOf               = 3     ///< number of fields
} cbReqCMWriteRegFId_T;

/*!
 * cbReqCMWriteReg Field Definitions
 */
static const NMFieldDef_T cbReqCMWriteRegFieldDefs[] =
{
  {
    .m_sFName                 = "addr",
    .m_eFId                   = cbReqCMWriteRegFIdaddr,
    .m_eFType                 = NMFTypeU8,
    .m_uOffset                = memberoffset(cbReqCMWriteReg_T, m_addr),
    .m_this.m_u8.m_bits       = (byte_t)(0),
    .m_this.m_u8.m_valMin     = (byte_t)(0),
    .m_this.m_u8.m_valMax     = (byte_t)(0),
    .m_this.m_u8.m_valConst   = (byte_t)(0),
  },
  {
    .m_sFName                 = "value",
    .m_eFId                   = cbReqCMWriteRegFIdvalue,
    .m_eFType                 = NMFTypeU16,
    .m_uOffset                = memberoffset(cbReqCMWriteReg_T, m_value),
    .m_this.m_u16.m_bits      = (byte_t)(0),
    .m_this.m_u16.m_valMin    = (ushort_t)(0),
    .m_this.m_u16.m_valMax    = (ushort_t)(0),
    .m_this.m_u16.m_valConst  = (ushort_t)(0),
  },
  EOFDEF
};

/*!
 * cbReqCMWriteReg Message Definition
 */
static const NMMsgDef_T cbReqCMWriteRegMsgDef =
{
  .m_sMsgName         = "cbReqCMWriteReg",
  .m_eMsgId           = cbMsgIdReqCMWriteReg,
  .m_uCount           = (size_t)(2),
  .m_pFields          = cbReqCMWriteRegFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqUploadParams Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbReqUploadParams Field Id Enumeration
 */
typedef enum
{
  cbReqUploadParamsFIdReserved          = 0,    ///< reserved field id
  cbReqUploadParamsFIddst               = 1,    ///< dst field id
  cbReqUploadParamsFIdparams            = 2,    ///< params field id
  cbReqUploadParamsFIdNumOf             = 3     ///< number of fields
} cbReqUploadParamsFId_T;

/*!
 * cbReqUploadParams Field Definitions
 */
static const NMFieldDef_T cbReqUploadParamsFieldDefs[] =
{
  {
    .m_sFName                 = "dst",
    .m_eFId                   = cbReqUploadParamsFIddst,
    .m_eFType                 = NMFTypeU8,
    .m_uOffset                = memberoffset(cbReqUploadParams_T, m_dst),
    .m_this.m_u8.m_bits       = (byte_t)(0),
    .m_this.m_u8.m_valMin     = (byte_t)(0),
    .m_this.m_u8.m_valMax     = (byte_t)(0),
    .m_this.m_u8.m_valConst   = (byte_t)(0),
  },
  {
    .m_sFName                 = "params",
    .m_eFId                   = cbReqUploadParamsFIdparams,
    .m_eFType                 = NMFTypeStruct,
    .m_uOffset                = memberoffset(cbReqUploadParams_T, m_params),
    .m_this.m_struct          = &cbCogniBoostParamsMsgDef,
  },
  EOFDEF
};

/*!
 * cbReqUploadParams Message Definition
 */
static const NMMsgDef_T cbReqUploadParamsMsgDef =
{
  .m_sMsgName         = "cbReqUploadParams",
  .m_eMsgId           = cbMsgIdReqUploadParams,
  .m_uCount           = (size_t)(2),
  .m_pFields          = cbReqUploadParamsFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqDownloadParams Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbReqDownloadParams Field Id Enumeration
 */
typedef enum
{
  cbReqDownloadParamsFIdReserved        = 0,    ///< reserved field id
  cbReqDownloadParamsFIdsrc             = 1,    ///< src field id
  cbReqDownloadParamsFIdNumOf           = 2     ///< number of fields
} cbReqDownloadParamsFId_T;

/*!
 * cbReqDownloadParams Field Definitions
 */
static const NMFieldDef_T cbReqDownloadParamsFieldDefs[] =
{
  {
    .m_sFName                 = "src",
    .m_eFId                   = cbReqDownloadParamsFIdsrc,
    .m_eFType                 = NMFTypeU8,
    .m_uOffset                = memberoffset(cbReqDownloadParams_T, m_src),
    .m_this.m_u8.m_bits       = (byte_t)(0),
    .m_this.m_u8.m_valMin     = (byte_t)(0),
    .m_this.m_u8.m_valMax     = (byte_t)(0),
    .m_this.m_u8.m_valConst   = (byte_t)(0),
  },
  EOFDEF
};

/*!
 * cbReqDownloadParams Message Definition
 */
static const NMMsgDef_T cbReqDownloadParamsMsgDef =
{
  .m_sMsgName         = "cbReqDownloadParams",
  .m_eMsgId           = cbMsgIdReqDownloadParams,
  .m_uCount           = (size_t)(1),
  .m_pFields          = cbReqDownloadParamsFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbRspDownloadParams Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbRspDownloadParams Field Id Enumeration
 */
typedef enum
{
  cbRspDownloadParamsFIdReserved        = 0,    ///< reserved field id
  cbRspDownloadParamsFIdparams          = 1,    ///< params field id
  cbRspDownloadParamsFIdNumOf           = 2     ///< number of fields
} cbRspDownloadParamsFId_T;

/*!
 * cbRspDownloadParams Field Definitions
 */
static const NMFieldDef_T cbRspDownloadParamsFieldDefs[] =
{
  {
    .m_sFName                 = "params",
    .m_eFId                   = cbRspDownloadParamsFIdparams,
    .m_eFType                 = NMFTypeStruct,
    .m_uOffset                = memberoffset(cbRspDownloadParams_T, m_params),
    .m_this.m_struct          = &cbCogniBoostParamsMsgDef,
  },
  EOFDEF
};

/*!
 * cbRspDownloadParams Message Definition
 */
static const NMMsgDef_T cbRspDownloadParamsMsgDef =
{
  .m_sMsgName         = "cbRspDownloadParams",
  .m_eMsgId           = cbMsgIdRspDownloadParams,
  .m_uCount           = (size_t)(1),
  .m_pFields          = cbRspDownloadParamsFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqUploadNNSetStart Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbReqUploadNNSetStart Field Id Enumeration
 */
typedef enum
{
  cbReqUploadNNSetStartFIdReserved      = 0,    ///< reserved field id
  cbReqUploadNNSetStartFIddst           = 1,    ///< dst field id
  cbReqUploadNNSetStartFIdnumNeurons    = 2,    ///< numNeurons field id
  cbReqUploadNNSetStartFIdneuron        = 3,    ///< neuron field id
  cbReqUploadNNSetStartFIdNumOf         = 4     ///< number of fields
} cbReqUploadNNSetStartFId_T;

/*!
 * cbReqUploadNNSetStart Field Definitions
 */
static const NMFieldDef_T cbReqUploadNNSetStartFieldDefs[] =
{
  {
    .m_sFName                 = "dst",
    .m_eFId                   = cbReqUploadNNSetStartFIddst,
    .m_eFType                 = NMFTypeU8,
    .m_uOffset                = memberoffset(cbReqUploadNNSetStart_T, m_dst),
    .m_this.m_u8.m_bits       = (byte_t)(0),
    .m_this.m_u8.m_valMin     = (byte_t)(0),
    .m_this.m_u8.m_valMax     = (byte_t)(0),
    .m_this.m_u8.m_valConst   = (byte_t)(0),
  },
  {
    .m_sFName                 = "numNeurons",
    .m_eFId                   = cbReqUploadNNSetStartFIdnumNeurons,
    .m_eFType                 = NMFTypeU16,
    .m_uOffset                = memberoffset(cbReqUploadNNSetStart_T, m_numNeurons),
    .m_this.m_u16.m_bits      = (byte_t)(0),
    .m_this.m_u16.m_valMin    = (ushort_t)(0),
    .m_this.m_u16.m_valMax    = (ushort_t)(0),
    .m_this.m_u16.m_valConst  = (ushort_t)(0),
  },
  {
    .m_sFName                 = "neuron",
    .m_eFId                   = cbReqUploadNNSetStartFIdneuron,
    .m_eFType                 = NMFTypeStruct,
    .m_uOffset                = memberoffset(cbReqUploadNNSetStart_T, m_neuron),
    .m_this.m_struct          = &cbNeuronMsgDef,
  },
  EOFDEF
};

/*!
 * cbReqUploadNNSetStart Message Definition
 */
static const NMMsgDef_T cbReqUploadNNSetStartMsgDef =
{
  .m_sMsgName         = "cbReqUploadNNSetStart",
  .m_eMsgId           = cbMsgIdReqUploadNNSetStart,
  .m_uCount           = (size_t)(3),
  .m_pFields          = cbReqUploadNNSetStartFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqUploadNNSetNext Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbReqUploadNNSetNext Field Id Enumeration
 */
typedef enum
{
  cbReqUploadNNSetNextFIdReserved       = 0,    ///< reserved field id
  cbReqUploadNNSetNextFIdneuron         = 1,    ///< neuron field id
  cbReqUploadNNSetNextFIdNumOf          = 2     ///< number of fields
} cbReqUploadNNSetNextFId_T;

/*!
 * cbReqUploadNNSetNext Field Definitions
 */
static const NMFieldDef_T cbReqUploadNNSetNextFieldDefs[] =
{
  {
    .m_sFName                 = "neuron",
    .m_eFId                   = cbReqUploadNNSetNextFIdneuron,
    .m_eFType                 = NMFTypeStruct,
    .m_uOffset                = memberoffset(cbReqUploadNNSetNext_T, m_neuron),
    .m_this.m_struct          = &cbNeuronMsgDef,
  },
  EOFDEF
};

/*!
 * cbReqUploadNNSetNext Message Definition
 */
static const NMMsgDef_T cbReqUploadNNSetNextMsgDef =
{
  .m_sMsgName         = "cbReqUploadNNSetNext",
  .m_eMsgId           = cbMsgIdReqUploadNNSetNext,
  .m_uCount           = (size_t)(1),
  .m_pFields          = cbReqUploadNNSetNextFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqDownloadNNSetStart Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbReqDownloadNNSetStart Field Id Enumeration
 */
typedef enum
{
  cbReqDownloadNNSetStartFIdReserved    = 0,    ///< reserved field id
  cbReqDownloadNNSetStartFIdsrc         = 1,    ///< src field id
  cbReqDownloadNNSetStartFIdNumOf       = 2     ///< number of fields
} cbReqDownloadNNSetStartFId_T;

/*!
 * cbReqDownloadNNSetStart Field Definitions
 */
static const NMFieldDef_T cbReqDownloadNNSetStartFieldDefs[] =
{
  {
    .m_sFName                 = "src",
    .m_eFId                   = cbReqDownloadNNSetStartFIdsrc,
    .m_eFType                 = NMFTypeU8,
    .m_uOffset                = memberoffset(cbReqDownloadNNSetStart_T, m_src),
    .m_this.m_u8.m_bits       = (byte_t)(0),
    .m_this.m_u8.m_valMin     = (byte_t)(0),
    .m_this.m_u8.m_valMax     = (byte_t)(0),
    .m_this.m_u8.m_valConst   = (byte_t)(0),
  },
  EOFDEF
};

/*!
 * cbReqDownloadNNSetStart Message Definition
 */
static const NMMsgDef_T cbReqDownloadNNSetStartMsgDef =
{
  .m_sMsgName         = "cbReqDownloadNNSetStart",
  .m_eMsgId           = cbMsgIdReqDownloadNNSetStart,
  .m_uCount           = (size_t)(1),
  .m_pFields          = cbReqDownloadNNSetStartFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbRspDownloadNNSetStart Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbRspDownloadNNSetStart Field Id Enumeration
 */
typedef enum
{
  cbRspDownloadNNSetStartFIdReserved    = 0,    ///< reserved field id
  cbRspDownloadNNSetStartFIdnumNeurons  = 1,    ///< numNeurons field id
  cbRspDownloadNNSetStartFIdneuron      = 2,    ///< neuron field id
  cbRspDownloadNNSetStartFIdNumOf       = 3     ///< number of fields
} cbRspDownloadNNSetStartFId_T;

/*!
 * cbRspDownloadNNSetStart Field Definitions
 */
static const NMFieldDef_T cbRspDownloadNNSetStartFieldDefs[] =
{
  {
    .m_sFName                 = "numNeurons",
    .m_eFId                   = cbRspDownloadNNSetStartFIdnumNeurons,
    .m_eFType                 = NMFTypeU16,
    .m_uOffset                = memberoffset(cbRspDownloadNNSetStart_T, m_numNeurons),
    .m_this.m_u16.m_bits      = (byte_t)(0),
    .m_this.m_u16.m_valMin    = (ushort_t)(0),
    .m_this.m_u16.m_valMax    = (ushort_t)(0),
    .m_this.m_u16.m_valConst  = (ushort_t)(0),
  },
  {
    .m_sFName                 = "neuron",
    .m_eFId                   = cbRspDownloadNNSetStartFIdneuron,
    .m_eFType                 = NMFTypeStruct,
    .m_uOffset                = memberoffset(cbRspDownloadNNSetStart_T, m_neuron),
    .m_this.m_struct          = &cbNeuronMsgDef,
  },
  EOFDEF
};

/*!
 * cbRspDownloadNNSetStart Message Definition
 */
static const NMMsgDef_T cbRspDownloadNNSetStartMsgDef =
{
  .m_sMsgName         = "cbRspDownloadNNSetStart",
  .m_eMsgId           = cbMsgIdRspDownloadNNSetStart,
  .m_uCount           = (size_t)(2),
  .m_pFields          = cbRspDownloadNNSetStartFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqDownloadNNSetNext Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbReqDownloadNNSetNext Field Id Enumeration
 */
typedef enum
{
  cbReqDownloadNNSetNextFIdReserved     = 0,    ///< reserved field id
  cbReqDownloadNNSetNextFIdNumOf        = 1     ///< number of fields
} cbReqDownloadNNSetNextFId_T;

/*!
 * cbReqDownloadNNSetNext Field Definitions
 */
static const NMFieldDef_T cbReqDownloadNNSetNextFieldDefs[] =
{
  EOFDEF
};

/*!
 * cbReqDownloadNNSetNext Message Definition
 */
static const NMMsgDef_T cbReqDownloadNNSetNextMsgDef =
{
  .m_sMsgName         = "cbReqDownloadNNSetNext",
  .m_eMsgId           = cbMsgIdReqDownloadNNSetNext,
  .m_uCount           = (size_t)(0),
  .m_pFields          = cbReqDownloadNNSetNextFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbRspDownloadNNSetNext Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbRspDownloadNNSetNext Field Id Enumeration
 */
typedef enum
{
  cbRspDownloadNNSetNextFIdReserved     = 0,    ///< reserved field id
  cbRspDownloadNNSetNextFIdneuron       = 1,    ///< neuron field id
  cbRspDownloadNNSetNextFIdNumOf        = 2     ///< number of fields
} cbRspDownloadNNSetNextFId_T;

/*!
 * cbRspDownloadNNSetNext Field Definitions
 */
static const NMFieldDef_T cbRspDownloadNNSetNextFieldDefs[] =
{
  {
    .m_sFName                 = "neuron",
    .m_eFId                   = cbRspDownloadNNSetNextFIdneuron,
    .m_eFType                 = NMFTypeStruct,
    .m_uOffset                = memberoffset(cbRspDownloadNNSetNext_T, m_neuron),
    .m_this.m_struct          = &cbNeuronMsgDef,
  },
  EOFDEF
};

/*!
 * cbRspDownloadNNSetNext Message Definition
 */
static const NMMsgDef_T cbRspDownloadNNSetNextMsgDef =
{
  .m_sMsgName         = "cbRspDownloadNNSetNext",
  .m_eMsgId           = cbMsgIdRspDownloadNNSetNext,
  .m_uCount           = (size_t)(1),
  .m_pFields          = cbRspDownloadNNSetNextFieldDefs
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqXferNNSetStop Definition
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * cbReqXferNNSetStop Field Id Enumeration
 */
typedef enum
{
  cbReqXferNNSetStopFIdReserved         = 0,    ///< reserved field id
  cbReqXferNNSetStopFIdabort            = 1,    ///< abort field id
  cbReqXferNNSetStopFIdNumOf            = 2     ///< number of fields
} cbReqXferNNSetStopFId_T;

/*!
 * cbReqXferNNSetStop Field Definitions
 */
static const NMFieldDef_T cbReqXferNNSetStopFieldDefs[] =
{
  {
    .m_sFName                 = "abort",
    .m_eFId                   = cbReqXferNNSetStopFIdabort,
    .m_eFType                 = NMFTypeBool,
    .m_uOffset                = memberoffset(cbReqXferNNSetStop_T, m_abort),
  },
  EOFDEF
};

/*!
 * cbReqXferNNSetStop Message Definition
 */
static const NMMsgDef_T cbReqXferNNSetStopMsgDef =
{
  .m_sMsgName         = "cbReqXferNNSetStop",
  .m_eMsgId           = cbMsgIdReqXferNNSetStop,
  .m_uCount           = (size_t)(1),
  .m_pFields          = cbReqXferNNSetStopFieldDefs
};


//-----------------------------------------------------------------------------
// Public Interface
//-----------------------------------------------------------------------------

/*!
 * cb Message Definition Look-Up Table. Indexed by cbMsgId_T enum.
 */
const NMMsgDef_T *cbMsgDefLookupTbl[] =
{
  NULL,                                 ///< [cbMsgIdNone]
  &cbRspOkMsgDef,                       ///< [cbMsgIdRspOk]
  &cbRspErrMsgDef,                      ///< [cbMsgIdRspErr]
  &cbReqVersionMsgDef,                  ///< [cbMsgIdReqVersion]
  &cbRspVersionMsgDef,                  ///< [cbMsgIdRspVersion]
  &cbReqPingMsgDef,                     ///< [cbMsgIdReqPing]
  &cbReqLoopbackMsgDef,                 ///< [cbMsgIdReqLoopback]
  &cbRspLoopbackMsgDef,                 ///< [cbMsgIdRspLoopback]
  &cbReqRebootMsgDef,                   ///< [cbMsgIdReqReboot]
  &cbReqSleepMsgDef,                    ///< [cbMsgIdReqSleep]
  &cbReqWakeUpMsgDef,                   ///< [cbMsgIdReqWakeUp]
  &cbReqSetOpParamBaudRateMsgDef,       ///< [cbMsgIdReqSetOpParamBaudRate]
  &cbReqGetOpParamBaudRateMsgDef,       ///< [cbMsgIdReqGetOpParamBaudRate]
  &cbRspGetOpParamBaudRateMsgDef,       ///< [cbMsgIdRspGetOpParamBaudRate]
  &cbReqSetOpParamAutoRestoreMsgDef,    ///< [cbMsgIdReqSetOpParamAutoRestore]
  &cbReqGetOpParamAutoRestoreMsgDef,    ///< [cbMsgIdReqGetOpParamAutoRestore]
  &cbRspGetOpParamAutoRestoreMsgDef,    ///< [cbMsgIdRspGetOpParamAutoRestore]
  &cbReqSetOpParamAutoSleepMsgDef,      ///< [cbMsgIdReqSetOpParamAutoSleep]
  &cbReqGetOpParamAutoSleepMsgDef,      ///< [cbMsgIdReqGetOpParamAutoSleep]
  &cbRspGetOpParamAutoSleepMsgDef,      ///< [cbMsgIdRspGetOpParamAutoSleep]
  &cbReqSetOpParamLedBlingMsgDef,       ///< [cbMsgIdReqSetOpParamLedBling]
  &cbReqGetOpParamLedBlingMsgDef,       ///< [cbMsgIdReqGetOpParamLedBling]
  &cbRspGetOpParamLedBlingMsgDef,       ///< [cbMsgIdRspGetOpParamLedBling]
  &cbReqSetCMParamClassifierMsgDef,     ///< [cbMsgIdReqSetCMParamClassifier]
  &cbReqGetCMParamClassifierMsgDef,     ///< [cbMsgIdReqGetCMParamClassifier]
  &cbRspGetCMParamClassifierMsgDef,     ///< [cbMsgIdRspGetCMParamClassifier]
  &cbReqSetCMParamContextMsgDef,        ///< [cbMsgIdReqSetCMParamContext]
  &cbReqGetCMParamContextMsgDef,        ///< [cbMsgIdReqGetCMParamContext]
  &cbRspGetCMParamContextMsgDef,        ///< [cbMsgIdRspGetCMParamContext]
  &cbReqSetCMParamNormMsgDef,           ///< [cbMsgIdReqSetCMParamNorm]
  &cbReqGetCMParamNormMsgDef,           ///< [cbMsgIdReqGetCMParamNorm]
  &cbRspGetCMParamNormMsgDef,           ///< [cbMsgIdRspGetCMParamNorm]
  &cbReqSetCMParamMinIFMsgDef,          ///< [cbMsgIdReqSetCMParamMinIF]
  &cbReqGetCMParamMinIFMsgDef,          ///< [cbMsgIdReqGetCMParamMinIF]
  &cbRspGetCMParamMinIFMsgDef,          ///< [cbMsgIdRspGetCMParamMinIF]
  &cbReqSetCMParamMaxIFMsgDef,          ///< [cbMsgIdReqSetCMParamMaxIF]
  &cbReqGetCMParamMaxIFMsgDef,          ///< [cbMsgIdReqGetCMParamMaxIF]
  &cbRspGetCMParamMaxIFMsgDef,          ///< [cbMsgIdRspGetCMParamMaxIF]
  &cbReqSetCMParamMaxClassifiedMsgDef,  ///< [cbMsgIdReqSetCMParamMaxClassified]
  &cbReqGetCMParamMaxClassifiedMsgDef,  ///< [cbMsgIdReqGetCMParamMaxClassified]
  &cbRspGetCMParamMaxClassifiedMsgDef,  ///< [cbMsgIdRspGetCMParamMaxClassified]
  &cbReqSetNNParamLabelMsgDef,          ///< [cbMsgIdReqSetNNParamLabel]
  &cbReqGetNNParamLabelMsgDef,          ///< [cbMsgIdReqGetNNParamLabel]
  &cbRspGetNNParamLabelMsgDef,          ///< [cbMsgIdRspGetNNParamLabel]
  &cbReqNVMemResetMsgDef,               ///< [cbMsgIdReqNVMemReset]
  &cbReqNVMemSaveMsgDef,                ///< [cbMsgIdReqNVMemSave]
  &cbReqNVMemRestoreMsgDef,             ///< [cbMsgIdReqNVMemRestore]
  &cbReqNVMemGetNNCountMsgDef,          ///< [cbMsgIdReqNVMemGetNNCount]
  &cbRspNVMemGetNNCountMsgDef,          ///< [cbMsgIdRspNVMemGetNNCount]
  &cbReqNNTrainMsgDef,                  ///< [cbMsgIdReqNNTrain]
  &cbRspNNTrainMsgDef,                  ///< [cbMsgIdRspNNTrain]
  &cbReqNNCatergorizeMsgDef,            ///< [cbMsgIdReqNNCatergorize]
  &cbRspNNCatergorizeMsgDef,            ///< [cbMsgIdRspNNCatergorize]
  &cbReqNNForgetMsgDef,                 ///< [cbMsgIdReqNNForget]
  &cbReqNNGetCountMsgDef,               ///< [cbMsgIdReqNNGetCount]
  &cbRspNNGetCountMsgDef,               ///< [cbMsgIdRspNNGetCount]
  &cbReqCMReadRegMsgDef,                ///< [cbMsgIdReqCMReadReg]
  &cbRspCMReadRegMsgDef,                ///< [cbMsgIdRspCMReadReg]
  &cbReqCMWriteRegMsgDef,               ///< [cbMsgIdReqCMWriteReg]
  &cbReqUploadParamsMsgDef,             ///< [cbMsgIdReqUploadParams]
  &cbReqDownloadParamsMsgDef,           ///< [cbMsgIdReqDownloadParams]
  &cbRspDownloadParamsMsgDef,           ///< [cbMsgIdRspDownloadParams]
  &cbReqUploadNNSetStartMsgDef,         ///< [cbMsgIdReqUploadNNSetStart]
  &cbReqUploadNNSetNextMsgDef,          ///< [cbMsgIdReqUploadNNSetNext]
  &cbReqDownloadNNSetStartMsgDef,       ///< [cbMsgIdReqDownloadNNSetStart]
  &cbRspDownloadNNSetStartMsgDef,       ///< [cbMsgIdRspDownloadNNSetStart]
  &cbReqDownloadNNSetNextMsgDef,        ///< [cbMsgIdReqDownloadNNSetNext]
  &cbRspDownloadNNSetNextMsgDef,        ///< [cbMsgIdRspDownloadNNSetNext]
  &cbReqXferNNSetStopMsgDef,            ///< [cbMsgIdReqXferNNSetStop]
  NULL                                  ///< [cbMsgIdNumOf]
};

/*!
 * cb Message Maximum Size Look-Up Table. Indexed by cbMsgId_T enum.
 */
size_t cbMsgMaxLenLookupTbl[] =
{
  (size_t)(0),                          ///< [cbMsgIdNone]
  (size_t)(3),                          ///< [cbMsgIdRspOk]
  (size_t)(264),                        ///< [cbMsgIdRspErr]
  (size_t)(3),                          ///< [cbMsgIdReqVersion]
  (size_t)((3+((3+CB_ID_STR_LEN_MAX)+((3+CB_ID_STR_LEN_MAX)+((3+CB_ID_STR_LEN_MAX)+((3+CB_ID_STR_LEN_MAX)+((3+CB_ID_STR_LEN_MAX)+((3+CB_ID_STR_LEN_MAX)+(3+CB_ID_STR_LEN_MAX))))))))),
                                        ///< [cbMsgIdRspVersion]
  (size_t)(3),                          ///< [cbMsgIdReqPing]
  (size_t)(262),                        ///< [cbMsgIdReqLoopback]
  (size_t)(262),                        ///< [cbMsgIdRspLoopback]
  (size_t)(6),                          ///< [cbMsgIdReqReboot]
  (size_t)(3),                          ///< [cbMsgIdReqSleep]
  (size_t)(3),                          ///< [cbMsgIdReqWakeUp]
  (size_t)(9),                          ///< [cbMsgIdReqSetOpParamBaudRate]
  (size_t)(3),                          ///< [cbMsgIdReqGetOpParamBaudRate]
  (size_t)(9),                          ///< [cbMsgIdRspGetOpParamBaudRate]
  (size_t)(6),                          ///< [cbMsgIdReqSetOpParamAutoRestore]
  (size_t)(3),                          ///< [cbMsgIdReqGetOpParamAutoRestore]
  (size_t)(6),                          ///< [cbMsgIdRspGetOpParamAutoRestore]
  (size_t)(7),                          ///< [cbMsgIdReqSetOpParamAutoSleep]
  (size_t)(3),                          ///< [cbMsgIdReqGetOpParamAutoSleep]
  (size_t)(7),                          ///< [cbMsgIdRspGetOpParamAutoSleep]
  (size_t)(29),                         ///< [cbMsgIdReqSetOpParamLedBling]
  (size_t)(3),                          ///< [cbMsgIdReqGetOpParamLedBling]
  (size_t)(29),                         ///< [cbMsgIdRspGetOpParamLedBling]
  (size_t)(6),                          ///< [cbMsgIdReqSetCMParamClassifier]
  (size_t)(3),                          ///< [cbMsgIdReqGetCMParamClassifier]
  (size_t)(6),                          ///< [cbMsgIdRspGetCMParamClassifier]
  (size_t)(6),                          ///< [cbMsgIdReqSetCMParamContext]
  (size_t)(3),                          ///< [cbMsgIdReqGetCMParamContext]
  (size_t)(6),                          ///< [cbMsgIdRspGetCMParamContext]
  (size_t)(6),                          ///< [cbMsgIdReqSetCMParamNorm]
  (size_t)(3),                          ///< [cbMsgIdReqGetCMParamNorm]
  (size_t)(6),                          ///< [cbMsgIdRspGetCMParamNorm]
  (size_t)(7),                          ///< [cbMsgIdReqSetCMParamMinIF]
  (size_t)(3),                          ///< [cbMsgIdReqGetCMParamMinIF]
  (size_t)(7),                          ///< [cbMsgIdRspGetCMParamMinIF]
  (size_t)(7),                          ///< [cbMsgIdReqSetCMParamMaxIF]
  (size_t)(3),                          ///< [cbMsgIdReqGetCMParamMaxIF]
  (size_t)(7),                          ///< [cbMsgIdRspGetCMParamMaxIF]
  (size_t)(6),                          ///< [cbMsgIdReqSetCMParamMaxClassified]
  (size_t)(3),                          ///< [cbMsgIdReqGetCMParamMaxClassified]
  (size_t)(6),                          ///< [cbMsgIdRspGetCMParamMaxClassified]
  (size_t)((3+(3+CB_PARAM_NN_LABEL_LEN_MAX))),
                                        ///< [cbMsgIdReqSetNNParamLabel]
  (size_t)(3),                          ///< [cbMsgIdReqGetNNParamLabel]
  (size_t)((3+(3+CB_PARAM_NN_LABEL_LEN_MAX))),
                                        ///< [cbMsgIdRspGetNNParamLabel]
  (size_t)(3),                          ///< [cbMsgIdReqNVMemReset]
  (size_t)(3),                          ///< [cbMsgIdReqNVMemSave]
  (size_t)(3),                          ///< [cbMsgIdReqNVMemRestore]
  (size_t)(3),                          ///< [cbMsgIdReqNVMemGetNNCount]
  (size_t)(7),                          ///< [cbMsgIdRspNVMemGetNNCount]
  (size_t)((3+(4+(4+CM_PATTERN_COMP_NUMOF)))),
                                        ///< [cbMsgIdReqNNTrain]
  (size_t)(7),                          ///< [cbMsgIdRspNNTrain]
  (size_t)((3+(4+CM_PATTERN_COMP_NUMOF))),
                                        ///< [cbMsgIdReqNNCatergorize]
  (size_t)((3+(4+(15*CB_PARAM_MAX_CLASSIFIED_MAX)))),
                                        ///< [cbMsgIdRspNNCatergorize]
  (size_t)(3),                          ///< [cbMsgIdReqNNForget]
  (size_t)(3),                          ///< [cbMsgIdReqNNGetCount]
  (size_t)(7),                          ///< [cbMsgIdRspNNGetCount]
  (size_t)(6),                          ///< [cbMsgIdReqCMReadReg]
  (size_t)(10),                         ///< [cbMsgIdRspCMReadReg]
  (size_t)(10),                         ///< [cbMsgIdReqCMWriteReg]
  (size_t)((3+(3+(3+(6+(3+(4+(26+(3+(3+(3+(4+(4+(4+(3+CB_PARAM_NN_LABEL_LEN_MAX))))))))))))))),
                                        ///< [cbMsgIdReqUploadParams]
  (size_t)(6),                          ///< [cbMsgIdReqDownloadParams]
  (size_t)((3+(3+(6+(3+(4+(26+(3+(3+(3+(4+(4+(4+(3+CB_PARAM_NN_LABEL_LEN_MAX)))))))))))))),
                                        ///< [cbMsgIdRspDownloadParams]
  (size_t)((3+(3+(4+(3+(4+(3+(3+(4+(4+(4+CM_PATTERN_COMP_NUMOF))))))))))),
                                        ///< [cbMsgIdReqUploadNNSetStart]
  (size_t)((3+(3+(4+(3+(3+(4+(4+(4+CM_PATTERN_COMP_NUMOF))))))))),
                                        ///< [cbMsgIdReqUploadNNSetNext]
  (size_t)(6),                          ///< [cbMsgIdReqDownloadNNSetStart]
  (size_t)((3+(4+(3+(4+(3+(3+(4+(4+(4+CM_PATTERN_COMP_NUMOF)))))))))),
                                        ///< [cbMsgIdRspDownloadNNSetStart]
  (size_t)(3),                          ///< [cbMsgIdReqDownloadNNSetNext]
  (size_t)((3+(3+(4+(3+(3+(4+(4+(4+CM_PATTERN_COMP_NUMOF))))))))),
                                        ///< [cbMsgIdRspDownloadNNSetNext]
  (size_t)(6),                          ///< [cbMsgIdReqXferNNSetStop]
  (size_t)(0)                           ///< [cbMsgIdNumOf]
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
const NMMsgDef_T * cbLookupMsgDef( cbMsgId_T eMsgId )
{
  if( (uint_t)eMsgId >= (uint_t)cbMsgIdNumOf )
  {
    return NULL;
  }
  else
  {
    return cbMsgDefLookupTbl[(uint_t)eMsgId];
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
size_t cbLookupMsgMaxLen( cbMsgId_T eMsgId )
{
  if( (uint_t)eMsgId >= (uint_t)cbMsgIdNumOf )
  {
    return (size_t)0;
  }
  else
  {
    return cbMsgMaxLenLookupTbl[(uint_t)eMsgId];
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
int cbPackMsg( cbMsgId_T eMsgId,
               void * pStruct,
               byte_t buf[],
               size_t bufSize,
               bool_t bTrace )
{
  const  NMMsgDef_T  *pMsgDef;
  
  if( (pMsgDef = cbLookupMsgDef(eMsgId)) == NULL )
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
int cbUnpackMsg( cbMsgId_T eMsgId,
                 byte_t buf[],
                 size_t uMsgLen,
                 void * pStruct,
                 bool_t bTrace )
{
  const  NMMsgDef_T  *pMsgDef;
  
  if( (pMsgDef = cbLookupMsgDef(eMsgId)) == NULL )
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

