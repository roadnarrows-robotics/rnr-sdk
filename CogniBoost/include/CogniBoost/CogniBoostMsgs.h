///////////////////////////////////////////////////////////////////////////////
//
// File: CogniBoostMsgs.h
//
/*!
 * \file
 *
 * \brief CogniBoost host client - CogniBoost device server message set.
 *
 * \warning This file was auto-generated on 2011.10.19 14:55:19 from the NetMsgs
 * XML specification CogniBoostMsgs.xml.
 *
 * \par Copyright:
 * (C) 2011. RoadNarrows LLC
 * (http://www.roadnarrows.com)
 * All Rights Reserved
 */
///////////////////////////////////////////////////////////////////////////////


#ifndef _COGNIBOOSTMSGS_H
#define _COGNIBOOSTMSGS_H

#include "rnr/rnrconfig.h"
#include "rnr/netmsgs.h"

#include "CogniBoost/CogniMem.h"
#include "CogniBoost/CogniBoost.h"


C_DECLS_BEGIN

/*!
 * cb Message Id Enumeration
 */
typedef enum
{
  cbMsgIdNone                           = 0,    ///< no message
  cbMsgIdRspOk                          = 1,    ///< RspOk
  cbMsgIdRspErr                         = 2,    ///< RspErr
  cbMsgIdReqVersion                     = 3,    ///< ReqVersion
  cbMsgIdRspVersion                     = 4,    ///< RspVersion
  cbMsgIdReqPing                        = 5,    ///< ReqPing
  cbMsgIdReqLoopback                    = 6,    ///< ReqLoopback
  cbMsgIdRspLoopback                    = 7,    ///< RspLoopback
  cbMsgIdReqReboot                      = 8,    ///< ReqReboot
  cbMsgIdReqSleep                       = 9,    ///< ReqSleep
  cbMsgIdReqWakeUp                      = 10,   ///< ReqWakeUp
  cbMsgIdReqSetOpParamBaudRate          = 11,   ///< ReqSetOpParamBaudRate
  cbMsgIdReqGetOpParamBaudRate          = 12,   ///< ReqGetOpParamBaudRate
  cbMsgIdRspGetOpParamBaudRate          = 13,   ///< RspGetOpParamBaudRate
  cbMsgIdReqSetOpParamAutoRestore       = 14,   ///< ReqSetOpParamAutoRestore
  cbMsgIdReqGetOpParamAutoRestore       = 15,   ///< ReqGetOpParamAutoRestore
  cbMsgIdRspGetOpParamAutoRestore       = 16,   ///< RspGetOpParamAutoRestore
  cbMsgIdReqSetOpParamAutoSleep         = 17,   ///< ReqSetOpParamAutoSleep
  cbMsgIdReqGetOpParamAutoSleep         = 18,   ///< ReqGetOpParamAutoSleep
  cbMsgIdRspGetOpParamAutoSleep         = 19,   ///< RspGetOpParamAutoSleep
  cbMsgIdReqSetOpParamLedBling          = 20,   ///< ReqSetOpParamLedBling
  cbMsgIdReqGetOpParamLedBling          = 21,   ///< ReqGetOpParamLedBling
  cbMsgIdRspGetOpParamLedBling          = 22,   ///< RspGetOpParamLedBling
  cbMsgIdReqSetCMParamClassifier        = 23,   ///< ReqSetCMParamClassifier
  cbMsgIdReqGetCMParamClassifier        = 24,   ///< ReqGetCMParamClassifier
  cbMsgIdRspGetCMParamClassifier        = 25,   ///< RspGetCMParamClassifier
  cbMsgIdReqSetCMParamContext           = 26,   ///< ReqSetCMParamContext
  cbMsgIdReqGetCMParamContext           = 27,   ///< ReqGetCMParamContext
  cbMsgIdRspGetCMParamContext           = 28,   ///< RspGetCMParamContext
  cbMsgIdReqSetCMParamNorm              = 29,   ///< ReqSetCMParamNorm
  cbMsgIdReqGetCMParamNorm              = 30,   ///< ReqGetCMParamNorm
  cbMsgIdRspGetCMParamNorm              = 31,   ///< RspGetCMParamNorm
  cbMsgIdReqSetCMParamMinIF             = 32,   ///< ReqSetCMParamMinIF
  cbMsgIdReqGetCMParamMinIF             = 33,   ///< ReqGetCMParamMinIF
  cbMsgIdRspGetCMParamMinIF             = 34,   ///< RspGetCMParamMinIF
  cbMsgIdReqSetCMParamMaxIF             = 35,   ///< ReqSetCMParamMaxIF
  cbMsgIdReqGetCMParamMaxIF             = 36,   ///< ReqGetCMParamMaxIF
  cbMsgIdRspGetCMParamMaxIF             = 37,   ///< RspGetCMParamMaxIF
  cbMsgIdReqSetCMParamMaxClassified     = 38,   ///< ReqSetCMParamMaxClassified
  cbMsgIdReqGetCMParamMaxClassified     = 39,   ///< ReqGetCMParamMaxClassified
  cbMsgIdRspGetCMParamMaxClassified     = 40,   ///< RspGetCMParamMaxClassified
  cbMsgIdReqSetNNParamLabel             = 41,   ///< ReqSetNNParamLabel
  cbMsgIdReqGetNNParamLabel             = 42,   ///< ReqGetNNParamLabel
  cbMsgIdRspGetNNParamLabel             = 43,   ///< RspGetNNParamLabel
  cbMsgIdReqNVMemReset                  = 44,   ///< ReqNVMemReset
  cbMsgIdReqNVMemSave                   = 45,   ///< ReqNVMemSave
  cbMsgIdReqNVMemRestore                = 46,   ///< ReqNVMemRestore
  cbMsgIdReqNVMemGetNNCount             = 47,   ///< ReqNVMemGetNNCount
  cbMsgIdRspNVMemGetNNCount             = 48,   ///< RspNVMemGetNNCount
  cbMsgIdReqNNTrain                     = 49,   ///< ReqNNTrain
  cbMsgIdRspNNTrain                     = 50,   ///< RspNNTrain
  cbMsgIdReqNNCatergorize               = 51,   ///< ReqNNCatergorize
  cbMsgIdRspNNCatergorize               = 52,   ///< RspNNCatergorize
  cbMsgIdReqNNForget                    = 53,   ///< ReqNNForget
  cbMsgIdReqNNGetCount                  = 54,   ///< ReqNNGetCount
  cbMsgIdRspNNGetCount                  = 55,   ///< RspNNGetCount
  cbMsgIdReqCMReadReg                   = 56,   ///< ReqCMReadReg
  cbMsgIdRspCMReadReg                   = 57,   ///< RspCMReadReg
  cbMsgIdReqCMWriteReg                  = 58,   ///< ReqCMWriteReg
  cbMsgIdReqUploadParams                = 59,   ///< ReqUploadParams
  cbMsgIdReqDownloadParams              = 60,   ///< ReqDownloadParams
  cbMsgIdRspDownloadParams              = 61,   ///< RspDownloadParams
  cbMsgIdReqUploadNNSetStart            = 62,   ///< ReqUploadNNSetStart
  cbMsgIdReqUploadNNSetNext             = 63,   ///< ReqUploadNNSetNext
  cbMsgIdReqDownloadNNSetStart          = 64,   ///< ReqDownloadNNSetStart
  cbMsgIdRspDownloadNNSetStart          = 65,   ///< RspDownloadNNSetStart
  cbMsgIdReqDownloadNNSetNext           = 66,   ///< ReqDownloadNNSetNext
  cbMsgIdRspDownloadNNSetNext           = 67,   ///< RspDownloadNNSetNext
  cbMsgIdReqXferNNSetStop               = 68,   ///< ReqXferNNSetStop
  cbMsgIdNumOf                          = 69    ///< number of message ids
} cbMsgId_T;


//-----------------------------------------------------------------------------
// Extended Field Types
//-----------------------------------------------------------------------------


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Field Type cbLedBlingParams Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * LedBlingParams Field Type
 */
typedef struct
{
  byte_t                  m_id;               ///< id
  ushort_t                m_msecPeriod;       ///< msecPeriod
  ushort_t                m_msecDwell;        ///< msecDwell
  uint_t                  m_rgb1;             ///< rgb1
  uint_t                  m_rgb2;             ///< rgb2
} cbLedBlingParams_T;                         ///< LedBlingParams structure


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Field Type cbCogniBoostParams Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * CogniBoostParams Field Type
 */
typedef struct
{
  uint_t                  m_opBaudRate;       ///< opBaudRate
  bool_t                  m_opAutoRestore;    ///< opAutoRestore
  ushort_t                m_opAutoSleep;      ///< opAutoSleep
  cbLedBlingParams_T      m_opLedBling;       ///< opLedBling
  byte_t                  m_cmClassifier;     ///< cmClassifier
  byte_t                  m_cmContext;        ///< cmContext
  byte_t                  m_cmNorm;           ///< cmNorm
  ushort_t                m_cmMinif;          ///< cmMinif
  ushort_t                m_cmMaxif;          ///< cmMaxif
  ushort_t                m_cmMaxClassified;  ///< cmMaxClassified
  char                    m_nnLabel[CB_PARAM_NN_LABEL_LEN_MAX+1];
                                              ///< nnLabel
} cbCogniBoostParams_T;                       ///< CogniBoostParams structure


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Field Type cbNeuron Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * Neuron Field Type
 */
typedef struct
{
  ushort_t                m_neuronId;         ///< neuronId
  byte_t                  m_context;          ///< context
  byte_t                  m_norm;             ///< norm
  ushort_t                m_aif;              ///< aif
  ushort_t                m_category;         ///< category
  struct
  {
    size_t                m_count;            ///< vector item count
    union
    {
      void                *m_pAlign;          ///< force alignment
      byte_t              m_buf[CM_PATTERN_COMP_NUMOF];
                                              ///< the item vector
    } u; ///< aligned vector items
  } m_pattern;                                ///< vector
} cbNeuron_T;                                 ///< Neuron structure


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Field Type cbClassification Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * Classification Field Type
 */
typedef struct
{
  ushort_t                m_category;         ///< category
  ushort_t                m_dist;             ///< dist
  ushort_t                m_nid;              ///< nid
} cbClassification_T;                         ///< Classification structure


//-----------------------------------------------------------------------------
// Message Types
//-----------------------------------------------------------------------------


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbRspErr Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*! RspErr emsg maximum string length */
#define CB_RSPERR_EMSG_LEN (NMFVAL_LEN_MAX_STRING)

/*!
 * Message RspErr Structure
 */
typedef struct
{
  byte_t                  m_ecode;            ///< ecode
  char                    m_emsg[CB_RSPERR_EMSG_LEN+1];
                                              ///< emsg
} cbRspErr_T;                                 ///< RspErr structure


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbRspVersion Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * Message RspVersion Structure
 */
typedef struct
{
  char                    m_mfgName[CB_ID_STR_LEN_MAX+1];
                                              ///< mfgName
  char                    m_prodName[CB_ID_STR_LEN_MAX+1];
                                              ///< prodName
  char                    m_hwSN[CB_ID_STR_LEN_MAX+1];
                                              ///< hwSN
  char                    m_hwVer[CB_ID_STR_LEN_MAX+1];
                                              ///< hwVer
  char                    m_fwApp[CB_ID_STR_LEN_MAX+1];
                                              ///< fwApp
  char                    m_fwVer[CB_ID_STR_LEN_MAX+1];
                                              ///< fwVer
  char                    m_fwDate[CB_ID_STR_LEN_MAX+1];
                                              ///< fwDate
} cbRspVersion_T;                             ///< RspVersion structure


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqLoopback Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*! ReqLoopback cdata maximum vector length */
#define CB_REQLOOPBACK_CDATA_LEN (NMFVAL_LEN_MAX_VECTOR)

/*!
 * Message ReqLoopback Structure
 */
typedef struct
{
  struct
  {
    size_t                m_count;            ///< vector item count
    union
    {
      void                *m_pAlign;          ///< force alignment
      byte_t              m_buf[CB_REQLOOPBACK_CDATA_LEN];
                                              ///< the item vector
    } u; ///< aligned vector items
  } m_cdata;                                  ///< vector
} cbReqLoopback_T;                            ///< ReqLoopback structure


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbRspLoopback Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*! RspLoopback cdata maximum vector length */
#define CB_RSPLOOPBACK_CDATA_LEN (NMFVAL_LEN_MAX_VECTOR)

/*!
 * Message RspLoopback Structure
 */
typedef struct
{
  struct
  {
    size_t                m_count;            ///< vector item count
    union
    {
      void                *m_pAlign;          ///< force alignment
      byte_t              m_buf[CB_RSPLOOPBACK_CDATA_LEN];
                                              ///< the item vector
    } u; ///< aligned vector items
  } m_cdata;                                  ///< vector
} cbRspLoopback_T;                            ///< RspLoopback structure


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqReboot Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * Message ReqReboot Structure
 */
typedef struct
{
  bool_t                  m_bootloader;       ///< bootloader
} cbReqReboot_T;                              ///< ReqReboot structure


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqSetOpParamBaudRate Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * Message ReqSetOpParamBaudRate Structure
 */
typedef struct
{
  uint_t                  m_baudrate;         ///< baudrate
} cbReqSetOpParamBaudRate_T;                  ///< ReqSetOpParamBaudRate structure


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbRspGetOpParamBaudRate Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * Message RspGetOpParamBaudRate Structure
 */
typedef struct
{
  uint_t                  m_baudrate;         ///< baudrate
} cbRspGetOpParamBaudRate_T;                  ///< RspGetOpParamBaudRate structure


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqSetOpParamAutoRestore Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * Message ReqSetOpParamAutoRestore Structure
 */
typedef struct
{
  bool_t                  m_enable;           ///< enable
} cbReqSetOpParamAutoRestore_T;               ///< ReqSetOpParamAutoRestore structure


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbRspGetOpParamAutoRestore Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * Message RspGetOpParamAutoRestore Structure
 */
typedef struct
{
  bool_t                  m_enable;           ///< enable
} cbRspGetOpParamAutoRestore_T;               ///< RspGetOpParamAutoRestore structure


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqSetOpParamAutoSleep Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * Message ReqSetOpParamAutoSleep Structure
 */
typedef struct
{
  ushort_t                m_sec;              ///< sec
} cbReqSetOpParamAutoSleep_T;                 ///< ReqSetOpParamAutoSleep structure


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbRspGetOpParamAutoSleep Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * Message RspGetOpParamAutoSleep Structure
 */
typedef struct
{
  ushort_t                m_sec;              ///< sec
} cbRspGetOpParamAutoSleep_T;                 ///< RspGetOpParamAutoSleep structure


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqSetOpParamLedBling Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * Message ReqSetOpParamLedBling Structure
 */
typedef struct
{
  cbLedBlingParams_T      m_params;           ///< params
} cbReqSetOpParamLedBling_T;                  ///< ReqSetOpParamLedBling structure


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbRspGetOpParamLedBling Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * Message RspGetOpParamLedBling Structure
 */
typedef struct
{
  cbLedBlingParams_T      m_params;           ///< params
} cbRspGetOpParamLedBling_T;                  ///< RspGetOpParamLedBling structure


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqSetCMParamClassifier Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * Message ReqSetCMParamClassifier Structure
 */
typedef struct
{
  byte_t                  m_classifier;       ///< classifier
} cbReqSetCMParamClassifier_T;                ///< ReqSetCMParamClassifier structure


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbRspGetCMParamClassifier Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * Message RspGetCMParamClassifier Structure
 */
typedef struct
{
  byte_t                  m_classifier;       ///< classifier
} cbRspGetCMParamClassifier_T;                ///< RspGetCMParamClassifier structure


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqSetCMParamContext Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * Message ReqSetCMParamContext Structure
 */
typedef struct
{
  byte_t                  m_context;          ///< context
} cbReqSetCMParamContext_T;                   ///< ReqSetCMParamContext structure


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbRspGetCMParamContext Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * Message RspGetCMParamContext Structure
 */
typedef struct
{
  byte_t                  m_context;          ///< context
} cbRspGetCMParamContext_T;                   ///< RspGetCMParamContext structure


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqSetCMParamNorm Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * Message ReqSetCMParamNorm Structure
 */
typedef struct
{
  byte_t                  m_norm;             ///< norm
} cbReqSetCMParamNorm_T;                      ///< ReqSetCMParamNorm structure


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbRspGetCMParamNorm Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * Message RspGetCMParamNorm Structure
 */
typedef struct
{
  byte_t                  m_norm;             ///< norm
} cbRspGetCMParamNorm_T;                      ///< RspGetCMParamNorm structure


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqSetCMParamMinIF Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * Message ReqSetCMParamMinIF Structure
 */
typedef struct
{
  ushort_t                m_minif;            ///< minif
} cbReqSetCMParamMinIF_T;                     ///< ReqSetCMParamMinIF structure


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbRspGetCMParamMinIF Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * Message RspGetCMParamMinIF Structure
 */
typedef struct
{
  ushort_t                m_minif;            ///< minif
} cbRspGetCMParamMinIF_T;                     ///< RspGetCMParamMinIF structure


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqSetCMParamMaxIF Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * Message ReqSetCMParamMaxIF Structure
 */
typedef struct
{
  ushort_t                m_maxif;            ///< maxif
} cbReqSetCMParamMaxIF_T;                     ///< ReqSetCMParamMaxIF structure


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbRspGetCMParamMaxIF Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * Message RspGetCMParamMaxIF Structure
 */
typedef struct
{
  ushort_t                m_maxif;            ///< maxif
} cbRspGetCMParamMaxIF_T;                     ///< RspGetCMParamMaxIF structure


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqSetCMParamMaxClassified Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * Message ReqSetCMParamMaxClassified Structure
 */
typedef struct
{
  byte_t                  m_max;              ///< max
} cbReqSetCMParamMaxClassified_T;             ///< ReqSetCMParamMaxClassified structure


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbRspGetCMParamMaxClassified Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * Message RspGetCMParamMaxClassified Structure
 */
typedef struct
{
  byte_t                  m_max;              ///< max
} cbRspGetCMParamMaxClassified_T;             ///< RspGetCMParamMaxClassified structure


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqSetNNParamLabel Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * Message ReqSetNNParamLabel Structure
 */
typedef struct
{
  char                    m_label[CB_PARAM_NN_LABEL_LEN_MAX+1];
                                              ///< label
} cbReqSetNNParamLabel_T;                     ///< ReqSetNNParamLabel structure


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbRspGetNNParamLabel Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * Message RspGetNNParamLabel Structure
 */
typedef struct
{
  char                    m_label[CB_PARAM_NN_LABEL_LEN_MAX+1];
                                              ///< label
} cbRspGetNNParamLabel_T;                     ///< RspGetNNParamLabel structure


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbRspNVMemGetNNCount Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * Message RspNVMemGetNNCount Structure
 */
typedef struct
{
  ushort_t                m_count;            ///< count
} cbRspNVMemGetNNCount_T;                     ///< RspNVMemGetNNCount structure


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqNNTrain Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * Message ReqNNTrain Structure
 */
typedef struct
{
  ushort_t                m_category;         ///< category
  struct
  {
    size_t                m_count;            ///< vector item count
    union
    {
      void                *m_pAlign;          ///< force alignment
      byte_t              m_buf[CM_PATTERN_COMP_NUMOF];
                                              ///< the item vector
    } u; ///< aligned vector items
  } m_pattern;                                ///< vector
} cbReqNNTrain_T;                             ///< ReqNNTrain structure


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbRspNNTrain Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * Message RspNNTrain Structure
 */
typedef struct
{
  ushort_t                m_neuronId;         ///< neuronId
} cbRspNNTrain_T;                             ///< RspNNTrain structure


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqNNCatergorize Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * Message ReqNNCatergorize Structure
 */
typedef struct
{
  struct
  {
    size_t                m_count;            ///< vector item count
    union
    {
      void                *m_pAlign;          ///< force alignment
      byte_t              m_buf[CM_PATTERN_COMP_NUMOF];
                                              ///< the item vector
    } u; ///< aligned vector items
  } m_pattern;                                ///< vector
} cbReqNNCatergorize_T;                       ///< ReqNNCatergorize structure


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbRspNNCatergorize Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * Message RspNNCatergorize Structure
 */
typedef struct
{
  struct
  {
    size_t                m_count;            ///< vector item count
    union
    {
      void                *m_pAlign;          ///< force alignment
      cbClassification_T  m_buf[CB_PARAM_MAX_CLASSIFIED_MAX];
                                              ///< the item vector
    } u; ///< aligned vector items
  } m_classified;                             ///< vector
} cbRspNNCatergorize_T;                       ///< RspNNCatergorize structure


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbRspNNGetCount Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * Message RspNNGetCount Structure
 */
typedef struct
{
  ushort_t                m_count;            ///< count
} cbRspNNGetCount_T;                          ///< RspNNGetCount structure


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqCMReadReg Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * Message ReqCMReadReg Structure
 */
typedef struct
{
  byte_t                  m_addr;             ///< addr
} cbReqCMReadReg_T;                           ///< ReqCMReadReg structure


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbRspCMReadReg Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * Message RspCMReadReg Structure
 */
typedef struct
{
  byte_t                  m_addr;             ///< addr
  ushort_t                m_value;            ///< value
} cbRspCMReadReg_T;                           ///< RspCMReadReg structure


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqCMWriteReg Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * Message ReqCMWriteReg Structure
 */
typedef struct
{
  byte_t                  m_addr;             ///< addr
  ushort_t                m_value;            ///< value
} cbReqCMWriteReg_T;                          ///< ReqCMWriteReg structure


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqUploadParams Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * Message ReqUploadParams Structure
 */
typedef struct
{
  byte_t                  m_dst;              ///< dst
  cbCogniBoostParams_T    m_params;           ///< params
} cbReqUploadParams_T;                        ///< ReqUploadParams structure


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqDownloadParams Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * Message ReqDownloadParams Structure
 */
typedef struct
{
  byte_t                  m_src;              ///< src
} cbReqDownloadParams_T;                      ///< ReqDownloadParams structure


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbRspDownloadParams Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * Message RspDownloadParams Structure
 */
typedef struct
{
  cbCogniBoostParams_T    m_params;           ///< params
} cbRspDownloadParams_T;                      ///< RspDownloadParams structure


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqUploadNNSetStart Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * Message ReqUploadNNSetStart Structure
 */
typedef struct
{
  byte_t                  m_dst;              ///< dst
  ushort_t                m_numNeurons;       ///< numNeurons
  cbNeuron_T              m_neuron;           ///< neuron
} cbReqUploadNNSetStart_T;                    ///< ReqUploadNNSetStart structure


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqUploadNNSetNext Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * Message ReqUploadNNSetNext Structure
 */
typedef struct
{
  cbNeuron_T              m_neuron;           ///< neuron
} cbReqUploadNNSetNext_T;                     ///< ReqUploadNNSetNext structure


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqDownloadNNSetStart Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * Message ReqDownloadNNSetStart Structure
 */
typedef struct
{
  byte_t                  m_src;              ///< src
} cbReqDownloadNNSetStart_T;                  ///< ReqDownloadNNSetStart structure


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbRspDownloadNNSetStart Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * Message RspDownloadNNSetStart Structure
 */
typedef struct
{
  ushort_t                m_numNeurons;       ///< numNeurons
  cbNeuron_T              m_neuron;           ///< neuron
} cbRspDownloadNNSetStart_T;                  ///< RspDownloadNNSetStart structure


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbRspDownloadNNSetNext Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * Message RspDownloadNNSetNext Structure
 */
typedef struct
{
  cbNeuron_T              m_neuron;           ///< neuron
} cbRspDownloadNNSetNext_T;                   ///< RspDownloadNNSetNext structure


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Message cbReqXferNNSetStop Declarations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * Message ReqXferNNSetStop Structure
 */
typedef struct
{
  bool_t                  m_abort;            ///< abort
} cbReqXferNNSetStop_T;                       ///< ReqXferNNSetStop structure


//-----------------------------------------------------------------------------
// External Data
//-----------------------------------------------------------------------------

//
// cb Message Definition Look-Up Table.
// (indexed by cbMsgId_T enum)
// 
extern const NMMsgDef_T *cbMsgDefLookupTbl[];

//
// cb Maximum Message Body Length (bytes) Look-Up Table.
// (indexed by cbMsgId_T enum)
// 
extern size_t cbMsgMaxLenLookupTbl[];


//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------

extern const NMMsgDef_T * cbLookupMsgDef( cbMsgId_T eMsgId );

extern size_t cbLookupMsgMaxLen( cbMsgId_T eMsgId );

extern int cbPackMsg( cbMsgId_T eMsgId,
                      void * pStruct,
                      byte_t buf[],
                      size_t bufSize,
                      bool_t bTrace );

extern int cbUnpackMsg( cbMsgId_T eMsgId,
                        byte_t buf[],
                        size_t uMsgLen,
                        void * pStruct,
                        bool_t bTrace );

/*!
 * \brief Pack a cbRspErr ITV message in big-endian byte order
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
INLINE_IN_H int cbPackRspErr( cbRspErr_T * pStruct,
                              byte_t buf[],
                              size_t bufSize,
                              bool_t bTrace )
{
  return cbPackMsg(cbMsgIdRspErr, pStruct, buf, bufSize, bTrace);
}

/*!
 * \brief Unpack a cbRspErr ITV message in big-endian byte order
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
INLINE_IN_H int cbUnpackRspErr( byte_t buf[],
                                size_t uMsgLen,
                                cbRspErr_T * pStruct,
                                bool_t bTrace )
{
  return cbUnpackMsg(cbMsgIdRspErr, buf, uMsgLen, pStruct, bTrace);
}

/*!
 * \brief Pack a cbRspVersion ITV message in big-endian byte order
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
INLINE_IN_H int cbPackRspVersion( cbRspVersion_T * pStruct,
                                  byte_t buf[],
                                  size_t bufSize,
                                  bool_t bTrace )
{
  return cbPackMsg(cbMsgIdRspVersion, pStruct, buf, bufSize, bTrace);
}

/*!
 * \brief Unpack a cbRspVersion ITV message in big-endian byte order
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
INLINE_IN_H int cbUnpackRspVersion( byte_t buf[],
                                    size_t uMsgLen,
                                    cbRspVersion_T * pStruct,
                                    bool_t bTrace )
{
  return cbUnpackMsg(cbMsgIdRspVersion, buf, uMsgLen, pStruct, bTrace);
}

/*!
 * \brief Pack a cbReqLoopback ITV message in big-endian byte order
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
INLINE_IN_H int cbPackReqLoopback( cbReqLoopback_T * pStruct,
                                   byte_t buf[],
                                   size_t bufSize,
                                   bool_t bTrace )
{
  return cbPackMsg(cbMsgIdReqLoopback, pStruct, buf, bufSize, bTrace);
}

/*!
 * \brief Unpack a cbReqLoopback ITV message in big-endian byte order
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
INLINE_IN_H int cbUnpackReqLoopback( byte_t buf[],
                                     size_t uMsgLen,
                                     cbReqLoopback_T * pStruct,
                                     bool_t bTrace )
{
  return cbUnpackMsg(cbMsgIdReqLoopback, buf, uMsgLen, pStruct, bTrace);
}

/*!
 * \brief Pack a cbRspLoopback ITV message in big-endian byte order
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
INLINE_IN_H int cbPackRspLoopback( cbRspLoopback_T * pStruct,
                                   byte_t buf[],
                                   size_t bufSize,
                                   bool_t bTrace )
{
  return cbPackMsg(cbMsgIdRspLoopback, pStruct, buf, bufSize, bTrace);
}

/*!
 * \brief Unpack a cbRspLoopback ITV message in big-endian byte order
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
INLINE_IN_H int cbUnpackRspLoopback( byte_t buf[],
                                     size_t uMsgLen,
                                     cbRspLoopback_T * pStruct,
                                     bool_t bTrace )
{
  return cbUnpackMsg(cbMsgIdRspLoopback, buf, uMsgLen, pStruct, bTrace);
}

/*!
 * \brief Pack a cbReqReboot ITV message in big-endian byte order
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
INLINE_IN_H int cbPackReqReboot( cbReqReboot_T * pStruct,
                                 byte_t buf[],
                                 size_t bufSize,
                                 bool_t bTrace )
{
  return cbPackMsg(cbMsgIdReqReboot, pStruct, buf, bufSize, bTrace);
}

/*!
 * \brief Unpack a cbReqReboot ITV message in big-endian byte order
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
INLINE_IN_H int cbUnpackReqReboot( byte_t buf[],
                                   size_t uMsgLen,
                                   cbReqReboot_T * pStruct,
                                   bool_t bTrace )
{
  return cbUnpackMsg(cbMsgIdReqReboot, buf, uMsgLen, pStruct, bTrace);
}

/*!
 * \brief Pack a cbReqSetOpParamBaudRate ITV message in big-endian byte order
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
INLINE_IN_H int cbPackReqSetOpParamBaudRate( cbReqSetOpParamBaudRate_T * pStruct,
                                             byte_t buf[],
                                             size_t bufSize,
                                             bool_t bTrace )
{
  return cbPackMsg(cbMsgIdReqSetOpParamBaudRate, pStruct, buf, bufSize, bTrace);
}

/*!
 * \brief Unpack a cbReqSetOpParamBaudRate ITV message in big-endian byte order
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
INLINE_IN_H int cbUnpackReqSetOpParamBaudRate( byte_t buf[],
                                               size_t uMsgLen,
                                               cbReqSetOpParamBaudRate_T * pStruct,
                                               bool_t bTrace )
{
  return cbUnpackMsg(cbMsgIdReqSetOpParamBaudRate, buf, uMsgLen, pStruct, bTrace);
}

/*!
 * \brief Pack a cbRspGetOpParamBaudRate ITV message in big-endian byte order
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
INLINE_IN_H int cbPackRspGetOpParamBaudRate( cbRspGetOpParamBaudRate_T * pStruct,
                                             byte_t buf[],
                                             size_t bufSize,
                                             bool_t bTrace )
{
  return cbPackMsg(cbMsgIdRspGetOpParamBaudRate, pStruct, buf, bufSize, bTrace);
}

/*!
 * \brief Unpack a cbRspGetOpParamBaudRate ITV message in big-endian byte order
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
INLINE_IN_H int cbUnpackRspGetOpParamBaudRate( byte_t buf[],
                                               size_t uMsgLen,
                                               cbRspGetOpParamBaudRate_T * pStruct,
                                               bool_t bTrace )
{
  return cbUnpackMsg(cbMsgIdRspGetOpParamBaudRate, buf, uMsgLen, pStruct, bTrace);
}

/*!
 * \brief Pack a cbReqSetOpParamAutoRestore ITV message in big-endian byte order
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
INLINE_IN_H int cbPackReqSetOpParamAutoRestore( cbReqSetOpParamAutoRestore_T * pStruct,
                                                byte_t buf[],
                                                size_t bufSize,
                                                bool_t bTrace )
{
  return cbPackMsg(cbMsgIdReqSetOpParamAutoRestore, pStruct, buf, bufSize, bTrace);
}

/*!
 * \brief Unpack a cbReqSetOpParamAutoRestore ITV message in big-endian byte order
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
INLINE_IN_H int cbUnpackReqSetOpParamAutoRestore( byte_t buf[],
                                                  size_t uMsgLen,
                                                  cbReqSetOpParamAutoRestore_T * pStruct,
                                                  bool_t bTrace )
{
  return cbUnpackMsg(cbMsgIdReqSetOpParamAutoRestore, buf, uMsgLen, pStruct, bTrace);
}

/*!
 * \brief Pack a cbRspGetOpParamAutoRestore ITV message in big-endian byte order
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
INLINE_IN_H int cbPackRspGetOpParamAutoRestore( cbRspGetOpParamAutoRestore_T * pStruct,
                                                byte_t buf[],
                                                size_t bufSize,
                                                bool_t bTrace )
{
  return cbPackMsg(cbMsgIdRspGetOpParamAutoRestore, pStruct, buf, bufSize, bTrace);
}

/*!
 * \brief Unpack a cbRspGetOpParamAutoRestore ITV message in big-endian byte order
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
INLINE_IN_H int cbUnpackRspGetOpParamAutoRestore( byte_t buf[],
                                                  size_t uMsgLen,
                                                  cbRspGetOpParamAutoRestore_T * pStruct,
                                                  bool_t bTrace )
{
  return cbUnpackMsg(cbMsgIdRspGetOpParamAutoRestore, buf, uMsgLen, pStruct, bTrace);
}

/*!
 * \brief Pack a cbReqSetOpParamAutoSleep ITV message in big-endian byte order
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
INLINE_IN_H int cbPackReqSetOpParamAutoSleep( cbReqSetOpParamAutoSleep_T * pStruct,
                                              byte_t buf[],
                                              size_t bufSize,
                                              bool_t bTrace )
{
  return cbPackMsg(cbMsgIdReqSetOpParamAutoSleep, pStruct, buf, bufSize, bTrace);
}

/*!
 * \brief Unpack a cbReqSetOpParamAutoSleep ITV message in big-endian byte order
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
INLINE_IN_H int cbUnpackReqSetOpParamAutoSleep( byte_t buf[],
                                                size_t uMsgLen,
                                                cbReqSetOpParamAutoSleep_T * pStruct,
                                                bool_t bTrace )
{
  return cbUnpackMsg(cbMsgIdReqSetOpParamAutoSleep, buf, uMsgLen, pStruct, bTrace);
}

/*!
 * \brief Pack a cbRspGetOpParamAutoSleep ITV message in big-endian byte order
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
INLINE_IN_H int cbPackRspGetOpParamAutoSleep( cbRspGetOpParamAutoSleep_T * pStruct,
                                              byte_t buf[],
                                              size_t bufSize,
                                              bool_t bTrace )
{
  return cbPackMsg(cbMsgIdRspGetOpParamAutoSleep, pStruct, buf, bufSize, bTrace);
}

/*!
 * \brief Unpack a cbRspGetOpParamAutoSleep ITV message in big-endian byte order
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
INLINE_IN_H int cbUnpackRspGetOpParamAutoSleep( byte_t buf[],
                                                size_t uMsgLen,
                                                cbRspGetOpParamAutoSleep_T * pStruct,
                                                bool_t bTrace )
{
  return cbUnpackMsg(cbMsgIdRspGetOpParamAutoSleep, buf, uMsgLen, pStruct, bTrace);
}

/*!
 * \brief Pack a cbReqSetOpParamLedBling ITV message in big-endian byte order
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
INLINE_IN_H int cbPackReqSetOpParamLedBling( cbReqSetOpParamLedBling_T * pStruct,
                                             byte_t buf[],
                                             size_t bufSize,
                                             bool_t bTrace )
{
  return cbPackMsg(cbMsgIdReqSetOpParamLedBling, pStruct, buf, bufSize, bTrace);
}

/*!
 * \brief Unpack a cbReqSetOpParamLedBling ITV message in big-endian byte order
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
INLINE_IN_H int cbUnpackReqSetOpParamLedBling( byte_t buf[],
                                               size_t uMsgLen,
                                               cbReqSetOpParamLedBling_T * pStruct,
                                               bool_t bTrace )
{
  return cbUnpackMsg(cbMsgIdReqSetOpParamLedBling, buf, uMsgLen, pStruct, bTrace);
}

/*!
 * \brief Pack a cbRspGetOpParamLedBling ITV message in big-endian byte order
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
INLINE_IN_H int cbPackRspGetOpParamLedBling( cbRspGetOpParamLedBling_T * pStruct,
                                             byte_t buf[],
                                             size_t bufSize,
                                             bool_t bTrace )
{
  return cbPackMsg(cbMsgIdRspGetOpParamLedBling, pStruct, buf, bufSize, bTrace);
}

/*!
 * \brief Unpack a cbRspGetOpParamLedBling ITV message in big-endian byte order
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
INLINE_IN_H int cbUnpackRspGetOpParamLedBling( byte_t buf[],
                                               size_t uMsgLen,
                                               cbRspGetOpParamLedBling_T * pStruct,
                                               bool_t bTrace )
{
  return cbUnpackMsg(cbMsgIdRspGetOpParamLedBling, buf, uMsgLen, pStruct, bTrace);
}

/*!
 * \brief Pack a cbReqSetCMParamClassifier ITV message in big-endian byte order
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
INLINE_IN_H int cbPackReqSetCMParamClassifier( cbReqSetCMParamClassifier_T * pStruct,
                                               byte_t buf[],
                                               size_t bufSize,
                                               bool_t bTrace )
{
  return cbPackMsg(cbMsgIdReqSetCMParamClassifier, pStruct, buf, bufSize, bTrace);
}

/*!
 * \brief Unpack a cbReqSetCMParamClassifier ITV message in big-endian byte order
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
INLINE_IN_H int cbUnpackReqSetCMParamClassifier( byte_t buf[],
                                                 size_t uMsgLen,
                                                 cbReqSetCMParamClassifier_T * pStruct,
                                                 bool_t bTrace )
{
  return cbUnpackMsg(cbMsgIdReqSetCMParamClassifier, buf, uMsgLen, pStruct, bTrace);
}

/*!
 * \brief Pack a cbRspGetCMParamClassifier ITV message in big-endian byte order
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
INLINE_IN_H int cbPackRspGetCMParamClassifier( cbRspGetCMParamClassifier_T * pStruct,
                                               byte_t buf[],
                                               size_t bufSize,
                                               bool_t bTrace )
{
  return cbPackMsg(cbMsgIdRspGetCMParamClassifier, pStruct, buf, bufSize, bTrace);
}

/*!
 * \brief Unpack a cbRspGetCMParamClassifier ITV message in big-endian byte order
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
INLINE_IN_H int cbUnpackRspGetCMParamClassifier( byte_t buf[],
                                                 size_t uMsgLen,
                                                 cbRspGetCMParamClassifier_T * pStruct,
                                                 bool_t bTrace )
{
  return cbUnpackMsg(cbMsgIdRspGetCMParamClassifier, buf, uMsgLen, pStruct, bTrace);
}

/*!
 * \brief Pack a cbReqSetCMParamContext ITV message in big-endian byte order
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
INLINE_IN_H int cbPackReqSetCMParamContext( cbReqSetCMParamContext_T * pStruct,
                                            byte_t buf[],
                                            size_t bufSize,
                                            bool_t bTrace )
{
  return cbPackMsg(cbMsgIdReqSetCMParamContext, pStruct, buf, bufSize, bTrace);
}

/*!
 * \brief Unpack a cbReqSetCMParamContext ITV message in big-endian byte order
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
INLINE_IN_H int cbUnpackReqSetCMParamContext( byte_t buf[],
                                              size_t uMsgLen,
                                              cbReqSetCMParamContext_T * pStruct,
                                              bool_t bTrace )
{
  return cbUnpackMsg(cbMsgIdReqSetCMParamContext, buf, uMsgLen, pStruct, bTrace);
}

/*!
 * \brief Pack a cbRspGetCMParamContext ITV message in big-endian byte order
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
INLINE_IN_H int cbPackRspGetCMParamContext( cbRspGetCMParamContext_T * pStruct,
                                            byte_t buf[],
                                            size_t bufSize,
                                            bool_t bTrace )
{
  return cbPackMsg(cbMsgIdRspGetCMParamContext, pStruct, buf, bufSize, bTrace);
}

/*!
 * \brief Unpack a cbRspGetCMParamContext ITV message in big-endian byte order
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
INLINE_IN_H int cbUnpackRspGetCMParamContext( byte_t buf[],
                                              size_t uMsgLen,
                                              cbRspGetCMParamContext_T * pStruct,
                                              bool_t bTrace )
{
  return cbUnpackMsg(cbMsgIdRspGetCMParamContext, buf, uMsgLen, pStruct, bTrace);
}

/*!
 * \brief Pack a cbReqSetCMParamNorm ITV message in big-endian byte order
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
INLINE_IN_H int cbPackReqSetCMParamNorm( cbReqSetCMParamNorm_T * pStruct,
                                         byte_t buf[],
                                         size_t bufSize,
                                         bool_t bTrace )
{
  return cbPackMsg(cbMsgIdReqSetCMParamNorm, pStruct, buf, bufSize, bTrace);
}

/*!
 * \brief Unpack a cbReqSetCMParamNorm ITV message in big-endian byte order
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
INLINE_IN_H int cbUnpackReqSetCMParamNorm( byte_t buf[],
                                           size_t uMsgLen,
                                           cbReqSetCMParamNorm_T * pStruct,
                                           bool_t bTrace )
{
  return cbUnpackMsg(cbMsgIdReqSetCMParamNorm, buf, uMsgLen, pStruct, bTrace);
}

/*!
 * \brief Pack a cbRspGetCMParamNorm ITV message in big-endian byte order
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
INLINE_IN_H int cbPackRspGetCMParamNorm( cbRspGetCMParamNorm_T * pStruct,
                                         byte_t buf[],
                                         size_t bufSize,
                                         bool_t bTrace )
{
  return cbPackMsg(cbMsgIdRspGetCMParamNorm, pStruct, buf, bufSize, bTrace);
}

/*!
 * \brief Unpack a cbRspGetCMParamNorm ITV message in big-endian byte order
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
INLINE_IN_H int cbUnpackRspGetCMParamNorm( byte_t buf[],
                                           size_t uMsgLen,
                                           cbRspGetCMParamNorm_T * pStruct,
                                           bool_t bTrace )
{
  return cbUnpackMsg(cbMsgIdRspGetCMParamNorm, buf, uMsgLen, pStruct, bTrace);
}

/*!
 * \brief Pack a cbReqSetCMParamMinIF ITV message in big-endian byte order
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
INLINE_IN_H int cbPackReqSetCMParamMinIF( cbReqSetCMParamMinIF_T * pStruct,
                                          byte_t buf[],
                                          size_t bufSize,
                                          bool_t bTrace )
{
  return cbPackMsg(cbMsgIdReqSetCMParamMinIF, pStruct, buf, bufSize, bTrace);
}

/*!
 * \brief Unpack a cbReqSetCMParamMinIF ITV message in big-endian byte order
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
INLINE_IN_H int cbUnpackReqSetCMParamMinIF( byte_t buf[],
                                            size_t uMsgLen,
                                            cbReqSetCMParamMinIF_T * pStruct,
                                            bool_t bTrace )
{
  return cbUnpackMsg(cbMsgIdReqSetCMParamMinIF, buf, uMsgLen, pStruct, bTrace);
}

/*!
 * \brief Pack a cbRspGetCMParamMinIF ITV message in big-endian byte order
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
INLINE_IN_H int cbPackRspGetCMParamMinIF( cbRspGetCMParamMinIF_T * pStruct,
                                          byte_t buf[],
                                          size_t bufSize,
                                          bool_t bTrace )
{
  return cbPackMsg(cbMsgIdRspGetCMParamMinIF, pStruct, buf, bufSize, bTrace);
}

/*!
 * \brief Unpack a cbRspGetCMParamMinIF ITV message in big-endian byte order
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
INLINE_IN_H int cbUnpackRspGetCMParamMinIF( byte_t buf[],
                                            size_t uMsgLen,
                                            cbRspGetCMParamMinIF_T * pStruct,
                                            bool_t bTrace )
{
  return cbUnpackMsg(cbMsgIdRspGetCMParamMinIF, buf, uMsgLen, pStruct, bTrace);
}

/*!
 * \brief Pack a cbReqSetCMParamMaxIF ITV message in big-endian byte order
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
INLINE_IN_H int cbPackReqSetCMParamMaxIF( cbReqSetCMParamMaxIF_T * pStruct,
                                          byte_t buf[],
                                          size_t bufSize,
                                          bool_t bTrace )
{
  return cbPackMsg(cbMsgIdReqSetCMParamMaxIF, pStruct, buf, bufSize, bTrace);
}

/*!
 * \brief Unpack a cbReqSetCMParamMaxIF ITV message in big-endian byte order
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
INLINE_IN_H int cbUnpackReqSetCMParamMaxIF( byte_t buf[],
                                            size_t uMsgLen,
                                            cbReqSetCMParamMaxIF_T * pStruct,
                                            bool_t bTrace )
{
  return cbUnpackMsg(cbMsgIdReqSetCMParamMaxIF, buf, uMsgLen, pStruct, bTrace);
}

/*!
 * \brief Pack a cbRspGetCMParamMaxIF ITV message in big-endian byte order
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
INLINE_IN_H int cbPackRspGetCMParamMaxIF( cbRspGetCMParamMaxIF_T * pStruct,
                                          byte_t buf[],
                                          size_t bufSize,
                                          bool_t bTrace )
{
  return cbPackMsg(cbMsgIdRspGetCMParamMaxIF, pStruct, buf, bufSize, bTrace);
}

/*!
 * \brief Unpack a cbRspGetCMParamMaxIF ITV message in big-endian byte order
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
INLINE_IN_H int cbUnpackRspGetCMParamMaxIF( byte_t buf[],
                                            size_t uMsgLen,
                                            cbRspGetCMParamMaxIF_T * pStruct,
                                            bool_t bTrace )
{
  return cbUnpackMsg(cbMsgIdRspGetCMParamMaxIF, buf, uMsgLen, pStruct, bTrace);
}

/*!
 * \brief Pack a cbReqSetCMParamMaxClassified ITV message in big-endian byte order
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
INLINE_IN_H int cbPackReqSetCMParamMaxClassified( cbReqSetCMParamMaxClassified_T * pStruct,
                                                  byte_t buf[],
                                                  size_t bufSize,
                                                  bool_t bTrace )
{
  return cbPackMsg(cbMsgIdReqSetCMParamMaxClassified, pStruct, buf, bufSize, bTrace);
}

/*!
 * \brief Unpack a cbReqSetCMParamMaxClassified ITV message in big-endian byte order
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
INLINE_IN_H int cbUnpackReqSetCMParamMaxClassified( byte_t buf[],
                                                    size_t uMsgLen,
                                                    cbReqSetCMParamMaxClassified_T * pStruct,
                                                    bool_t bTrace )
{
  return cbUnpackMsg(cbMsgIdReqSetCMParamMaxClassified, buf, uMsgLen, pStruct, bTrace);
}

/*!
 * \brief Pack a cbRspGetCMParamMaxClassified ITV message in big-endian byte order
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
INLINE_IN_H int cbPackRspGetCMParamMaxClassified( cbRspGetCMParamMaxClassified_T * pStruct,
                                                  byte_t buf[],
                                                  size_t bufSize,
                                                  bool_t bTrace )
{
  return cbPackMsg(cbMsgIdRspGetCMParamMaxClassified, pStruct, buf, bufSize, bTrace);
}

/*!
 * \brief Unpack a cbRspGetCMParamMaxClassified ITV message in big-endian byte order
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
INLINE_IN_H int cbUnpackRspGetCMParamMaxClassified( byte_t buf[],
                                                    size_t uMsgLen,
                                                    cbRspGetCMParamMaxClassified_T * pStruct,
                                                    bool_t bTrace )
{
  return cbUnpackMsg(cbMsgIdRspGetCMParamMaxClassified, buf, uMsgLen, pStruct, bTrace);
}

/*!
 * \brief Pack a cbReqSetNNParamLabel ITV message in big-endian byte order
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
INLINE_IN_H int cbPackReqSetNNParamLabel( cbReqSetNNParamLabel_T * pStruct,
                                          byte_t buf[],
                                          size_t bufSize,
                                          bool_t bTrace )
{
  return cbPackMsg(cbMsgIdReqSetNNParamLabel, pStruct, buf, bufSize, bTrace);
}

/*!
 * \brief Unpack a cbReqSetNNParamLabel ITV message in big-endian byte order
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
INLINE_IN_H int cbUnpackReqSetNNParamLabel( byte_t buf[],
                                            size_t uMsgLen,
                                            cbReqSetNNParamLabel_T * pStruct,
                                            bool_t bTrace )
{
  return cbUnpackMsg(cbMsgIdReqSetNNParamLabel, buf, uMsgLen, pStruct, bTrace);
}

/*!
 * \brief Pack a cbRspGetNNParamLabel ITV message in big-endian byte order
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
INLINE_IN_H int cbPackRspGetNNParamLabel( cbRspGetNNParamLabel_T * pStruct,
                                          byte_t buf[],
                                          size_t bufSize,
                                          bool_t bTrace )
{
  return cbPackMsg(cbMsgIdRspGetNNParamLabel, pStruct, buf, bufSize, bTrace);
}

/*!
 * \brief Unpack a cbRspGetNNParamLabel ITV message in big-endian byte order
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
INLINE_IN_H int cbUnpackRspGetNNParamLabel( byte_t buf[],
                                            size_t uMsgLen,
                                            cbRspGetNNParamLabel_T * pStruct,
                                            bool_t bTrace )
{
  return cbUnpackMsg(cbMsgIdRspGetNNParamLabel, buf, uMsgLen, pStruct, bTrace);
}

/*!
 * \brief Pack a cbRspNVMemGetNNCount ITV message in big-endian byte order
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
INLINE_IN_H int cbPackRspNVMemGetNNCount( cbRspNVMemGetNNCount_T * pStruct,
                                          byte_t buf[],
                                          size_t bufSize,
                                          bool_t bTrace )
{
  return cbPackMsg(cbMsgIdRspNVMemGetNNCount, pStruct, buf, bufSize, bTrace);
}

/*!
 * \brief Unpack a cbRspNVMemGetNNCount ITV message in big-endian byte order
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
INLINE_IN_H int cbUnpackRspNVMemGetNNCount( byte_t buf[],
                                            size_t uMsgLen,
                                            cbRspNVMemGetNNCount_T * pStruct,
                                            bool_t bTrace )
{
  return cbUnpackMsg(cbMsgIdRspNVMemGetNNCount, buf, uMsgLen, pStruct, bTrace);
}

/*!
 * \brief Pack a cbReqNNTrain ITV message in big-endian byte order
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
INLINE_IN_H int cbPackReqNNTrain( cbReqNNTrain_T * pStruct,
                                  byte_t buf[],
                                  size_t bufSize,
                                  bool_t bTrace )
{
  return cbPackMsg(cbMsgIdReqNNTrain, pStruct, buf, bufSize, bTrace);
}

/*!
 * \brief Unpack a cbReqNNTrain ITV message in big-endian byte order
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
INLINE_IN_H int cbUnpackReqNNTrain( byte_t buf[],
                                    size_t uMsgLen,
                                    cbReqNNTrain_T * pStruct,
                                    bool_t bTrace )
{
  return cbUnpackMsg(cbMsgIdReqNNTrain, buf, uMsgLen, pStruct, bTrace);
}

/*!
 * \brief Pack a cbRspNNTrain ITV message in big-endian byte order
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
INLINE_IN_H int cbPackRspNNTrain( cbRspNNTrain_T * pStruct,
                                  byte_t buf[],
                                  size_t bufSize,
                                  bool_t bTrace )
{
  return cbPackMsg(cbMsgIdRspNNTrain, pStruct, buf, bufSize, bTrace);
}

/*!
 * \brief Unpack a cbRspNNTrain ITV message in big-endian byte order
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
INLINE_IN_H int cbUnpackRspNNTrain( byte_t buf[],
                                    size_t uMsgLen,
                                    cbRspNNTrain_T * pStruct,
                                    bool_t bTrace )
{
  return cbUnpackMsg(cbMsgIdRspNNTrain, buf, uMsgLen, pStruct, bTrace);
}

/*!
 * \brief Pack a cbReqNNCatergorize ITV message in big-endian byte order
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
INLINE_IN_H int cbPackReqNNCatergorize( cbReqNNCatergorize_T * pStruct,
                                        byte_t buf[],
                                        size_t bufSize,
                                        bool_t bTrace )
{
  return cbPackMsg(cbMsgIdReqNNCatergorize, pStruct, buf, bufSize, bTrace);
}

/*!
 * \brief Unpack a cbReqNNCatergorize ITV message in big-endian byte order
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
INLINE_IN_H int cbUnpackReqNNCatergorize( byte_t buf[],
                                          size_t uMsgLen,
                                          cbReqNNCatergorize_T * pStruct,
                                          bool_t bTrace )
{
  return cbUnpackMsg(cbMsgIdReqNNCatergorize, buf, uMsgLen, pStruct, bTrace);
}

/*!
 * \brief Pack a cbRspNNCatergorize ITV message in big-endian byte order
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
INLINE_IN_H int cbPackRspNNCatergorize( cbRspNNCatergorize_T * pStruct,
                                        byte_t buf[],
                                        size_t bufSize,
                                        bool_t bTrace )
{
  return cbPackMsg(cbMsgIdRspNNCatergorize, pStruct, buf, bufSize, bTrace);
}

/*!
 * \brief Unpack a cbRspNNCatergorize ITV message in big-endian byte order
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
INLINE_IN_H int cbUnpackRspNNCatergorize( byte_t buf[],
                                          size_t uMsgLen,
                                          cbRspNNCatergorize_T * pStruct,
                                          bool_t bTrace )
{
  return cbUnpackMsg(cbMsgIdRspNNCatergorize, buf, uMsgLen, pStruct, bTrace);
}

/*!
 * \brief Pack a cbRspNNGetCount ITV message in big-endian byte order
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
INLINE_IN_H int cbPackRspNNGetCount( cbRspNNGetCount_T * pStruct,
                                     byte_t buf[],
                                     size_t bufSize,
                                     bool_t bTrace )
{
  return cbPackMsg(cbMsgIdRspNNGetCount, pStruct, buf, bufSize, bTrace);
}

/*!
 * \brief Unpack a cbRspNNGetCount ITV message in big-endian byte order
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
INLINE_IN_H int cbUnpackRspNNGetCount( byte_t buf[],
                                       size_t uMsgLen,
                                       cbRspNNGetCount_T * pStruct,
                                       bool_t bTrace )
{
  return cbUnpackMsg(cbMsgIdRspNNGetCount, buf, uMsgLen, pStruct, bTrace);
}

/*!
 * \brief Pack a cbReqCMReadReg ITV message in big-endian byte order
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
INLINE_IN_H int cbPackReqCMReadReg( cbReqCMReadReg_T * pStruct,
                                    byte_t buf[],
                                    size_t bufSize,
                                    bool_t bTrace )
{
  return cbPackMsg(cbMsgIdReqCMReadReg, pStruct, buf, bufSize, bTrace);
}

/*!
 * \brief Unpack a cbReqCMReadReg ITV message in big-endian byte order
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
INLINE_IN_H int cbUnpackReqCMReadReg( byte_t buf[],
                                      size_t uMsgLen,
                                      cbReqCMReadReg_T * pStruct,
                                      bool_t bTrace )
{
  return cbUnpackMsg(cbMsgIdReqCMReadReg, buf, uMsgLen, pStruct, bTrace);
}

/*!
 * \brief Pack a cbRspCMReadReg ITV message in big-endian byte order
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
INLINE_IN_H int cbPackRspCMReadReg( cbRspCMReadReg_T * pStruct,
                                    byte_t buf[],
                                    size_t bufSize,
                                    bool_t bTrace )
{
  return cbPackMsg(cbMsgIdRspCMReadReg, pStruct, buf, bufSize, bTrace);
}

/*!
 * \brief Unpack a cbRspCMReadReg ITV message in big-endian byte order
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
INLINE_IN_H int cbUnpackRspCMReadReg( byte_t buf[],
                                      size_t uMsgLen,
                                      cbRspCMReadReg_T * pStruct,
                                      bool_t bTrace )
{
  return cbUnpackMsg(cbMsgIdRspCMReadReg, buf, uMsgLen, pStruct, bTrace);
}

/*!
 * \brief Pack a cbReqCMWriteReg ITV message in big-endian byte order
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
INLINE_IN_H int cbPackReqCMWriteReg( cbReqCMWriteReg_T * pStruct,
                                     byte_t buf[],
                                     size_t bufSize,
                                     bool_t bTrace )
{
  return cbPackMsg(cbMsgIdReqCMWriteReg, pStruct, buf, bufSize, bTrace);
}

/*!
 * \brief Unpack a cbReqCMWriteReg ITV message in big-endian byte order
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
INLINE_IN_H int cbUnpackReqCMWriteReg( byte_t buf[],
                                       size_t uMsgLen,
                                       cbReqCMWriteReg_T * pStruct,
                                       bool_t bTrace )
{
  return cbUnpackMsg(cbMsgIdReqCMWriteReg, buf, uMsgLen, pStruct, bTrace);
}

/*!
 * \brief Pack a cbReqUploadParams ITV message in big-endian byte order
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
INLINE_IN_H int cbPackReqUploadParams( cbReqUploadParams_T * pStruct,
                                       byte_t buf[],
                                       size_t bufSize,
                                       bool_t bTrace )
{
  return cbPackMsg(cbMsgIdReqUploadParams, pStruct, buf, bufSize, bTrace);
}

/*!
 * \brief Unpack a cbReqUploadParams ITV message in big-endian byte order
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
INLINE_IN_H int cbUnpackReqUploadParams( byte_t buf[],
                                         size_t uMsgLen,
                                         cbReqUploadParams_T * pStruct,
                                         bool_t bTrace )
{
  return cbUnpackMsg(cbMsgIdReqUploadParams, buf, uMsgLen, pStruct, bTrace);
}

/*!
 * \brief Pack a cbReqDownloadParams ITV message in big-endian byte order
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
INLINE_IN_H int cbPackReqDownloadParams( cbReqDownloadParams_T * pStruct,
                                         byte_t buf[],
                                         size_t bufSize,
                                         bool_t bTrace )
{
  return cbPackMsg(cbMsgIdReqDownloadParams, pStruct, buf, bufSize, bTrace);
}

/*!
 * \brief Unpack a cbReqDownloadParams ITV message in big-endian byte order
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
INLINE_IN_H int cbUnpackReqDownloadParams( byte_t buf[],
                                           size_t uMsgLen,
                                           cbReqDownloadParams_T * pStruct,
                                           bool_t bTrace )
{
  return cbUnpackMsg(cbMsgIdReqDownloadParams, buf, uMsgLen, pStruct, bTrace);
}

/*!
 * \brief Pack a cbRspDownloadParams ITV message in big-endian byte order
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
INLINE_IN_H int cbPackRspDownloadParams( cbRspDownloadParams_T * pStruct,
                                         byte_t buf[],
                                         size_t bufSize,
                                         bool_t bTrace )
{
  return cbPackMsg(cbMsgIdRspDownloadParams, pStruct, buf, bufSize, bTrace);
}

/*!
 * \brief Unpack a cbRspDownloadParams ITV message in big-endian byte order
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
INLINE_IN_H int cbUnpackRspDownloadParams( byte_t buf[],
                                           size_t uMsgLen,
                                           cbRspDownloadParams_T * pStruct,
                                           bool_t bTrace )
{
  return cbUnpackMsg(cbMsgIdRspDownloadParams, buf, uMsgLen, pStruct, bTrace);
}

/*!
 * \brief Pack a cbReqUploadNNSetStart ITV message in big-endian byte order
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
INLINE_IN_H int cbPackReqUploadNNSetStart( cbReqUploadNNSetStart_T * pStruct,
                                           byte_t buf[],
                                           size_t bufSize,
                                           bool_t bTrace )
{
  return cbPackMsg(cbMsgIdReqUploadNNSetStart, pStruct, buf, bufSize, bTrace);
}

/*!
 * \brief Unpack a cbReqUploadNNSetStart ITV message in big-endian byte order
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
INLINE_IN_H int cbUnpackReqUploadNNSetStart( byte_t buf[],
                                             size_t uMsgLen,
                                             cbReqUploadNNSetStart_T * pStruct,
                                             bool_t bTrace )
{
  return cbUnpackMsg(cbMsgIdReqUploadNNSetStart, buf, uMsgLen, pStruct, bTrace);
}

/*!
 * \brief Pack a cbReqUploadNNSetNext ITV message in big-endian byte order
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
INLINE_IN_H int cbPackReqUploadNNSetNext( cbReqUploadNNSetNext_T * pStruct,
                                          byte_t buf[],
                                          size_t bufSize,
                                          bool_t bTrace )
{
  return cbPackMsg(cbMsgIdReqUploadNNSetNext, pStruct, buf, bufSize, bTrace);
}

/*!
 * \brief Unpack a cbReqUploadNNSetNext ITV message in big-endian byte order
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
INLINE_IN_H int cbUnpackReqUploadNNSetNext( byte_t buf[],
                                            size_t uMsgLen,
                                            cbReqUploadNNSetNext_T * pStruct,
                                            bool_t bTrace )
{
  return cbUnpackMsg(cbMsgIdReqUploadNNSetNext, buf, uMsgLen, pStruct, bTrace);
}

/*!
 * \brief Pack a cbReqDownloadNNSetStart ITV message in big-endian byte order
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
INLINE_IN_H int cbPackReqDownloadNNSetStart( cbReqDownloadNNSetStart_T * pStruct,
                                             byte_t buf[],
                                             size_t bufSize,
                                             bool_t bTrace )
{
  return cbPackMsg(cbMsgIdReqDownloadNNSetStart, pStruct, buf, bufSize, bTrace);
}

/*!
 * \brief Unpack a cbReqDownloadNNSetStart ITV message in big-endian byte order
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
INLINE_IN_H int cbUnpackReqDownloadNNSetStart( byte_t buf[],
                                               size_t uMsgLen,
                                               cbReqDownloadNNSetStart_T * pStruct,
                                               bool_t bTrace )
{
  return cbUnpackMsg(cbMsgIdReqDownloadNNSetStart, buf, uMsgLen, pStruct, bTrace);
}

/*!
 * \brief Pack a cbRspDownloadNNSetStart ITV message in big-endian byte order
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
INLINE_IN_H int cbPackRspDownloadNNSetStart( cbRspDownloadNNSetStart_T * pStruct,
                                             byte_t buf[],
                                             size_t bufSize,
                                             bool_t bTrace )
{
  return cbPackMsg(cbMsgIdRspDownloadNNSetStart, pStruct, buf, bufSize, bTrace);
}

/*!
 * \brief Unpack a cbRspDownloadNNSetStart ITV message in big-endian byte order
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
INLINE_IN_H int cbUnpackRspDownloadNNSetStart( byte_t buf[],
                                               size_t uMsgLen,
                                               cbRspDownloadNNSetStart_T * pStruct,
                                               bool_t bTrace )
{
  return cbUnpackMsg(cbMsgIdRspDownloadNNSetStart, buf, uMsgLen, pStruct, bTrace);
}

/*!
 * \brief Pack a cbRspDownloadNNSetNext ITV message in big-endian byte order
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
INLINE_IN_H int cbPackRspDownloadNNSetNext( cbRspDownloadNNSetNext_T * pStruct,
                                            byte_t buf[],
                                            size_t bufSize,
                                            bool_t bTrace )
{
  return cbPackMsg(cbMsgIdRspDownloadNNSetNext, pStruct, buf, bufSize, bTrace);
}

/*!
 * \brief Unpack a cbRspDownloadNNSetNext ITV message in big-endian byte order
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
INLINE_IN_H int cbUnpackRspDownloadNNSetNext( byte_t buf[],
                                              size_t uMsgLen,
                                              cbRspDownloadNNSetNext_T * pStruct,
                                              bool_t bTrace )
{
  return cbUnpackMsg(cbMsgIdRspDownloadNNSetNext, buf, uMsgLen, pStruct, bTrace);
}

/*!
 * \brief Pack a cbReqXferNNSetStop ITV message in big-endian byte order
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
INLINE_IN_H int cbPackReqXferNNSetStop( cbReqXferNNSetStop_T * pStruct,
                                        byte_t buf[],
                                        size_t bufSize,
                                        bool_t bTrace )
{
  return cbPackMsg(cbMsgIdReqXferNNSetStop, pStruct, buf, bufSize, bTrace);
}

/*!
 * \brief Unpack a cbReqXferNNSetStop ITV message in big-endian byte order
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
INLINE_IN_H int cbUnpackReqXferNNSetStop( byte_t buf[],
                                          size_t uMsgLen,
                                          cbReqXferNNSetStop_T * pStruct,
                                          bool_t bTrace )
{
  return cbUnpackMsg(cbMsgIdReqXferNNSetStop, buf, uMsgLen, pStruct, bTrace);
}


C_DECLS_END


#endif // _COGNIBOOSTMSGS_H
