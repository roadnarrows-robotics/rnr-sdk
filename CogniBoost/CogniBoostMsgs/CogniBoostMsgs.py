##############################################################################
#
# File: CogniBoostMsgs.py
#

"""
CogniBoostMsgs.py
"""

## \file
##
## \brief CogniBoost host client - CogniBoost device server message set.
##
## \warning This file was auto-generated on 2011.10.19 14:55:19 from the NetMsgs
## XML specification CogniBoostMsgs.xml.
##
## \par Copyright:
## (C) 2011. RoadNarrows LLC
## (http://www.roadnarrows.com)
## All Rights Reserved
##
##############################################################################


import NetMsgs.NetMsgsBase as nmBase
from NetMsgs.NetMsgsLibITV import NetMsgsITV


# -----------------------------------------------------------------------------
# cb Message Id Enumeration
# -----------------------------------------------------------------------------

class cbMsgId:
  """ cb Message Id Enumeration class. """
  NoId                            = 0     # no message id
  RspOk                           = 1     # RspOk
  RspErr                          = 2     # RspErr
  ReqVersion                      = 3     # ReqVersion
  RspVersion                      = 4     # RspVersion
  ReqPing                         = 5     # ReqPing
  ReqLoopback                     = 6     # ReqLoopback
  RspLoopback                     = 7     # RspLoopback
  ReqReboot                       = 8     # ReqReboot
  ReqSleep                        = 9     # ReqSleep
  ReqWakeUp                       = 10    # ReqWakeUp
  ReqSetOpParamBaudRate           = 11    # ReqSetOpParamBaudRate
  ReqGetOpParamBaudRate           = 12    # ReqGetOpParamBaudRate
  RspGetOpParamBaudRate           = 13    # RspGetOpParamBaudRate
  ReqSetOpParamAutoRestore        = 14    # ReqSetOpParamAutoRestore
  ReqGetOpParamAutoRestore        = 15    # ReqGetOpParamAutoRestore
  RspGetOpParamAutoRestore        = 16    # RspGetOpParamAutoRestore
  ReqSetOpParamAutoSleep          = 17    # ReqSetOpParamAutoSleep
  ReqGetOpParamAutoSleep          = 18    # ReqGetOpParamAutoSleep
  RspGetOpParamAutoSleep          = 19    # RspGetOpParamAutoSleep
  ReqSetOpParamLedBling           = 20    # ReqSetOpParamLedBling
  ReqGetOpParamLedBling           = 21    # ReqGetOpParamLedBling
  RspGetOpParamLedBling           = 22    # RspGetOpParamLedBling
  ReqSetCMParamClassifier         = 23    # ReqSetCMParamClassifier
  ReqGetCMParamClassifier         = 24    # ReqGetCMParamClassifier
  RspGetCMParamClassifier         = 25    # RspGetCMParamClassifier
  ReqSetCMParamContext            = 26    # ReqSetCMParamContext
  ReqGetCMParamContext            = 27    # ReqGetCMParamContext
  RspGetCMParamContext            = 28    # RspGetCMParamContext
  ReqSetCMParamNorm               = 29    # ReqSetCMParamNorm
  ReqGetCMParamNorm               = 30    # ReqGetCMParamNorm
  RspGetCMParamNorm               = 31    # RspGetCMParamNorm
  ReqSetCMParamMinIF              = 32    # ReqSetCMParamMinIF
  ReqGetCMParamMinIF              = 33    # ReqGetCMParamMinIF
  RspGetCMParamMinIF              = 34    # RspGetCMParamMinIF
  ReqSetCMParamMaxIF              = 35    # ReqSetCMParamMaxIF
  ReqGetCMParamMaxIF              = 36    # ReqGetCMParamMaxIF
  RspGetCMParamMaxIF              = 37    # RspGetCMParamMaxIF
  ReqSetCMParamMaxClassified      = 38    # ReqSetCMParamMaxClassified
  ReqGetCMParamMaxClassified      = 39    # ReqGetCMParamMaxClassified
  RspGetCMParamMaxClassified      = 40    # RspGetCMParamMaxClassified
  ReqSetNNParamLabel              = 41    # ReqSetNNParamLabel
  ReqGetNNParamLabel              = 42    # ReqGetNNParamLabel
  RspGetNNParamLabel              = 43    # RspGetNNParamLabel
  ReqNVMemReset                   = 44    # ReqNVMemReset
  ReqNVMemSave                    = 45    # ReqNVMemSave
  ReqNVMemRestore                 = 46    # ReqNVMemRestore
  ReqNVMemGetNNCount              = 47    # ReqNVMemGetNNCount
  RspNVMemGetNNCount              = 48    # RspNVMemGetNNCount
  ReqNNTrain                      = 49    # ReqNNTrain
  RspNNTrain                      = 50    # RspNNTrain
  ReqNNCatergorize                = 51    # ReqNNCatergorize
  RspNNCatergorize                = 52    # RspNNCatergorize
  ReqNNForget                     = 53    # ReqNNForget
  ReqNNGetCount                   = 54    # ReqNNGetCount
  RspNNGetCount                   = 55    # RspNNGetCount
  ReqCMReadReg                    = 56    # ReqCMReadReg
  RspCMReadReg                    = 57    # RspCMReadReg
  ReqCMWriteReg                   = 58    # ReqCMWriteReg
  ReqUploadParams                 = 59    # ReqUploadParams
  ReqDownloadParams               = 60    # ReqDownloadParams
  RspDownloadParams               = 61    # RspDownloadParams
  ReqUploadNNSetStart             = 62    # ReqUploadNNSetStart
  ReqUploadNNSetNext              = 63    # ReqUploadNNSetNext
  ReqDownloadNNSetStart           = 64    # ReqDownloadNNSetStart
  RspDownloadNNSetStart           = 65    # RspDownloadNNSetStart
  ReqDownloadNNSetNext            = 66    # ReqDownloadNNSetNext
  RspDownloadNNSetNext            = 67    # RspDownloadNNSetNext
  ReqXferNNSetStop                = 68    # ReqXferNNSetStop
  NumOf                           = 69    # number of message ids
##


# -----------------------------------------------------------------------------
# Extended Field Types
# -----------------------------------------------------------------------------

## LedBlingParams Extended Field Type 
cbExtFTypeLedBlingParams = {
  'fielddef': [
    {'fid':1, 'ftype':'B', 'max_count':1, 'name':'id', },
    {'fid':2, 'ftype':'H', 'max_count':1, 'name':'msecPeriod', },
    {'fid':3, 'ftype':'H', 'max_count':1, 'name':'msecDwell', },
    {'fid':4, 'ftype':'I', 'max_count':1, 'name':'rgb1', },
    {'fid':5, 'ftype':'I', 'max_count':1, 'name':'rgb2', },
  ],
  'ftype': '{',
  'max_count': 5,
  'msgid': cbMsgId.NoId,
  'name': 'LedBlingParams',
}
##

## CogniBoostParams Extended Field Type 
cbExtFTypeCogniBoostParams = {
  'fielddef': [
    {'fid':1, 'ftype':'I', 'max_count':1, 'name':'opBaudRate', },
    {'fid':2, 'ftype':'?', 'max_count':1, 'name':'opAutoRestore', },
    {'fid':3, 'ftype':'H', 'max_count':1, 'name':'opAutoSleep', },
    {
      'fid': 4,
      'ftype': '{',
      'max_count': 1,
      'msgdef': cbExtFTypeLedBlingParams,
      'name': 'opLedBling',
    },
    {'fid':5, 'ftype':'B', 'max_count':1, 'name':'cmClassifier', },
    {'fid':6, 'ftype':'B', 'max_count':1, 'name':'cmContext', },
    {'fid':7, 'ftype':'B', 'max_count':1, 'name':'cmNorm', },
    {'fid':8, 'ftype':'H', 'max_count':1, 'name':'cmMinif', },
    {'fid':9, 'ftype':'H', 'max_count':1, 'name':'cmMaxif', },
    {'fid':10, 'ftype':'H', 'max_count':1, 'name':'cmMaxClassified', },
    {
      'fid': 11,
      'ftype': 's',
      'max_count': CB_PARAM_NN_LABEL_LEN_MAX,
      'name': 'nnLabel',
    },
  ],
  'ftype': '{',
  'max_count': 11,
  'msgid': cbMsgId.NoId,
  'name': 'CogniBoostParams',
}
##

## Neuron Extended Field Type 
cbExtFTypeNeuron = {
  'fielddef': [
    {'fid':1, 'ftype':'H', 'max_count':1, 'name':'neuronId', },
    {'fid':2, 'ftype':'B', 'max_count':1, 'name':'context', },
    {'fid':3, 'ftype':'B', 'max_count':1, 'name':'norm', },
    {'fid':4, 'ftype':'H', 'max_count':1, 'name':'aif', },
    {'fid':5, 'ftype':'H', 'max_count':1, 'name':'category', },
    {
      'fid': 6,
      'ftype': '[',
      'max_count': CM_PATTERN_COMP_NUMOF,
      'name': 'pattern',
      'vdef': {
        'fid': nmBase.NMFIdNone,
        'ftype': 'B',
        'max_count': 1,
        'name': 'pattern_item',
      },
    },
  ],
  'ftype': '{',
  'max_count': 6,
  'msgid': cbMsgId.NoId,
  'name': 'Neuron',
}
##

## Classification Extended Field Type 
cbExtFTypeClassification = {
  'fielddef': [
    {'fid':1, 'ftype':'H', 'max_count':1, 'name':'category', },
    {'fid':2, 'ftype':'H', 'max_count':1, 'name':'dist', },
    {'fid':3, 'ftype':'H', 'max_count':1, 'name':'nid', },
  ],
  'ftype': '{',
  'max_count': 3,
  'msgid': cbMsgId.NoId,
  'name': 'Classification',
}
##

## cb Extended Field Type Dictionary
cbExtFieldTypes = {
  'Classification': cbExtFTypeClassification,
  'CogniBoostParams': cbExtFTypeCogniBoostParams,
  'LedBlingParams': cbExtFTypeLedBlingParams,
  'Neuron': cbExtFTypeNeuron,
}
##


# -----------------------------------------------------------------------------
# Message Definition Set
# -----------------------------------------------------------------------------

## RspOk Message Definition 
cbMsgDefRspOk = {
  'fielddef': [  ],
  'ftype': '{',
  'max_count': 0,
  'msgid': cbMsgId.RspOk,
  'name': 'RspOk',
}
##

## RspErr Message Definition 
cbMsgDefRspErr = {
  'fielddef': [
    {'fid':1, 'ftype':'B', 'max_count':1, 'name':'ecode', },
    {
      'fid': 2,
      'ftype': 's',
      'max_count': nmBase.NMStringMaxCount,
      'name': 'emsg',
    },
  ],
  'ftype': '{',
  'max_count': 2,
  'msgid': cbMsgId.RspErr,
  'name': 'RspErr',
}
##

## ReqVersion Message Definition 
cbMsgDefReqVersion = {
  'fielddef': [  ],
  'ftype': '{',
  'max_count': 0,
  'msgid': cbMsgId.ReqVersion,
  'name': 'ReqVersion',
}
##

## RspVersion Message Definition 
cbMsgDefRspVersion = {
  'fielddef': [
    {
      'fid': 1,
      'ftype': 's',
      'max_count': CB_ID_STR_LEN_MAX,
      'name': 'mfgName',
    },
    {
      'fid': 2,
      'ftype': 's',
      'max_count': CB_ID_STR_LEN_MAX,
      'name': 'prodName',
    },
    {'fid':3, 'ftype':'s', 'max_count':CB_ID_STR_LEN_MAX, 'name':'hwSN', },
    {
      'fid': 4,
      'ftype': 's',
      'max_count': CB_ID_STR_LEN_MAX,
      'name': 'hwVer',
    },
    {
      'fid': 5,
      'ftype': 's',
      'max_count': CB_ID_STR_LEN_MAX,
      'name': 'fwApp',
    },
    {
      'fid': 6,
      'ftype': 's',
      'max_count': CB_ID_STR_LEN_MAX,
      'name': 'fwVer',
    },
    {
      'fid': 7,
      'ftype': 's',
      'max_count': CB_ID_STR_LEN_MAX,
      'name': 'fwDate',
    },
  ],
  'ftype': '{',
  'max_count': 7,
  'msgid': cbMsgId.RspVersion,
  'name': 'RspVersion',
}
##

## ReqPing Message Definition 
cbMsgDefReqPing = {
  'fielddef': [  ],
  'ftype': '{',
  'max_count': 0,
  'msgid': cbMsgId.ReqPing,
  'name': 'ReqPing',
}
##

## ReqLoopback Message Definition 
cbMsgDefReqLoopback = {
  'fielddef': [
    {
      'fid': 1,
      'ftype': '[',
      'max_count': nmBase.NMVectorMaxCount,
      'name': 'cdata',
      'vdef': {
        'fid': nmBase.NMFIdNone,
        'ftype': 'B',
        'max_count': 1,
        'name': 'cdata_item',
      },
    },
  ],
  'ftype': '{',
  'max_count': 1,
  'msgid': cbMsgId.ReqLoopback,
  'name': 'ReqLoopback',
}
##

## RspLoopback Message Definition 
cbMsgDefRspLoopback = {
  'fielddef': [
    {
      'fid': 1,
      'ftype': '[',
      'max_count': nmBase.NMVectorMaxCount,
      'name': 'cdata',
      'vdef': {
        'fid': nmBase.NMFIdNone,
        'ftype': 'B',
        'max_count': 1,
        'name': 'cdata_item',
      },
    },
  ],
  'ftype': '{',
  'max_count': 1,
  'msgid': cbMsgId.RspLoopback,
  'name': 'RspLoopback',
}
##

## ReqReboot Message Definition 
cbMsgDefReqReboot = {
  'fielddef': [
    {'fid':1, 'ftype':'?', 'max_count':1, 'name':'bootloader', },
  ],
  'ftype': '{',
  'max_count': 1,
  'msgid': cbMsgId.ReqReboot,
  'name': 'ReqReboot',
}
##

## ReqSleep Message Definition 
cbMsgDefReqSleep = {
  'fielddef': [  ],
  'ftype': '{',
  'max_count': 0,
  'msgid': cbMsgId.ReqSleep,
  'name': 'ReqSleep',
}
##

## ReqWakeUp Message Definition 
cbMsgDefReqWakeUp = {
  'fielddef': [  ],
  'ftype': '{',
  'max_count': 0,
  'msgid': cbMsgId.ReqWakeUp,
  'name': 'ReqWakeUp',
}
##

## ReqSetOpParamBaudRate Message Definition 
cbMsgDefReqSetOpParamBaudRate = {
  'fielddef': [
    {'fid':1, 'ftype':'I', 'max_count':1, 'name':'baudrate', },
  ],
  'ftype': '{',
  'max_count': 1,
  'msgid': cbMsgId.ReqSetOpParamBaudRate,
  'name': 'ReqSetOpParamBaudRate',
}
##

## ReqGetOpParamBaudRate Message Definition 
cbMsgDefReqGetOpParamBaudRate = {
  'fielddef': [  ],
  'ftype': '{',
  'max_count': 0,
  'msgid': cbMsgId.ReqGetOpParamBaudRate,
  'name': 'ReqGetOpParamBaudRate',
}
##

## RspGetOpParamBaudRate Message Definition 
cbMsgDefRspGetOpParamBaudRate = {
  'fielddef': [
    {'fid':1, 'ftype':'I', 'max_count':1, 'name':'baudrate', },
  ],
  'ftype': '{',
  'max_count': 1,
  'msgid': cbMsgId.RspGetOpParamBaudRate,
  'name': 'RspGetOpParamBaudRate',
}
##

## ReqSetOpParamAutoRestore Message Definition 
cbMsgDefReqSetOpParamAutoRestore = {
  'fielddef': [
    {'fid':1, 'ftype':'?', 'max_count':1, 'name':'enable', },
  ],
  'ftype': '{',
  'max_count': 1,
  'msgid': cbMsgId.ReqSetOpParamAutoRestore,
  'name': 'ReqSetOpParamAutoRestore',
}
##

## ReqGetOpParamAutoRestore Message Definition 
cbMsgDefReqGetOpParamAutoRestore = {
  'fielddef': [  ],
  'ftype': '{',
  'max_count': 0,
  'msgid': cbMsgId.ReqGetOpParamAutoRestore,
  'name': 'ReqGetOpParamAutoRestore',
}
##

## RspGetOpParamAutoRestore Message Definition 
cbMsgDefRspGetOpParamAutoRestore = {
  'fielddef': [
    {'fid':1, 'ftype':'?', 'max_count':1, 'name':'enable', },
  ],
  'ftype': '{',
  'max_count': 1,
  'msgid': cbMsgId.RspGetOpParamAutoRestore,
  'name': 'RspGetOpParamAutoRestore',
}
##

## ReqSetOpParamAutoSleep Message Definition 
cbMsgDefReqSetOpParamAutoSleep = {
  'fielddef': [
    {'fid':1, 'ftype':'H', 'max_count':1, 'name':'sec', },
  ],
  'ftype': '{',
  'max_count': 1,
  'msgid': cbMsgId.ReqSetOpParamAutoSleep,
  'name': 'ReqSetOpParamAutoSleep',
}
##

## ReqGetOpParamAutoSleep Message Definition 
cbMsgDefReqGetOpParamAutoSleep = {
  'fielddef': [  ],
  'ftype': '{',
  'max_count': 0,
  'msgid': cbMsgId.ReqGetOpParamAutoSleep,
  'name': 'ReqGetOpParamAutoSleep',
}
##

## RspGetOpParamAutoSleep Message Definition 
cbMsgDefRspGetOpParamAutoSleep = {
  'fielddef': [
    {'fid':1, 'ftype':'H', 'max_count':1, 'name':'sec', },
  ],
  'ftype': '{',
  'max_count': 1,
  'msgid': cbMsgId.RspGetOpParamAutoSleep,
  'name': 'RspGetOpParamAutoSleep',
}
##

## ReqSetOpParamLedBling Message Definition 
cbMsgDefReqSetOpParamLedBling = {
  'fielddef': [
    {
      'fid': 1,
      'ftype': '{',
      'max_count': 1,
      'msgdef': cbExtFTypeLedBlingParams,
      'name': 'params',
    },
  ],
  'ftype': '{',
  'max_count': 1,
  'msgid': cbMsgId.ReqSetOpParamLedBling,
  'name': 'ReqSetOpParamLedBling',
}
##

## ReqGetOpParamLedBling Message Definition 
cbMsgDefReqGetOpParamLedBling = {
  'fielddef': [  ],
  'ftype': '{',
  'max_count': 0,
  'msgid': cbMsgId.ReqGetOpParamLedBling,
  'name': 'ReqGetOpParamLedBling',
}
##

## RspGetOpParamLedBling Message Definition 
cbMsgDefRspGetOpParamLedBling = {
  'fielddef': [
    {
      'fid': 1,
      'ftype': '{',
      'max_count': 1,
      'msgdef': cbExtFTypeLedBlingParams,
      'name': 'params',
    },
  ],
  'ftype': '{',
  'max_count': 1,
  'msgid': cbMsgId.RspGetOpParamLedBling,
  'name': 'RspGetOpParamLedBling',
}
##

## ReqSetCMParamClassifier Message Definition 
cbMsgDefReqSetCMParamClassifier = {
  'fielddef': [
    {'fid':1, 'ftype':'B', 'max_count':1, 'name':'classifier', },
  ],
  'ftype': '{',
  'max_count': 1,
  'msgid': cbMsgId.ReqSetCMParamClassifier,
  'name': 'ReqSetCMParamClassifier',
}
##

## ReqGetCMParamClassifier Message Definition 
cbMsgDefReqGetCMParamClassifier = {
  'fielddef': [  ],
  'ftype': '{',
  'max_count': 0,
  'msgid': cbMsgId.ReqGetCMParamClassifier,
  'name': 'ReqGetCMParamClassifier',
}
##

## RspGetCMParamClassifier Message Definition 
cbMsgDefRspGetCMParamClassifier = {
  'fielddef': [
    {'fid':1, 'ftype':'B', 'max_count':1, 'name':'classifier', },
  ],
  'ftype': '{',
  'max_count': 1,
  'msgid': cbMsgId.RspGetCMParamClassifier,
  'name': 'RspGetCMParamClassifier',
}
##

## ReqSetCMParamContext Message Definition 
cbMsgDefReqSetCMParamContext = {
  'fielddef': [
    {'fid':1, 'ftype':'B', 'max_count':1, 'name':'context', },
  ],
  'ftype': '{',
  'max_count': 1,
  'msgid': cbMsgId.ReqSetCMParamContext,
  'name': 'ReqSetCMParamContext',
}
##

## ReqGetCMParamContext Message Definition 
cbMsgDefReqGetCMParamContext = {
  'fielddef': [  ],
  'ftype': '{',
  'max_count': 0,
  'msgid': cbMsgId.ReqGetCMParamContext,
  'name': 'ReqGetCMParamContext',
}
##

## RspGetCMParamContext Message Definition 
cbMsgDefRspGetCMParamContext = {
  'fielddef': [
    {'fid':1, 'ftype':'B', 'max_count':1, 'name':'context', },
  ],
  'ftype': '{',
  'max_count': 1,
  'msgid': cbMsgId.RspGetCMParamContext,
  'name': 'RspGetCMParamContext',
}
##

## ReqSetCMParamNorm Message Definition 
cbMsgDefReqSetCMParamNorm = {
  'fielddef': [
    {'fid':1, 'ftype':'B', 'max_count':1, 'name':'norm', },
  ],
  'ftype': '{',
  'max_count': 1,
  'msgid': cbMsgId.ReqSetCMParamNorm,
  'name': 'ReqSetCMParamNorm',
}
##

## ReqGetCMParamNorm Message Definition 
cbMsgDefReqGetCMParamNorm = {
  'fielddef': [  ],
  'ftype': '{',
  'max_count': 0,
  'msgid': cbMsgId.ReqGetCMParamNorm,
  'name': 'ReqGetCMParamNorm',
}
##

## RspGetCMParamNorm Message Definition 
cbMsgDefRspGetCMParamNorm = {
  'fielddef': [
    {'fid':1, 'ftype':'B', 'max_count':1, 'name':'norm', },
  ],
  'ftype': '{',
  'max_count': 1,
  'msgid': cbMsgId.RspGetCMParamNorm,
  'name': 'RspGetCMParamNorm',
}
##

## ReqSetCMParamMinIF Message Definition 
cbMsgDefReqSetCMParamMinIF = {
  'fielddef': [
    {'fid':1, 'ftype':'H', 'max_count':1, 'name':'minif', },
  ],
  'ftype': '{',
  'max_count': 1,
  'msgid': cbMsgId.ReqSetCMParamMinIF,
  'name': 'ReqSetCMParamMinIF',
}
##

## ReqGetCMParamMinIF Message Definition 
cbMsgDefReqGetCMParamMinIF = {
  'fielddef': [  ],
  'ftype': '{',
  'max_count': 0,
  'msgid': cbMsgId.ReqGetCMParamMinIF,
  'name': 'ReqGetCMParamMinIF',
}
##

## RspGetCMParamMinIF Message Definition 
cbMsgDefRspGetCMParamMinIF = {
  'fielddef': [
    {'fid':1, 'ftype':'H', 'max_count':1, 'name':'minif', },
  ],
  'ftype': '{',
  'max_count': 1,
  'msgid': cbMsgId.RspGetCMParamMinIF,
  'name': 'RspGetCMParamMinIF',
}
##

## ReqSetCMParamMaxIF Message Definition 
cbMsgDefReqSetCMParamMaxIF = {
  'fielddef': [
    {'fid':1, 'ftype':'H', 'max_count':1, 'name':'maxif', },
  ],
  'ftype': '{',
  'max_count': 1,
  'msgid': cbMsgId.ReqSetCMParamMaxIF,
  'name': 'ReqSetCMParamMaxIF',
}
##

## ReqGetCMParamMaxIF Message Definition 
cbMsgDefReqGetCMParamMaxIF = {
  'fielddef': [  ],
  'ftype': '{',
  'max_count': 0,
  'msgid': cbMsgId.ReqGetCMParamMaxIF,
  'name': 'ReqGetCMParamMaxIF',
}
##

## RspGetCMParamMaxIF Message Definition 
cbMsgDefRspGetCMParamMaxIF = {
  'fielddef': [
    {'fid':1, 'ftype':'H', 'max_count':1, 'name':'maxif', },
  ],
  'ftype': '{',
  'max_count': 1,
  'msgid': cbMsgId.RspGetCMParamMaxIF,
  'name': 'RspGetCMParamMaxIF',
}
##

## ReqSetCMParamMaxClassified Message Definition 
cbMsgDefReqSetCMParamMaxClassified = {
  'fielddef': [
    {'fid':1, 'ftype':'B', 'max_count':1, 'name':'max', },
  ],
  'ftype': '{',
  'max_count': 1,
  'msgid': cbMsgId.ReqSetCMParamMaxClassified,
  'name': 'ReqSetCMParamMaxClassified',
}
##

## ReqGetCMParamMaxClassified Message Definition 
cbMsgDefReqGetCMParamMaxClassified = {
  'fielddef': [  ],
  'ftype': '{',
  'max_count': 0,
  'msgid': cbMsgId.ReqGetCMParamMaxClassified,
  'name': 'ReqGetCMParamMaxClassified',
}
##

## RspGetCMParamMaxClassified Message Definition 
cbMsgDefRspGetCMParamMaxClassified = {
  'fielddef': [
    {'fid':1, 'ftype':'B', 'max_count':1, 'name':'max', },
  ],
  'ftype': '{',
  'max_count': 1,
  'msgid': cbMsgId.RspGetCMParamMaxClassified,
  'name': 'RspGetCMParamMaxClassified',
}
##

## ReqSetNNParamLabel Message Definition 
cbMsgDefReqSetNNParamLabel = {
  'fielddef': [
    {
      'fid': 1,
      'ftype': 's',
      'max_count': CB_PARAM_NN_LABEL_LEN_MAX,
      'name': 'label',
    },
  ],
  'ftype': '{',
  'max_count': 1,
  'msgid': cbMsgId.ReqSetNNParamLabel,
  'name': 'ReqSetNNParamLabel',
}
##

## ReqGetNNParamLabel Message Definition 
cbMsgDefReqGetNNParamLabel = {
  'fielddef': [  ],
  'ftype': '{',
  'max_count': 0,
  'msgid': cbMsgId.ReqGetNNParamLabel,
  'name': 'ReqGetNNParamLabel',
}
##

## RspGetNNParamLabel Message Definition 
cbMsgDefRspGetNNParamLabel = {
  'fielddef': [
    {
      'fid': 1,
      'ftype': 's',
      'max_count': CB_PARAM_NN_LABEL_LEN_MAX,
      'name': 'label',
    },
  ],
  'ftype': '{',
  'max_count': 1,
  'msgid': cbMsgId.RspGetNNParamLabel,
  'name': 'RspGetNNParamLabel',
}
##

## ReqNVMemReset Message Definition 
cbMsgDefReqNVMemReset = {
  'fielddef': [  ],
  'ftype': '{',
  'max_count': 0,
  'msgid': cbMsgId.ReqNVMemReset,
  'name': 'ReqNVMemReset',
}
##

## ReqNVMemSave Message Definition 
cbMsgDefReqNVMemSave = {
  'fielddef': [  ],
  'ftype': '{',
  'max_count': 0,
  'msgid': cbMsgId.ReqNVMemSave,
  'name': 'ReqNVMemSave',
}
##

## ReqNVMemRestore Message Definition 
cbMsgDefReqNVMemRestore = {
  'fielddef': [  ],
  'ftype': '{',
  'max_count': 0,
  'msgid': cbMsgId.ReqNVMemRestore,
  'name': 'ReqNVMemRestore',
}
##

## ReqNVMemGetNNCount Message Definition 
cbMsgDefReqNVMemGetNNCount = {
  'fielddef': [  ],
  'ftype': '{',
  'max_count': 0,
  'msgid': cbMsgId.ReqNVMemGetNNCount,
  'name': 'ReqNVMemGetNNCount',
}
##

## RspNVMemGetNNCount Message Definition 
cbMsgDefRspNVMemGetNNCount = {
  'fielddef': [
    {'fid':1, 'ftype':'H', 'max_count':1, 'name':'count', },
  ],
  'ftype': '{',
  'max_count': 1,
  'msgid': cbMsgId.RspNVMemGetNNCount,
  'name': 'RspNVMemGetNNCount',
}
##

## ReqNNTrain Message Definition 
cbMsgDefReqNNTrain = {
  'fielddef': [
    {'fid':1, 'ftype':'H', 'max_count':1, 'name':'category', },
    {
      'fid': 2,
      'ftype': '[',
      'max_count': CM_PATTERN_COMP_NUMOF,
      'name': 'pattern',
      'vdef': {
        'fid': nmBase.NMFIdNone,
        'ftype': 'B',
        'max_count': 1,
        'name': 'pattern_item',
      },
    },
  ],
  'ftype': '{',
  'max_count': 2,
  'msgid': cbMsgId.ReqNNTrain,
  'name': 'ReqNNTrain',
}
##

## RspNNTrain Message Definition 
cbMsgDefRspNNTrain = {
  'fielddef': [
    {'fid':1, 'ftype':'H', 'max_count':1, 'name':'neuronId', },
  ],
  'ftype': '{',
  'max_count': 1,
  'msgid': cbMsgId.RspNNTrain,
  'name': 'RspNNTrain',
}
##

## ReqNNCatergorize Message Definition 
cbMsgDefReqNNCatergorize = {
  'fielddef': [
    {
      'fid': 1,
      'ftype': '[',
      'max_count': CM_PATTERN_COMP_NUMOF,
      'name': 'pattern',
      'vdef': {
        'fid': nmBase.NMFIdNone,
        'ftype': 'B',
        'max_count': 1,
        'name': 'pattern_item',
      },
    },
  ],
  'ftype': '{',
  'max_count': 1,
  'msgid': cbMsgId.ReqNNCatergorize,
  'name': 'ReqNNCatergorize',
}
##

## RspNNCatergorize Message Definition 
cbMsgDefRspNNCatergorize = {
  'fielddef': [
    {
      'fid': 1,
      'ftype': '[',
      'max_count': CB_PARAM_MAX_CLASSIFIED_MAX,
      'name': 'classified',
      'vdef': {
        'fid': nmBase.NMFIdNone,
        'ftype': '{',
        'max_count': 1,
        'msgdef': cbExtFTypeClassification,
        'name': 'classified_item',
      },
    },
  ],
  'ftype': '{',
  'max_count': 1,
  'msgid': cbMsgId.RspNNCatergorize,
  'name': 'RspNNCatergorize',
}
##

## ReqNNForget Message Definition 
cbMsgDefReqNNForget = {
  'fielddef': [  ],
  'ftype': '{',
  'max_count': 0,
  'msgid': cbMsgId.ReqNNForget,
  'name': 'ReqNNForget',
}
##

## ReqNNGetCount Message Definition 
cbMsgDefReqNNGetCount = {
  'fielddef': [  ],
  'ftype': '{',
  'max_count': 0,
  'msgid': cbMsgId.ReqNNGetCount,
  'name': 'ReqNNGetCount',
}
##

## RspNNGetCount Message Definition 
cbMsgDefRspNNGetCount = {
  'fielddef': [
    {'fid':1, 'ftype':'H', 'max_count':1, 'name':'count', },
  ],
  'ftype': '{',
  'max_count': 1,
  'msgid': cbMsgId.RspNNGetCount,
  'name': 'RspNNGetCount',
}
##

## ReqCMReadReg Message Definition 
cbMsgDefReqCMReadReg = {
  'fielddef': [
    {'fid':1, 'ftype':'B', 'max_count':1, 'name':'addr', },
  ],
  'ftype': '{',
  'max_count': 1,
  'msgid': cbMsgId.ReqCMReadReg,
  'name': 'ReqCMReadReg',
}
##

## RspCMReadReg Message Definition 
cbMsgDefRspCMReadReg = {
  'fielddef': [
    {'fid':1, 'ftype':'B', 'max_count':1, 'name':'addr', },
    {'fid':2, 'ftype':'H', 'max_count':1, 'name':'value', },
  ],
  'ftype': '{',
  'max_count': 2,
  'msgid': cbMsgId.RspCMReadReg,
  'name': 'RspCMReadReg',
}
##

## ReqCMWriteReg Message Definition 
cbMsgDefReqCMWriteReg = {
  'fielddef': [
    {'fid':1, 'ftype':'B', 'max_count':1, 'name':'addr', },
    {'fid':2, 'ftype':'H', 'max_count':1, 'name':'value', },
  ],
  'ftype': '{',
  'max_count': 2,
  'msgid': cbMsgId.ReqCMWriteReg,
  'name': 'ReqCMWriteReg',
}
##

## ReqUploadParams Message Definition 
cbMsgDefReqUploadParams = {
  'fielddef': [
    {'fid':1, 'ftype':'B', 'max_count':1, 'name':'dst', },
    {
      'fid': 2,
      'ftype': '{',
      'max_count': 1,
      'msgdef': cbExtFTypeCogniBoostParams,
      'name': 'params',
    },
  ],
  'ftype': '{',
  'max_count': 2,
  'msgid': cbMsgId.ReqUploadParams,
  'name': 'ReqUploadParams',
}
##

## ReqDownloadParams Message Definition 
cbMsgDefReqDownloadParams = {
  'fielddef': [
    {'fid':1, 'ftype':'B', 'max_count':1, 'name':'src', },
  ],
  'ftype': '{',
  'max_count': 1,
  'msgid': cbMsgId.ReqDownloadParams,
  'name': 'ReqDownloadParams',
}
##

## RspDownloadParams Message Definition 
cbMsgDefRspDownloadParams = {
  'fielddef': [
    {
      'fid': 1,
      'ftype': '{',
      'max_count': 1,
      'msgdef': cbExtFTypeCogniBoostParams,
      'name': 'params',
    },
  ],
  'ftype': '{',
  'max_count': 1,
  'msgid': cbMsgId.RspDownloadParams,
  'name': 'RspDownloadParams',
}
##

## ReqUploadNNSetStart Message Definition 
cbMsgDefReqUploadNNSetStart = {
  'fielddef': [
    {'fid':1, 'ftype':'B', 'max_count':1, 'name':'dst', },
    {'fid':2, 'ftype':'H', 'max_count':1, 'name':'numNeurons', },
    {
      'fid': 3,
      'ftype': '{',
      'max_count': 1,
      'msgdef': cbExtFTypeNeuron,
      'name': 'neuron',
    },
  ],
  'ftype': '{',
  'max_count': 3,
  'msgid': cbMsgId.ReqUploadNNSetStart,
  'name': 'ReqUploadNNSetStart',
}
##

## ReqUploadNNSetNext Message Definition 
cbMsgDefReqUploadNNSetNext = {
  'fielddef': [
    {
      'fid': 1,
      'ftype': '{',
      'max_count': 1,
      'msgdef': cbExtFTypeNeuron,
      'name': 'neuron',
    },
  ],
  'ftype': '{',
  'max_count': 1,
  'msgid': cbMsgId.ReqUploadNNSetNext,
  'name': 'ReqUploadNNSetNext',
}
##

## ReqDownloadNNSetStart Message Definition 
cbMsgDefReqDownloadNNSetStart = {
  'fielddef': [
    {'fid':1, 'ftype':'B', 'max_count':1, 'name':'src', },
  ],
  'ftype': '{',
  'max_count': 1,
  'msgid': cbMsgId.ReqDownloadNNSetStart,
  'name': 'ReqDownloadNNSetStart',
}
##

## RspDownloadNNSetStart Message Definition 
cbMsgDefRspDownloadNNSetStart = {
  'fielddef': [
    {'fid':1, 'ftype':'H', 'max_count':1, 'name':'numNeurons', },
    {
      'fid': 2,
      'ftype': '{',
      'max_count': 1,
      'msgdef': cbExtFTypeNeuron,
      'name': 'neuron',
    },
  ],
  'ftype': '{',
  'max_count': 2,
  'msgid': cbMsgId.RspDownloadNNSetStart,
  'name': 'RspDownloadNNSetStart',
}
##

## ReqDownloadNNSetNext Message Definition 
cbMsgDefReqDownloadNNSetNext = {
  'fielddef': [  ],
  'ftype': '{',
  'max_count': 0,
  'msgid': cbMsgId.ReqDownloadNNSetNext,
  'name': 'ReqDownloadNNSetNext',
}
##

## RspDownloadNNSetNext Message Definition 
cbMsgDefRspDownloadNNSetNext = {
  'fielddef': [
    {
      'fid': 1,
      'ftype': '{',
      'max_count': 1,
      'msgdef': cbExtFTypeNeuron,
      'name': 'neuron',
    },
  ],
  'ftype': '{',
  'max_count': 1,
  'msgid': cbMsgId.RspDownloadNNSetNext,
  'name': 'RspDownloadNNSetNext',
}
##

## ReqXferNNSetStop Message Definition 
cbMsgDefReqXferNNSetStop = {
  'fielddef': [
    {'fid':1, 'ftype':'?', 'max_count':1, 'name':'abort', },
  ],
  'ftype': '{',
  'max_count': 1,
  'msgid': cbMsgId.ReqXferNNSetStop,
  'name': 'ReqXferNNSetStop',
}
##

## cb Message Definition Set Dictionary
cbSetMsgDef = {
  cbMsgId.ReqCMReadReg: cbMsgDefReqCMReadReg,
  cbMsgId.ReqCMWriteReg: cbMsgDefReqCMWriteReg,
  cbMsgId.ReqDownloadNNSetNext: cbMsgDefReqDownloadNNSetNext,
  cbMsgId.ReqDownloadNNSetStart: cbMsgDefReqDownloadNNSetStart,
  cbMsgId.ReqDownloadParams: cbMsgDefReqDownloadParams,
  cbMsgId.ReqGetCMParamClassifier: cbMsgDefReqGetCMParamClassifier,
  cbMsgId.ReqGetCMParamContext: cbMsgDefReqGetCMParamContext,
  cbMsgId.ReqGetCMParamMaxClassified: cbMsgDefReqGetCMParamMaxClassified,
  cbMsgId.ReqGetCMParamMaxIF: cbMsgDefReqGetCMParamMaxIF,
  cbMsgId.ReqGetCMParamMinIF: cbMsgDefReqGetCMParamMinIF,
  cbMsgId.ReqGetCMParamNorm: cbMsgDefReqGetCMParamNorm,
  cbMsgId.ReqGetNNParamLabel: cbMsgDefReqGetNNParamLabel,
  cbMsgId.ReqGetOpParamAutoRestore: cbMsgDefReqGetOpParamAutoRestore,
  cbMsgId.ReqGetOpParamAutoSleep: cbMsgDefReqGetOpParamAutoSleep,
  cbMsgId.ReqGetOpParamBaudRate: cbMsgDefReqGetOpParamBaudRate,
  cbMsgId.ReqGetOpParamLedBling: cbMsgDefReqGetOpParamLedBling,
  cbMsgId.ReqLoopback: cbMsgDefReqLoopback,
  cbMsgId.ReqNNCatergorize: cbMsgDefReqNNCatergorize,
  cbMsgId.ReqNNForget: cbMsgDefReqNNForget,
  cbMsgId.ReqNNGetCount: cbMsgDefReqNNGetCount,
  cbMsgId.ReqNNTrain: cbMsgDefReqNNTrain,
  cbMsgId.ReqNVMemGetNNCount: cbMsgDefReqNVMemGetNNCount,
  cbMsgId.ReqNVMemReset: cbMsgDefReqNVMemReset,
  cbMsgId.ReqNVMemRestore: cbMsgDefReqNVMemRestore,
  cbMsgId.ReqNVMemSave: cbMsgDefReqNVMemSave,
  cbMsgId.ReqPing: cbMsgDefReqPing,
  cbMsgId.ReqReboot: cbMsgDefReqReboot,
  cbMsgId.ReqSetCMParamClassifier: cbMsgDefReqSetCMParamClassifier,
  cbMsgId.ReqSetCMParamContext: cbMsgDefReqSetCMParamContext,
  cbMsgId.ReqSetCMParamMaxClassified: cbMsgDefReqSetCMParamMaxClassified,
  cbMsgId.ReqSetCMParamMaxIF: cbMsgDefReqSetCMParamMaxIF,
  cbMsgId.ReqSetCMParamMinIF: cbMsgDefReqSetCMParamMinIF,
  cbMsgId.ReqSetCMParamNorm: cbMsgDefReqSetCMParamNorm,
  cbMsgId.ReqSetNNParamLabel: cbMsgDefReqSetNNParamLabel,
  cbMsgId.ReqSetOpParamAutoRestore: cbMsgDefReqSetOpParamAutoRestore,
  cbMsgId.ReqSetOpParamAutoSleep: cbMsgDefReqSetOpParamAutoSleep,
  cbMsgId.ReqSetOpParamBaudRate: cbMsgDefReqSetOpParamBaudRate,
  cbMsgId.ReqSetOpParamLedBling: cbMsgDefReqSetOpParamLedBling,
  cbMsgId.ReqSleep: cbMsgDefReqSleep,
  cbMsgId.ReqUploadNNSetNext: cbMsgDefReqUploadNNSetNext,
  cbMsgId.ReqUploadNNSetStart: cbMsgDefReqUploadNNSetStart,
  cbMsgId.ReqUploadParams: cbMsgDefReqUploadParams,
  cbMsgId.ReqVersion: cbMsgDefReqVersion,
  cbMsgId.ReqWakeUp: cbMsgDefReqWakeUp,
  cbMsgId.ReqXferNNSetStop: cbMsgDefReqXferNNSetStop,
  cbMsgId.RspCMReadReg: cbMsgDefRspCMReadReg,
  cbMsgId.RspDownloadNNSetNext: cbMsgDefRspDownloadNNSetNext,
  cbMsgId.RspDownloadNNSetStart: cbMsgDefRspDownloadNNSetStart,
  cbMsgId.RspDownloadParams: cbMsgDefRspDownloadParams,
  cbMsgId.RspErr: cbMsgDefRspErr,
  cbMsgId.RspGetCMParamClassifier: cbMsgDefRspGetCMParamClassifier,
  cbMsgId.RspGetCMParamContext: cbMsgDefRspGetCMParamContext,
  cbMsgId.RspGetCMParamMaxClassified: cbMsgDefRspGetCMParamMaxClassified,
  cbMsgId.RspGetCMParamMaxIF: cbMsgDefRspGetCMParamMaxIF,
  cbMsgId.RspGetCMParamMinIF: cbMsgDefRspGetCMParamMinIF,
  cbMsgId.RspGetCMParamNorm: cbMsgDefRspGetCMParamNorm,
  cbMsgId.RspGetNNParamLabel: cbMsgDefRspGetNNParamLabel,
  cbMsgId.RspGetOpParamAutoRestore: cbMsgDefRspGetOpParamAutoRestore,
  cbMsgId.RspGetOpParamAutoSleep: cbMsgDefRspGetOpParamAutoSleep,
  cbMsgId.RspGetOpParamBaudRate: cbMsgDefRspGetOpParamBaudRate,
  cbMsgId.RspGetOpParamLedBling: cbMsgDefRspGetOpParamLedBling,
  cbMsgId.RspLoopback: cbMsgDefRspLoopback,
  cbMsgId.RspNNCatergorize: cbMsgDefRspNNCatergorize,
  cbMsgId.RspNNGetCount: cbMsgDefRspNNGetCount,
  cbMsgId.RspNNTrain: cbMsgDefRspNNTrain,
  cbMsgId.RspNVMemGetNNCount: cbMsgDefRspNVMemGetNNCount,
  cbMsgId.RspOk: cbMsgDefRspOk,
  cbMsgId.RspVersion: cbMsgDefRspVersion,
}
##


# -----------------------------------------------------------------------------
# CLASS: cbNetMsgs
# -----------------------------------------------------------------------------

class cbNetMsgs(NetMsgsITV):
  """ cb NetMsgs Class. """

  #--
  def __init__(self, **kwargs):
    """ cb NetMsgs initialization. """
    kwargs['msgsetname'] = 'cbMsgSet'
    NetMsgsITV.__init__(self, cbSetMsgDef, **kwargs)
  ##
##
