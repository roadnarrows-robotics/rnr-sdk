##############################################################################
#
# File: bsDynaMsgs.py
#

"""
bsDynaMsgs.py
"""

## \file
##
## \brief \h_botsense Server/Client Dynamixel NetMsgs XML Definitions.
##
## \warning This file was auto-generated on 2016.05.02 14:14:54 from the NetMsgs
## XML specification bsDynaMsgs.xml.
##
## \par Copyright:
## (C) 2016. RoadNarrows LLC
## (http://www.roadnarrows.com)
## All Rights Reserved
##
##############################################################################


import NetMsgs.NetMsgsBase as nmBase
from NetMsgs.NetMsgsLibITV import NetMsgsITV


# -----------------------------------------------------------------------------
# BsDyna Message Id Enumeration
# -----------------------------------------------------------------------------

class BsDynaMsgId:
  """ BsDyna Message Id Enumeration class. """
  NoId                            = 0     # no message id
  ReqOpenArgs                     = 1     # ReqOpenArgs
  ReqSetBaudRate                  = 2     # ReqSetBaudRate
  ReqRead8                        = 3     # ReqRead8
  RspRead8                        = 4     # RspRead8
  ReqRead16                       = 5     # ReqRead16
  RspRead16                       = 6     # RspRead16
  ReqWrite8                       = 7     # ReqWrite8
  RspWrite8                       = 8     # RspWrite8
  ReqWrite16                      = 9     # ReqWrite16
  RspWrite16                      = 10    # RspWrite16
  ReqSyncWrite                    = 11    # ReqSyncWrite
  ReqPing                         = 12    # ReqPing
  RspPing                         = 13    # RspPing
  ReqReset                        = 14    # ReqReset
  ReqSetHalfDuplexCtl             = 15    # ReqSetHalfDuplexCtl
  NumOf                           = 16    # number of message ids
##


# -----------------------------------------------------------------------------
# Extended Field Types
# -----------------------------------------------------------------------------

## WriteTuple Extended Field Type 
BsDynaExtFTypeWriteTuple = {
  'fielddef': [
    {'fid':1, 'ftype':'B', 'max_count':1, 'name':'servo_id', },
    {'fid':2, 'ftype':'H', 'max_count':1, 'name':'val', },
  ],
  'ftype': '{',
  'max_count': 2,
  'msgid': BsDynaMsgId.NoId,
  'name': 'WriteTuple',
}
##

## BsDyna Extended Field Type Dictionary
BsDynaExtFieldTypes = {'WriteTuple':BsDynaExtFTypeWriteTuple, }
##


# -----------------------------------------------------------------------------
# Message Definition Set
# -----------------------------------------------------------------------------

## ReqOpenArgs Message Definition 
BsDynaMsgDefReqOpenArgs = {
  'fielddef': [
    {'fid':1, 'ftype':'I', 'max_count':1, 'name':'baudrate', },
  ],
  'ftype': '{',
  'max_count': 1,
  'msgid': BsDynaMsgId.ReqOpenArgs,
  'name': 'ReqOpenArgs',
}
##

## ReqSetBaudRate Message Definition 
BsDynaMsgDefReqSetBaudRate = {
  'fielddef': [
    {'fid':1, 'ftype':'I', 'max_count':1, 'name':'baudrate', },
  ],
  'ftype': '{',
  'max_count': 1,
  'msgid': BsDynaMsgId.ReqSetBaudRate,
  'name': 'ReqSetBaudRate',
}
##

## ReqRead8 Message Definition 
BsDynaMsgDefReqRead8 = {
  'fielddef': [
    {'fid':1, 'ftype':'B', 'max_count':1, 'name':'servo_id', },
    {'fid':2, 'ftype':'B', 'max_count':1, 'name':'addr', },
  ],
  'ftype': '{',
  'max_count': 2,
  'msgid': BsDynaMsgId.ReqRead8,
  'name': 'ReqRead8',
}
##

## RspRead8 Message Definition 
BsDynaMsgDefRspRead8 = {
  'fielddef': [
    {'fid':1, 'ftype':'B', 'max_count':1, 'name':'alarms', },
    {'fid':2, 'ftype':'B', 'max_count':1, 'name':'val', },
  ],
  'ftype': '{',
  'max_count': 2,
  'msgid': BsDynaMsgId.RspRead8,
  'name': 'RspRead8',
}
##

## ReqRead16 Message Definition 
BsDynaMsgDefReqRead16 = {
  'fielddef': [
    {'fid':1, 'ftype':'B', 'max_count':1, 'name':'servo_id', },
    {'fid':2, 'ftype':'B', 'max_count':1, 'name':'addr', },
  ],
  'ftype': '{',
  'max_count': 2,
  'msgid': BsDynaMsgId.ReqRead16,
  'name': 'ReqRead16',
}
##

## RspRead16 Message Definition 
BsDynaMsgDefRspRead16 = {
  'fielddef': [
    {'fid':1, 'ftype':'B', 'max_count':1, 'name':'alarms', },
    {'fid':2, 'ftype':'H', 'max_count':1, 'name':'val', },
  ],
  'ftype': '{',
  'max_count': 2,
  'msgid': BsDynaMsgId.RspRead16,
  'name': 'RspRead16',
}
##

## ReqWrite8 Message Definition 
BsDynaMsgDefReqWrite8 = {
  'fielddef': [
    {'fid':1, 'ftype':'B', 'max_count':1, 'name':'servo_id', },
    {'fid':2, 'ftype':'B', 'max_count':1, 'name':'addr', },
    {'fid':3, 'ftype':'B', 'max_count':1, 'name':'val', },
  ],
  'ftype': '{',
  'max_count': 3,
  'msgid': BsDynaMsgId.ReqWrite8,
  'name': 'ReqWrite8',
}
##

## RspWrite8 Message Definition 
BsDynaMsgDefRspWrite8 = {
  'fielddef': [
    {'fid':1, 'ftype':'B', 'max_count':1, 'name':'alarms', },
  ],
  'ftype': '{',
  'max_count': 1,
  'msgid': BsDynaMsgId.RspWrite8,
  'name': 'RspWrite8',
}
##

## ReqWrite16 Message Definition 
BsDynaMsgDefReqWrite16 = {
  'fielddef': [
    {'fid':1, 'ftype':'B', 'max_count':1, 'name':'servo_id', },
    {'fid':2, 'ftype':'B', 'max_count':1, 'name':'addr', },
    {'fid':3, 'ftype':'H', 'max_count':1, 'name':'val', },
  ],
  'ftype': '{',
  'max_count': 3,
  'msgid': BsDynaMsgId.ReqWrite16,
  'name': 'ReqWrite16',
}
##

## RspWrite16 Message Definition 
BsDynaMsgDefRspWrite16 = {
  'fielddef': [
    {'fid':1, 'ftype':'B', 'max_count':1, 'name':'alarms', },
  ],
  'ftype': '{',
  'max_count': 1,
  'msgid': BsDynaMsgId.RspWrite16,
  'name': 'RspWrite16',
}
##

## ReqSyncWrite Message Definition 
BsDynaMsgDefReqSyncWrite = {
  'fielddef': [
    {'fid':1, 'ftype':'B', 'max_count':1, 'name':'addr', },
    {'fid':2, 'ftype':'B', 'max_count':1, 'name':'data_size', },
    {
      'fid': 3,
      'ftype': '[',
      'max_count': DYNA_ID_NUMOF,
      'name': 'tuples',
      'vdef': {
        'fid': nmBase.NMFIdNone,
        'ftype': '{',
        'max_count': 1,
        'msgdef': BsDynaExtFTypeWriteTuple,
        'name': 'tuples_item',
      },
    },
  ],
  'ftype': '{',
  'max_count': 3,
  'msgid': BsDynaMsgId.ReqSyncWrite,
  'name': 'ReqSyncWrite',
}
##

## ReqPing Message Definition 
BsDynaMsgDefReqPing = {
  'fielddef': [
    {'fid':1, 'ftype':'B', 'max_count':1, 'name':'servo_id', },
  ],
  'ftype': '{',
  'max_count': 1,
  'msgid': BsDynaMsgId.ReqPing,
  'name': 'ReqPing',
}
##

## RspPing Message Definition 
BsDynaMsgDefRspPing = {
  'fielddef': [
    {'fid':1, 'ftype':'?', 'max_count':1, 'name':'pong', },
  ],
  'ftype': '{',
  'max_count': 1,
  'msgid': BsDynaMsgId.RspPing,
  'name': 'RspPing',
}
##

## ReqReset Message Definition 
BsDynaMsgDefReqReset = {
  'fielddef': [
    {'fid':1, 'ftype':'B', 'max_count':1, 'name':'servo_id', },
  ],
  'ftype': '{',
  'max_count': 1,
  'msgid': BsDynaMsgId.ReqReset,
  'name': 'ReqReset',
}
##

## ReqSetHalfDuplexCtl Message Definition 
BsDynaMsgDefReqSetHalfDuplexCtl = {
  'fielddef': [
    {'fid':1, 'ftype':'I', 'max_count':1, 'name':'signal', },
  ],
  'ftype': '{',
  'max_count': 1,
  'msgid': BsDynaMsgId.ReqSetHalfDuplexCtl,
  'name': 'ReqSetHalfDuplexCtl',
}
##

## BsDyna Message Definition Set Dictionary
BsDynaSetMsgDef = {
  BsDynaMsgId.ReqOpenArgs: BsDynaMsgDefReqOpenArgs,
  BsDynaMsgId.ReqPing: BsDynaMsgDefReqPing,
  BsDynaMsgId.ReqRead16: BsDynaMsgDefReqRead16,
  BsDynaMsgId.ReqRead8: BsDynaMsgDefReqRead8,
  BsDynaMsgId.ReqReset: BsDynaMsgDefReqReset,
  BsDynaMsgId.ReqSetBaudRate: BsDynaMsgDefReqSetBaudRate,
  BsDynaMsgId.ReqSetHalfDuplexCtl: BsDynaMsgDefReqSetHalfDuplexCtl,
  BsDynaMsgId.ReqSyncWrite: BsDynaMsgDefReqSyncWrite,
  BsDynaMsgId.ReqWrite16: BsDynaMsgDefReqWrite16,
  BsDynaMsgId.ReqWrite8: BsDynaMsgDefReqWrite8,
  BsDynaMsgId.RspPing: BsDynaMsgDefRspPing,
  BsDynaMsgId.RspRead16: BsDynaMsgDefRspRead16,
  BsDynaMsgId.RspRead8: BsDynaMsgDefRspRead8,
  BsDynaMsgId.RspWrite16: BsDynaMsgDefRspWrite16,
  BsDynaMsgId.RspWrite8: BsDynaMsgDefRspWrite8,
}
##


# -----------------------------------------------------------------------------
# CLASS: BsDynaNetMsgs
# -----------------------------------------------------------------------------

class BsDynaNetMsgs(NetMsgsITV):
  """ BsDyna NetMsgs Class. """

  #--
  def __init__(self, **kwargs):
    """ BsDyna NetMsgs initialization. """
    kwargs['msgsetname'] = 'BsDynaMsgSet'
    NetMsgsITV.__init__(self, BsDynaSetMsgDef, **kwargs)
  ##
##
