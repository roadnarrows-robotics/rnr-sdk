#
# File: testitv.py
#
# Test NetMsgsITV class
#
# Copyright:
#   (C) 2010.  RoadNarrows LLC.
#   (http://www.roadnarrows.com)
#   All Rights Reserved
#

import sys

import NetMsgs.NetMsgsBase as nmBase
from NetMsgs.NetMsgsLibITV import NetMsgsITV

Struct2MsgDef = {
  'name':   'StructMyStuff',
  'msgid':  0,
  'fielddef': [
    { 'name':   'Proud',
      'fid':    1,
      'ftype':  'Q',
    },
    { 'name':   'Peacock',
      'fid':    2,
      'ftype':  '?',
    },
  ]
}

#--
class UTNetMsgs(NetMsgsITV):
  def __init__(self, **kwargs):
    NetMsgsITV.__init__(self, {}, **kwargs)

    self.mMsgDefSet = UTMsgSet = {
      1: {
        'name':     'TstOogaMsg',
        'msgid':    1,
        'fielddef': [
          { 'name':   'Field1',
            'fid':    1,
            'ftype':  'B',
          },
          { 'name':   'Field2',
            'fid':    2,
            'ftype':  'i',
            'min':    -9,
            'max':    909,
          },
          { 'name':   'Field3',
            'fid':    3,
            'ftype':  's',
            'max_count':  12,
          },
          { 'name':   'Vec1',
            'fid':    4,
            'ftype':  '[',
            'max_count': 4,
            'vdef': {
              'name':   'Vec1_item',
              'fid':    0,
              'ftype':  'I'
            }
          },
          { 'name':   'pad_5',
            'fid':    5,
            'ftype':  'x',
            'count':  2,
          },
          { 'name':   'Vec2',
            'fid':    6,
            'ftype':  '[',
            'max_count': 3,
            'vdef': {
              'name':   'Vec2_item',
              'fid':    0,
              'ftype':  '{',
              'msgdef': Struct2MsgDef,
            }
          },
          { 'name':   'Struct1',
            'fid':    7,
            'ftype':  '{',
            'msgdef': {
              'name': 'Struct1_struct',
              'msgid':  0,
              'fielddef': [
                { 'name':   'Ooga',
                  'fid':    1,
                  'ftype':  's',
                  'max_count':  8,
                  'const': 'goodbye',
                },
                { 'name':   'Pi',
                  'fid':    2,
                  'ftype':  'f',
                },
                { 'name':   'Chi',
                  'fid':    3,
                  'ftype':  '{',
                  'msgdef': Struct2MsgDef,
                },
              ]
            }
          },
        ]
      },
    }

    self.Optimize()

##

ut = UTNetMsgs(trace=True)

txvals_struct1 = {
    'Pi': 3.14159,
    'Chi':{'Proud': 0xffeeddccbbaa, 'Peacock': True},
}

txvals = {
    'Field1': 0x32,
    'Field2': -1,
    'Field3': 'hello',
    'Vec1':   [1, 101, 1001],
    'Vec2':   [{'Proud': 0, 'Peacock': False}, {'Proud': 1, 'Peacock': True}],
    'Struct1': txvals_struct1,
}

nmBase.PrettyPrintAssignExpr('txvals', txvals, fp=sys.stderr)

try:
  buf = ut.nmPackMsg(1, txvals)
except nmBase.NetMsgsError, inst:
  print >>sys.stderr, "Error:", inst
  sys.exit(8)

rxvals = {}

try:
  n = ut.nmUnpackMsg(1, buf, rxvals)
except nmBase.NetMsgsError, inst:
  print >>sys.stderr, "Error:", inst
  sys.exit(8)

print >>sys.stderr
nmBase.PrettyPrintAssignExpr('rxvals', rxvals, fp=sys.stderr)
