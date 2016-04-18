#
# File: testlib.py
#
# Test NetMsgs run-time library core functions.
#
# Copyright:
#   (C) 2010.  RoadNarrows LLC.
#   (http://www.roadnarrows.com)
#   All Rights Reserved
#

import sys

from NetMsgs.NetMsgsLib import *

endian = 'little'

tval = True
buf = PackBool(tval, endian=endian)
rval, offset = UnpackBool(buf, endian=endian)
print 'bool:', tval, rval

tval = 'm'
buf = PackChar(tval, endian=endian)
rval, offset = UnpackChar(buf, endian=endian)
print 'char:', tval, rval

tval = 5
buf = PackU8(tval, endian=endian)
rval, offset = UnpackU8(buf, endian=endian)
print 'u8:', tval, rval

tval = -5
buf = PackS8(tval, endian=endian)
rval, offset = UnpackS8(buf, endian=endian)
print 's8:', tval, rval

tval = 505
buf = PackU16(tval, endian=endian)
rval, offset = UnpackU16(buf, endian=endian)
print 'u16:', tval, rval

tval = -505
buf = PackS16(tval, endian=endian)
rval, offset = UnpackS16(buf, endian=endian)
print 's16:', tval, rval

tval = 505000
buf = PackU32(tval, endian=endian)
rval, offset = UnpackU32(buf, endian=endian)
print 'u32:', tval, rval

tval = -505000
buf = PackS32(tval, endian=endian)
rval, offset = UnpackS32(buf, endian=endian)
print 's32:', tval, rval

tval = 5600700500
buf = PackU64(tval, endian=endian)
rval, offset = UnpackU64(buf, endian=endian)
print 'u64:', tval, rval

tval = -5600700500
buf = PackS64(tval, endian=endian)
rval, offset = UnpackS64(buf, endian=endian)
print 's64:', tval, rval

tval = 53.006
buf = PackF32(tval, endian=endian)
rval, offset = UnpackF32(buf, endian=endian)
print 'f32:', tval, rval

tval = -53.006e21
buf = PackF64(tval, endian=endian)
rval, offset = UnpackF64(buf, endian=endian)
print 'f64:', tval, rval

tval = 0xfacedead
buf = PackP32(tval, endian=endian)
rval, offset = UnpackP32(buf, endian=endian)
print 'p32:', hex(tval), hex(rval)

#tval = 0xfacedeadf00dbeef
tval = 0xf00dbeef
buf = PackP64(tval, endian=endian)
rval, offset = UnpackP64(buf, endian=endian)
print 'p64:', hex(tval), hex(rval)

tval = 'packratatonga'
buf = PackString(tval, count=20)
rval, offset = UnpackString(buf, count=20)
print 'string:', repr(tval), repr(rval)
