###############################################################################
#
# Package:  NetMsgs
#
# File: NetMsgsBase.py
#

""" 
NetMsgs Base Data Module
"""

## \file 
## \package NetMsgs.NetMsgsBase
##
## $LastChangedDate: 2012-07-23 14:06:10 -0600 (Mon, 23 Jul 2012) $
## $Rev: 2098 $
##
## \brief NetMsgs Base Data Module
##
## \sa
## \htmlonly
##  <a href="../pydoc/NetMsgs.NetMsgsBase.html">PyDoc Generated Documentation</a>
## \endhtmlonly
##
## \author Robin Knight (robin.knight@roadnarrows.com)
##  
## \copyright
##   \h_copy 2009-2017. RoadNarrows LLC.\n
##   http://www.roadnarrows.com\n
##   All Rights Reserved
##

# Permission is hereby granted, without written agreement and without
# license or royalty fees, to use, copy, modify, and distribute this
# software and its documentation for any purpose, provided that
# (1) The above copyright notice and the following two paragraphs
# appear in all copies of the source code and (2) redistributions
# including binaries reproduces these notices in the supporting
# documentation.   Substantial modifications to this software may be
# copyrighted by their authors and need not follow the licensing terms
# described here, provided that the new terms are clearly indicated in
# all files where they apply.
#
# IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
# OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
# PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
# DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
# EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
# THE POSSIBILITY OF SUCH DAMAGE.
#
# THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
# INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
# FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
# "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
# PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
#
###############################################################################

import sys

from NetMsgs.NetMsgsCore import *

## Message Encoding Type Enumeration
NMEncoding = ['flat', 'itv']    # future , 'cli']

## Message Byte Ordering Type Enumeration
NMEndian = ['big', 'little', 'native']

##
## Built-In message field types, keyed by XML field type name.
##   code    - message field type byte code
##   desc    - short description
##   flen    - packed message field length (bytes)
##   comp    - complexity. one of: simple compound na
##   T       - C/C++ type specifier
##   pre     - member name prefix (quasi-Hungarian)
##
NMBuiltInFieldTypes = {
  'pad': {        # not really a field type
    'code': NMFTypePad,
    'desc': "pad byte",
    'flen': 1,
    'comp': 'na',
    'T':    '',
    'pre':  '',
  },
  'char': {
    'code': NMFTypeChar,
    'desc': "8-bit character",
    'flen': NMFVAL_LEN_CHAR,
    'comp': 'simple',
    'T':    'char',
    'pre':  'c',
  },
  'u8': {
    'code': NMFTypeU8,
    'desc': "unsigned 8-bit integer",
    'flen': NMFVAL_LEN_U8,
    'comp': 'simple',
    'T':    'byte_t',
    'pre':  'by',
  },
  's8': {
    'code': NMFTypeS8,
    'desc': "signed 8-bit integer",
    'flen': NMFVAL_LEN_S8,
    'comp': 'simple',
    'T':    'signed char',
    'pre':  'hhi',
  },
  'bool': {
    'code': NMFTypeBool,
    'desc': "boolean 0=false, 1(non-zero)=true",
    'flen': NMFVAL_LEN_BOOL,
    'comp': 'simple',
    'T':    'bool_t',
    'pre':  'b',
  },
  'u16': {
    'code': NMFTypeU16,
    'desc': "unsigned 16-bit integer",
    'flen': NMFVAL_LEN_U16,
    'comp': 'simple',
    'T':    'ushort_t',
    'pre':  'hu',
  },
  's16': {
    'code': NMFTypeS16,
    'desc': "signed 16-bit integer",
    'flen': NMFVAL_LEN_S16,
    'comp': 'simple',
    'T':    'short',
    'pre':  'hi',
  },
  'u32': {
    'code': NMFTypeU32,
    'desc': "unsigned 32-bit integer",
    'flen': NMFVAL_LEN_U32,
    'comp': 'simple',
    'T':    'uint_t',
    'pre':  'u',
  },
  's32': {
    'code': NMFTypeS32,
    'desc': "signed 32-bit integer",
    'flen': NMFVAL_LEN_S32,
    'comp': 'simple',
    'T':    'int',
    'pre':  'i',
  },
  'u64': {
    'code': NMFTypeU64,
    'desc': "unsigned 64-bit integer",
    'flen': NMFVAL_LEN_U64,
    'comp': 'simple',
    'T':    'ulonglong_t',
    'pre':  'llu',
  },
  's64': {
    'code': NMFTypeS64,
    'desc': "signed 64-bit integer",
    'flen': NMFVAL_LEN_S64,
    'comp': 'simple',
    'T':    'long long',
    'pre':  'lli',
  },
  'f32': {
    'code': NMFTypeF32,
    'desc': "32-bit floating-point number",
    'flen': NMFVAL_LEN_F32,
    'comp': 'simple',
    'T':    'float',
    'pre':  'hf',
  },
  'f64': {
    'code': NMFTypeF64,
    'desc': "64-bit floating-point number",
    'flen': NMFVAL_LEN_F64,
    'comp': 'simple',
    'T':    'double',
    'pre':  'f',
  },
  'p32': {
    'code': NMFTypeP32,
    'desc': "32-bit pointer",
    'flen': NMFVAL_LEN_P32,
    'comp': 'simple',
    'T':    'void *',
    'pre':  'p',
  },
  'p64': {
    'code': NMFTypeP64,
    'desc': "64-bit pointer",
    'flen': NMFVAL_LEN_P64,
    'comp': 'simple',
    'T':    'void *',
    'pre':  'p',
  },
  'string': {
    'code': NMFTypeString,
    'desc': "char[] string",
    'flen': 'variable',
    'comp': 'compound',
    'T':    'char',
    'pre':  's',
  },
  'struct': {
    'code': NMFTypeStruct,
    'desc': "structure",
    'flen': 'variable',
    'comp': 'compound',
    'T':    'struct',
    'pre':  'st',
  },
  'vector': {
    'code': NMFTypeVector,
    'desc': "vector - one dimensional array",
    'flen': 'variable',
    'comp': 'compound',
    'T':    '',
    'pre':  'vec',
  },
}
##

#
# Aliases
#
NMBuiltInFieldTypes['byte']        = NMBuiltInFieldTypes['u8']
NMBuiltInFieldTypes['schar']       = NMBuiltInFieldTypes['s8']
NMBuiltInFieldTypes['ushort']      = NMBuiltInFieldTypes['u16']
NMBuiltInFieldTypes['short']       = NMBuiltInFieldTypes['s16']
NMBuiltInFieldTypes['uint']        = NMBuiltInFieldTypes['u32']
NMBuiltInFieldTypes['int']         = NMBuiltInFieldTypes['s32']
NMBuiltInFieldTypes['ulonglong']   = NMBuiltInFieldTypes['u64']
NMBuiltInFieldTypes['longlong']    = NMBuiltInFieldTypes['s64']
NMBuiltInFieldTypes['pointer']     = NMBuiltInFieldTypes['p32']
NMBuiltInFieldTypes['longpointer'] = NMBuiltInFieldTypes['p64']
NMBuiltInFieldTypes['float']       = NMBuiltInFieldTypes['f32']
NMBuiltInFieldTypes['double']      = NMBuiltInFieldTypes['f64']

## Get NetMsgs field type code given the XML field type.
NMFCode = lambda xmlftype: NMBuiltInFieldTypes[xmlftype]['code']

## The full set of XML ftype values
NMAliasMap = {
    'byte': 'u8', 'schar': 's8', 'ushort': 'u16', 'short': 'u16',
    'uint': 'u32', 'int': 's32', 'ulonglong': 'u64', 'longlong': 's64',
    'pointer': 'p32', 'longpointer': 'p64', 'float': 'f32', 'double': 'f64',
}

## Special DB dictionary order key
NMKeyOrder  = '>'

## Special DB pad field key
NMKeyPad    = '#'

## XML ftype attribute vector suffix string
NMVectorSuffix  = '[]'

## List of simple field types by XML ftype 
NMFTypeSimple = [
    'char', 'u8', 's8', 'bool', 'u16', 's16', 'u32', 's32', 'u64',
    's64', 'f32', 'f64', 'p32', 'p64', ]

## List of simple field types by field type code
NMFTypeCodeSimple = [
    NMFCode('char'),  NMFCode('u8'),    NMFCode('s8'),    NMFCode('bool'),
    NMFCode('u16'),   NMFCode('s16'),   NMFCode('u32'),   NMFCode('s32'),
    NMFCode('u64'),   NMFCode('s64'),   NMFCode('f32'),   NMFCode('f64'),
    NMFCode('p32'),   NMFCode('p64') ]

## List of compound field types by XML ftype 
NMFTypeCompound = [ 'string', 'struct', 'vector' ]

## List of compound field types by field type code
NMFTypeCodeCompound = [NMFCode('string'), NMFCode('struct'), NMFCode('vector')]

## List of number field types by XML ftype
NMFTypeNumber = [
      'u8', 's8', 'u16', 's16', 'u32', 's32', 'u64', 's64', 'f32', 'f64' ]

## Field type code to XML file type map
NMFTypeCode2Xml = {
    NMFCode('bool'): 'bool',      NMFCode('char'): 'char',
    NMFCode('u8'): 'u8',          NMFCode('s8'): 's8',
    NMFCode('u16'): 'u16',        NMFCode('s16'): 's16',
    NMFCode('u32'): 'u32',        NMFCode('s32'): 's32',
    NMFCode('u64'): 'u64',        NMFCode('s64'): 's64',
    NMFCode('f32'): 'f32',        NMFCode('f64'): 'f64',
    NMFCode('p32'): 'p32',        NMFCode('p64'): 'p64',
    NMFCode('string'): 'string',  NMFCode('pad'): 'pad',  
    NMFCode('struct'): 'struct',  NMFCode('vector'): 'vector'
}

## Field Header Lengths keyed by message encoding
NMFHdrLen = {
    'flat': {'simple': 0, 'string': 0, 'struct': 0, 'vector': 0},
    'itv':  {
      'simple': NMITV_FHDR_SIZE_SIMPLE, 'string': NMITV_FHDR_SIZE_STRING,
      'struct': NMITV_FHDR_SIZE_STRUCT, 'vector': NMITV_FHDR_SIZE_VECTOR},
}

## No field id value
NMFIdNone = NMFID_NONE

## Default pad count
NMPadDftCount = 1

## Pad field value
NMPadFVal = NMFTypePadTr

## Maximum and default string maximum length
NMStringMaxCount = NMFVAL_LEN_MAX_STRING

## Maximum and default vector maximum item count
NMVectorMaxCount = NMFVAL_LEN_MAX_VECTOR

## space quickie
space = lambda indent: "%*s" % (indent, '')

#--
def StrError(ecode):
  """ Get the error string describing the NetMsgs error code.
      
      The absolute value of the error code is taken prior retrieving the
      string. An unknown or out-of-range error code will be mapped to
      NM_ECODE_BADEC.
    
      Parameters:
        ecode     - NetMsgs error code.

      Return:
        The appropriate error code string.
  """
  sErr = nmStrError(ecode)
  if not sErr:
    sErr = 'Error'
  return sErr
##


#------------------------------------------------------------------------------
# CLASS: NetMsgsError
#------------------------------------------------------------------------------
class NetMsgsError(Exception):
  """ NetMsgs Exception Class. """

  def __init__(self, msg='XML Parser Error'):
    """ Raise exception.

        Parameters:
          msg    - Exception message string.
    """
    Exception.__init__(self, msg)
##


#-------------------------------------------------------------------------------
# Support Utilities
#-------------------------------------------------------------------------------

#--
def IsIdentifier(token):
  """ Returns True if token is a valid identifier, else False.

      Parameters:
        token   - Parsed token.
  """
  if not token:
    return False
  c = token[0]
  if not c.isalpha() and c != "_":
    return False
  for c in token[1:]:
    if not c.isalnum() and c != "_":
      return False
  return True
##

#--
def PrettyPrintCols(fp, cursor, *args, **kwargs):
  """ Pretty print argument strings aligned to column.

      Parameters:
        cursor    - Current column cursor position.
        args      - List of argument (pos, s) 2-tuples.
        kwargs    - Print control keywords.
  """
  while len(args) >= 2: 
    linecont = kwargs.get('linecont', '')
    force    = kwargs.get('force', False)
    pos = args[0]
    s   = args[1]
    args = args[2:]
    if (pos <= cursor) and (cursor > 0):
      if not force or cursor > 78:
        fp.write("%s\n" % (linecont))
        cursor = 0
      else:
        fp.write(" ")
        cursor += 1
    if pos > cursor:
      fp.write(space(pos-cursor))
      cursor = pos
    fp.write("%s" % (s))
    cursor += len(s)
  return cursor
##

#--
def PrintBuf(buf, count=None, preface='', nlfreq=None, indent=0, col=0,
                        fp=sys.stderr):
  """ Pretty print binary buffer to opened file stream.

      Parameters:
        buf     - Buffer to print.
        count   - Number of bytes to print.
        preface - Optional buffer preface string.
        nlfreq  - Newline frequency (None for no newlines).
        ident   - Indentation column alignment.
        col     - Current column position.
        fp      - Output file pointer.
  """
  if preface:
    s = "%s: " % (preface)
    col += len(s)
    fp.write(s)
  if count is None:
    count = len(buf)
  if (count > 0) and (col < indent):
    fp.write(space(indent-col))
  i = 0
  while i < count:
    c = buf[i]
    if nlfreq and ((i % nlfreq) == 0) and (i > 0):

      fp.write("\n%s" % space(indent))
      col = indent
    fp.write("0x%02x " % (ord(c)))
    i += 1
  fp.write('\n')
##

#--
def PrintBits(val, msbit, count=None, preface='', fp=sys.stderr):
  """ Pretty print bits in value to opened file stream. 

      Parameters:
        val     - Bits to print.
        msbit   - Starting most significant bit, zero based.
        count   - Number of bits to print (None = msbit+1).
        preface - Optional buffer preface string.
        fp      - Output file pointer.
  """
  if preface:
    s = "%s: " % (preface)
    fp.write(s)
  if count is None:
    count = msbit + 1
  i = 0
  while i < count:
    if ((msbit % 8) == 7) and (i > 0):
      fp.write(' ')
    if (val >> msbit) & 0x01:
      fp.write('1')
    else:
      fp.write('0')
    msbit -= 1
    i     += 1
##

#--
def _atval(val):
  """ Convert value to string equivalent.

      String values starting with an '@' are treated as variables, not strings.
      The '@' is stripped.

      Parameters:
        val   - ['@']value.

      Return:
        Converted value.
  """
  if (type(val) == str) and (len(val) > 0) and (val[0] == '@'):
    return val[1:]
  else:
    return repr(val)
##

## return number of spaces given column position and indentation
_nspaces = lambda col,indent: col<indent and indent-col or 0

#--
def PrettyPrintAssignExpr(name, val, col=0, indent=0, fp=sys.stderr):
  """ Pretty print name = value.

      Parameters:
        nam     - Variable name
        val     - Variable value.
        col     - Current column position.
        indent  - Indentation.
        fp      - Opened file pointer.
  """
  sp = _nspaces(col, indent)
  lhs = "%s%s = " % (space(sp), name)
  fp.write(lhs)
  PrettyPrintVal(val, col=len(lhs), indent=indent, fp=fp)
  fp.write('\n')
##

#--
def PrettyPrintVal(val, col=0, indent=0, fp=sys.stderr):
  """ Pretty print value.

      Parameters:
        val     - Variable value.
        col     - Current column position.
        indent  - Indentation.
        fp      - Opened file pointer.

      Return:
        New column position
  """
  if type(val) == dict:
    return PrettyPrintDict(val, col=col, indent=indent, fp=fp)
  elif type(val) == list:
    return PrettyPrintList(val, col=col, indent=indent, fp=fp)
  else:
    sp = _nspaces(col, indent)
    v = _atval(val)
    fp.write("%s%s" % (space(sp), v))
    return col + sp + len(v)
##

#--
def PrettyPrintDict(d, col=0, indent=0, fp=sys.stderr):
  """ Pretty print dictionary in sorted, indented clarity.

      Parameters:
        d       - The dictionary.
        col     - Current column position.
        indent  - Indentation.
        fp      - Opened file pointer.

      Return:
        New column position
  """
  sp = _nspaces(col, indent)
  s = repr(d)
  if col + sp + len(s) < 80:
    fp.write('%s{' % space(sp))
    col = col + sp + 1
    for k in sorted(d):
      key = _atval(k)
      fp.write("%s:" % key)
      col += len(key) + 1
      col = PrettyPrintVal(d[k], col=col, indent=indent, fp=fp)
      fp.write(', ')
      col += 2
    fp.write("}")
    return col + 1
  else:
    fp.write('%s{\n' % space(sp))
    col = 0
    indent += 2
    for k in sorted(d):
      key = _atval(k)
      key = "%s%s: " % (space(indent), key)
      fp.write(key)
      PrettyPrintVal(d[k], col=len(key), indent=indent, fp=fp)
      fp.write(',\n')
    indent -= 2
    fp.write("%s}" % (space(indent)))
    return indent + 1
##

#--
def PrettyPrintList(l, col=0, indent=0, fp=sys.stderr):
  """ Pretty print list.

      Parameters:
        l       - The list.
        col     - Current column position.
        indent  - Indentation.
        fp      - Opened file pointer.

      Return:
        New column position
  """
  sp = _nspaces(col, indent)
  issimple = True
  for v in l:
    if (type(v) == dict) or (type(v) == list):
      issimple = False
      break
  if issimple:
    fp.write('%s[ ' % space(sp))
    col = col + sp + 2
    for v in l:
      if col > 60:  # heuristiccally 'safe'
        fp.write('\n%s  ' % space(indent))
        col = col + sp + 2
      col = PrettyPrintVal(v, col=col, indent=indent, fp=fp)
      fp.write(', ')
      col += 2
    fp.write(' ]')
    return col + 2
  else:
    fp.write('%s[\n' % space(sp))
    col = 0
    indent += 2
    for v in l:
      PrettyPrintVal(v, col=col, indent=indent, fp=fp)
      fp.write(',\n')
    indent -= 2
    fp.write("%s]" % (space(indent)))
    return indent + 1
##
