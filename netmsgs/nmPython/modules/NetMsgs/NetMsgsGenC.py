###############################################################################
#
# Package:  NetMsgs
#
# File:     NetMsgsGenC.py
#

""" 
NetMsgs Generate C files module.

A pair of C .h header and .c source files are generated from a  RoadNarrows
NetMsg XML specification.
"""

## \file 
## \package NetMsgs.NetMsgsGenC
##
## $LastChangedDate: 2010-08-04 15:07:55 -0600 (Wed, 04 Aug 2010) $
## $Rev: 550 $
##
## \brief NetMsgs Generate C files module.
##
## A pair of C .h header and .c source files are generated from a  RoadNarrows
## NetMsg XML specification.
## 
## \todo Support hungarian optional keyword.
## \todo Make a smarter EvalNum to simplify the equation with unknowns.
##
## \sa
## \htmlonly
##  <a href="../pydoc/NetMsgs.NetMsgsGenC.html">PyDoc Generated Documentation</a>
## \endhtmlonly
##
## \author Robin Knight (robin.knight@roadnarrows.com)
##  
## \par Copyright:
##   (C) 2009-2010.  RoadNarrows LLC.\n
##   (http://www.roadnarrows.com)\n
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
import os
import time

import NetMsgs.NetMsgsBase as nmBase
import NetMsgs.NetMsgsXmlParser as NMXmlParser


#-----------------------------------------------------------------------------
# CLASS: NetMsgsGenC
#-----------------------------------------------------------------------------
class NetMsgsGenC:
  """ RoadNarrows Net Messages C Source Generator Class.

      The NetMsgsGenC class reads the post-parsed database of a NetMsgsXmlParser
      class instance and generates a C .h and .c file pair. The C files expects
      the libnetmsgs C library.
  """

  #--
  def __init__(self, xml, hfilepath, cfilepath, **kwargs):
    """ Initialize NetMsgsGenC instance.

        Parameters:
          xml         - NetMsgsXMLParser class instance.
          hfilepath   - Generated output h file.
          cfilepath   - Generated output c file.
          kwargs      - Optional keyword arguments.
                          debug     - Set debugging level: 0 == off, 1, 2, 3.
                          incprefix - Generated .h include prefix in .c file.
    """
    ## parsed xml instance
    self.mXml       = xml

    ## output .h header file name
    self.mHFilePath = hfilepath

    ## output .c header file name
    self.mCFilePath = cfilepath

    ## debug level
    self.mDebug     = kwargs.get('debug', 0)

    ## include prefix
    self.mIncPrefix = kwargs.get('incprefix', '')

    ## do [not] prepend RN Hungarian notation to message fields
    self.mHungarian = kwargs.get('hungarian', False)

    ## output .h header file basename
    self.mHBaseName = os.path.basename(self.mHFilePath)

    ## output .c header file basename
    self.mCBaseName = os.path.basename(self.mCFilePath)

    self.AugmentXmlDB()
  ##

  #--
  def AugmentXmlDB(self):
    """ Augument the XML database of parsed XML values with C generated
        parameters.
    """
    self.mXml['genc'] = { }
    encoding  = self.mXml['netmsgs']['encoding'].lower()
    if not encoding:
      encoding = 'itv'
    if encoding == 'itv':
      self.mXml['genc']['encoding_prefix'] = encoding.upper()
    else:
      self.mXml['genc']['encoding_prefix'] = encoding.capitalize()

    endian = self.mXml['netmsgs']['endian'].lower()
    if not endian:
      endian = 'big'
    self.mXml['genc']['endian_desc'] = endian + '-endian'
    self.mXml['genc']['endian_enum'] = 'NMEndian'+endian.capitalize()

    # perform pass 0 through database, setting C specific information
    self.Pass0()
  ##

  #--
  def Pass0(self):
    """ Make initial pass through the XML database and augument with
        specific C information.

        No generated C code is written to file output.
    """
    for xid in self.mXml['field_types'][nmBase.NMKeyOrder]:
      maxlen = self.Pass0XmlDef([xid], self.mXml['field_types'][xid], 0)
    for xid in self.mXml['msg_types'][nmBase.NMKeyOrder]:
      maxlen = self.Pass0XmlDef([xid], self.mXml['msg_types'][xid], 0)
    #self.mXml.PrettyPrintDB("Pass 0")
  ##

  #--
  def Pass0XmlDef(self, pathXids, xdef, depth):
    """ Process the given XmL definition and add C specific data.

        No generated C code is written to file output.

        Parameters:
          pathXids  - Path (list) of XML ftypedef, msgdef, or fielddef
                      identifiers.
          xdef      - XML ftypedef, msgdef, or fielddef XML specification.
          depth     - Recursive definition depth.

        Return:
          Maximum number of packed bytes required for this encoded definition.
    """
    ns            = self.mXml['meta']['ns']   # namespace
    xid           = pathXids[-1]              # 'basename'
    xdef['genc']  = { }                       # add 'genc' dictionary to xdef
    ftype         = xdef['ftype']             # XML field type

    # base field type
    base_ftype          = self.BaseFType(ftype)
    xdef['base_ftype']  = base_ftype

    # pad field 'type'
    if ftype == 'pad':
      maxlen = self.Pass0XmlDefPad(pathXids, xdef, depth)

    # structure field type
    elif ftype == 'struct':
      maxlen = self.Pass0XmlDefStruct(pathXids, xdef, depth)

    # vector field type
    elif ftype == 'vector':
      maxlen = self.Pass0XmlDefVector(pathXids, xdef, depth)

    # string field type
    elif ftype == 'string':
      maxlen = self.Pass0XmlDefString(pathXids, xdef, depth)

    # simple field type
    elif nmBase.NMBuiltInFieldTypes.has_key(ftype):
      maxlen = self.Pass0XmlDefSimple(pathXids, xdef, depth)

    # typdef'ed derived field type
    else:
      maxlen = self.Pass0XmlDefDerived(pathXids, xdef, depth)

    return maxlen
  ##

  #--
  def Pass0XmlDefPad(self, pathXids, xdef, depth):
    """ Process pad XML definition.

        Parameters:
          pathXids  - Path (list) of XML ftypedef, msgdef, or fielddef
                      identifiers.
          xdef      - XML ftypedef, msgdef, or fielddef XML specification.
          depth     - Recursive definition depth.

        Return:
          Maximum number of packed bytes required for this encoded definition.
    """
    # maximum pad field length (bytes)
    if self.IsDeprecated(xdef):
      maxlen = 0
      active_cnt = 0
    else:
      eq_sum = repr(nmBase.NMBuiltInFieldTypes['pad']['flen'])+'*'+xdef['count']
      maxlen = self.EvalNum(eq_sum)
      active_cnt = 1

    xdef['genc']['type_spec']   = None
    xdef['genc']['id']          = None
    xdef['genc']['const_expr']  = None
    xdef['genc']['comment']     = 'message padding'
    xdef['genc']['maxlen']      = maxlen
    xdef['genc']['active_cnt']  = active_cnt

    return maxlen
  ##

  #--
  def Pass0XmlDefStruct(self, pathXids, xdef, depth):
    """ Process struct XML definition.

        Parameters:
          pathXids  - Path (list) of XML ftypedef, msgdef, or fielddef
                      identifiers.
          xdef      - XML ftypedef, msgdef, or fielddef XML specification.
          depth     - Recursive definition depth.

        Return:
          Maximum number of packed bytes required for this encoded definition.
    """
    encoding    = self.mXml['netmsgs']['encoding']        # message encoding
    eq_sum      = [nmBase.NMFHdrLen[encoding]['struct']]  # field header length
    active_cnt  = 0                                       # active field count

    if self.IsDeprecated(xdef):
      maxlen = 0
    else:
      # recurse into structure definition
      for fname in xdef['fields'][nmBase.NMKeyOrder]:
        n = self.Pass0XmlDef(pathXids+[fname], xdef['fields'][fname], depth+1)
        eq_sum += [n]
        if n > 0:
          active_cnt += 1
      maxlen = self.EvalNum(*eq_sum)          # max length of structure

    ns          = self.mXml['meta']['ns']     # namespace
    xid         = pathXids[-1]                # 'basename'
    type_spec   = 'struct'                    # C type specifier
    cid         = self.MakeCId(xid, depth==0) # C indentifier
    const_expr  = None                        # C constant expression
    msgdef      = "%s%sMsgDef" % (ns, ''.join(pathXids))  # assoc. message def

    # augment db
    xdef['genc']['type_spec']   = type_spec
    xdef['genc']['id']          = cid
    xdef['genc']['const_expr']  = const_expr
    xdef['genc']['comment']     = "%s structure" % (xid)
    xdef['genc']['msgdef']      = msgdef
    xdef['genc']['maxlen']      = maxlen
    xdef['genc']['active_cnt']  = active_cnt

    return maxlen
  ##

  #--
  def Pass0XmlDefVector(self, pathXids, xdef, depth):
    """ Process vector XML definition.

        Parameters:
          pathXids  - Path (list) of XML ftypedef, msgdef, or fielddef
                      identifiers.
          xdef      - XML ftypedef, msgdef, or fielddef XML specification.
          depth     - Recursive definition depth.

        Return:
          Maximum number of packed bytes required for this encoded definition.
    """
    ns          = self.mXml['meta']['ns']     # namespace
    xid         = pathXids[-1]                # 'basename'
    type_spec   = 'struct'                    # C type specifier
    cid         = self.MakeCId(xid, depth==0) # C indentifier
    vtype       = xdef['vtype']               # vector item field type
    vtype_spec  = self.MakeTypeSpec(vtype)    # vector item C specifier
    base_vtype  = self.BaseFType(vtype)       # vector item base field type
    base_vname  = self.BaseXRef(vtype)        # vector item base field name

    fsize = xdef.get('size', 'NMFVAL_LEN_MAX_VECTOR')
    if nmBase.IsIdentifier(fsize) and fsize != 'NMFVAL_LEN_MAX_VECTOR':
      macro_len = {
          'name': fsize,
          'token_seq': None,
          'comment': None,
      }
    else:
      nlist = pathXids + ['len']
      macro_len = {
          'name': self.MakeCppMacroName(*nlist),
          'token_seq': '(' + fsize + ')',
          'comment': "%s maximum vector length" % (' '.join(pathXids)),
      }
    const_expr  = macro_len['name']
    fielddef    = "%s%sFieldDef" % (ns, ''.join(pathXids))

    # maximum vector field length (bytes)
    encoding  = self.mXml['netmsgs']['encoding']        # message encoding
    eq_sum    = [nmBase.NMFHdrLen[encoding]['vector']]  # field header length
    if macro_len['token_seq']:
      maxcount = macro_len['token_seq']
    else:
      maxcount = macro_len['name']
    if base_vname == 'this':
      fhdrlen = '0'
      flen    = repr(nmBase.NMBuiltInFieldTypes[base_vtype]['flen'])
    elif self.IsSimple(base_vtype):
      fhdrlen = '0'
      flen    = repr(nmBase.NMBuiltInFieldTypes[base_vtype]['flen'])
    else:
      fhdrlen = repr(nmBase.NMFHdrLen[encoding][base_vtype])
      flen    = self.mXml['field_types'][base_vname]['genc']['maxlen']

    if self.IsDeprecated(xdef):
      maxlen = 0
      active_cnt = 0
    else:
      eq_sum += [flen + '*' + maxcount]
      maxlen  = self.EvalNum(*eq_sum)
      active_cnt  = 1
      
    # augment db
    xdef['genc']['type_spec']   = type_spec
    xdef['genc']['id']          = cid
    xdef['genc']['const_expr']  = const_expr
    xdef['genc']['comment']     = "%s vector" % (xid)
    xdef['genc']['vtype_spec']  = vtype_spec
    xdef['genc']['base_vtype']  = base_vtype
    xdef['genc']['base_vname']  = base_vname
    xdef['genc']['macro_len']   = macro_len
    xdef['genc']['fielddef']    = fielddef
    xdef['genc']['maxlen']      = maxlen
    xdef['genc']['active_cnt']  = active_cnt

    return maxlen
  ##

  #--
  def Pass0XmlDefString(self, pathXids, xdef, depth):
    """ Process string XML definition.

        Parameters:
          pathXids  - Path (list) of XML ftypedef, msgdef, or fielddef
                      identifiers.
          xdef      - XML ftypedef, msgdef, or fielddef XML specification.
          depth     - Recursive definition depth.

        Return:
          Maximum number of packed bytes required for this encoded definition.
    """
    ns          = self.mXml['meta']['ns']     # namespace
    xid         = pathXids[-1]                # 'basename'
    ftype       = xdef['ftype']               # XML type
    type_spec   = nmBase.NMBuiltInFieldTypes[ftype]['T']  # C type specifier
    cid         = self.MakeCId(xid, depth==0) # C indentifier

    fsize = xdef.get('size', 'NMFVAL_LEN_MAX_STRING')
    if nmBase.IsIdentifier(fsize) and fsize != 'NMFVAL_LEN_MAX_STRING':
      macro_len = {
          'name': fsize,
          'token_seq': None,
          'comment': None,
      }
    else:
      nlist = pathXids + ['len']
      macro_len = {
          'name': self.MakeCppMacroName(*nlist),
          'token_seq': '(' + fsize + ')',
          'comment': "%s maximum string length" % (' '.join(pathXids)),
      }

    const_expr = macro_len['name'] + '+1'

    # maximum string field length (bytes)
    encoding  = self.mXml['netmsgs']['encoding']        # message encoding
    eq_sum    = [nmBase.NMFHdrLen[encoding]['string']]  # field header length
    if macro_len['token_seq']:
      maxcount = macro_len['token_seq']
    else:
      maxcount = macro_len['name']

    if self.IsDeprecated(xdef):
      maxlen = 0
      active_cnt = 0
    else:
      eq_sum += [maxcount]
      maxlen  = self.EvalNum(*eq_sum)
      active_cnt  = 1
      
    # augment db
    xdef['genc']['type_spec']   = type_spec
    xdef['genc']['id']          = cid
    xdef['genc']['const_expr']  = const_expr
    xdef['genc']['comment']     = "%s string" % (xid)
    xdef['genc']['macro_len']   = macro_len
    xdef['genc']['maxlen']      = maxlen
    xdef['genc']['active_cnt']  = active_cnt

    return maxlen
  ##

  #--
  def Pass0XmlDefSimple(self, pathXids, xdef, depth):
    """ Process simple XML definition.

        Parameters:
          pathXids  - Path (list) of XML ftypedef, msgdef, or fielddef
                      identifiers.
          xdef      - XML ftypedef, msgdef, or fielddef XML specification.
          depth     - Recursive definition depth.

        Return:
          Maximum number of packed bytes required for this encoded definition.
    """
    ns          = self.mXml['meta']['ns']     # namespace
    xid         = pathXids[-1]                # 'basename'
    ftype       = xdef['ftype']               # XML type
    base_ftype  = xdef['base_ftype']          # base field type
    type_spec   = nmBase.NMBuiltInFieldTypes[ftype]['T']  # C type specifier
    cid         = self.MakeCId(xid, depth==0) # C indentifier
    const_expr  = None                        # C constant expression

    # maximum simple field length (bytes)
    encoding  = self.mXml['netmsgs']['encoding']        # message encoding
    fhdr_len  = nmBase.NMFHdrLen[encoding]['simple']    # field header length
    flen      = nmBase.NMBuiltInFieldTypes[base_ftype]['flen']

    if self.IsDeprecated(xdef):
      maxlen = 0
      active_cnt = 0
    else:
      eq_sum  = [fhdr_len, flen]
      maxlen  = self.EvalNum(*eq_sum)
      active_cnt  = 1
      
    # augment db
    xdef['genc']['type_spec']   = type_spec
    xdef['genc']['id']          = cid
    xdef['genc']['const_expr']  = const_expr
    xdef['genc']['comment']     = "%s" % (xid)
    xdef['genc']['maxlen']      = maxlen
    xdef['genc']['active_cnt']  = active_cnt

    return maxlen
  ##

  #--
  def Pass0XmlDefDerived(self, pathXids, xdef, depth):
    """ Process derived type XML definition.

        Parameters:
          pathXids  - Path (list) of XML ftypedef, msgdef, or fielddef
                      identifiers.
          xdef      - XML ftypedef, msgdef, or fielddef XML specification.
          depth     - Recursive definition depth.

        Return:
          Maximum number of packed bytes required for this encoded definition.
    """
    ns          = self.mXml['meta']['ns']     # namespace
    xid         = pathXids[-1]                # 'basename'
    fname       = xdef['ftype']               # XML type is really a derived nam
    type_spec   = self.MakeTypeSpec(fname)    # C type specifier
    cid         = self.MakeCId(xid, depth==0) # C indentifier
    const_expr  = None                        # C constant expression
    base_ftype  = xdef['base_ftype']          # base XML type

    bname       = self.BaseXRef(fname)
    if bname == 'this':
      bname = fname
    bdef        = self.mXml['field_types'][bname]
    maxlen      = bdef['genc']['maxlen']
    active_cnt  = bdef['genc']['active_cnt']

    if base_ftype == 'struct':
      comment = "%s structure" % (xid)
      xdef['genc']['msgdef'] = "%s%sMsgDef" % (ns, fname)
    elif base_ftype == 'vector':
      vtype_spec  = bdef['genc']['vtype_spec']
      const_expr  = bdef['genc']['const_expr']
      comment     = "%s vector" % (xid)

      xdef['genc']['vtype_spec']  = vtype_spec
      xdef['genc']['base_vtype']  = bdef['vtype']
      xdef['genc']['base_vname']  = bname
      xdef['genc']['macro_len']   = bdef['genc']['macro_len']
      xdef['genc']['fielddef']    = "%s%sFieldDef" % (ns, bname)
    elif base_ftype == 'string':
      bname       = self.BaseXRef(fname)
      if bname == 'this':
        bname = fname
      bdef        = self.mXml['field_types'][bname]
      const_expr  = bdef['genc']['const_expr']
      comment     = "%s string" % (xid)

      xdef['genc']['macro_len']   = bdef['genc']['macro_len']
    else:
      comment = xid

    # augment db
    xdef['genc']['type_spec']   = type_spec
    xdef['genc']['id']          = cid
    xdef['genc']['const_expr']  = const_expr
    xdef['genc']['comment']     = comment
    xdef['genc']['maxlen']      = maxlen
    xdef['genc']['active_cnt']  = active_cnt

    return maxlen
  ##


  #.............................................................................
  # Generate Source Functions
  #.............................................................................

  #--
  def GenSource(self):
    """ Generate C .h and .c source file pair.

        Exceptions:
          Raises NetMsgError exception on error. 
    """
    self.GenHSource()
    self.GenCSource()
    #self.mXml.PrettyPrintDB("Final")
  ##

  #--
  def GenHSource(self):
    """ Generate h header.

        Exceptions:
          Raises NetMsgError exception on error. 
    """
    # open C header file for writing
    try:
      fp = open(self.mHFilePath, 'w')
    except IOError, err:
      raise nmBase.NetMsgsError("Error: %s" % (err))
    self.PrettyPrintTopComment(fp, self.mHBaseName, self.mXml['meta']['brief'])
    self.HGenPrologue(fp)
    self.HGenMsgIds(fp)
    self.HGenFieldTypes(fp)
    self.HGenMsgTypes(fp)
    self.HGenExternData(fp)
    self.HGenProtoTypes(fp)
    self.HGenEpilogue(fp)
    fp.close()
  ##

  #--
  def GenCSource(self):
    """ Generate h header.

        Exceptions:
          Raises NetMsgError exception on error. 
    """
    # open C source file for writing
    try:
      fp = open(self.mCFilePath, 'w')
    except IOError, err:
      raise nmBase.NetMsgsError("Error: %s" % (err))
    self.PrettyPrintTopComment(fp, self.mCBaseName, self.mXml['meta']['brief'])
    self.CGenPrologue(fp)
    self.PrettyPrintMajorDivComment(fp, "Private Interface")
    self.CGenFieldTypes(fp)
    self.CGenMsgTypes(fp)
    self.PrettyPrintMajorDivComment(fp, "Public Interface")
    self.CGenLookupTbls(fp)
    self.CGenPublicFuncDefs(fp)
    self.CGenEpilogue(fp)
    fp.close()
  ##


  #.............................................................................
  # H File Generator Functions
  #.............................................................................

  #--
  def HGenPrologue(self, fp):
    """ Generate .h header file prologue.

        Prologue includes core header file includes plus any prologue specified
        in the XML file.

        Parameters:
          fp    - Opened output file pointer.
    """
    hdef      = "_" + self.mHBaseName.replace(".", "_").upper()
    prologue  = self.mXml['meta']['h']['prologue']
    fp.write(
"""

#ifndef %s
#define %s

#include "rnr/rnrconfig.h"
#include "rnr/netmsgs.h"

""" % (hdef, hdef))
    if prologue:
      fp.write("%s\n\n" % (prologue))
    fp.write("""
C_DECLS_BEGIN

""")
  ##

  #--
  def HGenMsgIds(self, fp):
    """ Generate .h Message Id enumeration.

        Parameters:
          fp    - Opened output file pointer.
    """
    ns        = self.mXml['meta']['ns']
    msgorder  = self.mXml['msg_types'][nmBase.NMKeyOrder]
    section   = 'msg_types'
    if len(msgorder) == 0:
      return
    self.PrettyPrintBriefComment(fp, "%s Message Id Enumeration" % (ns))
    fp.write("typedef enum\n")
    fp.write("{\n")
    n     = 0
    nmBase.PrettyPrintCols(fp, 0,
        2,  ns + 'MsgIdNone',
        40, '= '+repr(n)+',',
        48, '///< no message\n')
    for msgid in msgorder:
      msgdef = self.mXml[section][msgid]
      n += 1
      enum = ns + "MsgId" + msgid
      if not self.IsDeprecated(msgdef):
        comment = "%s" % (msgid)
      else:
        comment = "%s (deprecated)" % (msgid)
      nmBase.PrettyPrintCols(fp, 0,
        2,  enum,
        40, '= '+repr(n)+',',
        48, "///< %s\n" % (comment))
    n += 1
    nmBase.PrettyPrintCols(fp, 0,
        2,  ns+'MsgIdNumOf',
        40, '= '+repr(n),
        48, '///< number of message ids\n')
    fp.write("} %s;\n" % (ns+'MsgId_T'))
    fp.write("\n")
  ##

  #--
  def HGenFieldTypes(self, fp):
    """ Generate .h header file field types section.

        Parameters:
          fp    - Opened output file pointer.
    """
    section = 'field_types'
    if len(self.mXml[section][nmBase.NMKeyOrder]) == 0:
      return
    self.PrettyPrintMajorDivComment(fp, "Extended Field Types")
    for ftid in self.mXml[section][nmBase.NMKeyOrder]:
      ftdef = self.mXml[section][ftid]
      self.PrettyPrintMinorDivComment(fp,
            "Field Type %s Declarations" % (self.mXml['meta']['ns']+ftid))
      self.HGenTypedef(fp, ftid, ftdef, "%s Field Type" % (ftid))
  ##

  #--
  def HGenMsgTypes(self, fp):
    """ Generate .h header file message types section.

        Parameters:
          fp    - Opened output file pointer.
    """
    section = 'msg_types'
    if len(self.mXml[section][nmBase.NMKeyOrder]) == 0:
      return
    self.PrettyPrintMajorDivComment(fp, "Message Types")
    for msgid in self.mXml[section][nmBase.NMKeyOrder]:
      msgdef = self.mXml[section][msgid]
      if not self.IsDeprecated(msgdef):
        if msgdef['genc']['active_cnt'] > 0:
          self.PrettyPrintMinorDivComment(fp,
            "Message %s Declarations" % (self.mXml['meta']['ns']+msgid))
        self.HGenTypedef(fp, msgid, msgdef, "Message %s Structure" % (msgid))
  ##

  #--
  def HGenTypedef(self, fp, xid, xdef, brief):
    """ Generate a commented type definition subsection.

        Parameters:
          fp      - Opened output file pointer.
          xid     - Unique id in XML database section.
          xdef    - XML ftypedef, msgdef, or fielddef XML database entry.
          brief   - Brief comment string.
    """
    ns      = self.mXml['meta']['ns']   # namespace
    ftype   = xdef['ftype']             # field type

    type_spec   = xdef['genc']['type_spec']   # C type specifier
    cid         = xdef['genc']['id']          # C identifier
    const_expr  = xdef['genc']['const_expr']  # C array constant expression

    if self.IsDeprecated(xdef):
      return

    # generate all #defines directly associated with the typedef
    self.HGenTypedefMacros(fp, xid, xdef)

    # generate C structure typedef
    if ftype == 'struct':
      if xdef['genc']['active_cnt'] == 0:
        return
      self.PrettyPrintBriefComment(fp, brief)
      fp.write("typedef %s\n{\n" %  (type_spec))
      self.HGenStructDeclList(fp, xdef['fields'], 1)
      nmBase.PrettyPrintCols(fp, 0,
        0,  "} %s;" % (cid),
        46, "///< %s\n" % (xdef['genc'].get('comment', cid[2:])))

    # generate C vector typedef
    elif ftype == 'vector':
      vtype_spec  = xdef['genc']['vtype_spec']
      self.PrettyPrintBriefComment(fp, brief)
      fp.write("typedef ")
      self.WriteVectortDecl(fp, 0, vtype_spec, cid, const_expr)

    # generate any other C typedef
    else:
      self.PrettyPrintBriefComment(fp, brief)
      self.WriteTypedefStmt(fp, type_spec, cid, const_expr)

    fp.write("\n")
  ##

  #--
  def HGenStructDeclList(self, fp, fields, depth):
    """ Generate comment structure declaration list (members).

        Parameters:
          fp      - Opened output file pointer.
          fields  - XML database field definition 'fields' entry.
          depth   - Current structure depth.
    """
    ns = self.mXml['meta']['ns']    # namespace
    for fname in fields[nmBase.NMKeyOrder]:
      fdef        = fields[fname]         # XML db field definition
      ftype       = fdef['ftype']         # field type
      base_ftype  = fdef['base_ftype']    # base field type

      type_spec   = fdef['genc']['type_spec']   # C type specifier
      cid         = fdef['genc']['id']          # C identifier
      const_expr  = fdef['genc']['const_expr']  # C array constant expression 

      # deprecated fields are not declared
      if self.IsDeprecated(fdef):
        continue

      # pad are not declared
      elif ftype == 'pad':
        continue

      # structure embedded structure member
      elif ftype == 'struct':
        fp.write("%*sstruct\n" % (depth*2, ''))
        fp.write("%*s{\n" % (depth*2, ''))
        self.HGenStructDeclList(fp, fdef['fields'], depth+1)
        nmBase.PrettyPrintCols(fp, 0,
          depth*2,  "} %s;" % (cid),
          46,       "///< %s\n" % (fdef['genc'].get('comment', cid[2:])))

      # string type
      elif ftype == 'string':
        self.WriteStructDecl(fp, depth*2, type_spec, cid, const_expr)

      # structure vector member
      elif ftype == 'vector':
        vtype_spec  = fdef['genc']['vtype_spec']
        self.WriteVectortDecl(fp, depth*2, vtype_spec, cid, const_expr)

      # simple type
      elif self.IsSimple(ftype):
        self.WriteStructDecl(fp, depth*2, type_spec, cid, const_expr)

      # derived string type (dimensioning already declared in typedef)
      elif base_ftype == 'string':
        self.WriteStructDecl(fp, depth*2, type_spec, cid, None)

      # derived vector type (dimensioning already declared in typedef)
      elif base_ftype == 'vector':
        self.WriteStructDecl(fp, depth*2, type_spec, cid, None)

      # other type
      else:
        self.WriteStructDecl(fp, depth*2, type_spec, cid, const_expr)
  ##

  #--
  def HGenTypedefMacros(self, fp, xid, xdef):
    """ Generation .h cpp #defines macros associated with field or message
        typedefs.

        Parameters:
          fp      - Opened output file pointer.
          xid     - Unique id in XML database section.
          xdef    - XML ftypedef, msgdef, or fielddef XML database entry.
    """
    if self.IsDeprecated(xdef):
      return

    for macro in ['macro_len']: # add others as needed
      if xdef['genc'].has_key(macro) and xdef['genc'][macro]['token_seq']:
        fp.write("/*! %s */\n" % (xdef['genc'][macro]['comment']))
        fp.write("#define %s %s\n\n" % \
            (xdef['genc'][macro]['name'], xdef['genc'][macro]['token_seq']))

    if xdef['ftype'] == 'struct':
      for fname in xdef['fields'][nmBase.NMKeyOrder]:
        self.HGenTypedefMacros(fp, fname, xdef['fields'][fname])
  ##

  #--
  def HGenExternData(self, fp):
    """ Generate .h header public external data.

        Parameters:
          fp    - Opened output file pointer.
    """
    ns = self.mXml['meta']['ns']
    self.PrettyPrintMajorDivComment(fp, "External Data")
    fp.write("""\
//
// %s Message Definition Look-Up Table.
// (indexed by %sMsgId_T enum)
// 
""" % (ns, ns))
    fp.write("extern const NMMsgDef_T *%sMsgDefLookupTbl[];\n\n" % (ns))

    fp.write("""\
//
// %s Maximum Message Body Length (bytes) Look-Up Table.
// (indexed by %sMsgId_T enum)
// 
""" % (ns, ns))
    fp.write("extern size_t %sMsgMaxLenLookupTbl[];\n\n" % (ns))
  ##

  #--
  def HGenProtoTypes(self, fp):
    """ Generate .h header function prototypes.

        Parameters:
          fp    - Opened output file pointer.
    """
    ns            = self.mXml['meta']['ns']
    encoding_pre  = self.mXml['genc']['encoding_prefix']
    endian_desc   = self.mXml['genc']['endian_desc']
    endian_enum   = self.mXml['genc']['endian_enum']

    self.PrettyPrintMajorDivComment(fp, "Function Prototypes")

    self.WriteFuncDef(fp, 'const', 'NMMsgDef_T *', "%sLookupMsgDef" % (ns),
          [(None, "%sMsgId_T" % (ns), "eMsgId")])

    self.WriteFuncDef(fp, None, 'size_t', "%sLookupMsgMaxLen" % (ns),
          [(None, "%sMsgId_T" % (ns), "eMsgId")])

    self.WriteFuncDef(fp, None, 'int', "%sPackMsg" % (ns),
          [(None, "%sMsgId_T" % (ns), "eMsgId"),
          (None, "void *", "pStruct"),
          (None, "byte_t", "buf[]"),
          (None, "size_t", "bufSize"),
          (None, "bool_t", "bTrace")])

    self.WriteFuncDef(fp, None, 'int', "%sUnpackMsg" % (ns),
          [(None, "%sMsgId_T" % (ns), "eMsgId"),
          (None, "byte_t", "buf[]"),
          (None, "size_t", "uMsgLen"),
          (None, "void *", "pStruct"),
          (None, "bool_t", "bTrace")])

    for msgid in self.mXml['msg_types'][nmBase.NMKeyOrder]:
      msgdef = self.mXml['msg_types'][msgid]
      if self.IsDeprecated(msgdef) or msgdef['genc']['active_cnt'] == 0:
        continue
      self.WriteFuncDef(fp, 'INLINE_IN_H', 'int', "%sPack%s" % (ns, msgid),
          [(None, "%s%s_T *" % (ns, msgid), "pStruct"),
          (None, "byte_t", "buf[]"),
          (None, "size_t", "bufSize"),
          (None, "bool_t", "bTrace")],
          stmts="""\
return %sPackMsg(%sMsgId%s, pStruct, buf, bufSize, bTrace);""" % \
            (ns, ns, msgid),
          comments="""\
\\brief Pack a %s%s %s message in %s byte order
       into the output buffer.

\\param [in] pStruct    Pointer to the associated, populated message
                       structure.
\\param [out] buf       Output message buffer.
\\param bufSize         Size of output buffer.
\\param bTrace          Do [not] trace packing.

\\return
On success, returns the number of bytes packed.
On error, returns the appropriate \\h_lt 0 negated NM_ECODE.""" % \
        (ns, msgid, encoding_pre, endian_desc))

      self.WriteFuncDef(fp, 'INLINE_IN_H', 'int', "%sUnpack%s" % (ns, msgid),
          [(None, "byte_t", "buf[]"),
          (None, "size_t", "uMsgLen"),
          (None, "%s%s_T *" % (ns, msgid), "pStruct"),
          (None, "bool_t", "bTrace")],
          stmts="""\
return %sUnpackMsg(%sMsgId%s, buf, uMsgLen, pStruct, bTrace);""" % \
              (ns, ns, msgid),
          comments="""\
\\brief Unpack a %s%s %s message in %s byte order
       from the input buffer.

\\param [in] buf        Output message buffer.
\\param uMsgLen         Length of message (bytes) in input buffer.
\\param [out] pStruct   Pointer to the associated message structure.
\\param bTrace          Do [not] trace packing.

\\return
On success, returns the number of bytes unpacked.
On error, returns the appropriate \\h_lt 0 negated NM_ECODE.""" % \
    (ns, msgid, encoding_pre, endian_desc))
  ##

  #--
  def HGenEpilogue(self, fp):
    """ Generate .h header file epilogue.

        Parameters:
          fp    - Opened output file pointer.
    """
    hdef      = "_" + self.mHBaseName.replace(".", "_").upper()
    epilogue  = self.mXml['meta']['h']['epilogue']
    fp.write(
"""
C_DECLS_END
""")
    if epilogue:
      fp.write("\n\n%s" % (epilogue))
    fp.write(
"""

#endif // %s\n""" % (hdef))
  ##


  #.............................................................................
  # C File Generator Functions
  #.............................................................................

  #--
  def CGenPrologue(self, fp):
    """ Generate .c source file prologue.

        Prologue includes core header file includes plus any prologue specified
        in the XML file.

        Parameters:
          fp    - Opened output file pointer.
    """
    incpre = self.mIncPrefix.strip()
    if len(incpre) > 0 and incpre[-1] != os.path.sep:
      incpre += os.path.sep
    prologue = self.mXml['meta']['c']['prologue']
    fp.write(
"""

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/netmsgs.h"

#include "%s%s"

#ifndef EOFDEF
/*! End of Field Definition entry. */
#define EOFDEF {NULL, 0, NMFTypeNone, 0, }
#endif // EOFDEF

""" % (incpre, self.mHBaseName))
    if prologue:
      fp.write("%s\n\n" % (prologue))
  ##

  #--
  def CGenFieldTypes(self, fp):
    """ Generate .c field and message definitions from 'field_types' section.

        Parameters:
          fp    - Opened output file pointer.
    """
    ns = self.mXml['meta']['ns']  # namespace
    for ftid in self.mXml['field_types'][nmBase.NMKeyOrder]:
      ftdef = self.mXml['field_types'][ftid]
      ftype = ftdef['ftype']
      if ftype in ['struct', 'vector']:
        self.PrettyPrintMinorDivComment(fp, "Field Type %s%s Definition" % \
          (ns, ftid))
        self.CGenDefsR(fp, [ftid], ftdef)
  ##

  #--
  def CGenMsgTypes(self, fp):
    """ Generate .c field and message definitions from 'msg_types' section.

        Parameters:
          fp    - Opened output file pointer.
    """
    ns = self.mXml['meta']['ns']  # namespace
    for msgid in self.mXml['msg_types'][nmBase.NMKeyOrder]:
      msgdef = self.mXml['msg_types'][msgid]
      if not self.IsDeprecated(msgdef):
        self.PrettyPrintMinorDivComment(fp, "Message %s%s Definition" % \
            (ns, msgid))
        self.CGenDefsR(fp, [msgid], msgdef)
  ##

  #--
  def CGenDefsR(self, fp, pathXids, xdef):
    """ Recursively generate .c message related data C definition statements.
    
        Data generated:
          field ID enum declarations
          field definition arrays
          message/struct definitions

        Parameters:
          fp        - Opened output file pointer.
          pathXids  - Path list of XML ftypedef, msgdef, or fielddef
                      identifiers.
          xdef      - XML ftypedef, msgdef, or fielddef XML specification.
    """
    if xdef['ftype'] == 'struct':
      for fname in xdef['fields'][nmBase.NMKeyOrder]:
        self.CGenDefsR(fp, pathXids+[fname], xdef['fields'][fname])
      self.CGenFIdEnum(fp, pathXids, xdef)
      self.CGenAnonStruct(fp, pathXids, xdef)
      self.CGenFieldDefs(fp, pathXids, xdef)
      self.CGenMsgDef(fp, pathXids, xdef)
    elif xdef['ftype'] == 'vector':
      self.CGenVectorItemFieldDef(fp, pathXids, xdef)
  ##

  #--
  def CGenFIdEnum(self, fp, pathXids, xdef):
    """ Generate .c field id enum declaration for the given definition.

        Parameters:
          fp        - Opened output file pointer.
          pathXids  - Path list of XML identifiers.
          xdef      - XML ftypedef, msgdef, or fielddef XML specification.
    """
    ns      = self.mXml['meta']['ns']   # namespace
    fields  = xdef['fields']            # field
    nsname  = ns + ''.join(pathXids)    # global name substring
    self.PrettyPrintBriefComment(fp, "%s Field Id Enumeration" % (nsname))
    fp.write("typedef enum\n{\n")
    enum = 0
    nmBase.PrettyPrintCols(fp, 0,
          2,  "%sFIdReserved" % (nsname), 
          40, "= %d,"  % (enum),
          48, "///< reserved field id\n",
          force=True)
    enum += 1
    for fname in fields[nmBase.NMKeyOrder]:
      fdef = fields[fname]
      if fdef['ftype'] == 'pad':
        fname = '_pad' + repr(enum)
      if not self.IsDeprecated(fdef):
        comment = "%s field id" % (fname)
      else:
        comment = "%s field id (deprecated)" % (fname)
      nmBase.PrettyPrintCols(fp, 0,
          2,  "%sFId%s" % (nsname, fname), 
          40, "= %d,"  % (enum),
          48, "///< %s\n" % (comment),
          force=True)
      enum += 1
    nmBase.PrettyPrintCols(fp, 0,
        2,  "%sFIdNumOf" % (nsname),
        40, "= %d" % (enum),
        48, "///< number of fields\n",
        force=True)
    fp.write("} %sFId_T;\n\n" % (nsname))
  ##

  #--
  def CGenAnonStruct(self, fp, pathXids, xdef):
    """ Generate .c anonomous structure type.

        Anonomous structures mirror any inline, non-global 'struct' field
        types. External structures are required to calculate member offsets
        correctlhy.

        Parameters:
          fp        - Opened output file pointer.
          pathXids  - Path list of XML identifiers.
          xdef      - XML ftypedef, msgdef, or fielddef XML specification.
    """
    ftype   = xdef['ftype']             # (derivied) XML field type

    if self.IsDeprecated(xdef):                   # deprecated element
      return
    elif len(pathXids) <= 1:                      # global struct defined in .h
      return
    elif self.mXml['field_types'].has_key(ftype): # typedef struct defined in .h
      return
    if xdef['genc']['active_cnt'] == 0:
      return

    ns      = self.mXml['meta']['ns']   # namespace
    xname   = ''.join(pathXids)         # XML full concatenated name
    nsname  = ns + xname                # global name substring
    fields  = xdef['fields']            # fields

    anon_type_spec = '_' + xname + '_T' # anonomum structure type specifier

    self.PrettyPrintBriefComment(fp,
          "%s Anonumous Struture Type" % (' '.join(pathXids)))
    fp.write("typedef struct\n{\n")
    self.HGenStructDeclList(fp, xdef['fields'], 1)
    fp.write("} %s;\n\n" %  (anon_type_spec))

    # augment database
    xdef['genc']['anon_type_spec'] = anon_type_spec
  ##

  #--
  def CGenFieldDefs(self, fp, pathXids, xdef):
    """ Generate .c field definition array for the given definition.

        Parameters:
          fp        - Opened output file pointer.
          pathXids  - Path list of XML identifiers.
          xdef      - XML ftypedef, msgdef, or fielddef XML specification.
    """
    ns      = self.mXml['meta']['ns']   # namespace
    fields  = xdef['fields']            # fields
    nsname  = ns + ''.join(pathXids)    # global name substring
    pid     = xdef['genc']['id']        # parent C identifer
    eofdef  = "EOFDEF"                  # end of field def macro
    enum    = 1                         # field enum

    struct_id = xdef['genc'].get('anon_type_spec', pid)

    self.PrettyPrintBriefComment(fp, "%s Field Definitions" % (nsname))
    fp.write("static const NMFieldDef_T %sFieldDefs[] =\n{\n" % (nsname))
    for fname in fields[nmBase.NMKeyOrder]:
      fdef        = fields[fname]       # XML field definition
      ftype       = fdef['ftype']       # field type
      base_ftype  = fdef['base_ftype']  # base field type
      cid         = fdef['genc']['id']  # C identifier

      if self.IsDeprecated(fdef):
        continue

      # pad 'field'
      if ftype == 'pad':
        sFName  = '_pad'+repr(enum) 
        eFId      = "%sFId%s" % (nsname, sFName)
        eFType    = "NMFType%s" % (base_ftype.capitalize())
        uOffset = "(size_t)0"
      else:
        sFName    = fname
        eFId      = "%sFId%s" % (nsname, fname)
        eFType    = "NMFType%s" % (base_ftype.capitalize())
        uOffset   = "memberoffset(%s, %s)" % (struct_id, cid)

      this, this_init = self.CMakeFieldDefThis(ftype, fdef)

      self.WriteFDefEntry(fp, sFName, eFId, eFType, uOffset, this, this_init)
      enum += 1

    nmBase.PrettyPrintCols(fp, 0, 2,  eofdef+'\n')
    fp.write("};\n\n")
  ##

  #--
  def CGenVectorItemFieldDef(self, fp, pathXids, xdef):
    """ Generate .c vector item field definition array for the given definition.

        Parameters:
          fp        - Opened output file pointer.
          pathXids  - Path list of XML identifiers.
          xdef      - XML ftypedef, msgdef, or fielddef XML specification.
    """
    ns          = self.mXml['meta']['ns'] # namespace
    fname       = pathXids[-1]            # XML field name
    nsname      = ns + ''.join(pathXids)  # global name substring
    eofdef      = "EOFDEF"                # end of field def macro
    ftype       = xdef['ftype']           # field type
    vtype       = xdef['vtype']           # vector item field type
    cid         = xdef['genc']['id']      # C identifier
    base_vtype  = xdef['genc']['base_vtype'] # base vector item field type
    base_vname  = xdef['genc']['base_vname'] # base vector item base name

    if self.IsDeprecated(xdef):
      return

    if base_vname == 'this':
      base_fdef   = xdef
    else:
      base_vname  = xdef['genc']['base_vname']
      base_fdef   = self.mXml['field_types'][base_vname]

    fielddef  = "%sFieldDef" % (nsname)
    sFName    = fname
    eFId      = "0"
    eFType    = "NMFType%s" % (base_vtype.capitalize())
    uOffset   = "(size_t)0"

    this, this_init = self.CMakeFieldDefThis(base_vtype, base_fdef)

    self.PrettyPrintBriefComment(fp, "%s Field Definitions" % (nsname))
    fp.write("static const NMFieldDef_T %s[] =\n{\n" % (fielddef))
    self.WriteFDefEntry(fp, sFName, eFId, eFType, uOffset, this, this_init)
    nmBase.PrettyPrintCols(fp, 0, 2,  eofdef+'\n')
    fp.write("};\n\n")
  ##

  #--
  def CMakeFieldDefThis(self, ftype, fdef):
    """ Make C field definition this specific initialization for field type.

        Parameters:
          ftype   - XML field type.
          fdef    - XML field definition.

        Return
          Returns the pair (this, this_init) where:
            this      - C union name.
            this_int  - List of (.member, value) pairs.
    """
    nothis_ftype = ['bool', 'char', 'p32', 'p64']

    # pad 'field'
    if ftype == 'pad':
      return self.CMakeFieldDefThisPad(ftype, fdef)

    # simple number field
    elif ftype in nmBase.NMFTypeNumber:
      return self.CMakeFieldDefThisNumber(ftype, fdef)

    # field with no extra infor
    elif ftype in nothis_ftype:
      return '', []

    # string field
    elif ftype == 'string':
      return self.CMakeFieldDefThisString(ftype, fdef)

    # structure field
    elif ftype == 'struct':
      return self.CMakeFieldDefThisStruct(ftype, fdef)

    # vector field
    elif ftype == 'vector':
      return self.CMakeFieldDefThisVector(ftype, fdef)

    # derived type
    else:
      return self.CMakeFieldDefThisDerived(ftype, fdef)
  ##

  #--
  def CMakeFieldDefThisPad(self, ftype, fdef):
    """ Make C field definition this specific initialization for pad 'field'.

        Parameters:
          ftype   - XML field type.
          fdef    - XML field definition.

        Return
          Returns the pair (this, this_init) where:
            this      - C union name.
            this_int  - List of (.member, value) pairs.
    """
    this    = "m_pad"
    this_init = [('m_uCount', "(byte_t)(%s)" % fdef.get('count', "1"))]
    return this, this_init
  ##

  #--
  def CMakeFieldDefThisNumber(self, ftype, fdef):
    """ Make C field definition this specific initialization for number field.

        Parameters:
          ftype   - XML field type.
          fdef    - XML field definition.

        Return
          Returns the pair (this, this_init) where:
            this      - C union name.
            this_int  - List of (.member, value) pairs.
    """
    this_init = []    # this initialization list
    bits      = ""    # present bits
    bitssep   = ""    # bits separator

    if ftype[0] == 's':
      this      = 'm_u' + ftype[1:]
      casttype  = nmBase.NMBuiltInFieldTypes['u'+ftype[1:]]['T']
    else:
      this      = 'm_' + ftype
      casttype  = nmBase.NMBuiltInFieldTypes[ftype]['T']
    if fdef.has_key('min'):
      val     = fdef['min']
      bits   += "%sNMBITS_HAS_MIN" % (bitssep)
      bitssep = "|"
    else:
      val     = '0'
    this_init += [('m_valMin', "(%s)(%s)" % (casttype, val))]

    if fdef.has_key('max'):
      val     = fdef['max']
      bits   += "%sNMBITS_HAS_MAX" % (bitssep)
      bitssep = "|"
    else:
      val     = '0'
    this_init += [('m_valMax', "(%s)(%s)" % (casttype, val))]

    if fdef.has_key('const'):
      val     = fdef['const']
      bits   += "%sNMBITS_HAS_CONST" % (bitssep)
      bitssep = "|"
    else:
      val     = '0'
    this_init += [('m_valConst', "(%s)(%s)" % (casttype, val))]

    if not bits:
      bits = "0"
    this_init = [('m_bits', "(byte_t)(%s)" % bits)] + this_init

    return this, this_init
  ##

  #--
  def CMakeFieldDefThisString(self, ftype, fdef):
    """ Make C field definition this specific initialization for string field.

        Parameters:
          ftype   - XML field type.
          fdef    - XML field definition.

        Return
          Returns the pair (this, this_init) where:
            this      - C union name.
            this_int  - List of (.member, value) pairs.
    """
    this      = 'm_string'
    this_init = []    # this initialization list
    casttype  = nmBase.NMBuiltInFieldTypes[ftype]['T']

    val = fdef['genc']['const_expr']
    this_init += [('m_uMaxCount', "(size_t)%s" % (val))]

    if fdef.has_key('const'):
      val     = fdef['const']
    else:
      val     = 'NULL'
    this_init += [('m_sConst', "(%s *)(%s)" % (casttype, val))]

    return this, this_init
  ##

  #--
  def CMakeFieldDefThisStruct(self, ftype, fdef):
    """ Make C field definition this specific initialization for struct field.

        Parameters:
          ftype   - XML field type.
          fdef    - XML field definition.

        Return
          Returns the pair (this, this_init) where:
            this      - C union name.
            this_int  - List of (.member, value) pairs.
    """
    this      = 'm_struct'
    this_init = [('', "&%s" % (fdef['genc']['msgdef']))]
    return this, this_init
  ##

  #--
  def CMakeFieldDefThisVector(self, ftype, fdef):
    """ Make C field definition this specific initialization for vector field.

        Parameters:
          ftype   - XML field type.
          fdef    - XML field definition.

        Return
          Returns the pair (this, this_init) where:
            this      - C union name.
            this_int  - List of (.member, value) pairs.
    """
    this      = 'm_vector'
    this_init = []          # this initialization list

    val = fdef['genc']['const_expr']
    this_init += [('m_uMaxCount', "(size_t)%s" % (val))]
    this_init += [('m_uElemSize',
                    "sizeof(%s)" % (fdef['genc']['vtype_spec']))]
    this_init += [('m_pThisElem', (fdef['genc']['fielddef']))]

    return this, this_init
  ##

  #--
  def CMakeFieldDefThisDerived(self, ftype, fdef):
    """ Make C field definition this specific initialization for vector field.

        Parameters:
          ftype   - XML field type.
          fdef    - XML field definition.

        Return
          Returns the pair (this, this_init) where:
            this      - C union name.
            this_int  - List of (.member, value) pairs.
    """
    base_ftype  = fdef['base_ftype']  # base field type
    if base_ftype in nmBase.NMFTypeNumber:
      return self.CMakeFieldDefThisNumber(base_ftype, fdef)
    elif base_ftype == 'string':
      return self.CMakeFieldDefThisString(base_ftype, fdef)
    elif base_ftype == 'struct':
      return self.CMakeFieldDefThisStruct(base_ftype, fdef)
    elif base_ftype == 'vector':
      return self.CMakeFieldDefThisVector(base_ftype, fdef)
    else:
      return ('', [])
  ##

  #--
  def CGenMsgDef(self, fp, pathXids, xdef):
    """ Generate .c source file [sub]message definition from the given
        definition.

        Parameters:
          fp        - Opened output file pointer.
          pathXids  - Path list of XML identifiers.
          xdef      - XML ftypedef, msgdef, or fielddef XML specification.
    """
    ns      = self.mXml['meta']['ns']   # namespace
    xname   = ''.join(pathXids)         # XML full concatenated name
    nsname  = ns + xname                # global name substring

    if self.IsDeprecated(xdef):
      return

    # only top-level stucture definitions under 'msg_types' section have
    # message ids.
    if len(pathXids) == 1 and self.mXml['msg_types'].has_key(pathXids[0]):
      msgid_enum = "%sMsgId%s" % (ns, xname), 
    else:
      msgid_enum = "%sMsgIdNone" % (ns), 
    self.PrettyPrintBriefComment(fp, "%s Message Definition" % (nsname))
    fp.write("static const NMMsgDef_T %sMsgDef =\n{\n" % (nsname))
    nmBase.PrettyPrintCols(fp, 0,
          2,  ".m_sMsgName",
          22, "=",
          24,  "\"%s\"," % (nsname), 
          2,  ".m_eMsgId",
          22, "=",
          24,  "%s," % (msgid_enum), 
          2,  ".m_uCount",
          22, "=",
          24,  "(size_t)(%d)," % (xdef['genc']['active_cnt']),
          2,  ".m_pFields",
          22, "=",
          24,  "%sFieldDefs\n" % (nsname))
    fp.write("};\n\n")
  ##

  #--
  def CGenLookupTbls(self, fp):
    """ Generate .c source file look-up tables.

        Parameters:
          fp    - Opened output file pointer.
    """
    ns = self.mXml['meta']['ns']  # namespace
    self.PrettyPrintBriefComment(fp,
        "%s Message Definition Look-Up Table. Indexed by %sMsgId_T enum." % \
            (ns, ns))
    fp.write("const NMMsgDef_T *%sMsgDefLookupTbl[] =\n" % ns)
    fp.write("{\n")
    nmBase.PrettyPrintCols(fp, 0,
        2, "NULL,",
        40, "///< [%sMsgIdNone]\n" % (ns))
    for msgid in self.mXml['msg_types'][nmBase.NMKeyOrder]:
      if not self.IsDeprecated(self.mXml['msg_types'][msgid]):
        nmBase.PrettyPrintCols(fp, 0,
          2,  "&%s%sMsgDef," % (ns, msgid),
          40, "///< [%sMsgId%s]\n" % (ns, msgid))
      else:
        nmBase.PrettyPrintCols(fp, 0,
          2,  "NULL,",
          40, "///< [%sMsgId%s]\n" % (ns, msgid))
    nmBase.PrettyPrintCols(fp, 0,
        2,  "NULL",
        40, "///< [%sMsgIdNumOf]\n" % (ns))
    fp.write("};\n\n")
       
    self.PrettyPrintBriefComment(fp,
        "%s Message Maximum Size Look-Up Table. Indexed by %sMsgId_T enum." % \
            (ns, ns))
    fp.write("size_t %sMsgMaxLenLookupTbl[] =\n" % ns)
    fp.write("{\n")
    nmBase.PrettyPrintCols(fp, 0,
        2, "(size_t)(0),",
        40, "///< [%sMsgIdNone]\n" % (ns))
    for msgid in self.mXml['msg_types'][nmBase.NMKeyOrder]:
      nmBase.PrettyPrintCols(fp, 0,
        2,  "(size_t)(%s)," % (self.mXml['msg_types'][msgid]['genc']['maxlen']),
        40, "///< [%sMsgId%s]\n" % (ns, msgid))
    nmBase.PrettyPrintCols(fp, 0,
        2,  "(size_t)(0)",
        40, "///< [%sMsgIdNumOf]\n" % (ns))
    fp.write("};\n\n")
  ##

  #--
  def CGenPublicFuncDefs(self, fp):
    """ Generate .c source file function definitions.

        Parameters:
          fp    - Opened output file pointer.
    """
    ns            = self.mXml['meta']['ns']
    encoding_pre  = self.mXml['genc']['encoding_prefix']
    endian_desc   = self.mXml['genc']['endian_desc']
    endian_enum   = self.mXml['genc']['endian_enum']

    self.PrettyPrintMajorDivComment(fp, "Functions")

    # lookup message definition function
    self.WriteFuncDef(fp, 'const', 'NMMsgDef_T *', "%sLookupMsgDef" % (ns),
          [(None, "%sMsgId_T" % (ns), "eMsgId")],
          stmts="""\
if( (uint_t)eMsgId >= (uint_t)%sMsgIdNumOf )
{
  return NULL;
}
else
{
  return %sMsgDefLookupTbl[(uint_t)eMsgId];
}""" % (ns, ns),
        comments="""\
\\brief Look up the message definition associated with the message id.

\\param eMsgId          Message Id.

\\return
On success, returns the pointer to the NMMsgDef_T.
On error, NULL is returned.""")

    # lookup message maximum lenght function
    self.WriteFuncDef(fp, None, 'size_t', "%sLookupMsgMaxLen" % (ns),
          [(None, "%sMsgId_T" % (ns), "eMsgId")],
          stmts="""\
if( (uint_t)eMsgId >= (uint_t)%sMsgIdNumOf )
{
  return (size_t)0;
}
else
{
  return %sMsgMaxLenLookupTbl[(uint_t)eMsgId];
}""" % (ns, ns),
        comments="""\
\\brief Look up the message maximum length associated with the message id.

The maximum length is the total number of packed bytes possible for the
given message. The message may be much shorter.

\\param eMsgId          Message Id.

\\return
On success, returns the number of bytes.
On error, 0 is returned.""")

    # packing function
    self.WriteFuncDef(fp, None, 'int', "%sPackMsg" % (ns),
          [(None, "%sMsgId_T" % (ns), "eMsgId"),
          (None, "void *", "pStruct"),
          (None, "byte_t", "buf[]"),
          (None, "size_t", "bufSize"),
          (None, "bool_t", "bTrace")],
          stmts="""\
const  NMMsgDef_T  *pMsgDef;

if( (pMsgDef = %sLookupMsgDef(eMsgId)) == NULL )
{
  LOGERROR("%%s(ecode=%%d): msgid=%%u.", 
            nmStrError(NM_ECODE_MSGID), NM_ECODE_MSGID, eMsgId);
  return -NM_ECODE_MSGID;
}

if( bTrace )
{
  return nmPack%sMsgDebug(pMsgDef, pStruct, buf, bufSize, %s);
}
else
{
  return nmPack%sMsg(pMsgDef, pStruct, buf, bufSize, %s);
}""" % (ns, encoding_pre, endian_enum, encoding_pre, endian_enum),
        comments="""\
\\brief Pack a %s message in %s byte order.

\\param eMsgId          Message Id.
\\param [in] pStruct    Pointer to the associated, populated message
                       structure.
\\param [out] buf       Output message buffer.
\\param bufSize         Size of output buffer.
\\param bTrace          Do [not] trace packing.

\\return
On success, returns the number of bytes packed.
On error, returns the appropriate \\< 0 negated NM_ECODE.""" % \
    (encoding_pre, endian_desc))

    # unpacking function
    self.WriteFuncDef(fp, None, 'int', "%sUnpackMsg" % (ns),
          [(None, "%sMsgId_T" % (ns), "eMsgId"),
          (None, "byte_t", "buf[]"),
          (None, "size_t", "uMsgLen"),
          (None, "void *", "pStruct"),
          (None, "bool_t", "bTrace")],
          stmts="""\
const  NMMsgDef_T  *pMsgDef;

if( (pMsgDef = %sLookupMsgDef(eMsgId)) == NULL )
{
  LOGERROR("%%s(ecode=%%d): msgid=%%u.", 
            nmStrError(NM_ECODE_MSGID), NM_ECODE_MSGID, eMsgId);
  return -NM_ECODE_MSGID;
}

if( bTrace )
{
  return nmUnpack%sMsgDebug(pMsgDef, buf, uMsgLen, pStruct, %s);
}
else
{
  return nmUnpack%sMsg(pMsgDef, buf, uMsgLen, pStruct, %s);
}""" % (ns, encoding_pre, endian_enum, encoding_pre, endian_enum),
        comments="""\
\\brief Unpack a %s message in %s byte order.

\\param eMsgId          Message Id.
\\param [in] buf        Input message buffer.
\\param uMsgLen         Length of message (bytes) in input buffer.
\\param [out] pStruct   Pointer to the associated message structure.
\\param bTrace          Do [not] trace packing.

\\return
On success, returns the number of bytes unpacked.
On error, returns the appropriate \\< 0 negated NM_ECODE.""" % \
    (encoding_pre, endian_desc))
  ##

  #--
  def CGenEpilogue(self, fp):
    """ Generate .c source file epilogue.

        Parameters:
          fp    - Opened output file pointer.
    """
    epilogue  = self.mXml['meta']['c']['epilogue']
    if epilogue:
      fp.write("\n\n%s" % (epilogue))
  ##


  #.............................................................................
  # CPP and C Statement Utilities
  #.............................................................................

  #--
  def MakeCppMacroName(self, *names):
    """ Make CPP #define macro name.

        Parameters:
          names   - List of argument subnames.

        Returns:
          String "NS_ARG1_..._ARGN"
    """
    defname = self.mXml['meta']['ns']
    for n in names:
      defname += '_' + n
    return defname.upper()
  ##
 
  #--
  def MakeTypeSpec(self, ftype):
    """ Make C type specifier.

        Parameters:
          ftype   - Field type.

        Returns:
          Type specifier.
    """
    if nmBase.NMBuiltInFieldTypes.has_key(ftype):
      type_spec = nmBase.NMBuiltInFieldTypes[ftype]['T']
    else:
      type_spec = self.mXml['meta']['ns'] + ftype + '_T'
    return type_spec
  ##

  #--
  def MakeCId(self, xid, isGlobal):
    """ Make C indentifier.

        Parameters:
          xid   - XML [sub]definition id.
          isGlobal  - Identifier does [not] have global scope.

        Returns:
          C identifier.
    """
    if isGlobal:
      cid = self.mXml['meta']['ns'] + xid + '_T'
    else:
      pre = 'm_'
      if self.mHungarian: pre += nmBase.NMBuiltInFieldTypes[ftype]['pre']
      cid = pre + xid
    return cid

  #--
  def WriteStructDecl(self, fp, indent, type_spec, cid, const_expr, **kwargs):
    """ Write commented structure declarator.

        struct_decl ::-
            type_spec cid ;
          | type_spec cid [ const_expr ] ;

        Parameters:
          fp          - Opened output file pointer.
          indent      - Indentation.
          type_spec   - C type specifier.
          cid         - C identifier.
          const_expr  - C array constant expression. None of not an array.
          kwargs      - Optional keyword arguments.
                          'comment':string  - Overide default comment.
    """
    if const_expr:
      post = '[' + const_expr + ']'
    else:
      post = ''
    comment = kwargs.get('comment', cid[2:])
    nmBase.PrettyPrintCols(fp, 0,
        indent, type_spec,
        26,     cid+post+';',
        46,     "///< %s\n" % (comment))
  ##

  #--
  def WriteVectortDecl(self, fp, indent, vtype_spec, cid, const_expr):
    """ Write commented structure vector declarator.

        struct_vector_decl ::-
            struct {
              size_t m_count ;
              union { 
                void * m_pAlign ;
                vtype_spec  m_buf [ const_expr ] ;
              } u ;
            } cid ;

        Parameters:
          fp          - Opened output file pointer.
          indent      - Indentation.
          vtype_spec  - C vector items type specifier.
          cid         - C identifier.
          const_expr  - C array constant expression. None of not an array.
          kwargs      - Optional keyword arguments.
                          'comment':string  - Overide default comment.
    """
    fp.write("%*s%s\n" % (indent, '', 'struct'))
    fp.write("%*s{\n" % (indent, ''))
    self.WriteStructDecl(fp, indent+2, 'size_t', 'm_count', None,
        comment="vector item count")
    fp.write("%*s%s\n" % (indent+2, '', 'union'))
    fp.write("%*s{\n" % (indent+2, ''))
    self.WriteStructDecl(fp, indent+4, 'void', '*m_pAlign', None,
        comment="force alignment")
    self.WriteStructDecl(fp, indent+4, vtype_spec, 'm_buf', const_expr,
        comment="the item vector")
    fp.write("%*s} u; ///< aligned vector items\n" % (indent+2, ''))
    nmBase.PrettyPrintCols(fp, 0,
        indent, "} %s;" % (cid),
        46,     "///< vector\n")
  ##

  #--
  def WriteTypedefStmt(self, fp, type_spec, cid, const_expr):
    """ Write typedef declaration.

        typedef_spec ::-
            typedef type_spec cid ;
          | typedef type_spec cid [ const_expr ] ;

        Parameters:
          fp          - Opened output file pointer.
          type_spec   - C type specifier.
          cid         - C identifier.
          const_expr  - C array constant expression. None of not an array.
    """
    if const_expr:
      post = '[' + const_expr + ']'
    else:
      post = ''
    fp.write("typedef %s %s%s;\n" % (type_spec, cid, post))
  ##

  #--
  def WriteFuncDef(self, fp, type_qual, type_spec, funcname, funcargs,
                        **kwargs):
    """ Write function declaration statement.

        func_def ::-
            extern type_qual type_spec funcname ( arg_list ) ;
          | extern type_spec funcname ( arg_list ) ;
          | type_qual type_spec funcname ( arg_list ) { stmts }
          | type_spec funcname ( arg_list ) { stmts }

        arg_list ::-
            empty
          ! arg_decl
          | arg_list , arg_decl

        arg_decl ::-
            type_qual type_spec cid 
            type_spec cid

        Parameters:
          fp          - Opened output file pointer.
          type_qual   - C function type qualifier(s).
          type_spec   - C function type specifier(s).
          funcname    - C function identifier.
          funcargs    - List of function arguments 3-tuple:
                          (type_qual(s), type_spec(s), cid)
          kwargs      - Dictionary of optional arguments.
                          'comments': string
                          'stmts': string
    """
    comments  = kwargs.get('comments', '')
    stmts     = kwargs.get('stmts', '')
    if type_qual:
      s = "%s %s %s(" % (type_qual, type_spec, funcname)
    else:
      s = "%s %s(" % (type_spec, funcname)
    if not stmts:
      s = 'extern ' + s
    if comments:
      fp.write("/*!\n")
      for l in comments.splitlines():
        fp.write(" * %s\n" % l)
      fp.write(" */\n")
    fp.write(s)
    cursor = len(s)
    indent = cursor + 1
    i = 1
    n = len(funcargs)
    sep = ',\n'
    for arg3tuple in funcargs:
      if i == n:
        sep = ''
      if arg3tuple[0]:
        s = "%s %s %s%s" % (arg3tuple[0], arg3tuple[1], arg3tuple[2], sep)
      else:
        s = "%s %s%s" % (arg3tuple[1], arg3tuple[2], sep)
      nmBase.PrettyPrintCols(fp, cursor, indent, s, force=True)
      cursor = 0
      i += 1
    if stmts:
      fp.write(" )\n")
      fp.write("{\n")
      for stmt in stmts.splitlines():
        fp.write("  %s\n" % stmt)
      fp.write("}\n\n")
    else:
      fp.write(" );\n\n")
  ###

  #--
  def WriteFDefEntry(self, fp, sFName, eFId, eFType, uOffset, this, this_init):
    """ Write an NMFieldDef_T entry.

        Parameters:
          fp          - Opened output file pointer.
          sFName      - m_sFName value.
          eFId        - m_eFId value.
          eFType      - m_eFType value.
          uOffset     - m_uOffset value.
          this        - m_this.<this>
          this_init   - List of (member, value) this pairs.
    """
    fp.write("  {\n")
    nmBase.PrettyPrintCols(fp, 0,
          4,  ".m_sFName",
          30, "=",
          32,  "\"%s\"," % (sFName), 
          4,  ".m_eFId",
          30, "=",
          32,  "%s," % (eFId), 
          4,  ".m_eFType",
          30, "=",
          32,  "%s," % (eFType), 
          4,  ".m_uOffset",
          30, "=",
          32,  "%s,\n" % (uOffset)) 
    for dotassign in this_init:
      if dotassign[0]:
        this_mem = '.' + dotassign[0]
      else:
        this_mem = ''
      this_val = dotassign[1]
      nmBase.PrettyPrintCols(fp, 0,
          4,  ".m_this.%s%s" % (this, this_mem),
          30, "=",
          32, "%s,\n" % this_val)
    fp.write("  },\n")
  ##

  #--
  def IsSimple(self, ftype):
    """ Returns True (False) is field type is (not) simple.

        Parameters:
          ftype   - (Derived) field type.

        Return:
          True or False.
    """
    ftype = self.BaseFType(ftype)
    if ftype == 'pad':
      return False
    elif nmBase.NMBuiltInFieldTypes[ftype]['comp'] == 'simple':
      return True
    else:
      return False
  ##

  #--
  def IsDeprecated(self, xdef):
    """ Returns True (False) is definition is (not) deprecated.

        Parameters:
          xdef      - XML ftypedef, msgdef, or fielddef XML specification.

        Return:
          True or False.
    """
    dispo = xdef.get('disposition', 'active')
    if dispo == 'deprecated':
      return True
    else:
      return False
  ##

  #--
  def BaseXRef(self, fname):
    """ Returns the base field XML cross-reference name for the given XML
        name.

        Parameters:
          fname   - (Derived) field name.

       Return:
          Returns 'this' if fname is a base field, otherwise the XML
          cross-referenced field name is returned.
    """
    if nmBase.NMBuiltInFieldTypes.has_key(fname):
      return 'this'
    ftype = self.mXml['field_types'][fname]['ftype']
    while not nmBase.NMBuiltInFieldTypes.has_key(ftype):
      fname = ftype
      ftype = self.mXml['field_types'][fname]['ftype']
    return fname
  ##

  #--
  def BaseFType(self, ftype):
    """ Returns the base field type for the given field type.

        Parameters:
          ftype   - (Derived) field type.

       Return:
          Base field type.
    """
    if ftype == 'pad':
      return ftype
    while not nmBase.NMBuiltInFieldTypes.has_key(ftype):
      if self.mXml['field_types'].has_key(ftype):
        ftype = self.mXml['field_types'][ftype]['ftype']
      else:
        raise nmBase.NetMsgsError("Error: %s is not a defined ftype" % (ftype))
    return ftype
  ##

  #--
  def EvalNum(self, *args):
    """ Evaluate a summation of terms.

        Paramters:
          args      List of summation terms, with each term one of:
                      an equation, string, number

        Examples:
          5                     -> '5'
          5, '(3 * 10)'         -> '35'
          5, 'MY_MAX', '1*255'  -> '300+MY_MAX'

        Return:
          Returns string of the evaluated results.
    """
    #n = 0
    #s = ''
    expr = ''
    for arg in args:
    #  if arg is None:
    #    continue
      if type(arg) == str:
        arg = arg.replace("NMFVAL_LEN_MAX_STRING",
                          repr(nmBase.NMStringMaxCount))
        arg = arg.replace("NMFVAL_LEN_MAX_VECTOR",
                          repr(nmBase.NMVectorMaxCount))
      else:
        arg = repr(arg)
      if expr:
        expr = expr + '+' + arg
      else:
        expr = arg
    return self._Simplify(expr)

    #  try:
    #    n += eval(arg)
    #  except:
    #    s += '+(' + arg + ')'
    #n = int(n)
    #return repr(n) + s
  ##

  #--
  def _Simplify(self, expr):
    """ Simplify algebraic expression. The simplification is very simple.
  
        The distribution law is not applied.
  
        Parameters:
          expr    - Algebraic expression in string format.
  
        Return:
          Simplified algebraic expression in string format.
    """
    expr  = expr.replace(' ', '')
    tree  = self._MakeTree(expr)
    val   = self._EvalTree(tree)
    if type(val) == str:
      return val
    else:
      return repr(val)
  ##
  
  #--
  def _EvalTree(self, tree):
    """ Evaluate algebraic expression in a tree format.
    
        tree: expr
              [lexpr, op, rexpr]
  
        Only '+' and '*' operators are supported.
  
        Parameters:
          tree    - Algebraic expression tree
  
        Return:
          Evaluated number or string.
    """
    if type(tree) == list:
      lval  = self._EvalTree(tree[0])
      op    = tree[1]
      rval  = self._EvalTree(tree[2])
      if type(lval) == str:
        if type(rval) == str:
          return '('+lval+op+rval+')'
        elif rval == 0:
          if op == '+':
            return lval
          elif op == '*':
            return 0
        elif (rval == 1) and (op == '*'):
          return lval
        else:
          return '('+lval+op+repr(rval)+')'
      elif type(rval) == str:
        if type(lval) == str:
          return '('+lval+op+rval+')'
        elif lval == 0:
          if op == '+':
            return rval
          elif op == '*':
            return 0
        elif (lval == 1) and (op == '*'):
          return rval
        else:
          return '('+repr(lval)+op+rval+')'
      elif op == '+':
        return lval + rval
      elif op == '*':
        return lval * rval
      else: # error
          return '('+repr(lval)+op+repr(rval)+')'
    else:
      try:
        return eval(tree)
      except:
        return tree
  ##
  
  #--
  def _MakeTree(self, expr):
    """ Make expression tree.
    
        tree: expr
              [lexpr, op, rexpr]
  
        Parameters:
          tree    - Algebraic expression tree
  
        Return:
          Expression tree.
    """
    if not expr:
      return ''
    if self._ParenExprParen(expr):
      return self._MakeTree(expr[1:-1])
    else:
      k = self._FindLowOp(expr, 0)
      if k is None:
        return expr
      else:
        return [self._MakeTree(expr[:k]), expr[k], self._MakeTree(expr[k+1:])]
  ##
  
  #--
  def _ParenExprParen(self, expr, start=0):
    """ Test if expression is of form '(' expr ')'
    
        Parameters:
          expr    - Algebraic expression string.
          start   - Start index in expr.
  
        Return:
          True or False.
    """
    if expr[start] != '(':
      return False
    n = self._FindRParen(expr, lparen=start)
    if n == len(expr)-1:
      return True
    else:
      return False
  ##
  
  #--
  def _FindRParen(self, expr, lparen=0):
    """ Find corresponding right parentheses.
  
        Parameters:
          expr    - Algebraic expression string.
          lparen  - Start index in expression of left parenthesis.
  
        Return:
          Right parenthesis index in expr.
    """
    depth = 1
    n = lparen + 1
    for c in expr[lparen+1:]:
      if c == '(':
        depth += 1
      elif c == ')':
        depth -= 1
        if depth == 0:
          return n
      n += 1
    return None
  ##
  
  #--
  def _FindLowOp(self, expr, start=0):
    """ Find operator with lowest precedence in expression.
  
        Only '+' and '*' operators are supported.
  
        Parameters:
          expr    - Algebraic expression string.
          start   - Start index in search.
  
        Return:
          Operator index in expression if found. Else None.
    """
    n = start
    op = ''
    m = 0
    while n < len(expr):
      c = expr[n]
      if c == '+':
        return n
      elif c == '*':
        op = c
        m = n
      elif c == '(':
        n = self._FindRParen(expr, lparen=n)
      n += 1
    if op == '*':
      return m
    else:
      return None
  ##


  #.............................................................................
  # Pretty Printing
  #.............................................................................

  #--
  def PrettyPrintTopComment(self, fp, srcbasename, brief):
    """ Prints source file top comment block.

        Parameters:
          fp          - Opened output file pointer.
          srcbasename - Base name of source file.
          brief       - Brief comment.
    """
    xmlfilename = self.mXml.GetXMLFileName()
    now         = time.localtime()
    thisdate    = "%d.%02d.%02d" % (now.tm_year, now.tm_mon, now.tm_mday)
    thistime    = "%02d:%02d:%02d" % (now.tm_hour, now.tm_min, now.tm_sec)
    fp.write("""\
///////////////////////////////////////////////////////////////////////////////
//
// File: %s
//
/*!
 * \\file
 *
 * \\brief %s
 *
 * \\warning This file was auto-generated on %s %s from the NetMsgs
 * XML specification %s.
 *
 * \\par Copyright:
 * (C) %d. RoadNarrows LLC
 * (http://www.roadnarrows.com)
 * All Rights Reserved
 */
///////////////////////////////////////////////////////////////////////////////
""" % (srcbasename, brief, thisdate, thistime, xmlfilename, now.tm_year))
  ##

  #--
  def PrettyPrintMajorDivComment(self, fp, comment):
    """ Prints major file division comment block.

        Parameters:
          fp          - Opened output file pointer.
          comment     - Major division comment line.
    """
    fp.write("""\

//-----------------------------------------------------------------------------
// %s
//-----------------------------------------------------------------------------

""" % (comment))
  ##

  #--
  def PrettyPrintMinorDivComment(self, fp, comment):
    """ Prints minor file division comment block.

        Parameters:
          fp          - Opened output file pointer.
          comment     - Minor division comment line.
    """
    fp.write("""\

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// %s
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

""" % (comment))
  ##

  #--
  def PrettyPrintBriefComment(self, fp, brief):
    """ Print doxygen brief comment block.

        Parameters:
          fp      - Opened output file pointer.
          brief   - Brief comment.
    """
    fp.write("""\
/*!
 * %s
 */
""" % (brief))
  ##
##
