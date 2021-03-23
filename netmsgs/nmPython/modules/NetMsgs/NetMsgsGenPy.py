###############################################################################
#
# Package:  NetMsgs
#
# File:     NetMsgsGenPy.py
#

""" 
NetMsgs Generate Python files module.

A .py source file is generated from a  RoadNarrows NetMsg XML specification.
"""

## \file 
## \package NetMsgs.NetMsgsGenPy
##
## $LastChangedDate: 2011-01-14 12:59:01 -0700 (Fri, 14 Jan 2011) $
## $Rev: 670 $
##
## \brief NetMsgs Generate C files module.
##
## A .py source file is generated from a  RoadNarrows NetMsg XML specification.
## 
## \todo Calculate maximum message lengths.
##
## \sa
## \htmlonly
##  <a href="../pydoc/NetMsgs.NetMsgsGenPy.html">PyDoc Generated Documentation</a>
## \endhtmlonly
##
## \author Robin Knight (robin.knight@roadnarrows.com)
##  
## \copyright
##   \h_copy 2010-2017. RoadNarrows LLC.\n
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
import os
import time

import NetMsgs.NetMsgsBase as nmBase
import NetMsgs.NetMsgsXmlParser as NMXmlParser

## space 'macro'
space = lambda indent: "%*s" % (indent, '')


#-----------------------------------------------------------------------------
# CLASS: NetMsgsGenC
#-----------------------------------------------------------------------------
class NetMsgsGenPy:
  """ RoadNarrows Net Messages Python Source Generator Class.

      The NetMsgsGenPy class reads the post-parsed database of a
      NetMsgsXmlParser class instance and generates a Python .py file. The
      python file expects the NetMsgs run-time python library.
  """

  #--
  def __init__(self, xml, pyfilepath, **kwargs):
    """ Initialize NetMsgsGenPy instance.

        Parameters:
          xml         - NetMsgsXMLParser class instance.
          pyfilepath  - Generated output python file.
          kwargs      - Optional keyword arguments.
                          debug     - Set debugging level: 0 == off, 1, 2, 3.
    """
    ## parsed xml instance
    self.mXml       = xml

    ## output .h header file name
    self.mPyFilePath = pyfilepath

    ## debug level
    self.mDebug     = kwargs.get('debug', 0)

    ## do [not] prepend RN Hungarian notation to message fields
    self.mHungarian = kwargs.get('hungarian', False)

    ## output .h header file basename
    self.mPyBaseName = os.path.basename(self.mPyFilePath)

    ## extended field type dictionary
    self.mFTDict = {}

    ## message definition set dictionary
    self.mMsgDefSet = {}

    self.AugmentXmlDB()
  ##

  #--
  def AugmentXmlDB(self):
    """ Augument the XML database of parsed XML values with python generated
        parameters.
    """
    self.mXml['genpy'] = { }
    encoding  = self.mXml['netmsgs']['encoding'].lower()
    if not encoding:
      encoding = 'itv'
    if encoding == 'itv':
      self.mXml['genpy']['encoding_prefix'] = encoding.upper()
    else:
      self.mXml['genpy']['encoding_prefix'] = encoding.capitalize()

    endian = self.mXml['netmsgs']['endian'].lower()
    if not endian:
      endian = 'big'
    self.mXml['genpy']['endian_desc'] = endian + '-endian'
    self.mXml['genpy']['endian_enum'] = 'NMEndian'+endian.capitalize()

    # perform pass 0 through database, setting C specific information
    self.Pass0()
  ##

  #--
  def Pass0(self):
    """ Make initial pass through the XML database and augument with
        specific python information.

        No generated python code is written to file output.
    """
    pass
  

  #.............................................................................
  # Generate Source Functions
  #.............................................................................

  #--
  def GenSource(self):
    """ Generate python .py source file.

        Exceptions:
          Raises NetMsgError exception on error. 
    """
    # open python file for writing
    try:
      fp = open(self.mPyFilePath, 'w')
    except IOError as err:
      raise nmBase.NetMsgsError("Error: %s" % (err))
    self.PrettyPrintTopComment(fp)
    self.GenPrologue(fp)
    self.GenMsgIds(fp)
    self.GenFTDict(fp)
    self.GenMsgDefSet(fp)
    self.GenClass(fp)
    self.GenEpilogue(fp)
    fp.close()
  ##
  
  #--
  def GenPrologue(self, fp):
    """ Generate .py file prologue.

        Prologue includes core imports plus any prologue specified
        in the XML file.

        Parameters:
          fp    - Opened output file pointer.
    """
    pre = self.mXml['genpy']['encoding_prefix']
    module = "NetMsgs.NetMsgsLib%s" % pre
    if self.mXml['netmsgs']['encoding'] == 'flat':
      baseclass = 'NetMsgs'
    else:
      baseclass = 'NetMsgs%s' % pre
    prologue = self.mXml['meta']['python']['prologue']
    fp.write(
"""

import NetMsgs.NetMsgsBase as nmBase
from %s import %s

""" % (module, baseclass))
    if prologue:
      fp.write("%s\n\n" % (prologue))
  ##

  #--
  def GenMsgIds(self, fp):
    """ Generate Message Id enumeration.

        Parameters:
          fp    - Opened output file pointer.
    """
    ns        = self.mXml['meta']['ns']
    msgorder  = self.mXml['msg_types'][nmBase.NMKeyOrder]
    if len(msgorder) == 0:
      return
    self.PrettyPrintMajorDivComment(fp, "%s Message Id Enumeration" % (ns))
    fp.write("class %sMsgId:\n" % ns)
    fp.write('  """ %s Message Id Enumeration class. """\n' % ns)
    n     = 0
    nmBase.PrettyPrintCols(fp, 0,
        2,  'NoId',
        34, '= ' + repr(n),
        42, '# no message id\n')
    for msgid in msgorder:
      n += 1
      msgdef = self.mXml['msg_types'][msgid]
      if self.IsDeprecated(msgdef):
        comment = "%s (deprecated)" % (msgid)
      else:
        comment = "%s" % (msgid)
      nmBase.PrettyPrintCols(fp, 0,
        2,  msgid,
        34, '= ' + repr(n),
        42, "# %s\n" % (comment))
    n += 1
    nmBase.PrettyPrintCols(fp, 0,
        2,  'NumOf',
        34, '= ' + repr(n),
        42, '# number of message ids\n')
    fp.write("##\n\n")
  ##

  #--
  def GenFTDict(self, fp):
    """ Generate field types dictionary.

        genftdict ::=
          ns+'ExtFieldTypes' '=' '{' ftdict '}'

        ftdict ::=
            ftdict_item
          | ftdict ',' ftdict_item

        ftdict_item ::=
            ftid ':' '{' msgdef '}'
          | ftid ':' '{' fielddef '}'

        Parameters:
          fp    - Opened output file pointer.
    """
    ns      = self.mXml['meta']['ns']
    section = 'field_types'
    if len(self.mXml[section][nmBase.NMKeyOrder]) == 0:
      return

    # generate dictionary
    self.mFTDict = {}
    self.mFTDict[nmBase.NMKeyOrder] = []
    nomsgid = "@%sMsgId.NoId" % (ns)
    nofid   = "@nmBase.NMFIdNone"
    for ftid in self.mXml[section][nmBase.NMKeyOrder]:
      ftdef = self.mXml[section][ftid]
      if not self.IsDeprecated(ftdef):
        xtype = ftdef['ftype']
        base_xtype = self.BaseXType(xtype)
        self.mFTDict[ftid] = {}
        self.mFTDict[nmBase.NMKeyOrder] += [ftid]
        if base_xtype == 'struct':
          self.GenMsgDef(ftid, nomsgid, ftdef, self.mFTDict[ftid])
        else:
          self.GenFieldDef(ftid, nofid, ftdef, self.mFTDict[ftid])

    # generate output
    self.PrettyPrintMajorDivComment(fp, "Extended Field Types")
    self.WriteFTDict(fp)
  ##

  #--
  def GenMsgDefSet(self, fp):
    """ Generate message definition set.

        genmsgdefset ::=
          ns+'MsgDefSet' '=' '{' msgdefset '}'

        msgdefset ::=
            msgdef_item
          | msgdefset ',' msgdef_item

        msgdef_item ::=
            msgid ':' '{' msgdef '}'

        Parameters:
          fp    - Opened output file pointer.
    """
    ns      = self.mXml['meta']['ns']
    section = 'msg_types'
    if len(self.mXml[section][nmBase.NMKeyOrder]) == 0:
      return

    # generate dictionary
    self.mMsgDefSet = {}
    self.mMsgDefSet[nmBase.NMKeyOrder] = []
    for msgid in self.mXml[section][nmBase.NMKeyOrder]:
      msgdef = self.mXml[section][msgid]
      if not self.IsDeprecated(msgdef):
        xtype = msgdef['ftype']
        enum = "@%sMsgId.%s" % (ns, msgid)
        self.mMsgDefSet[nmBase.NMKeyOrder] += [enum]
        self.mMsgDefSet[enum] = {}
        self.GenMsgDef(msgid, enum, msgdef, self.mMsgDefSet[enum])

    # generate output
    self.PrettyPrintMajorDivComment(fp, "Message Definition Set")
    self.WriteMsgDefSet(fp)
  ##

  #--
  def GenMsgDef(self, name, msgid, msgdef, dentry):
    """ Generate message definition dictionary entry.

        msgdef ::=
          msgdef_common ',' 'fielddef:' '[' fdeflist ']'

        msgdef_common ::=
          'name:' name ',' 'msgid:' msgid ',' 'max_count:' n

        fdeflist ::=
          '{' fielddef '}'
          fdeflist ',' '{' fielddef '}'

        name ::= (* message string name *)

        msgid ::= (* message id enumeration *)

        max_count ::= (* number of fields in message definition *)

        Parameters:
          name    - Name keyword value.
          msgid   - Message id keyword value.
          msgdef  - XML message definition.
          deentry - Output message definition entry.
    """
    ns          = self.mXml['meta']['ns']
    xtype       = msgdef['ftype']
    base_xtype  = self.BaseXType(xtype)

    # definition parameters
    dentry['name']      = name
    dentry['msgid']     = msgid
    dentry['ftype']     = nmBase.NMBuiltInFieldTypes[base_xtype]['code']
    dentry['max_count'] = 0
    dentry['fielddef']  = []

    # derived definition
    if xtype != base_xtype:
      lineage = self.GetLineage(xtype)
      eve     = lineage[0]  # mother of all
      ftentry = self.mFTDict[eve]
      dentry['max_count'] = ftentry['max_count']
      dentry['fielddef']  = "@%sExtFType%s['fielddef']" % (ns, eve)

    # in-situ definition
    else:
      if not msgdef.has_key('fields'):
        return
      fields    = msgdef['fields']
      max_count = 0
      fid       = 1
      for fname in fields[nmBase.NMKeyOrder]:
        fdef = fields[fname]
        if not self.IsDeprecated(fdef):
          d = {}
          self.GenFieldDef(fname, fid, fdef, d)
          dentry['fielddef'] += [d]
          max_count += 1
        fid += 1
      dentry['max_count'] = max_count
  ##

  #--
  def GenFieldDef(self, name, fid, fdef, dentry):
    """ Generate field definition dictionary entry.

        fielddef ::=
            fielddef_simple
          | fielddef_string
          | fielddef_struct
          | fielddef_vector

        fielddef_simple ::=
          fielddef_common ',' ['min:' m ',' 'max:' M ',' 'const:' C]

        fielddef_string ::=
          fielddef_common ',' ['const:' C]

        fielddef_struct ::=
          fielddef_common ','  'msgdef:' '{' msgdef '}'

        fielddef_vector ::=
          fielddef_common ','  'vdef:' '{' fielddef '}'

        fielddef_common ::=
          'name:' name ',' 'fid:' fid ',' 'ftype:' ftype ',' 'max_count:' n

        name ::= (* field string name *)

        msgid ::= (* field id enumeration *)

        ftype ::= (* field type code *)

        max_count ::=  (* number of vector items, string characters or 1 for
                          simpel types *)

        Parameters:
          name    - Name keyword value.
          fid     - Field id keyword value.
          fdef    - XML field definition.
          deentry - Output field definition entry.
    """
    ns          = self.mXml['meta']['ns']
    xtype       = fdef['ftype']
    base_xtype  = self.BaseXType(xtype)

    # field definition common parameters
    dentry['name']      = name
    dentry['fid']       = fid
    dentry['ftype']     = nmBase.NMBuiltInFieldTypes[base_xtype]['code']
    dentry['max_count'] = 1

    # derived definition
    if xtype != base_xtype:
      lineage = self.GetLineage(xtype)
      eve     = lineage[0]  # mother of all
      ftentry = self.mFTDict[eve]
    # in-situ definition
    else:
      eve = None

    # structure field definition
    if base_xtype == 'struct':
      nomsgid = "@%sMsgId.NoId" % (ns)
      subname = name + '_struct'

      # derived struct
      if eve:
        dentry['msgdef'] = "@%sExtFType%s" % (ns, eve)
      # in-situ definition
      else:
        dentry['msgdef'] = {}
        self.GenMsgDef(subname, nomsgid, fdef, dentry['msgdef'])

    # vector field definition
    elif base_xtype == 'vector':
      nofid   = "@nmBase.NMFIdNone"
      subname = name + '_item'

      # derived vector
      if eve:
        dentry['max_count'] = ftentry['max_count']
        dentry['vdef'] = "@%sExtFType%s['vdef']" % (ns, eve)
      # in-situ definition
      else:
        dentry['max_count'] = '@' + fdef.get('size', 'nmBase.NMVectorMaxCount')
        # create a vector item definition
        vdef            = {}
        vdef['name']    = subname
        vdef['ftype']   = fdef['vtype']
        vdef['fields']  = fdef.get('fields', {})
        dentry['vdef']  = {}
        self.GenFieldDef(subname, nofid, vdef, dentry['vdef'])

    # string field definition
    elif base_xtype == 'string':
      # derived string
      if eve:
        dentry['max_count'] = ftentry['max_count']
        if ftentry.has_key('const'):
          dentry['const'] = ftentry['const']
      # in-situ definition
      else:
        dentry['max_count'] = '@' + fdef.get('size', 'nmBase.NMStringMaxCount')
        if fdef.has_key('const'):
            dentry['const'] = '@' + fdef['const']

    # padding definition
    elif xtype == 'pad':
      dentry['max_count'] = '@' + fdef.get('count', 'nmBase.NMPadDftCount')

    # simple field definition
    else:
      # derived simple
      if eve:
        if ftentry.has_key('min'):
          dentry['min'] = ftentry['min']
        if ftentry.has_key('max'):
          dentry['max'] = ftentry['max']
        if ftentry.has_key('const'):
          dentry['const'] = ftentry['const']
      # in-situ definition
      else:
        if fdef.has_key('min'):
          dentry['min'] = '@' + fdef['min']
        if fdef.has_key('max'):
          dentry['max'] = '@' + fdef['max']
        if fdef.has_key('const'):
          dentry['const'] = '@' + fdef['const']
  ##

  def GenClass(self, fp):
    """ Generate derived NetMsgs class.

        Parameters:
          fp    - Opened output file pointer.
    """
    ns = self.mXml['meta']['ns']
    if not ns:
      ns = '_App'
    classname = "%sNetMsgs" % (ns)
    if self.mXml['netmsgs']['encoding'] == 'flat':
      baseclass = 'NetMsgs'
    else:
      pre = self.mXml['genpy']['encoding_prefix']
      baseclass = 'NetMsgs%s' % pre
    msgdefset   = "%sSetMsgDef" % (ns)
    msgsetname  = "%sMsgSet" % (ns)
    self.PrettyPrintMajorDivComment(fp, "CLASS: %s" % (classname))
    fp.write("""\
class %s(%s):
  \"\"\" %s NetMsgs Class. \"\"\"

  #--
  def __init__(self, **kwargs):
    \"\"\" %s NetMsgs initialization. \"\"\"
    kwargs['msgsetname'] = '%s'
    %s.__init__(self, %s, **kwargs)
  ##
##""" % (classname, baseclass, ns, ns, msgsetname, baseclass, msgdefset))
  ##

  #--
  def GenEpilogue(self, fp):
    """ Generate .py file epilogue.

        Parameters:
          fp    - Opened output file pointer.
    """
    epilogue  = self.mXml['meta']['python']['epilogue']
    if epilogue:
      fp.write("\n\n%s" % (epilogue))
    fp.write("\n")
  ##


  #.............................................................................
  # Output Functions
  #.............................................................................

  #--
  def WriteFTDict(self, fp):
    """ Write field types and field type dictionary.

        Parameters:
          fp    - Opened output file pointer.
    """
    ns  = self.mXml['meta']['ns']
    d   = {}
    for ftid in self.mFTDict[nmBase.NMKeyOrder]:
      ftname = "%sExtFType%s" % (ns, ftid)
      self.PrettyPrintBriefComment(fp, "%s Extended Field Type " % ftid)
      nmBase.PrettyPrintAssignExpr(ftname, self.mFTDict[ftid], fp=fp)
      fp.write("##\n\n")
      d[ftid] = '@' + ftname
    self.PrettyPrintBriefComment(fp, "%s Extended Field Type Dictionary" % ns)
    dname = "%sExtFieldTypes" % (ns)
    nmBase.PrettyPrintAssignExpr(dname, d, fp=fp)
    fp.write("##\n\n")
  ##

  #--
  def WriteMsgDefSet(self, fp):
    """ Write message definitions and message definition set dictionary.

        Parameters:
          fp    - Opened output file pointer.
    """
    ns  = self.mXml['meta']['ns']
    d   = {}
    for enum in self.mMsgDefSet[nmBase.NMKeyOrder]:
      msgdef  = self.mMsgDefSet[enum]
      name    = msgdef['name']
      mname   = "%sMsgDef%s" % (ns, name)
      self.PrettyPrintBriefComment(fp, "%s Message Definition " % name)
      nmBase.PrettyPrintAssignExpr(mname, msgdef, fp=fp)
      fp.write("##\n\n")
      d[enum] = '@' + mname
    self.PrettyPrintBriefComment(fp,
        "%s Message Definition Set Dictionary" % ns)
    dname = "%sSetMsgDef" % (ns)
    nmBase.PrettyPrintAssignExpr(dname, d, fp=fp)
    fp.write("##\n\n")
  ##


  #.............................................................................
  # Utilities
  #.............................................................................

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
  def BaseXType(self, ftype):
    """ Returns the base XML field type for the given field type.

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
  def GetLineage(self, xtype):
    """ Get lineage of XML field type

        Parameters:
          xtype   - XML field type

        Return:
          List of ancestors order from base to nearest to xtype.
    """
    lineage = []
    if xtype == 'pad':
      return lineage
    while not nmBase.NMBuiltInFieldTypes.has_key(xtype):
      if self.mXml['field_types'].has_key(xtype):
        lineage = [xtype] + lineage
        xtype = self.mXml['field_types'][xtype]['ftype']
      else:
        raise nmBase.NetMsgsError("Error: %s is not a defined ftype" % (xtype))
    return lineage
  ##

  #--
  def EvalNum(sefl, *args):
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
    n = 0
    s = ''
    for arg in args:
      if arg is None:
        continue
      elif type(arg) == str:
        arg = arg.replace("NMFVAL_LEN_MAX_STRING", "255")    # built-in
        arg = arg.replace("NMFVAL_LEN_MAX_VECTOR", "255")    # built-in
      else:
        arg = repr(arg)
      try:
        n += eval(arg)
      except:
        s += '+(' + arg + ')'
    n = int(n)
    return repr(n) + s
  ##


  #.............................................................................
  # Pretty Printing
  #.............................................................................

  #--
  def PrettyPrintTopComment(self, fp):
    """ Prints source file top comment block.

        Parameters:
          fp          - Opened output file pointer.
    """
    srcbasename = self.mPyBaseName
    brief       = self.mXml['meta']['brief']
    xmlfilename = self.mXml.GetXMLFileName()
    now         = time.localtime()
    thisdate    = "%d.%02d.%02d" % (now.tm_year, now.tm_mon, now.tm_mday)
    thistime    = "%02d:%02d:%02d" % (now.tm_hour, now.tm_min, now.tm_sec)
    fp.write("""\
##############################################################################
#
# File: %s
#

\"\"\"
%s
\"\"\"

## \\file
##
## \\brief %s
##
## \\warning This file was auto-generated on %s %s from the NetMsgs
## XML specification %s.
##
## \\par Copyright:
## (C) %d. RoadNarrows LLC
## (http://www.roadnarrows.com)
## All Rights Reserved
##
##############################################################################
""" % (srcbasename, srcbasename, brief, thisdate, thistime,
        xmlfilename, now.tm_year))
  ##

  #--
  def PrettyPrintMajorDivComment(self, fp, comment):
    """ Prints major file division comment block.

        Parameters:
          fp          - Opened output file pointer.
          comment     - Major division comment line.
    """
    fp.write("""\

# -----------------------------------------------------------------------------
# %s
# -----------------------------------------------------------------------------

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

# . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
# %s
# . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

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
## %s
""" % (brief))
  ##
##

