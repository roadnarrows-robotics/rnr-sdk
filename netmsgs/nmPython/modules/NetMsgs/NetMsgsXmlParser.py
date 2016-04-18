###############################################################################
#
# Package:  NetMsgs
#
# File:     NetMsgsXmlParser.py
#

""" 
NetMsgs XML parser module.
"""

## \file 
## \package NetMsgs.NetMsgsXmlParser
##
## $LastChangedDate: 2013-08-08 13:02:57 -0600 (Thu, 08 Aug 2013) $
## $Rev: 3183 $
##
## \brief NetMsgs XML parser module.
##
## \todo Add brief elements to xdefs for better comments to generated language
##  constructs
##
## \sa
## \htmlonly
##  <a href="../pydoc/NetMsgs.NetMsgsXmlParser.html">PyDoc Generated Documentation</a>
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

import os
import xml.parsers.expat as expat
import time

import NetMsgsBase as nmBase

#------------------------------------------------------------------------------
# CLASS: NetMsgsXmlParserError
#------------------------------------------------------------------------------
class NetMsgsXmlParserError(Exception):
  """ XML Parser Exception Class. """

  def __init__(self, msg='XML Parser Error'):
    """ Raise exception.

        Parameters:
          msg    - Exception message string.
    """
    Exception.__init__(self, msg)

##

#-----------------------------------------------------------------------------
# CLASS: NetMsgsXmlParser
#-----------------------------------------------------------------------------
class NetMsgsXmlParser:
  """ RoadNarrows Net Messages XML Parser Class.

      The NetMsgsXmlParser class parses a RoadNarrows netmsgs XML specification.
      The parsed information is available to the calling program.

      XML Syntax:
      <netmsgs>
        <meta>
        ...
        </meta>
        <field_types>
        ...
        </field_types>
        <msg_types>
        ...
        </msg_types>
      </netmsgs>
  """

  def __init__(self, filename=None, debug=0, **kwargs):
    """ Initialize NetMsgsXmlParser instance.

        Parameters:
          filename    - XML file name.
          debug       - Set debugging level: 0 == off, 1, 2, 3.
          kwargs      - Optional XML overrides:
                          encoding  - Message encoding. One of itv flat.
                          endian    - Byte order. One of big little native.
                          ns        - Message namespace (name prefix).
                          brief     - Brief description.
    """
    ## associated xml file name
    self.mFileName      = filename

    ## debug print at given <= level
    self.mDebug         = debug

    ## XML overrides
    self.mXmlOverrides              = {}
    self.mXmlOverrides['encoding']  = kwargs.get('encoding', None)
    self.mXmlOverrides['endian']    = kwargs.get('endian', None)
    self.mXmlOverrides['ns']        = kwargs.get('ns', None)
    self.mXmlOverrides['brief']     = kwargs.get('brief', None)

    if filename:
      ## xml file base name
      self.mBaseName  = os.path.basename(filename)
    else:
      self.mBaseName  = None

    ## xml tree starting at '_root'
    self.XmlTree   = {
        '_root':        [ 'netmsgs' ],
        'netmsgs':      [ 'meta', 'field_types', 'msg_types' ],
        'meta':         [ 'brief', 'ns', 'prologue', 'epilogue' ],
        'field_types':  [ 'ftypedef' ],
        'ftypedef':     [ 'fielddef' ],
        'msg_types':    [ 'msgdef' ],
        'msgdef':       [ 'fielddef', 'pad' ],
        'fielddef':     [ 'min', 'max', 'const', 'fielddef' ]
    }

    ## XML element start callbacks
    self.XmlCbStartElem = {
        'netmsgs':      self.XmlCbStartNetMsgs,
        'meta':         self.XmlCbStartSection,
        'brief':        None,
        'ns':           None,
        'prologue':     None,
        'epilogue':     None,
        'field_types':  self.XmlCbStartSection,
        'ftypedef':     self.XmlCbStartFTypeDef,
        'msg_types':    self.XmlCbStartSection,
        'msgdef':       self.XmlCbStartMsgDef,
        'fielddef':     self.XmlCbStartFieldDef,
        'const':        None,
        'min':          None,
        'max':          None,
        'pad':          self.XmlCbStartPad,
    }

    ## XML element start callbacks
    self.XmlCbEndElem = {
        'netmsgs':      None,
        'meta':         self.XmlCbEndSection,
        'brief':        self.XmlCbEndMetaData,
        'ns':           self.XmlCbEndMetaData,
        'prologue':     self.XmlCbEndMetaLang,
        'epilogue':     self.XmlCbEndMetaLang,
        'field_types':  self.XmlCbEndSection,
        'ftypedef':     self.XmlCbEndFTypeDef,
        'msg_types':    self.XmlCbEndSection,
        'msgdef':       self.XmlCbEndMsgDef,
        'fielddef':     self.XmlCbEndFieldDef,
        'const':        self.XmlCbEndFieldDefConst,
        'min':          self.XmlCbEndFieldDefMinMax,
        'max':          self.XmlCbEndFieldDefMinMax,
        'pad':          None,
    }

    ## parsed and augmented database
    self.mDB     = {                 
        'netmsgs':      { 'encoding': 'itv', 'endian': 'big' },
        'meta':         { 'brief': "Net Messages",
                          'ns':    "",
                          'c':     {'prologue': "", 'epilogue': ""},
                          'h':     {'prologue': "", 'epilogue': ""},
                          'python':{'prologue': "", 'epilogue': ""}},
        'field_types':  {nmBase.NMKeyOrder:[]},
        'msg_types':    {nmBase.NMKeyOrder:[]},
    }

    # XML working state
    self._XmlParser         = None    # xml parser
    self._XmlElemStack      = []      # xml element stack
    self._XmlCurElem        = None    # current element name (top of stack)
    self._XmlSection        = '_root' # major section context
    self._XmlFieldDefDepth  = 0       # depth of recursive <fielddef> element
    self._XmlElems          = []      # full list of elements

    # DB working state
    self._DbFieldDictStack    = []    # field defs stack
    self._DbCurFieldDict      = None  # working fielddef list

    # list of XML elements in preferred order
    self._InitElemOList(self.XmlTree['_root'], self._XmlElems)
  ##

  #--
  def _InitElemOList(self, elemList, olist):
    """ Make an ordered list of XML element names.

        Names are derieved from the XML tree.

        Parameters:
          elemList  - List of element names

        Return Value:
          Orders element list.
    """
    for elem in elemList:
      if elem not in olist:
        olist += [elem]
        if self.XmlTree.has_key(elem):
          self._InitElemOList(self.XmlTree[elem], olist)
  ##


  #.............................................................................
  # XML Get/Set Attribute Functions
  #.............................................................................

  #--
  def __getitem__(self, dbkey):
    """ x.__getitem__(dbkey) <==> x[dbkey]

        Get database entry.
    """
    return self.mDB[dbkey]
  ##

  #--
  def __setitem__(self, dbkey, val):
    """ x.__setitem__(dbkey, val) <==> x[dbkey]=val

        Set new or overwrite existing database entry.
    """
    self.mDB[dbkey] = val
  ##

  #--
  def GetDBData(self, dbkey):
    """ Get database entry.

        Parameters:
          dbkey   - Parsed data key.

        Return Value:
          Database data.
    """
    return self.mDB[dbkey]
  ##

  #--
  def GetDBDict(self):
    """ Get full database dictionary.

        Return Value:
          Dictionary.
    """
    return self.mDB
  ##

  #--
  def GetDBList(self):
    """ Get the full list of database keys.

        Return Value:
          List of pnames.
    """
    return self.mDB.keys()
  ##
    
  #--
  def GetElemList(self):
    """ Get the full list of XML element names.
        
        The list is in preferred output order.

        Return Value:
          List of element names.
    """
    return self._XmlElems
  ##

  #--
  def GetXMLFileName(self):
    """ Get the NetMsgs XML file name.
        
        Return Value:
          String.
    """
    return self.mFileName
  ##


  #.............................................................................
  # XML Parsing and Validation
  #.............................................................................
    
  #--
  def ParseFile(self, filename):
    """ Parse the Xml XML data specified in the given file.

        Parameters:
          filename    - README XML file name.
    """
    self.mFileName  = filename
    if filename:                      # xml file base name
      self.mBaseName    = os.path.basename(filename)
    self.Parse()
  ##

  #--
  def Parse(self):
    """ Parse the Xml XML data listed in the current file.
    """
    if not self.mFileName:
      self.Error('No filename')
    self.Debug(1, "Parsing RN NetMsgs XML file %s" % (self.mFileName))
    try:
      fp = open(self.mFileName, 'r')
    except IOError, err:
      self.Error(self.mFileName, err)
    self.Reset()
    self._XmlParser                           = expat.ParserCreate()
    self._XmlParser.returns_unicode           = False
    #self._XmlParser.UseForeignDTD(True)
    #self._XmlParser.ExternalEntityRefHandler  = self.XmlHandlerDTD
    #self._XmlParser.StartDoctypeDeclHandler   = self.XmlHandlerStartDoctype
    #self.EntityDeclHandler                    = self.XmlHandlerEntityDecl
    #self._XmlParser.ElementDeclHandler        = self.XmlHandlerElementDecl
    self._XmlParser.StartElementHandler       = self.XmlHandlerStartElem
    self._XmlParser.CharacterDataHandler      = self.XmlHandlerCharData
    self._XmlParser.EndElementHandler         = self.XmlHandlerEndElem
    self._XmlParser.CommentHandler            = self.XmlHandlerComment
    try:
      self._XmlParser.ParseFile(fp)
    except expat.ExpatError as e:
      self.ExpatError(expat.ErrorString(e.code))
    self.PostParse()
  ##

  #--
  def Reset(self):
    """ Reset the XML parser.
    """
    if self._XmlParser is not None:
      del self._XmlParser
      self._XmlParser = None
    self._XmlElemStack      = []
    self._XmlCurElem        = None
    self._XmlSection        = '_root'
    self._XmlFieldDefDepth  = 0
    self._DbFieldDictStack  = []
    self._DbCurFieldDict    = None
  ##

  #--
  def PostParse(self):
    """ Post-parse configuration and validation. """
    for k,v in self.mXmlOverrides.iteritems():
      if v is None:
        continue
      elif k == 'encoding':
        if v not in nmBase.NMEncoding:
          self.XmlError("XML Override: %s='%s'" % (k, v),
            "unknown message encoding")
        else:
          self.mDB['netmsgs'][k] = v
      elif k == 'endian':
        if v not in nmBase.NMEndian:
          self.XmlError("XML Override: %s='%s'" % (k, v),
            "unknown message field byte order")
        else:
          self.mDB['netmsgs'][k] = v
      elif k == 'ns':
        if not nmBase.IsIdentifier(v):
          self.XmlError("XML Override: %s='%s'" % (k, v),
          "namespace value must be an identifier")
        else:
          self.mDB['meta'][k] = v
      elif k == 'brief':
        self.mDB['meta'][k] = v
      else:
        self.XmlWarning("XML Override: %s" % (k, v),
            "unknown override")
  ##


  #.............................................................................
  # XML Parser Callback Handlers
  #.............................................................................
  
  #--
  def XmlHandlerDTD(self, context, base, systemId, publicId):
    """ XML Parser handler called back at start of DTD processing.
    """
    self.XmlDebug(1, "start-of-dtd-parse",
        "base=%s" % repr(base),
        "systemId=%s" % repr(systemId),
        "publicId=%s" % repr(publicId))
    self._XmlParser.ExternalEntityParserCreate(context)
    return 0 # error
  ##

  #--
  def XmlHandlerStartDoctype(self,  doctypeName, systemId, publicId,
                                    has_internal_subset):
    """ XML Parser handler called back at start of DOCTYPE processing.
    """
    self.XmlDebug(1, "start-of-DOCTYPE",
        "doctypeName=%s" % repr(doctypeName),
        "systemId=%s" % repr(systemId),
        "publicId=%s" % repr(publicId),
        "has_internal=%s" % repr(has_internal_subset))
  ##

  #--
  def XmlHandlerEntityDecl(entityName, is_parameter_entity, value, base,
                            systemId, publicId, notationName):
    """ XML Parser handler called back at start of DTD ENTITY processing.
    """
    self.XmlDebug(1, "entity-decl",
        "entityName=%s" % repr(entityName))
  ##

  #--
  def XmlHandlerElementDecl(self, name, model):
    """ XML Parser handler called back at start of DTD ELEMENT processing.
    """
    self.XmlDebug(1, "element-decl",
        "name=%s" % repr(name),
        "model=%s" % repr(model))
  ##

  #--
  def XmlHandlerStartElem(self, elem, attrs):
    """ XML Parser handler called back at start of element.

        Parameters:
          elem    - Element name.
          attrs   - Dictionary of element attributes.
    """
    self.XmlDebug(1, "start-of-element", "<%s> attrs=%s" % (elem, repr(attrs)))
    if elem in self._XmlElems:
      self.PushElem(elem, attrs)
      cb = self.XmlCbStartElem.get(elem)
      if cb:
        cb(elem, **attrs)
    else:
      self.XmlError("<%s>" % (elem), "unrecognized element")
  ##

  #--
  def XmlHandlerEndElem(self, elem):
    """ XML Parser handler called back at end of element.

        The associated database element(s) are set with the converted data.
        The data must conform to the requirements of the element.

        Parameters:
          elem    - Element name.
    """
    self.XmlDebug(1, "end-of-element", "<%s> data=" % (elem))
    if elem == self._XmlCurElem['elem']:
      elem  = self._XmlCurElem['elem']
      attrs = self._XmlCurElem['attrs']
      data  = self._XmlCurElem['data'].strip()
      self.XmlDebug(1, "%s" % (data))
      cb = self.XmlCbEndElem.get(elem)
      if cb:
        cb(elem, data, **attrs)
      self.PopElem()
    else:
      self.Error("Bug: Lost track of elements: Expected <%s>, got <%s>" % \
          (self._XmlCurElem['elem'], elem))
  ##

  #--
  def XmlHandlerCharData(self, data):
    """ XML Parser handler called back for each line of element data.

        Parameters:
          data    - Unstructured element data.
    """
    self.XmlDebug(2, "char data:", repr(data))
    self._XmlCurElem['data'] += data
  ##

  #--
  def XmlHandlerComment(self, comment):
    """ XML Parser handler called back for each end of a comment block.

        Parameters:
          comment - Comment text sans '<!-' and '-->'.
    """
    self.XmlDebug(1, "comment:", repr(comment))
  ##

  def XmlCbStartNetMsgs(self, elem, **attrs):
    """ Start-Of-Element callback for the 'netmsgs' top element.

        Parameters:
          elem    - Element name.
          attrs   - Element attributes.
    """
    encoding  = attrs.get('encoding', 'itv').lower()
    endian    = attrs.get('endian', 'big').lower()
    if encoding not in nmBase.NMEncoding:
      self.XmlError("<%s encoding='%s'>" % (elem, encoding),
          "unknown message encoding")
    if endian not in nmBase.NMEndian:
      self.XmlError("<%s endian='%s'>" % (elem, endian),
          "unknown message field byte order")
    self.mDB[elem]['encoding']  = encoding
    self.mDB[elem]['endian']    = endian
    self._XmlSection = elem
  ##

  def XmlCbStartSection(self, elem, **attrs):
    """ Start-Of-Element callback for a major XML section.

        Parameters:
          elem    - Element name.
          attrs   - Element attributes.
    """
    self._XmlSection = elem
  ##

  def XmlCbEndSection(self, elem, data, **attrs):
    """ End-Of-Element callback for a major XML section.

        Parameters:
          elem    - Element name.
          data    - Element data.
          attrs   - Element attributes.
    """
    self._XmlSection = 'netmsgs'
  ##

  #--
  def XmlCbEndMetaData(self, elem, data, **attrs):
    """ End Callback for the 'meta' general data sub-elements.

        The associated XML parser database entries are updated with the current
        parsed data.

        Parameters:
          elem    - Element name.
          data    - Element data.
          attrs   - Element attributes.
    """
    if elem == 'ns':
      if not nmBase.IsIdentifier(data):
        self.XmlError("<%s>%s</%s>" % (elem, data, elem),
          "namespace value must be an identifier")
    self.mDB['meta'][elem] = data
  ##

  #--
  def XmlCbEndMetaLang(self, elem, data, **attrs):
    """ End-Of-Element callback for the 'meta' language-specific data
        sub-elements.

        The associated XML parser database entries are updated with the current
        parsed data.

        Parameters:
          elem    - Element name.
          data    - Element data.
          attrs   - Element attributes.
    """
    lang = attrs.get('lang')
    if not lang:
      self.XmlError("<%s>" % (elem),
          "required '%s' attribute not specified." % (attr))
    self.mDB['meta'][lang][elem] = data
  ##

  #--
  def XmlCbStartFTypeDef(self, elem, **attrs):
    """ Start-Of-Element callback for the 'ftypedef' element.

        The associated XML parser database entries are updated with the current
        parsed data.

        Parameters:
          elem    - Element name.
          attrs   - Element attributes.
    """
    ftid          = self.XmlChkTokenId(elem, 'ftid', attrs.get('ftid'))
    ftype, vtype  = self.XmlChkTokenFType(elem, attrs.get('ftype'))
    fsize         = attrs.get('size')
    if self.mDB['field_types'].has_key(ftid):
      self.XmlError("<%s ftid='%s'>" % (elem, ftid),
          "field type id already exists.")
    self.mDB['field_types'][nmBase.NMKeyOrder] += [ftid]
    self.mDB['field_types'][ftid] = {'disposition': 'active', 'ftype': ftype}
    if fsize:
      if ftype in ['string', 'vector']: 
        self.mDB['field_types'][ftid]['size'] = fsize
      else:
        self.XmlWarning("<%s ftid='%s' size='%s'>" % (elem, ftid, fsize),
          "size attribute ignored.")
    if ftype == 'vector':
      self.mDB['field_types'][ftid]['vtype'] = vtype
    elif ftype == 'struct':
      self.mDB['field_types'][ftid]['fields'] = {nmBase.NMKeyOrder: []}
      self.PushFieldDict(self.mDB['field_types'][ftid]['fields'])
  ##

  #--
  def XmlCbEndFTypeDef(self, elem, data, **attrs):
    """ End-Of-Element callback for the 'ftypedef' element.

        The associated XML parser database entries are updated with the current
        parsed data.

        Parameters:
          elem    - Element name.
          data    - Element data.
          attrs   - Element attributes.
    """
    self.PopFieldDict()
  ##
  
  #--
  def XmlCbStartMsgDef(self, elem, **attrs):
    """ Start-Of-Element callback for the 'msgdef' element.

        The associated XML parser database entries are updated with the current
        parsed data.

        Parameters:
          elem    - Element name.
          attrs   - Element attributes.
    """
    msgid = self.XmlChkTokenId(elem, 'msgid', attrs.get('msgid'))
    if self.mDB['msg_types'].has_key(msgid):
      self.XmlError("<%s msgid='%s'>" % (elem, msgid),
          "messsage definition id already exists.")
    dispo = self.XmlChkTokenDispo(elem, attrs.get('disposition'))
    self.mDB['msg_types'][nmBase.NMKeyOrder] += [msgid]
    self.mDB['msg_types'][msgid] = {'disposition': dispo,
                                    'ftype': 'struct',
                                    'fields': {nmBase.NMKeyOrder:[]} }
    self.PushFieldDict(self.mDB['msg_types'][msgid]['fields'])
  ##

  #--
  def XmlCbEndMsgDef(self, elem, data, **attrs):
    """ End-Of-Element callback for the 'msgdef' element.

        The associated XML parser database entries are updated with the current
        parsed data.

        Parameters:
          elem    - Element name.
          data    - Element data.
          attrs   - Element attributes.
    """
    self.PopFieldDict()
  ##
  
  #--
  def XmlCbStartFieldDef(self, elem, **attrs):
    """ Start-Of-Element callback for the 'fielddef' element.

        The associated XML parser database entries are updated with the current
        parsed data.

        Parameters:
          elem    - Element name.
          attrs   - Element attributes.
    """
    self._XmlFieldDefDepth += 1
    pid, pattr = self.XmlFieldDefContext()
    if pattr['ftype'] != 'struct':
      self.XmlError("<%s>" % (elem),
          "field definition not allowed in current context.")
    fname         = self.XmlChkTokenId(elem, 'fname', attrs.get('fname'))
    ftype, vtype  = self.XmlChkTokenFType(elem, attrs.get('ftype'))
    fsize         = attrs.get('size')
    dispo         = self.XmlChkTokenDispo(elem, attrs.get('disposition'))
    if self._DbCurFieldDict.has_key(fname):
      self.XmlError("<%s fname='%s'>" % (elem, fname),
          "field definition name already exists in current context.")
    self._DbCurFieldDict[nmBase.NMKeyOrder] += [fname]
    self._DbCurFieldDict[fname] = {'ftype': ftype}
    if fsize:
      if ftype in ['string', 'vector']: 
        self._DbCurFieldDict[fname]['size'] = fsize
      else:
        self.XmlWarning("<%s fname='%s' size='%s'>" % (elem, fname, fsize),
          "size attribute ignored.")
    self._DbCurFieldDict[fname]['disposition'] = dispo
    if ftype == 'vector':
      self._DbCurFieldDict[fname]['vtype'] = vtype
    elif ftype == 'struct':
      self._DbCurFieldDict[fname]['fields'] = {nmBase.NMKeyOrder:[]}
      self.PushFieldDict(self._DbCurFieldDict[fname]['fields'])
  ##
  
  #--
  def XmlCbEndFieldDef(self, elem, data, **attrs):
    """ End-Of-Element callback for the 'fielddef' element.

        The associated XML parser database entries are updated with the current
        parsed data.

        Parameters:
          elem    - Element name.
          data    - Element data.
          attrs   - Element attributes.
    """
    self._XmlFieldDefDepth -= 1
    fname = attrs.get('fname')
    while len(self._DbFieldDictStack) > self._XmlFieldDefDepth+1:
      self.PopFieldDict()
  ##
  
  #--
  def XmlCbEndFieldDefConst(self, elem, data, **attrs):
    """ End-Of-Element callback for the 'fielddef' 'const' sub-element.

        The associated XML parser database entries are updated with the current
        parsed data.

        Parameters:
          elem    - Element name.
          attrs   - Element attributes.
    """
    pfname, pattr = self.XmlFieldDefContext()
    pftype = pattr['ftype']
    if self.IsSimple(pftype) or pftype == 'string':
      self._DbCurFieldDict[pfname][elem] = data
    elif  pftype == 'vector' and \
          ( self.IsSimple(self._DbCurFieldDict[pfname]['vtype']) or \
            self._DbCurFieldDict[pfname]['vtype'] == 'string' ):
      self._DbCurFieldDict[pfname][elem] = data
    else:
      self.XmlWarning("<%s>" % (elem),
            "element ignored in <fielddef fname='%s'> context." % (pfname))
  ##

  #--
  def XmlCbEndFieldDefMinMax(self, elem, data, **attrs):
    """ End-Of-Element callback for the 'fielddef' min/max limits sub-elements.

        The associated XML parser database entries are updated with the current
        parsed data.

        Parameters:
          elem    - Element name.
          data    - Element data.
          attrs   - Element attributes.
    """
    pfname, pattr = self.XmlFieldDefContext()
    pftype = pattr['ftype']
    if self.IsSimple(pftype):
      self._DbCurFieldDict[pfname][elem] = data
    elif pftype == 'vector' and \
        self.IsSimple(self._DbCurFieldDict[pfname]['vtype']):
      self._DbCurFieldDict[pfname][elem] = data
    else:
      self.XmlWarning("<%s>" % (elem),
            "element ignored in <fielddef fname='%s'> context." % (pfname))
  ##

  #--
  def XmlCbStartPad(self, elem, **attrs):
    """ Start-Of-Element callback for the 'pad' element.

        The associated XML parser database entries are updated with the current
        parsed data.

        Parameters:
          elem    - Element name.
          attrs   - Element attributes.
    """
    inst  = len(self._DbCurFieldDict[nmBase.NMKeyOrder])
    count = attrs.get('count', '1')
    fname = "%s%d" % (nmBase.NMKeyPad, inst)
    self._DbCurFieldDict[nmBase.NMKeyOrder] += [fname]
    self._DbCurFieldDict[fname] = {'ftype':'pad', 'count': count}
  ##
  

  #.............................................................................
  # XML Write Functions
  #.............................................................................

  #--
  def WriteXmlFile(self, filename, comment=None):
    """ Create a Product XML file from the given parsed or set data.

        Parameters:
          filename    - Product XML file name.
          comment     - Comment string.
    """
    fp = open(filename, 'w')
    self.WriteXml(fp, comment)
  ##

  #--
  def WriteXml(self, fp, comment=None):
    """ Create a Product XML file from the given parsed or set data.

        Parameters:
          fp          - Opened for writing file pointer.
          comment     - Comment string.
    """
    fp.write("<!--\n")
    if comment:
      fp.write("  - ")
      fp.write("%s\n" % (comment))
    fp.write("  - ")
    now = time.localtime()
    fp.write("%d.%02d.%02d %02d:%02d:%02d\n" % \
        ( now.tm_year, now.tm_mon, now.tm_mday, now.tm_hour, now.tm_min, 
          now.tm_sec))
    fp.write(" -->\n")
    self.WriteXmlTree(fp, 0, '_root', self.XmlTree['_root'])
    fp.flush()
    fp.close()
  ##

  #--
  def WriteXmlTree(self, fp, level, elemParent, elemList):
    """ Write out XML tree.

        Parameters:
          fp        - Opened for writing file pointer.
          level     - element level (depth)
          elemList  - element level (depth)
    """
    pass
  ##
  

  #.............................................................................
  # DB Print Functions
  #.............................................................................

  #--
  def PrettyPrintDB(self, comment=None):
    """ Pretty print database dictionary of parsed XML values. """
    now = time.localtime()

    print "#"
    if comment:
      print "# %s" % (comment)
    print "# %s database dump." % (self.__class__)
    print "# Input XML file: %s" % (self.mFileName)
    print "# Date: %d.%02d.%02d  %02d:%02d:%02d" % \
            (now.tm_year, now.tm_mon, now.tm_mday,
             now.tm_hour, now.tm_min, now.tm_sec)
    print "#"

    sections = self.mDB.keys()

    dbkey = 'netmsgs'
    print "%s:\n{" % (dbkey)
    print "  encoding: %s" % (self.mDB[dbkey]['encoding'])
    print "  endian:   %s" % (self.mDB[dbkey]['endian'])
    print '}'
    sections.remove(dbkey)

    dbkey = 'meta'
    print "%s:\n{" % (dbkey)
    print "  brief: %s" % (repr(self.mDB[dbkey]['brief']))
    print "  ns:    %s" % (repr(self.mDB[dbkey]['ns']))
    for lang in ['h', 'c', 'python']:
      print "  %s:prologue: %s" % (lang, self.mDB[dbkey][lang]['prologue'])
      print "  %s:epilogue: %s" % (lang, self.mDB[dbkey][lang]['epilogue'])
    print '}'
    sections.remove(dbkey)

    dbkey = 'field_types'
    print "%s:\n{" % (dbkey)
    print "  %s: %s" % \
          (nmBase.NMKeyOrder, repr(self.mDB[dbkey][nmBase.NMKeyOrder]))
    for ftid in self.mDB[dbkey][nmBase.NMKeyOrder]:
      self.PrettyPrintDBFieldDef(2, ftid, self.mDB[dbkey][ftid])
    print '}'
    sections.remove(dbkey)

    dbkey = 'msg_types'
    print "%s:\n{" % (dbkey)
    print "  %s: %s" % \
          (nmBase.NMKeyOrder, repr(self.mDB[dbkey][nmBase.NMKeyOrder]))
    for msgid in self.mDB[dbkey][nmBase.NMKeyOrder]:
      self.PrettyPrintDBFieldDef(2, msgid, self.mDB[dbkey][msgid])
    print '}'
    sections.remove(dbkey)

    # application specific db augments
    for dbkey in sections:
      print "%s:" % (dbkey)
      print "  %s" % (repr(self.mDB[dbkey]))
  ##

  #--
  def PrettyPrintDBFields(self, indent, fields):
    """ Pretty print database fields sub-dictionary of parsed XML values.
      
        Parameters:
          indent      - Indentation.
          fields      - mDB[...]['fields'] sub-dictionary.
    """
    dbkey = 'fields'
    print "%*s%s:" % (indent, '', dbkey)
    print "%*s{" % (indent, '')
    indent += 2
    print "%*s%s: %s" % (indent, '', \
        nmBase.NMKeyOrder, repr(fields[nmBase.NMKeyOrder]))
    for fname in fields[nmBase.NMKeyOrder]:
      self.PrettyPrintDBFieldDef(indent, fname, fields[fname])
    indent -= 2
    print "%*s}" % (indent, '')
  ##

  #--
  def PrettyPrintDBFieldDef(self, indent, fname, fielddef):
    """ Pretty print database fielddef sub-dictionary of parsed XML values.
      
        Parameters:
          indent      - Indentation.
          fields      - mDB[...]['fields'] sub-dictionary.
    """
    sections = fielddef.keys()
    if 'fields' in sections:
        sections.remove('fields')
    print "%*s%s:" % (indent, '', fname)
    print "%*s{" % (indent, '')
    indent += 2
    for k in ['ftype', 'vtype', 'size', 'const', 'min', 'max', 'count']:
      if fielddef.has_key(k):
        self.PrettyPrintVal(indent, k, fielddef[k])
        sections.remove(k)
    for k in sections:
      self.PrettyPrintVal(indent, k, fielddef[k])
    if fielddef.has_key('fields'):
      self.PrettyPrintDBFields(indent, fielddef['fields'])
    indent -= 2
    print "%*s}" % (indent, '')
  ##

  #--
  def PrettyPrintVal(self, indent, name, val):
    """ Pretty print value.

        Parameters:
          indent    - Indentation
          name      - name
          value     - dictionary or other value type.
    """
    print "%*s%s: " % (indent, '', name),
    if type(val) == dict:
      print "\n%*s{" % (indent, '')
      for k,v in val.iteritems():
        self.PrettyPrintVal(indent+2, k, v)
      print "%*s}" % (indent, '')
    else:
      print repr(val)
  ##


  #.............................................................................
  # XML Utilities
  #.............................................................................

  #--
  def XmlChkTokenId(self, elem, attr, id):
    """ Parse and validate XML element id attribute.

        Parameters:
          elem    - Element name value.
          attr    - Element attribute name.
          id      - Element attribute value.

        Return:
          id
    """
    if not id:
      self.XmlError("<%s>" % (elem),
          "required '%s' attribute not specified." % (attr))
    elif not nmBase.IsIdentifier(id):
      self.XmlError("<%s %s='%s'>" % (elem, attr, id),
          "'%s' not an identifier." % (attr))
    return id
  ##

  #--
  def  XmlChkTokenFType(self, elem, ftype):
    """ Parse and validate XML element ftype attribute.

        Parameters:
          elem    - Element name value.
          ftype   - Element ftype raw attribute value.

        Return:
          Returns ('vector', vtype) if vector.
          Returns (ftype, None) otherwise.
    """
    if not ftype:
      self.XmlError("<%s>" % (elem),
          "required 'ftype' attribute not specified.")
    ftype, vtype = self.XmlTokenFType(ftype)
    if vtype:
      if not nmBase.NMBuiltInFieldTypes.has_key(vtype) and \
         not self.mDB['field_types'].has_key(vtype):
        self.XmlError("<%s ftype='%s%s'>" % \
            (elem, vtype, nmBase.NMVectorSuffix),
            "vector field type is unknown.")
    else:
      if not nmBase.NMBuiltInFieldTypes.has_key(ftype) and \
         not self.mDB['field_types'].has_key(ftype):
        self.XmlError("<%s ftype='%s'>" % (elem, ftype),
          "field type is unknown.")
    return ftype, vtype
  ##

  #--
  def XmlTokenFType(self, token):
    """ Parse XML element ftype token.

        Parameters:
          token  - Element ftype token raw value.

        Return:
          Returns ('vector', vtype) if vector.
          Returns (ftype, None) otherwise.
    """
    if not token:
      ftype = None
      vtype = None
    elif token[-2:] == nmBase.NMVectorSuffix:
      ftype = 'vector'
      vtype = token[:-2]
    else:
      ftype = token
      vtype = None
    ftype = nmBase.NMAliasMap.get(ftype, ftype)
    vtype = nmBase.NMAliasMap.get(vtype, vtype)
    return (ftype, vtype)
  ##

  #--
  def XmlChkTokenDispo(self, elem, dispo):
    """ Parse and validate XML element 'disposition' attribute.

        Parameters:
          elem    - Element name value.
          dispo   - Element attribute value.

        Return:
          dispo
    """
    enity = ['active', 'deprecated']
    if not dispo:
      return 'active'
    elif dispo not in enity:
      self.XmlError("<%s %s='%s'>" % (elem, 'disposition', dispo),
          "attribute value not one of: %s" % repr(enity))
    return dispo
  ##

  #--
  def XmlFieldDefContext(self):
    """ Get current field definition context.

        Return:
          Returns (pid, pinfo) - the parent element id/name and parent
          (implicit) field attributes.
    """
    elemParen = self.GetParentElem()
    if elemParen['elem'] == 'ftypedef':         # <ftypedef> field type
      pid     = elemParen['attrs']['ftid']
      pftype  = elemParen['attrs']['ftype']
    elif elemParen['elem'] == 'msgdef':         # <msgdef> implicit structure
      pid     = elemParen['attrs']['msgid']
      pftype  = 'struct'     
    elif elemParen['elem'] == 'fielddef':       # <fielddef> field type
      pid     = elemParen['attrs']['fname']
      pftype  = elemParen['attrs']['ftype']
    else:
      pid    = None
      pftype = None
    pftype, pvtype = self.XmlTokenFType(pftype)
    return pid, {'ftype': pftype, 'vtype': pvtype}
  ##

  #--
  def IsSimple(self, ftype):
    """ Returns True (False) is field type is (not) simple.

        Parameters:
          ftype   - (Derived) field type.
    """
    if nmBase.NMBuiltInFieldTypes.has_key(ftype):
      if nmBase.NMBuiltInFieldTypes[ftype]['comp'] == 'simple':
        return True
      else:
        return False
    while self.mDB['field_types'].has_key(ftype):
      ftype = self.mDB['field_types'][ftype]['ftype']
      if nmBase.NMBuiltInFieldTypes.has_key(ftype):
        if nmBase.NMBuiltInFieldTypes[ftype]['comp'] == 'simple':
          return True
        else:
          return False
    return False
  ##

  #--
  def PushElem(self, elem, attrs):
    """ Push element name on stack of elements.

        Parameters:
          elem    - Element name.
          attrs   - Element attributes.
    """
    if len(self._XmlElemStack) > 0:
      elemParent = self._XmlElemStack[-1]['elem']
    else:
      elemParent = '_root'
    if elem not in self.XmlTree[elemParent]:
      self.XmlError("<%s>" % (elem),
          "element not a valid child element of <%s>." % (elemParent))
    self._XmlElemStack += [{'elem':elem, 'attrs':attrs, 'data':''}]
    self._XmlCurElem    = self._XmlElemStack[-1]
    self.XmlDebug(3, "elem stack push:", repr(self._XmlCurElem))
  ##

  #--
  def PopElem(self):
    """ Pop element from stack of elements.
    """
    if len(self._XmlElemStack) > 0:
      self._XmlElemStack  = self._XmlElemStack[0:-1]
    if len(self._XmlElemStack) > 0:
      self._XmlCurElem    = self._XmlElemStack[-1]
    else:
      self._XmlCurElem    = None
    self.XmlDebug(3, "elem stack pop:", repr(self._XmlCurElem))
  ##

  #--
  def PushFieldDict(self, fielddict):
    """ Push field definitions dictionary on top of stack."

        Parameters:
          fielddict    - Field definitions dictionary.
    """
    self._DbFieldDictStack += [fielddict]
    self._DbCurFieldDict    = fielddict
    self.XmlDebug(3, "fielddict stack push:", repr(self._DbCurFieldDict))
  ##

  #--
  def PopFieldDict(self):
    """ Pop field definitions dictionary from top of stack.
    """
    if len(self._DbFieldDictStack ) > 0:
      self._DbFieldDictStack = self._DbFieldDictStack[0:-1]
    if len(self._DbFieldDictStack ) > 0:
      self._DbCurFieldDict = self._DbFieldDictStack[-1]
    else:
      self._DbCurFieldDict = None
    self.XmlDebug(3, "fielddict stack pop:", repr(self._DbCurFieldDict))
  ##

  #--
  def GetParentElem(self):
    """ Get the parent element of current top element.
    """
    if len(self._XmlElemStack) > 1:
      return self._XmlElemStack[-2]
    else:
      return {} 
  ##

  #--
  def GetParentElemName(self):
    """ Get the parent element's name of current top element.
    """
    if len(self._XmlElemStack) > 1:
      return self._XmlElemStack[-2]['elem']
    else:
      return None 
  ##

  #--
  def XmlWarning(self, *args):
    """ Print XML syntax warning.

        Parameters:
          *args   - List of warning message arguments.
    """
    wmsg = "Warning: %s[L%d,C%d]" % \
      (self.mBaseName,
       self._XmlParser.CurrentLineNumber,
       self._XmlParser.CurrentColumnNumber)
    for a in args:
      wmsg += ": %s" %(a)
    print wmsg
  ##

  #--
  def ExpatError(self, *args):
    """ Raise XML parser error on expat parser error.

        Parameters:
          *args   - List of error message arguments.
    """
    emsg = "%s[L%d,C%d]" % \
      (self.mBaseName,
       self._XmlParser.ErrorLineNumber,
       self._XmlParser.ErrorColumnNumber)
    for a in args:
      emsg += ": %s" %(a)
    raise NetMsgsXmlParserError(emsg)
  ##

  #--
  def XmlError(self, *args):
    """ Raise XML parser error on XML syntax or semanitc error.

        Parameters:
          *args   - List of warning message arguments.
    """
    emsg = "%s[L%d,C%d]" % \
      (self.mBaseName,
       self._XmlParser.CurrentLineNumber,
       self._XmlParser.CurrentColumnNumber)
    for a in args:
      emsg += ": %s" %(a)
    raise NetMsgsXmlParserError(emsg)
  ##

  #--
  def Error(self, *args):
    """ Raise XML parser error on general error.

        Parameters:
          *args   - List of error message arguments.
    """
    if self.mBaseName:
      emsg = "%s" % (self.mBaseName)
    else:
      emsg = ''
    for a in args:
      emsg += ": %s" %(a)
    raise NetMsgsXmlParserError(emsg)
  ##

  #--
  def XmlDebug(self, level, *args):
    """ Print XML debugging info.

        Parameters:
          level   - Debugging level.
          *args   - List of debug message arguments.
    """
    if level > self.mDebug:
      return
    dmsg = "DBG: %s[L%d,C%d]" % \
      (self.mBaseName,
       self._XmlParser.CurrentLineNumber,
       self._XmlParser.CurrentColumnNumber)
    for a in args:
      dmsg += ": %s" %(a)
    print dmsg
  ##

  #--
  def Debug(self, level, *args):
    """ Print debugging info.

        Parameters:
          level   - Debugging level.
          *args   - List of debug message arguments.
    """
    if level > self.mDebug:
      return
    dmsg = 'DBG'
    for a in args:
      dmsg += ": %s" %(a)
    print dmsg
  ##
##


#-------------------------------------------------------------------------------
# Unit Test Main
#-------------------------------------------------------------------------------
if __name__ == '__main__':
  """ Unit Test """

  import sys
  import getopt

  ## command name
  _Argv0    = __file__

  #--
  class UTUsage(Exception):
    """ Command-Line Options Usage Exception Class. """
    def __init__(self, msg):
      """ Raise usage excpetion 
          
          Parameters:
            msg   - Error message string.
      """
      ## error message attribute
      self.msg = msg
  ##

  #--
  def UTPrintUsageErr(emsg):
    """ Print Error Usage Message.
          
        Parameters:
          msg   - Error message string.
    """
    if emsg:
      print "%s: %s" % (_Argv0, emsg)
    else:
      print "%s: error" % (_Argv0)
    print "Try '%s --help' for more information." % (_Argv0)
  ##

  #--
  def UTPrintUsage():
    """ Print Command-Line Usage Message. """
    print """
usage: %s [OPTIONS] <xmlfile>

       %s --help
    """  % (_Argv0, _Argv0)
    print """Options and arguments:
  -d, --debug=<level>       : Debug level. One of: 0 1 2 3
                              Default: 0  (off)
  -o, --overrides=<list>    : XML overrides. List of form name=value,...
                              Supported overrides:
                                encoding, endian, ns, brief

  -h, --help                : Display this help and exit.
    """
  ##

  #--
  def UTGetOptions(argv=None, **kwargs):
    """ Get Main Options

        Parameters:
          argv    - Argument list. If not None, the overrides command-line
                    arguments.
          kwargs  - Keyword argument list.  
    """
    global _Argv0

    if argv is None:
      argv = sys.argv

    _Argv0 = kwargs.get('argv0', __file__)

    # defaults
    kwargs['debug'] = 0

    # parse command-line options
    try:
      try:
        opts, args = getopt.getopt(argv[1:], "?hd:o:",
            ['help', 'debug=', "overrides=", ''])
      except getopt.error, msg:
        raise UTUsage(msg)
      for opt, optarg in opts:
        if opt in ('-h', '--help', '-?'):
          UTPrintUsage()
          sys.exit(0)
        elif opt in ('-d', '--debug'):
          try:
            kwargs['debug'] = int(optarg)
          except ValueError:
            raise UTUsage("%s %s: not an integer value" % (opt, optarg))
        elif opt in ('-o', '--overrides'):
          overrides = optarg.split(',')
          for entry in overrides:
            try:
              name,value = entry.split('=')
              name = name.strip()
              value = value.strip()
              kwargs[name] = value
            except:
              raise UTUsage("%s %s: bad syntax" % (opt, repr(optarg)))
    except UTUsage, err:
      UTPrintUsageErr(err.msg)
      sys.exit(2)

    if len(args) < 1:
      UTPrintUsageErr("No input xml file specified")
      sys.exit(2)
    else:
      kwargs['filename'] = args[0]

    return kwargs
  ##

  #--
  def UTMain(argv=None, **kwargs):
    """ Unit Test Main.
        
        Parameters:
          argv    - Optional argument list to override command-line arguments.
          kwargs  - Optional keyword argument list.
    """
    global _Argv0
  
    kwargs = UTGetOptions(argv, **kwargs)

    xmlNetMsgs = NetMsgsXmlParser(**kwargs)

    try:
      xmlNetMsgs.Parse()
    except NetMsgsXmlParserError as inst:
      print "Error: %s" % (inst)
      return 8

    xmlNetMsgs.PrettyPrintDB()
  ##

  ## Run unit test code.
  sys.exit( UTMain() )
