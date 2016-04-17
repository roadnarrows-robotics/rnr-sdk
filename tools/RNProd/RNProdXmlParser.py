###############################################################################
#
# RNProdXmlParser.py
#

""" RoadNarrows Product Make Tools - Base XML Parser

The RNProdXmlParser module provides the base virtual class to create or parse
an XML file.

Author: Robin D. Knight
Email:  robin.knight@roadnarrows.com
URL:    http://www.roadnarrows.com

Copyright (C) 2009.  RoadNarrows LLC.
All Rights Reserved

$LastChangedDate: 2009-09-04 12:09:46 -0600 (Fri, 04 Sep 2009) $
$Rev: 118 $
"""

#
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
################################################################################

import os
import time
import xml.parsers.expat as expat

import RNProdParam as Param


#------------------------------------------------------------------------------
# CLASS: XmlParserError
#------------------------------------------------------------------------------
class XmlParserError(Exception):
  """ XML Parser Exception Class. """

  def __init__(self, msg='XML Parser Error'):
    Exception.__init__(self, msg)


#------------------------------------------------------------------------------
# CLASS: XmlParser
#------------------------------------------------------------------------------
class XmlParser:
  """ XML Parser Base Class.

      The XmlParser virtual base class creates, parses, and encapsulates an
      XML file. The parsed information is available to the calling program.
  """

  #--
  def __init__(self, filename=None, debug=False):
    """ Initialize RNProdXml instance.

        Parameters:
          filename    - README XML file name.
          debug       - Turn on debugging.
    """
    self.mFileName      = filename        # associated xml file name
    self.mDebug         = debug           # do [not] print debug info
    self.mPData         = { }             # parsed data
    self.mPDataDft      = { }             # parsed data defaults

    self._XmlParser     = None            # xml parser
    self._XmlElemStack  = []              # xml element name stack
    self._XmlDataStack  = []              # xml element data stack
    self._XmlCurElem    = None            # current element name
    self._XmlCurData    = ''              # current element data
    self._XmlTree       = {'_root': []}   # xml tree starting at '_root'

    # list of elements in preferred order
    self._XmlElems = self._InitElemOList(self._XmlTree['_root'])

  #--
  def _InitElemOList(self, elemList):
    """ Make an ordered list of XML element names.

        Names are derieved from the XML tree.

        Parameters:
          elemList  - List of element names

        Return Value:
          Orders element list.
    """
    olist = []
    for elem in elemList:
      olist += [elem]
      if self._XmlTree.has_key(elem):
        olist += self._InitElemOList(self._XmlTree[elem])
    return olist

  #--
  def __getitem__(self, pname):
    """ x.__getitem__(pname) <==> x[pname]

        Get parsed and converted data.
    """
    return self.mPData[pname]

  #--
  def __setitem__(self, pname, val):
    """ x.__setitem__(pname, val) <==> x[pname]=val

        Set (new) pname data value.
    """
    self.mPData[pname] = val

  #--
  def GetPData(self, pname):
    """ Get parsed and converted data.

        Parameters:
          pname   - Parsed data key.

        Return Value:
          Parsed, converted data.
    """
    return self.mPData[pname]

  #--
  def GetPDataDict(self):
    """ Get dictionary of all parsed data.

        Return Value:
          Dictionary.
    """
    return self.mPData

  #--
  def HasPDataDft(self, pname):
    """ Returns True if data has a default value.

        Parameters:
          pname   - Parsed data key.

        Return Value:
          True or False
    """
    return self.mPDataDft.has_key(pname)

  #--
  def GetPDataDft(self, pname):
    """ Get the data default value.

        Parameters:
          pname   - Parsed data key.

        Return Value:
          Extracted parsed, converted data.
    """
    return self.mPDataDft[pname]

  #--
  def GetElemList(self):
    """ Get the full list of XML element names.
        
        The list is in preferred output order.

        Return Value:
          List of element names.
    """
    return self._XmlElems
    
  #--
  def GetPNameList(self):
    """ Get the full list of parsed data key names.

        Return Value:
          List of pnames.
    """
    return self.mPData.keys()
    
  #--
  def ParseFile(self, filename):
    """ Parse the Xml XML data specified in the given file.

        Parameters:
          filename    - README XML file name.
    """
    self.mFileName  = filename
    self.Parse()

  #--
  def Parse(self):
    """ Parse the Xml XML data listed in the current file.
    """
    if not self.mFileName:
      self._Error('No filename')
    try:
      fp = open(self.mFileName, 'r')
    except IOError, err:
      self._Error(self.mFileName, err)
    self.Reset()
    self._XmlParser = expat.ParserCreate()
    self._XmlParser.returns_unicode       = False
    self._XmlParser.StartElementHandler   = self.XmlHandlerStartElem
    self._XmlParser.CharacterDataHandler  = self.XmlHandlerCharData
    self._XmlParser.EndElementHandler     = self.XmlHandlerEndElem
    self._XmlParser.CommentHandler        = self.XmlHandlerComment
    try:
      self._XmlParser.ParseFile(fp)
    except expat.ExpatError as e:
      self._XmlError(expat.ErrorString(e.code))
    self.Validate()

  #--
  def Validate(self):
    """ Validate parsed data. """
    for pname in self.mPData.keys():
      if pname not in self._XmlElems: # ignore derived data - already validated
        continue
      elif self.mPData[pname]:        # data present and already validated
        continue
      elif not self.mPDataDft.has_key(pname): 
        self._Error("<%s>: required element data not specified" % (pname))

  #--
  def WriteXmlFile(self, filename, comment=None):
    """ Create a Product XML file from the given parsed or set data.

        Parameters:
          filename    - Product XML file name.
          comment     - Comment string.
    """
    fp = open(filename, 'w')
    self.WriteXml(fp, comment)

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
    self.WriteXmlTree(fp, 0, self._XmlTree['_root'])
    fp.flush()
    fp.close()

  #--
  def WriteXmlTree(self, fp, level, elemList):
    """ Write out XML tree.

        Parameters:
          fp        - Opened for writing file pointer.
          level     - element level (depth)
          elemList  - element level (depth)
    """
    for elem in elemList:
      if self._XmlTree.has_key(elem):
        fp.write("%*s<%s>\n" % (level*2, "", elem))
        self.WriteXmlTree(fp, level+1, self._XmlTree[elem])
        fp.write("%*s</%s>\n" % (level*2, "", elem))
      elif elem == 'pkg':     # special
        level += 1
        for pkg in self.mPData['pkg_list']:
          fp.write("%*s<%s>" % (level*2, "", elem))
          fp.write(pkg)
          fp.write("</%s>\n" % (elem))
        level -= 1
      else:
        fp.write("%*s<%s>" % (level*2, "", elem))
        fp.write(self.mPData[elem])
        fp.write("</%s>\n" % (elem))

  #--
  def Reset(self):
    """ Reset the XML parser.
    """
    if self._XmlParser is not None:
      del self._XmlParser
      self._XmlParser = None
    self._XmlElemStack  = []
    self._XmlDataStack  = []
    self._XmlCurElem    = None
    self._XmlCurData    = ''

  #--
  def SetElemVal(self, elem, data):
    """ Set the parsed and converted data associated with the given element.

        The data must conform to the requirements of the element, whether the
        data is read from an XML file or provided by the calling routine.

        Parameters:
          elem    - XML element name.
          data    - Unparsed string data.
    """
    data = data.strip()
    self.SetElemString(elem, data)

  #--
  def SetElemString(self, elem, data):
    """ Parse and convert element string data

        Parameters:
          elem    - XML element name.
          data    - String data.

        Return Value:
          Parsed associated pdata.
    """
    self.mPData[elem] = data
    return self.mPData[elem]
  
  #--
  def XmlHandlerStartElem(self, elem, attrs):
    """ Parser callback at start of element.

        Parameters:
          elem    - Element name.
          attrs   - Dictionary of element attributes.
    """
    if self.mDebug: print "start element:", elem, attrs
    if elem in self._XmlElems:
      self._PushElem(elem)
    else:
      self._XmlWarning(elem, "unrecognized element")

  #--
  def XmlHandlerEndElem(self, elem):
    """ Parser callback at end of element.

        Parameters:
          elem    - Element name.
    """
    if self.mDebug: print "end element:", elem
    if elem == self._XmlCurElem:
      self.SetElemVal(self._XmlCurElem, self._XmlCurData)
      self._PopElem()

  #--
  def XmlHandlerCharData(self, data):
    """ Parser callback for each line of element data.

        Parameters:
          data    - Unstructured element data.
    """
    if self.mDebug: print "char data:", repr(data)
    self._XmlCurData += data

  #--
  def XmlHandlerComment(self, comment):
    """ Parser callback for each comment block.

        Parameters:
          comment - Comment text sans '<!-' and '-->'.
    """
    if self.mDebug: print "comment:", repr(comment)

  #--
  def _PushElem(self, elem):
    """ Push element name on stack of elements."

        Parameters:
          elem    - Element name.
    """
    if len(self._XmlElemStack) > 0:
      elemParent = self._XmlElemStack[-1]
    else:
      elemParent = '_root'
    if elem not in self._XmlTree[elemParent]:
      self._Error("<%s>" % (elem),
          "element not a valid child element of <%s>" % (elemParent))
    if len(self._XmlDataStack) > 0:
      self._XmlDataStack[-1] += self._XmlCurData
    self._XmlElemStack += [elem]
    self._XmlDataStack += ['']
    self._XmlCurElem    = elem
    self._XmlCurData    = ''
    #if self.mDebug:
    #  print "push elem stack:", self._XmlElemStack, repr(self._XmlDataStack)

  #--
  def _PopElem(self):
    """ Pop element from stack of elements."
    """
    if len(self._XmlElemStack) > 0:
      self._XmlElemStack  = self._XmlElemStack[0:-1]
      self._XmlDataStack  = self._XmlDataStack[0:-1]
    if len(self._XmlElemStack) > 0:
      self._XmlCurElem    = self._XmlElemStack[-1]
      self._XmlCurData    = self._XmlDataStack[-1]
    else:
      self._XmlCurElem    = None
      self._XmlCurData    = ''
    #if self.mDebug:
    #  print "pop elem stack:", self._XmlElemStack, repr(self._XmlDataStack)

  #--
  def _GetParentElemName(self):
    """ Get the parent element's name of current top element.
    """
    if len(self._XmlElemStack) > 1:
      return self._XmlElemStack[-2]
    else:
      return None 

  #--
  def _XmlWarning(self, *args):
    """ Print XML syntax warning.

        Parameters:
          *args   - List of warning message arguments.
    """
    wmsg = "Warning: %s[%d,%d]" % \
      (self.mFileName,
       self._XmlParser.CurrentLineNumber,
       self._XmlParser.CurrentColumnNumber)
    for a in args:
      wmsg += ": %s" %(a)
    print wmsg

  #--
  def _XmlError(self, *args):
    """ Print XML syntax error.

        Parameters:
          *args   - List of error message arguments.
    """
    emsg = "%s[%d,%d]" % \
      (self.mFileName,
       self._XmlParser.ErrorLineNumber,
       self._XmlParser.ErrorColumnNumber)
    for a in args:
      emsg += ": %s" %(a)
    raise XmlParserError(emsg)

  #--
  def _Error(self, *args):
    """ Print error.

        Parameters:
          *args   - List of error message arguments.
    """
    if self.mFileName:
      emsg = "%s" % (self.mFileName)
    else:
      emsg = ''
    for a in args:
      emsg += ": %s" %(a)
    raise XmlParserError(emsg)
