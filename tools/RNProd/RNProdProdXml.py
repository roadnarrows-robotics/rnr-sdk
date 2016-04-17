###############################################################################
#
# RNProdProdXml.py
#

""" RoadNarrows Product Make Tools - Prod XML Parser

The RNProdXml module provides a class to create or parse a RoadNarrows product
Prod.xml file specified in XML. The parsed information is available to the
calling program in a standard internal representation.

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

import RNProdParam as Param
import RNProdXmlParser


#------------------------------------------------------------------------------
# CLASS: RNProdXml
#------------------------------------------------------------------------------
class RNProdProdXml(RNProdXmlParser.XmlParser):
  """ RoadNarrows Product XML Class.

      The RNProdXml class creates and parses a RoadNarrows product XML file.
      The parsed information is available to the calling program.

      XML Syntax:
      <rnprod>
        <stage_root>fqdir</stage_root>
        <rndiv>key</rndiv>
        <prod_name>name</prod_name>
        <prod_ver>x.y.z[-spec]</prod_ver>
        <synopsis>text</synopsis>
        <pkg_list>
          <pkg>fqdir</pkg>
          <pkg>fqdir</pkg>
          ...
        </pkg_list>
        <description>text</description>
        <except_file>xmlfile</except_file>
        <readme_file>xmlfile</readme_file>
      </rnprod>
  """

  #--
  def __init__(self, filename=None, debug=False):
    """ Initialize RNProdXml instance.

        Parameters:
          filename    - README XML file name.
          debug       - Turn on debugging.
    """
    RNProdXmlParser.XmlParser.__init__(self, filename, debug)

    self.mPData         = {               # parsed data
                            'stage_root':       Param.RNProdDftStageRoot,
                            'rndiv':            Param.RNProdDftRNDiv,
                            'rndiv_fqname':     None,
                            'rndiv_email':      None,
                            'rndiv_url':        None,
                            'rndiv_eula':       None,
                            'prod_name':        None,
                            'prod_fqname':      None,
                            'prod_ver':         None,
                            'prod_ver_major':   None,
                            'prod_ver_minor':   None,
                            'prod_ver_rev':     None,
                            'prod_ver_spec':    None,
                            'synopsis':         '',
                            'rnmake':           Param.RNProdPkgRNMake,
                            'pkg_list':         [],
                            'description':      '',
                            'except_file':      '',
                            'readme_file' :     ''
                          }
    self.mPDataDft      = {               # parsed data defaults
                            'stage_root':       Param.RNProdDftStageRoot,
                            'rndiv':            Param.RNProdDftRNDiv,
                            'synopsis':         '',
                            'description':      '',
                            'rnmake':           Param.RNProdPkgRNMake,
                            'except_file':      '',
                            'readme_file' :     ''
                          }

    self._XmlTree       = {               # xml tree starting at '_root'
                            '_root':  ['rnprod'],
                            'rnprod': [ 'stage_root', 'rndiv', 'prod_name',
                                        'prod_ver', 'synopsis', 'description',
                                        'rnmake', 'pkg_list',
                                        'except_file', 'readme_file'
                                      ],
                            'pkg_list': ['pkg']
                          }

    # list of elements in preferred order
    self._XmlElems = self._InitElemOList(self._XmlTree['_root'])

  #--
  def WriteXmlFile(self, filename):
    """ Create a Product XML file from the given parsed or set data.

        Parameters:
          filename    - Product XML file name.
    """
    fp = open(filename, 'w')
    self.WriteXml(fp)

  #--
  def WriteXml(self, fp):
    """ Create a Product XML file from the given parsed or set data.

        Parameters:
          fp    - Opened for writing file pointer.
    """
    comment = "RoadNarrows LLC %s Product Information." % \
                  (self.mPData['prod_fqname'])
    RNProdXmlParser.XmlParser.WriteXml(self, fp, comment)

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
    if elem == 'stage_root':
      self.SetElemStageRoot(data)
    elif elem == 'rndiv':
      self.SetElemRNDiv(data)
    elif elem == 'prod_name':
      self.SetElemProdName(data)
    elif elem == 'prod_ver':
      self.SetElemProdVer(data)
    elif elem == 'synopsis':
      self.SetElemSynopsis(data)
    elif elem == 'rnmake':
      self.SetElemRNMake(data)
    elif elem == 'pkg':
      self.AddElemPkg(data)
    elif elem == 'description':
      self.SetElemDescription(data)
    elif elem == 'except_file':
      self.SetElemExceptFile(data)
    elif elem == 'readme_file':
      self.SetElemReadMeFile(data)
    else:
      pass

  #--
  def SetElemStageRoot(self, data):
    """ Parse and convert 'stage_root' element raw string data.

        Parameters:
          data    - Raw string data.

        Return Value:
          Parsed associated pdata.
    """
    elem = 'stage_root'
    if not os.path.isdir(data):
      self._Error("<%s>" % (elem), "%s: not a directory" % (repr(data)))
    self.mPData[elem] = data
    return self.mPData[elem]
  
  #--
  def SetElemRNDiv(self, data):
    """ Parse and convert 'rndiv' element raw string data.

        Parameters:
          data    - Raw string data.

        Return Value:
          Parsed associated pdata.
    """
    elem = 'rndiv'
    if not data in Param.RNProdRNDiv.keys():
      self._Error("<%s>" % (elem), "%s: not a valid RN division" % (repr(data)))
    rndiv = self.mPData[elem]   = data
    self.mPData['rndiv_fqname'] = Param.RNProdRNDiv[rndiv]['rndiv_fqname']
    self.mPData['rndiv_email']  = Param.RNProdRNDiv[rndiv]['rndiv_email']
    self.mPData['rndiv_url']    = Param.RNProdRNDiv[rndiv]['rndiv_url']
    self.mPData['rndiv_eula']   = self.AtAtCbVirtual
    return self.mPData[elem]
  
  #--
  def SetElemProdName(self, data):
    """ Parse and convert 'prod_name' element raw string data.

        Parameters:
          data    - Raw string data.

        Return Value:
          Parsed associated pdata.
    """
    elem = 'prod_name'
    for c in data:
      if not c.isalnum() and c not in ['-', '_', '.']:
        self._Error("<%s>" % (elem), "not a valid product name")
    self.mPData[elem]   = data
    if self.mPData['prod_ver']:
      self.mPData['prod_fqname'] = self.mPData[elem] + '-' + \
                                    self.mPData['prod_ver']
    return self.mPData[elem]
  
  #--
  def SetElemProdVer(self, data):
    """ Parse and convert 'prod_ver' element raw string data.

        Parameters:
          data    - Raw string data.

        Return Value:
          Parsed associated pdata.
    """
    elem      = 'prod_ver'
    vernames  = [ elem,           'prod_ver_major',  'prod_ver_minor',
                  'prod_ver_rev', 'ver_spec']
    vercomps  = [data]
    verspec   = ''
    valstrs   = data.split('.')
    n = 1
    for s in valstrs:
      if not s:
        self._Error("<%s>" % (elem), "no %s component found" % (vernames[n]))
      if n == 3:
        k = s.find('-')
        if k > 0:
          verspec = s[k+1:]
          s = s[:k]
      try:
        int(s)
      except ValueError, inst:
        self._Error("<%s>" % (elem), "%s=%s: not a number" % (vernames[n], s))
      vercomps += [s]
      n += 1
    if n < 4:
      self._Error("<%s>" % (elem), "no %s component found" % (vernames[n]))
    if verspec:
      vercomps += [verspec]
    else:
      vercomps += ['']
    n = 0
    for comp in vercomps:
      self.mPData[vernames[n]] = vercomps[n]
      n += 1
    if self.mPData['prod_name']:
      self.mPData['prod_fqname'] = self.mPData['prod_name'] + '-' + \
                                    self.mPData[elem]
    return self.mPData[elem]
  
  #--
  def SetElemSynopsis(self, data):
    """ Parse and convert 'synopsis' element raw string data.

        Parameters:
          data    - Raw string data.

        Return Value:
          Parsed associated pdata.
    """
    elem = 'synopsis'
    self.mPData[elem] = data
    return self.mPData[elem]
  
  #--
  def SetElemDescription(self, data):
    """ Parse and convert 'description' element raw string data.

        Parameters:
          data    - Raw string data.

        Return Value:
          Parsed associated pdata.
    """
    elem = 'description'
    self.mPData[elem] = data
    return self.mPData[elem]
  
  #--
  def SetElemRNMake(self, data):
    """ Parse and convert 'rnmake' element raw string data.

        Parameters:
          data    - Raw string data.

        Return Value:
          Parsed associated pdata.
    """
    elem = 'rnmake'
    if not os.path.isdir(data):
      self._Error("<%s>" % (elem), "%s: not a directory" % (repr(data)))
    self.mPData[elem] = data
    return self.mPData[elem]
  
  #--
  def SetElemPkgList(self, data):
    """ Parse and convert 'pkg_list' element raw string data.

        Parameters:
          data    - Raw string data.

        Return Value:
          Parsed associated pdata.
    """
    elem = 'pkg_list'
    if type(data) != list:
      data = [data]
    if len(data) == 0:
      self._Error("<%s>" % (elem), "at least one package is required")
    for pkg in data:
      self.AddElemPkg(pkg)
    return self.mPData[elem]

  #--
  def AddElemPkg(self, data):
    """ Parse and convert 'pkg' element raw string data.

        The package is appended to the 'pkg_list' pdata.

        Parameters:
          data    - Raw string data.

        Return Value:
          Parsed associated pdata.
    """
    elem = 'pkg_list'
    if not os.path.isdir(data):
      self._Error("<%s>" % (elem), "%s: not a directory" % (repr(data)))
    self.mPData[elem] += [data]
    return self.mPData[elem]

  #--
  def SetElemExceptFile(self, data):
    """ Parse and convert 'except_file' element raw string data.

        Parameters:
          data    - Raw string data.

        Return Value:
          Parsed associated pdata.
    """
    elem = 'except_file'
    if data:
      if not os.path.isfile(data):
        self._Error("<%s>" % (elem), "%s: file does not exist" % (repr(data)))
    else:
      data = ''
    self.mPData[elem] = data
    return self.mPData[elem]
  
  #--
  def SetElemReadMeFile(self, data):
    """ Parse and convert 'readme_file' element raw string data.

        Parameters:
          data    - Raw string data.

        Return Value:
          Parsed associated pdata.
    """
    elem = 'readme_file'
    if data:
      if not os.path.isfile(data):
        self._Error("<%s>" % (elem), "%s: file does not exist" % (repr(data)))
    else:
      data = ''
    self.mPData[elem] = data
    return self.mPData[elem]
  
  #--
  def AtAtCbVirtual(self, fp, id, prodDict, fmt):
    self._Error("%s is virtual - override" % (id))

#-------------------------------------------------------------------------------
# Unit Test Code
#-------------------------------------------------------------------------------

if __name__ == '__main__':
  def main():
    """ Unit Test Main """
    x = RNProdXml(filename="tests/prod1.xml", debug=False)
    x.Parse()
    print x.mPData
    print x.GetElemList()
    x.WriteXmlFile("tests/prod.out.xml")

  # run unit test
  main()
