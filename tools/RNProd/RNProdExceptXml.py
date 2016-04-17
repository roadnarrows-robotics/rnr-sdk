###############################################################################
#
# RNProdExceptXml.py
#

""" RoadNarrows Product Make Tools - README XML Parser

The RNProdExceptXml module provides a class to parse a RoadNarrows product
Exceptions.xml file specified in XML. The parsed information is available to the
calling program.

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

import RNProdXmlParser


#------------------------------------------------------------------------------
# CLASS: RNProdExceptXml
#------------------------------------------------------------------------------
class RNProdExceptXml(RNProdXmlParser.XmlParser):
  """ RoadNarrows Product Exceptions XML Class.

      The RNProdExceptXml class parses a RoadNarrows product Exceptions.xml
      file specified in XML. The parsed information is available to the
      calling program.

      XML Syntax:
      <exceptions>
        <prune_stage>
          <path>relpathname</path>
          ...
        </prune_stage>
        <prune_src>
          <path>relpathname</path>
          ...
        </prune_src>
        <prune_srcdoc>
          <path>relpathname</path>
          ...
        </prune_srcdoc>
      </exceptions>
  """

  def __init__(self, filename=None, rndiv=None, debug=False):
    """ Initialize RNProdExceptXml instance.

        Parameters:
          filename    - README XML file name.
          rndiv       - RN Business division.
          debug       - Turn on debugging.
    """
    RNProdXmlParser.XmlParser.__init__(self, filename, debug)

    self.mPData         = {               # parsed data
                            'prune_stage':      [],
                            'prune_src':        [],
                            'prune_srcdoc':     []
                          }
    self.mPDataDft      = {               # parsed data defaults
                            'prune_stage':      [],
                            'prune_src':        [],
                            'prune_srcdoc':     []
                          }

    self._XmlTree       = {               # xml tree starting at '_root'
                            '_root':        ['exceptions'],
                            'exceptions':   [ 'prune_stage', 'prune_src',
                                              'prune_srcdoc',
                                            ],
                            'prune_stage':  [ 'path' ],
                            'prune_src':    [ 'path' ],
                            'prune_srcdoc': [ 'path' ]
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
    """ Create a README XML file from the given parsed or set data.

        Parameters:
          fp    - Opened for writing file pointer.
    """
    comment = "RoadNarrows LLC %s Product Information." % \
                  (self.mPData['package'])
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
    if elem == 'path':
      self.AddElemPath(self._GetParentElemName(), data)
    else:
      pass

  #--
  def SetElemPruneStage(self, data):
    """ Parse and convert 'prune_stage' element raw string data.

        Parameters:
          data    - Raw string data.

        Return Value:
          Parsed associated pdata.
    """
    elem = 'prune_stage'
    if type(data) != list:
      data = [data]
    self.mPData[elem] = []
    for p in data:
      self.AddElemPath(elem, p)
    return self.mPData[elem]

  #--
  def AddElemPath(self, elemList, data):
    """ Parse and convert 'Path' element raw string data.

        The package is appended to the 'elem' pdata.

        Parameters:
          elemList  - List container element name.
          data      - Raw string data.

        Return Value:
          Parsed associated pdata.
    """
    elem = 'path'
    data = os.path.normpath(data)
    self.mPData[elemList] += [data]
    return self.mPData[elemList]


#-------------------------------------------------------------------------------
# Unit Test Code
#-------------------------------------------------------------------------------

if __name__ == '__main__':
  def main():
    """ Unit Test Main """
    x = RNProdExceptXml(filename="tests/except1.xml", debug=True)
    x.Parse()
    for elem in ['prune_stage', 'prune_src', 'prune_srcdoc']:
      print elem, '=', repr(x[elem])

  # run unit test
  main()
