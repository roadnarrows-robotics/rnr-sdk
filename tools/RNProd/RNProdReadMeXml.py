###############################################################################
#
# RNProdReadMeXml.py
#

""" RoadNarrows Product Make Tools - README XML Parser

The RNProdReadMeXml module provides a class to parse a RoadNarrows package
README.xml file specified in XML. The parsed information is available to the
calling program.

Author: Robin D. Knight
Email:  robin.knight@roadnarrows.com
URL:    http://www.roadnarrows.com

Copyright (C) 2009.  RoadNarrows LLC.
All Rights Reserved

$LastChangedDate: 2010-08-04 15:01:38 -0600 (Wed, 04 Aug 2010) $
$Rev: 547 $
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

import time

import RNProdParam as Param
import RNProdXmlParser


#------------------------------------------------------------------------------
# CLASS: RNProdReadMeXml
#------------------------------------------------------------------------------
class RNProdReadMeXml(RNProdXmlParser.XmlParser):
  """ RoadNarrows Product ReadMe XML Class.

      The RNProdReadMeXml class parses a RoadNarrows package README.xml file
      specified in XML. The parsed information is available to the
      calling program.

      XML Syntax:
      <readme>
        <package>name</package>
        <synopsis>text</synopsis>
        <rndiv>key</rndiv>
        <description>long text</description>
        <dependency_list>
          <depend>pkg</depend>
          ...
        </dependency_list>
        <notes>notes</notes>
        <seealso>related packages, etc</seealso>
        <owner>author, company, urls, etc</owner>
      </readme>
  """

  def __init__(self, filename=None, rndiv=None, debug=False):
    """ Initialize RNProdReadMeXml instance.

        Parameters:
          filename    - README XML file name.
          rndiv       - RN Business division.
          debug       - Turn on debugging.
    """
    RNProdXmlParser.XmlParser.__init__(self, filename, debug)

    self.mPData         = {               # parsed data
                            'package':          '',
                            'synopsis':         '',
                            'rndiv':            Param.RNProdDftRNDiv,
                            'description':      '',
                            'notes':            '',
                            'seealso':          '',
                            'owner':            ''
                          }
    self.mPDataDft      = {               # parsed data defaults
                            'rndiv':            Param.RNProdDftRNDiv,
                            'description':      '',
                            'notes':            '',
                            'seealso':          '',
                            'owner':            ''
                          }

    self._XmlTree       = {               # xml tree starting at '_root'
                            '_root':  ['readme'],
                            'readme': [ 'package', 'synopsis', 'rndiv',
                                        'description', 'notes', 'seealso',
                                        'owner']
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
    if elem == 'rndiv':
      self.SetElemRNDiv(data)
    elif elem in self.mPData.keys():
      self.mPData[elem] = data.strip()
    else:
      pass

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
    return self.mPData[elem]
  

#-------------------------------------------------------------------------------
# Unit Test Code
#-------------------------------------------------------------------------------

if __name__ == '__main__':
  def main():
    """ Unit Test Main """
    x = RNProdReadMeXml(filename="tests/readme1.xml", debug=True)
    x.Parse()
    print x.GetPData('package')
    print repr(x['synopsis'])
    x.WriteXmlFile('tests/readme.out')

  # run unit test
  main()
