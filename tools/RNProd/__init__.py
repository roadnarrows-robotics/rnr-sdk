################################################################################
#
# __init__.py
#

""" RoadNarrows Product Make Tools

This package provides the set of tools to build a RoadNarrows digital
product from a set of packages.

Public Modules:
  RNProdAtAt      - Scan and process '@@' identifiers in template files.
  RNProdExceptXml - Class RNProdExceptXml module. Parses an RN product
                    Exceptions.xml file.
  RNProdParam     - Specifies the (semi) fixed RoadNarrows product parameters.
  RNProdProdXml   - Class RNProdXml module. Creates or parses a RoadNarrows
                    product Prod.xml master file.
  RNProdReadMeXml - Class RNProdReadMeXml module. Parses an RN package
                    README.txt XML formatted file.
  RNProdStage     - Interactive staging of a product.
  RNProdXmlParser - Base XML parser class module.

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

__all__ = [
  'RNProdAtAt',
  'RNProdExceptXml',
  'RNProdParam',
  'RNProdProdXml',
  'RNProdReadMeXml',
  'RNProdStage',
  'RNProdXmlParse'
]
