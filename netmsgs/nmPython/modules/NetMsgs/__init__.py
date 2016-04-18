################################################################################
#
# __init__.py
#

""" RoadNarrows Robotics NetMsgs Generation Tools.

The NetMsgs python module provides the set of tools to generate
language-specific packing and unpacking routines, plus run-time message
processing routines. The generated output is specified from RoadNarrows
NetMsgs XML input files.

Supported Output Language Generation:
  C <name>.h and <name>.c
  Python <name>.py

Public Modules:
  NetMsgCore          - Swigged core interface.
  NetMsgBase          - Base data and utilities.
  NetMsgsGenC         - Generate C .h and .c files from XML specification.
  NetMsgsGenPy        - Generate Python .py file from XML specification.
  NetMsgsLib          - Run-Time library packing/unpacking base functions.
  NetMsgsLibFlat      - Run-Time library packing/unpacking derived flat class.
  NetMsgsLibITV       - Run-Time library packing/unpacking derived ITV class.
  NetMsgsLibStreamBuf - Run-Time library packing/unpacking base class.
  NetMsgsXmlParser    - XML parser for NetMsgs XML.

Author: Robin D. Knight
Email:  robin.knight@roadnarrows.com
URL:    http://www.roadnarrows.com

Copyright (C) 2009-2010.  RoadNarrows LLC.
All Rights Reserved

$LastChangedDate: 2010-08-04 15:07:55 -0600 (Wed, 04 Aug 2010) $
$Rev: 550 $
"""

## \file 
## \package NetMsgs
##
## $LastChangedDate: 2010-08-04 15:07:55 -0600 (Wed, 04 Aug 2010) $
## $Rev: 550 $
##
## \brief NetMsgs Base Data Module
##
## \sa
## \htmlonly
##  <a href="../pydoc/NetMsgs.html">PyDoc Generated Documentation</a>
## \endhtmlonly
##
## \author Robin Knight (robin.knight@roadnarrows.com)
##  
## \par Copyright:
##   (C) 2009-2010.  RoadNarrows LLC.\n
##   (http://www.roadnarrows.com)\n
##   All Rights Reserved
##

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

## All modules in NetMsgs package.
__all__ = [
  'NetMsgsCore',
  'NetMsgsBase',
  'NetMsgsGenC',
  'NetMsgsGenPy',
  'NetMsgsLib',
  'NetMsgsLibFlat',
  'NetMsgsLibITV',
  'NetMsgsLibStreamBuf',
  'NetMsgsXmlParser',
]
