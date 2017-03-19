################################################################################
#
# __init__.py
#

"""
RoadNarrows BotSense Module.
"""

## \file 
## \package BotSense
##
## $LastChangedDate: 2010-10-05 08:17:38 -0600 (Tue, 05 Oct 2010) $
## $Rev: 615 $
##
## \brief BotSense Client Core
##
## The BotSense python package provides the core client modules plus the
## standard proxied device plug-ins. The client modules communicate with
## the \h_botsense bsProxy server.
##
## \sa
## \htmlonly
##  <a href="../pydoc/BotSense.html">PyDoc Generated Documentation</a>
## \endhtmlonly
##
## \author Robin Knight (robin.knight@roadnarrows.com)
##  
## \copyright
##   \h_copy 2010-2017. RoadNarrows LLC.\n
##   http://www.roadnarrows.com\n
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

## All modules in BotSense package.
__all__ = [
  'BotSenseCore',
  'BotSenseError',
  'BotSenseTypes',
  'BotSenseServer',
  'bsProxyMsgs',
  'bsI2C',
  'bsI2CMsgs',
  'bsNull',
  'bsNullMsgs',
  'bsSerial',
  'bsSerialMsgs',
]
