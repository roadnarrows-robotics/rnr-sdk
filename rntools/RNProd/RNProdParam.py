#!python
###############################################################################
#
# RNProdParam.py
#

""" RoadNarrows Product Make Tools - Product Parameters

The RNProdParam module specifies the (semi) fixed RoadNarrows product
parameters.

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

# Fully Qualified Directory Paths
RNProdFqDirTools            = '/prj/tools'
RNProdFqDirToolsRNProd      = RNProdFqDirTools+'/RNProd'
RNProdFqDirTemplates        = RNProdFqDirTools+'/templates'

# Relative Directories
RNProdDirDotBuild           = '.build'
RNProdDirMake               = 'make'
RNProdPkgRNMake             = '/prj/pkg/rnmake'

# Relative Template File Names
RNProdTemplateExt           = '.in'
RNProdTemplateTLMake        = 'Makefile.in'
RNProdTemplateProdMake      = 'Prod.mk.in'
RNProdTemplateSrcDocIndex   = 'srcdoc.html.in'
RNProdTemplateExcepts       = 'Exceptions.xml.in'
RNProdTemplateReadMe        = 'README.txt.in'
RNProdTemplateEULA          = 'EULA.txt.in'
RNProdTemplateEULAHtml      = 'EULA.html.in'
RNProdTemplateManifest      = 'Manifest.html.in'

# Product Defaults
RNProdDftStageRoot          = '/prj/stage'
RNProdDftRNDiv              = 'rnr'

# Make commands
RNProdMakeTarball           = 'make tarball-src'

# RN Divisions
RNProdRNDiv                 = {
    'rn':   { 'rndiv_fqname': 'RoadNarrows LLC',
              'rndiv_email':  'oneway@roadnarrows.com',
              'rndiv_url':    'http://www.roadnarrows.com',
              'rndiv_disclaimer':
'"This is free software; see the source for copying conditions. There is NO\\n'+
'warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE."'
            },
    'rnr':  { 'rndiv_fqname': 'RoadNarrows Robotics',
              'rndiv_email':  'oneway@roadnarrowsrobotics.com',
              'rndiv_url':    'http://www.roadnarrowsrobotics.com',
              'rndiv_disclaimer':
'"This is free software; see the source for copying conditions. There is NO\\n'+
'warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE."'
            },
    'rnis': { 'rndiv_fqname': 'RoadNarrows Intelligent Systems LLC',
              'rndiv_email':  'support@rnisys.com',
              'rndiv_url':    'http://www.rnisys.com',
              'rndiv_disclaimer':
'"RoadNarrows Intelligent Systesm End User License Agreement\\n\\n'+
'This open source file provided by RoadNarrows Intelligent Systems is\\n'+
'covered under the GNU Lesser General Public License (LGPLv3)."'
            },
}
