################################################################################
#
# ./sw/libkuon/Makefile

ifdef RNMAKE_DOXY
/*!
\file

$LastChangedDate: 2014-04-04 17:05:03 -0600 (Fri, 04 Apr 2014) $
$Rev: 3629 $

\brief Make the libkuon library.

RN Make System Specific Makefile

\author: Robin Knight   (robin.knight@roadnarrows.com)

##! \copyright
##!   \h_copy 2014-2017. RoadNarrows LLC.\n
##!   http://www.roadnarrows.com\n
##!   All Rights Reserved

\cond RNMAKE_DOXY
 */
endif

#
# @EulaBegin@
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
# @EulaEnd@
#
################################################################################

#------------------------------------------------------------------------------
# Required

# Package Root Directory
pkgroot		= ../..

#SUBDIRS =  ut

#------------------------------------------------------------------------------
# Libraries

# Distribution Static Libraries
DIST_STLIBS       = kuon

# Distribution Shared Libraries
DIST_SHLIBS       = kuon

# Source Files
kuon.SRC.CXX = \
	kuonDesc.cxx \
	kuonDescBase.cxx \
	kuonJoint.cxx \
	kuonProdBaseStd.cxx \
	kuonRobot.cxx \
	kuonSpec.cxx \
	kuonStatus.cxx \
	kuonUtils.cxx \
	kuonXmlCfg.cxx \

# Linked Libraries 
kuon.LIBS	= \
	RS160D \
	rnr_appkit \
	rnr_serial \
	rnr_tinyxml \
	rnr \
	stdc++

#------------------------------------------------------------------------------
# Optional Variables 
#

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"libkuon\""

#------------------------------------------------------------------------------
# Make Includes

# Default RoadNarrows make system base directory
ifndef rnmake
rnmake := $(realpath $(pkgroot)/../rnmake)
endif

# Include Rules Makefile
include $(rnmake)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
