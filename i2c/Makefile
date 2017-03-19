################################################################################
#
# ./Makefile

ifdef RNMAKE_DOXY
/*!
\file

$LastChangedDate: 2016-01-28 14:19:12 -0700 (Thu, 28 Jan 2016) $
$Rev: 4278 $

\brief
Make RoadNarrows i2c package.

RN Make System Specific Makefile

\author: Robin Knight (robin.knight@roadnarrows.com)

##! \copyright
##!   \h_copy 2005-2017. RoadNarrows LLC.\n
##!   http://www.roadnarrows.com\n
##!   All Rights Reserved

\cond RNMAKE_DOXY
 */
endif

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
pkgroot		= .


#------------------------------------------------------------------------------
# Subdirectories

#
# Subdirectories to Build
#
SUBDIRS 					= libi2c cmds pyModules

#------------------------------------------------------------------------------
# Interface Headers

#
# Distribution Header List Tags
#
# List of tags to lists of header files slated to be distributed and installed.
#
DIST_HDRS	= i2c

#
# Distribution Headers
#
# Format: <tag>.HDRS.H = [<subdirs>/]<hdr>.h ...
#
# Note: Any header file that requires distribution is expected to be in 
# 			$(topdir)/include. Otherwise, is it really a deliverable header?
#
i2c.HDRS.H = \
	rnr/i2c-dev.h \
	rnr/i2c.h \
	rnr/smbus.h


#------------------------------------------------------------------------------
# Optional Variables 
#

#
# Document SubPackage Name
# 
DOC_SUBDIR				=

# Sub[Package] Doxygen Configuration File
DOXY_CONF_FILE		= $(pkgroot)/make/doxy.conf


#------------------------------------------------------------------------------
# Make Includes
#

# Default RoadNarrows make system base directory
ifndef rnmake
rnmake := $(realpath $(pkgroot)/../rnmake)
endif

# Include Rules Makefile
include $(rnmake)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif

