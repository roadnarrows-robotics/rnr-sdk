################################################################################
#
# ./sw/gst/Makefile

ifdef RNMAKE_DOXY
/*!
\file

$LastChangedDate: 2015-12-03 17:11:37 -0700 (Thu, 03 Dec 2015) $  
$Rev: 4233 $ 

\brief
Make the Eudoxus GStreamer plug-ins and application.

RN Make System Specific Makefile

\author: Robin Knight (robin.knight@roadnarrows.com)

\par Copyright:
  (C) 2012-2015.  RoadNarrows LLC.
  (http://www.roadnarrows.com)
  All Rights Reserved

\cond RNMAKE_DOXY
 */
endif

# @EulaBegin@
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
# @EulaEnd@
#
################################################################################

#------------------------------------------------------------------------------
# Required

# Package Root Directory
pkgroot		= ../..

#------------------------------------------------------------------------------
# Target Specific

ifndef arch
ifdef RNMAKE_ARCH_DFT
    arch=$(RNMAKE_ARCH_DFT)
else
    arch = x86_64
endif
endif

ifeq "$(arch)" "linaro"
	EUDOXUS_CLIENT_ONLY =
else
ifeq "$(arch)" "overo"
	EUDOXUS_CLIENT_ONLY =
else
	EUDOXUS_CLIENT_ONLY = pcsviewersink
endif # not overo
endif # not linaro

#------------------------------------------------------------------------------

# Subdirectories to Build
SUBDIRS = \
	onisrc \
	onipdu \
	pcsfilesrc \
	$(EUDOXUS_CLIENT_ONLY) \
	unittest
					
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
