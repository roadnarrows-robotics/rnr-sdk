################################################################################
#
# ./dxl_sdk/Makefile

ifdef RNMAKE_DOXY
/*!
\file

$LastChangedDate: 2014-09-26 11:43:33 -0600 (Fri, 26 Sep 2014) $
$Rev: 3757 $

\brief
Make Robotis dynamixel SDK library libdxl.

RN Make System Specific Makefile

\author: Robin Knight (robin.knight@roadnarrows.com)

\par Copyright:
  (C) 2011.  RoadNarrows LLC.
  (http://www.roadnarrows.com)
  All Rights Reserved

\cond RNMAKE_DOXY
 */
endif

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

#------------------------------------------------------------------------------
# Required

# Package Root Directory
pkgroot		= ..

#------------------------------------------------------------------------------
# Libraries

# Distribution Static Libraries
DIST_STLIBS 			= dxl

# Distribution Shared Libraries
DIST_SHLIBS 			= dxl

# Source Files
dxl.SRC.C 	=	dxl_hal.c dynamixel.c

# Linked Libraries

# put the source virtually here (cwd)
VPATH = src

EXTRA_TGT_ALL_POST	= cp-hdrs

#------------------------------------------------------------------------------
# Make Includes
#

# Default RoadNarrows make system base directory
ifndef rnmake
rnmake := $(realpath $(pkgroot)/../rnmake)
endif

# Include Rules Makefile
include $(rnmake)/Rules.mk

#------------------------------------------------------------------------------
# Extras

PKGDIR_DXL_INC	:= include
DISTDIR_DXL_INC	:= $(DISTDIR_INCLUDE)/Dynamixel/dxl
PKG_DXL_HDRS		:= $(wildcard $(PKGDIR_DXL_INC)/*.h)
DIST_DXL_HDRS 	:= $(patsubst $(PKGDIR_DXL_INC)%,$(DISTDIR_DXL_INC)%,\
										$(PKG_DXL_HDRS))

.PHONY: cp-hdrs
cp-hdrs: dxl-mkdir $(DIST_DXL_HDRS)

.PHONY: dxl-mkdir
dxl-mkdir:
		@test -d $(DISTDIR_DXL_INC) || $(MKDIR) $(DISTDIR_DXL_INC)

$(DISTDIR_DXL_INC)/%.h : $(PKGDIR_DXL_INC)/%.h
	@printf "$(color_tgt_file)    $(<)$(color_end)\n"
	@$(CP) $(<) $(@)

ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
