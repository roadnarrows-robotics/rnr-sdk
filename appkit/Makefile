################################################################################
#
# ./Makefile

##! \file
##! 
##! $LastChangedDate: 2015-11-09 11:56:44 -0700 (Mon, 09 Nov 2015) $
##! $Rev: 4194 $
##! 
##! \brief Make the RoadNarrows Applicaiton Tool Kit package.
##! 
##! RN Make System Specific Makefile
##! 
##! \author: Robin Knight (robin.knight@roadnarrows.com)
##! 
##! \par Copyright
##!   \h_copy 2013-2017. RoadNarrows LLC.\n
##!   http://www.roadnarrows.com\n
##!   All Rights Reserved
##!
##! \cond RNMAKE_DOXY

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
pkgroot		= .

#------------------------------------------------------------------------------

# Subdirectories to Build
SUBDIRS =  sw examples

#------------------------------------------------------------------------------
# Interface Headers

#
# Distribution Header List Tags
#
# List of tags to lists of header files slated to be distributed and installed.
#
DIST_HDRS = tinyxml librnr_appkit # librnr_cam librnr_win librnr_wingtk

#
# Distribution Headers
#
# Format: <tag>.HDRS.H = [<subdirs>/]<hdr>.h ...
#
# Note: Any header file that requires distribution is expected to be in 
#       $(topdir)/include. Otherwise, is it really a deliverable header?
#
tinyxml.HDRS.H = \
	rnr/tinyxml/tinystr.h \
	rnr/tinyxml/tinyxml.h

librnr_appkit.HDRS.H = \
	rnr/appkit/CmdAddOns.h \
	rnr/appkit/CmdArgDef.h \
	rnr/appkit/CmdCore.h \
	rnr/appkit/CmdDef.h \
	rnr/appkit/CmdExtArg.h \
	rnr/appkit/CmdFormDef.h \
	rnr/appkit/CommandLine.h \
	rnr/appkit/IOManip.h \
	rnr/appkit/LogBook.h \
	rnr/appkit/LogStream.h \
	rnr/appkit/Random.h \
	rnr/appkit/ReadLine.h \
	rnr/appkit/RegEx.h \
	rnr/appkit/Session.h \
	rnr/appkit/State.h \
	rnr/appkit/StateKb.h \
	rnr/appkit/StateMach.h \
	rnr/appkit/StringTheory.h \
	rnr/appkit/Thread.h \
	rnr/appkit/Time.h \
	rnr/appkit/Token.h \
	rnr/appkit/Xml.h

librnr_cam.HDRS.H = \
	rnr/appkit/Camera.h \
	rnr/appkit/CameraCv.h \
	rnr/appkit/CameraEcon32.h \
	rnr/appkit/CameraEcon50.h \
	rnr/appkit/CameraGst.h

librnr_win.HDRS.H = \
	rnr/appkit/SessionWin.h \
	rnr/appkit/StateWin.h \
	rnr/appkit/StateWinCamera.h \
	rnr/appkit/Win.h \
	rnr/appkit/WinCvIoI.h \
	rnr/appkit/WinCvMouse.h \
	rnr/appkit/WinLookFeel.h \
	rnr/appkit/WinMenu.h \
	rnr/appkit/WinOpenCv.h

librnr_wingtk.HDRS.H = \
	rnr/appkit/WinGtkMenu.h \
	rnr/appkit/WinGtk.h


#------------------------------------------------------------------------------
# Documentation 

# Document SubPackage Name
DOC_SUBDIR        =

# Sub[Package] Doxygen Configuration File
DOXY_CONF_FILE    = $(pkgroot)/make/doxy.conf

#------------------------------------------------------------------------------
# Optional Variables 

EXTRA_TGT_ALL_POST = all-post 

EXTRA_TGT_DOC = 

#------------------------------------------------------------------------------
# Make Includes

# Default RoadNarrows make system base directory
ifndef rnmake
rnmake := $(realpath $(pkgroot)/../rnmake)
endif

# Include Rules Makefile
include $(rnmake)/Rules.mk

#------------------------------------------------------------------------------
# Extras

PKGDIR_SHARE					:= $(pkgroot)/share
PKGDIR_SHARE_IMAGES		:= $(pkgroot)/share/images
PKGDIR_SHARE_ICONS		:= $(pkgroot)/share/images/icons

DISTDIR_SHARE_IMAGES	:= $(DISTDIR_SHARE)/images
DISTDIR_SHARE_ICONS		:= $(DISTDIR_SHARE)/images/icons

PKG_SHARE_FILES 		:= $(wildcard $(PKGDIR_SHARE)/*.xml) \
											 $(wildcard $(PKGDIR_SHARE_IMAGES)/*.png) \
											 $(wildcard $(PKGDIR_SHARE_IMAGES)/*.jpg) \
											 $(wildcard $(PKGDIR_SHARE_ICONS)/*.png) \
											 $(wildcard $(PKGDIR_SHARE_ICONS)/*.jpg)
DIST_SHARE_FILES		:= $(patsubst $(PKGDIR_SHARE)%,$(DISTDIR_SHARE)%,\
											 $(PKG_SHARE_FILES))

all-post: share-mkdirs $(DIST_SHARE_FILES) 

.PHONY: share-mkdirs
share-mkdirs:
	@test -d $(DISTDIR_SHARE_IMAGES) || $(MKDIR) $(DISTDIR_SHARE_IMAGES)
	@test -d $(DISTDIR_SHARE_ICONS) || $(MKDIR) $(DISTDIR_SHARE_ICONS)

# images png rule 
$(DISTDIR_SHARE_IMAGES)/%.png : $(PKGDIR_SHARE_IMAGES)/%.png
	@printf "\n"
	@@printf "$(color_tgt_file)    $(<)$(color_end)\n"
	@$(CP) $(<) $(@)

# images jpg rule 
$(DISTDIR_SHARE_IMAGES)/%.jpg : $(PKGDIR_SHARE_IMAGES)/%.jpg
	@printf "\n"
	@@printf "$(color_tgt_file)    $(<)$(color_end)\n"
	@$(CP) $(<) $(@)

# images png rule 
$(DISTDIR_SHARE_ICONS)/%.png : $(PKGDIR_SHARE_ICONS)/%.png
	@printf "\n"
	@@printf "$(color_tgt_file)    $(<)$(color_end)\n"
	@$(CP) $(<) $(@)

# images jpg rule 
$(DISTDIR_SHARE_ICONS)/%.jpg : $(PKGDIR_SHARE_ICONS)/%.jpg
	@printf "\n"
	@@printf "$(color_tgt_file)    $(<)$(color_end)\n"
	@$(CP) $(<) $(@)


##! \endcond 
