################################################################################
#
# ./Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The \h_appkit package top-level makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics Application Kit

\pkgfile{Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com} 

\pkgcopyright{2013-2018,RoadNarrows LLC,http://www.roadnarrows.com}

\license{MIT}

\EulaBegin
\EulaEnd

 * \cond RNMAKE_DOXY
 */
endif
# 
################################################################################

#------------------------------------------------------------------------------
# Required

# Package Root Directory
RNMAKE_PKG_ROOT	= .

# Bootstrap package within RN Make System
include $(RNMAKE_PKG_ROOT)/make/Bootstrap.mk


#------------------------------------------------------------------------------
# Subdirectories

RNMAKE_SUBDIRS = sw examples


#------------------------------------------------------------------------------
# Interface Headers

#
# Distribution Header List Tags
#
# List of tags to lists of header files slated to be distributed and installed.
#
RNMAKE_DIST_HDRS = tinyxml librnr_appkit # librnr_cam librnr_win librnr_wingtk

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

# Sub[Package] Doxygen Configuration File
RNMAKE_DOXY_CONF_FILE    = $(RNMAKE_PKG_ROOT)/make/doxy.conf


#------------------------------------------------------------------------------
# Extras

EXTRA_TGT_ALL_POST = all-post 

PKGDIR_SHARE					:= $(RNMAKE_PKG_ROOT)/share
PKGDIR_SHARE_IMAGES		:= $(RNMAKE_PKG_ROOT)/share/images
PKGDIR_SHARE_ICONS		:= $(RNMAKE_PKG_ROOT)/share/images/icons

DISTDIR_SHARE_IMAGES	:= $(DISTDIR_SHARE)/images
DISTDIR_SHARE_ICONS		:= $(DISTDIR_SHARE)/images/icons

PKG_SHARE_FILES 		:= $(wildcard $(PKGDIR_SHARE)/*.xml) \
											 $(wildcard $(PKGDIR_SHARE_IMAGES)/*.png) \
											 $(wildcard $(PKGDIR_SHARE_IMAGES)/*.jpg) \
											 $(wildcard $(PKGDIR_SHARE_ICONS)/*.png) \
											 $(wildcard $(PKGDIR_SHARE_ICONS)/*.jpg)
DIST_SHARE_FILES		:= $(patsubst $(PKGDIR_SHARE)%,$(DISTDIR_SHARE)%,\
											 $(PKG_SHARE_FILES))

SHARE_FILES = \
	share/*.xml \
	share/images/*.png \
	share/images/*.jpg \
	share/images/icons/*.png \
	share/images/icons/*.jpg \

.PHONY: all-post
all-post: show-banner cp-share-files

#all-post: show-banner share-mkdirs $(DIST_SHARE_FILES) 

.PHONY: show-banner
show-banner:
	$(call printDirBanner,,all-post)
	@printf "Pre-install Configuration\n"

.PHONY: cp-share-files
cp-share-files:
	$(call printGoalDesc,$(@),Copying share files to $(DISTDIR_SHARE))
	$(call copySrcDirRel,$(SHARE_FILES),$(DISTDIR_SHARE),share)



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


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk

# include extras
include $(RNMAKE_ROOT)/Extras.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
