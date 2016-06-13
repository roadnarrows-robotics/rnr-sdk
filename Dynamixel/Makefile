################################################################################
#
# ./Makefile

ifdef RNMAKE_DOXY
/*!
\file

$LastChangedDate: 2015-01-12 10:56:06 -0700 (Mon, 12 Jan 2015) $
$Rev: 3845 $

\brief Make Dynamixel package.

RN Make System Specific Makefile

\par Usage:
make [RNMAKE_OPTS] [has_botsense={yes|no}] target

\author: Robin Knight (robin.knight@roadnarrows.com)

\par Copyright:
  (C) 2011-2016.  RoadNarrows LLC.
  (http://www.roadnarrows.com)
  All Rights Reserved

\cond RNMAKE_DOXY
 */
endif

# @EulaBegin@
# 
# Unless otherwise stated explicitly, all materials contained are copyrighted
# and may not be used without RoadNarrows LLC's written consent,
# except as provided in these terms and conditions or in the copyright
# notice (documents and software) or other proprietary notice provided with
# the relevant materials.
# 
# IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY 
# MEMBERS/EMPLOYEES/CONTRACTORS OF ROADNARROWS OR DISTRIBUTORS OF THIS SOFTWARE
# BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
# CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
# DOCUMENTATION, EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# 
# THE AUTHORS AND  ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
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
# Optional variables

# do [not] include botsense subpackage in package
ifeq "$(findstring $(has_botsense),n no 0 false)" ""
	has_botsense := yes
else
	has_botsense := no
endif

#------------------------------------------------------------------------------
# Subdirectories

# Subdirectories to Build
SUBDIRS = \
	libdxl \
	bsMsgs \
	libbsclient_dyna \
	libDynamixel \
	libbsserver_dyna \
	dynashell \
	examples

#	dxl_sdk-2.0

#------------------------------------------------------------------------------
# Interface Headers

#
# Distribution Header List Tags
#
# List of tags to lists of header files slated to be distributed and installed.
#
DIST_DYN_HDRS	= Dynamixel
DIST_DXL_HDRS	= dxl
DIST_BS_HDRS  = botsense
DIST_HDRS 		= $(DIST_DYN_HDRS) $(DIST_DXL_HDRS) $(DIST_BS_HDRS)

#
# Distribution Headers
#
# Format: <tag>.HDRS.H = [<subdirs>/]<hdr>.h ...
#
# Note: Any header file that requires distribution is expected to be in 
# 			$(topdir)/include. Otherwise, is it really a deliverable header?
#
$(DIST_DYN_HDRS).HDRS.H	=	$(shell cd include; ls $(DIST_DYN_HDRS)/*.h)
$(DIST_DXL_HDRS).HDRS.H =	\
	$(shell cd include; ls $(DIST_DYN_HDRS)/$(DIST_DXL_HDRS)/*.h)
$(DIST_BS_HDRS).HDRS.H = botsense/bsDyna.h botsense/bsDynaMsgs.h


#------------------------------------------------------------------------------
# Documentation 

# Document SubPackage Name
DOC_SUBDIR				=

# Sub[Package] Doxygen Configuration File
DOXY_CONF_FILE		= $(pkgroot)/make/doxy.conf

#------------------------------------------------------------------------------
# Optional Variables 
#

# TODO
#EXTRA_TGT_DOC       		= doc-python
EXTRA_TGT_INSTALL   		= install-extra
EXTRA_TGT_INSTALL_POST  = install-extra-post
EXTRA_TGT_CLEAN     		= clean-python
EXTRA_TGT_DISTCLEAN 		= distclean-python

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

PKGDIR_SHARE				:= $(pkgroot)/share
PKGDIR_SHARE_MSGS		:= $(pkgroot)/share/msgs
PKGDIR_SHARE_ETC		:= $(pkgroot)/share/etc
PKGDIR_SHARE_RULES	:= $(PKGDIR_SHARE_ETC)/udev/rules.d

DISTDIR_SHARE_MSGS 	:= $(DISTDIR_SHARE)/msgs
DISTDIR_SHARE_ETC 	:= $(DISTDIR_SHARE)/etc
DISTDIR_SHARE_RULES := $(DISTDIR_SHARE_ETC)/udev/rules.d
 
PKG_SHARE_FILES 		:= $(wildcard $(PKGDIR_SHARE)/*.xml) \
											 $(wildcard $(PKGDIR_SHARE_MSGS)/*.xml) \
											 $(wildcard $(PKGDIR_SHARE_RULES)/*.rules)

DIST_SHARE_FILES		:= $(patsubst $(PKGDIR_SHARE)%,$(DISTDIR_SHARE)%,\
											 $(PKG_SHARE_FILES))

# Target: install-extra
# Install botsense configuration and message definition files
install-extra: share-mkdirs $(DIST_SHARE_FILES)

# Configuration xml rule 
$(DISTDIR_SHARE)/%.xml : $(pkgroot)/share/%.xml
	@echo ""
	@echo "    $(<)"
	$(CP) $(<) $(@)

#  Message xml definitions rule 
$(DISTDIR_SHARE_MSGS)/%.xml : $(pkgroot)/share/msgs/%.xml
	@echo ""
	@echo "    $(<)"
	$(CP) $(<) $(@)

# share etc 
$(DISTDIR_SHARE_RULES)/%.rules : $(PKGDIR_SHARE_RULES)/%.rules
	@printf "\n"
	@printf "$(color_tgt_file)    $(<)$(color_end)\n"
	@$(CP) $(<) $(@)


.PHONY: share-mkdirs
share-mkdirs:
	@test -d $(DISTDIR_SHARE_MSGS) || $(MKDIR) $(DISTDIR_SHARE_MSGS)
	@test -d $(DISTDIR_SHARE_RULES) || $(MKDIR) $(DISTDIR_SHARE_RULES)

#
# If this package is including making the BotSense Dynamixel plugins, then
# install the share files also to the botsense installed package. Otherwise do
# nothing.
.PHONY: install-extra-post
ifeq "$(has_botsense)" "yes"
install-extra-post:
	@echo ""
	@echo "Installing system botsense share files to $(sharedir)/botsense"
	@$(rnmake)/utils/doinstall.sh -s 664 $(DISTDIR_SHARE) $(sharedir)/botsense
else
install-extra-post:

endif


.PHONY: doc-python clean-python distclean-python
doc-python clean-python distclean-python:
	@$(MAKE) $(EXTRA_MAKE_FLAGS) -C $(pkgroot)/bsPython $(@)


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
