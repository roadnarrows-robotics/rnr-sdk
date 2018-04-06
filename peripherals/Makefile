################################################################################
#
# ./Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The peripherals package top-level makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics Peripherals C++ Package

\pkgfile{Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com} 
\pkgauthor{Daniel Packard,daniel@roadnarrows.com} 
\pkgauthor{Rob Shiely,rob@roadnarrows.com} 
\pkgauthor{Jessica Trujillo,jessica@roadnarrows.com}
\pkgauthor{Maurice Woods III,maurice@roadnarrows.com}

\pkgcopyright{2012-2018,RoadNarrows LLC,http://www.roadnarrows.com}

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

RNMAKE_SUBDIRS = sw

#------------------------------------------------------------------------------
# Distribution Header List Tags

# List of tags to lists of header files slated to be distributed and installed.
#
RNMAKE_DIST_HDRS = peripherals

#
# Distribution Headers
#
# Format: <tag>.HDRS.H = [<subdirs>/]<hdr>.h ...
#
# Note: Any header file that requires distribution is expected to be in 
# 			$(topdir)/include. Otherwise, is it really a deliverable header?
#
peripherals.HDRS.H = \
	rnr/usbext.h \
	rnr/hid/HID.h \
	rnr/hid/HIDXbox360.h \
	rnr/imu/yost.h\
  rnr/mot/Mot.h\
	rnr/mot/MotRoboteq.h\
	rnr/mot/MotRoboteqSmall.h

#	econ/econ32.h \
	econ/econ50.h \
	econ/econ_error.h \
	econ/econ_fn_res_prototype.h \
	econ/econ_typedef.h \


#------------------------------------------------------------------------------
# Documentation

# Doxygen Configuration File
RNMAKE_DOXY_CONF_FILE	= $(pkgroot)/make/doxy.conf


#------------------------------------------------------------------------------
# Extras

EXTRA_TGT_ALL_POST = all-post

ETC_FILES	=	share/etc/*.xml \
						share/etc/ld.so.conf.d/*.conf \
						etc/udev/rules.d/*

# Post make all target
.PHONY: all-post
all-post: show-banner cp-etc-files

.PHONY: show-banner
show-banner:
	$(call printDirBanner,,all-post)
	@printf "Pre-install Configuration\n"

.PHONY: cp-etc-files
cp-etc-files:
	$(call printGoalDesc,$(@),Copying etc files to $(DISTDIR_ETC))
	$(call copySrcDirRel,$(ETC_FILES),$(DISTDIR_ETC))


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk

# include extras
include $(RNMAKE_ROOT)/Extras.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
