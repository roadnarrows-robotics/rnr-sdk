################################################################################
#
# ./Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief Top-level \h_librnr makefile.

An rnmake system package specific makefile.

\pkgsynopsis
RoadNarrows Robotics Common Library 1

\pkgfile{Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com}

\pkgcopyright{2005-2018,RoadNarrows LLC,http://www.roadnarrows.com}

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

RNMAKE_SUBDIRS	= examples

#------------------------------------------------------------------------------
# librnr

# Distribution Static Libraries
RNMAKE_DIST_STLIBS = rnr

# Distribution Shared Libraries
RNMAKE_DIST_SHLIBS = rnr

# Source Files
rnr.SRC.C =	assoc.c \
						char.c \
						checksum.c \
						color.c \
						config.c \
						dlistvoid.c \
						hash.c \
						log.c \
						new.c \
						opts.c \
						path.c \
						shm.c \
						simplebuf.c\
						sock.c \
						sockset.c \
						units.c \
						uri.c \
						win.c

# library target subdirectory
rnr.SUBDIR = rnr

# library dependencies
rnr.LIBS = pthread rt


#------------------------------------------------------------------------------
# Distribution Headers

#
# Distribution Header List Tags
#
# List of tags to lists of header files slated to be distributed and installed.
#
RNMAKE_DIST_HDRS	= rnr

#
# Distribution Headers
#
# Format: <tag>.HDRS.H = [<subdirs>/]<hdr>.h ...
#
rnr.HDRS.H =	rnr/assoc.h \
							rnr/char.h \
							rnr/checksum.h \
							rnr/color.h \
							rnr/config.h \
							rnr/dlist.h \
							rnr/dliststr.h \
							rnr/dlistvoid.h \
							rnr/hash.h \
							rnr/install.h \
							rnr/log.h \
							rnr/new.h \
							rnr/opts.h \
							rnr/path.h \
							rnr/pkg.h \
							rnr/rnrconfig.h \
							rnr/shm.h \
							rnr/simplebuf.h \
							rnr/sock.h \
							rnr/sockset.h \
							rnr/units.h \
							rnr/uri.h

#------------------------------------------------------------------------------
# Documentation

# Doxygen Configuration File
RNMAKE_DOXY_CONF_FILE		= $(RNMAKE_PKG_ROOT)/make/doxy.conf

#------------------------------------------------------------------------------
# Extras

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"librnr\""

# Uncomment to discover specific compiler warnings switches 
#EXTRA_CFLAGS = -fdiagnostics-show-option

EXTRA_TGT_ALL_POST = all-post 

ETC_FILES	=	share/etc/*.xml \
						share/etc/ld.so.conf.d/*.conf \
						etc/udev/rules.d/*

LIB_FILES = share/lib/cmake/rnr/rnr-sdk-config.cmake

PKGDIR_SHARE := $(RNMAKE_PKG_ROOT)/share

# post 'all' make target
.PHONY: all-post
all-post: show-banner cp-etc-files cp-lib-files cp-share-files

.PHONY: show-banner
show-banner:
	$(call printDirBanner,,all-post)
	@printf "Pre-install Configuration\n"

.PHONY: cp-etc-files
cp-etc-files:
	$(call printGoalDesc,$(@),Copying etc files to $(DISTDIR_ETC))
	$(call copySrcDirRel,$(ETC_FILES),$(DISTDIR_ETC))

.PHONY: cp-lib-files
cp-lib-files:
	$(call printGoalDesc,$(@),Copying extra lib files to $(DISTDIR_LIB))
	$(call copySrcDirRel,$(LIB_FILES),$(DISTDIR_LIB))

.PHONY: cp-share-files
cp-share-files:
	$(call printGoalDesc,$(@),Copying share directory to $(DISTDIR_SHARE))
	@$(CP_R) $(PKGDIR_SHARE)/* $(DISTDIR_SHARE)
	@find $(DISTDIR_SHARE) -name '*.svn' -print | xargs rm -fr


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk

# include extras (must include after top-level)
include $(RNMAKE_ROOT)/Extras.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
