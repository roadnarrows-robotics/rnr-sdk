################################################################################
#
# make/Pkg.mk
#
ifdef RNMAKE_DOXY
/*! 
\file 

\brief RoadNarrows Make System rnmake package master (meta) makefile.

\pkgsynopsis
(Meta) RN Make System

\pkgfile{make/Pkg.mk}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com}
\pkgauthor{Daniel Packard,daniel@roadnarrows.com}

\pkgcopyright{2005-2018,RoadNarrows LLC,http://www.roadnarrows.com}

\license{MIT}

\EulaBegin
\EulaEnd

\cond RNMAKE_DOXY
 */
endif
#
################################################################################

_PKG_MK = 1

ifndef RNMAKE_PKG_ROOT
$(error 'RNMAKE_PKG_ROOT' Not defined in including makefile)
endif

# The Package Definition
RNMAKE_PKG 								 = rnmake
RNMAKE_PKG_VERSION_MAJOR   = 3
RNMAKE_PKG_VERSION_MINOR   = 0
RNMAKE_PKG_VERSION_RELEASE = 0
RNMAKE_PKG_VERSION_DATE    = 2018
RNMAKE_PKG_AUTHORS			 	 = "Robin Knight, Daniel Packard"
RNMAKE_PKG_OWNERS					 = "RoadNarrows LLC"
RNMAKE_PKG_DISCLAIMER			 = \
"See the README and EULA files for any copyright and licensing information."

# Dotted full version number
RNMAKE_PKG_VERSION_DOTTED	= $(RNMAKE_PKG_VERSION_MAJOR).$(RNMAKE_PKG_VERSION_MINOR).$(RNMAKE_PKG_VERSION_RELEASE)

# Concatenated full version number
RNMAKE_PKG_VERSION_CAT = $(RNMAKE_PKG_VERSION_MAJOR)$(RNMAKE_PKG_VERSION_MINOR)$(RNMAKE_PKG_VERSION_RELEASE)

# Package full name
RNMAKE_PKG_FULL_NAME = $(RNMAKE_PKG)-$(RNMAKE_PKG_VERSION_DOTTED)

#------------------------------------------------------------------------------
# Optional Variables and Tweaks

# Package Include Directories
RNMAKE_PKG_INCDIRS = 

# Package System Include Directories
RNMAKE_PKG_SYS_INCDIRS =

# Link Library Extra Library Directories (exluding local library)
RNMAKE_PKG_LD_LIBDIRS = 

# Release Files (docs)
RNMAKE_PKG_REL_FILES = VERSION.txt README.md

# Doxy Parameters
HTML_HEADER     	= $(rnmake)/doxy/rn_doxy_header.html
HTML_FOOTER     	= $(rnmake)/doxy/rn_doxy_footer.html
HTML_STYLESHEET		= $(rnmake)/doxy/rn_doxy.css
DOXY_IMAGES 			= $(rnmake)/doxy/rn_images
