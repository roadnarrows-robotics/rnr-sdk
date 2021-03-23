################################################################################
#
# ./pyModules/src/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The \h_i2c python module extension makefile.

An rnmake system package specific makefile.

\pkgsynopsis
RoadNarrows \h_i2c Package

\pkgfile{pyModules/src/Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com}

\pkgcopyright{2016-2018,RoadNarrows LLC,http://www.roadnarrows.com}

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
RNMAKE_PKG_ROOT	= ../..

# Bootstrap package within RN Make System
include $(RNMAKE_PKG_ROOT)/make/Bootstrap.mk

#------------------------------------------------------------------------------
# Extras
#

EXTRA_TGT_ALL 	= all-extmod
EXTRA_TGT_CLEAN	= clean-extmod

SWIG_FILES 				= i2ccore.i
SWIG_EXTMOD_DIR 	= ../modules/rnr
SWIG_EXTMOD_LIBS 	= -lrnr_i2c -lrnr

EXTRA_CFLAGS += $(SWIG_CFLAGS)
EXTRA_LDFLAGS += $(SWIG_LDFLAGS)

all-extmod: swig-all

# Clean extension module intermediates
clean-extmod: swig-clean


#------------------------------------------------------------------------------
# Include RNMAKE rules makefiles

# Include top-level rules
include $(RNMAKE_ROOT)/Rules.mk

# Include swig rules (must be after top-level rules)
include $(RNMAKE_ROOT)/Rules.swig.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
