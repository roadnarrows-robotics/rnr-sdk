################################################################################
#
# ./fw/Diag/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The \h_cogniboost diagnostics firmware makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_cogniboost Package

\pkgfile{fw/Diag/Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com} 
\pkgauthor{Daniel Packard,daniel@roadnarrows.com} 
\pkgauthor{Brent Wilkins,brent@roadnarrows.com} 

\pkgcopyright{2011-2018,RoadNarrows LLC,http://www.roadnarrows.com}

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
RNMAKE_PKG_ROOT	= ..

# Bootstrap package within RN Make System
include $(RNMAKE_PKG_ROOT)/make/Bootstrap.mk

# only one architecure supported
override RNMAKE_ARCH := atmega32uc3

# define one, but not both
TGT_LIB		=
TGT_PGM 	= Diag
 
# Output format. One of: srec, ihex(defualt), binary
# Override default ihex output format here.
#FORMAT = 

# Set High Fuse value.
# Disable JTAG
# Preserve EEPROM memory over chip erases (flash downloads)
FUSE_HIGH = 0xD1

#------------------------------------------------------------------------------
# Programs

# Libraries to Link With
$(TGT_PGM).LIBS  = 

# C Source Files
$(TGT_PGM).SRC.C = diag.c

# Assembly Source Files
$(TGT_PGM).SRC.S = 


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
