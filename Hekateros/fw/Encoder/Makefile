################################################################################
#
# ./fw/Encoder/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The \h_hek encoder firmare makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_hek Robotic Arm Package

\pkgfile{fw/Encoder/Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com} 

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
RNMAKE_PKG_ROOT	= ../..

# Bootstrap package within RN Make System
include $(RNMAKE_PKG_ROOT)/make/Bootstrap.mk

# only one architecure supported
override arch := atmega168

# define one, but not both
TGT_LIB		=
TGT_PGM 	= Encoder
 
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
$(TGT_PGM).SRC.C = Encoder.c TWI_slave.c

# Assembly Source Files
$(TGT_PGM).SRC.S = 


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.AtmelMcu.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
