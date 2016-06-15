################################################################################
#
# ./fw/Diag/Makefile

ifdef RNMAKE_DOXY
/*!
\file

$LastChangedDate: 2011-09-12 16:10:26 -0600 (Mon, 12 Sep 2011) $
$Rev: 1282 $

\brief Make CogniBoost Diagnostic firmware.

RN Make System Specific Makefile

\author: Robin Knight (robin.knight@roadnarrows.com)
\author: Daniel Packard (daniel@roadnarrows.com) 
\author: Brent Wilkins  (brent@roadnarrows.com) 

\par Copyright:
  (C) 2011-2016.  RoadNarrows LLC.
  (http://www.roadnarrows.com)
  All Rights Reserved

\cond RNMAKE_DOXY
 */
endif

# @EulaBegin@
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
# @EulaEnd@
#
################################################################################

#------------------------------------------------------------------------------
# Required

# Package Root Directory
pkgroot		= ../..

# only one architecure supported
override arch := atmega32uc3

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
# Make Includes

# Default RoadNarrows make system base directory
ifndef rnmake
rnmake := $(realpath $(pkgroot)/../rnmake)
endif

# Include Rules Makefile
include $(rnmake)/Rules.AtmelMcu.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
