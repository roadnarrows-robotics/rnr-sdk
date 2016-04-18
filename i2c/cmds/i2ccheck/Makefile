################################################################################
#
# ./cmds/i2ccheck/Makefile

ifdef RNMAKE_DOXY
/*!
\file

$LastChangedDate: 2010-03-25 08:03:07 -0600 (Thu, 25 Mar 2010) $
$Rev: 309 $

\brief
Make i2ccheck command to check if I2C device with the specified address exists
on the  I2C Bus.

RN Make System Specific Makefile

\author: Robin Knight (robin.knight@roadnarrows.com)

\par Copyright:
  (C) 2007-2010.  RoadNarrows LLC.
  (http://www.roadnarrows.com)
  All Rights Reserved

\cond RNMAKE_DOXY
 */
endif

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
################################################################################

#------------------------------------------------------------------------------
# Required

# Package Root Directory
pkgroot		= ../..


#------------------------------------------------------------------------------
# Programs

DIST_PGMS         = i2ccheck

# Libraries to Link With
i2ccheck.LIBS      = rnr i2c
i2ccheck.LIBDEPS   = i2c


#------------------------------------------------------------------------------
# Sources

#
# C Source Files
#
i2ccheck.SRC.C = i2ccheck.c

#------------------------------------------------------------------------------
# Optional Variables 
#

# Sub[Package] Extra Include Directories
EXTRA_INCDIRS			= $(topdir)/cmds/include

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"i2ccheck\""

#------------------------------------------------------------------------------
# Make Includes

# Default RoadNarrows make system base directory
ifndef rnmake
rnmake := $(realpath $(pkgroot)/../rnmake)
endif

# Include Rules Makefile
include $(rnmake)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
