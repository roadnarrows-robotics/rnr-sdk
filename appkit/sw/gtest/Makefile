################################################################################
#
# ./sw/gtest/Makefile

ifdef RNMAKE_DOXY
/*!
\file

$LastChangedDate: 2013-02-18 12:42:09 -0700 (Mon, 18 Feb 2013) $
$Rev: 2691 $

\brief
Make gtest in RoadNarrows rnmake system.

RN Make System Specific Makefile

\author: @AUTHOR@ <@EMAIL@>

\par Copyright:
  (C) @DATE@ @OWNER@
  (@WEBSITE@)
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
# IN NO EVENT SHALL THE AUTHOR, @OWNER@, OR ANY MEMBERS/EMPLOYEES
# OF @OWNER@ OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
# PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
# DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
# EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
# THE POSSIBILITY OF SUCH DAMAGE.
#
# THE AUTHOR AND @OWNER@ SPECIFICALLY DISCLAIM ANY WARRANTIES,
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
# Libraries

# Distribution Static Libraries
#DIST_STLIBS    = gtest
LOC_STLIBS    = gtest

# Source Files
gtest.SRC.CXX  = gtest-all.cxx

# Linked Libraries 
gtest.LIBS	= gtest pthread stdc++

#------------------------------------------------------------------------------
# Test Programs

#------------------------------------------------------------------------------
# Optional Variables 
#

#------------------------------------------------------------------------------
# Make Includes
#

# Default RoadNarrows make system base directory
ifndef rnmake
rnmake := $(realpath $(pkgroot)/../rnmake)
endif

# Include Rules Makefile
include $(rnmake)/Rules.mk

ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif

