################################################################################
#
# ./libserial/Makefile

ifdef RNMAKE_DOXY
/*!
\file

$LastChangedDate: 2011-01-16 10:19:23 -0700 (Sun, 16 Jan 2011) $
$Rev: 674 $

\brief
Make libserial C libraries.

RN Make System Specific Makefile

\author: Robin Knight (robin.knight@roadnarrows.com)

\par Copyright:
  (C) 2008-2010.  RoadNarrows LLC.
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
pkgroot   = .


#------------------------------------------------------------------------------
# Libraries

# Distribution Static Libraries
DIST_STLIBS 			= serial

# Distribution Shared Libraries
DIST_SHLIBS 			= serial

# Source Files
serial.SRC.C 			= serdev.c

# Libraries to link with
serial.LIBS	= rnr


#------------------------------------------------------------------------------
# Interface Headers

#
# Distribution Header List Tags
#
# List of tags to lists of header files slated to be distributed and installed.
#
DIST_HDRS	= rnr

#
# Distribution Headers
#
# Format: <tag>.HDRS.H = [<subdirs>/]<hdr>.h ...
#
# Note: Any header file that requires distribution is expected to be in 
# 			$(topdir)/include. Otherwise, is it really a deliverable header?
#
rnr.HDRS.H 		=   rnr/serdev.h

#------------------------------------------------------------------------------
# Optional Variables 
#

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD=\"libserial\"

# Document SubPackage Name
DOC_SUBDIR        =

# Sub[Package] Doxygen Configuration File
DOXY_CONF_FILE    = $(pkgroot)/make/doxy.conf


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
