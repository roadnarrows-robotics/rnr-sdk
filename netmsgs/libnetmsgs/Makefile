################################################################################
#
# ./libnetmsgs/Makefile
#

ifdef RNMAKE_DOXY
/*!
\file

$LastChangedDate: 2011-11-18 13:32:30 -0700 (Fri, 18 Nov 2011) $
$Rev: 1578 $

\brief
Make libnetmsgs library.

RN Make System Specific Makefile

\author: Robin Knight (robin.knight@roadnarrows.com)

\par Copyright:
	(C) 2009-2011.  RoadNarrows LLC.
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
pkgroot		= ..


#------------------------------------------------------------------------------
# Libraries

# Distribution Static Libraries
DIST_STLIBS 			= netmsgs

# Distribution Shared Libraries
DIST_SHLIBS 			= netmsgs

# Source Files
netmsgs.SRC.C 	=	nmLibInternal.c \
									nmLibPack.c \
 									nmLibPackFlat.c \
									nmLibPackITV.c \
									nmLibUtils.c

# Linked libraries
netmsgs.LIBS	= rnr

#------------------------------------------------------------------------------
# Interface Headers

#------------------------------------------------------------------------------
# Optional Variables 
#

# Subpackage C PreProcessor Flags
# LOG									compile in logging
# LOGMOD							logging module's name (id)
# NMLIB_DEBUG_ENABLE	compile in library debuging
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"libnetmsgs\"" -DNMLIB_DEBUG_ENABLE

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
