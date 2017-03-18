################################################################################
#
# ./sw/rospkgs/Makefile

ifdef RNMAKE_DOXY
/*!
\file

$LastChangedDate: 2013-02-19 08:24:43 -0700 (Tue, 19 Feb 2013) $
$Rev: 2692 $

\brief
Make the appkit rospkg modules.

RN Make System Specific Makefile

\author: Robin Knight <robin.knight@roadnarrows.com>

##! \par Copyright
##!   \h_copy 2013-2017. RoadNarrows LLC.\n
##!   http://www.roadnarrows.com\n
##!   All Rights Reserved

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
# IN NO EVENT SHALL THE AUTHOR, RoadNarrows LLC, OR ANY MEMBERS/EMPLOYEES
# OF RoadNarrows LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
# PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
# DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
# EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
# THE POSSIBILITY OF SUCH DAMAGE.
#
# THE AUTHOR AND RoadNarrows LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
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
DIST_STLIBS       = 

# Distribution Shared Libraries
DIST_SHLIBS       = 


# Linked Libraries 

#------------------------------------------------------------------------------
# Optional Variables 
#

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"appkit-rospkg\""

#EXTRA_TGT_DEPS 			= extra-deps
#EXTRA_TGT_CLEAN 		= extra-clean

#------------------------------------------------------------------------------
# Make Includes

# Default RoadNarrows make system base directory
ifndef rnmake
rnmake := $(realpath $(pkgroot)/../rnmake)
endif

# Include Rules Makefile
include $(rnmake)/Rules.mk

#------------------------------------------------------------------------------
# Extra make targets (place after Rules.mk)

.PHONY: extra-deps
extra-deps:
	@test -h $(BSMSGS_C_FILE) || $(SYMLINK) $(BSMSGS_SRC_DIR)/$(BSMSGS_C_FILE) .

.PHONY: extra-clean
extra-clean: 
	@test ! -h $(BSMSGS_C_FILE) || $(UNLINK) $(BSMSGS_C_FILE)

ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
