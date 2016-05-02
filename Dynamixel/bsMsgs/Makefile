################################################################################
#
# ./bsMsgs/Makefile

ifdef RNMAKE_DOXY
/*!
\file

$LastChangedDate: 2015-01-12 10:56:06 -0700 (Mon, 12 Jan 2015) $
$Rev: 3845 $

\brief Make Dynamixel botsense messages.

RN Make System Specific Makefile

\author: Robin Knight (robin.knight@roadnarrows.com)

\par Copyright:
  (C) 2011-2016.  RoadNarrows LLC.
  (http://www.roadnarrows.com)
  All Rights Reserved

\cond RNMAKE_DOXY
 */
endif

# @EulaBegin@
# 
# Unless otherwise stated explicitly, all materials contained are copyrighted
# and may not be used without RoadNarrows LLC's written consent,
# except as provided in these terms and conditions or in the copyright
# notice (documents and software) or other proprietary notice provided with
# the relevant materials.
# 
# IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY 
# MEMBERS/EMPLOYEES/CONTRACTORS OF ROADNARROWS OR DISTRIBUTORS OF THIS SOFTWARE
# BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
# CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
# DOCUMENTATION, EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# 
# THE AUTHORS AND  ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
# INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
# FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
# "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
# PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
# 
# @EulaEnd@
#
################################################################################

#------------------------------------------------------------------------------
# Required

# Package Root Directory
pkgroot		= ..

#------------------------------------------------------------------------------
# Libraries

# botsense modules are located in a sub-directory of lib
LIB_SUBDIR = $(PKG_LIB_PLUGIN_SUBDIR)

#------------------------------------------------------------------------------
# Interface Headers

#------------------------------------------------------------------------------
# Optional Variables 
#

EXTRA_TGT_ALL 			= extra-make
EXTRA_TGT_ALL_POST	= extra-make-post
EXTRA_TGT_DEPS 			= extra-deps
EXTRA_TGT_CLEAN 		= extra-clean

#------------------------------------------------------------------------------
# Make Includes
#

# Default RoadNarrows make system base directory
ifndef rnmake
rnmake := $(realpath $(pkgroot)/../rnmake)
endif

# Include Rules Makefile
include $(rnmake)/Rules.mk

#------------------------------------------------------------------------------
# Extra make targets (place after Rules.mk)

# NetMsgs Variables
NETMSGS_XML_FILES	= bsDynaMsgs.xml
NETMSGS_H_DIR     = $(pkgroot)/include/botsense
NETMSGS_CFLAGS		= --incprefix=botsense
NETMSGS_PY_DIR		= $(pkgroot)/bsPython/modules/BotSense/Dynamixel
NETMSGS_SHARE_DIR	= $(pkgroot)/share/msgs

# Include NetMsgs Rules Makefile
include $(rnmake)/Rules.netmsgs.mk

SWIG_FILES 				= bsDynaCore.i
SWIG_EXTMOD_DIR 	= $(pkgroot)/bsPython/modules/BotSense/Dynamixel
SWIG_EXTMOD_LIBS 	= -lbotsense -lnetmsgs -li2c -lrnr

# Include Swig Rules Makefile
include $(rnmake)/Rules.swig.mk

.PHONY: extra-make
extra-make: netmsgs-all

.PHONY: extra-make-post
extra-make-post: swig-all

.PHONY: extra-deps
extra-deps: netmsgs-all
	@touch $(DEPSFILE)

.PHONY: extra-clean
extra-clean: netmsgs-clean swig-clean


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
