################################################################################
#
# ./bsPython/Makefile
#

ifdef RNMAKE_DOXY
/*!
\file

$LastChangedDate: 2011-06-22 08:41:22 -0600 (Wed, 22 Jun 2011) $
$Rev: 1061 $

\brief
Make BotSense Dynamixel python modules, extension modules, and scripts.

RN Make System Specific Makefile

\par Usage:
make [RNMAKE_OPTS] [has_botsense={yes|no}] target

\author: Robin Knight (robin.knight@roadnarrows.com)

\par Copyright:
	(C) 2011.  RoadNarrows LLC.
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
# Optional variables

# do [not] include botsense subpackage in package
ifeq "$(findstring $(has_botsense),n no 0 false)" ""
	has_botsense := yes
else
	has_botsense := no
endif

#------------------------------------------------------------------------------
# Subdirectories

# Subdirectories to Build
ifeq "$(has_botsense)" "yes"
SUBDIRS = src
endif

#------------------------------------------------------------------------------
# Optional Variables 
#

ifeq "$(has_botsense)" "yes"
EXTRA_TGT_ALL_POST 		= all-python-post
EXTRA_TGT_DOC 				= doc-python
EXTRA_TGT_CLEAN 			= clean-python
EXTRA_TGT_DISTCLEAN 	= distclean-python
endif

#------------------------------------------------------------------------------
# Make Includes

# Default RoadNarrows make system base directory
ifndef rnmake
rnmake := $(realpath $(pkgroot)/../rnmake)
endif

# Include Rules Makefile
include $(rnmake)/Rules.mk

#------------------------------------------------------------------------------
# Extras

# Include Python Rules Makefile
include $(rnmake)/Rules.python.mk

.PHONY: all-python-post
all-python-post: python-all

.PHONY: doc-python
doc-python: python-doc

.PHONY: clean-python
clean-python: python-clean

.PHONY: distclean-python
distclean-python: python-distclean


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
