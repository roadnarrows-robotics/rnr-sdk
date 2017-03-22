################################################################################
#
# ./examples/Makefile
#

ifdef RNMAKE_DOXY
/*!
\file

$LastChangedDate: 2013-02-08 12:20:58 -0700 (Fri, 08 Feb 2013) $
$Rev: 2676 $

\brief
Make librnr example applications.

RN Make System Specific Makefile

\author: Robin Knight (robin.knight@roadnarrows.com)

 * \copyright
 *   \h_copy 2005-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved

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
# Programs

LOC_PGMS         = example_assoc \
									 example_checksum \
									 example_config \
									 example_dlist \
									 example_hash \
									 example_log \
                   example_cpp \
                   example_uri \
                   example_shm 

# Libraries to Link With
example_assoc.LIBS      = rnr
example_assoc.LIBDEPS   = rnr
example_checksum.LIBS   = rnr rt
example_checksum.LIBDEPS  = rnr
example_config.LIBS     = rnr
example_config.LIBDEPS  = rnr
example_dlist.LIBS      = rnr
example_dlist.LIBDEPS   = rnr
example_hash.LIBS      	= rnr
example_hash.LIBDEPS   	= rnr
example_log.LIBS      	= rnr
example_log.LIBDEPS   	= rnr
example_cpp.LIBS      	= rnr stdc++
example_cpp.LIBDEPS   	= rnr
example_uri.LIBS      	= rnr
example_uri.LIBDEPS   	= rnr
example_shm.LIBS      	= rnr pthread rt
example_shm.LIBDEPS   	= rnr

#------------------------------------------------------------------------------
# Sources

#
# C Source Files
#
example_assoc.SRC.C = example_assoc.c

example_checksum.SRC.C = example_checksum.c

example_config.SRC.C = example_config.c

example_dlist.SRC.C = example_dlist.c

example_hash.SRC.C = example_hash.c

example_log.SRC.C = example_log.c

example_cpp.SRC.CXX = example_cpp.cxx

example_uri.SRC.C = example_uri.c

example_shm.SRC.C = example_shm.c

#------------------------------------------------------------------------------
# Optional Variables 
#

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"logexample\"" -DLOGMOD_COLOR=LOG_COLOR_BLUE

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
