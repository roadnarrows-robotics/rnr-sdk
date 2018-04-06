################################################################################
#
# ./examples/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief Make librnr example applications.

An rnmake system package specific makefile.

\pkgsynopsis
RoadNarrows Robotics Common Library 1

\pkgfile{examples/Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com}

\pkgcopyright{2005-2018,RoadNarrows LLC,http://www.roadnarrows.com}

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
RNMAKE_PKG_ROOT	= ..

# Bootstrap package within RN Make System
include $(RNMAKE_PKG_ROOT)/make/Bootstrap.mk


#------------------------------------------------------------------------------
# Programs

RNMAKE_LOC_PGMS =	example_assoc \
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
# Extras
#

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"logexample\"" -DLOGMOD_COLOR=LOG_COLOR_BLUE

#------------------------------------------------------------------------------
# Include rnmake top-level rules makefile

include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
