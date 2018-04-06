################################################################################
#
# ./bsProxy/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The \h_botsense proxy server bsProxy makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_botsense Client-Server Proxied Services Framework

\pkgfile{bsProxy/Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com} 

\pkgcopyright{2007-2018,RoadNarrows LLC,http://www.roadnarrows.com}

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

RNMAKE_DIST_PGMS = bsProxy

# Libraries to Link With
bsProxy.LIBS    = botsense rnr_netmsgs rnr_serial rnr_i2c rnr pthread $(DLLIB)
bsProxy.LIBDEPS = botsense

# C Source Files
bsProxy.SRC.C = \
	bsProxy.c \
	bsProxyClient.c \
	bsProxyMod.c \
	bsProxyRequest.c \
	bsProxyThread.c \
	bsProxyUtils.c \
	bsProxyVConn.c


#------------------------------------------------------------------------------
# Extras

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"BSPROXY\"" \
								 -DBS_PLUGIN_DIR="\"$(PKG_LIB_PLUGIN_SUBDIR)\""

# bsProxy supports dynamically link libraries
EXTRA_CFLAGS	+= $(DLLIB_APP_CFLAGS)


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
