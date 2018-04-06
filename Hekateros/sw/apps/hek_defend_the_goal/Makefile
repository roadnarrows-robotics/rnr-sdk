################################################################################
#
# ./sw/apps/hek_defend_the_goal/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The hek_defend_the_goal application makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_hek Robotic Arm Package

\pkgfile{sw/apps/hek_defend_the_goal/Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com} 
\pkgauthor{Rob Shiely,rob@roadnarrows.com}
\pkgauthor{Daniel Packard,daniel@roadnarrows.com} 

\pkgcopyright{2012-2018,RoadNarrows LLC,http://www.roadnarrows.com}

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
RNMAKE_PKG_ROOT	= ../../..

# Bootstrap package within RN Make System
include $(RNMAKE_PKG_ROOT)/make/Bootstrap.mk


#------------------------------------------------------------------------------
# Programs

#RNMAKE_DIST_PGMS = hek_defend_the_goal 

# Libraries to Link With
hek_defend_the_goal.LIBS =	\
	opencv_core opencv_highgui opencv_imgproc opencv_calib3d \
	jpeg \
	Dynamixel \
	rnrwin-gtk rnr \
	stdc++ \
	pthread \
	gtk-x11-2.0 gdk-x11-2.0 atk-1.0 gio-2.0 \
	pangoft2-1.0 gdk_pixbuf-2.0 m pangocairo-1.0 \
	cairo pango-1.0 \
	freetype fontconfig \
	gobject-2.0 gmodule-2.0 gthread-2.0 rt glib-2.0

hek_defend_the_goal.LIBDEPS   = 

# C++ Source Files
hek_defend_the_goal.SRC.CXX = 


#------------------------------------------------------------------------------
# Extras

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"hek_defend_the_goal\""


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
