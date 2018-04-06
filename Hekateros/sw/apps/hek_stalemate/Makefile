################################################################################
#
# ./sw/apps/hek_stalemate/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The hek_stalemate application makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_hek Robotic Arm Package

\pkgfile{sw/apps/hek_stalemate/Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com} 
\pkgauthor{Robin Knight,robin.knight@roadnarrows.com} 

\pkgcopyright{2011-2018,RoadNarrows LLC,http://www.roadnarrows.com}

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

#RNMAKE_DIST_PGMS = hek_stalemate 

# Libraries to Link With
hek_stalemate.LIBS =	\
	opencv_core opencv_highgui opencv_imgproc opencv_calib3d \
	jpeg \
	Dynamixel bsclient_dyna \
	rnrwin-gtk rnr \
	stdc++ \
	pthread \
	gtk-x11-2.0 gdk-x11-2.0 atk-1.0 gio-2.0 \
	pangoft2-1.0 gdk_pixbuf-2.0 m pangocairo-1.0 \
	cairo pango-1.0 \
	freetype fontconfig \
	gobject-2.0 gmodule-2.0 gthread-2.0 rt glib-2.0

hek_stalemate.LIBDEPS  = 

# Source Files
hek_stalemate.SRC.CXX = \
	StaleMate.cxx \
	SMBotPlanner.cxx \
	SMCalib.cxx \
	SMChess.cxx \
	SMGame.cxx \
	SMGui.cxx \
	SMHek.cxx \
	SMLiveFeed.cxx \
	SMMasterControl.cxx \
	SMOrp.cxx \
	SMSession.cxx \
	SMTests.cxx \
	SMTune.cxx \
	SMUci.cxx \
	SMUtil.cxx \
	SMVision.cxx


#------------------------------------------------------------------------------
# Extras

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"hek_stalemate\""

EXTRA_INCDIRS  = /usr/include/gtk-2.0 \
								 /usr/lib/gtk-2.0/include \
								 /usr/include/pango-1.0 \
								 /usr/include/cairo \
								 /usr/include/glib-2.0 \
								 /usr/lib/glib-2.0/include \
								 /usr/include/atk-1.0


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
