################################################################################
#
# ./sw/librnr_cam/ut/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The librnr_cam library unit tests makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics Application Kit

\pkgfile{sw/librnr_cam/ut/Makefile}

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

RNMAKE_TEST_PGMS = ut-librnr_cam

# Linked Libraries 
ut-librnr_cam.LIBS = \
	gtest rnr_appkit rnr_cam rnr_win rnr_wingtk rnr \
  $(OPENCV_COM_LIBS) \
	$(GTK_LIB) \
	pthread stdc++

ut-librnr_cam.LIBDEPS	= gtest rnr_appkit rnr_cam rnr_win rnrwin_gtk

# C++ Source Files
ut-librnr_cam.SRC.CXX = \
	ut-librnr_cam-main.cxx \
	ut-Camera.cxx

#------------------------------------------------------------------------------
# Extras

# Includes
EXTRA_SYS_INCDIRS	+= \
	$(GTK_INCDIR) \
	$(GTK_LIB_INCDIR) \
	$(GDK_PIXBUF_INCDIR) \
	$(PANGO_INCDIR) \
	$(CAIRO_INCDIR) \
	$(ATK_INCDIR) \
	$(GST_INCDIR) \
	$(LIBXML2_INCDIR) \
	$(GLIB_INCDIR) \
	$(GLIB_LIB_INCDIR)

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"UTlibrnr_cam\""

EXTRA_INCDIRS += /opt/include


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
