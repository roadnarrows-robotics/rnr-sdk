################################################################################
#
# ./sw/librnr_cam/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The librnr_cam library makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics Application Kit

\pkgfile{sw/librnr_cam/Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com} 

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
RNMAKE_PKG_ROOT	= ../..

# Bootstrap package within RN Make System
include $(RNMAKE_PKG_ROOT)/make/Bootstrap.mk


#------------------------------------------------------------------------------
# Subdirectories

RNMAKE_SUBDIRS =  ut


#------------------------------------------------------------------------------
# Libraries

# Distribution Static Libraries
RNMAKE_DIST_STLIBS = rnr_cam

# Distribution Shared Libraries
RNMAKE_DIST_SHLIBS = rnr_cam

# Target library subdirectory
rnr_cam.SUBDIR = rnr

# Source Files
rnr_cam.SRC.CXX = \
	Camera.cxx \
	CameraCv.cxx \
	CameraEcon32.cxx \
	CameraEcon50.cxx \
	CameraGst.cxx

rnr_cam.LIBS  = \
	tinyxml rnr_appkit rnr_win rnr_wingtk rnr_winqt rnr \
	$(OPENCV_COM_LIBS) \
	$(GTK_LIB) $(GDK_LIB) $(GDK_PIXBUF_LIB) $(ATK_LIB) $(PANGO_LIBS) \
	$(CAIRO_LIB) $(FONT_LIBS) $(GOBJ_LIBS) \
	$(GST_LIB_IF) $(GLIB_LIB)

rnr_cam.LIBDEPS  = rnr_tinyxml rnr_appkit rnr_win rnr_wingtk rnr_winqt

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
	$(GST_LIB_INCDIR) \
	$(LIBXML2_INCDIR) \
	$(GLIB_INCDIR) \
	$(GLIB_LIB_INCDIR)

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"librnr_cam\""


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
