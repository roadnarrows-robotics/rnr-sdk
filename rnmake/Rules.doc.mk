################################################################################
#
# Rules.doc.mk
#
ifdef RNMAKE_DOXY
/*! 
\file 

\brief Make package documentation.

This file is automatically included by \ref Rules.mk when one or more of the
documentation make goals are specified.

\pkgsynopsis
RN Make System

\pkgfile{Rules.doc.mk}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com}

\pkgcopyright{2005-2018,RoadNarrows LLC,http://www.roadnarrows.com}

\license{MIT}

\EulaBegin
\EulaEnd

\cond RNMAKE_DOXY
 */
endif
#
################################################################################

_RULES_DOC_MK = 1

# -------------------------------------------------------------------------
# Target: documents
# Desc:   Make documentation.
.PHONY: documents
documents: pkgbanner echo-documents docs-clean docs-src-gen docs-pub-gen \
						$(EXTRA_TGT_DOC)
	$(footer)

.PHONY: echo-documents
echo-documents:
	$(call printEchoTgtGoalDesc,Making documentation)

DOXY_VER=1.8.11

# documentation generator from source files
ifndef HTML_HEADER
HTML_HEADER     = $(RNMAKE_ROOT)/doxy/$(DOXY_VER)/rn_doxy_header.html
endif

ifndef HTML_FOOTER
HTML_FOOTER     = $(RNMAKE_ROOT)/doxy/$(DOXY_VER)/rn_doxy_footer.html
endif

ifndef HTML_STYLESHEET
HTML_STYLESHEET = $(RNMAKE_ROOT)/doxy/$(DOXY_VER)/rn_doxy.css
endif

ifndef DOXY_IMAGES
DOXY_IMAGES = $(RNMAKE_ROOT)/doxy/rn_images
endif

ifndef RN_DOXY_CONF_FILE
RN_DOXY_CONF_PATH = $(RNMAKE_ROOT)/doxy
RN_DOXY_CONF_FILE = $(RN_DOXY_CONF_PATH)/rn_doxy.conf
endif

docs-clean:
	$(printTgtGoal)
	$(RM) $(DISTDIR_DOC)

# generate doxygen source documetiona
docs-src-gen:
	$(printTgtGoal)
	@if [ "$(RNMAKE_DOXY_CONF_FILE)" ]; \
	then \
		echo "Making doxygen source documentation"; \
		test -d $(DISTDIR_DOC) || $(MKDIR) $(DISTDIR_DOC); \
		test -d $(DISTDIR_DOC_SRC_IMG) || $(MKDIR) $(DISTDIR_DOC_SRC_IMG); \
		$(CP) -r $(DOXY_IMAGES)/* $(DISTDIR_DOC_SRC_IMG)/.; \
		$(CP) -r $(HTML_STYLESHEET) $(DISTDIR_DOC_SRC)/.; \
		(cat $(RNMAKE_DOXY_CONF_FILE); \
		 echo "PROJECT_NUMBER=$(RNMAKE_PKG_VERSION_DOTTED)"; \
		 echo "HTML_HEADER=$(HTML_HEADER)"; \
		 echo "HTML_FOOTER=$(HTML_FOOTER)"; \
		 echo "OUTPUT_DIRECTORY=$(DISTDIR_DOC)"; \
		 echo "HTML_OUTPUT=$(DIST_SRCDOC)"; \
		 echo "PROJECT_LOGO=$(DISTDIR_DOC_SRC_IMG)/RNLogo.png"; \
		 echo "@INCLUDE_PATH=$(RN_DOXY_CONF_PATH)"; \
		 echo "@INCLUDE=$(RN_DOXY_CONF_FILE)"; \
		) | doxygen - >$(RNMAKE_PKG_ROOT)/doxy.out.log 2>$(RNMAKE_PKG_ROOT)/doxy.err.log; \
	fi
	$(footer)

# This utility scrip is no longer used after doxygen 1.7. DEPRECATED
#		$(RNMAKE_ROOT)/utils/doxyindex.sh \
#							-t "$(RNMAKE_PKG) v$(RNMAKE_PKG_VERSION_DOTTED)" \
#							-h $(HTML_HEADER) \
#							>$(DISTDIR_DOC)/$(DIST_SRCDOC)/index.html; \

# generate published documentation
docs-pub-gen:
	$(printTgtGoal)
	@pubdstdir="$(DISTDIR_DOC)/papers"; \
	pubsrcdir="$(RNMAKE_PKG_ROOT)/docs/published"; \
	unset flist; \
	test -d $${pubsrcdir} && flist=$$(ls $${pubsrcdir}); \
	if [ "$${flist}" != "" ]; \
	then \
		test -d $${pubdstdir} || $(MKDIR) $${pubdstdir}; \
		$(CP) -r $${pubsrcdir}/* $${pubdstdir}/. 2>/dev/null; \
	fi; \
	pubsrcdir="$(RNMAKE_PKG_ROOT)/3rdparty/published"; \
	unset flist; \
	test -d $${pubsrcdir} && flist=$$(ls $${pubsrcdir}); \
	if [ "$${flist}" != "" ]; \
	then \
		test -d $${pubdstdir} || $(MKDIR) $${pubdstdir}; \
		$(CP) -r $${pubsrcdir}/* $${pubdstdir}/. 2>/dev/null; \
	fi
	$(footer)

ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
