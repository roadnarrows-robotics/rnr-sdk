################################################################################
#
# version_h.mk
#
ifdef RNMAKE_DOXY
/*! 
\file 

\brief Auto-generate the version.h include file for the package.

\par Usage:
make RNMAKE_PKG_ROOT=\<dir\> version_h=\<file\> pkg_mk=\<file\> autogen

\pkgsynopsis
RN Make System

\pkgfile{version_h.mk}

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

_VERSION_H_MK = 1

include $(pkg_mk)

timestamp := $(shell date "+%Y.%m.%d %T")

# $(call genDefine,brief,macro,value)
# 	Generatate define.
define genDefine =
@echo '/*! $(1) */'"\n"'#define $(2) $(3)'"\n" >> $(version_h)
endef

.PHONY: autogen
autogen: 	echo-autogen \
					gen-top-comment \
					gen-doxy-comment \
					gen-begin-ifndef \
					gen-includes \
					gen-defines \
					gen-pkginfo \
					gen-end-ifndef \

.PHONY: echo-autogen
echo-autogen:
	@echo 'Auto-generating $(version_h)'

.PHONY: gen-top-comment
gen-top-comment:
	@echo "\
//\n\
// File: $(notdir $(version_h))\n\
//" > $(version_h)

.PHONY: gen-doxy-comment
gen-doxy-comment:
	@echo "\
/*!\n\
 * \\\\file\n\
 *\n\
 * \\\\brief Package version information.\n\
 *\n\
 * \\\\warning Auto-generated by Rules.mk on $(timestamp)\n\
 *\n\
 * \\\\par Copyright:\n\
 * All Rights Reserved\n\
 */\n" >> $(version_h)

.PHONY: gen-begin-ifndef
gen-begin-ifndef:
	@echo "\
#ifndef _VERSION_H\n\
#define _VERSION_H\n" >> $(version_h)

.PHONY: gen-includes
gen-includes:
	@echo "#include \"rnr/pkg.h\"\n" >> $(version_h)

.PHONY: gen-defines
gen-defines:
	$(call genDefine,package name,PKG_NAME,"$(RNMAKE_PKG)")
	$(call genDefine,package dotted version,PKG_VERSION,"$(RNMAKE_PKG_VERSION_DOTTED)")
	$(call genDefine,package build date,PKG_TIMESTAMP,"$(timestamp)")
	$(call genDefine,package extended creation date,PKG_DATE,"$(RNMAKE_PKG_VERSION_DATE)")
	$(call genDefine,package full name,PKG_FULL_NAME,"$(RNMAKE_PKG_FULL_NAME)")
	$(call genDefine,package author(s),PKG_AUTHORS,$(RNMAKE_PKG_AUTHORS))
	$(call genDefine,package owner(s),PKG_OWNERS,$(RNMAKE_PKG_OWNERS))
	$(call genDefine,package legal disclaimer,PKG_DISCLAIMER,$(RNMAKE_PKG_DISCLAIMER))

.PHONY: gen-pkginfo
gen-pkginfo:
	@echo "\
/*! The package */\n\
static const PkgInfo_T PkgInfo =\n\
{\n\
  PKG_NAME,\n\
  PKG_VERSION,\n\
  PKG_TIMESTAMP,\n\
  PKG_DATE,\n\
  PKG_FULL_NAME,\n\
  PKG_AUTHORS,\n\
  PKG_OWNERS,\n\
  PKG_DISCLAIMER\n\
};\n" >> $(version_h)

.PHONY: gen-end-ifndef
gen-end-ifndef:
	@echo "#endif // _VERSION_H\n" >> $(version_h)


ifdef RDK_ABC
	$(info rdk: here)
	@echo '//' > $@
	@echo '// File: $(notdir $@)' >> $@
	@echo '//' >> $@
	@echo '/*!' >> $@
	@echo	' * \\file' >> $@
	@echo	' *' >> $@
	@echo	' * \\brief Package version information.' >> $@
	@echo	' *' >> $@
	@echo ' * \\warning Auto-generated by Rules.mk on' `date`>> $@
	@echo	' *' >> $@
	@echo ' * \\par Copyright:' >> $@
	@echo ' * \\n All Rights Reserved' >> $@
	@echo ' */' >> $@
	@echo '#ifndef _VERSION_H' >> $@
	@echo '#define _VERSION_H' >> $@
	@echo '' >> $@
	@echo '#include "rnr/pkg.h"' >> $@
	@echo '' >> $@
	@echo '/*! \\brief package name */' >> $@
	@echo '#define PKG_NAME       "$(RNMAKE_PKG)"' >> $@
	@echo '' >> $@
	@echo '/*! \\brief package dotted version */' >> $@
	@echo '#define PKG_VERSION    "$(RNMAKE_PKG_VERSION_DOTTED)"' >> $@
	@echo '' >> $@
	@echo '/*! \\brief package build date */' >> $@
	@echo '#define PKG_TIMESTAMP  "$(timestamp)"' >> $@
	@echo '' >> $@
	@echo '/*! \\brief package extended creation date */' >> $@
	@echo '#define PKG_DATE       "$(RNMAKE_PKG_VERSION_DATE)"' >> $@
	@echo '' >> $@
	@echo '/*! \\brief package full name */' >> $@
	@echo '#define PKG_FULL_NAME  "$(RNMAKE_PKG_FULL_NAME)"' >> $@
	@echo '' >> $@
	@echo '/*! \\brief package author(s) */' >> $@
	@echo '#define PKG_AUTHORS    $(RNMAKE_PKG_AUTHORS)' >> $@
	@echo '' >> $@
	@echo '/*! \\brief package owner(s) */' >> $@
	@echo '#define PKG_OWNERS     $(RNMAKE_PKG_OWNERS)' >> $@
	@echo '' >> $@
	@echo '/*! \\brief package legal disclaimer */' >> $@
	@echo '#define PKG_DISCLAIMER $(RNMAKE_PKG_DISCLAIMER)' >> $@
	@echo '' >> $@
	@echo '/*! \\brief The package */'        >> $@
	@echo 'static const PkgInfo_T PkgInfo =' 	>> $@
	@echo '{'       													>> $@
	@echo '  PKG_NAME,' 											>> $@
	@echo '  PKG_VERSION,' 										>> $@
	@echo '  PKG_TIMESTAMP',									>> $@
	@echo '  PKG_DATE,' 											>> $@
	@echo '  PKG_FULL_NAME,' 									>> $@
	@echo '  PKG_AUTHORS,' 										>> $@
	@echo '  PKG_OWNERS,' 										>> $@
	@echo '  PKG_DISCLAIMER' 									>> $@
	@echo '};'       													>> $@
	@echo '' >> $@
	@echo '#endif // _VERSION_H' >> $@
endif

ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
