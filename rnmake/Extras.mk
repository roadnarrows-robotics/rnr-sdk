################################################################################
#
# Extras.mk
#
ifdef RNMAKE_DOXY
/*! 
\file 

\brief Optional extras that the including package makefiles may include.

Include this file in the local make file (usually near the bottom).

\pkgsynopsis
RN Make System

\pkgfile{Extras.mk}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com}

\pkgcopyright{2018,RoadNarrows LLC,http://www.roadnarrows.com}

\license{MIT}

\EulaBegin
\EulaEnd

\cond RNMAKE_DOXY
 */
endif
#
################################################################################

_EXTRAS_MK = 1

# $(call copySrcDst,src,dst)
# 	Template to a copy source file to destination file. The file is copied iff
# 	the destination does not exists or if the source is newer.
define copySrcDst
	@if [ "$(1)" != "" ]; \
	then \
		if [ ! -f "$(2)" -o "$(1)" -nt "$(2)" ]; \
		then \
			d=$$(dirname $(2)); \
			test -d $$d || $(MKDIR) $$d; \
			printf "  $(1)\n"; \
			$(CP) $(1) $(2); \
		fi; \
	fi
endef

# $(call copySrcDir,srcfile...,dir)
# 	Template to copy source files to the destination directory. Each source
# 	file is copied iff the destination file does not exists or if the source is
# 	newer. Subdirectories of the destination directory are created as needed.
#
# 	Examples:
# 		$(call copySrcDir,foo bar,egg) -->
# 				egg/foo egg/bar
# 		$(call copySrcDir,lode of/the/bride,mother) -->
# 				mother/lode mother/of/the/bride
define copySrcDir
	@if [ "$(1)" != "" ]; \
	then \
		flist="$(wildcard $(1))"; \
		for src in $${flist}; \
		do \
			dst=$(2)/$${src}; \
			if [ ! -f "$${dst}" -o "$${src}" -nt "$${dst}" ]; \
			then \
				d=$$(dirname $${dst}); \
				test -d $${d} || $(MKDIR) $${d}; \
				printf "  $${src}\n"; \
				$(CP) $${src} $${dst}; \
			fi; \
		done; \
	fi
endef

# $(call copySrcDirRel,srcfile...,dir[,prefix])
# 	Template to copy source files to the destination directory. Each source
# 	file is copied iff the destination file does not exists or if the source is
# 	newer. Subdirectories of the destination directory are created as needed.
# 	If the prefix paramter is specified, string up to prefix is stripped off
# 	the source files prior to copying. Otherwise the common directory part
# 	between the target and source are removed.
#
# 	Example 1:
# 		srcfiles = share/etc/rules.d/my.rules share/etc/ld.so.conf.d/my.conf
# 		etcdir   = /path/to/mypkg/dist/dist.arch/etc
# 		$(call copySrcDirRel,$(srcfiles),$(etcdir) -->
# 				/path/to/mypkg/dist/dist.arch/etc/rules.d/my.rules
# 				/path/to/mypkg/dist/dist.arch/etc/ld.so.conf.d/my.conf
# 	Example 2:
# 		srcfile = share/lib/cmake/mypkg-config.cmake
# 		libdir  = /path/to/mypkg/dist/dist.arch/lib
# 		$(call copySrcDirRel,$(srcfile),$(libdir)) -->
# 				/path/to/mypkg/dist/dist.arch/lib/cmake/mypkg-config.cmake
# 	Example 3:
# 		srcfiles = share/theroad share/food/cake
# 		sharedir = /path/to/mypkg/dist/dist.arch/share/mypkg-x.y.z
# 		$(call copySrcDirRel,$(srcfiles),$(sharedir),share) -->
# 				/path/to/mypkg/dist/dist.arch/share/mypkg-x.y.z/theroad
# 				/path/to/mypkg/dist/dist.arch/share/mypkg-x.y.z/food/cake
define copySrcDirRel
	@if [ "$(1)" != "" ]; \
	then \
		dstbase=$(notdir $(2)); \
		flist="$(wildcard $(addprefix $(RNMAKE_PKG_ROOT)/,$(1)))"; \
		for src in $${flist}; \
		do \
			if [ -z "$(3)" ]; \
			then \
				dst=$(2)/$${src##*$${dstbase}/}; \
			else \
				dst=$(2)/$${src##*$(3)/}; \
			fi; \
			if [ ! -f "$${dst}" -o "$${src}" -nt "$${dst}" ]; \
			then \
				dstdir=$$(dirname $${dst}); \
				test -d $${dstdir} || $(MKDIR) $${dstdir}; \
				printf "  $${src}\n"; \
				$(CP) $${src} $${dst}; \
			fi; \
		done; \
	fi
endef

# $(call copySrcDirRel,srcfile...,dir,prefix)
# 	Three argument version.
#
# 	Template to copy source files to the destination directory. Each source
# 	file is copied iff the destination file does not exists or if the source is
# 	newer. Subdirectories of the destination directory are created as needed.
#
#define copySrcDirRel
#	$(info rdk: here 3: '$(3)')
#	@if [ "$(1)" != "" ]; \
#	then \
#		flist="$(wildcard $(addprefix $(RNMAKE_PKG_ROOT)/,$(1)))"; \
#		for src in $${flist}; \
#		do \
#			dst=$(2)/$${src##*$(3)/}; \
#			if [ ! -f "$${dst}" -o "$${src}" -nt "$${dst}" ]; \
#			then \
#				dstdir=$$(dirname $${dst}); \
#				test -d $${dstdir} || $(MKDIR) $${dstdir}; \
#				printf "  $${src}\n"; \
#				$(CP) $${src} $${dst}; \
#			fi; \
#		done; \
#	fi
#endef


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
