################################################################################
#
# Std.mk
#
ifdef RNMAKE_DOXY
/*! 
\file 

\brief A collection of make defined functions, canned sequences, variables, and
targets that any RN Make System makefile may use.

This makefile is mostly independent of RNMAKE variables and, hence, can usually
be included anywhere in the including makefile.

Exceptions are:
- RNMAKE_ROOT for colors
- printDirBanner requires RNMAKE_PKG_ROOT
- printPkgBanner requires RNMAKE_PKG_ROOT

\pkgsynopsis
RN Make System

\pkgfile{Std.mk}

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

_STD_MK = 1

# include colars
ifeq ($(_COLORS_MK),)
  $(eval -include $(RNMAKE_ROOT)/Colors.mk)
endif

#------------------------------------------------------------------------------
# Inter-makefile helper functions and canned sequences

# $(call findGoals,goalpattern...)
#   Find goal patterns in command-line goal list. Returns whitespace separated
# 	list of matched goals or empty string.
define findGoals =
$(filter $(1),$(GOAL_LIST))
endef

# $(call includeIfGoals,goalpattern...,makefile)
# 	Conditionally include makefile if one of the goal patterns matches the
# 	command-line goal list.
define includeIfGoals =
$(if $(call findGoals,$(1)),$(eval include $(2)))
endef

# $(call neq,op1,op2)
# $(call eq,op1,op2)
# 	Comparison operators. GNU Make has no comparison functions. Why??? Fake it.
# 	Returns non-empty string if true.
neq = $(filter-out $(1),$(2))
eq  = $(if $(call neq,$(1),$(2)),,1)

# $(call isFile,file)
# $(call isDir,file)
# 	Tests if file exists and specifies a regular file (directory).
# 	Returns non-empty string on true, empty string on false.
isFile 	= $(shell  if [ -f $(1) ]; then echo 1; fi)
isDir 	= $(shell  if [ -d $(1) ]; then echo 1; fi)

# $(call findReqFile,file,errmsg)
# 	Find the required file. On failure calls error with appended optional
# 	errmsg. Returns absolute filename.
define findReqFile =
$(if $(realpath $(1)),$(strip $(realpath $(1))),\
$(error $(1): No such file$(if $(2),: $(strip $(2)),)))
endef

# $(call makePath,dir...)
# 	Make search path dir[:dir...].
define makePath =
$(shell 
			p=""; \
			for d in $(1); \
			do \
				if [ -z "$${p}" ];
				then \
					p="$${d}"; \
				else \
  				p="$${p}:$${d}"; \
				fi; \
			done; \
			echo "$${p}")
endef

# $(call printPkgBanner,pkgname,arch,goallist)
# 	Print the package banner. Typically:
# 		pkgname 	= RNMAKE_PKG_FULL_NAME
# 		arch			= RNMAKE_ARCH
# 		goallist	= MAKECMDGOALS
define printPkgBanner =
	@if [ "$(MAKELEVEL)" = "0" ]; then \
	printf "$(color_pkg_banner)$(boldline)\n";\
	printf "Package:       $(1)\n";\
	printf "Package Root:  $(RNMAKE_PKG_ROOT)\n";\
	printf "Architecture:  $(2)\n";\
	printf "Directory:     $(CURDIR)\n";\
	printf "Goal(s):       $(3)\n";\
	printf "Start:         `date`\n";\
	printf "$(boldline)$(color_end)\n";\
	fi
endef

# $(call printDirBanner,dir,goal)
# 	Print directory banner. The dir parameter is relative path from 
# 	$(RNMAKE_PKG_ROOT). If the dir parameter is empty, the basename of
# 	$(RNMAKE_PKG_ROOT) is used.
define printDirBanner =
	@if [ "$(1)" != "" ]; then \
		subdirname="$(patsubst $(dir $(RNMAKE_PKG_ROOT))%,%,$(CURDIR)/$(1))";\
	else \
		subdirname=$(notdir $(RNMAKE_PKG_ROOT));\
	fi; \
	printf "\n";\
	printf "$(color_dir_banner)$(normline)\n";\
	printf "Directory: $$subdirname\n";\
	printf "Goal:      $(2)\n";\
	printf "$(normline)$(color_end)\n";
endef

# $(call printGoalBanner,goal)
# 	Print goal banner.
define printGoalBanner =
	@if [ "$(1)" != "" ]; then \
		printf "\n     $(color_tgt_file)$(1)$(color_end)\n";\
	fi
endef

# $(call printGoalDesc,goal,desc)
#
# Print goal with optional description.
define printGoalDesc =
	$(call printGoalBanner,$(1));
	$(if $(2),@printf "$(2)\n",)
endef

#	$(call printEchoTgtGoalDesc,desc)
# 	Silly print goal with description. The goal is determined by stripping off
# 	the 'echo-' prefix from the current target $(@).
define printEchoTgtGoalDesc =
	$(call printGoalDesc,$(patsubst echo-%,%,$(@)),$(1))
endef

# $(call printFooter,curgoal,lastgoal)
# 	Conditionally print footer. If the goal is the last command-line goal and
# 	the make level is 0 (top), then the footer is printed.
define printFooter =
	@if [ "$(MAKELEVEL)" = "0" -a "$(1)" = "$(2)" ]; then \
	printf "\n";\
	printf "$(color_pkg_banner)              ###\n";\
	printf "Finished: `date`\n";\
	printf "              ###$(color_end)\n";\
	fi
endef

# $(printTgtGoal)
# 	Print goal banner using the current target $(@).
printTgtGoal = $(call printGoalBanner,$(@))


eqline := \
================================================================================
dotline := \
................................................................................
tildeline := \
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
underline := \
________________________________________________________________________________

boldline := $(eqline)
normline := $(tildeline)
