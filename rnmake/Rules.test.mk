################################################################################
#
# Package: 	RN Make System 
#
# File:			Rules.test.mk
#
ifdef RNMAKE_DOXY
/*! 
\file 

\brief Provides "test" targets for Rules.mk

\author Daniel Packard (daniel@roadnarrows.com)
\author Robin Knight (robin.knight@roadnarrows.com)

\par Copyright:
(C) 2005-2018.  RoadNarrows LLC.
(http://www.roadnarrows.com)
\n All Rights Reserved

\cond RNMAKE_DOXY
 */
endif
#
# Permission is hereby granted, without written agreement and without
# license or royalty fees, to use, copy, modify, and distribute this
# software and its documentation for any purpose, provided that
# (1) The above copyright notice and the following two paragraphs
# appear in all copies of the source code and (2) redistributions
# including binaries reproduces these notices in the supporting
# documentation.   Substantial modifications to this software may be
# copyrighted by their authors and need not follow the licensing terms
# described here, provided that the new terms are clearly indicated in
# all files where they apply.
#
# IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
# OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
# PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
# DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
# EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
# THE POSSIBILITY OF SUCH DAMAGE.
#
# THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
# INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
# FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
# "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
# PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
#
################################################################################

export _RULES_TEST_MK = 1

# include TEST_PGMS to LOC_PGMS
export RNMAKE_TEST = true
GOALS_WITH_SUBDIRS += run-test

# colors
color_test = $(color_pre)$(color_light_blue)

# Make specific test programs
.PHONY: 	test
test: pkgbanner all
	$(footer)

# Run test programs
.PHONY: run-test
run-test: pkgbanner echo-run-test do-test subdirs-run-test
	$(footer)

.PHONY: echo-run-test
echo-run-test:
	$(call fnEchoGoalDesc,Run all tests)

.PHONY: do-test
do-test:
	@for f in $(TEST_PGMS); \
		do \
			printf "\n$(color_test)         $$f$(color_end)\n"; \
			if [ -x ./$(LOCDIR_BIN)/$$f ]; \
			then \
			  ./$(LOCDIR_BIN)/$$f; \
			else \
				printf "$(color_error)Program $$f does not exist. Did you 'make test' first?$(color_end)\n\n"; \
				break; \
			fi; \
	 done;
	$(footer)


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
