################################################################################
#
# Rules.test.mk
#
ifdef RNMAKE_DOXY
/*! 
\file 

\brief Provides "test" targets.

This file is automatically included by \ref Rules.mk when one or more of the
test make goals are specified.

\pkgsynopsis
RN Make System

\pkgfile{Rules.test.mk}

\pkgauthor{Daniel Packard,daniel@roadnarrows.com}
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

_RULES_TEST_MK = 1

RNMAKE_TEST_ENABLED = y

GOALS_WITH_SUBDIRS += run-test

# Add test targets to local programs
RNMAKE_LOC_PGMS += $(RNMAKE_TEST_PGMS)

GOALS_WITH_SUBDIRS += test

# colors
color_test = $(color_pre)$(color_light_blue)

# Make specific test programs
.PHONY: 	test
test: pkgbanner all subdirs-test
	$(footer)

# Run test programs
.PHONY: run-test
run-test: pkgbanner echo-run-test do-test subdirs-run-test
	$(footer)

.PHONY: echo-run-test
echo-run-test:
	$(call printEchoTgtGoalDesc,Run all tests)

.PHONY: do-test
do-test:
	@for f in $(RNMAKE_TEST_PGMS); \
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
