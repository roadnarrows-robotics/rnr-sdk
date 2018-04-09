###############################################################################
# Makefile
#
# Version:
# 	$LastChangedDate: 2009-09-04 12:09:46 -0600 (Fri, 04 Sep 2009) $
# 	$Rev: 118 $
#
# RoadNarrows Product Make Makefile
###############################################################################

help:
	@echo ""
	@echo "Usage: make <tgt>"
	@echo "       <tgt> - target"
	@echo ""
	@echo " Distribution Package Targets"
	@echo "pkg          - makes RNProd distribution package (future)"
	@echo ""
	@echo " Utility Targets"
	@echo "clean            - remove intermediate and temporary files"
	@echo "distclean        - clean and remove distribution copies" 


#
# 
#
pkg : pkgname = RNProd

#
# Package Target
#
pkg: 
	@echo "Future target"
	

#
# Utilities
#

clean:
	find . -name '*.py[co]' | xargs rm -f
	find . -name '*~' | xargs rm -f
	find . -name '\#*\#' | xargs rm -f

distclean: clean
	rm -fr $(dist)
