#!/bin/sh
###############################################################################
# File: pypath.sh
#
# Description:
#   When building, testing, or running against the local package, PYTHONPATH
# needs to point at the local package directory.
#
# To run (anywhere from inside netmsgs package): . ./pypath.sh
#
###############################################################################

pkgdir=${PWD%%/netmsgs/*}

export PYTHONPATH=${pkgdir}/netmsgs/NetMsgs/modules:$PYTHONPATH
