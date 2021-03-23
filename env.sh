#
# RoadNarrows Robotics Software Development Kits environment bash shell script.
#
# source env.sh
#

_root=$(dirname ${BASH_SOURCE[0]})

# default rnr-sdk make variables
export RNR_WORKSPACE="${RNR_WORKSPACE:-$(realpath ${_root})}"
export RNR_ARCH_DFT="${RNR_ARCH_DFT:-x86_64}"
export RNR_INSTALL_XPREFIX="${RNR_INSTALL_XPREFIX:-${HOME}/xinstall}"
export RNR_PYTHON="${RNR_PYTHON:-/usr/bin/python3}"

# python3 version stripped of prefix string and revision number.
export RNR_PYTHON_VERSION=$(${RNR_PYTHON} --version 2>&1 | \
  sed -e 's/Python[[:blank:]]\+\([[:digit:]]\+\.[[:digit:]]\+\).*$/\1/')

# rnmake make variables
export RNMAKE_ROOT="${RNR_WORKSPACE}/rnmake"
export RNMAKE_ARCH_DFT="${RNR_ARCH_DFT}"
export RNMAKE_INSTALL_XPREFIX="${RNR_INSTALL_XPREFIX}"

#echo RNR_WORKSPACE="${RNR_WORKSPACE}"
#echo RNR_ARCH_DFT="${RNR_ARCH_DFT}"
#echo RNR_INSTALL_XPREFIX="${RNR_INSTALL_XPREFIX}"
#echo RNR_PYTHON="${RNR_PYTHON}"
#echo RNR_PYTHON_VERSION="${RNR_PYTHON_VERSION}"
#echo RNMAKE_ROOT="${RNMAKE_ROOT}"
#echo RNMAKE_ARCH_DFT="${RNMAKE_ARCH_DFT}"
#echo RNMAKE_INSTALL_XPREFIX="${RNMAKE_INSTALL_XPREFIX}"

unset _root
