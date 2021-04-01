#
# RoadNarrows Robotics Software Development Kits environment bash shell script.
#
# source env.sh [arch]
#

# _prepend dir path
#   Prepend directory dir to search path iff dir is not already in path.
_prepend()
{
  local npath=
  # empty path
  if [ -z "${2}" ]
  then
    npath="${1}"
  # search path for dir
  else
    local parray comp
    #for comp in ${2//:/$'\n'}
    IFS=':' read -a parray <<< "${2}"
    for comp in "${parray[@]}"
    do
      # already in path?
      if [ "${1}" = "${comp}" ]
      then
        npath="${2}"
        break
      fi
    done
  fi
  # prepend new directory
  if [ -z "${npath}" ]
  then
    npath="${1}:${2}"
  fi
  echo "${npath}"
}

_root=$(dirname ${BASH_SOURCE[0]})

# default rnr-sdk make variables
export RNR_WORKSPACE="${RNR_WORKSPACE:-$(realpath ${_root})}"
export RNR_ARCH_DFT="${RNR_ARCH_DFT:-x86_64}"
export RNR_INSTALL_XPREFIX="${RNR_INSTALL_XPREFIX:-${HOME}/xinstall}"
export RNR_PYTHON="${RNR_PYTHON:-/usr/bin/python3}"
export RNR_ARCH="${1:-${RNR_ARCH_DFT}}"

# python3 version stripped of prefix string and revision number.
export RNR_PYTHON_VERSION=$(${RNR_PYTHON} --version 2>&1 | \
  sed -e 's/Python[[:blank:]]\+\([[:digit:]]\+\.[[:digit:]]\+\).*$/\1/')

# RoadNarrows Robotics SDK packages, list in order of dependence.
# Notes:
#   + The botsense package should follow immediately after netmsgs but has
#     residual dependencies on i2c and libserial.
#   + The rnmake package is excluded.
RNR_PKG_LIST='
  rntools
  librnr
  netmsgs
  libserial
  i2c
  gpio
  odroid
  botsense
  appkit
  Dynamixel
  peripherals
  CogniBoost
  Hekateros
  Laelaps
  PanTilt
  Kuon
  Fusion
  Eudoxus'

# fix up paths
_prefix=${RNR_INSTALL_XPREFIX}/${RNR_ARCH}
_pysite=${_prefix}/lib/python${RNR_PYTHON_VERSION}/site-packages
export PATH=$(_prepend ${_prefix}/bin ${PATH})
export LD_LIBRARY_PATH=$(_prepend ${_prefix}/lib ${LD_LIBRARY_PATH})
export LD_LIBRARY_PATH=$(_prepend ${_prefix}/lib/rnr ${LD_LIBRARY_PATH})
export LD_LIBRARY_PATH=$(_prepend ${_prefix}/lib/botsense ${LD_LIBRARY_PATH})
export PYTHONPATH=$(_prepend ${_pysite} ${PYTHONPATH})

# rnmake make variables
export RNMAKE_ROOT="${RNR_WORKSPACE}/rnmake"
export RNMAKE_ARCH_DFT="${RNR_ARCH_DFT}"
export RNMAKE_INSTALL_XPREFIX="${RNR_INSTALL_XPREFIX}"

#echo RNR_WORKSPACE="${RNR_WORKSPACE}"
#echo RNR_ARCH_DFT="${RNR_ARCH_DFT}"
#echo RNR_ARCH="${RNR_ARCH}"
#echo RNR_INSTALL_XPREFIX="${RNR_INSTALL_XPREFIX}"
#echo RNR_PYTHON="${RNR_PYTHON}"
#echo RNR_PYTHON_VERSION="${RNR_PYTHON_VERSION}"
#echo RNMAKE_ROOT="${RNMAKE_ROOT}"
#echo RNMAKE_ARCH_DFT="${RNMAKE_ARCH_DFT}"
#echo RNMAKE_INSTALL_XPREFIX="${RNMAKE_INSTALL_XPREFIX}"

unset _prepend _root _prefix _pysite
