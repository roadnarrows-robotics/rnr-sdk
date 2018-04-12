#
# File:
#   rnrsdk.sh
#
# Usage:
#   source rnrsdk.sh
#
# Description:
#   RoadNarrows Robotics SDK specific environment and utilites.
#
#   Variables:
#     rnrsdkPkgDeps   Ordered list of rnr-sdk package dependencies.
#
#   Functions:
#     rnrsdkIsGitRepo() Test if directory is part of the roadNarrows-robotics
#                       rnr-sdk repository
#     rnrsdkSortByDep() Sort packages by rnr-sdk package dependencies.
#

#echo "DBG: rnrsdk.sh" >&2

# RoadNarrows Robotics SDK package dependencies
# Note: The botsense package should follow immediately after netmsgs but has
#       residual dependencies on i2c and libserial
rnrsdkPkgDeps='
  rnmake
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

#echo "DBG: rnrsdkPkgDeps:" ${rnrsdkPkgDeps} >&2

# rnrsdkIsGitRepo dir
rnrsdkIsGitRepo()
{
  #echo "DBG: rnrsdk_isGitRepo" ${1} >&2

  if ! which git >/dev/null
  then
    return 1
  fi

  rc=1
  cwd=${PWD}

  cd ${1}

  url=$(git config --get remote.origin.url)

  #echo "DBG: url:" ${url} >&2

  if [ "${url}" = "https://github.com/roadnarrows-robotics/rnr-sdk" ]
  then
    rc=0
  fi

  cd ${cwd}

  #echo "DBG: rc:" ${rc} >&2

  return ${rc}
}

# rnrsdkSortByDep [pkg [pkg ...]]
rnrsdkSortByDep()
{
  #echo "DBG: rnrsdkSortByDep" ${@} >&2
  ilist=${@}
  rnrlist=
  for dep in ${rnrsdkPkgDeps}
  do
    olist=
    havematch=false
    for pkg in ${ilist}
    do
      #if [[ ${havematch} == false ]]
      #then
        #echo "DBG: cmp:" ${dep} " <-> " ${pkg} >&2
      #fi
      b=$(basename ${pkg})
      if [[ ( ${havematch} == false ) && ( "${b}" == "${dep}" ) ]]
      then
        #echo "DBG: match: ${dep}" >&2
        rnrlist="${rnrlist} ${pkg}"
        havematch=true
      else
        olist="${olist} ${pkg}"
      fi
    done
    ilist="${olist}"
  done
  #echo "DBG: rnrlist: ${rnrlist}" >&2
  #echo "DBG: usrlist: ${ilist}" >&2
  echo ${rnrlist} ${ilist}
}
