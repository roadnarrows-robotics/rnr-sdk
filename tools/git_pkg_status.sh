#!/bin/bash
# Determine the status all ROS packages found in the user's workspace.
#

rosdistro="${ROS_DISTRO}"
rospkgpath="${ROS_PACKAGE_PATH}"

if [ ! -z "${1}" ]
then
  rosdistro="${1}"
fi

# print error
x=${rosdistro:?"No ROS distro specified."}

rosroot=/opt/ros/${rosdistro}

findws()
{
  echo "${rospkgpath}" | \
  tr ':' '\n' | \
  while read ws
  do
    t=${ws##${rosroot}}
    if [[ ${ws} = ${t} ]]
    then
      echo ${ws}
      return
    fi
  done
  echo ""
}

rosws=$(findws)

if [ ! -d "${rosws}" ]
then
  echo "${rosws}: ROS workspace does not exist." >&2 
  exit 2
fi

echo "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
echo " ROS distro: ${rosdistro}"
echo " Workspace:  ${rosws}"
echo " Date:       $(date)"
echo "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"

for pkg in ${rosws}/*
do
  if [ ! -d ${pkg} ]
  then
    continue
  fi
  cd ${pkg}
  pkgbase=$(basename ${pkg})
  printf "%-30s " ${pkgbase}
  f=$(git status -s -uno)
  if [ -z "${f}" ]
  then
    printf "  "
  else
    printf "M "
  fi
  if [ -f "${pkgbase}/package.xml" ]
  then
    version=$(xmllint --xpath "/package/version/text()" ${pkgbase}/package.xml)
  elif [ -f "./package.xml" ]
  then
    version=$(xmllint --xpath "/package/version/text()" ./package.xml)
  else
    version=""
  fi
  printf "%-10s " ${version}
  printf "\n"
done
