#!/bin/bash
#
# Update all ROS packages found under user's ROS workspace.
#
# Usage: rnros_git_pull_all.sh

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
echo " Pulling all workspace packages found under ${rosws}"
echo "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"

for pkg in ${rosws}/*
do
  if [ ! -d ${pkg} ]
  then
    continue
  fi
  cd ${pkg}
  echo
  echo "-----------------------------------------------------------------------"
  echo "Pulling package ${pkg}"
  echo "-----------------------------------------------------------------------"
  git pull
done
