#!/bin/bash
#
# File:
#   ls_sdk_installed.sh
#
# Usage:
#   ls_sdk_installed.sh [loc]
#     loc   {/prj | /usr/local | <dir>}. DEFAULT: /prj
#
# Description:
#   List relevant directories and files from all RoadNarrow SDK installed
#   packages.
#

loc='/prj'
etc=${loc}/etc

if [[ $# > 0 ]]
then
  loc="$1"
  etc=/etc
fi

if [[ ! -d "${loc}" ]]
then
  printf "'${loc}': Not a directory\n"
  exit 2
fi

lscolor()
{
  printf "\\033[1;31m${1}:\\033[0m\n"
  ls --color=always $*
  echo
}

clear
echo '#########################################################################'
echo "  ${loc}"
echo '#########################################################################'

lscolor ${loc}/include
lscolor ${loc}/include/botsense
lscolor ${loc}/include/CogniBoost
lscolor ${loc}/include/Dynamixel
lscolor ${loc}/include/Dynamixel/dxl
lscolor ${loc}/include/Hekateros
lscolor ${loc}/include/Kuon
lscolor ${loc}/include/Laelaps
lscolor ${loc}/include/rnr
lscolor ${loc}/include/rnr/appkit
lscolor ${loc}/include/rnr/hid
lscolor ${loc}/include/rnr/imu
lscolor ${loc}/include/rnr/mot
lscolor ${loc}/include/rnr/tinyxml

lscolor ${loc}/lib
lscolor ${loc}/lib/botsense
lscolor ${loc}/lib/cmake/rnr
lscolor ${loc}/lib/rnr
lscolor ${loc}/lib/python2.7/site-packages
lscolor ${loc}/lib/python2.7/site-packages/BotSense
lscolor ${loc}/lib/python2.7/site-packages/Hekateros
lscolor ${loc}/lib/python2.7/site-packages/Laelaps
lscolor ${loc}/lib/python2.7/site-packages/NetMsgs
lscolor ${loc}/lib/python2.7/site-packages/rnr

lscolor ${etc}/hekateros
lscolor ${etc}/kuon
lscolor ${etc}/laelaps
lscolor ${etc}/pan_tilt
lscolor ${etc}/init.d
lscolor ${etc}/rc3.d
lscolor ${etc}/ld.so.conf.d
lscolor ${etc}/profile.d
lscolor ${etc}/udev/rules.d

lscolor ${loc}/bin

