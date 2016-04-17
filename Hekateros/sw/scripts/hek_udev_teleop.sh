#!/bin/sh
#
# Script: hek_teleop_udev.sh
#

## OBSOLETE    OBSOLETE    OBSOLETE

## \file
##
## $LastChangedDate: 2015-05-01 10:34:15 -0600 (Fri, 01 May 2015) $  
## $Rev: 3972 $ 
## 
## \brief
## Hekateros teleoperation udev launch script.
##
## The script is executed by an udev rule when an Xbox360 is discovered on an
## USB port.
## 
## \author: Robin Knight (robin.knight@roadnarrows.com)
## 
## \par Copyright:
##   (C) 2013.  RoadNarrows LLC.
##   (http://www.roadnarrows.com)
##   All Rights Reserved

#
# @EulaBegin@
# @EulaEnd@
#

# common environment
. /opt/hekateros_ros/devel/setup.sh   # on target
#. /prj/catkin_rnr/devel/setup.sh    # test

kill_em_all()
{
  if [ -v pid1 ]
  then
    kill -15 $pid1
  fi
  if [ -v pid2 ]
  then
    kill -15 $pid2
  fi
  exit 0
}

trap kill_em_all 1 2 15

rosrun hid xbox_360 &
pid1=$!

sleep 1

rosrun hekateros_control hek_teleop &
pid2=$!

wait $pid1 $pid2 

exit 0
