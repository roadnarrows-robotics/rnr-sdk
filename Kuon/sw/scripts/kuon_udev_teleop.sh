#!/bin/sh
#
# Script: kuon_teleop_udev.sh
#

## \file
##
## $LastChangedDate: 2014-01-15 10:19:58 -0700 (Wed, 15 Jan 2014) $  
## $Rev: 3479 $ 
## 
## \brief
## Kuon teleoperation udev launch script.
##
## The script is executed by an udev rule when an Xbox360 is discovered on an
## USB port.
## 
## \author: Robin Knight (robin.knight@roadnarrows.com)
## 
## \par Copyright:
##   (C) 2014.  RoadNarrows LLC.
##   (http://www.roadnarrows.com)
##   All Rights Reserved

#
# @EulaBegin@
# @EulaEnd@
#

# common environment
. /opt/kuon_ros/devel/setup.sh   # on target
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

rosrun kuon_control kuon_teleop &
pid2=$!

wait $pid1 $pid2 

exit 0
