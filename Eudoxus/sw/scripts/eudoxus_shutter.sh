#!/bin/bash
#
# Script: eudoxus_shutter.sh
#

## \file
##
## $LastChangedDate$
## $Rev$
## 
## \brief Example script to demonstrate how to interface with the Eudoxus
##  user push button and LED.
##
## \par Note:
## The /sys/class/gpio exported interface is assumed already created.
## See gpioexport(1) and /etc/init.d/eudoxus_int(1)
## 
## \author: Robin Knight (robin.knight@roadnarrows.com)
## 
## \par Copyright:
##   (C) 2016.  RoadNarrows LLC.
##   (http://www.roadnarrows.com)
##   All Rights Reserved

#
# @EulaBegin@
# @EulaEnd@
#

# Exported GPIO numbers.
declare -r gpio_led=18      # blue user LED output GPIO
declare -r gpio_bttn=19     # user push button input GPIO

# GPIO values.
# For the user button, the released state is 1; pushed state is 0.
# For the user LED, the LED off state is 0, the on state is 1.
declare -i old_button=1   # old push button state
declare -i button=1       # current push button state
declare -i led=0          # current led state

# Times
declare -i t_led=$(date +%s)    # last led state change time
declare -i t_cur                # current time
declare -i t_diff               # time difference

# Some state variables
declare -i playing=0    # some user defined play state

#
# Hook to start user defined play function.
#
play_start()
{
  echo "Play started"
  playing=1 # start play function

  # ADD HOOK HERE
}

#
# Hook to stop user defined play function.
#
play_stop()
{
  echo "Play stopped"
  playing=0 # stop play function

  # ADD HOOK HERE
}

#
# Set LED
#   
# If not playing, thenthe LED should be solid on
# If playing, then the LED should slowly blink
#
setled()
{
  if [ ${playing} -eq 0 ]
  then
    if [ ${led} -eq 0 ]
    then
      led=1
      gpiowrite ${gpio_led} ${led}
      t_led=${t_cur}
    fi
  else
    t_cur=$(date +%s)
    t_diff=${t_cur}-${t_led}
    if [ ${t_diff} -ge 1 ]
    then
      if [ ${led} -eq 0 ]
      then
        led=1
      else
        led=0
      fi
      gpiowrite ${gpio_led} ${led}
      t_led=${t_cur}
    fi
  fi
}

#
# Block-wait on user push button events.
#
while :
do
  gpionotify --timeout=1.0 --monitor ${gpio_bttn} | \
  while read button
  do
    case ${button}
    in
      0) #echo "Button is in the pushed state"
          ;;
      1) #echo "Button is in the released state"
          # pushed --> released
          if [ ${old_button} -eq 0 ]
          then
            if [ ${playing} -eq 1 ] # currently playing
            then
              play_stop
            else # currently not playing
              play_start
            fi
          fi
          ;;
     -1|*)  echo "Error"
            exit 4;
          ;;
    esac
    old_button=${button}
    setled
  done
done

exit 0
