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
## See gpioexport(1) and /etc/init.d/eudoxus_init(1)
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

# Some user defined state variables
declare -i action_state=0   # action state, 0 is not running, 1 is running 

#
# Hook to test if user defined action is running.
#
# Return 0 or 1.
#
action_running()
{
  # ADD HOOK HERE

  echo ${action_state}
}

#
# Hook to start user defined action function.
#
action_start()
{
  echo "Action started"
  action_state=1 # start action function

  # ADD HOOK HERE
}

#
# Hook to stop user defined action function.
#
action_stop()
{
  echo "Action stopped"
  action_state=0 # stop action function

  # ADD HOOK HERE
}

#
# Set LED
#   
# If not action_state, thenthe LED should be solid on
# If action_state, then the LED should slowly blink
#
setled()
{
  if [ ${action_state} -eq 0 ]
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
            action_state=$(action_running)
            if [ ${paction_state} -eq 1 ] # currently action_state
            then
              action_stop
            else # currently not action_state
              action_start
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
