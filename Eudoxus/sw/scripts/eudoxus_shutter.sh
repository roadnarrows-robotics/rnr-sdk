#!/bin/bash
# Example script to show how to interface with the user push button and LED.
#
# Note:
# The /sys/class/gpio exported interface is assumed already created.
# See gpioexport(1) and /etc/init.d/eudoxus_int(1)
#
# RoadNarrows LLC
#

# Exported GPIO numbers.
declare -r gpio_led=18
declare -r gpio_bttn=19

# GPIO old and current values.
# For the user button, the released state is 1; pushed state is 0.
# For the user LED, the LED off is 0, on is 1.
declare -i old_button=1   # old push button state
declare -i button=1       # current push button state
declare -i led=0          # current led state

# Times
declare -i t_led=$(date +%s)
declare -i t_cur
declare -i t_diff

# Some state variables
declare -i playing=0    # some undefined play state

setled()
{
  if [ ${playing} -eq 0 ]
  then
    if [ ${led} -eq 0 ]
    then
      led=1
      gpiowrite ${gpio_led} ${led}
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
# Block-wait on user button events.
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
            # currently playing
            if [ ${playing} -eq 1 ]
            then
              echo "Play stopped"
              playing=0 # turn off play
            else
              echo "Play started"
              playing=1 # start play
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

oldstate='1'
clear
banner "Robin"

while :
do
  v=$(cat /sys/class/gpio/gpio19/value)
  case ${v}
  in
    0*) state='0';;
    1*) state='1';;
  esac

  if [[ ${state} != ${oldstate} ]]
  then
    case ${state}
    in
      0) banner "One Sick"; banner "  Dude";;
      1) clear; banner "Robin";;
    esac
    oldstate=${state}
  fi

  sleep 0.1
done
