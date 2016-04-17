#!/bin/bash
/home/root/battery-logger &

while [ 1 ]
do
	cat /sys/power/state
	sleep 1
done
