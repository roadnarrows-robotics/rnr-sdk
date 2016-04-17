# This example reads the value from gpio147, which is physically connected
# to the camera button.

while :
do
 cat /sys/class/gpio/gpio147/value
 usleep 500000
done
