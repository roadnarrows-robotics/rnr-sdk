clear;
echo '########################################################################';

lscolor()
{
  printf "\\033[1;31m${1}:\\033[0m\n"
  ls --color=always $*
  echo
}

lscolor include
lscolor include/botsense
lscolor include/CogniBoost
lscolor include/Dynamixel
lscolor include/Dynamixel/dxl
lscolor include/Hekateros
lscolor include/Kuon
lscolor include/Laelaps
lscolor include/rnr
lscolor include/rnr/appkit
lscolor include/rnr/hid
lscolor include/rnr/imu
lscolor include/rnr/mot
lscolor include/rnr/tinyxml

lscolor lib
lscolor lib/botsense
lscolor lib/cmake/rnr
lscolor lib/rnr
lscolor lib/python2.7/site-packages
lscolor lib/python2.7/site-packages/BotSense
lscolor lib/python2.7/site-packages/Hekateros
lscolor lib/python2.7/site-packages/Laelaps
lscolor lib/python2.7/site-packages/NetMsgs
lscolor lib/python2.7/site-packages/rnr

lscolor bin
