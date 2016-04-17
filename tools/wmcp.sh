#!/bin/bash

echo "Copying excutables"
cp /prj/xinstall/overo/bin/WM* /media/system/usr/bin/.

echo "Copying libraries"
cp /prj/xinstall/overo/lib/libWM*.* /media/system/usr/lib/.
cp /prj/xinstall/overo/lib/libtinyxml.* /media/system/usr/lib/.

echo "Syncing"
sync
