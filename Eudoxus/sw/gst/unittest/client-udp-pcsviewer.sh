#!/bin/bash

# Package:    Eudoxus
# Subpackage: GStreamer
# Elements:   onipdudec, pcsviewersink
# File:       client-pcsviewer.sh

port=$1

dbg_level='--gst-debug-level=4'
dbg_no_color='--gst-debug-no-color'
#dbg_cat='--gst-debug=pcsviewersink:5'
plugin_path='--gst-plugin-path=/prj/pkg/Eudoxus/dist/dist.x86_64/lib/gst'

# simple debugging test script
gst-launch-0.10 ${dbg_level} ${dbg_cat} ${dbg_no_color} ${plugin_path} \
    udpsrc port=${port} \
  ! onipdudec \
  ! pcsviewersink # point-size=1 # point-color=0x11ff00
