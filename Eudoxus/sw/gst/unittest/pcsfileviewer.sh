#!/bin/bash

# Package:    Eudoxus
# Subpackage: GStreamer
# Elements:   onipdudec, pcsviewersink
# File:       client-pcsviewer.sh

#dbg_level='--gst-debug-level=3'
dbg_cat='--gst-debug=pcsfilesrc:5,pcsviewersink:5'
plugin_path='--gst-plugin-path=/prj/pkg/Eudoxus/dist/dist.x86_64/lib/gst'

# simple debugging test script
gst-launch ${dbg_level} ${dbg_cat} ${plugin_path} \
    pcsfilesrc location=${1} loop=true \
  ! oni/oni,framerate=5/1 \
  ! pcsviewersink
