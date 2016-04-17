#!/bin/bash

# Package:    Eudoxus
# Subpackage: GStreamer
# Elements:   onipdudec, pcsviewersink
# File:       client-pcsviewer.sh

port=4000

#dbg_level='--gst-debug-level=3'
dbg_cat='--gst-debug=tcpclientsrc:5,onipdudec:5,pcsviewersink:5'
plugin_path='--gst-plugin-path=/prj/pkg/Eudoxus/dist/dist.x86_64/lib/gst'

# simple debugging test script
gst-launch ${dbg_level} ${dbg_cat} ${plugin_path} \
    tcpclientsrc protocol=none host=eudoxus port=${port} \
  ! oni/pdu \
  ! onipdudec \
  ! pcsviewersink
