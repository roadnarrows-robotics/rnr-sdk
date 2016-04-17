#!/bin/bash

# Package:    Eudoxus
# Subpackage: GStreamer
# Elements:   onisrc, onipduenc
# File:       eudoxus-udp.sh

host="192.168.9.24"
port=4000

#dbg_level='--gst-debug-level=3'
#dbg_cat='--gst-debug=onisrc:5'
plugin_path='--gst-plugin-path=/prj/pkg/Eudoxus/dist/dist.linaro/lib/gst'

# simple debugging test script
gst-launch-0.10 ${dbg_level} ${dbg_cat} ${plugin_path} \
    onisrc depth-node=true

#  ! oni/pcs-ascii,width=320,height=240 \
#  ! onipduenc \
#  ! udpsink host=${host} port=${port}
