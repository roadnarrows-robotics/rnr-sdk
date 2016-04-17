#!/bin/bash

#dbg_level='--gst-debug-level=1'
dbg_cat='--gst-debug=onisrc:3' #,onipduenc:5'
plugin_path='--gst-plugin-path=/usr/local/lib/gst'

# simple debugging test script
gst-launch ${dbg_level} ${dbg_cat} ${plugin_path} \
    onisrc depth-node=true image-node=true \
  ! oni/pcs-oni,width=640,height=480,framerate=30/1 \
  ! onipduenc compression=none \
  ! udpsink host=192.168.9.140 port=4000

