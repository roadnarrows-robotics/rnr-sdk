#!/bin/bash

#dbg_level='--gst-debug-level=3'
dbg_cat='--gst-debug=onisrc:5'
plugin_path='--gst-plugin-path=/usr/local/lib/gst'

# simple debugging test script
gst-launch ${dbg_level} ${dbg_cat} ${plugin_path} \
    onisrc  depth-node=true image-node=false \
  ! oni/pcs-oni,width=320,height=240 \
  ! filesink location=/tmp/out.pcd
