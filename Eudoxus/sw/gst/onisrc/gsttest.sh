#!/bin/bash

# Package:    Eudoxus
# Subpackage: GStreamer
# Element:    onisrc
# File:       gsttest.sh

dbg_level='--gst-debug-level=2'
dbg_cat='--gst-debug=onisrc:5'
plugin_path='--gst-plugin-path=/prj/pkg/Eudoxus/dist/dist.x86_64/lib/gst'

# simple debugging test script
gst-launch ${dbg_level} ${dbg_cat} ${plugin_path} \
    onisrc  depth-node=true image-node=true \
  ! oni/pcs-ascii,width=320,height=240 \
  ! filesink location=out.pcs
