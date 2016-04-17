#!/bin/bash

# Package:    Eudoxus
# Subpackage: GStreamer
# Elements:   onisrc, onipduenc
# File:       eudoxus-viewer.sh

#dbg_level='--gst-debug-level=3'
#dbg_cat='--gst-debug=onisrc:5'
plugin_path='--gst-plugin-path=/prj/pkg/Eudoxus/dist/dist.odroid/lib/gst'

# simple debugging test script
gst-launch-1.0 ${dbg_level} ${dbg_cat} ${plugin_path} \
    onisrc depth-node=true \
  ! pcsviewersink
