#!/bin/bash

# Package:    Eudoxus
# Subpackage: GStreamer
# Element:    pscfilesrc
# File:       gsttest.sh

dbg_level='--gst-debug-level=2'
dbg_cat='--gst-debug=pscfilesrc:5'
plugin_path='--gst-plugin-path=/prj/pkg/Eudoxus/dist/dist.x86_64/lib/gst'

# simple debugging test script
gst-launch ${dbg_level} ${dbg_cat} ${plugin_path} \
    pcsfilesrc location=../testdata/pcl_logo0.pcs \
  ! filesink location=out.pcs
