#!/bin/bash

# Package:    Eudoxus
# Subpackage: GStreamer
# Element:    pcsviewersink
# File:       gsttest.sh

dbg_level='--gst-debug-level=3'
dbg_cat='--gst-debug=pcsviewersink:3,multifilesrc:3'
plugin_path='--gst-plugin-path=/prj/pkg/Eudoxus/dist/dist.x86_64/lib/gst'

# simple debugging test script
#gst-launch ${dbg_level} ${dbg_cat} ${plugin_path} \
#    multifilesrc location=testdata/frame-small%d.pcs loop=true \
#  ! pcsviewersink point-color=0x4484ff point-size=3

#gst-launch ${dbg_level} ${dbg_cat} ${plugin_path} \
#    udpsrc port=4000 \
#  ! onipdudec \
#  ! pcsviewersink

#gst-launch ${dbg_level} ${dbg_cat} ${plugin_path} \
#    multifilesrc location=testdata/f%d.pcs start-index=55 stop-index=58 \
#                  blocksize=4000000 loop=true \
#  ! onipduenc \
#  ! udpsink host=192.168.9.140 port=4000

#gdb --args gst-inspect ${dbg_level} ${dbg_cat} ${plugin_path} pcsviewersink

gst-launch-0.10 -v ${dbg_level} ${dbg_cat} ${plugin_path} \
    multifilesrc location=../testdata/pcl_logo%d.pcs \
                  blocksize=4000000 loop=true \
  ! pcsviewersink
