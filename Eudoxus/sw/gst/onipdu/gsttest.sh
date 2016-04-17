#!/bin/bash

# Package:    Eudoxus
# Subpackage: GStreamer
# Element:    onipduenc, onipdudec
# File:       gsttest.sh

#dbg_level='--gst-debug-level=2'
#dbg_cat='--gst-debug=onipduenc:5,onipdudec:5'
plugin_path='--gst-plugin-path=/prj/pkg/Eudoxus/dist/dist.x86_64/lib/gst'

gst-inspect  ${dbg_level} ${dbg_cat} ${plugin_path} onipduenc

# simple file transfer - the in and out files should be exact copies.
#gst-launch ${dbg_level} ${dbg_cat} ${plugin_path} \
#  filesrc location=gstonipdu.cxx \
#  ! onipduenc \
#  ! onipdudec \
#  ! filesink location=o.out

# file transfer over udp - the in and out files should be exact copies.
#gst-launch ${dbg_level} ${dbg_cat} ${plugin_path} \
#  filesrc location=gstonipdu.cxx \
#  ! onipduenc \
#  ! udpsink host=192.168.9.140 port=4000

#gst-launch ${dbg_level} ${dbg_cat} ${plugin_path} \
#  ! udpsrc port=4000 \
#  ! onipdudec \
#  ! filesink location=o.out

#gst-launch ${dbg_level} ${dbg_cat} ${plugin_path} \
#    filesrc location=../pcsviewersink/testdata/pcl_logo0.pcs blocksize=4000000 \
#  ! onipduenc compression=zlib \
#  ! udpsink host=192.168.9.140 port=4000

gst-launch ${dbg_level} ${dbg_cat} ${plugin_path} \
    multifilesrc location=../testdata/pcl_logo%d.pcs \
                blocksize=4000000 loop=true \
  ! onipduenc compression=zlib \
  ! udpsink host=192.168.2.16 port=4000
