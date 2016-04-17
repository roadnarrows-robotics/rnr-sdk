#!/bin/bash

#dbg_level='--gst-debug-level=2'
dbg_cat='--gst-debug=onisrc:3,onipduenc:3,tcpserversink:5'
plugin_path='--gst-plugin-path=/usr/local/lib/gst'

# simple debugging test script
gst-launch ${dbg_level} ${dbg_cat} ${plugin_path} \
    onisrc depth-node=true \
  ! oni/pcs-binary,width=320,height=240 \
  ! onipduenc \
  ! oni/pdu \
  ! tcpserversink protocol=none port=4000

