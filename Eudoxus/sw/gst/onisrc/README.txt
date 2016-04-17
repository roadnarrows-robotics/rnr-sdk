#
# onisrc
#

# To perform a simple test, run:

gst-launch --gst-plugin-path=/prj/pkg/Eudoxus/dist/dist.x86_64/lib \
  onisrc nodes=depth ! filesink location=tmp.oni

The file tmp.oni should be a file viewable from the NiViewer application.

# To perform a full test streaming over IP, run:

# On the target machine
gst-launch --gst-plugin-path=/prj/pkg/Eudoxus/dist/dist.x86_64/lib \
  onisrc nodes=depth,image ! udpsink host=<addr> port=<num>

# On the host machine
gst-launch udpsrc port=<num> ! filesink location=tmp.oni

