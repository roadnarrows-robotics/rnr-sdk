#
# rnnamefilter
#

# To perform a simple test, replace '<filename>' with a file name of and
# existing file and run:

gst-launch --gst-plugin-path=/prj/pkg/Eudoxus/dist/dist.x86_64/lib \
  filesrc location=<filename> ! rnnamefilter ! filesink location=tmp.out


The contents of <filename> should be identical with tmp.out.
