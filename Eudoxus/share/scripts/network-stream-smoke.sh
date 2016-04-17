CLIENT="192.168.9.140"
PORT="4000"

#gst-launch v4l2src ! video/x-raw-yuv,width=320,height=240 ! queue !  ffmpegcolorspace ! queue ! smokeenc ! udpsink host=${CLIENT} port=$PORT

#gst-launch videotestsrc ! smokeenc ! udpsink host=${CLIENT} port=$PORT

#gst-launch -v --gst-debug-level=5 --gst-debug-no-color --gst-plugin-path=/usr/local/lib/gst onisrc node=depth ! queue ! identity ! udpsink host=${CLIENT} port=${PORT}

gst-launch -v --gst-debug-level=2 --gst-debug=onisrc:3 --gst-plugin-path=/usr/local/lib/gst onisrc node=depth ! oni/pcs-binary,width=640,height=480 ! udpsink host=${CLIENT} port=${PORT}

# to catch on client
#gst-launch udpsrc port=${PORT} ! smokedec ! autovideosink 
