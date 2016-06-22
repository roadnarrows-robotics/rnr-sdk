###############################################################################
#
# /etc/profile.d/kuon.sh
#
# RoadNarrows Kuon global environment.
# 2014.03.13
#
###############################################################################

# ROS environment
if [ -f /opt/kuon_ros/devel/setup.sh ]
then
  . /opt/kuon_ros/devel/setup.sh
elif [ -f /prj/ros/groovy/devel/setup.sh ]
then
  . /opt/ros/groovy/setup.sh
elif [ -f /opt/ros/groovy/setup.sh ]
then
  . /opt/ros/groovy/setup.sh
fi

# ROS gstreamer config
export GSCAM_CONFIG="v4l2src device=/dev/eecam ! video/x-raw-rgb,width=320,height=240,framerate=10/1 ! ffmpegcolorspace"

# rnmake
export RNMAKE_ARCH_DFT=linaro

# paths
export PYTHONPATH=$PYTHONPATH:/prj/lib/python2.7/site-packages
export PATH=$PATH:/prj/bin
