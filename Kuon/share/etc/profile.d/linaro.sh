###############################################################################
#
# /etc/profile.d/linaro.sh
#
# RoadNarrows Hekateros linaro linux tweaks
# 2013.09.19
#
###############################################################################


# disable bad kernel timeout if root
if [ "${UID}" = "0" ]
then
  echo 0 > /proc/sys/kernel/hung_task_timeout_secs
fi
