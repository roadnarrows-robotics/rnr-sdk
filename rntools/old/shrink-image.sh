#!/bin/bash
# This isn't actually a fully working script yet.  Manual work will need done.
# This info came from: http://softwarebakery.com/shrinking-images-on-linux

# Remove this if the script is ever finished
echo "This script isn't ready to be run"
echo "Please treat as a README for now"
exit

# Make sure loopback is enabled
sudo modprobe loop

# Request a new free loopback device
sudo losetup -f

# Create a device at the returned path from previous command
sudo losetup /dev/loop0 myimage.img

# Load the partitions too
sudo partprobe /dev/loop0

# Resize the partition(s)
sudo gparted /dev/loop0
# Resize the rootfs or overo partition as small as desired.  You can go as low
# as no free space, but add back 10 MiB or so for file system-related data, or
# even more if you want to actually be able to use the image. After you apply
# the changes close GParted.

# Unload the loopback device
sudo losetup -d /dev/loop0

# Now that the data is 'compressed' we can truncate the image file to match the
# new size.

# First find the end of the data
fdisk -l myimage.img

# Disk myimage.img: 6144 MB, 6144000000 bytes, 12000000 sectors
# Units = sectors of 1 * 512 = 512 bytes
# Sector size (logical/physical): 512 bytes / 512 bytes
# I/O size (minimum/optimal): 512 bytes / 512 bytes
# Disk identifier: 0x000ea37d

#       Device Boot      Start         End      Blocks   Id  System
# myimage.img1            2048     9181183     4589568    b  W95 FAT32
#
# This shows that the last (only) partition ends on sector 9181183 and sectors
# are 512 bytes in size.  We need to add 1 since blocks start at 0
truncate --size=$[(9181183+1)*512] myimage.img

