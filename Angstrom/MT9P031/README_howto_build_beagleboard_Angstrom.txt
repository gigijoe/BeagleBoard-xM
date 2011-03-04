Compiler:
---------
using $Beagleboard/trunk/tools/arm-2009q1
-cd $Beagleboard/trunk/kernel-2.6.32
-make ARCH=arm omap3_beagle_cam_defconfig
-export PATH=$PATH:<absolute path>/arm-2009q1/bin
-make ARCH=arm CROSS_COMPILE=arm-none-linux-gnueabi- uImage

Preparing sd-card:
------------------
Format & partition sd-card at /dev/sda
-sudo $Beagleboard/binaries/mkcard.sh /dev/sda    

It creates two partitions /dev/sda1 and /dev/sda2, FAT32 and ext2 respectively

Mount /dev/sda, copy files to both partitions[/media/boot and /media/Angstrom] as follows: 

-sudo tar -C /media/Angstrom -xjvf $Beagleboard/binaries/filesystem/Angstrom-Beagleboard-demo-image-glibc-ipk-2011.1-beagleboard.rootfs.tar.bz2
-cp $Beagleboard/binaries/boot/MLO /media/boot/
-cp $Beagleboard/binaries/boot/u-boot.bin /media/boot/
-cp $Beagleboard/binaries/boot/uImage /media/boot/
-sync ; sudo umount /media/*

Use mplayer to observe preview:
-----------------------------------
Output preview on display-framebuffer (fbdev) using mplayer
- mplayer tv:// -tv driver=v4l2:width=640:height=480:device=/dev/video0:fps=30 -vo fbdev

Use mplayer to take snapshots:
-----------------------------------
save images as jpeg in current directory
- mplayer tv:// -tv driver=v4l2:width=640:height=480:device=/dev/video0:fps=30 -vo jpeg


