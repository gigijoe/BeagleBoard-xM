/**
README: Build instructions to get Angstrom Filesystem up and running on Beagleboard-xM
*/

Building u-boot and MLO:
-------------------------
Refer to "http://www.angstrom-distribution.org/demo/beagleboard/" for getting pre-built binaries. 

Building Filesystem for Angstrom:
--------------------
Refer to "http://www.angstrom-distribution.org/demo/beagleboard/" for getting latest build of filesystem 
(say, "Angstrom-Beagleboard-demo-image-glibc-ipk-2011.1-beagleboard.rootfs.tar.bz2).   

Compiler:
---------
using $Beagleboard/trunk/tools/arm-2009q1

Building kernel:
----------------
-cd $Beagleboard/trunk/kernel-2.6.32
-make ARCH=arm omap3_beagle_cam_defconfig
-export PATH=$PATH:<absolute path>/arm-2009q1/bin
-make ARCH=arm CROSS_COMPILE=arm-none-linux-gnueabi- uImage

Preparing sd-card for boot:
---------------------------
Format & partition sd-card at /dev/sda using mkcard.sh.  
-sudo $Beagleboard/binaries/mkcard.sh /dev/sda    

It creates two partitions /dev/sda1 and /dev/sda2, FAT32 and ext2 respectively

Mount /dev/sda, copy files to both partitions[/media/boot and /media/Angstrom] as follows: 

-sudo tar -C /media/Angstrom -xjvf $Beagleboard/binaries/filesystem/Angstrom-Beagleboard-demo-image-glibc-ipk-2011.1-beagleboard.rootfs.tar.bz2
-cp $Beagleboard/binaries/boot/MLO /media/boot/
-cp $Beagleboard/binaries/boot/u-boot.bin /media/boot/
-cp $Beagleboard/binaries/boot/uImage /media/boot/
-sync ; sudo umount /media/*

Set Environment Variables (optional):
--------------------------
As Nand is absent in Beagleboard-xM Rev.B, One has to make its own "boot.scr" to change environment variables. By default, it 
uses default environment while boot up.

use boot_cmd file for generating boot.scr
	Example of a boot_cmd file:
		mmc init
		setenv console tty0 console=ttyS2,115200n8
		setenv bootargs 'console=ttyS2,115200n8 mpurate=800 mem=99M@0x80000000 mem=384M@0x88000000 camera=lbcm5m1 vram=12M omapfb.mode=dvi:1280x720MR-16@60 omapdss.def_disp=dvi root=/dev/mmcblk0p2 rootfstype=ext3 rootwait'
		fatload mmc 1 0x82000000 uImage
		fatload mmc 1 0x88000000 rootfs.ext2
		bootm 0x82000000

Command for making boot.scr:
-mkimage -A arm -O linux -T script -C none -a 0 -e 0 -n "Beagleboard-xM boot script" -d boot_cmd boot.scr

