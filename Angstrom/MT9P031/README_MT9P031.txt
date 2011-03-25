/**
README: Readme to customize driver(mt9p031.c)
*/

Todo: Test Framerates, to test Exposure Max and Min

MACROS:
---------
MT9P031_DEBUG: 
-Enables Sysfs interface for debug
-Enables DPRINTK_DRIVER(..) for debug 

Mt9P031_HEADBOARD:
-Define while using mt9P031-headboards from Aptina (no need for Leopard imaging boards)
 It enables call to function "mt9p031_config_PCA9543A()" which configures I2c level shifter "PCA9543A" on headboard for channel 0.

Supported Frame Sizes:
----------------------
VGA: { 640, 480 },
720P: { 1280, 720 },
1080P: { 1920, 1080 },
3MP: { 2048, 1536 },
5MP: { 2592, 1944 }

Supported FPS:
--------------
10, 30 and 60

Supported Gain:
----------------
upto 8X in steps of 0.125. 


Use mplayer for preview: (choose appropriate sizes)
-----------------------------------
Output preview on display-framebuffer (fbdev) using mplayer
- mplayer tv:// -tv driver=v4l2:width=640:height=480:device=/dev/video0:fps=30 -vo fbdev

Use mplayer for snapshots: (choose appropriate sizes)
-----------------------------------
save images as jpeg in current directory
- mplayer tv:// -tv driver=v4l2:width=640:height=480:device=/dev/video0:fps=30 -vo jpeg



-----------------------------------------------------------------------
                             SYSFS
-----------------------------------------------------------------------

For Using Sysfs On beagle do following:
		-cd /sys/devices/platform/i2c_omap.2/i2c-2/2-0048/

Adjusting exposure time(in ms)[Range 15000-128000] using sysfs
-----------------------------------------------------------------------
	for writing exposure time to 33000
		-echo 33000 > exposure_val
	
	for reading exposure time
		-cat exposure_val
		
Adjusting Global(Analog) gain indexwise, incremental basis 0.5 [Range 0-14] using sysfs
---------------------------------------------------------------------------
	for writing gain to index 4
		-echo 4 > gain_val
	
	for reading exposure time
		-cat gain_val
		
For more details, refer to Gain index-table, MT9P031_EV_GAIN_TBL[] available in driver: linux/driver/media/video/mt9p031.c		

Basic Register Read/Write
-------------------------------------------------------------------------
	for reading Register 0x07:
		-echo 0x07 > basic_reg_addr
		-cat basic_reg_val

	for writing 0x4 to Register 0x08:
		-echo 0x08 > basic_reg_addr
		-echo 0x04 > basic_reg_val
		
		

Change individual gain and Integration time registers in real-time using sysfs:
----------------------------------------------------------------------------------
Registers                    Register-Address    Typical values
----------------------------------------------------------------------------------
Green1 Gain		        0x2B				0x11-0x51

Blue Gain			0x2C				0x11-0x51

Red Gain			0x2D				0x11-0x51

Green2 Gain			0x2E				0x11-0x51

Shutter_Width_Low		0x09				0x400-0x1000

