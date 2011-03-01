/*
 * mt9p031 Camera Driver
 *
 * Based on ov772x driver, mt9m111 driver,
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/videodev2.h>
#include <linux/sysfs.h>

#include <media/mt9p031.h>
#include <media/v4l2-int-device.h>
#include <media/v4l2-chip-ident.h>

/************************************************************************


			macro


************************************************************************/


/*
 * frame size
 */

#define MAX_WIDTH_PREV   1024
#define MAX_HEIGHT_PREV  768



/* we use this number to decide if we can increase the exposure, move from skipping to binning, etc.
   the constraint is that this is the minimal frame rate that can still be considered smooth video.
*/
#define MIN_VIDEO_FPS 21


/*
 * frame size
 */
#define MT9P031_MAX_HEIGHT		1944
#define MT9P031_MAX_WIDTH		2592
#define MT9P031_MIN_HEIGHT		2
#define MT9P031_MIN_WIDTH		2

#define QVGA_HEIGHT  			240
#define QVGA_WIDTH   			320
#define VGA_HEIGHT  			480
#define VGA_WIDTH   			640


#define REG_MT9P031_CHIP_VERSION                0x00
#define REG_MT9P031_ROWSTART			0x01
#define REG_MT9P031_COLSTART			0x02
#define REG_MT9P031_HEIGHT			0x03
#define REG_MT9P031_WIDTH			0x04
#define REG_MT9P031_HBLANK			0x05
#define REG_MT9P031_VBLANK			0x06
#define REG_MT9P031_OUT_CTRL			0x07
#define REG_MT9P031_SHUTTER_WIDTH_U		0x08
#define REG_MT9P031_SHUTTER_WIDTH_L		0x09
#define REG_MT9P031_PCLK_CTRL			0x0a
#define REG_MT9P031_RESTART			0x0b
#define REG_MT9P031_SHUTTER_DELAY		0x0c
#define REG_MT9P031_RESET			0x0d

#define REG_MT9P031_PLL_CTRL			0x10
#define REG_MT9P031_PLL_CONF1			0x11
#define REG_MT9P031_PLL_CONF2			0x12

#define REG_MT9P031_READ_MODE1			0x1e
#define REG_MT9P031_READ_MODE2			0x20
#define REG_MT9P031_ROW_ADDR_MODE		0x22
#define REG_MT9P031_COL_ADDR_MODE		0x23
#define REG_MT9P031_GREEN_1_GAIN		0x2b
#define REG_MT9P031_BLUE_GAIN			0x2c
#define REG_MT9P031_RED_GAIN			0x2d
#define REG_MT9P031_GREEN_2_GAIN		0x2e
#define REG_MT9P031_GLOBAL_GAIN			0x35

#define REG_MT9P031_BLC_SAMPLE_SIZE		0x5b
#define REG_MT9P031_BLC_TUNE1			0x5c
#define REG_MT9P031_BLC_DELTA_TRSHLD		0x5d
#define REG_MT9P031_BLC_TUNE2			0x5e
#define REG_MT9P031_BLC_TARGET_TRSHLD		0x5f
#define REG_MT9P031_GREEN_OFFS1			0x60
#define REG_MT9P031_GREEN_OFFS2			0x61
#define REG_MT9P031_BLACK_LEVEL_CALIB		0x62
#define REG_MT9P031_RED_OFFS			0x63
#define REG_MT9P031_BLUE_OFFS			0x64
#define REG_MT9P031_SELF_TEST			0xa0
#define REG_MT9P031_TEST_GREEN			0xa1
#define REG_MT9P031_TEST_BLUE			0xa2
#define REG_MT9P031_TEST_RED			0xa3
#define REG_MT9P031_TEST_WIDTH			0xa4
#define REG_MT9P031_CHIP_VERSION_ALT            0x0FF

#define MT9P031_NORMAL_OPERATION_MODE           (0x1F82) //write
#define MT9P031_OUTPUT_CTRL_CHIP_UNSELECT       (0x1F80) //write
#define MT9P031_OUTPUT_CTRL_HALT                (0x1F83)
#define ADDRESS_MODE_MASK			(0x0037)
#define MT9P031_SHUTTER_WIDTH_UPPER_SHIFT	(16)
#define REG_MT9P031_BLK_DEF_OFFSET		(0x4B)


/*
 * macro of read/write
 */
#define ECHECKER(ret, x)		\
	do {				\
		(ret) = (x);		\
		if ((ret) < 0)		\
			return (ret);	\
	} while (0)

#if 0
#define mt9p031_reg_write(ret, client, a, b) \
	ECHECKER(ret, __mt9p031_reg_write(client, a, b))

#define mt9p031_reg_read(ret, client, a) \
	ECHECKER(ret, __mt9p031_reg_read(client, a))
#endif

/*
 * Logical address
 */
#define _VAR(id, offset, base)	(base | (id & 0x1f) << 10 | (offset & 0x3ff))
#define VAR(id, offset)  _VAR(id, offset, 0x0000)
#define VAR8(id, offset) _VAR(id, offset, 0x8000)

/************************************************************************


			struct


************************************************************************/
struct mt9p031_frame_size {
	u16 width;
	u16 height;
};

struct mt9p031_priv {
	struct mt9p031_platform_data	*pdata;
	struct v4l2_int_device		*v4l2_int_device;
	struct mt9p031_camera_info	info;
	struct i2c_client		*client;
	struct v4l2_pix_format		 pix;
	int				 model;
	u32				 flags;
/* for flags */
#define INIT_DONE  (1<<0)
};

struct mt9p031_priv sysPriv;
/************************************************************************


			supported sizes


************************************************************************/
const static struct mt9p031_frame_size mt9p031_sizes[] = {
	
	{ 1280, 720 },
#if 0 //Vivek
	{ 1024, 768 },
	{ 1280, 1024 },
	{ 1280, 720 },
	{ 2592, 1944 } 
#endif
};

const struct v4l2_fract mt9p031_frameintervals[] = {
	{  .numerator = 1, .denominator = 30 },	
	
};

static int mt9p031_config_PCA9543A(const struct i2c_client *client)
{
	struct i2c_msg msg;
	int ret;
	u8 buf;
	buf = 0x21;
	
	msg.addr  = (0xE6 >> 1);	//slave address of PCA9543A
	msg.flags = 0;
	msg.len   = 1;
	msg.buf   = &buf;
	
	ret = i2c_transfer(client->adapter, &msg, 1);
	printk(KERN_INFO "mt9p031_level_shifter_config done\n");
	
	return 0;
		
}

static int mt9p031_reg_read(const struct i2c_client *client, u16 command)
{
	struct i2c_msg msg[2];
	u8 buf[2];
	int ret;

	// 8-bit/ byte addressable register
	buf[0] = command & 0xff;

	msg[0].addr  = client->addr;
	msg[0].flags = 0;
	msg[0].len   = 1;
	msg[0].buf   = buf ;
	ret = i2c_transfer(client->adapter, &msg[0], 1);
	//printk(KERN_INFO "mt9p031_reg_read 0x%x 0x%x 0x%x\n", msg[0].buf[0], ret, command);
	
	msg[1].addr  = client->addr;
	msg[1].flags = I2C_M_RD; //1
	msg[1].len   = 2;
	msg[1].buf   = buf;

	/*
	 * if return value of this function is < 0,
	 * it mean error.
	 * else, under 16bit is valid data.
	 */
	ret = i2c_transfer(client->adapter, &msg[1], 1);
	//printk(KERN_INFO "mt9p031_reg_read Value 0x%x %d\n", ret, ret);
	if (ret < 0)
		return ret;

	memcpy(&ret, buf, 2);
	//printk(KERN_INFO "mt9p031_reg_read Value 0x%x 0x%x\n", swab16(ret), ret);
	return swab16(ret);
}

static int mt9p031_reg_write(const struct i2c_client *client,
			       u16 command, u16 data)
{
	struct i2c_msg msg;
	u8 buf[3];
	int ret;

	// 8-bit/ byte addressable register
		
	buf[0] = command & 0xff;
	data = swab16(data);
	memcpy(buf + 1, &data,    2);

	msg.addr  = client->addr;
	msg.flags = 0;
	msg.len   = 3;
	msg.buf   = buf;
		
	/*
	 * i2c_transfer return message length,
	 * but this function should return 0 if correct case
	 */
	ret = i2c_transfer(client->adapter, &msg, 1);
	//printk(KERN_INFO "mt9p031_reg_write 0x%x%x%x %d\n", buf[0], buf[1], buf[2], ret);
	if (ret >= 0)
		ret = 0;

	return ret;
}

static int mt9p031_reg_set(struct i2c_client *client, const u8 reg, const u16 data)
{
    int ret = mt9p031_reg_read(client, reg);
    if (ret<  0)
        return ret;
    return mt9p031_reg_write(client, reg, ret | data);
}

static int mt9p031_reg_clear(struct i2c_client *client, const u8 reg,
                     const u16 data)
{
    int ret = mt9p031_reg_read(client, reg);
    if (ret<  0)
        return ret;
    return mt9p031_reg_write(client, reg, ret&  ~data);
}



static int mt9p031_init_camera(const struct i2c_client *client)
{
	int ret;

	printk(KERN_INFO "mt9p031_init_camera Enter\n");
	
	ret = mt9p031_reg_write(client, REG_MT9P031_RESET, 0x0001);	//High
	//mdelay(100);

	ret |= mt9p031_reg_write(client, REG_MT9P031_RESET, 0x0000);	//Low
	mdelay(100);

	ret |= mt9p031_reg_write(client, 0x08, 0x0000);  	//Shutter_width_U
	ret |= mt9p031_reg_write(client, 0x09, 0x00E6);  	//shutter_width_L
	ret |= mt9p031_reg_write(client, 0x0C, 0x0613);  	//shutter_delay_reg
	ret |= mt9p031_reg_write(client, 0x2B, 0x0051);  	//Green1_gain_reg
	ret |= mt9p031_reg_write(client, 0x2C, 0x0058);  	//Blue_gain_reg
	ret |= mt9p031_reg_write(client, 0x2D, 0x0053);  	//Red_gain_reg
	ret |= mt9p031_reg_write(client, 0x2E, 0x0051);  	//Green2_gain_reg
	
	ret |= mt9p031_reg_write(client, 0x1E, 0x0006);  	//Read_mode_1 //disable AB
	
	ret |= mt9p031_reg_write(client, 0x05, 0x0600);  	//Horz_blznk_reg = 1536
	
	ret |= mt9p031_reg_write(client, 0x10, 0x0051);  	//PLL_CTRL; power up pll
	ret |= mt9p031_reg_write(client, 0x11, 0x1801);		//PLL_CONFIG_1: m=24, n=1
	ret |= mt9p031_reg_write(client, 0x12, 0x0002);		//PLL_CONFIG_2: p1=2, p2=0
	mdelay(10);  										//wait 1 ms for VCO to lock
	ret |= mt9p031_reg_write(client, 0x10, 0x0053);		//PLL_CONTROL; use PLL
	mdelay(200);
	
	ret |= mt9p031_reg_write(client, 0x05, 0x01C2);  	//Horz_blank_reg
	ret |= mt9p031_reg_write(client, 0x07, 0x1F8E);		//Enable parll fifo data
	
	ret |= mt9p031_reg_write(client, 0x09, 0x1000);	//Shutter width lower; integration time register

	printk(KERN_INFO "mt9p031_init_camera Everything Done %d\n", ret);

	return ret>= 0 ? 0 : -EIO;
}


/**
 * mt9p031_v4l2_int_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 */


static int mt9p031_v4l2_int_s_parm(struct v4l2_int_device *s,
				   struct v4l2_streamparm *a)
{
	printk(KERN_INFO "mt9p031_v4l2_int_s_parm \n");
	/* TODO: set paramters */
	return 0;
}

static int mt9p031_v4l2_int_g_parm(struct v4l2_int_device *s,
				   struct v4l2_streamparm *a)
{
	struct v4l2_captureparm *cparm = &a->parm.capture;

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	memset(a, 0, sizeof(*a));
	a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	cparm->capability = V4L2_CAP_TIMEPERFRAME;
	/* FIXME: Is 30 fps really the only option? */
	cparm->timeperframe.numerator = 1;
	cparm->timeperframe.denominator = 30;

	return 0;
}


static const struct v4l2_fmtdesc mt9p031_formats[] =
{
    	{
       		.description = "Bayer (sRGB) 10 bit",
		.pixelformat = V4L2_PIX_FMT_SRGGB10,
	},
};

static const unsigned int mt9p031_num_formats = ARRAY_SIZE(mt9p031_formats);

#if 0
static struct v4l2_queryctrl mt9p031_qctrl[] = {
    {
        .id = V4L2_CID_EXPOSURE,
        .type = V4L2_CTRL_TYPE_INTEGER,
        .name = "Exposure",
        .minimum = 0,
        .maximum = (1<<  10) - 1,
        .step = 1,
        .default_value = 0x0020,
        .flags = 0,
    },
    {
        .id = V4L2_CID_GAIN,
        .type = V4L2_CTRL_TYPE_INTEGER,
        .name = "Gain",
        .minimum = 0,
        .maximum = (1<<  10) - 1,
        .step = 1,
        .default_value = 0x0020,
        .flags = 0,
    },
    {
        .id = V4L2_CID_RED_BALANCE,
        .type = V4L2_CTRL_TYPE_INTEGER,
        .name = "Red Balance",
        .minimum = -1<<  9,
        .maximum = (1<<  9) - 1,
        .step = 1,
        .default_value = 0,
        .flags = 0,
    },
    {
        .id = V4L2_CID_BLUE_BALANCE,
        .type = V4L2_CTRL_TYPE_INTEGER,
        .name = "Blue Balance",
        .minimum = -1<<  9,
        .maximum = (1<<  9) - 1,
        .step = 1,
        .default_value = 0,
        .flags = 0,
    },
    {
        .id = CTRL_R_GAIN,
        .type = V4L2_CTRL_TYPE_INTEGER,
        .name = "red_gain",
        .minimum = 0,
        .maximum = (1<<  10) - 1,
        .step = 1,
        .default_value = 0x0020,
        .flags = 0,
    },
    {
        .id = CTRL_B_GAIN,
        .type = V4L2_CTRL_TYPE_INTEGER,
        .name = "blue_gain",
        .minimum = 0,
        .maximum = (1<<  10) - 1,
        .step = 1,
        .default_value = 0x0020,
        .flags = 0,
    },
    {
        .id = CTRL_G_1_GAIN,
        .type = V4L2_CTRL_TYPE_INTEGER,
        .name = "green_1_gain",
        .minimum = -1<<  9,
        .maximum = (1<<  9) - 1,
        .step = 1,
        .default_value = 0,
        .flags = 0,
    },
    {
        .id = CTRL_G_2_GAIN,
        .type = V4L2_CTRL_TYPE_INTEGER,
        .name = "green_2_gain",
        .minimum = -1<<  9,
        .maximum = (1<<  9) - 1,
        .step = 1,
        .default_value = 0,
        .flags = 0,
    },
    {
        .id = CTRL_R_OFFSET,
        .type = V4L2_CTRL_TYPE_INTEGER,
        .name = "ctrl_r_offset",
        .minimum = -1<<  9,
        .maximum = (1<<  9) - 1,
        .step = 1,
        .default_value = 0,
        .flags = 0,
    },
    {
        .id = CTRL_B_OFFSET,
        .type = V4L2_CTRL_TYPE_INTEGER,
        .name = "ctrl_b_offset",
        .minimum = -1<<  9,
        .maximum = (1<<  9) - 1,
        .step = 1,
        .default_value = 0,
        .flags = 0,
    },
    {
        .id = CTRL_G1_OFFSET,
        .type = V4L2_CTRL_TYPE_INTEGER,
        .name = "green_g1_gain",
        .minimum = -1<<  9,
        .maximum = (1<<  9) - 1,
        .step = 1,
        .default_value = 0,
        .flags = 0,
    },
    {
        .id = CTRL_G2_OFFSET,
        .type = V4L2_CTRL_TYPE_INTEGER,
        .name = "green_g2_gain",
        .minimum = -1<<  9,
        .maximum = (1<<  9) - 1,
        .step = 1,
        .default_value = 0,
        .flags = 0,
    }
};

#endif	
/************************************************************************


			i2c driver


************************************************************************/

static int mt9p031_detect(struct i2c_client *client)
{
	struct mt9p031_priv *priv = i2c_get_clientdata(client);
	const char          *devname;
	int                  chipid, ret;
	
	printk(KERN_INFO "mt9p031_detect Enter\n");
	/*
	 * Set Normal Mode
	 */
	ret = mt9p031_reg_write(client, REG_MT9P031_OUT_CTRL, MT9P031_NORMAL_OPERATION_MODE);
	
	printk(KERN_INFO "Chip Select Done\n");
	/*
	 * check and show chip ID
	 */
	
	chipid = mt9p031_reg_read(client, REG_MT9P031_CHIP_VERSION);
	printk(KERN_INFO "Chip ID Read 0x%x\n", chipid);
	switch (chipid) {
	case 0x1801:
		devname = "mt9p031";
		priv->model = V4L2_IDENT_MT9P031;
		break;
	default:
		dev_err(&client->dev, "Product ID error %04x\n", chipid);
		return -ENODEV;
	}

	dev_info(&client->dev, "%s chip ID %04x\n", devname, chipid);

	return 0;
}

static int mt9p031_v4l2_int_s_power(struct v4l2_int_device *s,
				    enum v4l2_power power)
{
	struct mt9p031_priv *priv = s->priv;
	struct i2c_client *client = priv->client;
	u16 param = (MT9P031_FLAG_PCLK_RISING_EDGE &
		     priv->info.flags) ? 0x0001 : 0x0000;
	int ret;

	switch (power) {
	case V4L2_POWER_STANDBY:
		/* FALLTHROUGH */
	case V4L2_POWER_OFF:
		ret = priv->pdata->power_set(s, power);
		printk(KERN_INFO "v4l2_int_s_power %d\n", ret);
		if (ret < 0) {
			dev_err(&client->dev, "Unable to set target board power "
					 "state (OFF/STANDBY)\n");
			return ret;
		}
		break;
	case V4L2_POWER_ON:
		ret = priv->pdata->power_set(s, power);

		if (ret < 0) {
			dev_err(&client->dev, "Unable to set target board power "
					 "state (ON)\n");
			return ret;
		}
		if (!(priv->flags & INIT_DONE)) {
			ECHECKER(ret, mt9p031_detect(client));

			priv->flags |= INIT_DONE;
		}

		ECHECKER(ret, mt9p031_init_camera(client));

	}
	printk(KERN_INFO "v4l2_int_s_power exit\n");
	return 0;
}

#if 0
static int mt9p031_queryctrl(struct v4l2_int_device *s, struct v4l2_queryctrl *qc)
{
    int i;
    for (i = 0; i<  ARRAY_SIZE(mt9p031_qctrl); i++) {
        if (qc->id&&  qc->id == mt9p031_qctrl[i].id) {
            memcpy(qc,&(mt9p031_qctrl[i]), sizeof(*qc));
            return 0;
        }
    }
    return -EINVAL;
}
#endif

static int mt9p031_v4l2_int_g_priv(struct v4l2_int_device *s, void *p)
{
	struct mt9p031_priv *priv = s->priv;

	return priv->pdata->priv_data_set(p);
}

struct mt9p031_format_params {
	int width;
	int height;
	int col_size;
	int row_size;
	int hblank;
	int vblank;
	int shutter_width;
	int row_addr_mode;
	int col_addr_mode;
	int black_level;
	int pixel_clk_control;
	int row_start;
	int col_start;
	int read_mode_2_config;
	int output_ctrl;
	int pll_m;
	int pll_n;
	int pll_p1;
};

enum formats {
	//VGA_BIN_30FPS,
	SVGA_30FPS,
#if 0	
	XGA_30FPS,
	SXGA_30FPS,
	HD_READY_720P_30FPS,
	FULL_5MP_7FPS,
#endif
};

const struct mt9p031_format_params mt9p031_supported_formats[] =
	{
		{ 1280, 720, 0x09FF, 0x059F, 0x0000,  0x0000,  0x0400, 0x0011, 0x0011, 0, 0, 0x0040, 0x0018, 0x0060, 0, 29, 4, },  // SVGA_30FPS
#if 0 //vivekk..test
		{1024, 768, 2047, 1535,    0, 8, 767, 0x11, 0x11, 64, 0x8000,  0,  0, 64, 0x1F82, 0xFF, 29, 4, },  // XGA_30FPS
		{1280,1024, 2575, 1951,  776,10, 744, 0x11, 0x11, 64, 0x8000, 32, 24, 32, 0x1F8E, 0x1C,  4, 2, },  // SXGA_30FPS
		{1280, 720, 2583, 1491,  786, 0, 740, 0x11, 0x11, 64, 0x8000,272, 24, 32, 0x1F8E,   16,  3, 2, },
		{2592,1944, 2591, 1943,  450, 9,1942, 0x00, 0x00, 64, 0x8000, 66, 32, 64, 0x1F82, 12,  4, 2, },  // 5MP_7FPS
#endif		
	};


static int mt9p031_set_params(struct i2c_client *client, u32 width, u32 height,
			      u32 pixelformat)
{
	struct mt9p031_priv *priv = i2c_get_clientdata(client);
	int ret;
	enum formats i;

	/*
	 * frame size check
	 */
	//mt9p031_frame_check(&width, &height);

	/*
	 * get color format
	 */
	
	priv->pix.width  = (u16)width;
	priv->pix.height = (u16)height;

	priv->pix.pixelformat = pixelformat;

	priv->pix.width = 1280;
	priv->pix.height = 720;
	
	ret = mt9p031_reg_write(client, 0x01, 0x0040);		//ROW_WINDOW_START_REG
	ret |= mt9p031_reg_write(client, 0x02, 0x0018);		//COL_WINDOW_START_REG
	ret |= mt9p031_reg_write(client, 0x03, 0x059F);		//ROW_WINDOW_SIZE_REG=1439
	ret |= mt9p031_reg_write(client, 0x04, 0x09FF);		//COL_WINDOW_SIZE_REG=2559
	ret |= mt9p031_reg_write(client, 0x05, 0x0000);		//HORZ_BLANK=0
	ret |= mt9p031_reg_write(client, 0x06, 0x0000);		//VERT_BLANK_REG=720
	ret |= mt9p031_reg_write(client, 0x09, 0x0400);		//SHUTTER_WIDTH_LOW (INTEG_TIME_REG = 1024)
	ret |= mt9p031_reg_write(client, 0x22, 0x0011);		//ROW_MODE, ROW_SKIP=1, ROW_BIN=1	
	ret |= mt9p031_reg_write(client, 0x23, 0x0011);		//COL_MODE, COL_SKIP=1, COL_BIN=1
	ret |= mt9p031_reg_write(client, 0x20, 0x0060);		//READ_MODE_2, COL_SUM
	ret |= mt9p031_reg_write(client, 0x08, 0x0000);		//SHUTTER_WIDTH_HI
	ret |= mt9p031_reg_write(client, 0x09, 0x1000);		//SHUTTER_WIDTH_LOW (INTEG_TIME_REG)
	ret |= mt9p031_reg_write(client, 0x0C, 0x0000);		//SHUTTER_DELAY_REG
	
	//Sub-Sample optimization
	ret = mt9p031_reg_write(client, 0x70, 0x5C);	
	ret |= mt9p031_reg_write(client, 0x71, 0x5B00);
	ret |= mt9p031_reg_write(client, 0x72, 0x5900);
	ret |= mt9p031_reg_write(client, 0x73, 0x200);
	ret |= mt9p031_reg_write(client, 0x74, 0x200);
	ret |= mt9p031_reg_write(client, 0x75, 0x2800);
	ret |= mt9p031_reg_write(client, 0x76, 0x3E29);
	ret |= mt9p031_reg_write(client, 0x77, 0x3E29);
	ret |= mt9p031_reg_write(client, 0x78, 0x583F);
	ret |= mt9p031_reg_write(client, 0x79, 0x5B00);
	ret |= mt9p031_reg_write(client, 0x7a, 0x5A00);
	ret |= mt9p031_reg_write(client, 0x7b, 0x5900);
	ret |= mt9p031_reg_write(client, 0x7c, 0x5900);
	ret |= mt9p031_reg_write(client, 0x7e, 0x5900);
	ret |= mt9p031_reg_write(client, 0x7f, 0x5900);
	ret |= mt9p031_reg_write(client, 0x6, 0x0);
	ret |= mt9p031_reg_write(client, 0x29, 0x481);
	ret |= mt9p031_reg_write(client, 0x3e, 0x87);
	ret |= mt9p031_reg_write(client, 0x3f, 0x7);
	ret |= mt9p031_reg_write(client, 0x41, 0x3);
	ret |= mt9p031_reg_write(client, 0x48, 0x18);
	ret |= mt9p031_reg_write(client, 0x5f, 0x1C16);
	ret |= mt9p031_reg_write(client, 0x57, 0x7);
	ret |= mt9p031_reg_write(client, 0x2A, 0xFF74);
	
	return ret;
}

static int mt9p031_v4l2_int_cropcap(struct v4l2_int_device *s,
				    struct v4l2_cropcap *a)
{
	a->bounds.left			= 0;
	a->bounds.top			= 0;
	a->bounds.width			= VGA_WIDTH;
	a->bounds.height		= VGA_HEIGHT;
	a->defrect			= a->bounds;
	a->type				= V4L2_BUF_TYPE_VIDEO_CAPTURE;
	a->pixelaspect.numerator	= 1;
	a->pixelaspect.denominator	= 1;

	return 0;
}

static int mt9p031_v4l2_int_g_crop(struct v4l2_int_device *s,
				   struct v4l2_crop *a)
{
	a->c.left	= 0;
	a->c.top	= 0;
	a->c.width	= VGA_WIDTH;
	a->c.height	= VGA_HEIGHT;
	a->type		= V4L2_BUF_TYPE_VIDEO_CAPTURE;

	return 0;
}

static int mt9p031_v4l2_int_s_crop(struct v4l2_int_device *s,
				   struct v4l2_crop *a)
{
	if ((a->c.left != 0) ||
	    (a->c.top != 0) ||
	    (a->c.width != VGA_WIDTH) ||
	    (a->c.height != VGA_HEIGHT)) {
		return -EINVAL;
	}
	return 0;
}


static int mt9p031_v4l2_int_try_fmt_cap(struct v4l2_int_device *s,
					struct v4l2_format *f)
{
	//mt9p031_frame_check(&f->fmt.pix.width, &f->fmt.pix.height);

	/* TODO: set colorspace */
	f->fmt.pix.field = V4L2_FIELD_NONE;

	return 0;
}

static int mt9p031_v4l2_int_s_fmt_cap(struct v4l2_int_device *s,
				      struct v4l2_format *f)
{
	struct mt9p031_priv *priv = s->priv;
	struct i2c_client *client = priv->client;
	printk(KERN_INFO " v4l2_int_s_fmt_cap enter%d %d %d\n", f->fmt.pix.pixelformat, f->fmt.pix.width, f->fmt.pix.height);

	/* TODO: set colorspace */
	return mt9p031_set_params(client, f->fmt.pix.width, f->fmt.pix.height,
				  f->fmt.pix.pixelformat);
}

static int mt9p031_v4l2_int_enum_framesizes(struct v4l2_int_device *s,
					    struct v4l2_frmsizeenum *frms)
{
	int ifmt;

	for (ifmt = 0; ifmt < ARRAY_SIZE(mt9p031_formats); ifmt++)
		if (mt9p031_formats[ifmt].pixelformat == frms->pixel_format)
			break;

	if (ifmt == ARRAY_SIZE(mt9p031_formats))
		return -EINVAL;

	/* Do we already reached all discrete framesizes? */
	if (frms->index >= ARRAY_SIZE(mt9p031_sizes))
		return -EINVAL;

	frms->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	frms->discrete.width = mt9p031_sizes[frms->index].width;
	frms->discrete.height = mt9p031_sizes[frms->index].height;
	printk(KERN_INFO " v4l2_int_enum_framesizes end %d %d %d\n", frms->discrete.width, frms->discrete.height, frms->index);

	return 0;

}


/**
 * ioctl_enum_fmt_cap - Implement the CAPTURE buffer VIDIOC_ENUM_FMT ioctl
 * @s: pointer to standard V4L2 device structure
 * @fmt: standard V4L2 VIDIOC_ENUM_FMT ioctl structure
 *
 * Implement the VIDIOC_ENUM_FMT ioctl for the CAPTURE buffer type.
 */

static int mt9p031_v4l2_int_enum_fmt_cap(struct v4l2_int_device *s,
					 struct v4l2_fmtdesc *fmt)
{
	int index = fmt->index;
	enum v4l2_buf_type type = fmt->type;

	memset(fmt, 0, sizeof(*fmt));
	fmt->index = index;
	fmt->type = type;

	switch (fmt->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		if (index >= ARRAY_SIZE(mt9p031_formats))
			return -EINVAL;
	break;
	default:
		return -EINVAL;
	}

	//fmt->flags = mt9p031_formats[index].flags;
	strlcpy(fmt->description, mt9p031_formats[index].description,
					sizeof(fmt->description));
	fmt->pixelformat = mt9p031_formats[index].pixelformat;

	return 0;
}

static int mt9p031_v4l2_int_g_ifparm(struct v4l2_int_device *s,
				     struct v4l2_ifparm *p)
{
	struct mt9p031_priv *priv = s->priv;
	int rval;

	if (p == NULL)
		return -EINVAL;

	if (!priv->pdata->ifparm)
		return -EINVAL;

	rval = priv->pdata->ifparm(p);
	if (rval) {
		v4l_err(priv->client, "g_ifparm.Err[%d]\n", rval);
		return rval;
	}

	p->u.ycbcr.clock_curr = 40 * 1000000; /* temporal value */

	return 0;
}


static int mt9p031_v4l2_int_g_fmt_cap(struct v4l2_int_device *s,
				      struct v4l2_format *f)
{

	struct mt9p031_priv *priv = s->priv;
	struct i2c_client *client = priv->client;
	
	printk(KERN_INFO " v4l2_int_g_fmt_cap enter%d %d %d\n", priv->pix.pixelformat, priv->pix.width, priv->pix.height);	

	f->fmt.pix.width	= priv->pix.width;
	f->fmt.pix.height	= priv->pix.height;
	/* TODO: set colorspace */
	f->fmt.pix.pixelformat	= V4L2_COLORSPACE_SRGB;
	f->fmt.pix.pixelformat	= priv->pix.pixelformat;
	f->fmt.pix.field	= V4L2_FIELD_NONE;

	return 0;
}

static int mt9p031_v4l2_int_enum_frameintervals(struct v4l2_int_device *s,
						struct v4l2_frmivalenum *frmi)
{
	int ifmt;

	for (ifmt = 0; ifmt < ARRAY_SIZE(mt9p031_formats); ifmt++)
		if (mt9p031_formats[ifmt].pixelformat == frmi->pixel_format)
			break;

	if (ifmt == ARRAY_SIZE(mt9p031_formats))
		return -EINVAL;

	if (frmi->index >= ARRAY_SIZE(mt9p031_frameintervals))
		return -EINVAL;

	frmi->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	frmi->discrete.numerator =
				mt9p031_frameintervals[frmi->index].numerator;
	frmi->discrete.denominator =
				mt9p031_frameintervals[frmi->index].denominator;
	return 0;
}


static struct v4l2_int_ioctl_desc mt9p031_ioctl_desc[] = {
	{ .num = vidioc_int_enum_framesizes_num,
	  .func = (v4l2_int_ioctl_func *)mt9p031_v4l2_int_enum_framesizes },
	{ .num = vidioc_int_enum_frameintervals_num,
	  .func = (v4l2_int_ioctl_func *)mt9p031_v4l2_int_enum_frameintervals },
	{ .num = vidioc_int_s_power_num,
	  .func = (v4l2_int_ioctl_func *)mt9p031_v4l2_int_s_power },
	{ .num = vidioc_int_g_priv_num,
	  .func = (v4l2_int_ioctl_func *)mt9p031_v4l2_int_g_priv },
	{ .num = vidioc_int_g_ifparm_num,
	  .func = (v4l2_int_ioctl_func *)mt9p031_v4l2_int_g_ifparm },
	{ .num = vidioc_int_enum_fmt_cap_num,
	  .func = (v4l2_int_ioctl_func *)mt9p031_v4l2_int_enum_fmt_cap },
	{ .num = vidioc_int_try_fmt_cap_num,
	  .func = (v4l2_int_ioctl_func *)mt9p031_v4l2_int_try_fmt_cap },
	{ .num = vidioc_int_g_fmt_cap_num,
	  .func = (v4l2_int_ioctl_func *)mt9p031_v4l2_int_g_fmt_cap }, 
	{ .num = vidioc_int_s_fmt_cap_num,
	  .func = (v4l2_int_ioctl_func *)mt9p031_v4l2_int_s_fmt_cap },
	{ .num = vidioc_int_g_parm_num,
	  .func = (v4l2_int_ioctl_func *)mt9p031_v4l2_int_g_parm },
	{ .num = vidioc_int_s_parm_num,
	  .func = (v4l2_int_ioctl_func *)mt9p031_v4l2_int_s_parm },
#if 0
	{ .num = vidioc_int_cropcap_num,
	  .func = (v4l2_int_ioctl_func *)mt9p031_v4l2_int_cropcap },
	{ .num = vidioc_int_g_crop_num,
	  .func = (v4l2_int_ioctl_func *)mt9p031_v4l2_int_g_crop },
	{ .num = vidioc_int_s_crop_num,
	  .func = (v4l2_int_ioctl_func *)mt9p031_v4l2_int_s_crop },
#endif
};

/**
 * ---------------------------------------------------------------------------------
 * Sysfs
 * ---------------------------------------------------------------------------------
 */

/* Basic register read write support */
static u16 mt9p031_attr_basic_addr  = 0x0000;

static ssize_t
mt9p031_basic_reg_addr_show( struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "0x%x\n", mt9p031_attr_basic_addr);
}

static ssize_t
mt9p031_basic_reg_addr_store( struct device *dev, struct device_attribute *attr, const char *buf, size_t n)
{
    u16 val;
    sscanf(buf, "%x", &val);
    mt9p031_attr_basic_addr = (u16) val;
    return n;
}

static DEVICE_ATTR( basic_reg_addr, S_IRUGO|S_IWUSR, mt9p031_basic_reg_addr_show, mt9p031_basic_reg_addr_store);


static ssize_t
mt9p031_basic_reg_val_show( struct device *dev, struct device_attribute *attr, char *buf)
{
    u16 val;
    val = mt9p031_reg_read(sysPriv.client, mt9p031_attr_basic_addr);
    if(val < 0){        
	printk(KERN_INFO "mt9p031: Basic register read failed");
        return 0; // nothing processed
    } else {
	return sprintf(buf, "0x%x\n", val);
    }
}

static ssize_t
mt9p031_basic_reg_val_store( struct device *dev, struct device_attribute *attr, const char *buf, size_t n)
{
    u32 val;
    sscanf(buf, "%x", &val);

    if (mt9p031_reg_write(sysPriv.client, mt9p031_attr_basic_addr, (u16)val)) {
        printk(KERN_INFO "mt9p031: Basic regiser write failed");
        return 0; // nothing processed
    } else {
        return n;
    }
}
static DEVICE_ATTR( basic_reg_val, S_IRUGO|S_IWUSR, mt9p031_basic_reg_val_show, mt9p031_basic_reg_val_store);

static struct attribute *mt9p031_sysfs_attr[] = {
    &dev_attr_basic_reg_addr.attr,
    &dev_attr_basic_reg_val.attr,
};

static int mt9p031_sysfs_add(struct kobject *kobj)
{
    int i = ARRAY_SIZE(mt9p031_sysfs_attr);
    int rval = 0;

    do {
        rval = sysfs_create_file(kobj, mt9p031_sysfs_attr[--i]);
    } while((i > 0) && (rval == 0));
    return rval;
}

static int mt9p031_sysfs_rm(struct kobject *kobj)
{
    int i = ARRAY_SIZE(mt9p031_sysfs_attr);
    int rval = 0;

    do {
        sysfs_remove_file(kobj, mt9p031_sysfs_attr[--i]);
    } while(i > 0);
    return rval;
}



static struct v4l2_int_slave mt9p031_slave = {
	.ioctls = mt9p031_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(mt9p031_ioctl_desc),
};

static int mt9p031_probe(struct i2c_client *client,
			 const struct i2c_device_id *did)
{
	struct mt9p031_priv        *priv;
	struct v4l2_int_device     *v4l2_int_device;
	int                         ret;

	if (!client->dev.platform_data) {
		dev_err(&client->dev, "no platform data?\n");
		return -ENODEV;
	}

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	v4l2_int_device = kzalloc(sizeof(*v4l2_int_device), GFP_KERNEL);
	if (!v4l2_int_device) {
		kfree(priv);
		return -ENOMEM;
	}

	mt9p031_config_PCA9543A(client);
	mdelay(100);
	v4l2_int_device->module = THIS_MODULE;
	strncpy(v4l2_int_device->name, "mt9p031", sizeof(v4l2_int_device->name));
	
	v4l2_int_device->type = v4l2_int_type_slave;
	v4l2_int_device->u.slave = &mt9p031_slave;

	v4l2_int_device->priv = priv;

	priv->v4l2_int_device = v4l2_int_device;
	priv->client = client;
	priv->pdata = client->dev.platform_data;
	
	/* Revisit: Init Sensor info settings */
	priv->info.divider.m = 24;
	priv->info.divider.n = 1;
	priv->info.divider.p1 = 0;
	priv->info.divider.p2 = 8;
	priv->info.divider.p3 = 0;
	priv->info.divider.p4 = 11;
	priv->info.divider.p5 = 11;
	priv->info.divider.p6 = 8;
	priv->info.divider.p7 = 0;
	priv->info.flags = MT9P031_FLAG_PCLK_RISING_EDGE;

	i2c_set_clientdata(client, priv);

	/* Setting Pixel Values */
	priv->pix.width       = mt9p031_sizes[0].width;
        priv->pix.height      = mt9p031_sizes[0].height;
        priv->pix.pixelformat = mt9p031_formats[0].pixelformat;
	
#if 0
    	/* Default Crop capabilities */
    	priv->croprect.left   = 0;
    	priv->croprect.top    = 0;
    	priv->croprect.width  = priv->sensor_info.max_width - 1;
    	priv->croprect.height = priv->sensor_info.max_height - 1;
#endif
	sysPriv.client = priv->client;

	ret = v4l2_int_device_register(priv->v4l2_int_device);
	if (ret) {
		i2c_set_clientdata(client, NULL);
		kfree(v4l2_int_device);
		kfree(priv);
	}

	mt9p031_sysfs_add(&client->dev.kobj);
	return ret;
}

static int mt9p031_remove(struct i2c_client *client)
{
	struct mt9p031_priv *priv = i2c_get_clientdata(client);

	v4l2_int_device_unregister(priv->v4l2_int_device);
	i2c_set_clientdata(client, NULL);
	mt9p031_sysfs_rm(&client->dev.kobj);
	
	kfree(priv->v4l2_int_device);
	kfree(priv);
	return 0;
}

static const struct i2c_device_id mt9p031_id[] = {
	{ "mt9p031", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mt9p031_id);

static struct i2c_driver mt9p031_i2c_driver = {
	.driver = {
		.name = "mt9p031",
	},
	.probe    = mt9p031_probe,
	.remove   = mt9p031_remove,
	.id_table = mt9p031_id,
};

/************************************************************************


			module function


************************************************************************/
static int __init mt9p031_module_init(void)
{
	return i2c_add_driver(&mt9p031_i2c_driver);
}

static void __exit mt9p031_module_exit(void)
{
	i2c_del_driver(&mt9p031_i2c_driver);
}

module_init(mt9p031_module_init);
module_exit(mt9p031_module_exit);

MODULE_DESCRIPTION("mt9p031 sensor driver");
MODULE_AUTHOR("Aptina");
MODULE_LICENSE("GPL v2");

