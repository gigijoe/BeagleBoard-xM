/*
 * Driver for Leopard Module Board used in Beagleboard (xM)
 *
 * Copyright (C) 2010 Texas Instruments Inc
 * Author: Sergio Aguirre <saaguirre@ti.com>
 *
 * Based on work done by:
 *     Vaibhav Hiremath <hvaibhav@ti.com>
 *     Anuj Aggarwal <anuj.aggarwal@ti.com>
 *     Sivaraj R <sivaraj@ti.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/mm.h>
#include <linux/videodev2.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <linux/platform_device.h>

#include <plat/mux.h>
#include <plat/board.h>
#include <plat/control.h>

#include <media/v4l2-int-device.h>

/* Include V4L2 ISP-Camera driver related header file */
#include <../drivers/media/video/omap34xxcam.h>
#include <../drivers/media/video/isp/ispreg.h>

#include "mux.h"

#define CAM_USE_XCLKA		0

#define LEOPARD_RESET_GPIO	98

static struct regulator *cam_1v8_reg;
static struct regulator *cam_2v8_reg;

#if defined(CONFIG_VIDEO_MT9V113) || defined(CONFIG_VIDEO_MT9V113_MODULE)
#include <media/mt9v113.h>

#define ISP_MT9V113_MCLK	216000000

/* Arbitrary memory handling limit */
#define MT9V113_MAX_FRAME_SIZE	PAGE_ALIGN(640 * 480 * 4)

static struct isp_interface_config mt9v113_if_config = {
	.ccdc_par_ser		= ISP_PARLL,
	.dataline_shift		= 0x2,
	.hsvs_syncdetect	= ISPCTRL_SYNC_DETECT_VSRISE,
	.strobe			= 0x0,
	.prestrobe		= 0x0,
	.shutter		= 0x0,
	.cam_mclk		= ISP_MT9V113_MCLK,
	.wenlog 		= ISPCCDC_CFG_WENLOG_AND,
	.wait_hs_vs		= 2,
	.u.par.par_bridge	= 0x3,
	.u.par.par_clk_pol	= 0x0,
};

static struct v4l2_ifparm mt9v113_ifparm_s = {
	.if_type = V4L2_IF_TYPE_YCbCr,
	.u 	 = {
		.ycbcr = {
			.frame_start_on_rising_vs = 1,
			.bt_sync_correct	= 0,
			.swap			= 0,
			.latch_clk_inv		= 0,
			.nobt_hs_inv		= 0,
			.nobt_vs_inv		= 0,
			.clock_min		= MT9V113_CLK_MIN,
			.clock_max		= MT9V113_CLK_MAX,
		},
	},
};

/**
 * @brief mt9v113_ifparm - Returns the mt9v113 interface parameters
 *
 * @param p - pointer to v4l2_ifparm structure
 *
 * @return result of operation - 0 is success
 */
static int mt9v113_ifparm(struct v4l2_ifparm *p)
{
	if (p == NULL)
		return -EINVAL;

	*p = mt9v113_ifparm_s;
	return 0;
}

static struct omap34xxcam_hw_config mt9v113_hwc = {
	.dev_index		= 0,
	.dev_minor		= -1,
	.dev_type		= OMAP34XXCAM_SLAVE_SENSOR,
	.u.sensor.sensor_isp	= 1,
	.u.sensor.capture_mem	= MT9V113_MAX_FRAME_SIZE * 2,
	.u.sensor.ival_default	= { 1, 10 },
};

/**
 * @brief mt9v113_set_prv_data - Returns mt9v113 omap34xx driver private data
 *
 * @param priv - pointer to omap34xxcam_hw_config structure
 *
 * @return result of operation - 0 is success
 */
static int mt9v113_set_prv_data(void *priv)
{
	struct omap34xxcam_hw_config *hwc = priv;

	if (priv == NULL)
		return -EINVAL;

	*hwc = mt9v113_hwc;
	return 0;
}

/**
 * @brief mt9v113_power_set - Power-on or power-off TVP5146 device
 *
 * @param power - enum, Power on/off, resume/standby
 *
 * @return result of operation - 0 is success
 */
static int mt9v113_power_set(struct v4l2_int_device *s, enum v4l2_power power)
{
	struct omap34xxcam_videodev *vdev = s->u.slave->master->priv;

	switch (power) {
	case V4L2_POWER_STANDBY:
		break;

	case V4L2_POWER_OFF:
		isp_set_xclk(vdev->cam->isp, 0, CAM_USE_XCLKA);

		if (regulator_is_enabled(cam_1v8_reg))
			regulator_disable(cam_1v8_reg);
		if (regulator_is_enabled(cam_2v8_reg))
			regulator_disable(cam_2v8_reg);

		break;

	case V4L2_POWER_ON:

		isp_configure_interface(vdev->cam->isp, &mt9v113_if_config);
		/* Set RESET_BAR to 0 */
		gpio_set_value(LEOPARD_RESET_GPIO, 0);

		/* turn on VDD */
		regulator_enable(cam_1v8_reg);
		mdelay(1);
		/* turn on VDD_IO */
		regulator_enable(cam_2v8_reg);
		mdelay(50);

		/* Enable EXTCLK */
		isp_set_xclk(vdev->cam->isp, MT9V113_CLK_MIN*2, CAM_USE_XCLKA);
		/*
		 * Wait at least 70 CLK cycles (w/EXTCLK = 6MHz, or CLK_MIN):
		 * ((1000000 * 70) / 6000000) = aprox 12 us.
		 */
		udelay(12);
		/* Set RESET_BAR to 1 */
		gpio_set_value(LEOPARD_RESET_GPIO, 1);
		/*
		 * Wait at least 100 CLK cycles (w/EXTCLK = 6MHz, or CLK_MIN):
		 * ((1000000 * 100) / 6000000) = aprox 17 us.
		 */
		udelay(17);

		break;

	default:
		return -ENODEV;
	}

	return 0;
}

struct mt9v113_platform_data mt9v113_pdata = {
	.master		= "omap34xxcam",
	.power_set	= mt9v113_power_set,
	.priv_data_set	= mt9v113_set_prv_data,
	.ifparm		= mt9v113_ifparm,
};

#endif				/* #ifdef CONFIG_VIDEO_MT9V113 */

#if defined(CONFIG_VIDEO_MT9T112) || defined(CONFIG_VIDEO_MT9T112_MODULE)
#include <media/mt9t112.h>

#define ISP_MT9T112_MCLK	216000000

/* Arbitrary memory handling limit */
#define MT9T112_BIGGEST_FRAME_BYTE_SIZE	PAGE_ALIGN((2048 * 2) * 1536 * 4)

static struct isp_interface_config mt9t112_if_config = {
	.ccdc_par_ser		= ISP_PARLL,
	.dataline_shift		= 0x2,
	.hsvs_syncdetect	= ISPCTRL_SYNC_DETECT_VSRISE,
	.strobe			= 0x0,
	.prestrobe		= 0x0,
	.shutter		= 0x0,
	.cam_mclk		= ISP_MT9T112_MCLK,
	.wenlog 		= ISPCCDC_CFG_WENLOG_AND,
	.wait_hs_vs		= 2,
	.u.par.par_bridge	= 0x2,
	.u.par.par_clk_pol	= 0x0,
};

static struct v4l2_ifparm mt9t112_ifparm_s = {
	.if_type = V4L2_IF_TYPE_YCbCr,
	.u 	 = {
		.ycbcr = {
			.frame_start_on_rising_vs = 1,
			.bt_sync_correct	= 0,
			.swap			= 0,
			.latch_clk_inv		= 0,
			.nobt_hs_inv		= 0,	/* active high */
			.nobt_vs_inv		= 0,	/* active high */
			.clock_min		= MT9T112_CLK_MIN,
			.clock_max		= MT9T112_CLK_MAX,
		},
	},
};

/**
 * @brief mt9t112_ifparm - Returns the mt9t112 interface parameters
 *
 * @param p - pointer to v4l2_ifparm structure
 *
 * @return result of operation - 0 is success
 */
static int mt9t112_ifparm(struct v4l2_ifparm *p)
{
	if (p == NULL)
		return -EINVAL;

	*p = mt9t112_ifparm_s;
	return 0;
}

#if defined(CONFIG_VIDEO_OMAP3) || defined(CONFIG_VIDEO_OMAP3_MODULE)
static struct omap34xxcam_hw_config mt9t112_hwc = {
	.dev_index		= 1,
	.dev_minor		= -1,
	.dev_type		= OMAP34XXCAM_SLAVE_SENSOR,
	.u.sensor.sensor_isp	= 0,
	.u.sensor.capture_mem	= MT9T112_BIGGEST_FRAME_BYTE_SIZE,
	.u.sensor.ival_default	= { 1, 30 },
};
#endif

/**
 * @brief mt9t112_set_prv_data - Returns mt9t112 omap34xx driver private data
 *
 * @param priv - pointer to omap34xxcam_hw_config structure
 *
 * @return result of operation - 0 is success
 */
static int mt9t112_set_prv_data(void *priv)
{
#if defined(CONFIG_VIDEO_OMAP3) || defined(CONFIG_VIDEO_OMAP3_MODULE)
	struct omap34xxcam_hw_config *hwc = priv;

	if (priv == NULL)
		return -EINVAL;

	*hwc = mt9t112_hwc;
	return 0;
#else
	return -EINVAL;
#endif
}

/**
 * @brief mt9t112_power_set - Power-on or power-off MT9T112 device
 *
 * @param power - enum, Power on/off, resume/standby
 *
 * @return result of operation - 0 is success
 */
static int mt9t112_power_set(struct v4l2_int_device *s, enum v4l2_power power)
{
	struct omap34xxcam_videodev *vdev = s->u.slave->master->priv;
	switch (power) {
	case V4L2_POWER_OFF:
	case V4L2_POWER_STANDBY:
		isp_set_xclk(vdev->cam->isp, 0, CAM_USE_XCLKA);

		if (regulator_is_enabled(cam_1v8_reg))
			regulator_disable(cam_1v8_reg);
		if (regulator_is_enabled(cam_2v8_reg))
			regulator_disable(cam_2v8_reg);
		break;

	case V4L2_POWER_ON:
#if defined(CONFIG_VIDEO_OMAP3) || defined(CONFIG_VIDEO_OMAP3_MODULE)
		isp_configure_interface(vdev->cam->isp, &mt9t112_if_config);
#endif

		/* Set RESET_BAR to 0 */
		gpio_set_value(LEOPARD_RESET_GPIO, 0);

		/* turn on VDD */
		regulator_enable(cam_1v8_reg);

		mdelay(1);

		/* turn on VDD_IO */
		regulator_enable(cam_2v8_reg);

		mdelay(50);

		/* Enable EXTCLK */
		isp_set_xclk(vdev->cam->isp, 12000000, CAM_USE_XCLKA);

		/*
		 * Wait at least 70 CLK cycles (w/EXTCLK = 24MHz):
		 * ((1000000 * 70) / 24000000) = aprox 2.91 us.
		 */

		udelay(3);

		/* Set RESET_BAR to 1 */
		gpio_set_value(LEOPARD_RESET_GPIO, 1);

		/*
		 * Wait at least 100 CLK cycles (w/EXTCLK = 24MHz):
		 * ((1000000 * 100) / 24000000) = aprox 4.16 us.
		 */

		udelay(5);

		break;

	default:
		return -ENODEV;
		break;
	}
	return 0;
}

struct mt9t112_platform_data mt9t112_pdata = {
	.master		= "omap34xxcam",
	.power_set	= mt9t112_power_set,
	.priv_data_set	= mt9t112_set_prv_data,
	.ifparm		= mt9t112_ifparm,
};

#endif				/* #ifdef CONFIG_VIDEO_MT9T112 */

#if defined(CONFIG_VIDEO_MT9P031) || defined(CONFIG_VIDEO_MT9P031_MODULE)
#include <media/mt9p031.h>

#define ISP_MT9P031_MCLK	216000000

/* Arbitrary memory handling limit */
#define MT9P031_BIGGEST_FRAME_BYTE_SIZE	PAGE_ALIGN((2592 * 1944) * 2 * 4 )

static struct isp_interface_config mt9p031_if_config = {
	.ccdc_par_ser		= ISP_PARLL,
	.dataline_shift		= 0x1,
	.hsvs_syncdetect	= ISPCTRL_SYNC_DETECT_VSRISE,
	.strobe			= 0x0,
	.prestrobe		= 0x0,
	.shutter		= 0x0,
	.cam_mclk		= ISP_MT9P031_MCLK,
	.wenlog 		= ISPCCDC_CFG_WENLOG_AND,
	.wait_hs_vs		= 2,
	.u.par.par_bridge	= 0x0,
	.u.par.par_clk_pol	= 0x0,
};

static struct v4l2_ifparm mt9p031_ifparm_s = {
	.if_type = V4L2_IF_TYPE_RAW,
	.u 	 = {
		.raw = {
			.frame_start_on_rising_vs = 1,
			.bt_sync_correct	= 0,
			.swap			= 0,
			.latch_clk_inv		= 0,
			.nobt_hs_inv		= 0,	/* active high */
			.nobt_vs_inv		= 0,	/* active high */
			.clock_min		= MT9P031_CLK_MIN,
			.clock_max		= MT9P031_CLK_MAX,
		},
	},
};

/**
 * @brief mt9p031_ifparm - Returns the mt9p031 interface parameters
 *
 * @param p - pointer to v4l2_ifparm structure
 *
 * @return result of operation - 0 is success
 */
static int mt9p031_ifparm(struct v4l2_ifparm *p)
{	
	if (p == NULL)
		return -EINVAL;

	*p = mt9p031_ifparm_s;
	return 0;
}

#if defined(CONFIG_VIDEO_OMAP3) || defined(CONFIG_VIDEO_OMAP3_MODULE)
static struct omap34xxcam_hw_config mt9p031_hwc = {
	.dev_index		= 1,
	.dev_minor		= -1,
	.dev_type		= OMAP34XXCAM_SLAVE_SENSOR,
	.u.sensor.sensor_isp	= 0,
	.u.sensor.capture_mem	= MT9P031_BIGGEST_FRAME_BYTE_SIZE,
	.u.sensor.ival_default	= { 1, 30 },
};
#endif

/**
 * @brief mt9p031_set_prv_data - Returns mt9p031 omap34xx driver private data
 *
 * @param priv - pointer to omap34xxcam_hw_config structure
 *
 * @return result of operation - 0 is success
 */
static int mt9p031_set_prv_data(void *priv)
{
#if defined(CONFIG_VIDEO_OMAP3) || defined(CONFIG_VIDEO_OMAP3_MODULE)
	struct omap34xxcam_hw_config *hwc = priv;

	if (priv == NULL)
		return -EINVAL;

	*hwc = mt9p031_hwc;
	return 0;
#else
	return -EINVAL;
#endif
}

/**
 * @brief mt9p031_power_set - Power-on or power-off mt9p031 device
 *
 * @param power - enum, Power on/off, resume/standby
 *
 * @return result of operation - 0 is success
 */
static int mt9p031_power_set(struct v4l2_int_device *s, enum v4l2_power power)
{	
	struct omap34xxcam_videodev *vdev = s->u.slave->master->priv;
	switch (power) {
	case V4L2_POWER_OFF:
	case V4L2_POWER_STANDBY:
		isp_set_xclk(vdev->cam->isp, 0, CAM_USE_XCLKA);

		if (regulator_is_enabled(cam_1v8_reg))
			regulator_disable(cam_1v8_reg);
		if (regulator_is_enabled(cam_2v8_reg))
			regulator_disable(cam_2v8_reg);
		break;

	case V4L2_POWER_ON:
#if defined(CONFIG_VIDEO_OMAP3) || defined(CONFIG_VIDEO_OMAP3_MODULE)
		isp_configure_interface(vdev->cam->isp, &mt9p031_if_config);
#endif

		/* Set RESET_BAR to 0 */
		gpio_set_value(LEOPARD_RESET_GPIO, 0);

		/* turn on VDD */
		regulator_enable(cam_1v8_reg);

		mdelay(1);

		/* turn on VDD_IO */
		regulator_enable(cam_2v8_reg);

		mdelay(50);

		/* Enable EXTCLK */
		isp_set_xclk(vdev->cam->isp, 24000000, CAM_USE_XCLKA);  //works for 36MHz too; try at lower freq

		/*
		 * Wait at least 70 CLK cycles (w/EXTCLK = 24MHz):
		 * ((1000000 * 70) / 24000000) = aprox 2.91 us.
		 */

		udelay(3);

		/* Set RESET_BAR to 1 */
		gpio_set_value(LEOPARD_RESET_GPIO, 1);

		/*
		 * Wait at least 100 CLK cycles (w/EXTCLK = 24MHz):
		 * ((1000000 * 100) / 24000000) = aprox 4.16 us.
		 */

		udelay(5);

		break;

	default:
		return -ENODEV;
		break;
	}
	return 0;
}

static u32 mt9p031_set_xclk(struct v4l2_int_device *s, u32 xclkfreq)
{	
    struct omap34xxcam_videodev *vdev = s->u.slave->master->priv;
    return isp_set_xclk(vdev->cam->isp, xclkfreq, 0);
}


struct mt9p031_platform_data mt9p031_pdata = {
	.master		= "omap34xxcam",
	.power_set	= mt9p031_power_set,
	.set_xclk       = mt9p031_set_xclk,
	.priv_data_set	= mt9p031_set_prv_data,
	.ifparm		= mt9p031_ifparm,
};

#endif				/* #ifdef CONFIG_VIDEO_MT9P031 */

static int beagle_cam_probe(struct platform_device *pdev)
{
	cam_1v8_reg = regulator_get(&pdev->dev, "cam_1v8");
	if (IS_ERR(cam_1v8_reg)) {
		dev_err(&pdev->dev, "cam_1v8 regulator missing\n");
		return PTR_ERR(cam_1v8_reg);
	}

	cam_2v8_reg = regulator_get(&pdev->dev, "cam_2v8");
	if (IS_ERR(cam_2v8_reg)) {
		dev_err(&pdev->dev, "cam_2v8 regulator missing\n");
		regulator_put(cam_1v8_reg);
		return PTR_ERR(cam_2v8_reg);
	}

	if (gpio_request(LEOPARD_RESET_GPIO, "cam_rst") != 0) {
		dev_err(&pdev->dev, "Could not request GPIO %d",
			LEOPARD_RESET_GPIO);
		regulator_put(cam_2v8_reg);
		regulator_put(cam_1v8_reg);
		return -ENODEV;
	}

	/* set to output mode, default value 0 */
	gpio_direction_output(LEOPARD_RESET_GPIO, 0);

	/* MUX init */
	omap_ctrl_writew(OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
			 0x10C); /* CAM_HS */
	omap_ctrl_writew(OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
			 0x10E); /* CAM_VS */
	omap_ctrl_writew(OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
			 0x110); /* CAM_XCLKA */
	omap_ctrl_writew(OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
			 0x112); /* CAM_PCLK */
	omap_ctrl_writew(OMAP_PIN_INPUT | OMAP_MUX_MODE0,
			 0x116); /* CAM_D0 */
	omap_ctrl_writew(OMAP_PIN_INPUT | OMAP_MUX_MODE0,
			 0x118); /* CAM_D1 */
	omap_ctrl_writew(OMAP_PIN_INPUT | OMAP_MUX_MODE0,
			 0x11A); /* CAM_D2 */
	omap_ctrl_writew(OMAP_PIN_INPUT | OMAP_MUX_MODE0,
			 0x11C); /* CAM_D3 */
	omap_ctrl_writew(OMAP_PIN_INPUT | OMAP_MUX_MODE0,
			 0x11E); /* CAM_D4 */
	omap_ctrl_writew(OMAP_PIN_INPUT | OMAP_MUX_MODE0,
			 0x120); /* CAM_D5 */
	omap_ctrl_writew(OMAP_PIN_INPUT | OMAP_MUX_MODE0,
			 0x122); /* CAM_D6 */
	omap_ctrl_writew(OMAP_PIN_INPUT | OMAP_MUX_MODE0,
			 0x124); /* CAM_D7 */
	omap_ctrl_writew(OMAP_PIN_INPUT | OMAP_MUX_MODE0,
			 0x126); /* CAM_D8 */
	omap_ctrl_writew(OMAP_PIN_INPUT | OMAP_MUX_MODE0,
			 0x128); /* CAM_D9 */
	omap_ctrl_writew(OMAP_PIN_INPUT | OMAP_MUX_MODE0,
			 0x12A); /* CAM_D10 */
	omap_ctrl_writew(OMAP_PIN_INPUT | OMAP_MUX_MODE0,
			 0x12C); /* CAM_D11 */

	printk(KERN_INFO "omap3beaglelmb: Driver registration complete\n");

	return 0;
}

static int beagle_cam_remove(struct platform_device *pdev)
{
	if (regulator_is_enabled(cam_1v8_reg))
		regulator_disable(cam_1v8_reg);
	regulator_put(cam_1v8_reg);

	if (regulator_is_enabled(cam_2v8_reg))
		regulator_disable(cam_2v8_reg);
	regulator_put(cam_2v8_reg);

	gpio_free(LEOPARD_RESET_GPIO);

	return 0;
}

static int beagle_cam_suspend(struct device *dev)
{
	return 0;
}

static int beagle_cam_resume(struct device *dev)
{
	return 0;
}

static struct dev_pm_ops beagle_cam_pm_ops = {
	.suspend = beagle_cam_suspend,
	.resume  = beagle_cam_resume,
};

static struct platform_driver beagle_cam_driver = {
	.probe		= beagle_cam_probe,
	.remove		= beagle_cam_remove,
	.driver		= {
		.name	= "beagle_cam",
		.pm	= &beagle_cam_pm_ops,
	},
};

/**
 * @brief omap3beaglelmb_init - module init function. Should be called before any
 *                          client driver init call
 *
 * @return result of operation - 0 is success
 */
int __init omap3beaglelmb_init(void)
{
	/* NOTE: Beagle xM boards are the only ones with camera interface */
	if (cpu_is_omap3630())
		platform_driver_register(&beagle_cam_driver);

	return 0;
}
late_initcall(omap3beaglelmb_init);
