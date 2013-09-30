/* linux/arch/arm/mach-exynos/mach-odroid-fimc.inc
 *
 * code include file for odroid variants
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __MACH_ODROID_FIMC_INC__
#define __MACH_ODROID_FIMC_INC__

/* hardkernel mt9m113 */
#if defined(CONFIG_VIDEO_MT9M113) || defined(CONFIG_VIDEO_MT9M113_MODULE)
#warning MT9M113 code enabled

static int mt9m113_cam0_reset(int dummy)
{
	int err;
	/* Camera A */
	err = gpio_request(EXYNOS4_GPX2(7), "GPX2"); //reset
	if (err)
		printk(KERN_ERR "#### failed to request GPX1_2 ####\n");

	err = gpio_request(EXYNOS4212_GPJ1(4), "GPJ1"); //stnby
	if (err)
		printk(KERN_ERR "#### failed to request GPJ1_4 ####\n");

	s3c_gpio_setpull(EXYNOS4_GPX2(7), S3C_GPIO_PULL_NONE);
	s3c_gpio_setpull(EXYNOS4212_GPJ1(4), S3C_GPIO_PULL_NONE);

	gpio_direction_output(EXYNOS4212_GPJ1(4), 1);
	gpio_direction_output(EXYNOS4_GPX2(7), 0);
	msleep(50);
	gpio_direction_output(EXYNOS4_GPX2(7), 1);
	msleep(100);
	gpio_direction_output(EXYNOS4212_GPJ1(4), 0);
	
	gpio_free(EXYNOS4_GPX2(7));
	gpio_free(EXYNOS4212_GPJ1(4));
	
	printk("Cam0 Reset\n");
	return 0;
}

static struct mt9m113_platform_data mt9m113_plat = {
	.default_width = 1920,
	.default_height = 1080,
	.pixelformat = V4L2_PIX_FMT_UYVY,
	.freq = 24000000,
	.is_mipi = 0,
};
static struct i2c_board_info mt9m113_i2c_info = {
	I2C_BOARD_INFO("MT9M113", 0x7a>>1),
	.platform_data = &mt9m113_plat,
};

static struct s3c_platform_camera mt9m113 = {

#if defined(CONFIG_MT9M113_CSI_D)
#ifndef CONFIG_CSI_D
#define CONFIG_CSI_D
#endif
	.id		= CAMERA_CSI_D,
	.type		= CAM_TYPE_MIPI,
#elif defined(CONFIG_MT9M113_CSI_C)
#ifndef CONFIG_CSI_C
#define CONFIG_CSI_C
#endif
	.id		= CAMERA_CSI_C,
	.type		= CAM_TYPE_MIPI,
#elif defined(CONFIG_MT9M113_ITU_B)
#ifndef CONFIG_ITU_B
#define CONFIG_ITU_B
#endif
	.id		= CAMERA_PAR_B,
	.type		= CAM_TYPE_ITU,
#else
#ifndef CONFIG_ITU_A
#define CONFIG_ITU_A
#endif
	.id		= CAMERA_PAR_A,
	.type		= CAM_TYPE_ITU,
#endif

	.clk_name	= "sclk_cam0",
	.cam_power	= mt9m113_cam0_reset,
	.i2c_busnum = 5,
	.fmt		= ITU_601_YCBCR422_8BIT,
	.order422	= CAM_ORDER422_8BIT_CBYCRY,
	.info		= &mt9m113_i2c_info,
	.pixelformat	= V4L2_PIX_FMT_UYVY,
	.srclk_name	= "xusbxti",

	.clk_rate	= 24000000,
	.line_length	= 1920,
	.width		= 640,
	.height		= 480,
	.window		= {
		.left	= 0,
		.top	= 0,
		.width	= 640,
		.height	= 480,
	},

	.mipi_lanes	= 2,
	.mipi_settle	= 12,
	.mipi_align	= 24,

	/* Polarity */
	.inv_pclk	= 1,
	.inv_vsync	= 1,
	.inv_href	= 0,
	.inv_hsync	= 0,
	.use_isp	= 0,
	.initialized	= 0,
};
#endif //if defined(CONFIG_VIDEO_MT9M113) || defined(CONFIG_VIDEO_MT9M113_MODULE)

/* dohofpga1 */
#if defined(CONFIG_VIDEO_DOHOFPGA1) || defined(CONFIG_VIDEO_DOHOFPGA1_MODULE)
#warning DOHOFPGA1 code enabled

static int dohofpga1_cam0_reset(int dummy)
{
	int err;
	/* Camera A */
	err = gpio_request(EXYNOS4_GPX2(7), "GPX2"); //reset
	if (err)
		printk(KERN_ERR "#### failed to request GPX1_2 ####\n");

	err = gpio_request(EXYNOS4212_GPJ1(4), "GPJ1"); //stnby
	if (err)
		printk(KERN_ERR "#### failed to request GPJ1_4 ####\n");

	s3c_gpio_setpull(EXYNOS4_GPX2(7), S3C_GPIO_PULL_NONE);
	s3c_gpio_setpull(EXYNOS4212_GPJ1(4), S3C_GPIO_PULL_NONE);

	gpio_direction_output(EXYNOS4212_GPJ1(4), 1);
	gpio_direction_output(EXYNOS4_GPX2(7), 0);
	msleep(50);
	gpio_direction_output(EXYNOS4_GPX2(7), 1);
	msleep(100);
	gpio_direction_output(EXYNOS4212_GPJ1(4), 0);
	
	gpio_free(EXYNOS4_GPX2(7));
	gpio_free(EXYNOS4212_GPJ1(4));
	
	printk("Cam0 Reset\n");
	return 0;
}

static struct dohofpga1_platform_data dohofpga1_plat = {
	.default_width = 1920,
	.default_height = 1080,
	.pixelformat = V4L2_PIX_FMT_UYVY,//V4L2_PIX_FMT_GREY,//
	.freq = 50000000,//24000000,
	.is_mipi = 0,
};
static struct i2c_board_info dohofpga1_i2c_info = {
	I2C_BOARD_INFO("DOHOFPGA1", 0x7a>>1),
	.platform_data = &dohofpga1_plat,
};

static struct s3c_platform_camera dohofpga1 = 
{

#if defined(CONFIG_DOHOFPGA1_CSI_D)
#ifndef CONFIG_CSI_D
#define CONFIG_CSI_D
#endif
	.id		= CAMERA_CSI_D,
	.type		= CAM_TYPE_MIPI,
#elif defined(CONFIG_DOHOFPGA1_CSI_C)
#ifndef CONFIG_CSI_C
#define CONFIG_CSI_C
#endif
	.id		= CAMERA_CSI_C,
	.type		= CAM_TYPE_MIPI,
#elif defined(CONFIG_DOHOFPGA1_ITU_B)
#ifndef CONFIG_ITU_B
#define CONFIG_ITU_B
#endif
	.id		= CAMERA_PAR_B,
	.type		= CAM_TYPE_ITU,
#else
#ifndef CONFIG_ITU_A
#define CONFIG_ITU_A
#endif
	.id		= CAMERA_PAR_A,
	.type		= CAM_TYPE_ITU,
#endif

	.clk_name	= "sclk_cam0",
	.cam_power	= dohofpga1_cam0_reset,
	.i2c_busnum = 5,
	.fmt		= ITU_601_YCBCR422_8BIT,
	.order422	= CAM_ORDER422_8BIT_CBYCRY,
	.info		= &dohofpga1_i2c_info,
	.pixelformat	= V4L2_PIX_FMT_UYVY,//
	.srclk_name	= "xusbxti",

	.clk_rate	= 50000000,//24000000,
	.line_length	= 1920,
	.width		= 640,
	.height		= 480,
	.window		= {
		.left	= 0,
		.top	= 0,
		.width	= 640,
		.height	= 480,
	},

	.mipi_lanes	= 2,
	.mipi_settle	= 12,
	.mipi_align	= 24,

	/* Polarity */
	.inv_pclk	= 1,
	.inv_vsync	= 1,
	.inv_href	= 0,
	.inv_hsync	= 0,
	.use_isp	= 0,
	.initialized	= 0,
};
#endif //if defined(CONFIG_VIDEO_DOHOFPGA1) || defined(CONFIG_VIDEO_DOHOFPGA1_MODULE)


/* dohofpga2 */
#if defined(CONFIG_VIDEO_DOHOFPGA2) || defined(CONFIG_VIDEO_DOHOFPGA2_MODULE)
#warning DOHOFPGA2 code enabled

static int dohofpga2_cam0_reset(int dummy)
{
	int err;
	/* Camera A */
	err = gpio_request(EXYNOS4_GPX2(7), "GPX2"); //reset
	if (err)
		printk(KERN_ERR "#### failed to request GPX1_2 ####\n");

	err = gpio_request(EXYNOS4212_GPJ1(4), "GPJ1"); //stnby
	if (err)
		printk(KERN_ERR "#### failed to request GPJ1_4 ####\n");

	s3c_gpio_setpull(EXYNOS4_GPX2(7), S3C_GPIO_PULL_NONE);
	s3c_gpio_setpull(EXYNOS4212_GPJ1(4), S3C_GPIO_PULL_NONE);

	gpio_direction_output(EXYNOS4212_GPJ1(4), 1);
	gpio_direction_output(EXYNOS4_GPX2(7), 0);
	msleep(50);
	gpio_direction_output(EXYNOS4_GPX2(7), 1);
	msleep(100);
	gpio_direction_output(EXYNOS4212_GPJ1(4), 0);
	
	gpio_free(EXYNOS4_GPX2(7));
	gpio_free(EXYNOS4212_GPJ1(4));
	
	printk("Cam0 Reset\n");
	return 0;
}

static struct dohofpga2_platform_data dohofpga2_plat = {
	.default_width = 1920,
	.default_height = 1080,
	.pixelformat = V4L2_PIX_FMT_UYVY,//V4L2_PIX_FMT_GREY,//
	.freq = 50000000,//24000000,
	.is_mipi = 0,
};
static struct i2c_board_info dohofpga2_i2c_info = {
	I2C_BOARD_INFO("DOHOFPGA2", 0x7a>>1),
	.platform_data = &dohofpga2_plat,
};

static struct s3c_platform_camera dohofpga2 = 
{

#if defined(CONFIG_DOHOFPGA2_CSI_D)
#ifndef CONFIG_CSI_D
#define CONFIG_CSI_D
#endif
	.id		= CAMERA_CSI_D,
	.type		= CAM_TYPE_MIPI,
#elif defined(CONFIG_DOHOFPGA2_CSI_C)
#ifndef CONFIG_CSI_C
#define CONFIG_CSI_C
#endif
	.id		= CAMERA_CSI_C,
	.type		= CAM_TYPE_MIPI,
#elif defined(CONFIG_DOHOFPGA2_ITU_B)
#ifndef CONFIG_ITU_B
#define CONFIG_ITU_B
#endif
	.id		= CAMERA_PAR_B,
	.type		= CAM_TYPE_ITU,
#else
#ifndef CONFIG_ITU_A
#define CONFIG_ITU_A
#endif
	.id		= CAMERA_PAR_A,
	.type		= CAM_TYPE_ITU,
#endif

	.clk_name	= "sclk_cam0",
	.cam_power	= dohofpga2_cam0_reset,
	.i2c_busnum = 5,
	.fmt		= ITU_601_YCBCR422_8BIT,
	.order422	= CAM_ORDER422_8BIT_CBYCRY,
	.info		= &dohofpga2_i2c_info,
	.pixelformat	= V4L2_PIX_FMT_UYVY,//
	.srclk_name	= "xusbxti",

	.clk_rate	= 50000000,//24000000,
	.line_length	= 1920,
	.width		= 640,
	.height		= 480,
	.window		= {
		.left	= 0,
		.top	= 0,
		.width	= 640,
		.height	= 480,
	},

	.mipi_lanes	= 2,
	.mipi_settle	= 12,
	.mipi_align	= 24,

	/* Polarity */
	.inv_pclk	= 1,
	.inv_vsync	= 1,
	.inv_href	= 0,
	.inv_hsync	= 0,
	.use_isp	= 0,
	.initialized	= 0,
};
#endif //if defined(CONFIG_VIDEO_DOHOFPGA2) || defined(CONFIG_VIDEO_DOHOFPGA2_MODULE)

/* hardkernel s5k4ecgx */
#if defined(CONFIG_VIDEO_S5K4ECGX) || defined(CONFIG_VIDEO_S5K4ECGX_MODULE)
static int s5k4ecgx_power(int enable)
{
	int err = 0;

	err = gpio_request(EXYNOS4_GPX3(1), "GPX3");
	if (err)
		printk(KERN_ERR "#### failed to request GPX3_1 ####\n");

	s3c_gpio_setpull(EXYNOS4_GPX3(1), S3C_GPIO_PULL_NONE);

	if(enable) {
		gpio_direction_output(EXYNOS4_GPX3(1), 0);
		mdelay(200);
		gpio_direction_output(EXYNOS4_GPX3(1), 1);
		mdelay(50);
	}
	else {
		gpio_direction_output(EXYNOS4_GPX3(1), 0);
	}
	gpio_free(EXYNOS4_GPX3(1));

	return 0;
}

static struct s5k4ecgx_platform_data s5k4ecgx_plat = 
{
	.default_width = 640,
	.default_height = 480,
	.pixelformat = V4L2_PIX_FMT_UYVY,
	.freq = 24000000,

	.is_mipi = 1,
	.streamoff_delay = 100,
	.set_power = s5k4ecgx_power,
};

static struct i2c_board_info s5k4ecgx_i2c_info = 
{
	I2C_BOARD_INFO("S5K4ECGX", 0x5a>>1),
	.platform_data = &s5k4ecgx_plat,
};

static struct s3c_platform_camera s5k4ecgx = {
#ifdef CONFIG_S5K4ECGX_CSI_C
#ifndef CONFIG_CSI_C
#define CONFIG_CSI_C
#endif
	.id		= CAMERA_CSI_C,
	.clk_name	= "sclk_cam1",
	.cam_power	= s5k4ecgx_power,
#endif
	.i2c_busnum = 5,
	.type		= CAM_TYPE_MIPI,
	.fmt		= MIPI_CSI_YCBCR422_8BIT,
	.order422	= CAM_ORDER422_8BIT_CBYCRY,
	.info		= &s5k4ecgx_i2c_info,
	.pixelformat	= V4L2_PIX_FMT_UYVY,
	.srclk_name	= "xusbxti",
	.clk_rate	= 24000000,
	.line_length	= 1920,
	.width		= 1920,
	.height		= 1080,
	.window		= {
		.left	= 0,
		.top	= 0,
		.width	= 1920,
		.height	= 1080,
	},

	.mipi_lanes	= 2,
	.mipi_settle = 12,
	.mipi_align	= 32,

	/* Polarity */
	.inv_pclk	= 0,
	.inv_vsync	= 1,
	.inv_href	= 0,
	.inv_hsync	= 0,
	.use_isp	= 0,
	.initialized	= 0,
};
#endif //if defined(CONFIG_VIDEO_S5K4ECGX) || defined(CONFIG_VIDEO_S5K4ECGX_MODULE)

#ifdef WRITEBACK_ENABLED

static struct i2c_board_info writeback_i2c_info = 
{
  I2C_BOARD_INFO("WriteBack", 0x0),
};

static struct s3c_platform_camera writeback = 
{
	.id		= CAMERA_WB,
	.fmt		= ITU_601_YCBCR422_8BIT,
	.order422	= CAM_ORDER422_8BIT_CBYCRY,
	.i2c_busnum	= 0,
	.info		= &writeback_i2c_info,
	.pixelformat	= V4L2_PIX_FMT_YUV444,
	.line_length	= 800,
	.width		= 480,
	.height		= 800,
	.window		= 
	{
		.left	= 0,
		.top	= 0,
		.width	= 480,
		.height	= 800,
    },
	.initialized	= 0,
};
#endif // ifdef WRITEBACK_ENABLED

/* Interface setting */
static struct s3c_platform_fimc fimc_plat = 
{
#ifdef CONFIG_ITU_A
#warning default camera is PAR_A
	.default_cam	= CAMERA_PAR_A,
#endif
#ifdef CONFIG_ITU_B
#warning default camera is PAR_B
	.default_cam	= CAMERA_PAR_B,
#endif
#ifdef CONFIG_CSI_C
#warning default camera is CSI_C
	.default_cam	= CAMERA_CSI_C,
#endif
#ifdef CONFIG_CSI_D
#warning default camera is ICSI_D
	.default_cam	= CAMERA_CSI_D,
#endif
#ifdef WRITEBACK_ENABLED
#warning default camera is CAMERA_WB
	.default_cam	= CAMERA_WB,
#endif
	.camera		= 
	{
#if defined(CONFIG_VIDEO_DOHOFPGA1) || defined(CONFIG_VIDEO_DOHOFPGA1_MODULE)
#warning added dohofpga1
		&dohofpga1,
#endif

#if defined(CONFIG_VIDEO_DOHOFPGA2) || defined(CONFIG_VIDEO_DOHOFPGA2_MODULE)
#warning added dohofpga2
		&dohofpga2,
#endif

#if defined(CONFIG_VIDEO_MT9M113) || defined(CONFIG_VIDEO_MT9M113_MODULE)
#warning added mt9m113
		&mt9m113,
#endif

#if defined(CONFIG_VIDEO_S5K4ECGX) || defined(CONFIG_VIDEO_S5K4ECGX_MODULE)
#warning added s5k4ecgx
		&s5k4ecgx,
#endif

#ifdef WRITEBACK_ENABLED
#warning added writeback
		&writeback,
#endif
	},
	.hw_ver		= 0x51,
};

#endif // ifndef __MACH_ODROID_FIMC_INC__
