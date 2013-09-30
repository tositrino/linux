/* include/media/video/dohofpga1_platform.h
 *
 * Copyright (c) 2013 institute for visual computing, ETH Zuerich
 * http://ivc.ethz.ch/
 *
 * based on mt9m113
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 * http://www.samsung.com/
 *
 * Driver for dominik honeggers fpga camara interface 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __DOHOFPGA1_PLATFORM_H__
#define __DOHOFPGA1_PLATFORM_H__

#include <linux/device.h>
#include <media/v4l2-mediabus.h>

struct dohofpga1_platform_data 
{
  unsigned int default_width;
  unsigned int default_height;
  unsigned int pixelformat;

  int freq;  /* MCLK in KHz */

  /* This SoC supports Parallel & CSI-2 */
  int is_mipi;
};

struct dohofpga1_mbus_platform_data 
{
  int id;
  struct v4l2_mbus_framefmt fmt;
  unsigned long clk_rate; /* master clock frequency in Hz */
  int (*set_power)(int on);
  int (*set_clock)(struct device *dev, int on);
};

#endif //ifndef __DOHOFPGA1_PLATFORM_H__
