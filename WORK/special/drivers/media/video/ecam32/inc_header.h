/**********************************************************************************************************
 *   Copyright (C) 2009 by e-con Systems.                                                                 *
 *   www.e-consystems.com                                                                                 *
 *                                                                                                        *
 *   This file is licensed under the terms of the GNU General Public License                              *
 *   version 2. This program is licensed "as is" without any warranty of any                              *
 *   kind, whether express or implied.                                                                    *
 *                                                                                                        *
 *                                                                                                        *
 *   PROJECT	           :        OMAP Camera development                                               *
 *   MODULE NAME           :                                                                              *
 *   MODULE VERSION        :        VER 3.0                                                               *
 *                                                                                                        *
 *                                                                                                        *
 *                                                                                                        *
 *   Version No	: 000-0001                                                          CODE_REV  : 0.0.1.1   *
 **********************************************************************************************************/

/*
 *==========================================================================================================
 *                                        REVISION HISTORY                                  
 *----------------------------------------------------------------------------------------------------------
 * CODE_REV  REASON FOR MODIFICATION                MODIFIED FUNCTION NAME  	            AUTHOR
 *----------------------------------------------------------------------------------------------------------
 * 
 * 0.0.1.0   ------------------------ Driver development ---------------------------- Ananthapadmanaban
 *
 * 3.0       Flash support added  
 *==========================================================================================================
 */
/*
 * include configuration code for selecting sensor and flash driver code
 */
#include "auto_conf.h"


/*
 * Define the Include header file Macro
 */
#define MODULE_NAME	"V4l2 driver module"

#define CONFIG_DRIVER
#define USE_KERNEL_THREAD
#define KERNEL_ARM_OMAP
#define KERNEL_ARM_OMAP3530
#define USE_KERNEL_MEMORY_MANAGE
#define CONFIG_KERNEL_ERR_INCLUDED

#include "resource/include/Headerfile.h"

/*
 * driver specific header files
 */

#include <media/v4l2-dev.h>
#include <mach/gpio.h>

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,4,0))

#ifndef gpio_get_value
#define gpio_get_value    __gpio_get_value
#endif

#ifndef gpio_set_value
#define gpio_set_value  __gpio_set_value
#endif

#ifndef gpio_cansleep     
#define gpio_cansleep     __gpio_cansleep
#endif

#endif

#include <media/v4l2-ioctl.h>

/*
 * Include module specific code here
 */
#include "omap_camera_interface.h"
#include "isp.h"
#include "v4l2_driver_base.h"
#include "omap_v4l2.h"
#include "i2c.h"


/*
 * include the sensor file
 */
#if defined (CONFIG_USE_OV5642_SENSOR)
	#include "ov5642/sens_ov5642.h"
#elif defined (CONFIG_USE_OV3640_SENSOR)
	#include "ov3640/sens_ov3640.h"
#elif defined (CONFIG_USE_OV10630_SENSOR)
	#include "ov10630/sens_ov10630.h"
#elif defined (CONFIG_USE_OV10633_SENSOR)
	#include "ov10633/sens_ov10633.h"
#else
	#warning "Sensor related driver file not selected"
#endif

/*
 * include Flash related header file
 */

#if defined(CONFIG_USE_LM3553_FLASH)
	#include "lm3553/lm3553_flash.h"
#elif defined(CONFIG_USE_STCF03_FLASH)
	#include "stcf03/stcf03_flash.h"
#else
	#warning "Flash related driver file not selected"
#endif

/*
 * Tracking maintenance 
 */
#include "svn_revision.h"
/*
 * Include function protype here
 */

#include "fn_protype.h"
