/*
 * ------------------------------------------------------------------------------------------------------------
 *                                              e_CAM32_OMAP_GSTIX
 * ------------------------------------------------------------------------------------------------------------
 */

#define CONFIG_BOARD_e_CAM32_OMAP_GSTIX

/*
 * ------------------------------------------------------------------------------------------------------------
 *                                                Memory related
 * ------------------------------------------------------------------------------------------------------------
 */

/*
 * e_CAM50_OMAP_GSTIX
 *
 *
 * Modified version:
 * etc/rc5.d/S30gstti-init
 *
 * # Start Addr    Size    Description
 * # -------------------------------------------
 * # 0x80000000    93 MB   Linux
 * # 0x85700000     6 MB   e-con camera
 * # 0x86300000    16 MB   CMEM
 * # 0x87300000    13 MB   CODEC SERVER
 * # 0x88000000   128 MB   RAM starts if available is 256 MB ram
 *
 * Boot arguments used
 * -------------------
 * setenv mmcargs 'setenv bootargs console=${console} vram=${vram} mem=87M@0x80000000 mem=128M@0x88000000 omapfb.mode=dvi:${dvimode} omapfb.debug=y omapdss.def_disp=${defaultdisplay} root=/dev/mmcblk0p2 rw rootfstype=ext3 rootwait i2c_bus=3,100'
 */

#define CONFIG_RAM_SIZE_IN_MB			256
#define CONFIG_CMEM_DRIVER_AVAILABLE
#define CONFIG_ALLOW_DRIVER_PHY_MEMORY_OVERLAP

#define PHY_MEM_HIGH_ALLOCATE_IN_MB		6	

#if (CONFIG_RAM_SIZE_IN_MB == 128)

	#if defined(CONFIG_CMEM_DRIVER_AVAILABLE)
		#define PHY_MEM_END		0x862FFFFF	// start address is 0x8570_0000  @ 12M
	#else
		#define PHY_MEM_END		0x87FFFFFF
	#endif
	
#elif (CONFIG_RAM_SIZE_IN_MB == 256)

	#if defined(CONFIG_CMEM_DRIVER_AVAILABLE)
		#define PHY_MEM_END		0x862FFFFF	// start address is 0x8570_0000  @ 12M
	#else
		#define PHY_MEM_END		0x8FFFFFFF
	#endif

#endif
#define PHY_MEM_START				(PHY_MEM_END - ((PHY_MEM_HIGH_ALLOCATE_IN_MB * 1024 * 1024)-1))

/*
 * ------------------------------------------------------------------------------------------------------------
 *                                             Driver related
 * ------------------------------------------------------------------------------------------------------------
 */

#define STILL_IMAGE_CAPTURE_FRAME_NUMBER	3
#define CONFIG_USE_TI_RESIZER			DISABLE
#define CONFIG_ISP_DATA_LINE_SHIFT		DISABLE
#define CONFIG_ISP_SLV0_DISCARD_COUNT		0
#define CONFIG_SENS_MCLK			0
#undef CONFIG_CTRL_FRAME_RATE_FRM_SENSOR
/*
 * ------------------------------------------------------------------------------------------------------------
 *                                             Sensor related
 * ------------------------------------------------------------------------------------------------------------
 */

#define CONFIG_USE_OV3640_SENSOR
#define CONFIG_OV3640_DEFAULT_FLIP		DISABLE	
#define CONFIG_OV3640_DEFAULT_MIRROR		DISABLE

/*
 * ------------------------------------------------------------------------------------------------------------
 *                                             Flash related
 * ------------------------------------------------------------------------------------------------------------
 */

#define CONFIG_USE_LM3553_FLASH
/*
 * FIXME:
 * 	When Flash lumination value is set more than 40 then board re-boots at snap mode.
 */
#define CONIFG_LM3553_FLASH_MAX_LUM_VALUE		40



