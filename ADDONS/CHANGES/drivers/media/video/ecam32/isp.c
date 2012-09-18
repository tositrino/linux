/**********************************************************************************************************
 *   Copyright (C) 2009 by e-con Systems                                                                  *
 *   www.e-consystems.com                                                                                 *
 *                                                                                                        *
 *   This file is licensed under the terms of the GNU General Public License                              *
 *   version 2. This program is licensed "as is" without any warranty of any                              *
 *   kind, whether express or implied.                                                                    *
 *                                                                                                        *
 *                                                                                                        *
 *   PROJECT	           :        OMAP Camera development                                               *
 *   MODULE NAME           :                                                                              *
 *   MODULE VERSION        :                                                                              *
 *                                                                                                        *
 *                                                                                                        *
 *                                                                                                        *
 *   Version No	: 000-0001                                                          CODE_REV  : 0.0.1.0   *
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
 *
 *==========================================================================================================
 */

/*
 * ----------------------------------------------------------------------------------------------------------
 *
 * 					Header files inclusion part
 *
 * ----------------------------------------------------------------------------------------------------------
 */

#include "inc_header.h"

static UINT32 cam_mclk	= CONFIG_SENS_MCLK;
module_param(cam_mclk,int, 0444);

/************************************************************************************************************
 *  
 *  MODULE TYPE	:	ISR ROUTINE				MODULE ID	:	OMAP_V4L2_BASE	
 *  Name	:	omap34xx_isp_isr	
 *  Parameter1	:	INT32 irq
 *  Parameter2	:	PINT0 _cam	- private data number
 *  Returns	:	irq - handled information
 *  Description	: 	interrupt service routine 
 *  Comments	:  	
 ************************************************************************************************************/

irqreturn_t omap34xx_isp_isr(INT32 irq,PINT0 _cam)
{
	cam_data *cam	= _cam;
	FNRESLT ret_val;
	static UINT32 using_dummy_buffer	= DISABLE;
	static UINT32 fill_dummy_buf		= DISABLE;
	union
	{
		UVINT32 ISP_IRQ0ENABLE;
		struct isp_irq bit;
	}isp_status;
	isp_status.ISP_IRQ0ENABLE				= (cam->isp->isp_main.reg.isp_irq0status.ISP_IRQ0STATUS & CCDC_VD0);
	cam->isp->isp_main.reg.isp_irq0status.ISP_IRQ0STATUS	= isp_status.ISP_IRQ0ENABLE;
	cam->isp->isp_main.reg.isp_irq1status.ISP_IRQ1STATUS	= cam->isp->isp_main.reg.isp_irq1status.ISP_IRQ1STATUS;

	if(isp_status.bit.ccdc_vd0_irq			== ENABLED)
	{
		if(cam->task.bit.still	== ENABLE)
		{
			cam->still.frame_count++;
			if(cam->still.frame_count >= STILL_IMAGE_CAPTURE_FRAME_NUMBER)
			{
				wake_up_interruptible(&cam->still.dma_frame_complete_still);
				cam->still.wait_queue_head_t_dma_frame_complete_still	= ENABLE;
			}
		}else if(cam->task.bit.capture	== ENABLE)
		{

#if (!defined(CONFIG_CTRL_FRAME_RATE_FRM_SENSOR))
			static UINT32 frame_skip_count;
			static UINT32 capture_frame_rate;
			static UINT32 current_fps	= SENS_MAX_FPS;
			static struct timeval timestamp_rec;
			struct timeval timestamp;
			static INT32 frame_rate_denominator = SENS_MAX_FPS;

			if(frame_rate_denominator != cam->capture.s_parm.parm.capture.timeperframe.denominator)
			{
				frame_skip_count	= DISABLE;
				frame_rate_denominator	= cam->capture.s_parm.parm.capture.timeperframe.denominator;
			}

			do_gettimeofday(&timestamp);

			if(timestamp_rec.tv_sec	== DISABLE && timestamp_rec.tv_usec == DISABLE)
			{
				do_gettimeofday(&timestamp_rec);
			}

			if (timestamp.tv_sec > timestamp_rec.tv_sec)
			{
				current_fps		= capture_frame_rate;
				capture_frame_rate	= DISABLE;
				timestamp_rec		= timestamp;
			}
			capture_frame_rate++;
#endif 

 /*
 * Process the completed buffer
  */
			if(using_dummy_buffer	== DISABLE)
			{
				if(cam->capture.processing)
				{
					__link_node(cam->capture.filled, cam->capture.processing);
					do_gettimeofday(&cam->capture.processing->buffer.timestamp);
					cam->capture.processing->buffer.flags	|= V4L2_BUF_FLAG_DONE;
					cam->capture.valid_buf++;
				}
			}

			if(fill_dummy_buf	== DISABLE)
			{

/*
 * Process the next processing buffer
 */
				cam->capture.processing			= cam->capture.Need_to_be_filled;
				if(cam->capture.processing)
				{
					__update_base(cam->capture.Need_to_be_filled);
					using_dummy_buffer	= DISABLE;
				}else
				{
					if((cam->capture.filled) && (cam->capture.filled->next))
					{
						cam->capture.processing		= cam->capture.filled;
					}

					if(cam->capture.processing)
					{
						__update_base(cam->capture.filled);
						cam->capture.valid_buf--;
						using_dummy_buffer	= DISABLE;
					}else
					{
						cam->capture.processing	= &cam->capture.frame[cam->capture.available_buf-1];
						using_dummy_buffer	= ENABLE;
					}
				}
			}else
			{
				cam->capture.processing	= &cam->capture.frame[cam->capture.available_buf-1];
				using_dummy_buffer	= ENABLE;
			}

			if(cam->capture.valid_buf)
			{
				wake_up_interruptible(&cam->capture.capture_frame_complete);
			}

			cam->capture.processing->next		= NULL;
			ret_val	= isp_prg_sdram_addr(cam);
			if(CHECK_IN_FAIL_LIMIT(ret_val))
			{
				
			}

#ifndef CONFIG_CTRL_FRAME_RATE_FRM_SENSOR
			frame_skip_count	+= ((1000000*cam->capture.s_parm.parm.capture.			\
							timeperframe.denominator)/current_fps);
			if(frame_skip_count > 1000000)
			{
				frame_skip_count -=1000000;
				fill_dummy_buf	= DISABLE;
			}else
			{
				fill_dummy_buf	= ENABLE;
			}
#endif
		}
		return IRQ_HANDLED;
	}
	return IRQ_NONE;
}
/************************************************************************************************************
 *  
 *  MODULE TYPE	:	FUNCTION				MODULE ID	:	OMAP_V4L2_BASE	
 *  Name	:	isp_set_xclk
 *  Parameter1	:	cam_data *cam	- Base address of camera structure pointer
 *  Parameter2	:	xclk		- Needed mclk given to sensor
 *  Parameter3	:	xclksel		- Needed xclk mode in the omap
*  
 *  Returns	:	FNRESLT		- On Success Zero (or) positive value be returned to the calling
 *  					  Functions and On error a negative value be returned
 *
 *  					  Note: 
 *  					  	For more detail about the return values please refer
 *  					  error.c and error.h file available in the current project
 *  Description	: 	Sets the mclk provided to the sensor
 *  Comments	:  	
 ************************************************************************************************************/

FNRESLT isp_set_xclk(cam_data *cam,UINT32 xclk, UINT8 xclksel, UPINT32 current_xclk)
{
#define CM_CAM_MCLK_HZ			216000000
#define ISPTCTRL_CTRL_DIV_BYPASS	0x1F

	UINT32 divisor;
	UINT32 currentxclk;

	if (xclk >= CM_CAM_MCLK_HZ)
	{
		divisor = ISPTCTRL_CTRL_DIV_BYPASS;
		currentxclk = CM_CAM_MCLK_HZ;
	}else if (xclk >= 2)
	{
		divisor = CM_CAM_MCLK_HZ / xclk;
		if (divisor >= ISPTCTRL_CTRL_DIV_BYPASS)
			divisor = ISPTCTRL_CTRL_DIV_BYPASS - 1;
		currentxclk = CM_CAM_MCLK_HZ / divisor;
	}else
	{
		divisor = xclk;
		currentxclk = 0;
	}

	switch (xclksel)
	{
		case 0:
		{
			cam->isp->isp_main.reg.tctrl_ctrl.bit.diva	= divisor;
		}break;

		case 1:
		{
			cam->isp->isp_main.reg.tctrl_ctrl.bit.divb	= divisor;
		}break;

		default:
		{
			return FAIL;
		}
	}

	if(current_xclk)
	{
		*current_xclk	= currentxclk;
	}

	return SUCCESS;
}
/************************************************************************************************************
 *  
 *  MODULE TYPE	:	FUNCTION				MODULE ID	:	OMAP_V4L2_BASE	
 *  Name	:	mclk_to_sensor
 *  Parameter1	:	cam_data *cam	- Base address of camera structure pointer
 *  Parameter2	:	option		- command to perform 
 *  
 *  Returns	:	FNRESLT		- On Success Zero (or) positive value be returned to the calling
 *  					  Functions and On error a negative value be returned
 *
 *  					  Note: 
 *  					  	For more detail about the return values please refer
 *  					  error.c and error.h file available in the current project
 *
 *  Description	: 	Function callback exposed to other layers.
 *  Comments	:  	
 ************************************************************************************************************/

FNRESLT mclk_to_sensor(cam_data *cam,UINT32 xclk,UPINT32 clk_set)
{
	FNRESLT ret_val;

	ret_val	= isp_set_xclk(cam,xclk,0,clk_set);
	if(CHECK_IN_FAIL_LIMIT(ret_val))
	{
		TRACE_ERR_AND_RET(FAIL);		
	}

	return SUCCESS;
}

/************************************************************************************************************
 *  
 *  MODULE TYPE	:	FUNCTION				MODULE ID	:	OMAP_V4L2_BASE	
 *  Name	:	omap_isp_base_struct
 *  Parameter1	:	cam_data *cam	- Base address of camera structure pointer
 *  Parameter2	:	option		- command to perform 
 *  
 *  Returns	:	FNRESLT		- On Success Zero (or) positive value be returned to the calling
 *  					  Functions and On error a negative value be returned
 *
 *  					  Note: 
 *  					  	For more detail about the return values please refer
 *  					  error.c and error.h file available in the current project
 *
 *  Description	: 	maintain the base pointer of pin configuration.
 *  Comments	:  	
 ************************************************************************************************************/

FNRESLT omap_isp_base_struct(cam_data *cam,UINT8 option)
{

/*
 * SET_ADDRESS 
 * GET_ADDRESS 
 * MAKE_ADDRESS_INVALID
 * CREATE_ADDRESS
 */
	static UINT32 g_cam_isp;

	if(cam == NULL)
	{
		TRACE_ERROR(MEMORY_NOT_VALID);	
		return MEMORY_NOT_VALID;
	}
	switch(option)
	{
		case SET_ADDRESS:
		{
			g_cam_isp =(UINT32)cam->isp;			
		}break;
		case GET_ADDRESS:
		{
			cam->isp	= (isp_reg_bit_access*)g_cam_isp;
		}break;
		case MAKE_ADDRESS_INVALID:
		{
			if(cam->isp	== NULL)
			{
				TRACE_ERR_AND_RET(FAIL);
			}
			iounmap(cam->isp);
			cam->isp	= NULL;
			g_cam_isp	= DISABLE;
		}break;
		case CREATE_ADDRESS:
		{
			cam->isp	= ioremap(BADDR_ISP,MAP_ISP_REGION);
			if(cam->isp	== NULL)
			{
				printk(KERN_ERR "Unable to remap the isp registers\n");
				TRACE_ERR_AND_RET(FAIL);
			}			
		}break;
		default:
		{
			TRACE_ERR_AND_RET(FAIL);
		}
	}
	return SUCCESS;
}

/************************************************************************************************************
 *  
 *  MODULE TYPE	:	FUNCTION				MODULE ID	:	OMAP_V4L2_BASE	
 *  Name	:	
 *  Parameter1	:	cam_data *cam	- Base address of camera structure pointer
 *  Returns	:	FNRESLT		- On Success Zero (or) positive value be returned to the calling
 *  					  Functions and On error a negative value be returned
 *
 *  					  Note: 
 *  					  	For more detail about the return values please refer
 *  					  error.c and error.h file available in the current project
 *
 *  Description	: 	reset the isp and ccdc interface
 *  Comments	:  	
 ************************************************************************************************************/
FNRESLT isp_reset(cam_data *cam)
{
	UINT32 time_out	= 10;
	if(cam == NULL)
	{
		TRACE_ERR_AND_RET(FAIL);
	}
	
/*
 * Clear all the status register and irq registers
 */
	cam->isp->isp_main.reg.isp_irq0enable.ISP_IRQ0ENABLE	= DISABLE;
	cam->isp->isp_main.reg.isp_irq0status.ISP_IRQ0STATUS	= cam->isp->isp_main.reg.isp_irq0status.ISP_IRQ0STATUS;

	cam->isp->isp_main.reg.isp_irq1enable.ISP_IRQ1ENABLE	= DISABLE;
	cam->isp->isp_main.reg.isp_irq1status.ISP_IRQ1STATUS	= cam->isp->isp_main.reg.isp_irq1status.ISP_IRQ1STATUS;

/*
 * Enable the soft reset
 */
	cam->isp->isp_main.reg.isp_sysconfig.bit.soft_reset	= ENABLE;

/*
 * Check the reset is done 
 */
	for(;time_out--;)
	{
		if(cam->isp->isp_main.reg.isp_sysstatus.bit.reset_done	== ENABLE)
		{
			break;
		}
		mdelay(100);
	}

	cam->isp->isp_main.reg.isp_sysconfig.bit.midle_mode	= ENABLE;
	cam->isp->isp_main.reg.isp_sysconfig.bit.auto_idle	= DISABLE;
	return SUCCESS;
}

/************************************************************************************************************
 *  
 *  MODULE TYPE	:	FUNCTION				MODULE ID	:	OMAP_V4L2_BASE	
 *  Name	:	
 *  Parameter1	:	cam_data *cam	- Base address of camera structure pointer
 *  Returns	:	FNRESLT		- On Success Zero (or) positive value be returned to the calling
 *  					  Functions and On error a negative value be returned
 *
 *  					  Note: 
 *  					  	For more detail about the return values please refer
 *  					  error.c and error.h file available in the current project
 *
 *  Description	: 	program the ccdc sdram address.
 *  Comments	:  	In the programed sdram address the new frame from the sensor be filled.
 ************************************************************************************************************/

FNRESLT program_dummy_isp_sdram_addr(cam_data *cam)
 {
	if(cam->capture.available_buf	<= DISABLE)
	{
		TRACE_ERR_AND_RET(FAIL);
	}

	cam->isp->isp_ccdc.reg.CCDC_SDR_ADDR	= cam->capture.frame[cam->capture.available_buf].buffer.m.offset;
 	return SUCCESS;
 }
/************************************************************************************************************
 *  
 *  MODULE TYPE	:	FUNCTION				MODULE ID	:	OMAP_V4L2_BASE	
 *  Name	:	
 *  Parameter1	:	cam_data *cam	- Base address of camera structure pointer
 *  Returns	:	FNRESLT		- On Success Zero (or) positive value be returned to the calling
 *  					  Functions and On error a negative value be returned
 *
 *  					  Note: 
 *  					  	For more detail about the return values please refer
 *  					  error.c and error.h file available in the current project
 *
 *  Description	: 	program the ccdc sdram address.
 *  Comments	:  	In the programed sdram address the new frame from the sensor be filled.
 ************************************************************************************************************/
FNRESLT isp_prg_sdram_addr(cam_data *cam)
{
	static struct timeval timestamp;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,34))
	struct tm timecode;
#endif

	if(cam == NULL)
	{
		TRACE_ERR_AND_RET(FAIL);
	}
/*
 * Take the current time stamp
 */
	do_gettimeofday(&timestamp);

	if(cam->task.bit.still	== ENABLE)
	{
		cam->isp->isp_ccdc.reg.CCDC_SDR_ADDR	= cam->still.phy_addr;
	}else if(cam->task.bit.capture == ENABLE)
	{
		cam->isp->isp_ccdc.reg.CCDC_SDR_ADDR			= cam->capture.processing->buffer.m.offset;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,34))
		time_to_tm(cam->capture.processing->buffer.timestamp.tv_sec,DISABLE, &timecode);
		cam->capture.processing->buffer.timecode.seconds	= timecode.tm_sec;
		cam->capture.processing->buffer.timecode.minutes	= timecode.tm_min;
		cam->capture.processing->buffer.timecode.hours		= timecode.tm_hour;
#if (SENS_MAX_FPS <= 30)
		cam->capture.processing->buffer.timecode.type		= V4L2_TC_TYPE_30FPS;
#else
		cam->capture.processing->buffer.timecode.type		= V4L2_TC_TYPE_60FPS;
#endif
		cam->capture.processing->buffer.timecode.flags		= V4L2_TC_FLAG_COLORFRAME;
		cam->capture.processing->buffer.timecode.frames		= cam->capture.buffer_sequence;
#endif
		cam->capture.processing->buffer.sequence		= cam->capture.buffer_sequence;

	}
	
	return SUCCESS;
}

/************************************************************************************************************
 *  
 *  MODULE TYPE	:	FUNCTION				MODULE ID	:	OMAP_V4L2_BASE	
 *  Name	:	
 *  Parameter1	:	cam_data *cam	- Base address of camera structure pointer
 *  Returns	:	FNRESLT		- On Success Zero (or) positive value be returned to the calling
 *  					  Functions and On error a negative value be returned
 *
 *  					  Note: 
 *  					  	For more detail about the return values please refer
 *  					  error.c and error.h file available in the current project
 *
 *  Description	: 	disable the irq0 interrupt
 *  Comments	:  	
 ************************************************************************************************************/
FNRESLT disable_isp_irq0(cam_data *cam)
{
	cam->isp->isp_main.reg.isp_irq0enable.ISP_IRQ0ENABLE	= DISABLE;
	return SUCCESS;
}

/************************************************************************************************************
 *  
 *  MODULE TYPE	:	FUNCTION				MODULE ID	:	OMAP_V4L2_BASE	
 *  Name	:	
 *  Parameter1	:	cam_data *cam	- Base address of camera structure pointer
 *  Returns	:	FNRESLT		- On Success Zero (or) positive value be returned to the calling
 *  					  Functions and On error a negative value be returned
 *
 *  					  Note: 
 *  					  	For more detail about the return values please refer
 *  					  error.c and error.h file available in the current project
 *
 *  Description	: 	Configure the isp side
 *  Comments	:  	
 ************************************************************************************************************/

FNRESLT isp_configure(cam_data *cam)
{
	if(cam == NULL)
	{
		TRACE_ERR_AND_RET(FAIL);
	}

	cam->isp->isp_main.reg.isp_ctrl.ISP_CTRL		= DISABLE;
	cam->isp->isp_main.reg.isp_ctrl.bit.ccdc_clk_en		= ENABLE;
	cam->isp->isp_main.reg.isp_ctrl.bit.par_bridge		= ISPM_ISP_CTRL_MEM_ORDER_MSB_LSB;

	cam->isp->isp_main.reg.isp_ctrl.bit.par_ser_clk_sel	= DISABLE;
	cam->isp->isp_main.reg.isp_ctrl.bit.ccdc_ram_en		= ENABLE;
	cam->isp->isp_main.reg.isp_ctrl.bit.sync_detect		= ISPM_ISP_CTRL_SYNC_DETECT_VS_FALL;
#if (CONFIG_ISP_DATA_LINE_SHIFT == ENABLE)
	cam->isp->isp_main.reg.isp_ctrl.bit.shift		= ISPM_ISP_CTRL_BIT_SHIFT_CAMEXT13_0_CAM13_0;
#elif (CONFIG_ISP_DATA_LINE_SHIFT == DISABLE)
	cam->isp->isp_main.reg.isp_ctrl.bit.shift		= ISPM_ISP_CTRL_BIT_SHIFT_CAMEXT13_2_CAM11_0;
#endif

//	cam->isp->isp_main.reg.isp_ctrl.bit.sbl_wr1_ram_en	= DISABLE;
//	cam->isp->isp_main.reg.isp_ctrl.bit.sbl_rd_ram_en	= DISABLE;

	/*
	 * Pixel clock polority
	 * 1. From camera side Falling edge the data will be put into the bus.
	 *    In omap side at raising edge the data will be taken form sampling.
	 */

	cam->isp->isp_main.reg.isp_ctrl.bit.par_clk_pol		= DISABLE;	
	cam->isp->isp_ccdc.reg.ccdc_hsize_off.bit.lnofst	= cam->capture.v2f.fmt.pix.bytesperline;

	cam->isp->isp_ccdc.reg.ccdc_cfg.bit.vdlc		= ENABLE;

	cam->isp->isp_ccdc.reg.ccdc_syn_mode.bit.vdhden		= ENABLE;
	cam->isp->isp_ccdc.reg.ccdc_syn_mode.bit.datsiz		= 0x0;
	cam->isp->isp_ccdc.reg.ccdc_syn_mode.bit.inpmod		= ISP_CCDC_CCDC_SYNC_MODE_IMPMOD_YUV_16BIT;
	cam->isp->isp_ccdc.reg.ccdc_syn_mode.bit.wen		= ENABLE;
	cam->isp->isp_ccdc.reg.ccdc_syn_mode.bit.exwen		= DISABLE;
	cam->isp->isp_ccdc.reg.ccdc_horz_info.bit.nph		= cam->capture.v2f.fmt.pix.width -1;

	cam->isp->isp_ccdc.reg.ccdc_vert_start.bit.slv0		= CONFIG_ISP_SLV0_DISCARD_COUNT;
	cam->isp->isp_ccdc.reg.ccdc_vert_start.bit.slv1		= DISABLE;

	switch(cam->cam_sensor.fmt.fmt.pix.pixelformat)	
	{
		case V4L2_PIX_FMT_YUV420:
		{
			cam->isp->isp_ccdc.reg.ccdc_vert_lines.bit.nlv	= (cam->capture.v2f.fmt.pix.height*3/4) -1;
		}break;

		default:
		{
			cam->isp->isp_ccdc.reg.ccdc_vert_lines.bit.nlv	= cam->capture.v2f.fmt.pix.height -1;
		}break;
	}
	return SUCCESS;
}
/************************************************************************************************************
 *  
 *  MODULE TYPE	:	FUNCTION				MODULE ID	:	OMAP_V4L2_BASE	
 *  Name	:	
 *  Parameter1	:	cam_data *cam	- Base address of camera structure pointer
 *  Returns	:	FNRESLT		- On Success Zero (or) positive value be returned to the calling
 *  					  Functions and On error a negative value be returned
 *
 *  					  Note: 
 *  					  	For more detail about the return values please refer
 *  					  error.c and error.h file available in the current project
 *
 *  Description	: 	After configuration isp irq0 is enabled
 *  Comments	:  	
 ************************************************************************************************************/
FNRESLT enable_isp_irq0(cam_data *cam)
{
	cam->isp->isp_ccdc.reg.ccdc_vdint.bit.vdint0		= cam->capture.v2f.fmt.pix.height -1;
	cam->isp->isp_main.reg.isp_irq0enable.ISP_IRQ0ENABLE	= DISABLE;
	cam->isp->isp_main.reg.isp_irq0enable.bit.ccdc_vd0_irq	= ENABLE;

	return SUCCESS;
}
/************************************************************************************************************
 *  
 *  MODULE TYPE	:	FUNCTION				MODULE ID	:	OMAP_V4L2_BASE	
 *  Name	:	
 *  Parameter1	:	cam_data *cam	- Base address of camera structure pointer
 *  Returns	:	FNRESLT		- On Success Zero (or) positive value be returned to the calling
 *  					  Functions and On error a negative value be returned
 *
 *  					  Note: 
 *  					  	For more detail about the return values please refer
 *  					  error.c and error.h file available in the current project
 *
 *  Description	: 	enable the ccdc unit.
 *  Comments	:  	Once ccdc unit is enabled it will copy the frame into programmed sdram address.
 ************************************************************************************************************/
FNRESLT enable_ccdc(cam_data *cam)
{
	if(cam == NULL)
	{
		TRACE_ERR_AND_RET(FAIL);
	}
	cam->isp->isp_ccdc.reg.ccdc_pcr.bit.enable		= ENABLE;
	return SUCCESS;
}

/************************************************************************************************************
 *  
 *  MODULE TYPE	:	FUNCTION				MODULE ID	:	OMAP_V4L2_BASE	
 *  Name	:	disable_ccdc
 *  Parameter1	:	cam_data *cam	- Base address of camera structure pointer
 *  Returns	:	FNRESLT		- On Success Zero (or) positive value be returned to the calling
 *  					  Functions and On error a negative value be returned
 *
 *  					  Note: 
 *  					  	For more detail about the return values please refer
 *  					  error.c and error.h file available in the current project
 *
 *  Description	: 	disable ccdc unit
 *  Comments	:  	
 ************************************************************************************************************/
FNRESLT disable_ccdc(cam_data *cam)
{
	if(cam == NULL)
	{
		TRACE_ERR_AND_RET(FAIL);
	}
	cam->isp->isp_ccdc.reg.ccdc_pcr.bit.enable		= DISABLE;
	return SUCCESS;
}

/************************************************************************************************************
 *  
 *  MODULE TYPE	:	FUNCTION				MODULE ID	:	OMAP_V4L2_BASE	
 *  Name	:	init_cam_isp_ccdc
 *  Parameter1	:	cam_data *cam	- Base address of camera structure pointer
  *  Returns	:	FNRESLT		- On Success Zero (or) positive value be returned to the calling
 *  					  Functions and On error a negative value be returned
 *
 *  					  Note: 
 *  					  	For more detail about the return values please refer
 *  					  error.c and error.h file available in the current project
 *
 *  Description	: 	init routine of ccdc done here
 *  Comments	:  	
 ************************************************************************************************************/
FNRESLT init_cam_isp_ccdc(cam_data *cam)
{
	FNRESLT ret_val;
/*
 * Validate the input
 */
	if(cam == NULL)
	{
		TRACE_ERR_AND_RET(FAIL);
	}

/*
 * map the physical address of isp registers 
 * to kernel virtual address
 */
#if (CONFIG_USE_TI_RESIZER == ENABLE)
	ret_val	= omap_isp_base_struct(cam,CREATE_ADDRESS);
	if(CHECK_IN_FAIL_LIMIT(ret_val))
	{
		printk(KERN_ERR "Failed to map the camera isp registers\n");
		TRACE_ERR_AND_RET(FAIL);		
	}
#endif
	ret_val	= omap_isp_base_struct(cam,SET_ADDRESS);
	if(CHECK_IN_FAIL_LIMIT(ret_val))
	{
		printk(KERN_ERR "Failed to map the camera isp registers\n");
		TRACE_ERR_AND_RET(FAIL);		
	}

	if(cam_mclk)
	{
		ret_val	= isp_set_xclk(cam,cam_mclk,0,NULL);
		if(CHECK_IN_FAIL_LIMIT(ret_val))
		{
			TRACE_ERR_AND_RET(FAIL);		
		}
	}
/*
 * Call back function for changing the mclk is assinged here
 */
	cam->modify_mclk_to_sensor	= mclk_to_sensor;
	
	return SUCCESS;
}

/************************************************************************************************************
 *  
 *  MODULE TYPE	:	FUNCTION				MODULE ID	:	OMAP_V4L2_BASE	
 *  Name	:	exit_cam_isp_ccdc
 *  Parameter1	:	cam_data *cam	- Base address of camera structure pointer
 *  Returns	:	FNRESLT		- On Success Zero (or) positive value be returned to the calling
 *  					  Functions and On error a negative value be returned
 *
 *  					  Note: 
 *  					  	For more detail about the return values please refer
 *  					  error.c and error.h file available in the current project
 *
 *  Description	: 	Perform cleanup routine done here
 *  Comments	:  	
 ************************************************************************************************************/
FNRESLT exit_cam_isp_ccdc(cam_data *cam)
{
	FNRESLT ret_val;
/*
 * Validate the input
 */
	if(cam == NULL)
	{
		TRACE_ERR_AND_RET(FAIL);
	}
/*
 * unmap the kernel space and 
 * Perform cleaning of structure
 */

	ret_val	= omap_isp_base_struct(cam,MAKE_ADDRESS_INVALID);
	if(CHECK_IN_FAIL_LIMIT(ret_val))
	{
		printk(KERN_ERR "Failed to unmap the camera isp registers\n");
		TRACE_ERR_AND_RET(FAIL);		
	}

	return SUCCESS;
}
