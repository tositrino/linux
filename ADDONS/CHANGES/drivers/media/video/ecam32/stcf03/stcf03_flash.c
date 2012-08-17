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
 *   MODULE NAME           :        OV3640                                                                *
 *   MODULE VERSION        :        VER 1.0                                                               *
 *                                                                                                        *
 *                                                                                                        *
 *                                                                                                        *
 *   Version No	: 000-0001                                                          CODE_REV  : 0.0.0.0   *
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
 *==========================================================================================================
 */


/*
 * ----------------------------------------------------------------------------------------------------------
 *
 * 					Header files inclusion part
 *
 * ----------------------------------------------------------------------------------------------------------
 */
#include "../inc_header.h"
/*
 * local variables
 */
static INT32 torch_lum_value;
static INT32 flash_lum_value;

/************************************************************************************************************
 *  
 *  MODULE TYPE	:	FUNCTION				MODULE ID	:	
 *  Name	:	flash_i2c_client_xfer
 *  Parameter1	:	INT32 addr
 *  Parameter2	:	UPINT8 reg
 *  Parameter3	:	PINT8 buf
 *  Parameter4	:	INT32 num
 *  Parameter5	:	INT32 tran_flag
 *  Returns	:	FNRESLT		- On Success Zero (or) positive value be returned to the calling
 *  					  Functions and On error a negative value be returned
 *
 *  					  Note: 
 *  					  	For more detail about the return values please refer
 *  					  error.c and error.h file available in the current project
 *  Description	: 	
 *  Comments	:  	
 ************************************************************************************************************/
FNRESLT flash_i2c_client_xfer(INT32 addr, UPINT8 reg, PINT8 buf, INT32 num, INT32 tran_flag)
{

	struct i2c_msg msg[2];
	FNRESLT ret_val;
	INT32 ret;
	cam_data *cam	= NULL;

	ret_val	= v4l2_base_struct(&cam,GET_ADDRESS);
	if(CHECK_IN_FAIL_LIMIT(ret_val))
	{
		TRACE_ERR_AND_RET(FAIL);	
	}

/*
 * FIXME:
 * 	
 * 	I2C Write:
 *	 	In i2c msg[0] "address part in write is success" but if we put data in msg [1] 
 * 		that is not properly sent to device.
 *
 * 		so in the msg[0] part itself the data also sent here.
 *
 * 	I2CRead:
 * 		But in read No problem is found and working fine
 */

	if(tran_flag & I2C_FLAG_READ)
	{

		msg[0].addr	= addr;
		msg[0].len	= 1;
		msg[0].buf	= reg;
		msg[0].flags	= tran_flag;
		msg[0].flags	&= ~I2C_M_RD;

		msg[1].addr	= addr;
		msg[1].len	= num;
		msg[1].buf	= buf;
		msg[1].flags	= tran_flag;

		if (tran_flag & I2C_FLAG_READ)
		{
			msg[1].flags |= I2C_M_RD;
		}else
		{
			msg[1].flags &= ~I2C_M_RD;
		}

		if (cam->cam_flash.client->adapter == NULL)
		{
			printk("%s:adapter error\n", __func__);
			return -1;
		}

		ret = i2c_transfer(cam->cam_flash.client->adapter, msg, 2);
		if (ret >= 0)
		{
			/* printk("%s:i2c transfer num:%d\n", __func__, ret); */
			return SUCCESS;
		}

	}else
	{
		UINT8 reg_addr_data[2];
		
		reg_addr_data[0] = *reg;
		reg_addr_data[1] = *buf;
		
		msg[0].addr	= addr;
		msg[0].len	= 2;
		msg[0].buf	= reg_addr_data;
		msg[0].flags	= tran_flag;
		msg[0].flags	&= ~I2C_M_RD;

		ret = i2c_transfer(cam->cam_flash.client->adapter, msg, 1);
		if (ret >= 0)
		{
			/* printk("%s:i2c transfer num:%d\n", __func__, ret); */
			return SUCCESS;
		}
	}
	printk("%s:i2c transfer error:%d\n", __func__, ret);
	return FAIL;

}
/************************************************************************************************************
 *  
 *  MODULE TYPE	:	FUNCTION				MODULE ID	:	
 *  Name	:	stcf03_write
 *  Parameter1	:	UINT8 reg
 *  Parameter2	:	UINT8 data
 *  Returns	:	FNRESLT		- On Success Zero (or) positive value be returned to the calling
 *  					  Functions and On error a negative value be returned
 *
 *  					  Note: 
 *  					  	For more detail about the return values please refer
 *  					  error.c and error.h file available in the current project
 *  Description	: 	
 *  Comments	:  	
 ************************************************************************************************************/
FNRESLT stcf03_write(UINT8 reg,UINT8 data)
{
	printk(KERN_DEBUG "stcf03: write : Address : %x Data %x \n",reg,data);

	if (flash_i2c_client_xfer(STCF03_SLAVE_ADDRESS,&reg,&data, 0, 0) < 0)
	{
		printk("Flash write failed :%s: reg=%x",__func__, reg);
		return FAIL;
	}	
	return SUCCESS;
}
/************************************************************************************************************
 *  
 *  MODULE TYPE	:	FUNCTION				MODULE ID	:	
 *  Name	:	stcf03_read
 *  Parameter1	:	UINT8 reg
 *  Parameter2	:	UPINT8 data
 *  Returns	:	FNRESLT		- On Success Zero (or) positive value be returned to the calling
 *  					  Functions and On error a negative value be returned
 *
 *  					  Note: 
 *  					  	For more detail about the return values please refer
 *  					  error.c and error.h file available in the current project
 *  Description	: 	
 *  Comments	:  	
 ************************************************************************************************************/
FNRESLT stcf03_read(UINT8 reg,UPINT8 data)
{
	if (flash_i2c_client_xfer(STCF03_SLAVE_ADDRESS,&reg, data, 1, 1) < 0)
	{
		printk("Flash write failed :%s: reg=%x",__func__, reg);
		return FAIL;
	}
	printk(KERN_DEBUG "stcf03: read : Address : %x Data %x \n",reg,*data);
	return SUCCESS;
}

/************************************************************************************************************
 *  
 *  MODULE TYPE	:	FUNCTION				MODULE ID	:	
 *  Name	:	stcf03_probe
 *  Parameter1	:	struct i2c_client *client
 *  Parameter2	:	const struct i2c_device_id *id
 *  Returns	:	LINT32	- On sucess returns 0
 *  				- On Failure a negative number be returned
 *
 *  Description	: 	Configure the gpio levels for ov3640 driver
 *  Comments	:  	
 ************************************************************************************************************/
static INT32 __init stcf03_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	cam_data *cam	= NULL;
	FNRESLT ret_val;

	if (i2c_get_clientdata(client))
	{
		TRACE_ERR_AND_RET(FAIL);
	}

	ret_val	= v4l2_base_struct(&cam,GET_ADDRESS);
	if(CHECK_IN_FAIL_LIMIT(ret_val))
	{
		printk(KERN_ERR "Failed to register the camera device\n");
		goto exit;
	}
	
/*
 * 	set the client data 
 */
	cam->cam_flash.client =	client;

	return 0;
	exit:
	{
		return -EINVAL;
	}	
}

/************************************************************************************************************
 *  
 *  MODULE TYPE	:	FUNCTION				MODULE ID	:	
 *  Name	:	stcf03_remove
 *  Parameter1	:	struct i2c_client *client
 *  Returns	:	LINT32	- On sucess returns 0
 *  				- On Failure a negative number be returned
 *
 *  Description	: 	remove routine of ov3640 i2c driver
 *  Comments	:  	
 ************************************************************************************************************/

static INT32 __exit stcf03_remove(struct i2c_client *client)
{
	if (!client->adapter)
		return -ENODEV;	/* our client isn't attached */

	i2c_set_clientdata(client, NULL);
	return SUCCESS;
}

/************************************************************************************************************
 *  
 *  MODULE TYPE	:	FUNCTION				MODULE ID	:	
 *  Name	:	stcf03_init
 *  Parameter1	:	cam_data *cam
 *  Returns	:	FNRESLT		- On Success Zero (or) positive value be returned to the calling
 *  					  Functions and On error a negative value be returned
 *
 *  					  Note: 
 *  					  	For more detail about the return values please refer
 *  					  error.c and error.h file available in the current project
 *  Description	: 	
 *  Comments	:  	
 ************************************************************************************************************/
FNRESLT stcf03_init(cam_data *cam)
{
	FNRESLT ret_val;
/*
 * i2c driver init
 */
	static const struct i2c_device_id stcf03_id[] =			\
	{
		{ STCF03_DRIVER_NAME, 0 },
		{ },
	};
	MODULE_DEVICE_TABLE(i2c, stcf03_id);
	
	cam->cam_flash.i2c.driver.name	= STCF03_DRIVER_NAME;
	cam->cam_flash.i2c.driver.owner	= THIS_MODULE;
	cam->cam_flash.i2c.probe	= stcf03_probe;
	cam->cam_flash.i2c.remove	= __exit_p(stcf03_remove);
	cam->cam_flash.i2c.id_table	= stcf03_id;

	if(i2c_add_driver(&cam->cam_flash.i2c))
	{
		TRACE_ERR_AND_RET(FAIL);
	}

	if(cam->cam_flash.client	== NULL)
	{
		ret_val	= cam->cam_flash.exit(cam);
		if(CHECK_IN_FAIL_LIMIT(ret_val))
		{
			TRACE_ERR_AND_RET(FAIL);
		}
		return SUCCESS;
	}

	return SUCCESS;
}

/************************************************************************************************************
 *  
 *  MODULE TYPE	:	FUNCTION				MODULE ID	:	
 *  Name	:	stcf03_exit
 *  Parameter1	:	cam_data *cam
 *  Returns	:	FNRESLT		- On Success Zero (or) positive value be returned to the calling
 *  					  Functions and On error a negative value be returned
 *
 *  					  Note: 
 *  					  	For more detail about the return values please refer
 *  					  error.c and error.h file available in the current project
 *  Description	: 	
 *  Comments	:  	
 ************************************************************************************************************/
FNRESLT stcf03_exit(cam_data *cam)
{
	if(cam->cam_flash.client)
	{
		stcf03_write(0x00,0x00);		
	}

	i2c_del_driver(&cam->cam_flash.i2c);
	memset(&cam->cam_flash,0x00,sizeof(struct _flash_driver));
	return SUCCESS;
}

/************************************************************************************************************
 *  
 *  MODULE TYPE	:	FUNCTION				MODULE ID	:	
 *  Name	:	stcf03_flash
 *  Parameter1	:	cam_data *cam
 *  Returns	:	FNRESLT		- On Success Zero (or) positive value be returned to the calling
 *  					  Functions and On error a negative value be returned
 *
 *  					  Note: 
 *  					  	For more detail about the return values please refer
 *  					  error.c and error.h file available in the current project
 *  Description	: 	
 *  Comments	:  	
 ************************************************************************************************************/
FNRESLT stcf03_flash(cam_data *cam)
{
	FNRESLT ret_val;
	switch(cam->cmd)
	{
		case GET_DATA:
		{
			cam->ctrl.value	= cam->cam_sensor.sens_strobe_en;
		}break;

		case SET_DATA:
		{
			if((cam->ctrl.value >= 0) && (cam->ctrl.value <= 1))
			{
				if(cam->ctrl.value)
				{
/*
 * Flash (or) Torch can be enabled, So proper routine to disable torch
 */
					cam->cmd	= SET_DATA;
					cam->ctrl.value	= DISABLE;
					ret_val = cam->cam_flash.torch(cam);
					if(CHECK_IN_FAIL_LIMIT(ret_val))
					{
						TRACE_ERR_AND_RET(FAIL);
					}

					cam->cam_sensor.sens_strobe_en	= ENABLE;
					ret_val = stcf03_write(0x00,0xCF);
					if(CHECK_IN_FAIL_LIMIT(ret_val))
					{
						TRACE_ERR_AND_RET(FAIL);
					}
				}else
				{
					cam->cam_sensor.sens_strobe_en	= DISABLE;
				}
			}
		}break;

		case QUERY_DATA:
		{
			cam->qctrl.id	= V4L2_SENS_FLASH_FLASH;
			cam->qctrl.type	= V4L2_CTRL_TYPE_BOOLEAN;
			strncpy(cam->qctrl.name,"flash ctrl",strlen("flash ctrl"));
			cam->qctrl.minimum = 0;
			cam->qctrl.maximum = 1;
			cam->qctrl.step = 1;
			cam->qctrl.default_value = 0;
		}break;
	}
	return SUCCESS;
}

/************************************************************************************************************
 *  
 *  MODULE TYPE	:	FUNCTION				MODULE ID	:	
 *  Name	:	stcf03_torch
 *  Parameter1	:	cam_data *cam	- Base address of camera structure pointer
 *  Returns	:	FNRESLT		- On Success Zero (or) positive value be returned to the calling
 *  					  Functions and On error a negative value be returned
 *
 *  					  Note: 
 *  					  	For more detail about the return values please refer
 *  					  error.c and error.h file available in the current project
 *  Description	: 	
 *  Comments	:  	
 ************************************************************************************************************/
FNRESLT stcf03_torch(cam_data *cam)
{
	FNRESLT ret_val;
	static UINT32 torch_value;
	switch(cam->cmd)
	{
		case GET_DATA:
		{
			cam->ctrl.value	= torch_value;
		}break;

		case SET_DATA:
		{
			if((cam->ctrl.value >= 0) && (cam->ctrl.value <= 1))
			{
				if(cam->ctrl.value)
				{
/*
 * Flash (or) Torch can be enabled, So proper routine to disable flash
 */
					cam->cmd	= SET_DATA;
					cam->ctrl.value	= DISABLE;
					ret_val = cam->cam_flash.flash(cam);
					if(CHECK_IN_FAIL_LIMIT(ret_val))
					{
						TRACE_ERR_AND_RET(FAIL);
					}

					torch_value	= ENABLE;
					ret_val=stcf03_write(0x00,0xAF);
					if(CHECK_IN_FAIL_LIMIT(ret_val))
					{
						TRACE_ERR_AND_RET(FAIL);
					}
				}else
				{
					if(cam->cam_sensor.sens_strobe_en	== DISABLE)
					{
						ret_val = stcf03_write(0x00,0x00);
						if(CHECK_IN_FAIL_LIMIT(ret_val))
						{
							TRACE_ERR_AND_RET(FAIL);
						}
					}
					torch_value	= DISABLE;
				}
			}
		}break;

		case QUERY_DATA:
		{
			cam->qctrl.id	= V4L2_SENS_FLASH_TORCH;
			cam->qctrl.type	= V4L2_CTRL_TYPE_BOOLEAN;
			strncpy(cam->qctrl.name,"torch ctrl",strlen("torch ctrl"));
			cam->qctrl.minimum = 0;
			cam->qctrl.maximum = 1;
			cam->qctrl.step = 1;
			cam->qctrl.default_value = 0;
		}break;
	}
	return SUCCESS;

}
/************************************************************************************************************
 *  
 *  MODULE TYPE	:	FUNCTION				MODULE ID	:	
 *  Name	:	stcf03_lum_ctrl
 *  Parameter1	:	cam_data *cam	- Base address of camera structure pointer
 *  Returns	:	FNRESLT		- On Success Zero (or) positive value be returned to the calling
 *  					  Functions and On error a negative value be returned
 *
 *  					  Note: 
 *  					  	For more detail about the return values please refer
 *  					  error.c and error.h file available in the current project
 *  Description	: 	
 *  Comments	:  	
 ************************************************************************************************************/
FNRESLT stcf03_flash_lum_ctrl(cam_data *cam)
{
	FNRESLT ret_val;
	switch(cam->cmd)
	{
		case GET_DATA:
		{
			cam->ctrl.value	= flash_lum_value;
		}break;

		case SET_DATA:
		{
			if((cam->ctrl.value >= 0) && (cam->ctrl.value <= 15))
			{
				ret_val	= stcf03_write(0x01,(0xF0 & torch_lum_value) | (0xF & cam->ctrl.value));
				if(CHECK_IN_FAIL_LIMIT(ret_val))
				{
					TRACE_ERR_AND_RET(FAIL);
				}
				flash_lum_value	= cam->ctrl.value;
			}

		}break;

		case QUERY_DATA:
		{
			cam->qctrl.id	= V4L2_SENS_FLASH_FLASH_LUM;
			cam->qctrl.type	= V4L2_CTRL_TYPE_INTEGER;
			strncpy(cam->qctrl.name,"flash lum ctrl",strlen("flash lum ctrl"));
			cam->qctrl.minimum = 0;
			cam->qctrl.maximum = 15;
			cam->qctrl.step = 1;
			cam->qctrl.default_value = 0;
			cam->qctrl.flags = V4L2_CTRL_FLAG_SLIDER;
		}break;
	}
	return SUCCESS;
}
/************************************************************************************************************
 *  
 *  MODULE TYPE	:	FUNCTION				MODULE ID	:	
 *  Name	:	stcf03_lum_ctrl
 *  Parameter1	:	cam_data *cam	- Base address of camera structure pointer
 *  Returns	:	FNRESLT		- On Success Zero (or) positive value be returned to the calling
 *  					  Functions and On error a negative value be returned
 *
 *  					  Note: 
 *  					  	For more detail about the return values please refer
 *  					  error.c and error.h file available in the current project
 *  Description	: 	
 *  Comments	:  	
 ************************************************************************************************************/
FNRESLT stcf03_torch_lum_ctrl(cam_data *cam)
{
	FNRESLT ret_val;
	switch(cam->cmd)
	{
		case GET_DATA:
		{
			cam->ctrl.value	= torch_lum_value;
		}break;

		case SET_DATA:
		{
			if((cam->ctrl.value >= 0) && (cam->ctrl.value <= 15))
			{
				ret_val = stcf03_write(0x01,(0x0F & flash_lum_value) | ((0xF & cam->ctrl.value)<<4));
				if(CHECK_IN_FAIL_LIMIT(ret_val))
				{
					TRACE_ERR_AND_RET(FAIL);
				}
				torch_lum_value	= cam->ctrl.value;
			}
		}break;

		case QUERY_DATA:
		{
			cam->qctrl.id	= V4L2_SENS_FLASH_TORCH_LUM;
			cam->qctrl.type	= V4L2_CTRL_TYPE_INTEGER;
			strncpy(cam->qctrl.name,"torch lum ctrl",strlen("torch lum ctrl"));
			cam->qctrl.minimum = 0;
			cam->qctrl.maximum = 15;
			cam->qctrl.step = 1;
			cam->qctrl.default_value = 0;
			cam->qctrl.flags = V4L2_CTRL_FLAG_SLIDER;
		}break;
	}
	return SUCCESS;
}

/************************************************************************************************************
 *  
 *  MODULE TYPE	:	FUNCTION				MODULE ID	:	
 *  Name	:	register_flash_driver
 *  Parameter1	:	cam_data *cam	- Base address of camera structure pointer
 *  Returns	:	FNRESLT		- On Success Zero (or) positive value be returned to the calling
 *  					  Functions and On error a negative value be returned
 *
 *  					  Note: 
 *  					  	For more detail about the return values please refer
 *  					  error.c and error.h file available in the current project
 *  Description	: 	
 *  Comments	:  	
 ************************************************************************************************************/
FNRESLT register_flash_driver(cam_data *cam)
{
	cam->cam_flash.init		= stcf03_init;
	cam->cam_flash.exit		= stcf03_exit;
	cam->cam_flash.flash		= stcf03_flash;
	cam->cam_flash.torch		= stcf03_torch;
	cam->cam_flash.flash_lum_ctrl	= stcf03_flash_lum_ctrl;
	cam->cam_flash.torch_lum_ctrl	= stcf03_torch_lum_ctrl;
	return SUCCESS;
}
