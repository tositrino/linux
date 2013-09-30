/* linux/drivers/media/video/dohofpga1.c
 *
 * Copyright (c) 2013 institute for visual computing, ETH Zuerich
 * http://ivc.ethz.ch/
 *
 * based on mt9m113.c
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

#define __DOHOFPGA1_C__

#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/version.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/dohofpga1_platform.h>

#ifdef CONFIG_VIDEO_SAMSUNG_V4L2
#include <linux/videodev2_samsung.h>
#endif

#include "dohofpga1.h"

/* Default resolution & pixelformat. plz ref dohofpga1_platform.h */
#define DEFAULT_RES    WVGA  /* Index of resoultion */
#define DEFAUT_FPS_INDEX  DOHOFPGA1_15FPS
#define DEFAULT_FMT    V4L2_PIX_FMT_UYVY//  /* YUV422 */

#define DOHOFPGA1_JPEG_MAXSIZE  0x3A0000
#define DOHOFPGA1_THUMB_MAXSIZE  0xFC00
#define DOHOFPGA1_POST_MAXSIZE  0xBB800

// code

static inline struct dohofpga1_state *to_state(struct v4l2_subdev *sd)
{
  return container_of(sd, struct dohofpga1_state, sd);
}

/*
 * DOHOFPGA1 register structure : 2bytes address, 2bytes value
 * retry on write failure up-to 5 times
 */

static inline int dohofpga1_write(struct v4l2_subdev *sd, u16 addr, u16 val)
{
  struct i2c_client *client = v4l2_get_subdevdata(sd);
  struct i2c_msg msg[1];
  unsigned char reg[4];
  int err   = 0;
  int retry = 1;

  if (!client->adapter) return -ENODEV;

  while(retry)
  {
    retry--;
    msg->addr = client->addr;
    msg->flags = 0;
    msg->len = 4;
    msg->buf = reg;

    reg[0] = addr >> 8;
    reg[1] = addr & 0xff;
    reg[2] = val >> 8;
    reg[3] = val & 0xff;

    err = i2c_transfer(client->adapter, msg, 1);
    if (err >= 0 ) return err ;
    dev_err(&client->dev, "%s: address: 0x%02x%02x, " \
            "value: 0x%02x%02x error..\n", __func__, \
            reg[0], reg[1], reg[2], reg[3]);
  }
  return err;
}

static int dohofpga1_i2c_write(struct v4l2_subdev *sd, unsigned char i2c_data[],
        unsigned char length)
{
  struct i2c_client *client = v4l2_get_subdevdata(sd);
  unsigned char buf[length], i;
  struct i2c_msg msg = {client->addr, 0, length, buf};

  for (i = 0; i < length; i++) buf[i] = i2c_data[i];

  return i2c_transfer(client->adapter, &msg, 1) == 1 ? 0 : -EIO;
}

const char * const *dohofpga1_ctrl_get_menu(u32 id)
{
  switch (id) 
  {
    case V4L2_CID_WHITE_BALANCE_PRESET:
      return dohofpga1_querymenu_wb_preset;
    case V4L2_CID_COLORFX:
      return dohofpga1_querymenu_effect_mode;
    case V4L2_CID_EXPOSURE:
      return dohofpga1_querymenu_ev_bias_mode;
    default:
      return v4l2_ctrl_get_menu(id);
  }
}

static inline struct v4l2_queryctrl const *dohofpga1_find_qctrl(int id)
{
  int i;
  for (i = 0; i < ARRAY_SIZE(dohofpga1_controls); i++)
    if (dohofpga1_controls[i].id == id)
      return &dohofpga1_controls[i];
  return NULL;
}

static int dohofpga1_queryctrl(struct v4l2_subdev *sd, struct v4l2_queryctrl *qc)
{
  int i;
  for (i = 0; i < ARRAY_SIZE(dohofpga1_controls); i++) 
  {
    if (dohofpga1_controls[i].id == qc->id) 
    {
      memcpy(qc, &dohofpga1_controls[i], \
             sizeof(struct v4l2_queryctrl));
      return 0;
    }
  }
  return -EINVAL;
}


static int dohofpga1_querymenu(struct v4l2_subdev *sd, struct v4l2_querymenu *qm)
{
  struct v4l2_queryctrl qctrl;
  qctrl.id = qm->id;
  dohofpga1_queryctrl(sd, &qctrl);
  return v4l2_ctrl_query_menu(qm, &qctrl, dohofpga1_ctrl_get_menu(qm->id));
}

/*
 * Clock configuration
 * Configure expected MCLK from host and return EINVAL if not supported clock
 * frequency is expected
 *  freq : in Hz
 *  flag : not supported for now
 */
static int dohofpga1_s_crystal_freq(struct v4l2_subdev *sd, u32  freq, u32 flags)
{
  int err = -EINVAL;
  return err;
}

static int dohofpga1_g_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *fmt)
{
  struct dohofpga1_state *state = to_state(sd);
  int err = 0;
  *fmt = state->fmt;
  return err;
}

static int dohofpga1_s_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *fmt)
{
  struct i2c_client *client = v4l2_get_subdevdata(sd);
  struct dohofpga1_state *state = to_state(sd);
  int err = 0;

  dev_dbg(&client->dev, "requested res(%d, %d)\n",
          fmt->width, fmt->height);

  printk( "requested res(%d, %d)\n",fmt->width, fmt->height);
  if((fmt->width == 640) && (fmt->height == 480))
  {
    err = __dohofpga1_init_2byte(sd, \
      (unsigned short *) set_resol_640x480, DOHOFPGA1_set_resol);
    err = __dohofpga1_init_2byte(sd, \
      (unsigned short *) Refresh, DOHOFPGA1_Refresh);
    printk( "set resolution %dx%d done.\n",fmt->width,fmt->height);
    state->fmt = *fmt;
  }
  else if((fmt->width == 1280) && (fmt->height == 720))
  {
    err = __dohofpga1_init_2byte(sd, \
      (unsigned short *) set_resol_1280x720, DOHOFPGA1_set_resol);
    err = __dohofpga1_init_2byte(sd, \
      (unsigned short *) Refresh, DOHOFPGA1_Refresh);
    printk( "set resolution %dx%d done.\n",fmt->width,fmt->height);
    state->fmt = *fmt;
  }

  /*
  if (!state->fmt.width ||
    !state->fmt.height ||
    !state->fmt.code)
    state->fmt = *fmt;
  else
    *fmt = state->fmt;
  */
  return err;
}

static int dohofpga1_enum_framesizes(struct v4l2_subdev *sd,
          struct v4l2_frmsizeenum *fsize)
{
  int err = 0;
  return err;
}

static int dohofpga1_enum_frameintervals(struct v4l2_subdev *sd,
          struct v4l2_frmivalenum *fival)
{
  int err = 0;
  return err;
}

static int dohofpga1_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
  int err = 0;
  return err;
}

static int dohofpga1_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
  int err = 0;
  return err;
}

static int dohofpga1_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
  struct i2c_client *client = v4l2_get_subdevdata(sd);
  struct dohofpga1_state *state = to_state(sd);
  struct dohofpga1_userset userset = state->userset;
  int err = -EINVAL;

  switch (ctrl->id) 
  {
    case V4L2_CID_CAM_JPEG_MEMSIZE:
      ctrl->value = DOHOFPGA1_JPEG_MAXSIZE +
        DOHOFPGA1_THUMB_MAXSIZE + DOHOFPGA1_POST_MAXSIZE;
      err = 0;
      break;
    case V4L2_CID_EXPOSURE:
      ctrl->value = userset.exposure_bias;
      err = 0;
      break;
    case V4L2_CID_AUTO_WHITE_BALANCE:
      ctrl->value = userset.auto_wb;
      err = 0;
      break;
    case V4L2_CID_WHITE_BALANCE_PRESET:
      ctrl->value = userset.manual_wb;
      err = 0;
      break;
    case V4L2_CID_WHITE_BALANCE_TEMPERATURE:
      ctrl->value = userset.wb_temp;
      err = 0;
      break;
    case V4L2_CID_COLORFX:
      ctrl->value = userset.effect;
      err = 0;
      break;
    case V4L2_CID_CONTRAST:
      ctrl->value = userset.contrast;
      err = 0;
      break;
    case V4L2_CID_SATURATION:
      ctrl->value = userset.saturation;
      err = 0;
      break;
    case V4L2_CID_SHARPNESS:
      ctrl->value = userset.saturation;
      err = 0;
      break;
    default:
      dev_err(&client->dev, "%s: no such ctrl\n", __func__);
      break;
  }

  return err;
}


static int dohofpga1_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
#ifdef DOHOFPGA1_COMPLETE
  struct i2c_client *client = v4l2_get_subdevdata(sd);
  struct dohofpga1_state *state = to_state(sd);
  struct dohofpga1_userset userset = state->userset;
  int err = -EINVAL;

  switch (ctrl->id) 
  {
    case V4L2_CID_EXPOSURE:
      dev_dbg(&client->dev, "%s: V4L2_CID_EXPOSURE\n", \
        __func__);
      err = dohofpga1_write_regs(sd, dohofpga1_regs_ev_bias[ctrl->value]);
      break;
    case V4L2_CID_AUTO_WHITE_BALANCE:
      dev_dbg(&client->dev, "%s: V4L2_CID_AUTO_WHITE_BALANCE\n", \
        __func__);
      err = dohofpga1_write_regs(sd, \
        dohofpga1_regs_awb_enable[ctrl->value]);
      break;
    case V4L2_CID_WHITE_BALANCE_PRESET:
      dev_dbg(&client->dev, "%s: V4L2_CID_WHITE_BALANCE_PRESET\n", \
        __func__);
      err = dohofpga1_write_regs(sd, \
        dohofpga1_regs_wb_preset[ctrl->value]);
      break;
    case V4L2_CID_WHITE_BALANCE_TEMPERATURE:
      dev_dbg(&client->dev, \
        "%s: V4L2_CID_WHITE_BALANCE_TEMPERATURE\n", __func__);
      err = dohofpga1_write_regs(sd, \
        dohofpga1_regs_wb_temperature[ctrl->value]);
      break;
    case V4L2_CID_COLORFX:
      dev_dbg(&client->dev, "%s: V4L2_CID_COLORFX\n", __func__);
      err = dohofpga1_write_regs(sd, \
        dohofpga1_regs_color_effect[ctrl->value]);
      break;
    case V4L2_CID_CONTRAST:
      dev_dbg(&client->dev, "%s: V4L2_CID_CONTRAST\n", __func__);
      err = dohofpga1_write_regs(sd, \
        dohofpga1_regs_contrast_bias[ctrl->value]);
      break;
    case V4L2_CID_SATURATION:
      dev_dbg(&client->dev, "%s: V4L2_CID_SATURATION\n", __func__);
      err = dohofpga1_write_regs(sd, \
        dohofpga1_regs_saturation_bias[ctrl->value]);
      break;
    case V4L2_CID_SHARPNESS:
      dev_dbg(&client->dev, "%s: V4L2_CID_SHARPNESS\n", __func__);
      err = dohofpga1_write_regs(sd, \
        dohofpga1_regs_sharpness_bias[ctrl->value]);
      break;
    default:
      dev_err(&client->dev, "%s: no such control\n", __func__);
      break;
  }
  if (err < 0)
    goto out;
  else
    return 0;

  out:
    dev_dbg(&client->dev, "%s: vidioc_s_ctrl failed\n", __func__);
    return err;
  #else
    return 0;
  #endif
}


static int __dohofpga1_init_2byte(struct v4l2_subdev *sd, 
                                 unsigned short reg[], int total)
{
#ifdef __DOHOFPGA1_REGWRITE__
  struct i2c_client *client = v4l2_get_subdevdata(sd);
  int err = -EINVAL, i;
  unsigned short *item;

  for (i = 0; i < total ; i+=2) 
  {
    item = (unsigned short *) &reg[i];
    if (item[0] == REG_DELAY) 
    {
      mdelay(item[1]);
      err = 0;
    } 
    else 
    {
      err = dohofpga1_write(sd, item[0], item[1]);
    }

    if (err < 0)
    {
      v4l_info(client, "%s: register set failed\n",__func__);
      printk("%s: register set failed\n", __func__);
    }
  }
  return err;
#else
  return 0;
#endif
}

static int dohofpga1_init(struct v4l2_subdev *sd, u32 val)
{
  struct i2c_client *client = v4l2_get_subdevdata(sd);
  int err = -EINVAL;

  v4l_info(client, "%s: camera initialization start\n", __func__);

  err = __dohofpga1_init_2byte(sd, \
        (unsigned short *) dohofpga1_init0, DOHOFPGA1_INIT0);

  if (err < 0) 
  {
    v4l_info(client, "%s: camera initialization failed\n",__func__);
    //return -EIO;  /* FIXME */
  }
  return 0;
}

static int dohofpga1_s_power(struct v4l2_subdev *sd, int on)
{
  struct i2c_client *client = v4l2_get_subdevdata(sd);
  struct dohofpga1_state *state = to_state(sd);
  struct dohofpga1_mbus_platform_data *pdata = state->pdata;
  int ret;

  printk("dohofpga1_s_power()\n") ;
  /* bug report */
  BUG_ON(!pdata);
  if(pdata->set_clock) 
  {
    ret = pdata->set_clock(&client->dev, on);
    if(ret)
      return -EIO;
  }

  /* setting power */
  if(pdata->set_power) 
  {
    ret = pdata->set_power(on);
    if (ret)
      return -EIO;
    if(on)
      return dohofpga1_init(sd, 0);
  }
  return 0;
}

static int dohofpga1_sleep(struct v4l2_subdev *sd)
{
  struct i2c_client *client = v4l2_get_subdevdata(sd);
  int err = -EINVAL, i;

  v4l_info(client, "%s: sleep mode\n", __func__);

  for (i = 0; i < DOHOFPGA1_SLEEP_REGS; i++) 
  {
    if (dohofpga1_sleep_reg[i][0] == REG_DELAY) 
    {
      mdelay(dohofpga1_sleep_reg[i][1]);
      err = 0;
    } else 
    {
      err = dohofpga1_write(sd, dohofpga1_sleep_reg[i][0], \
                           dohofpga1_sleep_reg[i][1]);
    }
    if (err < 0)
      v4l_info(client, "%s: register set failed\n", __func__);
  }

  if (err < 0) 
  {
    v4l_err(client, "%s: sleep failed\n", __func__);
    return -EIO;
  }
  return 0;
}

static int dohofpga1_wakeup(struct v4l2_subdev *sd)
{
  struct i2c_client *client = v4l2_get_subdevdata(sd);
  int err = -EINVAL, i;

  v4l_info(client, "%s: wakeup mode\n", __func__);

  for (i = 0; i < DOHOFPGA1_WAKEUP_REGS; i++) 
  {
    if (dohofpga1_wakeup_reg[i][0] == REG_DELAY) 
    {
      mdelay(dohofpga1_wakeup_reg[i][1]);
      err = 0;
    } else 
    {
      err = dohofpga1_write(sd, dohofpga1_wakeup_reg[i][0], \
                           dohofpga1_wakeup_reg[i][1]);
    }
    if (err < 0)
      v4l_info(client, "%s: register set failed\n", __func__);
  }

  if (err < 0) 
  {
    v4l_err(client, "%s: wake up failed\n", __func__);
    return -EIO;
  }
  return 0;
}


static int dohofpga1_s_stream(struct v4l2_subdev *sd, int enable)
{
  struct dohofpga1_state *state = to_state(sd);
  struct i2c_client *client = v4l2_get_subdevdata(sd);

  int err=0;
  printk("%s %d \n",__func__,enable);

  switch (enable) 
  {
    case STREAM_MODE_CAM_ON:
      printk("%s STREAM_MODE_CAM_ON\n",__func__);
      err = __dohofpga1_init_2byte(sd, \
           (unsigned short *) Viewfinder_ON, DOHOFPGA1_Viewfinder_ON);
      if (err < 0) 
      {
        v4l_info(client, "%s: camera i2c setting failed\n", __func__);
        //return -EIO;  /* FIXME */
      }
      break;
    case STREAM_MODE_CAM_OFF:
      printk("%s STREAM_MODE_CAM_OFF\n",__func__);
      err = __dohofpga1_init_2byte(sd, \
            (unsigned short *) Viewfinder_OFF, DOHOFPGA1_Viewfinder_OFF);
      if (err < 0) 
      {
        v4l_info(client, "%s: camera i2c setting failed\n", __func__);
        //return -EIO;  /* FIXME */
      }
      break;
  }

  return err;
  //return enable ? dohofpga1_wakeup(sd) : dohofpga1_sleep(sd);
}

/*
 * dohofpga1_probe
 * Fetching platform data is being done with s_config subdev call.
 * In probe routine, we just register subdev device
 */

static int dohofpga1_probe(struct i2c_client *client,
       const struct i2c_device_id *id)
{
  struct dohofpga1_state *state;
  struct v4l2_subdev *sd;
  struct dohofpga1_mbus_platform_data *pdata = client->dev.platform_data;

  printk("dohofpga1_probe() - started \n");

  if (!pdata)
  {
    dev_err( &client->dev, "null platform data");
    return -EIO;
  }

  state = kzalloc(sizeof(struct dohofpga1_state), GFP_KERNEL);
  if (state == NULL) return -ENOMEM;

  sd = &state->sd;
  strcpy(sd->name, DOHOFPGA1_DRIVER_NAME);
  state->pdata = client->dev.platform_data;

  /* set default data from sensor specific value */
  state->fmt.width = pdata->fmt.width;
  state->fmt.height = pdata->fmt.height;
  state->fmt.code = pdata->fmt.code;

  /* Registering subdev */
  v4l2_i2c_subdev_init(sd, client, &dohofpga1_ops);

  /* needed for acquiring subdevice by this module name */
  snprintf(sd->name, sizeof(sd->name), DOHOFPGA1_DRIVER_NAME);

  dev_info(&client->dev, "id: %d, fmt.code: %d, res: res: %d x %d",
      pdata->id, pdata->fmt.code,
      pdata->fmt.width, pdata->fmt.height);
  dev_info(&client->dev, "dohofpga1 has been probed\n");

  printk("dohofpga1_probe() - finished \n") ;
  return 0;
}

static int dohofpga1_remove(struct i2c_client *client)
{
  struct v4l2_subdev *sd = i2c_get_clientdata(client);

  printk("dohofpga1_remove()\n") ;
  v4l2_device_unregister_subdev(sd);
  kfree(to_state(sd));
  return 0;
}

static int __init dohofpga1_mod_init(void)
{
  int err=i2c_add_driver(&dohofpga1_i2c_driver);
  printk("dohofpga1_mod_init() - i2c_add_driver() returned %d\n", err) ;
  return err;
}

static void __exit dohofpga1_mod_exit(void)
{
  i2c_del_driver(&dohofpga1_i2c_driver);
  printk("dohofpga1_mod_exit() - i2c_del_driver() called\n") ;
  return ;
}

module_init(dohofpga1_mod_init);

module_exit(dohofpga1_mod_exit);

MODULE_DESCRIPTION("DOHOFPGA1 - ivc/doho fpga camera interface driver");
MODULE_AUTHOR("dominik.honegger <dominik.honegger@inf.ethz.ch>");
MODULE_LICENSE("GPL");

