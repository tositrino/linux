/* linux/drivers/media/video/dohofpga2.c
 *
 * Copyright (c) 2013 institute for visual computing, ETH Zuerich
 * http://ivc.ethz.ch/
 *
 * based on s5k4egcx.c
 * Copyright (C) 2012, Hardkernel Co.,Ltd.
 * Author: ruppi.kim@hardkernel.com
 *
 * Driver for dominik honeggers fpga camara interface 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

#define __DOHOFPGA2_C__

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/version.h>
#include <linux/vmalloc.h>
#include <linux/completion.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/dohofpga2_platform.h>
#include <linux/videodev2_samsung.h>
#include <linux/workqueue.h>

#include <linux/delay.h>
#ifdef CONFIG_EXYNOS4_CPUFREQ
#include <linux/cpufreq.h>
#include <mach/cpufreq.h>
#endif

#include "dohofpga2_regs.h"
#include "dohofpga2.h"

#ifdef CONFIG_LOAD_FILE
/* The Path of Setfile */
#include "dohofpga2_loadfile.inc"
#endif

/**
 * dohofpga2_i2c_read_twobyte: Read 2 bytes from sensor
 */

static int dohofpga2_i2c_read_twobyte(struct i2c_client *client,
          u16 subaddr, u16 *data)
{
  int err = 0 ;
  u8 buf[2];
  struct i2c_msg msg[2];

  cpu_to_be16s(&subaddr);

  msg[0].addr = client->addr;
  msg[0].flags = 0;
  msg[0].len = 2;
  msg[0].buf = (u8 *)&subaddr;

  msg[1].addr = client->addr;
  msg[1].flags = I2C_M_RD;
  msg[1].len = 2;
  msg[1].buf = buf;

  if ( (dohofpga2_i2c & DOHOFPGA2_I2C_READ) == DOHOFPGA2_I2C_READ )
  {
    err = i2c_transfer(client->adapter, msg, 2);
    CHECK_ERR_COND_MSG(err != 2, -EIO, "fail to read register\n");
    *data = ((buf[0] << 8) | buf[1]);
  }
  else
  {
    cam_dbg("%s: i2c read disabled by config\n", __func__);
    *data = 0x0000;
  }
  return err;
}

/**
 * dohofpga2_i2c_write_twobyte: Write (I2C) multiple bytes to the camera sensor
 * @client: pointer to i2c_client
 * @cmd: command register
 * @w_data: data to be written
 * @w_len: length of data to be written
 *
 * Returns 0 on success, <0 on error
 */

static int dohofpga2_i2c_write_twobyte(struct i2c_client *client,
           u16 addr, u16 w_data)
{
  int err = 0;
  int retry_count = 5;
  int ret = 0;
  u8 buf[4] = {0,};
  struct i2c_msg msg = {
    .addr  = client->addr,
    .flags  = 0,
    .len  = 4,
    .buf  = buf,
  };

  buf[0] = addr >> 8;
  buf[1] = addr;
  buf[2] = w_data >> 8;
  buf[3] = w_data & 0xff;

  if ( (dohofpga2_i2c & DOHOFPGA2_I2C_WRITE) == DOHOFPGA2_I2C_WRITE )
  {
    /* cam_dbg("I2C writing: 0x%02X%02X%02X%02X\n",
       buf[0], buf[1], buf[2], buf[3]); */
    do 
    {
      ret = i2c_transfer(client->adapter, &msg, 1);
      if (likely(ret == 1)) break;
      msleep(POLL_TIME_MS);
      cam_err("%s: ERROR(%d), write (%04X, %04X), retry %d.\n",
              __func__, ret, addr, w_data, retry_count);
    } while (retry_count-- > 0);
    CHECK_ERR_COND_MSG(ret != 1, -EIO, "I2C does not working.\n\n");
  }
  else
  {
    cam_dbg("%s: i2c write disabled by config\n", __func__);
  }
  return err;
}

static int dohofpga2_i2c_transfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
  if ( (dohofpga2_i2c & DOHOFPGA2_I2C_TRANSFER) == DOHOFPGA2_I2C_TRANSFER )
    return i2c_transfer(adap,msgs,num);
  cam_dbg("%s: i2c transfer disabled by config\n", __func__);
  return 0;
}

/* Write register
 * If success, return value: 0
 * If fail, return value: -EIO
 */

static int dohofpga2_write_regs(struct v4l2_subdev *sd, const u32 regs[],
           int size)
{
  struct i2c_client *client = v4l2_get_subdevdata(sd);
  u16 delay = 0;
  int i, err = 0;

  if ( (dohofpga2_i2c & DOHOFPGA2_I2C_ENABLE) == DOHOFPGA2_I2C_ENABLE)
  {
    cam_dbg("%s: i2c enabled, writing regs\n", __func__);
    for (i = 0; i < size; i++) 
    {
      if ((regs[i] & DOHOFPGA2_DELAY) == DOHOFPGA2_DELAY) 
      {
        delay = regs[i] & 0xFFFF;
        debug_msleep(sd, delay);
        continue;
      }

      err = dohofpga2_i2c_write_twobyte(client,
            (regs[i] >> 16), regs[i]);
      CHECK_ERR_MSG(err, "write registers\n")
    }
  }
  else
  {
    cam_dbg("%s: i2c disabled by config\n", __func__);
  }
  return err;
}

#define BURST_MODE_BUFFER_MAX_SIZE 2700

u8 dohofpga2_burstmode_buf[BURST_MODE_BUFFER_MAX_SIZE];

/* PX: */

static int dohofpga2_burst_write_regs(struct v4l2_subdev *sd,
      const u32 list[], u32 size, char *name)
{
  struct i2c_client *client = v4l2_get_subdevdata(sd);
  int err = 0;
  int i = 0, idx = 0;
  u16 subaddr = 0, next_subaddr = 0, value = 0;
  struct i2c_msg msg = {
    .addr  = client->addr,
    .flags  = 0,
    .len  = 0,
    .buf  = dohofpga2_burstmode_buf,
  };

  if ( (dohofpga2_i2c & DOHOFPGA2_I2C_ENABLE) == DOHOFPGA2_I2C_ENABLE)
  {
    cam_dbg("%s: i2c enabled, writing regs\n", __func__);
    cam_trace("E\n");

    for (i = 0; i < size; i++) 
    {
      CHECK_ERR_COND_MSG((idx > (BURST_MODE_BUFFER_MAX_SIZE - 10)),
                          err, "BURST MOD buffer overflow!\n")

      subaddr = (list[i] & 0xFFFF0000) >> 16;
      if (subaddr == 0x0F12)
        next_subaddr = (list[i+1] & 0xFFFF0000) >> 16;

      value = list[i] & 0x0000FFFF;

      switch (subaddr) 
      {
        case 0x0F12:
          /* make and fill buffer for burst mode write. */
          if (idx == 0) 
          {
            dohofpga2_burstmode_buf[idx++] = 0x0F;
            dohofpga2_burstmode_buf[idx++] = 0x12;
          }
          dohofpga2_burstmode_buf[idx++] = value >> 8;
          dohofpga2_burstmode_buf[idx++] = value & 0xFF;

          /* write in burstmode*/
          if (next_subaddr != 0x0F12) 
          {
            msg.len = idx;
            err = dohofpga2_i2c_transfer(client->adapter,
                  &msg, 1) == 1 ? 0 : -EIO;
            CHECK_ERR_MSG(err, "dohofpga2_i2c_transfer\n");
            /* cam_dbg("dohofpga2_sensor_burst_write,
                  idx = %d\n", idx); */
            idx = 0;
          }
          break;

        case 0xFFFF:
          debug_msleep(sd, value);
          break;

        default:
          idx = 0;
          err = dohofpga2_i2c_write_twobyte(client,
                subaddr, value);
          CHECK_ERR_MSG(err, "i2c_write_twobytes\n");
      }
    }
  }
  else
  {
    cam_dbg("%s: i2c disabled by config\n", __func__);
  }
  return err;
}

static int dohofpga2_set_from_table(struct v4l2_subdev *sd,
        const char *setting_name,
        const struct dohofpga2_regset_table *table,
        u32 table_size, s32 index)
{
  int err = 0;

  cam_dbg("%s: set %s index %d\n",
          __func__, setting_name, index); 

  CHECK_ERR_COND_MSG(((index < 0) || (index >= table_size)),
    -EINVAL, "index(%d) out of range[0:%d] for table for %s\n",
    index, table_size, setting_name);

  table += index;
  CHECK_ERR_COND_MSG(!table->reg, -EFAULT, \
    "table=%s, index=%d, reg = NULL\n", setting_name, index);

#ifdef CONFIG_LOAD_FILE
  cam_dbg("%s: \"%s\", reg_name=%s\n", __func__,
      setting_name, table->name);
  return dohofpga2_write_regs_from_sd(sd, table->name);

#else /* CONFIG_LOAD_FILE */

#ifdef DEBUG_WRITE_REGS
  cam_dbg("%s: \"%s\", reg_name=%s\n", __func__,
      setting_name, table->name);
#endif /* DEBUG_WRITE_REGS */

  err = dohofpga2_write_regs(sd, table->reg, table->array_size);
  CHECK_ERR_MSG(err, "write regs(%s), err=%d\n", setting_name, err);
  return 0;
#endif /* CONFIG_LOAD_FILE */

}

static int dohofpga2_set_parameter(struct v4l2_subdev *sd,
        int *current_value_ptr,
        int new_value,
        const char *setting_name,
        const struct dohofpga2_regset_table *table,
        int table_size)
{
  int err;

  if (*current_value_ptr == new_value)
    return 0;

  err = dohofpga2_set_from_table(sd, setting_name, table,
        table_size, new_value);

  if (!err)
    *current_value_ptr = new_value;
  return err;
}

static inline int dohofpga2_save_ctrl(struct v4l2_subdev *sd,
          struct v4l2_control *ctrl)
{
  int ctrl_cnt = ARRAY_SIZE(dohofpga2_ctrls);
  int i;

  /* cam_trace("E, Ctrl-ID = 0x%X", ctrl->id);*/

  for (i = 0; i < ctrl_cnt; i++) {
    if (ctrl->id == dohofpga2_ctrls[i].id) {
      dohofpga2_ctrls[i].value = ctrl->value;
      break;
    }
  }

  if (unlikely(i >= ctrl_cnt))
    cam_trace("WARNING, not saved ctrl-ID=0x%X\n", ctrl->id);

  return 0;
}

/**
 * dohofpga2_is_hwflash_on - check whether flash device is on
 *
 * Refer to state->flash_on to check whether flash is in use in driver.
 */
static inline int dohofpga2_is_hwflash_on(struct v4l2_subdev *sd)
{
  return 0;
}

/**
 * dohofpga2_flash_en - contro Flash LED
 * @mode: DOHOFPGA2_FLASH_MODE_NORMAL or DOHOFPGA2_FLASH_MODE_MOVIE
 * @onoff: DOHOFPGA2_FLASH_ON or DOHOFPGA2_FLASH_OFF
 */

static int dohofpga2_flash_en(struct v4l2_subdev *sd, s32 mode, s32 onoff)
{
  struct dohofpga2_state *state = to_state(sd);

  if (unlikely(state->ignore_flash)) {
    cam_warn("WARNING, we ignore flash command.\n");
    return 0;
  }

#ifdef DOHOFPGA2_SUPPORT_FLASH
  return state->pdata->flash_en(mode, onoff);
#endif
  return 0;
}

/**
 * dohofpga2_flash_torch - turn flash on/off as torch for preflash, recording
 * @onoff: DOHOFPGA2_FLASH_ON or DOHOFPGA2_FLASH_OFF
 *
 * This func set state->flash_on properly.
 */
static inline int dohofpga2_flash_torch(struct v4l2_subdev *sd, s32 onoff)
{
  struct dohofpga2_state *state = to_state(sd);
  int err = 0;

  err = dohofpga2_flash_en(sd, DOHOFPGA2_FLASH_MODE_MOVIE, onoff);
  state->flash_on = (onoff == DOHOFPGA2_FLASH_ON) ? 1 : 0;

  return err;
}

static int dohofpga2_set_jpeg_quality(struct v4l2_subdev *sd)
{
  struct i2c_client *client = v4l2_get_subdevdata(sd);
  struct dohofpga2_state *state =
    container_of(sd, struct dohofpga2_state, sd);

  dev_dbg(&client->dev,
    "%s: jpeg.quality %d\n", __func__, state->jpeg.quality);
  if (state->jpeg.quality < 0)
    state->jpeg.quality = 0;
  if (state->jpeg.quality > 100)
    state->jpeg.quality = 100;

  switch (state->jpeg.quality) {
  case 90 ... 100:
    dev_dbg(&client->dev,
      "%s: setting to high jpeg quality\n", __func__);
    return dohofpga2_set_from_table(sd, "jpeg quality high",
        &state->regs->jpeg_quality_high, 1, 0);
  case 80 ... 89:
    dev_dbg(&client->dev,
      "%s: setting to normal jpeg quality\n", __func__);
    return dohofpga2_set_from_table(sd, "jpeg quality normal",
        &state->regs->jpeg_quality_normal, 1, 0);
  default:
    dev_dbg(&client->dev,
      "%s: setting to low jpeg quality\n", __func__);
    return dohofpga2_set_from_table(sd, "jpeg quality low",
        &state->regs->jpeg_quality_low, 1, 0);
  }
}

static int dohofpga2_set_capture_size(struct v4l2_subdev *sd)
{
  struct dohofpga2_state *state = to_state(sd);

  /* Don't forget the below codes.
   * We set here state->preview to NULL after reconfiguring
   * capure config if capture ratio does't match with preview ratio.
   */
  state->preview = NULL;
  return 0;
}

static int dohofpga2_set_sensor_mode(struct v4l2_subdev *sd, s32 val)
{
  struct dohofpga2_state *state = to_state(sd);

  state->hd_videomode = 0;

  switch (val) {
  case SENSOR_MOVIE:
    /* We does not support movie mode when in VT. */
    if (state->vt_mode) {
      state->sensor_mode = SENSOR_CAMERA;
      cam_err("%s: ERROR, Not support movie\n", __func__);
      break;
    }
    /* We do not break. */

  case SENSOR_CAMERA:
    state->sensor_mode = val;
    break;

  case 2:  /* 720p HD video mode */
    state->sensor_mode = SENSOR_MOVIE;
    state->hd_videomode = 1;
    break;

  default:
    cam_err("%s: ERROR, Not support.(%d)\n", __func__, val);
    state->sensor_mode = SENSOR_CAMERA;
    WARN_ON(1);
    break;
  }

  return 0;
}

static int dohofpga2_set_frame_rate(struct v4l2_subdev *sd, s32 fps)
{
  struct dohofpga2_state *state = to_state(sd);
  int err = -EIO;
  int i = 0, fps_index = -1;

  cam_info("set frame rate %d\n", fps);

  for (i = 0; i < ARRAY_SIZE(dohofpga2_framerates); i++) {
    if (fps == dohofpga2_framerates[i].fps) {
      fps_index = dohofpga2_framerates[i].index;
      state->fps = fps;
      state->req_fps = -1;
      break;
    }
  }

  if (unlikely(fps_index < 0)) {
    cam_err("%s: WARNING, Not supported FPS(%d)\n", __func__, fps);
    return 0;
  }

  if (!state->hd_videomode) {
    err = dohofpga2_set_from_table(sd, "fps", state->regs->fps,
        ARRAY_SIZE(state->regs->fps), fps_index);
    CHECK_ERR_MSG(err, "fail to set framerate\n")
  }

  return 0;
}

static int dohofpga2_return_focus(struct v4l2_subdev *sd)
{
  int err;
  struct i2c_client *client = v4l2_get_subdevdata(sd);
  struct dohofpga2_state *state =
      container_of(sd, struct dohofpga2_state, sd);

  err = dohofpga2_set_from_table(sd,
    "af normal mode 1",
    &state->regs->af_normal_mode_1, 1, 0);
  if (err < 0)
    goto fail;
  msleep(FIRST_SETTING_FOCUS_MODE_DELAY_MS);
  err = dohofpga2_set_from_table(sd,
    "af normal mode 2",
    &state->regs->af_normal_mode_2, 1, 0);
  if (err < 0)
    goto fail;
  msleep(SECOND_SETTING_FOCUS_MODE_DELAY_MS);
  err = dohofpga2_set_from_table(sd,
    "af normal mode 3",
    &state->regs->af_normal_mode_3, 1, 0);
  if (err < 0)
    goto fail;

  return 0;
fail:
  dev_err(&client->dev,
    "%s: i2c_write failed\n", __func__);
  return -EIO;
}

static int dohofpga2_init_param(struct v4l2_subdev *sd)
{
  struct v4l2_control ctrl;
  int i,e,err=0;

  cam_dbg("%s start:\n", __func__ );
  for (i = 0; i < ARRAY_SIZE(dohofpga2_ctrls); i++) 
  {
    cam_dbg(" * control %d, id=0x%08X, value=%d, default=%d\n", i,
            dohofpga2_ctrls[i].id ,
            dohofpga2_ctrls[i].value,
            dohofpga2_ctrls[i].default_value
            );

    if (dohofpga2_ctrls[i].value != dohofpga2_ctrls[i].default_value) 
    {
      ctrl.id = dohofpga2_ctrls[i].id;
      ctrl.value = dohofpga2_ctrls[i].value;
      cam_dbg("   reset control %d to default value\n", i);
      if ( (e = dohofpga2_s_ctrl(sd, &ctrl)) != 0 ) err=e;
    }
  }
  cam_dbg("%s finished, status=%d:\n", __func__, err );
  return err;
}


static int dohofpga2_set_focus_mode(struct v4l2_subdev *sd, int value)
{
  struct i2c_client *client = v4l2_get_subdevdata(sd);
  struct dohofpga2_state *state =
    container_of(sd, struct dohofpga2_state, sd);
  struct sec_cam_parm *parms =
    (struct sec_cam_parm *)&state->strm.parm.raw_data;
  int err=0;


  cam_dbg("%s value(%d)\n", __func__, value);

  if (parms->focus_mode == value) goto done ;

  switch (value) 
  {
    case FOCUS_MODE_MACRO:
      dev_dbg(&client->dev,
        "%s: FOCUS_MODE_MACRO\n", __func__);
      err = dohofpga2_set_from_table(sd, "af macro mode 1",
            &state->regs->af_macro_mode_1, 1, 0);
      if (err < 0) goto fail;
      msleep(FIRST_SETTING_FOCUS_MODE_DELAY_MS);
      err = dohofpga2_set_from_table(sd, "af macro mode 2",
            &state->regs->af_macro_mode_2, 1, 0);
      if (err < 0) goto fail;
      msleep(SECOND_SETTING_FOCUS_MODE_DELAY_MS);
      err = dohofpga2_set_from_table(sd, "af macro mode 3",
            &state->regs->af_macro_mode_3, 1, 0);
      if (err < 0) goto fail;
      parms->focus_mode = FOCUS_MODE_MACRO;
      break;

    case FOCUS_MODE_INFINITY:
    case FOCUS_MODE_AUTO:
      err = dohofpga2_set_from_table(sd,
            "af normal mode 1",
            &state->regs->af_normal_mode_1, 1, 0);
      if (err < 0) goto fail;
      msleep(FIRST_SETTING_FOCUS_MODE_DELAY_MS);
      err = dohofpga2_set_from_table(sd,
            "af normal mode 2",
            &state->regs->af_normal_mode_2, 1, 0);
      if (err < 0) goto fail;
      msleep(SECOND_SETTING_FOCUS_MODE_DELAY_MS);
      err = dohofpga2_set_from_table(sd,
            "af normal mode 3",
            &state->regs->af_normal_mode_3, 1, 0);
      if (err < 0) goto fail;
      parms->focus_mode = value;
      break;
    default:
      err=-EINVAL;
  }

  done:
  cam_dbg("%s finished, status=%d:\n", __func__, err );
  return err;

  fail:
  cam_dbg("%s finished, status=%d:\n", __func__, err );
  dev_err(&client->dev, "%s: i2c_write failed\n", __func__);
  return -EIO;
}

static int dohofpga2_start_auto_focus(struct v4l2_subdev *sd)
{
  struct dohofpga2_state *state =
    container_of(sd, struct dohofpga2_state, sd);
  struct sec_cam_parm *parms =
    (struct sec_cam_parm *)&state->strm.parm.raw_data;

  cam_dbg("%s: start SINGLE AF operation, flash mode %d\n",
    __func__, parms->flash_mode);

  /* in case user calls auto_focus repeatedly without a cancel
   * or a capture, we need to cancel here to allow ae_awb
   * to work again, or else we could be locked forever while
   * that app is running, which is not the expected behavior.
   */
  dohofpga2_set_from_table(sd, "ae awb lock off",
        &state->regs->ae_awb_lock_off, 1, 0);

  if(state->recording)
    dohofpga2_set_from_table(sd, "video af start",
          &state->regs->video_af_start, 1, 0);
  else 
    dohofpga2_set_from_table(sd, "single af start",
          &state->regs->single_af_start, 1, 0);

  state->af_status = AF_INITIAL;

  return 0;
}

/* called by HAL after auto focus was finished.
 * it might off the assist flash
 */
static int dohofpga2_finish_auto_focus(struct v4l2_subdev *sd)
{
  struct i2c_client *client = v4l2_get_subdevdata(sd);
  struct dohofpga2_state *state =
    container_of(sd, struct dohofpga2_state, sd);

  /* restore write mode */
  dohofpga2_i2c_write_twobyte(client, 0x0028, 0x7000);

  if (state->flash_on) {
//    struct dohofpga2_platform_data *pd = client->dev.platform_data;
    dohofpga2_set_from_table(sd, "AF assist flash end",
        &state->regs->af_assist_flash_end, 1, 0);
    state->flash_on = false;
//    pd->af_assist_onoff(0);
  }

  cam_dbg("%s: single AF finished\n", __func__);
  state->af_status = AF_NONE;
  return 0;
}

static int dohofpga2_stop_auto_focus(struct v4l2_subdev *sd)
{
  struct i2c_client *client = v4l2_get_subdevdata(sd);
  struct dohofpga2_state *state =
    container_of(sd, struct dohofpga2_state, sd);
  struct sec_cam_parm *parms =
    (struct sec_cam_parm *)&state->strm.parm.raw_data;
  int focus_mode = parms->focus_mode;

  dev_dbg(&client->dev, "%s: single AF Off command Setting\n", __func__);

  /* always cancel ae_awb, in case AF already finished before
   * we got called.
   */
  /* restore write mode */
  dohofpga2_i2c_write_twobyte(client, 0x0028, 0x7000);

  dohofpga2_set_from_table(sd, "ae awb lock off",
        &state->regs->ae_awb_lock_off, 1, 0);
  if (state->flash_on)
    dohofpga2_finish_auto_focus(sd);

  if (state->af_status != AF_START) {
    /* we weren't in the middle auto focus operation, we're done */
    cam_dbg("%s: auto focus not in progress, done\n", __func__);

    if (focus_mode == FOCUS_MODE_MACRO) {
      /* for change focus mode forcely */
      parms->focus_mode = -1;
      dohofpga2_set_focus_mode(sd, FOCUS_MODE_MACRO);
    } else if (focus_mode == FOCUS_MODE_AUTO) {
      /* for change focus mode forcely */
      parms->focus_mode = -1;
      dohofpga2_set_focus_mode(sd, FOCUS_MODE_AUTO);
    }

    return 0;
  }

  /* auto focus was in progress.  the other thread
   * is either in the middle of dohofpga2_get_auto_focus_result_first(),
   * dohofpga2_get_auto_focus_result_second()
   * or will call it shortly.  set a flag to have
   * it abort it's polling.  that thread will
   * also do cleanup like restore focus position.
   *
   * it might be enough to just send sensor commands
   * to abort auto focus and the other thread would get
   * that state from it's polling calls, but I'm not sure.
   */
  state->af_status = AF_CANCEL;
  cam_dbg("%s: sending Single_AF_Off commands to sensor\n", __func__);

  dohofpga2_set_from_table(sd, "single af off 1",
        &state->regs->single_af_off_1, 1, 0);

  msleep(state->one_frame_delay_ms);

  dohofpga2_set_from_table(sd, "single af off 2",
        &state->regs->single_af_off_2, 1, 0);

  return 0;
}

/* called by HAL after auto focus was started to get the first search result*/
static int dohofpga2_get_auto_focus_result_first(struct v4l2_subdev *sd,
          struct v4l2_control *ctrl)
{
  struct i2c_client *client = v4l2_get_subdevdata(sd);
  struct dohofpga2_state *state =
    container_of(sd, struct dohofpga2_state, sd);
  u16 read_value;

  if (state->af_status == AF_INITIAL) {
    cam_dbg("%s: Check AF Result\n", __func__);
    if (state->af_status == AF_NONE) {
      dev_dbg(&client->dev,
        "%s: auto focus never started, returning 0x2\n",
        __func__);
      ctrl->value = AUTO_FOCUS_CANCELLED;
      return 0;
    }

    /* must delay 2 frame times before checking result of 1st phase */
    mutex_unlock(&state->ctrl_lock);
    msleep(state->one_frame_delay_ms*2);
    mutex_lock(&state->ctrl_lock);

    /* lock AE and AWB after first AF search */
    dohofpga2_set_from_table(sd, "ae awb lock on",
          &state->regs->ae_awb_lock_on, 1, 0);

    cam_dbg( "%s: 1st AF search\n", __func__);
    /* enter read mode */
    dohofpga2_i2c_write_twobyte(client, 0x002C, 0x7000);
    state->af_status = AF_START;
  } else if (state->af_status == AF_CANCEL) {
    cam_dbg("%s: AF is cancelled while doing\n", __func__);
    ctrl->value = AUTO_FOCUS_CANCELLED;
    dohofpga2_finish_auto_focus(sd);
    return 0;
  }
  dohofpga2_set_from_table(sd, "get 1st af search status",
        &state->regs->get_1st_af_search_status,
        1, 0);
  dohofpga2_i2c_read_twobyte(client, 0x0F12, &read_value);
  cam_dbg("%s: 1st AF --- read_value == 0x%x\n",  __func__, read_value);
  ctrl->value = read_value;
  return 0;
}

/* called by HAL after first search was succeed to get the second search result*/
static int dohofpga2_get_auto_focus_result_second(struct v4l2_subdev *sd,
          struct v4l2_control *ctrl)
{
  struct i2c_client *client = v4l2_get_subdevdata(sd);
  struct dohofpga2_state *state =
    container_of(sd, struct dohofpga2_state, sd);
  u16 read_value;

  if (state->af_status == AF_CANCEL) {
    cam_dbg("%s: AF is cancelled while doing\n", __func__);
    ctrl->value = AUTO_FOCUS_CANCELLED;
    dohofpga2_finish_auto_focus(sd);
    return 0;
  }
  dohofpga2_set_from_table(sd, "get 2nd af search status",
        &state->regs->get_2nd_af_search_status,
        1, 0);
  dohofpga2_i2c_read_twobyte(client, 0x0F12, &read_value);
  cam_dbg("%s: 2nd AF --- read_value == 0x%x\n", __func__, read_value);
  ctrl->value = read_value;
  return 0;
}

static int dohofpga2_init_regs(struct v4l2_subdev *sd)
{
  struct i2c_client *client = v4l2_get_subdevdata(sd);
  struct dohofpga2_state *state = to_state(sd);
  u16 read_value = 0;
  int err = 0 ;

  /* we'd prefer to do this in probe, but the framework hasn't
   * turned on the camera yet so our i2c operations would fail
   * if we tried to do it in probe, so we have to do it here
   * and keep track if we succeeded or not.
   */

  cam_dbg("%s: start\n", __func__);

  if ( (dohofpga2_i2c & DOHOFPGA2_I2C_ENABLE) == DOHOFPGA2_I2C_ENABLE)
  {
    /* enter read mode */
    err = dohofpga2_i2c_write_twobyte(client, 0x002C, 0x7000);
    if (unlikely(err < 0)) return -ENODEV;

    dohofpga2_i2c_write_twobyte(client, 0x002E, 0x01A4);
    dohofpga2_i2c_read_twobyte(client, 0x0F12, &read_value);
    if (likely(read_value == DOHOFPGA2_CHIP_ID))
      cam_dbg("Sensor ChipID: 0x%04X\n", DOHOFPGA2_CHIP_ID);
    else
      cam_dbg("Sensor ChipID: 0x%04X, unknown ChipID\n", read_value);

    dohofpga2_i2c_write_twobyte(client, 0x002C, 0x7000);
    dohofpga2_i2c_write_twobyte(client, 0x002E, 0x01A6);
    dohofpga2_i2c_read_twobyte(client, 0x0F12, &read_value);
    if (likely(read_value == DOHOFPGA2_CHIP_REV))
      cam_dbg("Sensor revision: 0x%04X\n", DOHOFPGA2_CHIP_REV);
    else
      cam_dbg("Sensor revision: 0x%04X, unknown revision\n", read_value);

    /* restore write mode */
    err = dohofpga2_i2c_write_twobyte(client, 0x0028, 0x7000);
    CHECK_ERR_COND(err < 0, -ENODEV);
  }
  else
  {
    cam_dbg("%s: i2c disabled by config\n", __func__);
    cam_dbg("Sensor ChipID   : 0x%04X\n", DOHOFPGA2_CHIP_ID);
    cam_dbg("Sensor Revision : 0x%04X\n", DOHOFPGA2_CHIP_REV);
  }

  state->regs = &reg_datas;

  return err;
}

static const struct dohofpga2_framesize *dohofpga2_get_framesize
  (const struct dohofpga2_framesize *frmsizes,
  u32 frmsize_count, u32 index)
{
  int i = 0;

  for (i = 0; i < frmsize_count; i++) {
    if (frmsizes[i].index == index)
      return &frmsizes[i];
  }

  return NULL;
}

/* This function is called from the g_ctrl api
 *
 * This function should be called only after the s_fmt call,
 * which sets the required width/height value.
 *
 * It checks a list of available frame sizes and sets the
 * most appropriate frame size.
 *
 * The list is stored in an increasing order (as far as possible).
 * Hence the first entry (searching from the beginning) where both the
 * width and height is more than the required value is returned.
 * In case of no perfect match, we set the last entry (which is supposed
 * to be the largest resolution supported.)
 */
static void dohofpga2_set_framesize(struct v4l2_subdev *sd,
        const struct dohofpga2_framesize *frmsizes,
        u32 num_frmsize, bool preview)
{
  struct dohofpga2_state *state = to_state(sd);
  const struct dohofpga2_framesize **found_frmsize = NULL;
  u32 width = state->req_fmt.width;
  u32 height = state->req_fmt.height;
  int i = 0;

  cam_dbg("%s: Requested Res %dx%d\n", __func__,
      width, height);

  found_frmsize = (const struct dohofpga2_framesize **)
      (preview ? &state->preview : &state->capture);

  for (i = 0; i < num_frmsize; i++) {
    if ((frmsizes[i].width == width) &&
      (frmsizes[i].height == height)) {
      *found_frmsize = &frmsizes[i];
      break;
    }
  }

  if (*found_frmsize == NULL) {
    cam_err("%s: ERROR, invalid frame size %dx%d\n", __func__,
            width, height);
    *found_frmsize = preview ?
      dohofpga2_get_framesize(frmsizes, num_frmsize,
          DOHOFPGA2_PREVIEW_VGA) :
      dohofpga2_get_framesize(frmsizes, num_frmsize,
          DOHOFPGA2_CAPTURE_3MP);
    BUG_ON(!(*found_frmsize));
  }

  if (preview) {
    cam_info("Preview Res Set: %dx%d, index %d\n",
      (*found_frmsize)->width, (*found_frmsize)->height,
      (*found_frmsize)->index);
  }
  else {
    cam_info("Capture Res Set: %dx%d, index %d\n",
      (*found_frmsize)->width, (*found_frmsize)->height,
      (*found_frmsize)->index);
  }
}

static int dohofpga2_check_esd(struct v4l2_subdev *sd)
{
  struct dohofpga2_state *state = to_state(sd);
  struct i2c_client *client = v4l2_get_subdevdata(sd);
  int err = -EINVAL;
  u16 read_value = 0;

  err = dohofpga2_set_from_table(sd, "get_esd_status",
    &state->regs->get_esd_status, 1, 0);
  CHECK_ERR(err);
  err = dohofpga2_i2c_read_twobyte(client, 0x0F12, &read_value);
  CHECK_ERR(err);

  dohofpga2_i2c_write_twobyte(client, 0x0028, 0x7000);

  if (read_value != 0xAAAA)
    goto esd_out;

  cam_info("Check ESD: not detected\n\n");
  return 0;

esd_out:
  cam_err("Check ESD: ERROR, ESD Shock detected! (val=0x%X)\n\n",
    read_value);
  return -ERESTART;
}


/* PX: Set ISO */
static int __used dohofpga2_set_iso(struct v4l2_subdev *sd, s32 val)
{
  struct dohofpga2_state *state = to_state(sd);
  int err = -EINVAL;

retry:
  switch (val) {
  case ISO_AUTO:
  case ISO_50:
  case ISO_100:
  case ISO_200:
  case ISO_400:
    err = dohofpga2_set_from_table(sd, "iso",
      state->regs->iso, ARRAY_SIZE(state->regs->iso),
      val);
    break;

  default:
    cam_err("%s: ERROR, invalid arguement(%d)\n", __func__, val);
    val = ISO_AUTO;
    goto retry;
    break;
  }

  cam_trace("X\n");
  return 0;
}

static int dohofpga2_set_preview_size(struct v4l2_subdev *sd)
{
  struct dohofpga2_state *state = to_state(sd);
  int err = -EINVAL;

  cam_trace("E, wide_cmd=%d\n", state->wide_cmd);

  switch (state->wide_cmd) {
  case WIDE_REQ_CHANGE:
    cam_info("%s: Wide Capture setting\n", __func__);
    err = dohofpga2_set_from_table(sd, "change_wide_cap",
      &state->regs->change_wide_cap, 1, 0);
    break;

  case WIDE_REQ_RESTORE:
    cam_info("%s:Restore capture setting\n", __func__);
    err = dohofpga2_set_from_table(sd, "restore_capture",
        &state->regs->restore_cap, 1, 0);
    /* We do not break */

  default:
    err = dohofpga2_set_from_table(sd, "preview_size",
        state->regs->preview_size,
        ARRAY_SIZE(state->regs->preview_size),
        state->preview->index);
    break;
  }
  CHECK_ERR(err);

  return 0;
}

static int dohofpga2_set_preview_start(struct v4l2_subdev *sd)
{
  struct dohofpga2_state *state = to_state(sd);
  int err = -EINVAL;
  /* bool set_size = true; */
  cam_dbg("Camera Preview start, runmode = %d\n", state->runmode);

  if (state->need_update_frmsize) {
    err = dohofpga2_set_preview_size(sd);
    state->need_update_frmsize = 0;
    CHECK_ERR_MSG(err, "failed to set preview size(%d)\n", err);

    dohofpga2_set_focus_mode(sd, FOCUS_MODE_AUTO);
    dohofpga2_start_auto_focus(sd);
  }

  if (state->runmode == DOHOFPGA2_RUNMODE_CAPTURING) {

    err = dohofpga2_set_from_table(sd, "preview_return",
        &state->regs->preview_return, 1, 0);
        
    CHECK_ERR_MSG(err, "fail to set Preview_Return (%d)\n", err)
  } 
  state->runmode = DOHOFPGA2_RUNMODE_RUNNING;

  return 0;
}

/* PX: Start capture */
static int dohofpga2_set_capture_start(struct v4l2_subdev *sd)
{
  struct dohofpga2_state *state = to_state(sd);
  struct i2c_client *client = v4l2_get_subdevdata(sd);
  u16 read_value;
  int poll_time_ms;
  int err = -ENODEV;

  /* Set capture size */
  err = dohofpga2_set_capture_size(sd);
  CHECK_ERR_MSG(err, "fail to set capture size (%d)\n", err);

  /* Send capture start command. */
  cam_dbg("Send Capture_Start cmd idx: %d\n",state->capture->index);
  err = dohofpga2_set_from_table(sd, "capture_start",
      state->regs->capture_start,
      ARRAY_SIZE(state->regs->capture_start),
      state->capture->index);

  /* a shot takes takes at least 50ms so sleep that amount first
   * and then start polling for completion.
   */
  msleep(5);
  /* Enter read mode */
  dohofpga2_i2c_write_twobyte(client, 0x002C, 0x7000);
  poll_time_ms = 5;
  do {
    dohofpga2_set_from_table(sd, "get capture status",
          &state->regs->get_capture_status, 1, 0);
    dohofpga2_i2c_read_twobyte(client, 0x0F12, &read_value);
    cam_dbg("%s: dohofpga2_Capture_Start check = %#x\n",  __func__, read_value);
    if (read_value != 0x00)
      break;
    msleep(POLL_TIME_MS);
    poll_time_ms += POLL_TIME_MS;
  } while (poll_time_ms < CAPTURE_POLL_TIME_MS);

  cam_dbg("%s: capture done check finished after %d ms\n",__func__, poll_time_ms);

  /* restore write mode */
  dohofpga2_i2c_write_twobyte(client, 0x0028, 0x7000);

  state->runmode = DOHOFPGA2_RUNMODE_CAPTURING;

  CHECK_ERR_MSG(err, "fail to capture_start (%d)\n", err);

  return 0;
}

static int dohofpga2_s_mbus_fmt(struct v4l2_subdev *sd,
        struct v4l2_mbus_framefmt *fmt)
{
  struct dohofpga2_state *state = to_state(sd);

  if(!state->initialized) return -1;

  cam_dbg("%s: pixelformat = 0x%x, colorspace = 0x%x, width = %d, height = %d\n",
    __func__, fmt->code, fmt->colorspace, fmt->width, fmt->height);

  v4l2_fill_pix_format(&state->req_fmt, fmt);

  if(fmt->colorspace == V4L2_COLORSPACE_JPEG) 
    state->format_mode = V4L2_PIX_FMT_MODE_CAPTURE;
  else  state->format_mode = V4L2_PIX_FMT_MODE_PREVIEW;
  state->wide_cmd = WIDE_REQ_NONE;

  if (state->format_mode != V4L2_PIX_FMT_MODE_CAPTURE) {
    if(state->pdata->set_power) {
      state->pdata->set_power(0);
      mdelay(100);
      state->pdata->set_power(1);
      dohofpga2_init(sd, 0);
      state->wide_cmd = WIDE_REQ_NONE;
      state->need_update_frmsize = 1;
    }
    dohofpga2_set_framesize(sd, dohofpga2_preview_frmsizes,
        ARRAY_SIZE(dohofpga2_preview_frmsizes),
        true);

    if(state->preview->index ==5)   state->recording = 1;
    else               state->recording = 0;
  } else {
    /*
     * In case of image capture mode,
     * if the given image resolution is not supported,
     * use the next higher image resolution. */
    dohofpga2_set_framesize(sd, dohofpga2_capture_frmsizes,
        ARRAY_SIZE(dohofpga2_capture_frmsizes),
        false);

    /* for maket app.
     * Samsung camera app does not use unmatched ratio.*/
    if (unlikely(FRM_RATIO(state->preview)
        != FRM_RATIO(state->capture))) {
      state->wide_cmd = WIDE_REQ_NONE;
    }
  }
  return 0;
}

static int dohofpga2_enum_mbus_fmt(struct v4l2_subdev *sd, unsigned int index,
          enum v4l2_mbus_pixelcode *code)
{
  cam_dbg("%s: index = %d\n", __func__, index);

  if (index >= ARRAY_SIZE(capture_fmts))
    return -EINVAL;

  *code = capture_fmts[index].code;

  return 0;
}

static int dohofpga2_try_mbus_fmt(struct v4l2_subdev *sd,
        struct v4l2_mbus_framefmt *fmt)
{
  int num_entries;
  int i;

  num_entries = ARRAY_SIZE(capture_fmts);

  cam_dbg("%s: code = 0x%x , colorspace = 0x%x, num_entries = %d\n",
    __func__, fmt->code, fmt->colorspace, num_entries);

  for (i = 0; i < num_entries; i++) {
    if (capture_fmts[i].code == fmt->code &&
        capture_fmts[i].colorspace == fmt->colorspace) {
      cam_dbg("%s: match found, returning 0\n", __func__);
      return 0;
    }
  }

  cam_err("%s: no match found, returning -EINVAL\n", __func__);
  return -EINVAL;
}


static int dohofpga2_enum_framesizes(struct v4l2_subdev *sd,
          struct v4l2_frmsizeenum *fsize)
{
  struct dohofpga2_state *state = to_state(sd);

  /*
  * The camera interface should read this value, this is the resolution
  * at which the sensor would provide framedata to the camera i/f
  * In case of image capture,
  * this returns the default camera resolution (VGA)
  */

  if (state->format_mode != V4L2_PIX_FMT_MODE_CAPTURE) {
    if (unlikely(state->preview == NULL)) {
      cam_err("%s: ERROR\n", __func__);
      return -EFAULT;
    }

    fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
    fsize->discrete.width = state->preview->width;
    fsize->discrete.height = state->preview->height;
  } else {
    if (unlikely(state->capture == NULL)) {
      cam_err("%s: ERROR\n", __func__);
      return -EFAULT;
    }

    fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
    fsize->discrete.width = state->capture->width;
    fsize->discrete.height = state->capture->height;
  }

  return 0;
}

static int dohofpga2_g_parm(struct v4l2_subdev *sd,
      struct v4l2_streamparm *param)
{
  return 0;
}

static int dohofpga2_s_parm(struct v4l2_subdev *sd,
      struct v4l2_streamparm *param)
{
  int err = 0;
  struct dohofpga2_state *state = to_state(sd);

  state->req_fps = param->parm.capture.timeperframe.denominator /
      param->parm.capture.timeperframe.numerator;

  cam_dbg("s_parm state->fps=%d, state->req_fps=%d\n",
    state->fps, state->req_fps);

  if ((state->req_fps < 0) || (state->req_fps > 30)) {
    cam_err("%s: ERROR, invalid frame rate %d. we'll set to 30\n",
        __func__, state->req_fps);
    state->req_fps = 30;
  }

  if (state->initialized && (state->scene_mode == SCENE_MODE_NONE)) {
    err = dohofpga2_set_frame_rate(sd, state->req_fps);
    CHECK_ERR(err);
  }
  return 0;
}

static int dohofpga2_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
  struct dohofpga2_state *state = to_state(sd);
  int err = 0;

  cam_dbg("%s: ID =%d, val = %d\n",
    __func__, ctrl->id - V4L2_CID_PRIVATE_BASE, ctrl->value);

  mutex_lock(&state->ctrl_lock);

  switch (ctrl->id) 
  {
  case V4L2_CID_WHITE_BALANCE_PRESET:
    break;
  case V4L2_CID_COLORFX:
    break;
  case V4L2_CID_CONTRAST:
    break;
  case V4L2_CID_SATURATION:
    break;
  case V4L2_CID_SHARPNESS:
    break;
  case V4L2_CID_CAM_JPEG_MEMSIZE:
    ctrl->value = SENSOR_JPEG_SNAPSHOT_MEMSIZE +
      SENSOR_JPEG_THUMB_MAXSIZE + SENSOR_JPEG_POST_MAXSIZE;
    break;
  case V4L2_CID_CAM_JPEG_MAIN_OFFSET:
    ctrl->value = state->jpeg.main_offset;
    break;      
  case V4L2_CID_CAM_JPEG_POSTVIEW_OFFSET:
      ctrl->value = state->jpeg.postview_offset;
    break;
  case V4L2_CID_CAM_JPEG_MAIN_SIZE:
      ctrl->value = state->jpeg.main_size;
    break;
  case V4L2_CID_CAM_JPEG_QUALITY:
      ctrl->value = state->jpeg.quality;
    break;
  case V4L2_CID_CAM_JPEG_THUMB_SIZE:
    ctrl->value = state->jpeg.thumb_size;
    break;
  case V4L2_CID_CAM_JPEG_THUMB_OFFSET:
    ctrl->value = state->jpeg.thumb_offset;
    break;
  case V4L2_CID_CAMERA_EXIF_ISO:
    if (state->sensor_mode == SENSOR_CAMERA)
      ctrl->value = state->exif.iso;
    else
      ctrl->value = 100;
      break;
  case V4L2_CID_CAMERA_EXIF_TV:
    ctrl->value = state->exif.tv;
    break;

  case V4L2_CID_CAMERA_EXIF_BV:
    ctrl->value = state->exif.bv;
    break;

  case V4L2_CID_CAMERA_EXIF_EBV:
    ctrl->value = state->exif.ebv;
    break;

  case V4L2_CID_CAMERA_AUTO_FOCUS_RESULT_FIRST:
    err = dohofpga2_get_auto_focus_result_first(sd, ctrl);
    break;
  case V4L2_CID_CAMERA_AUTO_FOCUS_RESULT_SECOND:
    err = dohofpga2_get_auto_focus_result_second(sd, ctrl);
    break;

  default:
    err= -ENOIOCTLCMD;
    break;
  }
  mutex_unlock(&state->ctrl_lock);
  return err;
}

static int dohofpga2_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
  struct i2c_client *client = v4l2_get_subdevdata(sd);
  struct dohofpga2_state *state = to_state(sd);
  struct sec_cam_parm *parms =
    (struct sec_cam_parm *)&state->strm.parm.raw_data;
  int err = -ENOIOCTLCMD;
  int value = ctrl->value;

  cam_dbg("%s: ID =%d, val = %d\n",
    __func__, ctrl->id - V4L2_CID_PRIVATE_BASE, ctrl->value);

  if (ctrl->id != V4L2_CID_CAMERA_SET_AUTO_FOCUS)
    mutex_lock(&state->ctrl_lock);

  switch (ctrl->id) {
  case V4L2_CID_CAMERA_SENSOR_MODE:
    err = dohofpga2_set_sensor_mode(sd, ctrl->value);
    break;

  case V4L2_CID_CAMERA_OBJECT_POSITION_X:
    err = 0;
  case V4L2_CID_CAMERA_OBJECT_POSITION_Y:
    err = 0;
    break;

  case V4L2_CID_CAMERA_FOCUS_MODE:
    err = dohofpga2_set_focus_mode(sd, value);
    break;
  case V4L2_CID_CAMERA_SET_AUTO_FOCUS:
    if (value == AUTO_FOCUS_ON)
      err = dohofpga2_start_auto_focus(sd);
    else if (value == AUTO_FOCUS_OFF)
      err = dohofpga2_stop_auto_focus(sd);
    else {
      err = -EINVAL;
      dev_err(&client->dev,
        "%s: bad focus value requestion %d\n",
        __func__, value);
    }
    break;
    
  case V4L2_CID_CAMERA_RETURN_FOCUS:
    if (parms->focus_mode != FOCUS_MODE_MACRO)
      err = dohofpga2_return_focus(sd);
    break;
  case V4L2_CID_CAMERA_FINISH_AUTO_FOCUS:
    err = dohofpga2_finish_auto_focus(sd);
    break;
        
  case V4L2_CID_CAMERA_FLASH_MODE:
    err = 0;
    break;

  case V4L2_CID_CAMERA_BRIGHTNESS:
    if (state->runmode == DOHOFPGA2_RUNMODE_RUNNING) {
      err = dohofpga2_set_parameter(sd, &parms->brightness,
            value, "brightness",
            state->regs->ev,
            ARRAY_SIZE(state->regs->ev));
    }
    break;

  case V4L2_CID_CAMERA_WHITE_BALANCE:
    err = dohofpga2_set_from_table(sd, "white balance",
      state->regs->white_balance,
      ARRAY_SIZE(state->regs->white_balance), ctrl->value);
    state->wb_mode = ctrl->value;
    break;

  case V4L2_CID_CAMERA_EFFECT:
    err = dohofpga2_set_from_table(sd, "effects",
      state->regs->effect,
      ARRAY_SIZE(state->regs->effect), ctrl->value);
    break;

  case V4L2_CID_CAMERA_METERING:
    if (state->runmode == DOHOFPGA2_RUNMODE_RUNNING) {
      err = dohofpga2_set_parameter(sd, &parms->metering,
          value, "metering",
          state->regs->metering,
          ARRAY_SIZE(state->regs->metering));
    }
    break;

  case V4L2_CID_CAMERA_CONTRAST:
    err = dohofpga2_set_parameter(sd, &parms->contrast,
          value, "contrast",
          state->regs->contrast,
          ARRAY_SIZE(state->regs->contrast));
    break;

  case V4L2_CID_CAMERA_SATURATION:
    err = dohofpga2_set_parameter(sd, &parms->saturation,
          value, "saturation",
          state->regs->saturation,
          ARRAY_SIZE(state->regs->saturation));
    break;

  case V4L2_CID_CAMERA_SHARPNESS:
    err = dohofpga2_set_parameter(sd, &parms->sharpness,
          value, "sharpness",
          state->regs->sharpness,
          ARRAY_SIZE(state->regs->sharpness));
    break;

  case V4L2_CID_CAMERA_SCENE_MODE:
    err = dohofpga2_set_parameter(sd, &parms->scene_mode,
          SCENE_MODE_NONE, "scene_mode",
          state->regs->scene_mode,
          ARRAY_SIZE(state->regs->scene_mode));
    if (err < 0) {
      dev_err(&client->dev,
        "%s: failed to set scene-mode default value\n",
        __func__);
      break;
    }
    if (value != SCENE_MODE_NONE) {
      err = dohofpga2_set_parameter(sd, &parms->scene_mode,
          value, "scene_mode",
          state->regs->scene_mode,
          ARRAY_SIZE(state->regs->scene_mode));
    }
    if (parms->scene_mode == SCENE_MODE_NIGHTSHOT) {
      state->one_frame_delay_ms =
        NIGHT_MODE_MAX_ONE_FRAME_DELAY_MS;
    } else {
      state->one_frame_delay_ms =
        NORMAL_MODE_MAX_ONE_FRAME_DELAY_MS;
    }
    break;

  case V4L2_CID_CAMERA_CHECK_ESD:
    err = dohofpga2_check_esd(sd);
    break;
  case V4L2_CID_CAMERA_CAPTURE:
    err = 0;
    break;
  case V4L2_CID_CAM_JPEG_QUALITY:
    state->jpeg.quality = ctrl->value;
    err = dohofpga2_set_jpeg_quality(sd);
    break;
  case V4L2_CID_CAMERA_ISO:
    if (state->runmode == DOHOFPGA2_RUNMODE_RUNNING) {
      err = dohofpga2_set_parameter(sd, &parms->iso,
            value, "iso",
            state->regs->iso,
            ARRAY_SIZE(state->regs->iso));
    }
    break;
  case V4L2_CID_CAMERA_FRAME_RATE:
  case V4L2_CID_CAMERA_ZOOM:
    break;
  default:
    cam_err("%s: WARNING, unknown Ctrl-ID : %d\n",
      __func__, ctrl->id - V4L2_CID_PRIVATE_BASE);
    err = 0; /* we return no error. */
    break;
  }

  if (ctrl->id != V4L2_CID_CAMERA_SET_AUTO_FOCUS)
    mutex_unlock(&state->ctrl_lock);

  return err;
}

static int dohofpga2_s_ext_ctrl(struct v4l2_subdev *sd,
            struct v4l2_ext_control *ctrl)
{
  return 0;
}

static int dohofpga2_s_ext_ctrls(struct v4l2_subdev *sd,
        struct v4l2_ext_controls *ctrls)
{
  struct v4l2_ext_control *ctrl = ctrls->controls;
  int ret=0;
  int i;

  for (i = 0; i < ctrls->count; i++, ctrl++) 
  {
    ret = dohofpga2_s_ext_ctrl(sd, ctrl);

    if (ret) 
    {
      ctrls->error_idx = i;
      break;
    }
  }

  return ret;
}

static int dohofpga2_s_stream(struct v4l2_subdev *sd, int enable)
{
  struct dohofpga2_state *state = to_state(sd);
  int err = 0;

  BUG_ON(!state->initialized);

  if(enable) 
  {
    switch (state->sensor_mode) 
    {
      case SENSOR_CAMERA:
        if (state->format_mode == V4L2_PIX_FMT_MODE_CAPTURE)
          err = dohofpga2_set_capture_start(sd);
        else
          err = dohofpga2_set_preview_start(sd);
        break;
      default:
        break;
    }
  }
  return err;
}

static int dohofpga2_reset(struct v4l2_subdev *sd, u32 val)
{
  struct dohofpga2_state *state = to_state(sd);

  cam_trace("EX\n");

  dohofpga2_return_focus(sd);
  state->initialized = 0;

  return 0;
}

static int dohofpga2_init(struct v4l2_subdev *sd, u32 val)
{
  struct dohofpga2_state *state = to_state(sd);
  int err = -EINVAL;

  cam_info("%s started for %s version %s\n", __func__, DOHOFPGA2_DRIVER_NAME,
            DOHOFPGA2_DRIVER_VER );

#if defined(CONFIG_EXYNOS4_CPUFREQ)
  cam_dbg("%s: locking cpufreq at %d\n", __func__, state->cpufreq_lock_level);
  if (state->cpufreq_lock_level == CPUFREQ_ENTRY_INVALID) 
  {
    err = exynos_cpufreq_get_level(1400 * 1000,
      &state->cpufreq_lock_level);
    CHECK_ERR_MSG(err, "failed get DVFS level\n");
  }
  err = exynos_cpufreq_lock(DVFS_LOCK_ID_CAM, state->cpufreq_lock_level);
  CHECK_ERR_MSG(err, "failed lock DVFS\n");
#endif

  state->initialized = 0;
  err = dohofpga2_init_regs(sd);
  CHECK_ERR_MSG(err, "failed to indentify sensor chip\n");

  if (dohofpga2_set_from_table(sd, "init arm",
      &state->regs->init_arm, 1, 0) < 0)
    return -EIO;

  err = DOHOFPGA2_BURST_WRITE_REGS(sd, dohofpga2_init_reg);
  CHECK_ERR_MSG(err, "failed to initialize camera device\n");

  state->runmode = DOHOFPGA2_RUNMODE_INIT;

  /* Default state values */
  state->flash_mode = FLASH_MODE_OFF;
  state->scene_mode = SCENE_MODE_NONE;
  state->flash_on = 0;
  state->light_level = 0xFFFFFFFF;
  state->initialized = 1;

  if (state->sensor_mode == SENSOR_MOVIE)
  {
    err = dohofpga2_init_param(sd);
    CHECK_ERR_MSG(err, "failed to initialize camera controls\n");
  }

  if (state->req_fps >= 0) 
  {
    err = dohofpga2_set_frame_rate(sd, state->req_fps);
    CHECK_ERR_MSG(err, "failed to initialize frmae rate to %d\n", state->req_fps );
  }

#if defined(CONFIG_EXYNOS4_CPUFREQ)
  cam_dbg("%s: unlocking cpufreq\n", __func__);
  exynos_cpufreq_lock_free(DVFS_LOCK_ID_CAM);
#endif

  cam_dbg("%s: stop, return = %d\n", __func__, err);
  return err;
}

/*
 * s_config subdev ops
 * With camera device, we need to re-initialize
 * every single opening time therefor,
 * it is not necessary to be initialized on probe time.
 * except for version checking
 * NOTE: version checking is optional
 */

static int dohofpga2_s_config(struct v4l2_subdev *sd,
      int irq, void *platform_data)
{
  struct dohofpga2_state *state = to_state(sd);
  int i;
  int err = 0;

  if (!platform_data) 
  {
    cam_err("%s: ERROR, no platform data\n", __func__);
    return -ENODEV;
  }

  state->pdata = platform_data;
  state->dbg_level = &state->pdata->dbg_level;

#if defined(CONFIG_EXYNOS4_CPUFREQ)
  state->cpufreq_lock_level = CPUFREQ_ENTRY_INVALID;
#endif

  /*
   * Assign default format and resolution
   * Use configured default information in platform data
   * or without them, use default information in driver
   */
  state->req_fmt.width = state->pdata->default_width;
  state->req_fmt.height = state->pdata->default_height;

  if (!state->pdata->pixelformat)
    state->req_fmt.pixelformat = DEFAULT_PIX_FMT;
  else
    state->req_fmt.pixelformat = state->pdata->pixelformat;

  if (!state->pdata->freq)
    state->freq = DEFAULT_MCLK;  /* 24MHz default */
  else
    state->freq = state->pdata->freq;

  state->preview = state->capture = NULL;
  state->sensor_mode = SENSOR_CAMERA;
  state->hd_videomode = 0;
  state->format_mode = V4L2_PIX_FMT_MODE_PREVIEW;
  state->fps = 0;
  state->req_fps = -1;

  for (i = 0; i < ARRAY_SIZE(dohofpga2_ctrls); i++)
    dohofpga2_ctrls[i].value = dohofpga2_ctrls[i].default_value;

#ifdef DOHOFPGA2_SUPPORT_FLASH
  if (dohofpga2_is_hwflash_on(sd))
    state->ignore_flash = 1;
#endif

#if defined(FEATURE_YUV_CAPTURE)
  state->jpeg.enable = 0;
  state->jpeg.quality = 100;
  state->jpeg.main_offset = 0; /* 0x500 */

  /* Maximum size 2592 * 1944 * 2 = 10077696 */
  state->jpeg.main_size = SENSOR_JPEG_SNAPSHOT_MEMSIZE;

  state->jpeg.thumb_size = SENSOR_JPEG_THUMB_MAXSIZE; /* 0x27C */
  state->jpeg.thumb_offset = SENSOR_JPEG_SNAPSHOT_MEMSIZE; /* 0x27C */

  state->jpeg.postview_offset = SENSOR_JPEG_SNAPSHOT_MEMSIZE + SENSOR_JPEG_THUMB_MAXSIZE;

#endif

#ifdef CONFIG_LOAD_FILE
  err = loadFile();
  if (unlikely(err < 0))
    cam_err("failed to load file ERR=%d\n", err);
#endif

  return err;
}

static const struct v4l2_subdev_core_ops dohofpga2_core_ops = 
{
  .init        = dohofpga2_init,  /* initializing API */
  .g_ctrl      = dohofpga2_g_ctrl,
  .s_ctrl      = dohofpga2_s_ctrl,
  .s_ext_ctrls = dohofpga2_s_ext_ctrls,
  .reset = dohofpga2_reset,
};

static const struct v4l2_subdev_video_ops dohofpga2_video_ops = 
{
  .s_mbus_fmt      = dohofpga2_s_mbus_fmt,
  .enum_framesizes = dohofpga2_enum_framesizes,
  .enum_mbus_fmt   = dohofpga2_enum_mbus_fmt,
  .try_mbus_fmt    = dohofpga2_try_mbus_fmt,
  .g_parm          = dohofpga2_g_parm,
  .s_parm          = dohofpga2_s_parm,
  .s_stream        = dohofpga2_s_stream,
};

static const struct v4l2_subdev_ops dohofpga2_ops = 
{
  .core = &dohofpga2_core_ops,
  .video = &dohofpga2_video_ops,
};


/*
 * dohofpga2_probe
 * Fetching platform data is being done with s_config subdev call.
 * In probe routine, we just register subdev device
 */

static int dohofpga2_probe(struct i2c_client *client,
      const struct i2c_device_id *id)
{
  struct v4l2_subdev *sd;
  struct dohofpga2_state *state;
  int err = 0;

  pr_info("[%s]: %s called for %s version %s\n", DOHOFPGA2_DRIVER_NAME, 
          __func__, DOHOFPGA2_DRIVER_NAME, DOHOFPGA2_DRIVER_VER );

  state = kzalloc(sizeof(struct dohofpga2_state), GFP_KERNEL);
  if (unlikely(!state)) 
  {
    dev_err(&client->dev, "probe, fail to get memory\n");
    return -ENOMEM;
  }

  mutex_init(&state->ctrl_lock);
  mutex_init(&state->af_lock);

  state->runmode = DOHOFPGA2_RUNMODE_NOTREADY;
  sd = &state->sd;
  strcpy(sd->name, DOHOFPGA2_DRIVER_NAME);

  /* Registering subdev */
  v4l2_i2c_subdev_init(sd, client, &dohofpga2_ops);

  err = dohofpga2_s_config(sd, 0, client->dev.platform_data);
  CHECK_ERR_MSG(err, "fail to s_config\n");

  cam_dbg("driver probed!! driver_string=\"%s\" dev_name=\"%s\"\n",
          dev_driver_string(&client->dev), dev_name(&client->dev));

  return err;
}

static int dohofpga2_remove(struct i2c_client *client)
{
  struct v4l2_subdev *sd = i2c_get_clientdata(client);
  struct dohofpga2_state *state = to_state(sd);

  pr_info("[%s]: %s called for %s version %s\n", DOHOFPGA2_DRIVER_NAME, 
          __func__, DOHOFPGA2_DRIVER_NAME, DOHOFPGA2_DRIVER_VER );

  /* Check whether flash is on when unlolading driver,
   * to preventing Market App from controlling improperly flash.
   * It isn't necessary in case that you power flash down
   * in power routine to turn camera off.*/

  if (unlikely(state->flash_on && !state->ignore_flash))
    dohofpga2_flash_torch(sd, DOHOFPGA2_FLASH_OFF);

  v4l2_device_unregister_subdev(sd);
  mutex_destroy(&state->ctrl_lock);
  mutex_destroy(&state->af_lock);
  kfree(state);

#ifdef CONFIG_LOAD_FILE
  large_file ? vfree(testBuf) : kfree(testBuf);
  large_file = 0;
  testBuf = NULL;
#endif

  cam_dbg("driver removed!! driver_string=\"%s\" dev_name=\"%s\"\n",
          dev_driver_string(&client->dev), dev_name(&client->dev));
  return 0;
}

static const struct i2c_device_id dohofpga2_id[] = 
{
  { DOHOFPGA2_DRIVER_NAME, 0 },
  {}
};

MODULE_DEVICE_TABLE(i2c, dohofpga2_id);

static struct i2c_driver v4l2_i2c_driver = {
  .driver.name  = DOHOFPGA2_DRIVER_NAME,
  .probe        = dohofpga2_probe,
  .remove       = dohofpga2_remove,
  .id_table     = dohofpga2_id,
};

static int __init v4l2_i2c_drv_init(void)
{
  pr_info("[%s]: %s called for %s version %s\n", DOHOFPGA2_DRIVER_NAME, 
          __func__, DOHOFPGA2_DRIVER_NAME, DOHOFPGA2_DRIVER_VER );
  return i2c_add_driver(&v4l2_i2c_driver);
}

static void __exit v4l2_i2c_drv_cleanup(void)
{
  pr_info("[%s]: %s called for %s version %s\n", DOHOFPGA2_DRIVER_NAME, 
          __func__, DOHOFPGA2_DRIVER_NAME, DOHOFPGA2_DRIVER_VER );
  i2c_del_driver(&v4l2_i2c_driver);
}

module_init( v4l2_i2c_drv_init );

module_exit( v4l2_i2c_drv_cleanup );

MODULE_DESCRIPTION("DOHOFPGA2 - ivc/doho fpga camera interface driver");
MODULE_AUTHOR("dominik.honegger <dominik.honegger@inf.ethz.ch>");
MODULE_VERSION(DOHOFPGA2_DRIVER_VER);
MODULE_LICENSE("GPL");
