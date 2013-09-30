/* linux/drivers/media/video/dohofpga2_loadfile.inc
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

#ifndef __DOHOFPGA2_INC__
#define __DOHOFPGA2_INC__

#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

struct test 
{
  u8 data;
  struct test *nextBuf;
};

static struct test *testBuf;
static s32 large_file;

#define TEST_INIT  \
{      \
  .data = 0;  \
  .nextBuf = NULL;  \
}

#if defined(CONFIG_VIDEO_DOHOFPGA2_ODROIDQ)
#define TUNING_FILE_PATH "/mnt/sdcard/dohofpga2_regs_odroid.h"
#else
#define TUNING_FILE_PATH NULL
#endif
#endif /* CONFIG_LOAD_FILE*/


static int loadFile(void)
{
  struct file *fp = NULL;
  struct test *nextBuf = NULL;

  u8 *nBuf = NULL;
  size_t file_size = 0, max_size = 0, testBuf_size = 0;
  ssize_t nread = 0;
  s32 check = 0, starCheck = 0;
  s32 tmp_large_file = 0;
  s32 i = 0;
  int ret = 0;
  loff_t pos;

  mm_segment_t fs = get_fs();
  set_fs(get_ds());

  cam_info("%s: E\n", __func__);

  BUG_ON(testBuf);

  fp = filp_open(TUNING_FILE_PATH, O_RDONLY, 0);
  if (IS_ERR(fp)) {
    cam_err("file open error\n");
    return PTR_ERR(fp);
  }

  file_size = (size_t) fp->f_path.dentry->d_inode->i_size;
  max_size = file_size;

  cam_dbg("file_size = %d\n", file_size);

  nBuf = kmalloc(file_size, GFP_ATOMIC);
  if (nBuf == NULL) {
    cam_dbg("Fail to 1st get memory\n");
    nBuf = vmalloc(file_size);
    if (nBuf == NULL) {
      cam_err("ERR: nBuf Out of Memory\n");
      ret = -ENOMEM;
      goto error_out;
    }
    tmp_large_file = 1;
  }

  testBuf_size = sizeof(struct test) * file_size;
  if (tmp_large_file) {
    testBuf = (struct test *)vmalloc(testBuf_size);
    large_file = 1;
  } else {
    testBuf = kmalloc(testBuf_size, GFP_ATOMIC);
    if (testBuf == NULL) {
      cam_dbg("Fail to get mem(%d bytes)\n", testBuf_size);
      testBuf = (struct test *)vmalloc(testBuf_size);
      large_file = 1;
    }
  }
  if (testBuf == NULL) {
    cam_err("ERR: Out of Memory\n");
    ret = -ENOMEM;
    goto error_out;
  }

  pos = 0;
  memset(nBuf, 0, file_size);
  memset(testBuf, 0, file_size * sizeof(struct test));

  nread = vfs_read(fp, (char __user *)nBuf, file_size, &pos);
  if (nread != file_size) {
    cam_err("failed to read file ret = %d\n", nread);
    ret = -1;
    goto error_out;
  }

  set_fs(fs);

  i = max_size;

  cam_dbg(KERN_DEBUG "i = %d\n", i);

  while (i) {
    testBuf[max_size - i].data = *nBuf;
    if (i != 1) {
      testBuf[max_size - i].nextBuf =
        &testBuf[max_size - i + 1];
    } else {
      testBuf[max_size - i].nextBuf = NULL;
      break;
    }
    i--;
    nBuf++;
  }

  i = max_size;
  nextBuf = &testBuf[0];

  while (i - 1) {
    if (!check && !starCheck) {
      if (testBuf[max_size - i].data == '/') {
        if (testBuf[max_size-i].nextBuf != NULL) {
          if (testBuf[max_size-i].nextBuf->data
                == '/') {
            check = 1;/* when find '//' */
            i--;
          } else if (
              testBuf[max_size-i].nextBuf->data
              == '*') {
            /* when find '/ *' */
            starCheck = 1;
            i--;
          }
        } else
          break;
      }
      if (!check && !starCheck) {
        /* ignore '\t' */
        if (testBuf[max_size - i].data != '\t') {
          nextBuf->nextBuf = &testBuf[max_size-i];
          nextBuf = &testBuf[max_size - i];
        }
      }
    } else if (check && !starCheck) {
      if (testBuf[max_size - i].data == '/') {
        if (testBuf[max_size-i].nextBuf != NULL) {
          if (testBuf[max_size-i].nextBuf->data
              == '*') {
            /* when find '/ *' */
            starCheck = 1;
            check = 0;
            i--;
          }
        } else
          break;
      }

       /* when find '\n' */
      if (testBuf[max_size - i].data == '\n' && check) {
        check = 0;
        nextBuf->nextBuf = &testBuf[max_size - i];
        nextBuf = &testBuf[max_size - i];
      }

    } else if (!check && starCheck) {
      if (testBuf[max_size - i].data == '*') {
        if (testBuf[max_size-i].nextBuf != NULL) {
          if (testBuf[max_size-i].nextBuf->data
              == '/') {
            /* when find '* /' */
            starCheck = 0;
            i--;
          }
        } else
          break;
      }
    }

    i--;

    if (i < 2) {
      nextBuf = NULL;
      break;
    }

    if (testBuf[max_size - i].nextBuf == NULL) {
      nextBuf = NULL;
      break;
    }
  }

error_out:
  tmp_large_file ? vfree(nBuf) : kfree(nBuf);

  if (fp)
    filp_close(fp, current->files);
  return ret;
}

static int dohofpga2_write_regs_from_sd(struct v4l2_subdev *sd,
            const u8 s_name[])
{
  struct i2c_client *client = v4l2_get_subdevdata(sd);
  struct test *tempData = NULL;

  int ret = -EAGAIN;
  u32 temp;
  u32 delay = 0;
  u8 data[11];
  s32 searched = 0;
  size_t size = strlen(s_name);
  s32 i;
#ifdef DEBUG_WRITE_REGS
  u8 regs_name[128] = {0,};

  BUG_ON(size > sizeof(regs_name));
#endif

  cam_dbg("%s: E size = %d, string = %s\n", __func__, size, s_name);
  tempData = &testBuf[0];
  while (!searched) {
    searched = 1;
    for (i = 0; i < size; i++) {
      if (tempData->data != s_name[i]) {
        searched = 0;
        break;
      }
#ifdef DEBUG_WRITE_REGS
      regs_name[i] = tempData->data;
#endif
      tempData = tempData->nextBuf;
    }
#ifdef DEBUG_WRITE_REGS
    if (i > 9) {
      regs_name[i] = '\0';
      cam_dbg("Searching: regs_name = %s\n", regs_name);
    }
#endif
    tempData = tempData->nextBuf;
  }
  /* structure is get..*/
#ifdef DEBUG_WRITE_REGS
  regs_name[i] = '\0';
  cam_dbg("Searched regs_name = %s\n\n", regs_name);
#endif

  while (1) {
    if (tempData->data == '{')
      break;
    else
      tempData = tempData->nextBuf;
  }

  while (1) {
    searched = 0;
    while (1) {
      if (tempData->data == 'x') {
        /* get 10 strings.*/
        data[0] = '0';
        for (i = 1; i < 11; i++) {
          data[i] = tempData->data;
          tempData = tempData->nextBuf;
        }
        /*cam_dbg("%s\n", data);*/
        temp = simple_strtoul(data, NULL, 16);
        break;
      } else if (tempData->data == '}') {
        searched = 1;
        break;
      } else
        tempData = tempData->nextBuf;

      if (tempData->nextBuf == NULL)
        return -1;
    }

    if (searched)
      break;

    if ((temp & DOHOFPGA2_DELAY) == DOHOFPGA2_DELAY) {
      delay = temp & 0x0FFFF;
      debug_msleep(sd, delay);
      continue;
    }

    /* cam_dbg("I2C writing: 0x%08X,\n",temp);*/
    ret = dohofpga2_i2c_write_twobyte(client,
      (temp >> 16), (u16)temp);

    /* In error circumstances */
    /* Give second shot */
    if (unlikely(ret)) {
      dev_info(&client->dev,
          "dohofpga2 i2c retry one more time\n");
      ret = dohofpga2_i2c_write_twobyte(client,
        (temp >> 16), (u16)temp);

      /* Give it one more shot */
      if (unlikely(ret)) {
        dev_info(&client->dev,
            "dohofpga2 i2c retry twice\n");
        ret = dohofpga2_i2c_write_twobyte(client,
          (temp >> 16), (u16)temp);
      }
    }
  }

  return ret;
}


#endif // ifndef __DOHOFPGA2_INC__

