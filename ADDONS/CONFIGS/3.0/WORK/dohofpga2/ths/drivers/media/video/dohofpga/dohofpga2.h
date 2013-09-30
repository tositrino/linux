/* linux/drivers/media/video/dohofpga2.h
 *
 * Copyright (c) 2013 institute for visual computing, ETH Zuerich
 * http://ivc.ethz.ch/
 *
 * based on s5k4egcx.h
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

#ifndef __DOHOFPGA2_H__
#define __DOHOFPGA2_H__

#define DOHOFPGA2_DRIVER_NAME  "DOHOFPGA2"
#define DOHOFPGA2_DRIVER_VER    "0.5.13"

#define DOHOFPGA2_DELAY        0xFFFF0000

/************************************
 * FEATURE DEFINITIONS
 ************************************/
#define FEATURE_YUV_CAPTURE
/* #define CONFIG_LOAD_FILE */ /* for tuning */

/** Debuging Feature **/
#define CONFIG_CAM_DEBUG
#define CONFIG_CAM_TRACE       /* Enable it with CONFIG_CAM_DEBUG */
#define CONFIG_CAM_AF_DEBUG    /* Enable it with CONFIG_CAM_DEBUG */
#define DEBUG_WRITE_REGS

/***********************************/
#define CONFIG_VIDEO_DOHOFPGA2_DEBUG
#ifdef CONFIG_VIDEO_DOHOFPGA2_DEBUG
enum 
{
  DOHOFPGA2_DEBUG_I2C         = 1U << 0,
  DOHOFPGA2_DEBUG_I2C_BURSTS  = 1U << 1,
};


#define dohofpga2_debug(mask, x...) \
  do { \
    if (dohofpga2_debug_mask & mask) \
      pr_info(x);  \
  } while (0)
#else
#define dohofpga2_debug(mask, x...)
#endif

#define TAG_NAME  "["DOHOFPGA2_DRIVER_NAME"]"" "
#define cam_err(fmt, ...)  printk(KERN_ERR TAG_NAME fmt, ##__VA_ARGS__)
#define cam_warn(fmt, ...) printk(KERN_WARNING TAG_NAME fmt, ##__VA_ARGS__)
#define cam_info(fmt, ...) printk(KERN_INFO TAG_NAME fmt, ##__VA_ARGS__)

#if defined(CONFIG_CAM_DEBUG)
#define cam_dbg(fmt, ...)  printk(KERN_DEBUG TAG_NAME fmt, ##__VA_ARGS__)
#else
#define cam_dbg(fmt, ...)  do { \
    if (*to_state(sd)->dbg_level & CAMDBG_LEVEL_DEBUG) \
      printk(KERN_DEBUG TAG_NAME fmt, ##__VA_ARGS__); \
  } while (0)
#endif

#if defined(CONFIG_CAM_DEBUG) && defined(CONFIG_CAM_TRACE)
#define cam_trace(fmt, ...)  cam_dbg("%s: " fmt, __func__, ##__VA_ARGS__);
#else
#define cam_trace(fmt, ...)  \
  do { \
    if (*to_state(sd)->dbg_level & CAMDBG_LEVEL_TRACE) \
      printk(KERN_DEBUG TAG_NAME "%s: " fmt, \
        __func__, ##__VA_ARGS__); \
  } while (0)
#endif

#if defined(CONFIG_CAM_DEBUG) && defined(CONFIG_CAM_AF_DEBUG)
#define af_dbg(fmt, ...)  cam_dbg(fmt, ##__VA_ARGS__);
#else
#define af_dbg(fmt, ...)
#endif

#define CHECK_ERR_COND(condition, ret)  \
  do { if (unlikely(condition)) return ret; } while (0)
#define CHECK_ERR_COND_MSG(condition, ret, fmt, ...) \
    if (unlikely(condition)) { \
      cam_err("%s: ERROR, " fmt, __func__, ##__VA_ARGS__); \
      return ret; \
    }

#define CHECK_ERR(x)  CHECK_ERR_COND(((x) < 0), (x))
#define CHECK_ERR_MSG(x, fmt, ...) \
  CHECK_ERR_COND_MSG(((x) < 0), (x), fmt, ##__VA_ARGS__)


#ifdef CONFIG_LOAD_FILE
#define DOHOFPGA2_BURST_WRITE_REGS(sd, A) ({ \
  int ret; \
    cam_info("BURST_WRITE_REGS: reg_name=%s from setfile\n", #A); \
    ret = dohofpga2_write_regs_from_sd(sd, #A); \
    ret; \
  })
#else
#define DOHOFPGA2_BURST_WRITE_REGS(sd, A) \
  dohofpga2_burst_write_regs(sd, A, (sizeof(A) / sizeof(A[0])), #A)
#endif

/* result values returned to HAL */
enum {
  AUTO_FOCUS_FAILED,
  AUTO_FOCUS_DONE,
  AUTO_FOCUS_CANCELLED,
};

enum af_operation_status {
  AF_NONE = 0,
  AF_START,
  AF_CANCEL,
  AF_INITIAL,
};

enum preflash_status {
  PREFLASH_NONE = 0,
  PREFLASH_OFF,
  PREFLASH_ON,
};

enum dohofpga2_oprmode {
  DOHOFPGA2_OPRMODE_VIDEO = 0,
  DOHOFPGA2_OPRMODE_IMAGE = 1,
};

enum stream_cmd {
  STREAM_STOP,
  STREAM_START,
};

enum wide_req_cmd {
  WIDE_REQ_NONE,
  WIDE_REQ_CHANGE,
  WIDE_REQ_RESTORE,
};

enum dohofpga2_preview_frame_size {
  DOHOFPGA2_PREVIEW_QCIF = 0,  /*1 176x144 */
  DOHOFPGA2_PREVIEW_CIF,    /*2 352x288 */ 
  DOHOFPGA2_PREVIEW_VGA,    /*3 640x480 */
  DOHOFPGA2_PREVIEW_D1,    /*4 720x480 */
  DOHOFPGA2_PREVIEW_960,    /*5 960x640 */
  DOHOFPGA2_PREVIEW_720P,    /*6 1280x720 */
  DOHOFPGA2_PREVIEW_MAX,
};

/* Capture Size List: Capture size is defined as below.
 *
 *  DOHOFPGA2_CAPTURE_VGA:    640x480
 *  DOHOFPGA2_CAPTURE_WVGA:    800x480
 *  DOHOFPGA2_CAPTURE_SVGA:    800x600
 *  DOHOFPGA2_CAPTURE_WSVGA:    1024x600
 *  DOHOFPGA2_CAPTURE_1MP:    1280x960
 *  DOHOFPGA2_CAPTURE_W1MP:    1600x960
 *  DOHOFPGA2_CAPTURE_2MP:    UXGA - 1600x1200
 *  DOHOFPGA2_CAPTURE_W2MP:    35mm Academy Offset Standard 1.66
 *          2048x1232, 2.4MP
 *  DOHOFPGA2_CAPTURE_3MP:    QXGA  - 2048x1536
 *  DOHOFPGA2_CAPTURE_W4MP:    WQXGA - 2560x1536
 *  DOHOFPGA2_CAPTURE_5MP:    2560x1920
 */

enum dohofpga2_capture_frame_size {
  DOHOFPGA2_CAPTURE_VGA = 0,  /* 640x480 */
  DOHOFPGA2_CAPTURE_1MP,    /* 1280x960 */
  DOHOFPGA2_CAPTURE_2MP,    /* UXGA  - 1600x1200 */
  DOHOFPGA2_CAPTURE_3MP,    /* QXGA  - 2048x1536 */
  DOHOFPGA2_CAPTURE_5MP,    /* 2560x1920 */
  DOHOFPGA2_CAPTURE_MAX,
};

#ifdef CONFIG_VIDEO_DOHOFPGA2_P2
#define PREVIEW_WIDE_SIZE  DOHOFPGA2_PREVIEW_1024x552
#else
#define PREVIEW_WIDE_SIZE  DOHOFPGA2_PREVIEW_1024x576
#endif
#define CAPTURE_WIDE_SIZE  DOHOFPGA2_CAPTURE_W2MP

enum dohofpga2_fps_index {
  I_FPS_0,
  I_FPS_7,
  I_FPS_10,
  I_FPS_12,
  I_FPS_15,
  I_FPS_25,
  I_FPS_30,
  I_FPS_MAX,
};

enum ae_awb_lock {
  AEAWB_UNLOCK = 0,
  AEAWB_LOCK,
  AEAWB_LOCK_MAX,
};

struct dohofpga2_control {
  u32 id;
  s32 value;
  s32 default_value;
};

#define DOHOFPGA2_INIT_CONTROL(ctrl_id, default_val) \
  {          \
    .id = ctrl_id,      \
    .value = default_val,    \
    .default_value = default_val,  \
  }

struct dohofpga2_framesize {
  s32 index;
  u32 width;
  u32 height;
};

#define FRM_RATIO(framesize) \
  (((framesize)->width) * 10 / ((framesize)->height))

struct dohofpga2_fps {
  u32 index;
  u32 fps;
};

struct dohofpga2_version {
  u32 major;
  u32 minor;
};

struct dohofpga2_date_info {
  u32 year;
  u32 month;
  u32 date;
};

enum dohofpga2_runmode {
  DOHOFPGA2_RUNMODE_NOTREADY,
  DOHOFPGA2_RUNMODE_INIT,
  /*DOHOFPGA2_RUNMODE_IDLE,*/
  DOHOFPGA2_RUNMODE_RUNNING, /* previewing */
  DOHOFPGA2_RUNMODE_RUNNING_STOP,
  DOHOFPGA2_RUNMODE_CAPTURING,
  DOHOFPGA2_RUNMODE_CAPTURE_STOP,
};

struct dohofpga2_firmware {
  u32 addr;
  u32 size;
};

struct dohofpga2_jpeg_param {
  u32 enable;
  u32 quality;
  u32 main_size;    /* Main JPEG file size */
  u32 thumb_size;    /* Thumbnail file size */
  u32 main_offset;
  u32 thumb_offset;
  u32 postview_offset;
};

struct dohofpga2_position {
  s32 x;
  s32 y;
};

struct dohofpga2_rect {
  s32 x;
  s32 y;
  u32 width;
  u32 height;
};

struct gps_info_common {
  u32 direction;
  u32 dgree;
  u32 minute;
  u32 second;
};

struct dohofpga2_gps_info {
  u8 gps_buf[8];
  u8 altitude_buf[4];
  s32 gps_timeStamp;
};

struct dohofpga2_exif {
  u16 exp_time_den;
  u16 iso;
  u16 flash;

  int tv;      /* shutter speed */
  int bv;      /* brightness */
  int ebv;    /* exposure bias */
};

/* EXIF - flash filed */
#define EXIF_FLASH_FIRED    (0x01)
#define EXIF_FLASH_MODE_FIRING    (0x01)
#define EXIF_FLASH_MODE_SUPPRESSION  (0x01 << 1)
#define EXIF_FLASH_MODE_AUTO    (0x03 << 3)

struct dohofpga2_regset {
  u32 size;
  u8 *data;
};

struct dohofpga2_stream_time {
  struct timeval curr_time;
  struct timeval before_time;
};

#define GET_ELAPSED_TIME(cur, before) \
    (((cur).tv_sec - (before).tv_sec) * USEC_PER_SEC \
    + ((cur).tv_usec - (before).tv_usec))

#ifdef CONFIG_LOAD_FILE
#define DEBUG_WRITE_REGS
struct dohofpga2_regset_table {
  const char  *const name;
};

#define DOHOFPGA2_REGSET(x, y)    \
  [(x)] = {      \
    .name    = #y,  \
}

#define DOHOFPGA2_REGSET_TABLE(y)  \
  {        \
    .name    = #y,  \
}
#else
struct dohofpga2_regset_table {
  const u32  *const reg;
  const u32  array_size;
#ifdef DEBUG_WRITE_REGS
  const char  *const name;
#endif
};

#ifdef DEBUG_WRITE_REGS
#define DOHOFPGA2_REGSET(x, y)    \
  [(x)] = {          \
    .reg    = (y),      \
    .array_size  = ARRAY_SIZE((y)),  \
    .name    = #y,      \
}

#define DOHOFPGA2_REGSET_TABLE(y)    \
  {          \
    .reg    = (y),      \
    .array_size  = ARRAY_SIZE((y)),  \
    .name    = #y,      \
}
#else
#define DOHOFPGA2_REGSET(x, y)    \
  [(x)] = {          \
    .reg    = (y),      \
    .array_size  = ARRAY_SIZE((y)),  \
}

#define DOHOFPGA2_REGSET_TABLE(y)    \
  {          \
    .reg    = (y),      \
    .array_size  = ARRAY_SIZE((y)),  \
}
#endif /* DEBUG_WRITE_REGS */
#endif /* CONFIG_LOAD_FILE */

struct dohofpga2_regs {
  struct dohofpga2_regset_table ev[EV_MAX];
  struct dohofpga2_regset_table metering[METERING_MAX];
  struct dohofpga2_regset_table iso[ISO_MAX];
  struct dohofpga2_regset_table effect[IMAGE_EFFECT_MAX];
  struct dohofpga2_regset_table white_balance[WHITE_BALANCE_MAX];
  struct dohofpga2_regset_table preview_size[DOHOFPGA2_PREVIEW_MAX];
  struct dohofpga2_regset_table capture_start[DOHOFPGA2_CAPTURE_MAX];
  struct dohofpga2_regset_table scene_mode[SCENE_MODE_MAX];
  struct dohofpga2_regset_table saturation[SATURATION_MAX];
  struct dohofpga2_regset_table contrast[CONTRAST_MAX];
  struct dohofpga2_regset_table sharpness[SHARPNESS_MAX];
  struct dohofpga2_regset_table fps[I_FPS_MAX];
  struct dohofpga2_regset_table preview_return;
  struct dohofpga2_regset_table jpeg_quality_high;
  struct dohofpga2_regset_table jpeg_quality_normal;
  struct dohofpga2_regset_table jpeg_quality_low;
  struct dohofpga2_regset_table flash_start;
  struct dohofpga2_regset_table flash_end;

  struct dohofpga2_regset_table af_assist_flash_start;
  struct dohofpga2_regset_table af_assist_flash_end;
  struct dohofpga2_regset_table af_low_light_mode_on;
  struct dohofpga2_regset_table af_low_light_mode_off;

  struct dohofpga2_regset_table ae_awb_lock_on;
  struct dohofpga2_regset_table ae_awb_lock_off;
  struct dohofpga2_regset_table restore_cap;
  struct dohofpga2_regset_table change_wide_cap;
  struct dohofpga2_regset_table af_macro_mode_1;
  struct dohofpga2_regset_table af_macro_mode_2;
  struct dohofpga2_regset_table af_macro_mode_3;
  struct dohofpga2_regset_table af_normal_mode_1;
  struct dohofpga2_regset_table af_normal_mode_2;
  struct dohofpga2_regset_table af_normal_mode_3;

  struct dohofpga2_regset_table single_af_start;
  struct dohofpga2_regset_table video_af_start;
  struct dohofpga2_regset_table single_af_off_1;
  struct dohofpga2_regset_table single_af_off_2;
  struct dohofpga2_regset_table init_arm;
  struct dohofpga2_regset_table init_reg;

  struct dohofpga2_regset_table get_ae_stable_status;
  struct dohofpga2_regset_table get_light_level;
  struct dohofpga2_regset_table get_1st_af_search_status;
  struct dohofpga2_regset_table get_2nd_af_search_status;
  struct dohofpga2_regset_table get_capture_status;
  
  struct dohofpga2_regset_table get_esd_status;
  struct dohofpga2_regset_table get_iso;
  struct dohofpga2_regset_table get_ev;
  struct dohofpga2_regset_table get_ae_stable;
  struct dohofpga2_regset_table get_shutterspeed;
  struct dohofpga2_regset_table update_preview;
  struct dohofpga2_regset_table update_hd_preview;
  struct dohofpga2_regset_table stream_stop;
};

struct dohofpga2_state {
  struct dohofpga2_platform_data *pdata;
  struct v4l2_subdev sd;
  struct v4l2_pix_format req_fmt;
  struct v4l2_streamparm strm;
  
  struct dohofpga2_framesize *preview;
  struct dohofpga2_framesize *capture;
  struct dohofpga2_exif exif;
  struct dohofpga2_jpeg_param jpeg;
  struct dohofpga2_stream_time stream_time;
  const struct dohofpga2_regs *regs;
  struct mutex ctrl_lock;
  struct mutex af_lock;
  enum dohofpga2_runmode runmode;
  enum v4l2_sensor_mode sensor_mode;
  enum v4l2_pix_format_mode format_mode;
  enum v4l2_flash_mode flash_mode;
  enum v4l2_scene_mode scene_mode;
  enum v4l2_wb_mode wb_mode;
  enum af_operation_status af_status;
  /* To switch from nornal ratio to wide ratio.*/
  enum wide_req_cmd wide_cmd;

  bool sensor_af_in_low_light_mode;

  s32 vt_mode;
  s32 req_fps;
  s32 fps;
  s32 freq;    /* MCLK in Hz */
  u32 one_frame_delay_ms;
  u32 light_level;  /* light level */
  u8 *dbg_level;

#if defined(CONFIG_EXYNOS4_CPUFREQ)
  u32 cpufreq_lock_level;
#endif
  u32 recording:1;
  u32 hd_videomode:1;
  u32 flash_on:1;
  u32 ignore_flash:1;
  u32 need_update_frmsize:1;
  u32 need_wait_streamoff:1;
  u32 initialized:1;
};

static inline struct  dohofpga2_state *to_state(struct v4l2_subdev *sd)
{
  return container_of(sd, struct dohofpga2_state, sd);
}

static inline void debug_msleep(struct v4l2_subdev *sd, u32 msecs)
{
  cam_dbg("delay for %dms\n", msecs);
  msleep(msecs);
}

#if defined(FEATURE_YUV_CAPTURE)
/* JPEG MEMORY SIZE */
#define SENSOR_JPEG_OUTPUT_MAXSIZE  0x29999A /*2726298bytes, 2.6M */
#define EXTRA_MEMSIZE      (0 * SZ_1K)
#define SENSOR_JPEG_SNAPSHOT_MEMSIZE   0x99C600
#define SENSOR_JPEG_THUMB_MAXSIZE    0xFC00
#define SENSOR_JPEG_POST_MAXSIZE    0xBB800
#endif

/*********** Sensor specific ************/
#define DOHOFPGA2_I2C_ALLOW  0
#define DOHOFPGA2_CHIP_ID    0x1234
#define DOHOFPGA2_CHIP_REV   0x5678

#define FORMAT_FLAGS_COMPRESSED    0x3

#define POLL_TIME_MS      5
#define CAPTURE_POLL_TIME_MS    500

/* maximum time for one frame in norma light */
#define ONE_FRAME_DELAY_MS_NORMAL    66
/* maximum time for one frame in low light: minimum 10fps. */
#define ONE_FRAME_DELAY_MS_LOW      100
/* maximum time for one frame in night mode: 6fps */
#define ONE_FRAME_DELAY_MS_NIGHTMODE    166

/* maximum time for one frame at minimum fps (15fps) in normal mode */
#define NORMAL_MODE_MAX_ONE_FRAME_DELAY_MS     67
/* maximum time for one frame at minimum fps (4fps) in night mode */
#define NIGHT_MODE_MAX_ONE_FRAME_DELAY_MS     250

/* level at or below which we need to enable flash when in auto mode */
#define LOW_LIGHT_LEVEL    0x1D

/* level at or below which we need to use low light capture mode */
#define HIGH_LIGHT_LEVEL  0x80

#define FIRST_AF_SEARCH_COUNT   80
#define SECOND_AF_SEARCH_COUNT  80
#define AE_STABLE_SEARCH_COUNT  7 /* 4->7. but ae-unstable still occurs. */

#define FIRST_SETTING_FOCUS_MODE_DELAY_MS  100
#define SECOND_SETTING_FOCUS_MODE_DELAY_MS  200

/* Sensor AF first,second window size.
 * we use constant values intead of reading sensor register */
#define FIRST_WINSIZE_X      512
#define FIRST_WINSIZE_Y      568
#define SCND_WINSIZE_X      230
#define SCND_WINSIZE_Y      306

#ifdef __DOHOFPGA2_C__

static uint32_t dohofpga2_debug_mask = DOHOFPGA2_DEBUG_I2C_BURSTS | DOHOFPGA2_DEBUG_I2C;
module_param_named(debug_mask, dohofpga2_debug_mask, uint, S_IWUSR | S_IRUGO);

static int dohofpga2_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl);
static int dohofpga2_init(struct v4l2_subdev *sd, u32 val);

static const struct dohofpga2_fps dohofpga2_framerates[] = {
  { I_FPS_0,  FRAME_RATE_AUTO },
  { I_FPS_15,  FRAME_RATE_15 },
  { I_FPS_25,  FRAME_RATE_25 },
  { I_FPS_30,  FRAME_RATE_30 },
};

static const struct dohofpga2_framesize dohofpga2_preview_frmsizes[] = {
  { DOHOFPGA2_PREVIEW_QCIF,  176,  144 }, 
  { DOHOFPGA2_PREVIEW_CIF,    352,  288 }, 
  { DOHOFPGA2_PREVIEW_VGA,    640,  480 }, 
  { DOHOFPGA2_PREVIEW_D1,    720,  480 }, 
  { DOHOFPGA2_PREVIEW_960,    960,  640 }, 
  { DOHOFPGA2_PREVIEW_720P,   1280, 720 }, 
};

static const struct dohofpga2_framesize dohofpga2_capture_frmsizes[] = {
  { DOHOFPGA2_CAPTURE_VGA,    640,  480 },
  { DOHOFPGA2_CAPTURE_1MP,    1280, 960 },
  { DOHOFPGA2_CAPTURE_2MP,    1600, 1200 },
  { DOHOFPGA2_CAPTURE_3MP,    2048, 1536 },
  { DOHOFPGA2_CAPTURE_5MP,    2560, 1920 },
};

static struct dohofpga2_control dohofpga2_ctrls[] = {
  DOHOFPGA2_INIT_CONTROL(V4L2_CID_CAMERA_FLASH_MODE, \
          FLASH_MODE_OFF),

  DOHOFPGA2_INIT_CONTROL(V4L2_CID_CAMERA_BRIGHTNESS, \
          EV_DEFAULT),

  DOHOFPGA2_INIT_CONTROL(V4L2_CID_CAMERA_METERING, \
          METERING_MATRIX),

  DOHOFPGA2_INIT_CONTROL(V4L2_CID_CAMERA_WHITE_BALANCE, \
          WHITE_BALANCE_AUTO),

  DOHOFPGA2_INIT_CONTROL(V4L2_CID_CAMERA_EFFECT, \
          IMAGE_EFFECT_NONE),
};

static const struct dohofpga2_regs reg_datas = {
  .ev = {
    DOHOFPGA2_REGSET(EV_MINUS_4,   dohofpga2_EV_Minus_4),
    DOHOFPGA2_REGSET(EV_MINUS_3,   dohofpga2_EV_Minus_3),
    DOHOFPGA2_REGSET(EV_MINUS_2,   dohofpga2_EV_Minus_2),
    DOHOFPGA2_REGSET(EV_MINUS_1,   dohofpga2_EV_Minus_1),
    DOHOFPGA2_REGSET(EV_DEFAULT,   dohofpga2_EV_Default),
    DOHOFPGA2_REGSET(EV_PLUS_1,     dohofpga2_EV_Plus_1),
    DOHOFPGA2_REGSET(EV_PLUS_2,     dohofpga2_EV_Plus_2),
    DOHOFPGA2_REGSET(EV_PLUS_3,     dohofpga2_EV_Plus_3),
    DOHOFPGA2_REGSET(EV_PLUS_4,     dohofpga2_EV_Plus_4),

  },
  .metering = {
    DOHOFPGA2_REGSET(METERING_MATRIX,   dohofpga2_Metering_Matrix),
    DOHOFPGA2_REGSET(METERING_CENTER,   dohofpga2_Metering_Center),
    DOHOFPGA2_REGSET(METERING_SPOT,     dohofpga2_Metering_Spot),
  },
  .iso = {
    DOHOFPGA2_REGSET(ISO_AUTO,   dohofpga2_ISO_Auto),
    DOHOFPGA2_REGSET(ISO_100,   dohofpga2_ISO_100),
    DOHOFPGA2_REGSET(ISO_200,   dohofpga2_ISO_200),
    DOHOFPGA2_REGSET(ISO_400,   dohofpga2_ISO_400),
  },
  .effect = {
    DOHOFPGA2_REGSET(IMAGE_EFFECT_NONE,     dohofpga2_Effect_Normal),
    DOHOFPGA2_REGSET(IMAGE_EFFECT_BNW,     dohofpga2_Effect_Black_White),
    DOHOFPGA2_REGSET(IMAGE_EFFECT_SEPIA,   dohofpga2_Effect_Sepia),
    DOHOFPGA2_REGSET(IMAGE_EFFECT_NEGATIVE,   dohofpga2_Effect_Negative),
  },
  .white_balance = {
    DOHOFPGA2_REGSET(WHITE_BALANCE_AUTO,     dohofpga2_WB_Auto),
    DOHOFPGA2_REGSET(WHITE_BALANCE_SUNNY,     dohofpga2_WB_Sunny),
    DOHOFPGA2_REGSET(WHITE_BALANCE_CLOUDY,     dohofpga2_WB_Cloudy),
    DOHOFPGA2_REGSET(WHITE_BALANCE_TUNGSTEN,    dohofpga2_WB_Tungsten),
    DOHOFPGA2_REGSET(WHITE_BALANCE_FLUORESCENT,  dohofpga2_WB_Fluorescent),
  },
  .preview_size = {
    DOHOFPGA2_REGSET(DOHOFPGA2_PREVIEW_QCIF,    dohofpga2_176_Preview),
    DOHOFPGA2_REGSET(DOHOFPGA2_PREVIEW_CIF,     dohofpga2_352_Preview),
    DOHOFPGA2_REGSET(DOHOFPGA2_PREVIEW_VGA,     dohofpga2_640_Preview),
    DOHOFPGA2_REGSET(DOHOFPGA2_PREVIEW_D1,    dohofpga2_720_Preview),
    DOHOFPGA2_REGSET(DOHOFPGA2_PREVIEW_960,    dohofpga2_960_Preview),
    DOHOFPGA2_REGSET(DOHOFPGA2_PREVIEW_720P,    dohofpga2_1280_Preview),
  },
  .scene_mode = {
    DOHOFPGA2_REGSET(SCENE_MODE_NONE,       dohofpga2_Scene_Default),
    DOHOFPGA2_REGSET(SCENE_MODE_PORTRAIT,     dohofpga2_Scene_Portrait),
    DOHOFPGA2_REGSET(SCENE_MODE_NIGHTSHOT,     dohofpga2_Scene_Nightshot),
    DOHOFPGA2_REGSET(SCENE_MODE_LANDSCAPE,     dohofpga2_Scene_Landscape),
    DOHOFPGA2_REGSET(SCENE_MODE_SPORTS,       dohofpga2_Scene_Sports),
    DOHOFPGA2_REGSET(SCENE_MODE_PARTY_INDOOR,   dohofpga2_Scene_Party_Indoor),
    DOHOFPGA2_REGSET(SCENE_MODE_BEACH_SNOW,     dohofpga2_Scene_Beach_Snow),
    DOHOFPGA2_REGSET(SCENE_MODE_SUNSET,       dohofpga2_Scene_Sunset),
    DOHOFPGA2_REGSET(SCENE_MODE_DUSK_DAWN,    dohofpga2_Scene_Duskdawn),
    DOHOFPGA2_REGSET(SCENE_MODE_TEXT,       dohofpga2_Scene_Text),
    DOHOFPGA2_REGSET(SCENE_MODE_CANDLE_LIGHT,   dohofpga2_Scene_Candle_Light),
  },
  .saturation = {
    DOHOFPGA2_REGSET(SATURATION_MINUS_2,     dohofpga2_Saturation_Minus_2),
    DOHOFPGA2_REGSET(SATURATION_MINUS_1,     dohofpga2_Saturation_Minus_1),
    DOHOFPGA2_REGSET(SATURATION_DEFAULT,     dohofpga2_Saturation_Default),
    DOHOFPGA2_REGSET(SATURATION_PLUS_1,       dohofpga2_Saturation_Plus_1),
    DOHOFPGA2_REGSET(SATURATION_PLUS_2,       dohofpga2_Saturation_Plus_2),
  },
  .contrast = {
    DOHOFPGA2_REGSET(CONTRAST_MINUS_2,     dohofpga2_Contrast_Minus_2),
    DOHOFPGA2_REGSET(CONTRAST_MINUS_1,     dohofpga2_Contrast_Minus_1),
    DOHOFPGA2_REGSET(CONTRAST_DEFAULT,     dohofpga2_Contrast_Default),
    DOHOFPGA2_REGSET(CONTRAST_PLUS_1,    dohofpga2_Contrast_Plus_1),
    DOHOFPGA2_REGSET(CONTRAST_PLUS_2,     dohofpga2_Contrast_Plus_2),
  },
  .sharpness = {
    DOHOFPGA2_REGSET(SHARPNESS_MINUS_2,     dohofpga2_Sharpness_Minus_2),
    DOHOFPGA2_REGSET(SHARPNESS_MINUS_1,     dohofpga2_Sharpness_Minus_1),
    DOHOFPGA2_REGSET(SHARPNESS_DEFAULT,     dohofpga2_Sharpness_Default),
    DOHOFPGA2_REGSET(SHARPNESS_PLUS_1,     dohofpga2_Sharpness_Plus_1),
    DOHOFPGA2_REGSET(SHARPNESS_PLUS_2,     dohofpga2_Sharpness_Plus_2),
  },
  .fps = {
    DOHOFPGA2_REGSET(I_FPS_0,     dohofpga2_FPS_Auto),
    DOHOFPGA2_REGSET(I_FPS_15,     dohofpga2_FPS_15),
    DOHOFPGA2_REGSET(I_FPS_25,     dohofpga2_FPS_25),
    DOHOFPGA2_REGSET(I_FPS_30,     dohofpga2_FPS_30),
  },
  .jpeg_quality_high     =  DOHOFPGA2_REGSET_TABLE(dohofpga2_Jpeg_Quality_High),
  .jpeg_quality_normal   =  DOHOFPGA2_REGSET_TABLE(dohofpga2_Jpeg_Quality_Normal),
  .jpeg_quality_low     =   DOHOFPGA2_REGSET_TABLE(dohofpga2_Jpeg_Quality_Low),

  .preview_return     =  DOHOFPGA2_REGSET_TABLE(dohofpga2_Preview_Return),

  .flash_start       =  DOHOFPGA2_REGSET_TABLE(dohofpga2_Flash_Start),
  .flash_end         =  DOHOFPGA2_REGSET_TABLE(dohofpga2_Flash_End),

  .af_assist_flash_start   =  DOHOFPGA2_REGSET_TABLE(dohofpga2_Pre_Flash_Start),
  .af_assist_flash_end   =  DOHOFPGA2_REGSET_TABLE(dohofpga2_Pre_Flash_End),
  .af_low_light_mode_on   =  DOHOFPGA2_REGSET_TABLE(dohofpga2_AF_Low_Light_Mode_On),
  .af_low_light_mode_off   =  DOHOFPGA2_REGSET_TABLE(dohofpga2_AF_Low_Light_Mode_Off),

  .ae_awb_lock_on      =  DOHOFPGA2_REGSET_TABLE(dohofpga2_AE_AWB_Lock_On),
  .ae_awb_lock_off    =  DOHOFPGA2_REGSET_TABLE(dohofpga2_AE_AWB_Lock_Off),

  .restore_cap      =  DOHOFPGA2_REGSET_TABLE(dohofpga2_restore_capture_reg),
  .change_wide_cap     =   DOHOFPGA2_REGSET_TABLE(dohofpga2_change_wide_cap),

  .af_macro_mode_1     = DOHOFPGA2_REGSET_TABLE(dohofpga2_AF_Macro_mode_1),
  .af_macro_mode_2     = DOHOFPGA2_REGSET_TABLE(dohofpga2_AF_Macro_mode_2),
  .af_macro_mode_3     = DOHOFPGA2_REGSET_TABLE(dohofpga2_AF_Macro_mode_3),
  .af_normal_mode_1    = DOHOFPGA2_REGSET_TABLE(dohofpga2_AF_Normal_mode_1),
  .af_normal_mode_2    = DOHOFPGA2_REGSET_TABLE(dohofpga2_AF_Normal_mode_2),
  .af_normal_mode_3    = DOHOFPGA2_REGSET_TABLE(dohofpga2_AF_Normal_mode_3),

  .single_af_start = DOHOFPGA2_REGSET_TABLE(dohofpga2_Single_AF_Start),
  .video_af_start = DOHOFPGA2_REGSET_TABLE(dohofpga2_Video_AF_Start),
  .single_af_off_1 = DOHOFPGA2_REGSET_TABLE(dohofpga2_Single_AF_Off_1),
  .single_af_off_2 = DOHOFPGA2_REGSET_TABLE(dohofpga2_Single_AF_Off_2),

  .capture_start = {
    DOHOFPGA2_REGSET(DOHOFPGA2_CAPTURE_VGA, dohofpga2_VGA_Capture),
    DOHOFPGA2_REGSET(DOHOFPGA2_CAPTURE_1MP, dohofpga2_1M_Capture),
    DOHOFPGA2_REGSET(DOHOFPGA2_CAPTURE_2MP, dohofpga2_2M_Capture),
    DOHOFPGA2_REGSET(DOHOFPGA2_CAPTURE_3MP, dohofpga2_3M_Capture),
    DOHOFPGA2_REGSET(DOHOFPGA2_CAPTURE_5MP, dohofpga2_5M_Capture),
  },
  .init_arm        =   DOHOFPGA2_REGSET_TABLE(dohofpga2_init_arm),
  .init_reg         =   DOHOFPGA2_REGSET_TABLE(dohofpga2_init_reg),
  .get_esd_status     =   DOHOFPGA2_REGSET_TABLE(dohofpga2_get_esd_status),
  .stream_stop       =   DOHOFPGA2_REGSET_TABLE(dohofpga2_stream_stop_reg),
  .get_light_level     =   DOHOFPGA2_REGSET_TABLE(dohofpga2_Get_Light_Level),

  .get_1st_af_search_status =
      DOHOFPGA2_REGSET_TABLE(dohofpga2_get_1st_af_search_status),
  .get_2nd_af_search_status =
      DOHOFPGA2_REGSET_TABLE(dohofpga2_get_2nd_af_search_status),
  .get_capture_status =
    DOHOFPGA2_REGSET_TABLE(dohofpga2_get_capture_status),

  .get_iso         =   DOHOFPGA2_REGSET_TABLE(dohofpga2_get_iso_reg),
  .get_ev           =   DOHOFPGA2_REGSET_TABLE(dohofpga2_get_ev_reg),
  .get_ae_stable       =   DOHOFPGA2_REGSET_TABLE(dohofpga2_Get_AE_Stable_Status),
  .get_shutterspeed     =   DOHOFPGA2_REGSET_TABLE(dohofpga2_get_shutterspeed_reg),
  .update_preview     =   DOHOFPGA2_REGSET_TABLE(dohofpga2_update_preview_reg),
  .update_hd_preview     =   DOHOFPGA2_REGSET_TABLE(dohofpga2_update_hd_preview_reg),
};

static const struct v4l2_mbus_framefmt capture_fmts[] = {
  {
    .code    = V4L2_MBUS_FMT_FIXED,
    .colorspace  = V4L2_COLORSPACE_JPEG,
  },
};

#define DOHOFPGA2_I2C_READ      0x0001
#define DOHOFPGA2_I2C_WRITE     0x0002
#define DOHOFPGA2_I2C_TRANSFER  0x0004
#define DOHOFPGA2_I2C_ENABLE    ( DOHOFPGA2_I2C_READ | \
                                  DOHOFPGA2_I2C_WRITE | \
                                  DOHOFPGA2_I2C_TRANSFER \
                                )

#define DOHOFPGA2_I2C_DISABLE   0x0000

static u16 dohofpga2_i2c      = DOHOFPGA2_I2C_DISABLE ;

#endif // ifdef __DOHOFPGA2_C__

#endif // ifndef __DOHOFPGA2_H__
