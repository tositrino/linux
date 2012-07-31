# Makefile for ecam32 v4l2 driver

ccflags-$(CONFIG_VIDEO_ECAM32_DEBUG) += -DDEBUG

v4l2_driver-objs	:=	init_module.o isp.o omap_hwr_base.o 				\
				ov3640/sens_ov3640.o  v4l2_driver_base.o                  \
				omap_camera_interface.o omap_v4l2_fops_base.o  			\
				isp.o omap_v4l2_ioctl_fops.o i2c.o				\
				lm3553/lm3553_flash.o exit_module.o					\
				resource/src/error.o resource/src/kernel/phy_mem.o

obj-$(CONFIG_VIDEO_ECAM32) += v4l2_driver.o
