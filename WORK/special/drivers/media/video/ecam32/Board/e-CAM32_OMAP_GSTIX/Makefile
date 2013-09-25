v4l2_driver-objs	:=	init_module.o isp.o omap_hwr_base.o 				\
				ov3640/sens_ov3640.o  v4l2_driver_base.o                  \
				omap_camera_interface.o omap_v4l2_fops_base.o  			\
				isp.o omap_v4l2_ioctl_fops.o i2c.o				\
				lm3553/lm3553_flash.o exit_module.o					\
				resource/src/error.o resource/src/kernel/phy_mem.o

obj-m			+=v4l2_driver.o

omap:
	make -C $(KERNEL_PATH) M=$(PWD) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) modules
clean:
	make -C $(KERNEL_PATH) M=$(PWD) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) clean
