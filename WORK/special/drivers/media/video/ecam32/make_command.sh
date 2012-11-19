BOARD_NAME="clean"
CONFIG_BUILD_ALL="0"

function setup_environment
{
	echo "-----------------------------------------------------------------------------------------"
	echo "                        Configure Build for $BOARD_NAME		                       "
	echo "-----------------------------------------------------------------------------------------"

	if [ Board/$BOARD_NAME/auto_conf.h -nt auto_conf.h ]; then
		cp Board/$BOARD_NAME/auto_conf.h ./
		touch Board/$BOARD_NAME/auto_conf.h
	fi

	if [ Board/$BOARD_NAME/auto_conf.h -ot auto_conf.h ]; then
		cp Board/$BOARD_NAME/auto_conf.h ./
		touch Board/$BOARD_NAME/auto_conf.h
	fi
	cp Board/$BOARD_NAME/Makefile ./

	svn update -q

	SVN_REVISION=`svn info | grep 'Revision' | cut -d ' ' -f 2`
	SVN_DATE=`svn info | grep 'Date' | cut -d ' ' -f 4`
	SVN_TIME=`svn info | grep 'Date' | cut -d ' ' -f 5`

	if [ -z $SVN_REVISION ]; then
		echo "Warning! Please use svn version of source copy."
	else
		echo "#define PRODUCT_NAME \"$BOARD_NAME\""  > tmp.h
		echo "#define SVN_REVISION \"$SVN_REVISION\"" >> tmp.h
		echo "#define SVN_DATE \"$SVN_DATE\"" >> tmp.h
		echo "#define SVN_TIME \"$SVN_TIME\"" >> tmp.h

		cmp -s tmp.h svn_revision.h
		RET_VAL=$?
		if [ $RET_VAL -eq 0 ]; then
			rm tmp.h
		else
			mv tmp.h svn_revision.h
		fi
	fi
}

function make_check_error
{
	if [ $? = "0" ]; then
		echo "                                                                                         "
		echo "                          Build completed successfully                                   "
		echo "-----------------------------------------------------------------------------------------"
		return 0
	else
		echo "                                                                                         "
		echo "                              :-( Build Failed :-(                                       "
		echo "-----------------------------------------------------------------------------------------"
		return 1
	fi
}

if [ -z $1 ]; then
	echo "==============================================================================================================="
	echo "Please select any one of the parameter as first argument in this shell script"
	echo " clean                               = Clean the build environment for you"

	echo "-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-"
	echo " e-CAM50_OMAP_GSTIX                  = gumstix 5MP using ov5642 in j5 connector of processor board"
	echo " e-CAM50_CU35x_GSTIX                 = gumstix 5MP using ov5642 custom lens setup module in j5 connector of processor board"
	echo "-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-"
	echo " e-CAM50_OMAP35x                     = omap35x 5MP using ov5642 A1 connector of omap3-evm-revG Board"
	echo " e-CAM50_OMAP35x_Linux_2.6.32        = omap35x 5MP using ov5642 A1 connector of omap3-evm-revG Board"
	echo " e-CAM50_CU35x                       = omap35x 5MP using ov5642 custom lens in Micron connector"
	echo " e-CAM50_OMAP35x_MICRON_JAMEERR      = omap evm rev-G using ov5642 in Micron connector"
	echo "-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-"
	echo " e-CAM50_DM37x                       = DM37x 5MP using ov5642 in Micron connector of omap3-evm-revG Board"
	echo " e-CAM50_CU37x                       = DM37x 5MP using ov5642 custom lens in Micron connector"
	echo "---------------------------------------------------------------------------------------------------------------"
	echo " e-CAM32_OMAP_GSTIX                  = gumstix 3MP using ov3640 in j5 connector of processor board"
	echo " e-CAM32_OMAP35x                     = omap35x 3MP using ov3640 A1 connector of omap3-evm-revG Board"
	echo " e-CAM32_OMAP35x_MICRON              = omap35x 3MP using ov3640 in Micron connector of omap3-evm-revG Board"
	echo "-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-"
	echo " e-CAM32_DM37x                       = DM37x 3MP using ov3640 in Micron connector of omap3-evm-revG Board"
	echo "---------------------------------------------------------------------------------------------------------------"
	echo " DEP_e-CAM1M_CU35x                   = omap evm rev-G using ov10630 in Micron connector"
	echo " e-CAM1M_CU35x                       = omap evm rev-G using ov10633 in Micron connector"
	echo "-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-"
	echo " e-CAM1M_DM37x                       = DM37x omap evm rev-G using ov10630 in Micron connector"
	echo "---------------------------------------------------------------------------------------------------------------"
	echo " build_all                           = Check the integraty of the development with all the boards "
	echo "---------------------------------------------------------------------------------------------------------------"
	echo ""
	echo "Example :"
	echo ". ./make_command.sh clean"
	echo "==============================================================================================================="
else
	if [ $1 = "clean" ]; then
		rm -rf `find ./ -name "*.o" -o -name "*.o.cmd" -o -name "*.ko"  \
			-o -name "*.sh.swp" -o -name ".tmp_versions" 		\
			-o -name "*.ko.cmd" -o -name "*.mod.*" 			\
			-o -name "*.||der" -o -name "*.symvers"			\
			-o -name "*.*.swo" -o -name ".*.o.d" 			\
			-o -name "*.order" -o -name "tags"`
	else
		if [ $1 = "build_all" ]; then
			 CONFIG_BUILD_ALL="1"
		fi
		if [ $1 = "e-CAM50_OMAP_GSTIX" ] || [ $CONFIG_BUILD_ALL = "1" ]; then
			BOARD_NAME="e-CAM50_OMAP_GSTIX"
			setup_environment
			echo "Select the kernel version"
			echo "1 -> 2.6.32-psp-102g"
			echo "2 -> 2.6.34-r100 "
			read KERNEL_VERSION

			if [ $KERNEL_VERSION = "1" ]; then
				export KERNEL_PATH_SELECTED=/media/hdd1/svn/camera_products/e-CAM50_OMAP_GSTIX/linux-omap-psp-2.6.32-r102g 
			else
				export KERNEL_PATH_SELECTED=/media/hdd1/svn/camera_products/e-CAM50_OMAP_GSTIX/linux-omap3-2.6.34-r100
			fi
			make	KERNEL_PATH=$KERNEL_PATH_SELECTED \
				CROSS_COMPILE=/media/hdd1/svn/sdk/toolchain/arm-2008q1/bin/arm-none-linux-gnueabi-
			make_check_error
			if [ $? = "1" ]; then
				return $?
			fi
		fi

		if [ $1 = "e-CAM50_CU35x_GSTIX" ] || [ $CONFIG_BUILD_ALL = "1" ]; then
			BOARD_NAME="e-CAM50_CU35x_GSTIX"
			setup_environment
			make 	KERNEL_PATH=/media/hdd1/svn/camera_products/e-CAM50_CU35x_GSTIX/linux-omap3-2.6.34-r100 \
				CROSS_COMPILE=/media/hdd1/svn/sdk/toolchain/arm-2008q1/bin/arm-none-linux-gnueabi-
			make_check_error
			if [ $? = "1" ]; then
				return $?
			fi
		fi

		if [ $1 = "e-CAM50_OMAP35x" ] || [ $CONFIG_BUILD_ALL = "1" ]; then
			BOARD_NAME="e-CAM50_OMAP35x"
			setup_environment
			make 	KERNEL_PATH=/media/hdd1/svn/camera_products/e-CAM50_OMAP35x/linux-02.01.03.11 \
				CROSS_COMPILE=/media/hdd1/svn/sdk/toolchain/arm-2008q1/bin/arm-none-linux-gnueabi-
			make_check_error
			if [ $? = "1" ]; then
				return $?
			fi
		fi

		if [ $1 = "e-CAM50_OMAP35x_Linux_2.6.32" ] || [ $CONFIG_BUILD_ALL = "1" ]; then
			BOARD_NAME="e-CAM50_OMAP35x_Linux_2.6.32"
			setup_environment
			make 	KERNEL_PATH=/media/hdd1/svn/camera_products/e-CAM50_OMAP35x/linux-03.00.01.06 \
				CROSS_COMPILE=/media/hdd1/svn/sdk/toolchain/arm-2008q1/bin/arm-none-linux-gnueabi-
			make_check_error
			if [ $? = "1" ]; then
				return $?
			fi
		fi

		if [ $1 = "e-CAM50_CU35x" ] || [ $CONFIG_BUILD_ALL = "1" ]; then
			BOARD_NAME="e-CAM50_CU35x"
			setup_environment
			make 	KERNEL_PATH=/media/hdd1/svn/camera_products/e-CAM50_CU5642_MOD/linux-02.01.03.11 \
				CROSS_COMPILE=/media/hdd1/svn/sdk/toolchain/arm-2008q1/bin/arm-none-linux-gnueabi-
			make_check_error
			if [ $? = "1" ]; then
				return $?
			fi
		fi

		if [ $1 = "e-CAM50_OMAP35x_MICRON_JAMEERR" ] || [ $CONFIG_BUILD_ALL = "1" ]; then
			BOARD_NAME="e-CAM50_OMAP35x_MICRON_JAMEERR"
			setup_environment
			make KERNEL_PATH=/media/hdd1/svn/camera_products/e-CAM50_CU5642_MOD/linux-02.01.03.11 \
				CROSS_COMPILE=/media/hdd1/svn/sdk/toolchain/arm-2008q1/bin/arm-none-linux-gnueabi-
			make_check_error
			if [ $? = "1" ]; then
				return $?
			fi
		fi

		if [ $1 = "e-CAM50_DM37x" ] || [ $CONFIG_BUILD_ALL = "1" ]; then
			BOARD_NAME="e-CAM50_DM37x"
			setup_environment
			make 	KERNEL_PATH=/media/hdd1/svn/camera_products/e-CAM50_DM37x/linux-03.00.01.06 \
				CROSS_COMPILE=/media/hdd1/svn/sdk/omap37x/AM35x-OMAP35x-PSP-SDK-03.00.01.06/tool-chain/arm-2009q1/bin/arm-none-linux-gnueabi-
			make_check_error
			if [ $? = "1" ]; then
				return $?
			fi
		fi

		if [ $1 = "e-CAM50_CU37x" ] || [ $CONFIG_BUILD_ALL = "1" ]; then
			BOARD_NAME="e-CAM50_CU37x"
			setup_environment
			make 	KERNEL_PATH=/media/hdd1/svn/camera_products/e-CAM50_CU37x/linux-03.00.01.06 \
				CROSS_COMPILE=/media/hdd1/svn/sdk/omap37x/AM35x-OMAP35x-PSP-SDK-03.00.01.06/tool-chain/arm-2009q1/bin/arm-none-linux-gnueabi-
			make_check_error
			if [ $? = "1" ]; then
				return $?
			fi
		fi

		if [ $1 = "e-CAM32_OMAP_GSTIX" ] || [ $CONFIG_BUILD_ALL = "1" ]; then
			BOARD_NAME="e-CAM32_OMAP_GSTIX"
			setup_environment
			echo "Select the kernel version"
			echo "1 -> 2.6.34 "
			echo "2 -> 2.6.39 "
			read KERNEL_VERSION

			if [ $KERNEL_VERSION = "1" ]; then
				export KERNEL_PATH_SELECTED=/media/hdd1/svn/camera_products/e-CAM32_OMAP_GSTIX/gumstix_ov3640_kernel/linux-omap3-2.6.34-r81 
			else
				export KERNEL_PATH_SELECTED=/media/hdd1/svn/camera_products/e-CAM32_OMAP_GSTIX/linux-omap3-2.6.39-r102
			fi
			make	KERNEL_PATH=$KERNEL_PATH_SELECTED \
				CROSS_COMPILE=/media/hdd1/svn/sdk/toolchain/arm-2008q1/bin/arm-none-linux-gnueabi-
			make_check_error
			if [ $? = "1" ]; then
				return $?
			fi
		fi

		if [ $1 = "e-CAM32_OMAP35x" ] || [ $CONFIG_BUILD_ALL = "1" ]; then
			BOARD_NAME="e-CAM32_OMAP35x"
			setup_environment
			make 	KERNEL_PATH=/media/hdd1/svn/camera_products/e-CAM32_OMAP35x/linux-02.01.03.11 \
				CROSS_COMPILE=/media/hdd1/svn/sdk/toolchain/arm-2008q1/bin/arm-none-linux-gnueabi-
			make_check_error
			if [ $? = "1" ]; then
				return $?
			fi
		fi

		if [ $1 = "e-CAM32_OMAP35x_MICRON" ] || [ $CONFIG_BUILD_ALL = "1" ]; then
			BOARD_NAME="e-CAM32_OMAP35x_MICRON"
			setup_environment
			make 	KERNEL_PATH=/media/hdd1/svn/camera_products/e-CAM32_OMAP35x_MICRON/linux-02.01.03.11/ \
				CROSS_COMPILE=/media/hdd1/svn/sdk/toolchain/arm-2008q1/bin/arm-none-linux-gnueabi-
			make_check_error
			if [ $? = "1" ]; then
				return $?
			fi
		fi

		if [ $1 = "e-CAM32_DM37x" ] || [ $CONFIG_BUILD_ALL = "1" ]; then
			BOARD_NAME="e-CAM32_DM37x"
			setup_environment
			make 	KERNEL_PATH=/media/hdd1/svn/camera_products/e-CAM32_DM37x/linux-03.00.01.06 \
				CROSS_COMPILE=/media/hdd1/svn/sdk/omap37x/AM35x-OMAP35x-PSP-SDK-03.00.01.06/tool-chain/arm-2009q1/bin/arm-none-linux-gnueabi-
			make_check_error
			if [ $? = "1" ]; then
				return $?
			fi
		fi

		if [ $1 = "DEP_e-CAM1M_CU35x" ] || [ $CONFIG_BUILD_ALL = "1" ]; then
			BOARD_NAME="DEP_e-CAM1M_CU35x"
			setup_environment
			make KERNEL_PATH=/media/hdd1/svn/camera_products/DEP_e-CAM1M_CU35x/linux-02.01.03.11 \
				CROSS_COMPILE=/media/hdd1/svn/sdk/toolchain/arm-2008q1/bin/arm-none-linux-gnueabi-
			make_check_error
			if [ $? = "1" ]; then
				return $?
			fi
		fi

		if [ $1 = "e-CAM1M_CU35x" ] || [ $CONFIG_BUILD_ALL = "1" ]; then
			BOARD_NAME="e-CAM1M_CU35x"
			setup_environment
			make KERNEL_PATH=/media/hdd1/svn/camera_products/e-CAM1M_CU35x/linux-02.01.03.11 \
				CROSS_COMPILE=/media/hdd1/svn/sdk/toolchain/arm-2008q1/bin/arm-none-linux-gnueabi-
			make_check_error
			if [ $? = "1" ]; then
				return $?
			fi
		fi

		if [ $1 = "e-CAM1M_DM37x" ] || [ $CONFIG_BUILD_ALL = "1" ]; then
			BOARD_NAME="e-CAM1M_DM37x"
			setup_environment
			make 	KERNEL_PATH=/media/hdd1/svn/camera_products/e-CAM1M_DM37x/linux-03.00.01.06 \
				CROSS_COMPILE=/media/hdd1/svn/sdk/omap37x/AM35x-OMAP35x-PSP-SDK-03.00.01.06/tool-chain/arm-2009q1/bin/arm-none-linux-gnueabi-
			make_check_error
			if [ $? = "1" ]; then
				return $?
			fi
		fi
	fi
 fi
