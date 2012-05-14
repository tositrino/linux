/*
 * Copyright (c) 2004, 2005, id Quantique SA, Switzerland
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * Neither the name of id Quantique nor the names of its contributors may be
 * used to endorse or promote products derived from this software without
 * specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef QUANTIS_IOCTL_H
#define QUANTIS_IOCTL_H

#ifdef __cplusplus
extern "C" {
#endif 
	
#ifdef linux
#ifndef __KERNEL__
#include <sys/cdefs.h>
#include <sys/ioctl.h>
#endif
#endif /* linux */
  
#ifdef __FreeBSD__
#ifndef KERNEL
#include <sys/types.h>
#endif
#include <sys/ioccom.h>
#endif /* __FreeBSD__ */

#if defined(__SVR4) && defined(__sun)
#include <sys/modctl.h>
#endif

  /* Magic key to ensure IOCTL are OK */
#define QUANTIS_IOC_MAGIC  'q'
  
  /* get driver version */
#define QUANTIS_IOCTL_GET_DRIVER_VERSION _IOR(QUANTIS_IOC_MAGIC, 0, unsigned int)
  
  /* get number of detected cards */
#define QUANTIS_IOCTL_GET_CARD_COUNT     _IOR(QUANTIS_IOC_MAGIC, 1, unsigned int)
  
  /* get mask of detected modules */
#define QUANTIS_IOCTL_GET_MODULES_MASK   _IOR(QUANTIS_IOC_MAGIC, 2, unsigned int)
  
  /* get card serial number */
#define QUANTIS_IOCTL_GET_BOARD_VERSION  _IOR(QUANTIS_IOC_MAGIC, 3, unsigned int)
  
  /* reset one board */
#define QUANTIS_IOCTL_RESET_BOARD        _IO(QUANTIS_IOC_MAGIC, 4)
  
  /* enable mask module */
#define QUANTIS_IOCTL_ENABLE_MODULE      _IOW(QUANTIS_IOC_MAGIC, 5, unsigned int)
  
  /* disable mask modules */
#define QUANTIS_IOCTL_DISABLE_MODULE     _IOW(QUANTIS_IOC_MAGIC, 6, unsigned int)
  
  /* set debug level */
#define QUANTIS_IOCTL_SET_DEBUG_LEVEL    _IOW(QUANTIS_IOC_MAGIC, 7, unsigned int)
  
  /* get status of modules */
#define QUANTIS_IOCTL_GET_MODULES_STATUS _IOR(QUANTIS_IOC_MAGIC, 8, unsigned int)
  
  /* max number of IOCTL */
#define QUANTIS_IOCTL_MAXNR 8

#ifdef __cplusplus
}
#endif
	
  
#endif // QUANTIS_IOCTL_H
