/*
 * Copyright (c) 2004, 2005, 2006 id Quantique SA, Switzerland
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

/**# Introduction **/

/**## Preamble **/

/**## Kernel includes **/
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/ioport.h>
#include <linux/pci.h>
#include <linux/sched.h>

#include <asm/ioctl.h>
#include <asm/uaccess.h>
#include <asm/io.h>

#ifdef USE_DEVFS
#include <linux/devfs_fs_kernel.h>
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 0))
#define LINUX_2_6__
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 14))
#define pm_message_t u32
#endif

#ifndef module_param
#define module_param(v,t,p) MODULE_PARM(v, "i");
#endif

/** We also include the file containing the definition of our IOCTL
 ** command numbers.
 **/
#include "quantisio.h"

#define DRIVER_NAME "Quantis-PCI driver"
#define DRIVER_SHORT_NAME "qrandom"

#define QUANTIS_DEFAULT_MAJOR 0
#define QUANTIS_MAX_CARDS 10

#define TOSTRING(__STRING__) #__STRING__
#define QUANTIS_DESC_MAJOR(__NUMBER__) TOSTRING(quantis major number (default __NUMBER__))
#define QUANTIS_DESC_TIMEOUT(__NUMBER__) TOSTRING(quantis timeout (default __NUMBER__))

/**## Module parameters
 **
 **/

/** When the major number is 0, it is automatically assigned by the
 ** kernel.
 **/
int quantisMajor = QUANTIS_DEFAULT_MAJOR;
module_param(quantisMajor, int, 0);
MODULE_PARM_DESC(quantisMajor, QUANTIS_DESC_MAJOR(QUANTIS_DEFAULT_MAJOR));

#define QUANTIS_TIMEOUT 1000
int quantisTimeOut = QUANTIS_TIMEOUT;
module_param(quantisTimeOut, int, 0);
MODULE_PARM_DESC(quantisTimeOut, QUANTIS_DESC_TIMEOUT(QUANTIS_TIMEOUT));

#if defined(USE_DEVFS) && defined(REPLACE_DEV_RANDOM)
int quantisReplaceRandom = -1;
MODULE_PARM(quantisReplaceRandom, "i");
MODULE_PARM_DESC(quantisReplaceRandom,
         "(USE AT YOUR OWN RISK): Replace /dev/random and /dev/urandom with "
         "quantis random number generator (default off)");
#endif

/**## Logging routines **/
#define QUANTIS_INFO(fmt, args...) \
  printk(KERN_INFO "%s: " fmt "\n" , DRIVER_SHORT_NAME , ## args)
#define QUANTIS_WARNING(fmt, args...) \
printk(KERN_WARNING "%s: WARNING " fmt "\n" , DRIVER_SHORT_NAME , ## args)
#define QUANTIS_ERROR(fmt, args...) \
  printk(KERN_ERR "%s: ERROR " fmt "\n" , DRIVER_SHORT_NAME , ## args)

#ifdef DEBUG
#define QUANTIS_DEBUG(fmt, args...) \
  printk(KERN_INFO "%s: " fmt "\n" , DRIVER_SHORT_NAME , ## args)
#else
#define QUANTIS_DEBUG(fmt, args...)
#endif

/**# PCI card structures **/

#define INLINE inline
struct quantis_card_s;
#define QUANTIS_SOFT_T struct quantis_card_s
#include "quantis-common.h"

typedef struct quantis_card_s {
  struct pci_dev   *bus;
  u_int32_t        *regs;
  struct semaphore mutex;
  unsigned char    buffer[4 * QUANTIS_FIFO_SIZE];

#ifdef USE_DEVFS
  devfs_handle_t   handle;
#endif
} quantis_card_t;

#if defined(USE_DEVFS)
/** Handle for the qrandom directory. **/
devfs_handle_t dir_handle;

#if defined(REPLACE_DEV_RANDOM)
struct file_operations *random_prev_ops = NULL;
devfs_handle_t random_handle = NULL;
struct file_operations *urandom_prev_ops = NULL;
devfs_handle_t urandom_handle = NULL;
#endif 
#endif /* USE_DEVFS */

/* XXX use mutexes around global variables */
quantis_card_t   quantisCards[QUANTIS_MAX_CARDS];
int              quantisCardCount = 0;
struct semaphore quantisMutex;

/**## Shared functions
 **
 ** Drivers are not allowed to directly access the hardware, but have
 ** to use the shared quantis functions.
 **
 ** We use two macros to read and write PCI card registers, using the
 ** bus space interface. We always read 32 bits values. We define the
 ** `quantis_soft_state_t' type for the shared functions, and inline
 ** them, too.
 **/
#define QUANTIS_REG(scp, reg) ((scp)->regs[reg / 4])
#define QUANTIS_SET_REG(scp, reg, val) (scp)->regs[(reg / 4)] = (val)
#define QUANTIS_DEBUG0 QUANTIS_DEBUG
#define QUANTIS_DEBUG2 QUANTIS_DEBUG

/** After this setup, we can include the common functions. **/
#include "quantis-common.c"

static int quantis_open(struct inode *inode, struct file *file);
static int quantis_close(struct inode *inode, struct file *file);
static ssize_t quantis_read(struct file *file, char *buffer,
                            size_t length, loff_t *ppos);
static int quantis_ioctl(struct inode *inode, struct file *file,
                         unsigned int cmd, unsigned long arg);

static struct file_operations quantis_fops = {
  owner:    THIS_MODULE,
  read:     quantis_read,
  //ioctl:    quantis_ioctl,
  open:     quantis_open,
  release:  quantis_close
};

/**# Kernel PCI interface routines **/
__devinitdata static struct pci_device_id pci_ids[] = {
  { QUANTIS_VENDOR_ID, PCI_ANY_ID, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
  { 0, }
};
MODULE_DEVICE_TABLE(pci, pci_ids);

/**## Power management **/
#ifdef CONFIG_PM
static int quantis_suspend(struct pci_dev *pdev, pm_message_t state) {
  return 0;
}

static int quantis_resume(struct pci_dev *pdev) {
  return 0;
}
#endif

/**## File operations **/
static int quantis_open(struct inode *inode, struct file *file) {
  int card = MINOR(inode->i_rdev);
  if(card > (quantisCardCount - 1))
    return -ENODEV;

#ifdef LINUX_2_6__
  if (!try_module_get(THIS_MODULE)) {
    return -EBUSY;
  }
#else
  MOD_INC_USE_COUNT;
#endif

  file->private_data = &quantisCards[card];
  return 0;
}

static int quantis_close(struct inode *inode, struct file *file) {
#ifdef LINUX_2_6__
  module_put(THIS_MODULE);
#else
  MOD_DEC_USE_COUNT;
#endif

  return 0;
}

static ssize_t quantis_read(struct file *file, char *buffer,
                            size_t length, loff_t *ppos) {
  int ret;
  quantis_card_t *device;

  /** Verify this is a write access. **/
  if(!access_ok(VERIFY_WRITE, buffer, length)) {
    QUANTIS_ERROR("write access denied for buffer %p, length 0x%08x",
                  buffer, length);
    return -EFAULT;
  }

  device = file->private_data;
  if (down_interruptible(&(device->mutex)))
    return -ERESTARTSYS;

  ret = quantis_rng_read(device, device->buffer, length);

  if (ret < 0) {
    ret = -EIO;
  } else {
      if (__copy_to_user(buffer, device->buffer, ret))
          ret = -EFAULT;
  }

  up(&(device->mutex));
  return ret;
}

static int quantis_ioctl(struct inode *inode, struct file *file,
                         unsigned int cmd, unsigned long arg)
{
  int status;
  unsigned int card = MINOR(inode->i_rdev);
  quantis_card_t *device;

  device = &quantisCards[card];
  if(down_interruptible(&device->mutex))
    return -ERESTARTSYS;

  switch (cmd) {
  case QUANTIS_IOCTL_GET_DRIVER_VERSION:
    status = put_user((u_int32_t)QUANTIS_DRIVER_VERSION, (u_int32_t *)arg);
    break;

  case QUANTIS_IOCTL_GET_CARD_COUNT:
    status = put_user((u_int32_t)quantisCardCount, (u_int32_t *)arg);
    break;

  case QUANTIS_IOCTL_GET_BOARD_VERSION:
    {
      u_int32_t version = quantis_rng_version(device);
      status = put_user(version, (u_int32_t *)arg);
      break;
    }

  case QUANTIS_IOCTL_RESET_BOARD:
    quantis_rng_reset(device);
    status = 0;
    break;

  case QUANTIS_IOCTL_GET_MODULES_MASK:
    {
      u_int32_t mask =  quantis_rng_modules_mask(device);
      status = put_user(mask, (u_int32_t *)arg);
      break;
    }

  case QUANTIS_IOCTL_ENABLE_MODULE:
    {
      u_int32_t modules;
      get_user(modules, (u_int32_t *)arg);
      quantis_rng_enable_modules(device, modules);
      status = 0;
    }
    break;

  case QUANTIS_IOCTL_DISABLE_MODULE:
    {
      u_int32_t modules;
      get_user(modules, (u_int32_t *)arg);
      quantis_rng_disable_modules(device, modules);
      status = 0;
    }
    break;

  case QUANTIS_IOCTL_GET_MODULES_STATUS:
    {
      u_int32_t modules_status;
      modules_status = quantis_rng_modules_status(device);
      status = put_user(modules_status, (u_int32_t *)arg);
    }
    break;

  default:
    QUANTIS_DEBUG("no such IOCTL");
    status = -ENOTTY;
  }

  up(&device->mutex);
  return status;
}

#if defined(USE_DEVFS) && defined(REPLACE_DEV_RANDOM)
static devfs_handle_t quantis_devfs_replace(char *name,
                                            unsigned int major,
                                            unsigned int minor,
                                            struct file_operations **old_ops,
                                            unsigned int new_major,
                                            unsigned int new_minor,
                                            struct file_operations *new_ops) {
  devfs_handle_t old_h, new_h;
  char fulldevname[16];

  old_h = devfs_find_handle(NULL, name, major, minor, DEVFS_SPECIAL_CHR, 0);
  if (old_h == NULL)
    return NULL;

  *old_ops = devfs_get_ops(old_h);
  sprintf(fulldevname, DRIVER_SHORT_NAME "/%d", new_minor);

  QUANTIS_DEBUG("replacing %s device with /dev/%s", name, fulldevname);
  devfs_unregister(old_h);
  if ((new_h = devfs_register(NULL, name, DEVFS_FL_NONE,
                              new_major, new_minor,
                              S_IRUGO | S_IFCHR | S_IWUGO,
                              new_ops, 0)) == NULL) {
    /* restore random */
    if (!devfs_register(NULL, name, DEVFS_FL_NONE, major, minor,
                        S_IRUGO | S_IWUSR | S_IFCHR, *old_ops, NULL)) {
      QUANTIS_DEBUG("Could not restore %s", name);
      *old_ops = NULL;
      return NULL;
    }
  }
  return new_h;
}

static void quantis_restore_random(void) {
  if (random_prev_ops && random_handle) {
    QUANTIS_DEBUG("restoring random device");
    devfs_unregister(random_handle);
    if (!devfs_register(NULL, "random", DEVFS_FL_NONE,
                       1, 8, S_IRUGO | S_IWUSR | S_IFCHR, random_prev_ops, NULL))
      QUANTIS_DEBUG("Could not restore /dev/random");
    urandom_handle = NULL;
    urandom_prev_ops = NULL;
  }

  if (urandom_prev_ops && urandom_handle) {
    QUANTIS_DEBUG("restoring urandom device");
    devfs_unregister(urandom_handle);
    if (!devfs_register(NULL, "urandom", DEVFS_FL_NONE,
                        1, 9, S_IRUGO | S_IWUSR | S_IFCHR,
                        urandom_prev_ops, NULL))
        QUANTIS_DEBUG("Could not restore /dev/urandom");

    urandom_handle = NULL;
    urandom_prev_ops = NULL;
  }
}

static int quantis_replace_random(int card) {
  random_handle = quantis_devfs_replace("random", 1, 8, &random_prev_ops,
                                        quantisMajor, card, &quantis_fops);
  if (random_handle == NULL)
    return -1;
  urandom_handle = quantis_devfs_replace("urandom", 1, 9, &urandom_prev_ops,
                                         quantisMajor, card, &quantis_fops);
  if (urandom_handle == NULL) {
    quantis_restore_random();
    return -1;
  }

  return 0;
}
#endif

static int __devinit quantis_probe(struct pci_dev *pdev,
                                   const struct pci_device_id *ent) {
  int status;
  quantis_card_t *device;

#ifdef USE_DEVFS
  char devname[4];
#endif

  down(&quantisMutex);
  if (quantisCardCount >= QUANTIS_MAX_CARDS) {
    QUANTIS_ERROR("Not supporting more than %d cards", QUANTIS_MAX_CARDS);
    up(&quantisMutex);
    return -ENXIO;
  }
  device = &quantisCards[quantisCardCount];
  device->bus = pdev;
#ifdef USE_DEVFS
  device->handle = NULL;
#endif
  quantisCardCount++;
  up(&quantisMutex);
  QUANTIS_DEBUG("now %d card(s) managed by this driver", quantisCardCount);

  if ((status = pci_enable_device(pdev)) != 0) {
    QUANTIS_ERROR("can't enable pci device");
    return status;
  }

  if ((status = pci_request_regions(pdev, DRIVER_SHORT_NAME)) != 0) {
    QUANTIS_ERROR("can't enable pci device");
    return status;
  }

  down(&quantisMutex);
  device->regs =
    (u_int32_t *)ioremap_nocache(pci_resource_start(pdev, 1),
                                 QUANTIS_REG_LENGTH);
  sema_init(&device->mutex, 1);
  up(&quantisMutex);

  down(&device->mutex);
  quantis_rng_reset(device);
  up(&device->mutex);

#if defined(USE_DEVFS)
  sprintf(devname, "%d", quantisCardCount-1);
  QUANTIS_DEBUG("trying to register in devfs /dev/%s/%s", DRIVER_SHORT_NAME, devname);
  device->handle = devfs_register(dir_handle, devname, DEVFS_FL_DEFAULT,
                                  quantisMajor, quantisCardCount-1,
                                  S_IFCHR | S_IRUGO | S_IWUGO,
                                  &quantis_fops, 0);
  QUANTIS_DEBUG("devfs returned %p", device->handle);

#if defined(REPLACE_DEV_RANDOM)
  if (dir_handle && (quantisReplaceRandom == quantisCardCount - 1))
    quantis_replace_random(quantisCardCount - 1);
#endif
#endif /* USE_DEVFS */

  QUANTIS_INFO("found card %d, with serial 0x%08x",
               quantisCardCount-1, quantis_rng_version(device));

  return status;
}

static void __devexit quantis_remove_one(struct pci_dev *pdev) {
  int i;

  iounmap(quantisCards[quantisCardCount].regs);
  pci_release_regions(pdev);
  pci_disable_device(pdev);

  down(&quantisMutex);

  for(i = 0; i < QUANTIS_MAX_CARDS; i++) {
    if(quantisCards[i].bus == pdev) {
      quantisCards[i].bus = NULL;

#ifdef USE_DEVFS
#if defined(REPLACE_DEV_RANDOM)
      if (i == quantisReplaceRandom) {
          quantis_restore_random();
      }
#endif
      QUANTIS_DEBUG("unregister device");
      if (quantisCards[i].handle)
          devfs_unregister(quantisCards[i].handle);
      QUANTIS_DEBUG("unregistered device");
#endif
    }
  }
  quantisCardCount--;

  up(&quantisMutex);
  QUANTIS_DEBUG("now %d card(s) managed by this driver", quantisCardCount);
}

static int quantis_proc_read(char *buf, char **start,
                             off_t offset, int count, int *eof, void *data) {
  int len = 0;
  int card_count;
  int i;

  down(&quantisMutex);
  card_count = quantisCardCount;
  up(&quantisMutex);

  len += snprintf(buf + len, count - len, DRIVER_NAME"\n");

#ifdef DEBUG
  len += snprintf(buf + len, count - len, "*** debug build ***\n");
#endif

  len += snprintf(buf + len, count - len,
                  "version %d.%d with support for %d cards\n",
                  QUANTIS_DRIVER_VERSION / 10, QUANTIS_DRIVER_VERSION % 10,
                  QUANTIS_MAX_CARDS);
  len += snprintf(buf + len, count - len,
                  "driver '%s', device major %d\n",
                  DRIVER_SHORT_NAME, quantisMajor);
  len += snprintf(buf + len, count - len, "found %d card(s)\n", card_count);

  for(i = 0; i < card_count; i++) {
      len += snprintf(buf + len, count - len, "card %d version:0x%08x\n", i,
                      quantis_rng_version(&quantisCards[i]));
  }

  *eof = 1;
  return len;
}

static int __init quantis_proc_register (void) {
  struct proc_dir_entry *proc_entry;
  proc_entry = create_proc_read_entry(DRIVER_SHORT_NAME, 0, 0,
                                      quantis_proc_read, 0);
  if (proc_entry)
    return 0;
  else
    return -EBUSY;
}

static struct pci_driver quantis_pci_driver = {
  .name      = DRIVER_NAME,
  .id_table  = pci_ids,
  .probe     = quantis_probe,
  .remove    = __devexit_p(quantis_remove_one),
#ifdef CONFIG_PM
  .suspend   = quantis_suspend,
  .resume    = quantis_resume,
#endif
};

typedef enum  {
  STATE_NOTHING,
  STATE_ALLOC,
  STATE_DEVFS,
  STATE_REGISTER,
  STATE_PCI,
  STATE_PROC,
  STATE_OK
} init_state_t;

static void quantis_exit_module_state(init_state_t state) {
  switch(state) {
  case STATE_OK:
    remove_proc_entry(DRIVER_SHORT_NAME, 0);

  case STATE_PROC:
    pci_unregister_driver(&quantis_pci_driver);

  case STATE_PCI:
      unregister_chrdev(quantisMajor, DRIVER_NAME);

  case STATE_REGISTER:
#ifdef USE_DEVFS
    QUANTIS_DEBUG("unregister devfs dir");
    if (dir_handle)
      devfs_unregister(dir_handle);
#ifdef REPLACE_DEV_RANDOM
    quantis_restore_random();
#endif
#endif

  case STATE_DEVFS:
  case STATE_NOTHING:
    break;

  default:
    QUANTIS_ERROR("exit module with unknown state (%d)",state);
    break;
  }
}

static void __exit quantis_exit_module(void) {
  quantis_exit_module_state(STATE_OK);
}

static int __init quantis_init_module (void) {
  int status;

  init_state_t state=STATE_NOTHING;

  QUANTIS_INFO("%s version %d.%d with support for %d cards",
               DRIVER_NAME, QUANTIS_DRIVER_VERSION / 10,
               QUANTIS_DRIVER_VERSION % 10,
               QUANTIS_MAX_CARDS);

  sema_init(&quantisMutex,1);

#ifdef USE_DEVFS
  state = STATE_DEVFS;
  if ((dir_handle = devfs_mk_dir(NULL, DRIVER_SHORT_NAME, NULL)) == NULL) {
    QUANTIS_ERROR("can't create /dev/%s/",DRIVER_SHORT_NAME);
    quantis_exit_module_state(state);
    return -EBUSY;
  }
#endif

  state = STATE_REGISTER;
  status = register_chrdev(quantisMajor, DRIVER_NAME, &quantis_fops);
  if (status >= 0) {
    if (quantisMajor == 0) {
        QUANTIS_INFO("assigned major number %d", status);
        quantisMajor = status;
    }
  } else {
    QUANTIS_ERROR("Can't register driver");
    quantis_exit_module_state(state);
    return status;
  }

  state = STATE_PCI;
#ifdef LINUX_2_6__
  if ((status = pci_register_driver(&quantis_pci_driver)) < 0) {
#else
  if ((status = pci_module_init(&quantis_pci_driver)) < 0) {
#endif
    QUANTIS_ERROR("PCI init module failed");
    quantis_exit_module_state(state);
    return status;
  }

  state = STATE_PROC;
  if((status = quantis_proc_register()) != 0) {
    QUANTIS_ERROR("PCI init module failed");
    quantis_exit_module_state(state);
    return status;
  }

  QUANTIS_INFO("Successful load, %d card(s) found", quantisCardCount);
  state = STATE_OK;
  return 0;
}

module_init(quantis_init_module);
module_exit(quantis_exit_module);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("id Quantique");
MODULE_DESCRIPTION(DRIVER_NAME);

#ifndef LINUX_2_6__
EXPORT_NO_SYMBOLS;
#endif

