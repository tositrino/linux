/*
  hdrv.c
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <asm/uaccess.h>

#define  HDRV_NAME           "hdrv"
#define  HDRV_CLASS_NAME     "hdrvclass"

static char  *hdrv_name    = CONFIG_HDRV_DEV_NAME ;
static char  *hdrv_message = CONFIG_HDRV_DEV_MESSAGE ;

static dev_t  hdrv_dev_number;
static struct cdev *driver_object;
static struct class *hdrv_dev_class;
static struct device *hdrv_dev;

static int driver_open( struct inode *geraete_datei, struct file *instanz )
{
	dev_info( hdrv_dev, HDRV_NAME ": driver_open called\n" );
	return 0;
}

static int driver_close( struct inode *geraete_datei, struct file *instanz )
{
	dev_info( hdrv_dev, HDRV_NAME": driver_close called\n" );
	return 0;
}

static ssize_t driver_read( struct file *instanz, char __user *user,
		size_t count, loff_t *offset )
{
	unsigned long not_copied, to_copy;

	to_copy = min( count, strlen(hdrv_message)+1 );
	not_copied=copy_to_user(user,hdrv_message,to_copy);
	return to_copy-not_copied;
}

static struct file_operations fops = {
	.owner= THIS_MODULE,
	.read= driver_read,
	.open= driver_open, 
	.release= driver_close,
};

static int __init mod_init( void )
{
	if( alloc_chrdev_region(&hdrv_dev_number,0,1,HDRV_CLASS_NAME)<0 )
		return -EIO;
	driver_object = cdev_alloc(); /* Anmeldeobjekt reservieren */
	if( driver_object==NULL )
		goto free_device_number;
	driver_object->owner = THIS_MODULE;
	driver_object->ops = &fops;
	if( cdev_add(driver_object,hdrv_dev_number,1) )
		goto free_cdev;
	/* Eintrag im Sysfs, damit Udev den Geraetedateieintrag erzeugt. */
	hdrv_dev_class = class_create( THIS_MODULE, HDRV_CLASS_NAME );
	if( IS_ERR( hdrv_dev_class ) ) {
		pr_err( HDRV_NAME": no udev support\n");
		goto free_cdev;
	}
	hdrv_dev = device_create( hdrv_dev_class, NULL, hdrv_dev_number,
			NULL, "%s", hdrv_name );
	return 0;
free_cdev:
	kobject_put( &driver_object->kobj );
free_device_number:
	unregister_chrdev_region( hdrv_dev_number, 1 );
	return -EIO;
}

static void __exit mod_exit( void )
{
	/* Loeschen des Sysfs-Eintrags und damit der Geraetedatei */
	device_destroy( hdrv_dev_class, hdrv_dev_number );
	class_destroy( hdrv_dev_class );
	/* Abmelden des Treibers */
	cdev_del( driver_object );
	unregister_chrdev_region( hdrv_dev_number, 1 );
	return;
}

module_init( mod_init );
module_exit( mod_exit );

/* Metainformation */
MODULE_AUTHOR("ths");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("A virtual device, which returns a string.");
