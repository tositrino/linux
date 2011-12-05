/*
 *  linux/init/version.c
 *
 *  Copyright (C) 1992  Theodore Ts'o
 *
 *  May be freely distributed as part of Linux.
 *
 *  THS changes
 *  -----------
 *
 *  - 11/11/08 kernel-3.1.0
 *
 *  - colored banner defines
 *    __CBANNER_DEFAULT__ : define default banner id
 *    __CBANNER_LOGO__    : define text logo color to use 
 *    __CBANNER_TEXT__    : define logo color to use 
 *    __CBANNER_DELAY__   : define wait time 
 *  - added extern data
 *    extern char  cbanner_wait;
 *    extern char  cbanner_mode;
 *    extern char *cbanner_list[];
 *  - added functions and setup hook :
 *    static int __init cbanner_setup(char *options)
 *    function cbanner_display(void)
 *    __setup("cbanner=", cbanner_setup);
 *
 *     0x05 - magenta
 *     0x07 - standard
 *     0x09 - lightblue
 *     0x0b - lightcyan
 * 
 */
 
#define THS_RELEASE          "mmxi-xi-viii"
#define __CBANNER_DEFAULT__  0x00
#define __CBANNER_LOGO__     0x08
#define __CBANNER_TEXT__     0x09
#define __CBANNER_DELAY__    0x200

#include <generated/compile.h>
#include <linux/module.h>
#include <linux/uts.h>
#include <linux/utsname.h>
#include <generated/utsrelease.h>
#include <linux/version.h>

#ifndef CONFIG_KALLSYMS
#define version(a) Version_ ## a
#define version_string(a) version(a)

extern int version_string(LINUX_VERSION_CODE);
int version_string(LINUX_VERSION_CODE);
#endif

struct uts_namespace init_uts_ns = {
	.kref = {
		.refcount	= ATOMIC_INIT(2),
	},
	.name = {
		.sysname	= UTS_SYSNAME,
		.nodename	= UTS_NODENAME,
		.release	= UTS_RELEASE,
		.version	= UTS_VERSION,
		.machine	= UTS_MACHINE,
		.domainname	= UTS_DOMAINNAME,
	},
	.user_ns = &init_user_ns,
};
EXPORT_SYMBOL_GPL(init_uts_ns);

/* FIXED STRINGS! Don't touch! */
const char linux_banner[] =
	"Linux version " UTS_RELEASE " (" LINUX_COMPILE_BY "@"
	LINUX_COMPILE_HOST ") (" LINUX_COMPILER ") " UTS_VERSION "\n";

const char linux_proc_banner[] =
	"%s version %s"
	" (" LINUX_COMPILE_BY "@" LINUX_COMPILE_HOST ")"
	" (" LINUX_COMPILER ") %s\n";

unsigned char  cbanner_wait   = 0;
unsigned char  cbanner_mode   = 1;
unsigned short cbanner_bcolor = __CBANNER_LOGO__;
unsigned short cbanner_tcolor = __CBANNER_TEXT__;
unsigned long  cbanner_delay  = __CBANNER_DELAY__;
unsigned short cbanner_count  = 3;
unsigned short cbanner_id     = __CBANNER_DEFAULT__;

const char *cbanner_data[]  = {

                               //12345678901234567890123456789012345678901234567890123456789012345678901234567890
                                "88888 8   8 888888                              eeea    88      eeeea    \n"
                                "  8   8   8 8        e   e  eeee eeeee  eeeee      8     8      8   8    \n"
                                "  8e  8eee8 8eeeee   8   8  8    8   8  8   8      8     8      8   8    \n"
                                "  88  88  8     88   8eee8e 8eee 8eee8e 8e  8    e88     8      8  88    \n"
                                "  88  88  8 e   88   88   8 88   88   8 88  8     88    8888    8  88    \n"
                                "  88  88  8 8eee88   88   8 88ee 88   8 88  8   8e88 88 8888 88 88888    \n"
                                "ths kernel release " THS_RELEASE " / linux-" UTS_RELEASE "\n",

                                "e   e                                 eeea    88      eeeea  o8o  db  o8o \n"
                                "8   8   eeee eeeee  eeeee eeee e         8     8      8   8  o8888db8888o \n"
                                "8eee8e  8    8   8  8   8 8    8         8     8      8   8      odbo     \n"
                                "88   8  8eee 8eee8e 8e  8 8eee 8e      e88     8      8  88  o88o db o88o \n"
                                "88   8  88   88   8 88  8 88   88       88    8888    8  88  oo   db   oo \n"
                                "88   8  88ee 88   8 88  8 88ee 88eee  8e88 88 8888 88 88888       db      \n",
                                "ths kernel release " THS_RELEASE " / linux-" UTS_RELEASE "\n",

                                "/-----------------------------------------\\\n"
                                "|                  _/\\_                   |\n"
                                "|  moc            <o88o>             con  |\n"
                                "| oooooon          <88>        .moooooooo |\n"
                                "| oocoooooooon.    <88>    .moooooooooooo |\n"
                                "|  ooooooooooooon._d88b_.moooooooooooooo  |\n"
                                "|   (oooooooooooooo8888ooooooooooooooo)   |\n"
                                "|     (ooooooooooooo88ooooooooooooo)      |\n"
                                "|        oooooooooo8888oooooooooo         |\n"
                                "|               -ooo88ooo-                |\n"
                                "|          .mooooooY88Yoooooon.           |\n"
                                "|     __cooooooooo/ 88 \\oooooooooa,,      |\n"
                                "|   _ooooooooooooo: oo :8oooooooooooa,    |\n"
                                "|  /ooooooooooooo/  oo  \\oooooooooooo8\\   |\n"
                                "|  oooooooooooo     oo     oooooooooooo   |\n"
                                "|  \\oooooooo/       oo       \\oooooooo/   |\n"
                                "|                   oo                    |\n"
                                "|                   oo                    |\n"
                                "|                   oo                    |\n"
                                "|                   oo                    |\n"
                                "|                   oo                    |\n"
                                "\\-----------------------------------------/\n",
                                "odonata kernel release " THS_RELEASE " / linux-" UTS_RELEASE "\n" 

                             };






