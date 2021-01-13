// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2011 - 2012 Samsung Electronics
 * EXT4 filesystem implementation in Uboot by
 * Uma Shankar <uma.shankar@samsung.com>
 * Manjunatha C Achar <a.manjunatha@samsung.com>
 *
 * Ext4fs support
 * made from existing cmd_ext2.c file of Uboot
 *
 * (C) Copyright 2004
 * esd gmbh <www.esd-electronics.com>
 * Reinhard Arlt <reinhard.arlt@esd-electronics.com>
 *
 * made from cmd_reiserfs by
 *
 * (C) Copyright 2003 - 2004
 * Sysgo Real-Time Solutions, AG <www.elinos.com>
 * Pavel Bartusek <pba@sysgo.com>
 */

/*
 * Changelog:
 *	0.1 - Newly created file for ext4fs support. Taken from cmd_ext2.c
 *	        file in uboot. Added ext4fs ls load and write support.
 */

#include <common.h>
#include <part.h>
#include <config.h>
#include <command.h>
#include <image.h>
#include <linux/ctype.h>
#include <asm/byteorder.h>
#include <ext4fs.h>
#include <linux/stat.h>
#include <malloc.h>
#include <fs.h>

#if defined(CONFIG_CMD_USB) && defined(CONFIG_USB_STORAGE)
#include <usb.h>
#endif

enum _FS_TYPE{
	FS_TYPE_EXT2,
	FS_TYPE_EXT3,
	FS_TYPE_EXT4
};

int do_ext4_size(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	return do_size(cmdtp, flag, argc, argv, FS_TYPE_EXT);
}

int do_ext4_load(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	return do_load(cmdtp, flag, argc, argv, FS_TYPE_EXT);
}

int do_ext4_ls(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	return do_ls(cmdtp, flag, argc, argv, FS_TYPE_EXT);
}

#if defined(CONFIG_CMD_EXT4_WRITE)
int do_ext4_write(struct cmd_tbl *cmdtp, int flag, int argc,
		  char *const argv[])
{
	return do_save(cmdtp, flag, argc, argv, FS_TYPE_EXT);
}

U_BOOT_CMD(ext4write, 7, 1, do_ext4_write,
	   "create a file in the root directory",
	   "<interface> <dev[:part]> <addr> <absolute filename path>\n"
	   "    [sizebytes] [file offset]\n"
	   "    - create a file in / directory");

#endif

U_BOOT_CMD(
	ext4size,	4,	0,	do_ext4_size,
	"determine a file's size",
	"<interface> <dev[:part]> <filename>\n"
	"    - Find file 'filename' from 'dev' on 'interface'\n"
	"      and determine its size."
);

U_BOOT_CMD(ext4ls, 4, 1, do_ext4_ls,
	   "list files in a directory (default /)",
	   "<interface> <dev[:part]> [directory]\n"
	   "    - list files from 'dev' on 'interface' in a 'directory'");

U_BOOT_CMD(ext4load, 7, 0, do_ext4_load,
	   "load binary file from a Ext4 filesystem",
	   "<interface> [<dev[:part]> [addr [filename [bytes [pos]]]]]\n"
	   "    - load binary file 'filename' from 'dev' on 'interface'\n"
	   "      to address 'addr' from ext4 filesystem");

int ext_format (int argc, char *const argv[], char filesystem_type)
{
	int dev = 0;
	int part = 1;
	struct blk_desc *dev_desc = NULL;
	struct disk_partition info;

	if(argc < 2) {
		printf("usage : ext2format <interface> <dev[:part]>\n");
		return (0);
	}
	part = blk_get_device_part_str(argv[1], argv[2], &dev_desc, &info, 1);
	if (part < 0)
		return 1;
	dev = dev_desc->devnum;
	if (dev_desc == NULL) {
		puts("\n** Invalid boot device **\n");
		return 1;
	}

	printf("Start format MMC%d partition%d ....\n", dev, part);

	switch (filesystem_type) {
	case FS_TYPE_EXT4:
	case FS_TYPE_EXT3:
	case FS_TYPE_EXT2:
		if (ext4fs_format(dev_desc, part, filesystem_type,info) != 0)
			printf("Format failure!!!\n");
		break;

	default:
		printf("FileSystem Type Value is not invalidate=%d \n", filesystem_type);
		break;
	}
	return 0;
}

int do_ext2_format(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	return ext_format(argc, argv, FS_TYPE_EXT2);
}

U_BOOT_CMD(
	ext2format, 3, 0, do_ext2_format,
	"ext2format - disk format by ext2",
	"<interface (only support mmc)> <dev:partition num>\n"
	"	- format by ext2 on 'interface'"
);

int do_ext3_format(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	return ext_format(argc, argv, FS_TYPE_EXT3);
}

U_BOOT_CMD(
	ext3format, 3, 0, do_ext3_format,
	"ext3format - disk format by ext3",
	"<interface (only support mmc)> <dev:partition num>\n"
	"	- format by ext3 on 'interface'"
);

int do_ext4_format(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	return ext_format(argc, argv, FS_TYPE_EXT4);
}

U_BOOT_CMD(
	ext4format, 3, 0, do_ext4_format,
	"ext4format - disk format by ext4",
	"<interface (only support mmc)> <dev:partition num>\n"
	"	- format by ext4 on 'interface'"
);
