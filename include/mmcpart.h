#ifndef _MMCPART_H_
#define _MMCPART_H_

#include <common.h>
#include <command.h>
#include <env.h>

#define UBOOT_BLK_START (0)
#define UBOOT_BLK_COUNT (0x45E)

#define ENV_BLK_START   (0x460)

#define BLK_SHIFT       (9)
#define BLK_SIZE        (1 << BLK_SHIFT)
#define KB_BLK(x)      ((x) * 1024 / BLK_SIZE)
#define MB_BLK(x)      (KB_BLK(x) * 1024)

enum _part_order {
	uboot = 0,
	env,
	kernel,
	dtb,
	mbr1,
	mbr2,
	mbr3,
	mbr4,
	mbr_end,
	PART_MAX = mbr_end,
};

#define PART_EXT4	(0x83)
#define PART_FAT	(0x0C)

#define PART_FLAG_BOOTABLE	(1 << 0)
#define PART_FLAG_EXT4		(1 << 1)
#define PART_FLAG_FAT		(1 << 2)
#define PART_FLAG_NONE		(0)

struct _part_info {
	const char * name;
	unsigned long blk_start;
	unsigned long blk_count;
	unsigned char part_id;
	unsigned char flag;
};

struct _input_argv {
	unsigned long addr;
	unsigned long blk_size;
	const struct _part_info *part;
};

#define DEF_PART(_idx, _name, _flag) \
	[_idx] = {\
		.name		= _name,\
		.flag		= (_flag),\
		.part_id	= 0xFF,\
	}

// defined in fdisk.c ==============================================
int get_mmc_part_info(char *device_name, int part_num,
	int *block_start, int *block_count, unsigned char *part_Id);
// =================================================================


// -----------------------------------------------------------------
/* copyright (c) 2010 Samsung Electronics Co., Ltd.
 *              http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

typedef struct _ext4_file_header {
	unsigned int magic;
	unsigned short major;
	unsigned short minor;
	unsigned short file_header_size;
	unsigned short chunk_header_size;
	unsigned int block_size;
	unsigned int total_blocks;
	unsigned int total_chunks;
	unsigned int crc32;
}ext4_file_header;

typedef struct _ext4_chunk_header {
	unsigned short type;
	unsigned short reserved;
	unsigned int chunk_size;
	unsigned int total_size;
}ext4_chunk_header;

#define EXT4_FILE_HEADER_MAGIC	0xED26FF3A
#define EXT4_FILE_HEADER_MAJOR	0x0001
#define EXT4_FILE_HEADER_MINOR	0x0000
#define EXT4_FILE_BLOCK_SIZE	0x1000

#define EXT4_FILE_HEADER_SIZE	(sizeof(struct _ext4_file_header))
#define EXT4_CHUNK_HEADER_SIZE	(sizeof(struct _ext4_chunk_header))

#define EXT4_CHUNK_TYPE_RAW			0xCAC1
#define EXT4_CHUNK_TYPE_FILL		0xCAC2
#define EXT4_CHUNK_TYPE_NONE		0xCAC3

int itop4412_partition_init(void);
// -----------------------------------------------------------------

#endif // _MMCPART_H_
