// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2011 - 2012 Samsung Electronics
 * EXT4 filesystem implementation in Uboot by
 * Uma Shankar <uma.shankar@samsung.com>
 * Manjunatha C Achar <a.manjunatha@samsung.com>
 *
 * ext4ls and ext4load : Based on ext2 ls and load support in Uboot.
 *		       Ext4 read optimization taken from Open-Moko
 *		       Qi bootloader
 *
 * (C) Copyright 2004
 * esd gmbh <www.esd-electronics.com>
 * Reinhard Arlt <reinhard.arlt@esd-electronics.com>
 *
 * based on code from grub2 fs/ext2.c and fs/fshelp.c by
 * GRUB  --  GRand Unified Bootloader
 * Copyright (C) 2003, 2004  Free Software Foundation, Inc.
 *
 * ext4write : Based on generic ext4 protocol.
 */

#include <common.h>
#include <blk.h>
#include <ext_common.h>
#include <ext4fs.h>
#include "ext4_common.h"
#include <div64.h>
#include <malloc.h>
#include <part.h>
#include <uuid.h>
#include <memalign.h>

int ext4fs_symlinknest;
struct ext_filesystem ext_fs;

struct ext_filesystem *get_fs(void)
{
	return &ext_fs;
}

void ext4fs_free_node(struct ext2fs_node *node, struct ext2fs_node *currroot)
{
	if ((node != &ext4fs_root->diropen) && (node != currroot))
		free(node);
}

/*
 * Taken from openmoko-kernel mailing list: By Andy green
 * Optimized read file API : collects and defers contiguous sector
 * reads into one potentially more efficient larger sequential read action
 */
int ext4fs_read_file(struct ext2fs_node *node, loff_t pos,
		loff_t len, char *buf, loff_t *actread)
{
	struct ext_filesystem *fs = get_fs();
	int i;
	lbaint_t blockcnt;
	int log2blksz = fs->dev_desc->log2blksz;
	int log2_fs_blocksize = LOG2_BLOCK_SIZE(node->data) - log2blksz;
	int blocksize = (1 << (log2_fs_blocksize + log2blksz));
	unsigned int filesize = le32_to_cpu(node->inode.size);
	lbaint_t previous_block_number = -1;
	lbaint_t delayed_start = 0;
	lbaint_t delayed_extent = 0;
	lbaint_t delayed_skipfirst = 0;
	lbaint_t delayed_next = 0;
	char *delayed_buf = NULL;
	char *start_buf = buf;
	short status;
	struct ext_block_cache cache;

	ext_cache_init(&cache);

	/* Adjust len so it we can't read past the end of the file. */
	if (len + pos > filesize)
		len = (filesize - pos);

	if (blocksize <= 0 || len <= 0) {
		ext_cache_fini(&cache);
		return -1;
	}

	blockcnt = lldiv(((len + pos) + blocksize - 1), blocksize);

	for (i = lldiv(pos, blocksize); i < blockcnt; i++) {
		long int blknr;
		int blockoff = pos - (blocksize * i);
		int blockend = blocksize;
		int skipfirst = 0;
		blknr = read_allocated_block(&node->inode, i, &cache);
		if (blknr < 0) {
			ext_cache_fini(&cache);
			return -1;
		}

		blknr = blknr << log2_fs_blocksize;

		/* Last block.  */
		if (i == blockcnt - 1) {
			blockend = (len + pos) - (blocksize * i);

			/* The last portion is exactly blocksize. */
			if (!blockend)
				blockend = blocksize;
		}

		/* First block. */
		if (i == lldiv(pos, blocksize)) {
			skipfirst = blockoff;
			blockend -= skipfirst;
		}
		if (blknr) {
			int status;

			if (previous_block_number != -1) {
				if (delayed_next == blknr) {
					delayed_extent += blockend;
					delayed_next += blockend >> log2blksz;
				} else {	/* spill */
					status = ext4fs_devread(delayed_start,
							delayed_skipfirst,
							delayed_extent,
							delayed_buf);
					if (status == 0) {
						ext_cache_fini(&cache);
						return -1;
					}
					previous_block_number = blknr;
					delayed_start = blknr;
					delayed_extent = blockend;
					delayed_skipfirst = skipfirst;
					delayed_buf = buf;
					delayed_next = blknr +
						(blockend >> log2blksz);
				}
			} else {
				previous_block_number = blknr;
				delayed_start = blknr;
				delayed_extent = blockend;
				delayed_skipfirst = skipfirst;
				delayed_buf = buf;
				delayed_next = blknr +
					(blockend >> log2blksz);
			}
		} else {
			int n;
			int n_left;
			if (previous_block_number != -1) {
				/* spill */
				status = ext4fs_devread(delayed_start,
							delayed_skipfirst,
							delayed_extent,
							delayed_buf);
				if (status == 0) {
					ext_cache_fini(&cache);
					return -1;
				}
				previous_block_number = -1;
			}
			/* Zero no more than `len' bytes. */
			n = blocksize - skipfirst;
			n_left = len - ( buf - start_buf );
			if (n > n_left)
				n = n_left;
			memset(buf, 0, n);
		}
		buf += blocksize - skipfirst;
	}
	if (previous_block_number != -1) {
		/* spill */
		status = ext4fs_devread(delayed_start,
					delayed_skipfirst, delayed_extent,
					delayed_buf);
		if (status == 0) {
			ext_cache_fini(&cache);
			return -1;
		}
		previous_block_number = -1;
	}

	*actread  = len;
	ext_cache_fini(&cache);
	return 0;
}

int ext4fs_ls(const char *dirname)
{
	struct ext2fs_node *dirnode = NULL;
	int status;

	if (dirname == NULL)
		return 0;

	status = ext4fs_find_file(dirname, &ext4fs_root->diropen, &dirnode,
				  FILETYPE_DIRECTORY);
	if (status != 1) {
		printf("** Can not find directory. **\n");
		if (dirnode)
			ext4fs_free_node(dirnode, &ext4fs_root->diropen);
		return 1;
	}

	ext4fs_iterate_dir(dirnode, NULL, NULL, NULL);
	ext4fs_free_node(dirnode, &ext4fs_root->diropen);

	return 0;
}

int ext4fs_exists(const char *filename)
{
	loff_t file_len;
	int ret;

	ret = ext4fs_open(filename, &file_len);
	return ret == 0;
}

int ext4fs_size(const char *filename, loff_t *size)
{
	return ext4fs_open(filename, size);
}

int ext4fs_read(char *buf, loff_t offset, loff_t len, loff_t *actread)
{
	if (ext4fs_root == NULL || ext4fs_file == NULL)
		return -1;

	return ext4fs_read_file(ext4fs_file, offset, len, buf, actread);
}

int ext4fs_probe(struct blk_desc *fs_dev_desc,
		 struct disk_partition *fs_partition)
{
	ext4fs_set_blk_dev(fs_dev_desc, fs_partition);

	if (!ext4fs_mount(fs_partition->size)) {
		ext4fs_close();
		return -1;
	}

	return 0;
}

int ext4_read_file(const char *filename, void *buf, loff_t offset, loff_t len,
		   loff_t *len_read)
{
	loff_t file_len;
	int ret;

	ret = ext4fs_open(filename, &file_len);
	if (ret < 0) {
		printf("** File not found %s **\n", filename);
		return -1;
	}

	if (len == 0)
		len = file_len;

	return ext4fs_read(buf, offset, len, len_read);
}

int ext4fs_uuid(char *uuid_str)
{
	if (ext4fs_root == NULL)
		return -1;

#ifdef CONFIG_LIB_UUID
	uuid_bin_to_str((unsigned char *)ext4fs_root->sblock.unique_id,
			uuid_str, UUID_STR_FORMAT_STD);

	return 0;
#else
	return -ENOSYS;
#endif
}

void ext_cache_init(struct ext_block_cache *cache)
{
	memset(cache, 0, sizeof(*cache));
}

void ext_cache_fini(struct ext_block_cache *cache)
{
	free(cache->buf);
	ext_cache_init(cache);
}

int ext_cache_read(struct ext_block_cache *cache, lbaint_t block, int size)
{
	/* This could be more lenient, but this is simple and enough for now */
	if (cache->buf && cache->block == block && cache->size == size)
		return 1;
	ext_cache_fini(cache);
	cache->buf = memalign(ARCH_DMA_MINALIGN, size);
	if (!cache->buf)
		return 0;
	if (!ext4fs_devread(block, 0, size, cache->buf)) {
		ext_cache_fini(cache);
		return 0;
	}
	cache->block = block;
	cache->size = size;
	return 1;
}


#define SEC_PER_BLOCK	8

unsigned int get_random_val(unsigned int next)
{
	next = next * 1103515245 + 12345;
	return((unsigned)(next/65536) % 65536);
}

unsigned int default_journal_size(uint32_t block_cnt)
{
	if (block_cnt < 2048)
		return -1;
	if (block_cnt < 32768)
		return (1024);
	if (block_cnt < 256 * 1024)
		return (4096);
	if (block_cnt < 512 * 1024)
		return (8192);
	if (block_cnt < 1024 * 1024)
		return (16384);
	return (32768);
}

int ext4fs_format(struct blk_desc *dev_desc, int part_no, char set_journaling,struct disk_partition info)
{

	unsigned char buffer[SECTOR_SIZE];

	/* check if we have a MBR (on floppies we have only a PBR) */
	if (blk_dread(dev_desc, 0, 1, (ulong *) buffer) != 1) {
		printf ("** Can't read from device %d **\n",(unsigned int)dev_desc);
		return -1;
	}
	if (buffer[0x1fe] != 0x55 ||
		buffer[0x1fe + 1] != 0xaa) {
		printf("** MBR is broken **\n");
		/* no signature found */
		return -1;
	}

	printf("Partition%d: Start Address(0x%x), Size(0x%x)\n", part_no, (unsigned int)info.start, (unsigned int)info.size);
	/* Write Super Block */
	int blk_group_num = info.size / 0x40000;
	if (info.size % 0x40000)
		blk_group_num += 1;

	int bgcnt;
	uint32_t start_block = 0;
	uint8_t *img; /* Super Block Image */
	uint32_t uuid[4];
	img = malloc_cache_aligned(sizeof(__u8) * 4096);
	if (img == NULL) {
		printf("Can't make img buffer~~!!\n");
		return -1;
	}
	uint8_t *img2; /* Group Descriptor Table */
	img2 = malloc_cache_aligned(sizeof(__u8) * 4096);
	if (img2 == NULL) {
		printf("Can't make img2 buffer~~!!\n");
		return -1;
	}
	uint8_t *reserve_img;
	reserve_img = malloc_cache_aligned(sizeof(__u8) * 4096);
	if (reserve_img == NULL) {
		printf("Can't make reserve_img buffer~~!!\n");
		return -1;
	}

	uint8_t *img3; /* Block Bitmap */
	img3 = malloc_cache_aligned(sizeof(__u8) * 4096);
	if (img3 == NULL) {
		printf("Can't make img3 buffer~~!!\n");
		return -1;
	}
	uint8_t *img4;
	img4 = malloc_cache_aligned(sizeof(__u8) * 4096);
	if (img4 == NULL) {
		printf("Can't make img4 buffer~~!!\n");
		return -1;
	}
	uint8_t *zerobuf = malloc_cache_aligned(sizeof(__u8) * 4096 * 80);
	if (zerobuf == NULL) {
		printf("Can't make zero buffer~~!!\n");
		return -1;
	}

	uint8_t *img5;
	img5 = malloc_cache_aligned(sizeof(__u8) * 2048);
	if (img5 == NULL) {
		printf("Can't make img5 buffer~~!!\n");
		return -1;
	}
	uint8_t *rootdata = malloc_cache_aligned(sizeof(__u8) * 4096);
	if (rootdata == NULL) {
		printf("Can't make rootdata buffer~~!!\n");
		return -1;
	}
	uint8_t *inode_data = malloc_cache_aligned(sizeof(__u8) * 4096);
	if (inode_data == NULL) {
		printf("Can't make inodedata buffer~~!!\n");
		return -1;
	}

	uint8_t *inode_data2 = malloc_cache_aligned(sizeof(__u8) * 4096);
	if (inode_data2 == NULL) {
		printf("Can't make inodedata2 buffer~~!!\n");
		return -1;
	}

	printf("Start ext2format...\n");
	for (bgcnt = 0; bgcnt < blk_group_num; bgcnt++) {
		printf("Wirte %d/%dblock-group\n",bgcnt,  blk_group_num);
		int i = 0;
		int rsv_blk_cnt = 0;
		struct super_block *sblock;
		memset(img, 0x0, sizeof(__u8) * 4096);
		if (bgcnt == 0)
			sblock = (struct super_block *)(img + 1024);
		else
			sblock = (struct super_block *)img;

		uint32_t total_blocks = info.size / SEC_PER_BLOCK; /* 4KB block */
		mk4(sblock->total_blocks, total_blocks); /* block count */
		uint32_t total_inodes = (total_blocks / 0x20);
		if (total_blocks % 0x20) {
			total_inodes += 1;
			/* Fix Me... it's just temp code */
			/* Check... 'total_blocks % 0x20'*/
			if (total_blocks > 0x8000 && total_blocks < 0x10000) {
				total_inodes += 1;
			}
		}
		total_inodes *= 0x20;

		if (total_blocks > 0x3E800) /* if size > 1GB */
			total_inodes /= 2;

		uint32_t inodes_per_group = ((total_inodes -1) / blk_group_num) + 1;
		inodes_per_group = (inodes_per_group / 0x20) + 1;
		inodes_per_group *= 0x20;

		if (total_blocks > 0x8000)
			mk4(sblock->inodes_per_group, inodes_per_group); /* inode for group */
		else
			mk4(sblock->inodes_per_group, total_inodes); /* inode for group */

		uint32_t reserved_blocks = total_blocks * 5 / 100;
		mk4(sblock->reserved_blocks, reserved_blocks); /* reserved block count */
		mk4(sblock->first_data_block, 0); /* frist data block */
		mk4(sblock->log2_block_size, 2); /* log block size */
		mk4(sblock->log2_fragment_size, 2); /* log fragmentation size */
		/* 8(1bit) * 4096(1block:4K) */
		mk4(sblock->blocks_per_group, 0x8000); /* block per group */
		uint32_t blocks_per_group = 0x8000;
		mk4(sblock->fragments_per_group, 0x8000); /* fragmentation per group */
		mk4(sblock->mtime, 0); /* mtime */
		mk4(sblock->wtime, 0x33); /* wtime : 0x33 means nothing*/
		mk2(sblock->mnt_count, 0); /* mount count*/
		mk2(sblock->max_mnt_count, 0x20); /* max mount count */
		mk2(sblock->magic, EXT2_MAGIC); /* magic signature */
		mk2(sblock->fs_state, 1); /* state (1:valid, 2:error) */
		mk2(sblock->error_handling, 1); /* errors */
		mk2(sblock->minor_revision_level, 0); /* minor version */
		mk4(sblock->lastcheck, 0); /* last check time */
		mk4(sblock->checkinterval, 0xed4e00); /* check interval */
		mk4(sblock->creator_os, 0); /* creator os */
		mk4(sblock->revision_level, 1); /* major version */
		mk2(sblock->uid_reserved, 0); /* UID that can use reserved blocks */
		mk2(sblock->gid_reserved, 0); /* GID that can use reserved blocks */
		mk4(sblock->first_inode, 0xb); /* first non-reserved inode */
		mk2(sblock->inode_size, 0x80); /* inode structure size */
		mk2(sblock->block_group_number, 0); /* block group num */
		uint32_t feature_compatibility = 0x30; /* default */
		if (set_journaling) {
			feature_compatibility += 0x4; /* 0x4 means HAS_JOURNAL */
		}
		mk4(sblock->feature_compatibility, feature_compatibility); /* compatible feature */
		mk4(sblock->feature_incompat, 0x2); /* incompatible feature */
		mk4(sblock->feature_ro_compat, 0x3); /* read-only feature */
		if (bgcnt == 0) {
			/* Set Unique ID */
			uuid[0] = get_random_val(get_timer(0));
			uuid[0] |= (get_random_val(get_timer(0)) << 16);
			uuid[1] = get_random_val(get_timer(0));
			uuid[1] |= (get_random_val(get_timer(0)) << 16);
			uuid[2] = get_random_val(get_timer(0));
			uuid[2] |= (get_random_val(get_timer(0)) << 16);
			uuid[3] = get_random_val(get_timer(0));
			uuid[3] |= (get_random_val(get_timer(0)) << 16);
		}
		mk4(sblock->unique_id, uuid[0]); /* UUID filesystem ID */
		mk4(sblock->unique_id + 4, uuid[1]); /* UUID filesystem ID */
		mk4(sblock->unique_id + 8, uuid[2]); /* UUID filesystem ID */
		mk4(sblock->unique_id + 12, uuid[3]); /* UUID filesystem ID */
		mk4(sblock->compression_info, 0); /* algorithm usage bitmap */

		uint32_t reserved_gdt_blk = total_blocks >> 12;
		mk2(sblock->reserved_gtd_blk, reserved_gdt_blk); /* reserved gtd blocks */
		if (set_journaling) {
			/* 8th inode is reserved for journaling */
			mk4(sblock->journal_inode_num, 8);
			sblock->journal_backup_type = 1;
		}
		mk2(sblock->hash_seed, get_random_val(get_timer(0))); /* Hash Seed */
		mk2(sblock->hash_seed + 2, get_random_val(get_timer(0)));
		mk2(sblock->hash_seed + 4, get_random_val(get_timer(0)));
		mk2(sblock->hash_seed + 6, get_random_val(get_timer(0)));
		mk2(sblock->hash_seed + 8, get_random_val(get_timer(0)));
		mk2(sblock->hash_seed + 10, get_random_val(get_timer(0)));
		mk2(sblock->hash_seed + 12, get_random_val(get_timer(0)));
		mk2(sblock->hash_seed + 14, get_random_val(get_timer(0)));
		sblock->hash_version = 0x2;
		mk4(sblock->mkfs_time, 0x33); /* 0x33 means nothing */

		/* Write Group Descriptor Table */
		struct group_desc_table *desc_table;
		memset(img2, 0x0, sizeof(__u8) * 4096);

		int offset = 0;
		uint32_t free_blk_cnt = 0;
		uint32_t free_inode_cnt = 0;
		uint32_t free_blocks = 0;
		uint32_t free_inodes = 0;
		uint32_t start_blkbit_addr = 0;
		uint32_t start_indbit_addr = 0;
		uint32_t start_indtable_addr = 0;

		uint32_t current_free_blk = 0;
		uint32_t reserved_blk_cnt = 0;
		uint32_t first_inode_table_addr = 0;
		uint32_t current_blkbit_addr = 0;
		uint32_t current_indbit_addr = 0;
		uint32_t current_indtable_addr= 0;

		char have_rev_gdt = 0;
		int remain_blocks = 0;
		int jnl_blocks = 0;
		int32_t jnldata_start_addr = 0;
		/* second group inode table offset */
		int32_t seblk_offset = 0;
		/* Starting block address of block bitmap */
		for (i = 0; i < blk_group_num; i++) {
			char is_revgdt = 0;
			/* 0 group description table need to change for default
			 * directory.
			 */
			start_blkbit_addr = i * blocks_per_group;
			start_indbit_addr = 1 + i * blocks_per_group;
			start_indtable_addr = 2 + i * blocks_per_group;
			desc_table = (struct group_desc_table *)(img2 + offset);

			/* last block is not full size */
			if (i == blk_group_num - 1) {
				//uint32_t remain_blocks = total_blocks -
				//			(blocks_per_group * i);
				uint32_t remain_blocks = total_blocks % 0x8000;
				free_blk_cnt = remain_blocks - 4 -
					(inodes_per_group * 128 / 4096) + 2;
				free_inode_cnt = inodes_per_group;
			} else {
				/*
				 * 0x8000 : total blocks per group
				 * 4 : super block(1), group descriptor table(1),
				 *     block bitmap(1), inode bitmap(1)
				 * (inodes_per_group * 128 / 4096) : inode table
				 * 2 : ?? (block start from 0??)
				 *     reserved block include superblock&gdt
				 */
				free_blk_cnt = 0x8000 - 4 - (inodes_per_group * 128 / 4096) + 2;
				free_inode_cnt = inodes_per_group;
			}

			if (i == 0) {
				is_revgdt = 1;
				reserved_blk_cnt++;
				/* For default creation */
				free_blk_cnt -= 6;
				if (total_blocks > 0x8000) /* if size > 128MB */
					free_inode_cnt -= 0xb;
				else
					free_inode_cnt -= 0x2b;

				start_blkbit_addr += reserved_gdt_blk + 2;
				start_indbit_addr += reserved_gdt_blk + 2;
				start_indtable_addr += reserved_gdt_blk + 2;
				free_blk_cnt -= (reserved_gdt_blk + 2);

				first_inode_table_addr = start_indtable_addr;
			}

			/* delete journaling block from free blocks */
			if (set_journaling) {
				if (i == 0) {
					jnl_blocks = default_journal_size(total_blocks);
					/* 1024 means number of pointing by 1blcok (4096/4) */
					jnl_blocks += (jnl_blocks / 1024);
					if (jnl_blocks >= 4096) {
						/*
						 * 2 blocks are needed to use double
						 * indirect pointer
						 */
						jnl_blocks += 2;
					}
					printf("Reserved blocks for jounaling : %d\n", jnl_blocks);
					if (free_blk_cnt >= jnl_blocks) {
						free_blk_cnt -= jnl_blocks;
					}
					else {
						remain_blocks = jnl_blocks -
							free_blk_cnt;
						free_blk_cnt = 0;
					}
				} else {

					if (remain_blocks > 0) {
						free_blk_cnt -= remain_blocks;
						remain_blocks = 0;
					}
				}
			}

			/*
			 * Keep reserved region.???
			 * If card is higher than 16GB, below codes must be fixed.
			 * Please excute mke2fs on linux and check messages
			 * 'superblock backups stored on blocks:'.
			 */
			if (i < 10) {
				if (i % 2) {
					is_revgdt  = 1;
					reserved_blk_cnt++;
					start_blkbit_addr += reserved_gdt_blk + 2;
					start_indbit_addr += reserved_gdt_blk + 2;
					start_indtable_addr += reserved_gdt_blk + 2;
					free_blk_cnt -= (reserved_gdt_blk + 2);
					rsv_blk_cnt++;
				}
			} else if (i > 20 && i < 30) {
				if (i == 25 || i == 27) {
					is_revgdt = 1;
					reserved_blk_cnt++;
					start_blkbit_addr += reserved_gdt_blk + 2;
					start_indbit_addr += reserved_gdt_blk + 2;
					start_indtable_addr += reserved_gdt_blk + 2;
					free_blk_cnt -= (reserved_gdt_blk + 2);
					rsv_blk_cnt++;
				}
			} else if (i > 40 && i < 50) {
				if (i == 49) {
					is_revgdt = 1;
					reserved_blk_cnt++;
					start_blkbit_addr += reserved_gdt_blk + 2;
					start_indbit_addr += reserved_gdt_blk + 2;
					start_indtable_addr += reserved_gdt_blk + 2;
					free_blk_cnt -= (reserved_gdt_blk + 2);
					rsv_blk_cnt++;
				}
			}

			free_blocks += free_blk_cnt;
			free_inodes += free_inode_cnt;

			mk4(desc_table->start_blkbit_addr, start_blkbit_addr);
			mk4(desc_table->start_indbit_addr, start_indbit_addr);
			mk4(desc_table->start_inode_table, start_indtable_addr);
			mk2(desc_table->free_blk_cnt, free_blk_cnt);

			if (i == 1) {
				seblk_offset = start_indtable_addr - 0x8000;
			}
			if (i == bgcnt) { //original code
			//if (i == 0) { //first block
			//if (i == blk_group_num -1) { // last block
				current_free_blk = free_blk_cnt;
				current_blkbit_addr = start_blkbit_addr;
				current_indbit_addr = start_indbit_addr;
				current_indtable_addr = start_indtable_addr;
				if (is_revgdt)
					have_rev_gdt = 1;

			}
			mk2(desc_table->free_inode_cnt, free_inode_cnt);
			if (i == 0)
				mk2(desc_table->directories_cnt, 2);
			else
				mk2(desc_table->directories_cnt, 0);
			offset += sizeof(struct group_desc_table);
		}

		if (set_journaling) {

			/* set journal blocks
			 * journal data start address =
			 * next of first inode_table block
			 * + (size of inode table)
			 * + 6 (size of root & reserved data region)
			 */
			if (total_blocks > 0x8000) { /* if size > 128MB */
				jnldata_start_addr = first_inode_table_addr
						+ (inodes_per_group * 128 / 4096) + 6;
			}
			else {
				jnldata_start_addr = first_inode_table_addr
						+ (inodes_per_group * 128 / 4096) + 5;
			}

			mk4(sblock->journal_blocks, jnldata_start_addr);
			mk4(sblock->journal_blocks + 4, jnldata_start_addr + 1);
			mk4(sblock->journal_blocks + 8, jnldata_start_addr + 2);
			mk4(sblock->journal_blocks + 12, jnldata_start_addr + 3);
			mk4(sblock->journal_blocks + 16, jnldata_start_addr + 4);
			mk4(sblock->journal_blocks + 20, jnldata_start_addr + 5);
			mk4(sblock->journal_blocks + 24, jnldata_start_addr + 6);
			mk4(sblock->journal_blocks + 28, jnldata_start_addr + 7);
			mk4(sblock->journal_blocks + 32, jnldata_start_addr + 8);
			mk4(sblock->journal_blocks + 36, jnldata_start_addr + 9);
			mk4(sblock->journal_blocks + 40, jnldata_start_addr + 10);
			mk4(sblock->journal_blocks + 44, jnldata_start_addr + 11);
			mk4(sblock->journal_blocks + 48, jnldata_start_addr + 12);

			int default_jnl_blocks = default_journal_size(total_blocks);
			uint32_t temp_val=0;
			if (default_jnl_blocks == 1024)
				temp_val = 0x400000;
			else if (default_jnl_blocks == 4096)
				temp_val = 0x1000000;
			else if (default_jnl_blocks == 8192)
				temp_val = 0x2000000;
			else if (default_jnl_blocks == 16384)
				temp_val = 0x4000000;
			else if (default_jnl_blocks == 32768)
				temp_val = 0x8000000;
			mk4(sblock->journal_blocks + 64, temp_val);

			/* 400 : 0x400(1024) means amount of pointing by 1block
			 *       4096(1block) / 4 = 1024
			 */
			if (default_jnl_blocks != 1024)
				mk4(sblock->journal_blocks + 52, jnldata_start_addr + 0x400 + 13);
		}
		mk4(sblock->free_blocks, free_blocks); /* free block count */
		mk4(sblock->free_inodes, free_inodes); /* free inode count */
		free_inodes += 0x10;
		free_inodes &= 0xfffffff0;
		mk4(sblock->total_inodes, free_inodes); /* inode count */

		/* print value */
		/*
		printf("Print Super Blocks\n");
		for(i = 0;i < 4096; i++) {
			if(img[i] == 0)
				printf("00 ");
			else
				printf("%02x ", img[i]);

			if (!((i+1) % 4))
				printf("| ");
			if (!((i+1) % 16))
				printf("\n");
			if (!((i+1) % 512))
				printf("---------------------------------------------------%d\n", i+1);
		}
		*/

		/* Write super-block to mmc */
		start_block = info.start + (bgcnt * 0x8000 * 4096 / 512);
		printf("Start write addr : 0x%x\n",start_block);
		if (have_rev_gdt) {
			if (blk_dwrite(dev_desc, start_block, 8, (ulong *)img) != 8) {
				printf("Can't write Superblock(%d)~~~!!!\n", bgcnt);
			}
		}

		/*
		printf("\n\nPrint Group Descriptor Table\n");
		for(i = 0;i < 4096; i++) {
			if(img2[i] == 0)
				printf("00 ");
			else
				printf("%02x ", img2[i]);
			if (!((i+1) % 4))
				printf("| ");
			if (!((i+1) % 16))
				printf("\n");
			if (!((i+1) % 512))
				printf("---------------------------------------------------%d\n", i+1);
		}
		*/

		/* Write Group Descriptor Table */
		if (have_rev_gdt) {
			start_block += 8;
			if (blk_dwrite(dev_desc, start_block, 8,
				(ulong *)img2) != 8) {
				printf("Can't write Descriptor Table(%d)~~~!!!\n", bgcnt);
			}
		}

		/* Write reserved region */
		if (bgcnt == 0) {
			memset(reserve_img, 0x0, sizeof(__u8) * 4096);

			int rev_cnt;
			for (rev_cnt = 2; rev_cnt <= reserved_gdt_blk +
				1; rev_cnt++) {
				start_block += 8;
				if (rsv_blk_cnt >= 1)
					mk4(reserve_img, 0x8000 + rev_cnt);
				if (rsv_blk_cnt >= 2)
					mk4(reserve_img + 4, 0x18000 + rev_cnt);
				if (rsv_blk_cnt >= 3)
					mk4(reserve_img + 8, 0x28000 + rev_cnt);
				if (rsv_blk_cnt >= 4)
					mk4(reserve_img + 12, 0x38000 + rev_cnt);
				if (rsv_blk_cnt >= 5)
					mk4(reserve_img + 16, 0x48000 + rev_cnt);
				if (rsv_blk_cnt >= 6)
					mk4(reserve_img + 20, 0xc8000 + rev_cnt);
				if (rsv_blk_cnt >= 7)
					mk4(reserve_img + 24, 0xd8000 + rev_cnt);
				if (rsv_blk_cnt >= 8)
					mk4(reserve_img + 28, 0x188000 + rev_cnt);
				if (blk_dwrite(dev_desc, start_block, 8,
					(ulong *)reserve_img) != 8) {
					printf("Can't write reserve(%d)~~~!!!\n", bgcnt);
				}
			}
		}

		/* Write Block Bitmap */
		uint8_t *blk_bitmap=NULL;
		int used_blocks;
		int set_full;
		int j;
		int set_val;
		uint32_t bitmap_val;
		int empty_blocks;


		if (bgcnt != blk_group_num - 1) {
			memset(img3, 0x00, sizeof(__u8) * 4096);

			used_blocks = 0x8000 - current_free_blk;
			set_full = used_blocks / 0x20;
			set_val = used_blocks % 0x20;
			bitmap_val = 0;

			for (i = 0;i <= set_full;i++) {
				blk_bitmap = (uint8_t *)(img3 + (4 * i));
				if(i == set_full) {
					for (j = 0;j <= set_val - 1;j++) {
						bitmap_val |= 1 << j;
					}
					mk4(blk_bitmap, bitmap_val);
				} else {
					mk4(blk_bitmap, 0xffffffff);
				}
			}
		} else {
			memset(img3, 0xff, sizeof(__u8) * 4096);

			used_blocks = 0x8000 - (current_free_blk + (0x8000 -
					(total_blocks % 0x8000)));

			set_full = used_blocks / 0x20;
			set_val = used_blocks % 0x20;

			bitmap_val = 0x0;
			//==> write empty blocks
			empty_blocks = (current_free_blk + used_blocks) / 32;
			int temp = (current_free_blk + used_blocks) % 32;
			for (i = 0; i <= empty_blocks; i++) {
				blk_bitmap = (uint8_t *)(img3 + (4 * i));
				if (i == empty_blocks ) {
					for (j = temp;j < 32; j++) {
						bitmap_val |= 1 << j;
					}
					mk4(blk_bitmap, bitmap_val);
				} else {
					mk4(blk_bitmap, 0x00);
				}
			}

			bitmap_val = 0;
			for (i = 0;i <= set_full;i++) {
				blk_bitmap = (uint8_t *)(img3 + (4 * i));
				if(i == set_full) {
					for (j = 0;j <= set_val - 1;j++) {
						bitmap_val |= 1 << j;
					}
					mk4(blk_bitmap, bitmap_val);
				} else {
					mk4(blk_bitmap, 0xffffffff);
				}
			}
		}

		/*
		printf("\n\nPrint Block Bitmap(0x%x)\n",current_free_blk);
		for(i = 0;i < 4096; i++) {
			if(img3[i] == 0)
				printf("00 ");
			else
				printf("%02x ", img3[i]);
			if (!((i+1) % 4))
				printf("| ");
			if (!((i+1) % 16))
				printf("\n");
			if (!((i+1) % 512))
				printf("---------------------------------------------------%d\n", i+1);
		}
		*/

		/* Write block bitmap */
		start_block = (current_blkbit_addr * 8) + info.start;

		if (blk_dwrite(dev_desc, start_block, 8,
			(ulong *)img3) != 8) {
			printf("Can't write Descriptor Table(%d)~~~!!!\n", bgcnt);
		}

		/* Write inode bitmap */
		uint8_t *inode_bitmap;

		memset(img4, 0xff, sizeof(__u8) * 4096);

		int empty_inode = inodes_per_group;
		int set_empty = empty_inode / 0x20;

		// Fix me~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		if (total_blocks < 0x8000) /* if size < 128MB */
			set_empty -= 1;

		for (i = 0;i < set_empty;i++) {
			inode_bitmap = (uint8_t *)(img4 + (4 * i));
			mk4(inode_bitmap, 0);
		}

		if (bgcnt == 0) {
			inode_bitmap = (uint8_t *)img4;
			mk4(inode_bitmap, 0x7ff);
		}

		/*
		printf("\n\nPrint Inode Bitmap0\n");
		for(i = 0;i < 4096; i++) {
			if(img4[i] == 0)
				printf("00 ");
			else
				printf("%02x ", img4[i]);
			if (!((i+1) % 4))
				printf("| ");
			if (!((i+1) % 16))
				printf("\n");
			if (!((i+1) % 512))
				printf("---------------------------------------------------%d\n", i+1);
		}
		*/

		/* Write inode bitmap */
		start_block = (current_indbit_addr * 8) + info.start;
		if (blk_dwrite(dev_desc, start_block, 8,
			(ulong *)img4) != 8) {
			printf("Can't write inode bitmap(%d)~~~!!!\n", bgcnt);
		}

		/* Write Inode Table & data */
		start_block = (current_indtable_addr * 8) + info.start;
		//Erase to 0x00 to initialize inode table.....
		memset(zerobuf, 0x00, sizeof(__u8) * 4096 * 80);
		int indt_blknum = (inodes_per_group * 128 / 4096) + 1;
		printf("Erase inode table(%d) - 0x%x", bgcnt, start_block);
		for (j = 0; j < (indt_blknum /80) + 4; j++) { // 2 means root + a
			start_block += j * 8;
			if (blk_dwrite(dev_desc,
				start_block, 640, (ulong *)zerobuf) != 640) {
				printf("Can't erase inode table(%d)~~~!!!\n", bgcnt);
			}
			printf(".");
		}
		printf("\n");

		struct inode_desc *inode;

		memset(img5, 0x00, sizeof(__u8) * 2048);

		// 1st inode : bad inode
		inode = (struct inode_desc *)img5;
		mk4(inode->access_time, 0x33); /* 0x33 means nothing */
		mk4(inode->change_time, 0x33); /* 0x33 means nothing */
		mk4(inode->modify_time, 0x33); /* 0x33 means nothing */

		// 2nd inode : Root inode
		inode = (struct inode_desc *)(img5 + sizeof(struct inode_desc));
		mk2(inode->file_mode, 0x41ed);
		mk4(inode->size_byte, 0x1000);
		mk4(inode->access_time, 0x33); /* 0x33 means nothing */
		mk4(inode->change_time, 0x33); /* 0x33 means nothing */
		mk4(inode->modify_time, 0x33); /* 0x33 means nothing */
		mk2(inode->link_count, 0x3);
		mk2(inode->block_count, 0x8);
		/* root data block addr =
		 * inode_table_addr + inode table size
		 */
		uint32_t root_data_blk = (inodes_per_group * 128) / 4096;
		if (total_blocks > 0x8000) /* if size > 128MB */
			root_data_blk += first_inode_table_addr;
		else
			root_data_blk += first_inode_table_addr - 1;

		mk4(inode->block_pointers, root_data_blk);
		// Write root directory in data region.
		if (bgcnt == 0) {
			/* root data */
			memset(rootdata, 0x00, sizeof(__u8) * 4096);
			mk4(rootdata, 0x2);
			mk4(rootdata + 4, 0x201000c);
			mk4(rootdata + 8, 0x2e);
			mk4(rootdata + 12, 0x2);
			mk4(rootdata + 16, 0x202000c);
			mk4(rootdata + 20, 0x2e2e);
			mk4(rootdata + 24, 0xb);
			mk4(rootdata + 28, 0x20a0fe8);
			mk4(rootdata + 32, 0x74736f6c);
			mk4(rootdata + 36, 0x756f662b);
			mk4(rootdata + 40, 0x646e);
			if (blk_dwrite(dev_desc,
				(root_data_blk * 8) + info.start, 8,
				(ulong *)rootdata) != 8) {
				printf("Can't write rootdata~~~!!!\n");
			}

			/* root + 1 data */
			root_data_blk++;
			memset(inode_data, 0x00, sizeof(__u8) * 4096);
			mk4(inode_data, 0xb);
			mk4(inode_data + 4, 0x201000c);
			mk4(inode_data + 8, 0x2e);
			mk4(inode_data + 12, 0x2);
			mk4(inode_data + 16, 0x2020ff4);
			mk4(inode_data + 20, 0x2e2e);
			if (blk_dwrite(dev_desc,
				(root_data_blk * 8) + info.start, 8,
				(ulong*)inode_data) != 8) {
				printf("Can't write root+1~~~!!!\n");
			}
			/* root + 2 data */
			root_data_blk++;
			memset(inode_data, 0x00, sizeof(__u8) * 4096);
			mk4(inode_data + 4, 0x1000);
			if (blk_dwrite(dev_desc,
				(root_data_blk * 8) + info.start, 8,
				(ulong*)inode_data) != 8) {
				printf("Can't write root+1~~~!!!\n");
			}

			/* root + 3 data */
			root_data_blk++;
			memset(inode_data, 0x00, sizeof(__u8) * 4096);
			mk4(inode_data + 4, 0x1000);
			if (blk_dwrite(dev_desc,
				(root_data_blk * 8) + info.start, 8,
				(ulong*)inode_data) != 8) {
				printf("Can't write root+1~~~!!!\n");
			}

			/* root + 4 data */
			root_data_blk++;
			memset(inode_data, 0x00, sizeof(__u8) * 4096);
			mk4(inode_data + 4, 0x1000);
			if (blk_dwrite(dev_desc,
				(root_data_blk * 8) + info.start, 8,
				(ulong*)inode_data) != 8) {
				printf("Can't write root+1~~~!!!\n");
			}
		}

		// 3 ~ 6th inode : empty
		// 7th inode : reserved_gdt_blocks
		inode = (struct inode_desc *)(img5 + sizeof(struct inode_desc) * 6);
		mk2(inode->file_mode, 0x8180);
		mk4(inode->size_byte, 0x40c000);
		mk4(inode->access_time, 0x33); /* 0x33 means nothing */
		mk4(inode->change_time, 0x33); /* 0x33 means nothing */
		mk4(inode->modify_time, 0x33); /* 0x33 means nothing */
		mk2(inode->link_count, 0x1);
		uint32_t inode7_cnt;
		// 4096 * 512 ==> changing from ext2block to mmc block....
		inode7_cnt = ((reserved_gdt_blk * reserved_blk_cnt) + 1) * 4096 / 512;
		mk2(inode->block_count, inode7_cnt);
		root_data_blk ++;
		mk4(inode->block_pointers + 52, root_data_blk);
		mk4(inode->directory_acl, 1);

		// Write 7th inode data
		if (bgcnt == 0) {
			memset(inode_data, 0x00, sizeof(__u8) * 4096);
			/* original code
			uint32_t temp;
			if (reserved_gdt_blk % 2) {
				temp = reserved_gdt_blk /= 2;
				temp++;
				temp *= 2;
			}
			for(j=0; j< temp; j++) {
				if ( j != 0)
					mk4(inode_data + (j * 4), j + 1);
			}
			*/
			for(j=0; j< current_blkbit_addr - 1; j++) {
				if ( j != 0)
					mk4(inode_data + (j * 4), j + 1);
			}
			if (blk_dwrite(dev_desc,
				(root_data_blk * 8) + info.start, 8,
				(ulong*)inode_data) != 8) {
				printf("Can't write 7th inode~~~!!!\n");
			}
		}

		if (set_journaling) {
			// 8th inode : reserved for journaling
			inode = (struct inode_desc *)(img5 + sizeof(struct
					inode_desc) * 7);
			mk2(inode->file_mode, 0x8180);
			uint32_t indirect_pointer_addr = 0;
			uint32_t d_indirect_pointer_addr = 0;
			int default_jnl_blocks = default_journal_size(total_blocks);
			uint32_t temp_val=0;
			if (default_jnl_blocks == 1024)
				temp_val = 0x400000;
			else if (default_jnl_blocks == 4096)
				temp_val = 0x1000000;
			else if (default_jnl_blocks == 8192)
				temp_val = 0x2000000;
			else if (default_jnl_blocks == 16384)
				temp_val = 0x4000000;
			else if (default_jnl_blocks == 32768)
				temp_val = 0x8000000;
			mk4(inode->size_byte, temp_val);
			mk4(inode->change_time, 0x33);
			mk4(inode->modify_time, 0x33);
			mk2(inode->link_count, 0x1);
			// set journaling blocks by 512 per 1block.
			int inode_jnl_blocks = jnl_blocks * 4096 / 512;
			if (total_blocks > 0x8000) /* if size > 128MB */
				mk4(inode->block_count, inode_jnl_blocks);
			else
				mk4(inode->block_count, inode_jnl_blocks + 8);

			// set journaling block address
			mk4(inode->block_pointers, jnldata_start_addr);
			mk4(inode->block_pointers + 4, jnldata_start_addr + 1);
			mk4(inode->block_pointers + 8, jnldata_start_addr + 2);
			mk4(inode->block_pointers + 12, jnldata_start_addr + 3);
			mk4(inode->block_pointers + 16, jnldata_start_addr + 4);
			mk4(inode->block_pointers + 20, jnldata_start_addr + 5);
			mk4(inode->block_pointers + 24, jnldata_start_addr + 6);
			mk4(inode->block_pointers + 28, jnldata_start_addr + 7);
			mk4(inode->block_pointers + 32, jnldata_start_addr + 8);
			mk4(inode->block_pointers + 36, jnldata_start_addr + 9);
			mk4(inode->block_pointers + 40, jnldata_start_addr + 10);
			mk4(inode->block_pointers + 44, jnldata_start_addr + 11);
			indirect_pointer_addr = jnldata_start_addr + 12;
			mk4(inode->block_pointers + 48, indirect_pointer_addr);

			/* If journaling data blocks are bigger than 4096, it
			 * use double indirect pointer (block13)
			 */
			if (default_jnl_blocks >= 4096) {
				/* 400 : 0x400(1024) means amount of pointing by 1block
				 *       4096(1block) / 4 = 1024
				 */
				d_indirect_pointer_addr = jnldata_start_addr
							+ 0x400 + 13;
				mk4(inode->block_pointers + 52,
					d_indirect_pointer_addr);
			}
			// Write 8th inode data
			if (bgcnt == 0) {
				memset(inode_data, 0x00, sizeof(__u8) * 4096);
				mk4(inode_data, 0x98393bc0);
				mk4(inode_data + 4, 0x4000000);
				mk4(inode_data + 12, 0x100000);
				temp_val >>= 4;
				mk4(inode_data + 16, temp_val);
				mk4(inode_data + 20, 0x1000000);
				mk4(inode_data + 24, 0x1000000);
				// set UUID
				mk4(inode_data + 48, uuid[0]);
				mk4(inode_data + 52, uuid[1]);
				mk4(inode_data + 56, uuid[2]);
				mk4(inode_data + 60, uuid[3]);
				mk4(inode_data + 64, 0x1000000);

				//write data to sd/mmc
				if (blk_dwrite(dev_desc,
					(jnldata_start_addr * 8) + info.start, 8,
					(ulong*)inode_data) != 8) {
					printf("Can't write 8th inode~~~!!!\n");
				}

				// Write indirect pointer value
				// 12 means num of default direct pointer
				uint32_t remain_blocks = jnl_blocks - 12;
				memset(inode_data, 0x00, sizeof(__u8) * 4096);
				for (j = 0; j < 1024; j++) {
					mk4(inode_data + (j * 4),
						indirect_pointer_addr + 1 + j);
					remain_blocks--;
					if (remain_blocks == 0)
						break;
				}
				//write data to sd/mmc
				if (blk_dwrite(dev_desc,
					(indirect_pointer_addr * 8) + info.start, 8,
					(ulong*)inode_data) != 8) {
					printf("Can't write 8th inode~~~!!!\n");
				}

				//Write double indirect pointer
				if (remain_blocks != 0) {
					memset(inode_data, 0x00, sizeof(__u8) * 4096);
					int k;
					int needed_blocks = remain_blocks / 1024;
					// Check me==================>
					if (total_blocks > 0x8000) /* if size > 128MB */
						if (remain_blocks % 1024)
							needed_blocks += 1;
					//<==========================
					remain_blocks -= (needed_blocks + 2);

					for (j=0;j < needed_blocks; j++) {
						int now_val =
						d_indirect_pointer_addr + 1 + (j * 0x400) + j;
						mk4(inode_data + (j * 4), now_val);

						memset(inode_data2, 0x00, sizeof(__u8) * 4096);
						for (k = 0; k < 1024; k++) {
							uint32_t tmp_val = now_val + 1 + k;
							if (tmp_val >= 0x8000) {
								tmp_val +=
								seblk_offset + (inodes_per_group * 128 / 4096);
							}
							mk4(inode_data2 + (k * 4), tmp_val);
							remain_blocks--;
							if(remain_blocks == 0)
								break;
						}
						//write data to sd/mmc
						if (blk_dwrite(dev_desc,
							(now_val * 8) + info.start, 8,
							(ulong*)inode_data2) != 8) {
							printf("Can't  inodeval~~~!!!\n");
						}
					}
					//write data to sd/mmc
					printf("d_indirect_point:0x%x\n",
						(unsigned int)((d_indirect_pointer_addr * 8) + info.start));
					if (blk_dwrite(dev_desc,
						(d_indirect_pointer_addr * 8) + info.start, 8,
						(ulong*)inode_data) != 8) {
						printf("Can't write 8th inode~~~!!!\n");
					}
				}
			}
		}

		// 9  ~ 10th inode : empty
		// 11th inode : first inode without reserved
		//              It point next of root(lost+found)
		inode = (struct inode_desc *)(img5 + sizeof(struct inode_desc) * 10);
		mk2(inode->file_mode, 0x41c0);
		mk4(inode->size_byte, 0x4000);
		mk4(inode->access_time, 0x33); /* 0x33 means nothing */
		mk4(inode->change_time, 0x33); /* 0x33 means nothing */
		mk4(inode->modify_time, 0x33); /* 0x33 means nothing */
		mk2(inode->link_count, 0x2);
		mk2(inode->block_count, 0x20);
		root_data_blk -= 4;
		mk4(inode->block_pointers, root_data_blk);
		mk4(inode->block_pointers + 4, root_data_blk + 1);
		mk4(inode->block_pointers + 8, root_data_blk + 2);
		mk4(inode->block_pointers + 12, root_data_blk + 3);


		/*
		printf("\n\nPrint inode table0\n");
		for(i = 0;i < 2048; i++) {
			if(img5[i] == 0)
				printf("00 ");
			else
				printf("%02x ", img5[i]);
			if (!((i+1) % 4))
				printf("| ");
			if (!((i+1) % 16))
				printf("\n");
			if (!((i+1) % 512))
				printf("---------------------------------------------------%d\n", i+1);
		}
		*/

		if (bgcnt == 0) {
			/* Write inode table */
			start_block = (current_indtable_addr * 8) + info.start;
			if (blk_dwrite(dev_desc, start_block, 4,
				(ulong *)img5) != 4) {
				printf("Can't write inode table(%d)~~~!!!\n", bgcnt);
			}
		}
	}
	free(img);
	free(img2);
	free(reserve_img);
	free(img3);
	free(img4);
	free(zerobuf);
	free(img5);
	free(rootdata);
	free(inode_data);

	return 0;
}
