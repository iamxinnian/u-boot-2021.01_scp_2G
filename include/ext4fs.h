/*
 * (C) Copyright 2011 - 2012 Samsung Electronics
 * EXT4 filesystem implementation in Uboot by
 * Uma Shankar <uma.shankar@samsung.com>
 * Manjunatha C Achar <a.manjunatha@samsung.com>
 *
 * Ext4 Extent data structures are taken from  original ext4 fs code
 * as found in the linux kernel.
 *
 * Copyright (c) 2003-2006, Cluster File Systems, Inc, info@clusterfs.com
 * Written by Alex Tomas <alex@clusterfs.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef __EXT4__
#define __EXT4__
#include <ext_common.h>

struct disk_partition;

#define EXT4_INDEX_FL		0x00001000 /* Inode uses hash tree index */
#define EXT4_EXTENTS_FL		0x00080000 /* Inode uses extents */
#define EXT4_EXT_MAGIC			0xf30a
#define EXT4_FEATURE_RO_COMPAT_GDT_CSUM	0x0010
#define EXT4_FEATURE_RO_COMPAT_METADATA_CSUM 0x0400
#define EXT4_FEATURE_INCOMPAT_EXTENTS	0x0040
#define EXT4_FEATURE_INCOMPAT_64BIT	0x0080
#define EXT4_INDIRECT_BLOCKS		12

#define EXT4_BG_INODE_UNINIT		0x0001
#define EXT4_BG_BLOCK_UNINIT		0x0002
#define EXT4_BG_INODE_ZEROED		0x0004

#define mk1(p, x)	\
 	(p) = (__u8)(x)

#define mk2(p, x)		\
	(p)[0] = (__u8)(x),	\
	(p)[1] = (__u8)((x) >> 010)

#define mk4(p, x)			\
	(p)[0] = (__u8)(x),		\
	(p)[1] = (__u8)((x) >> 010),	\
	(p)[2] = (__u8)((x) >> 020),	\
	(p)[3] = (__u8)((x) >> 030)

/*
 * Format device by ext2.
 */
struct super_block {
	__u8 total_inodes[4];
	__u8 total_blocks[4];
	__u8 reserved_blocks[4];
	__u8 free_blocks[4];
	__u8 free_inodes[4];
	__u8 first_data_block[4];
	__u8 log2_block_size[4];
	__u8 log2_fragment_size[4];
	__u8 blocks_per_group[4];
	__u8 fragments_per_group[4];
	__u8 inodes_per_group[4];
	__u8 mtime[4];
	__u8 wtime[4];
	__u8 mnt_count[2];
	__u8 max_mnt_count[2];
	__u8 magic[2];
	__u8 fs_state[2];
	__u8 error_handling[2];
	__u8 minor_revision_level[2];
	__u8 lastcheck[4];
	__u8 checkinterval[4];
	__u8 creator_os[4];
	__u8 revision_level[4];
	__u8 uid_reserved[2];
	__u8 gid_reserved[2];
	__u8 first_inode[4];
	__u8 inode_size[2];
	__u8 block_group_number[2];
	__u8 feature_compatibility[4];
	__u8 feature_incompat[4];
	__u8 feature_ro_compat[4];
	__u8 unique_id[16];
	__u8 volume_name[16];
	__u8 last_mounted_on[64];
	__u8 compression_info[4];
	__u8 prealloc_blk_cnt;
	__u8 prealloc_dir_cnt;
	__u8 reserved_gtd_blk[2];
	__u8 journal_uuid[16];
	__u8 journal_inode_num[4];
	__u8 journal_device[4];
	__u8 orphan_inode_list[4];
	__u8 hash_seed[16];
	__u8 hash_version;
	__u8 journal_backup_type;
	__u8 gtd_size[2];
	__u8 mount_option[4];
	__u8 first_meta_blk[4];
	__u8 mkfs_time[4];
	__u8 journal_blocks[68];
	__u8 blocks_cnt_hi[4];
	__u8 reserved_blk_cnt_hi[4];
	__u8 free_blocks_hi[4];
	__u8 min_extra_isize[2];
	__u8 want_extra_isize[2];
	__u8 flags[4];
	__u8 raid_stride[2];
	__u8 mmp_interval[2];
	__u8 mmp_block[8];
	__u8 raid_stripe_width[4];
};

struct group_desc_table {
	__u8 start_blkbit_addr[4];
	__u8 start_indbit_addr[4];
	__u8 start_inode_table[4];
	__u8 free_blk_cnt[2];
	__u8 free_inode_cnt[2];
	__u8 directories_cnt[2];
	__u8 padding[2];
	__u8 reserved[12];
};

struct inode_desc {
	__u8 file_mode[2];
	__u8 uid[2];
	__u8 size_byte[4];
	__u8 access_time[4];
	__u8 change_time[4];
	__u8 modify_time[4];
	__u8 deletion_time[4];
	__u8 group_id[2];
	__u8 link_count[2];
	__u8 block_count[4];
	__u8 flags[4];
	__u8 os_description1[4];
	__u8 block_pointers[60];
	__u8 generation_num[4];
	__u8 file_acl[4];
	__u8 directory_acl[4];
	__u8 address_fragmentation[4];
	__u8 os_description2[12];
};

/*
 * ext4_inode has i_block array (60 bytes total).
 * The first 12 bytes store ext4_extent_header;
 * the remainder stores an array of ext4_extent.
 */

/*
 * This is the extent on-disk structure.
 * It's used at the bottom of the tree.
 */
struct ext4_extent {
	__le32	ee_block;	/* first logical block extent covers */
	__le16	ee_len;		/* number of blocks covered by extent */
	__le16	ee_start_hi;	/* high 16 bits of physical block */
	__le32	ee_start_lo;	/* low 32 bits of physical block */
};

/*
 * This is index on-disk structure.
 * It's used at all the levels except the bottom.
 */
struct ext4_extent_idx {
	__le32	ei_block;	/* index covers logical blocks from 'block' */
	__le32	ei_leaf_lo;	/* pointer to the physical block of the next *
				 * level. leaf or next index could be there */
	__le16	ei_leaf_hi;	/* high 16 bits of physical block */
	__u16	ei_unused;
};

/* Each block (leaves and indexes), even inode-stored has header. */
struct ext4_extent_header {
	__le16	eh_magic;	/* probably will support different formats */
	__le16	eh_entries;	/* number of valid entries */
	__le16	eh_max;		/* capacity of store in entries */
	__le16	eh_depth;	/* has tree real underlying blocks? */
	__le32	eh_generation;	/* generation of the tree */
};

struct ext_filesystem {
	/* Total Sector of partition */
	uint64_t total_sect;
	/* Block size  of partition */
	uint32_t blksz;
	/* Inode size of partition */
	uint32_t inodesz;
	/* Sectors per Block */
	uint32_t sect_perblk;
	/* Group Descriptor size */
	uint16_t gdsize;
	/* Group Descriptor Block Number */
	uint32_t gdtable_blkno;
	/* Total block groups of partition */
	uint32_t no_blkgrp;
	/* No of blocks required for bgdtable */
	uint32_t no_blk_pergdt;
	/* Superblock */
	struct ext2_sblock *sb;
	/* Block group descritpor table */
	char *gdtable;

	/* Block Bitmap Related */
	unsigned char **blk_bmaps;
	long int curr_blkno;
	uint16_t first_pass_bbmap;

	/* Inode Bitmap Related */
	unsigned char **inode_bmaps;
	int curr_inode_no;
	uint16_t first_pass_ibmap;

	/* Journal Related */

	/* Block Device Descriptor */
	struct blk_desc *dev_desc;
};

struct ext_block_cache {
	char *buf;
	lbaint_t block;
	int size;
};

extern struct ext2_data *ext4fs_root;
extern struct ext2fs_node *ext4fs_file;

#if defined(CONFIG_EXT4_WRITE)
extern struct ext2_inode *g_parent_inode;
extern int gd_index;
extern int gindex;

int ext4fs_init(void);
void ext4fs_deinit(void);
int ext4fs_filename_unlink(char *filename);
int ext4fs_write(const char *fname, const char *buffer,
				 unsigned long sizebytes, int type);
int ext4_write_file(const char *filename, void *buf, loff_t offset, loff_t len,
		    loff_t *actwrite);
int ext4fs_create_link(const char *target, const char *fname);
#endif

struct ext_filesystem *get_fs(void);
int ext4fs_open(const char *filename, loff_t *len);
int ext4fs_read(char *buf, loff_t offset, loff_t len, loff_t *actread);
int ext4fs_mount(unsigned part_length);
void ext4fs_close(void);
void ext4fs_reinit_global(void);
int ext4fs_ls(const char *dirname);
int ext4fs_exists(const char *filename);
int ext4fs_size(const char *filename, loff_t *size);
void ext4fs_free_node(struct ext2fs_node *node, struct ext2fs_node *currroot);
int ext4fs_devread(lbaint_t sector, int byte_offset, int byte_len, char *buf);
void ext4fs_set_blk_dev(struct blk_desc *rbdd, struct disk_partition *info);
long int read_allocated_block(struct ext2_inode *inode, int fileblock,
			      struct ext_block_cache *cache);
int ext4fs_probe(struct blk_desc *fs_dev_desc,
		 struct disk_partition *fs_partition);
int ext4_read_file(const char *filename, void *buf, loff_t offset, loff_t len,
		   loff_t *actread);
int ext4_read_superblock(char *buffer);
int ext4fs_uuid(char *uuid_str);
void ext_cache_init(struct ext_block_cache *cache);
void ext_cache_fini(struct ext_block_cache *cache);
int ext_cache_read(struct ext_block_cache *cache, lbaint_t block, int size);
int ext4fs_format(struct blk_desc *dev_desc, int part_no, char set_journaling,struct disk_partition info);
#endif
