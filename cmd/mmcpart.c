#include <mmcpart.h>

static struct _part_info parts[PART_MAX] = {
	DEF_PART(uboot,  "bootloader", PART_FLAG_BOOTABLE),
	DEF_PART(env,    "env",        PART_FLAG_NONE),
	DEF_PART(kernel, "kernel",     PART_FLAG_NONE),
	DEF_PART(dtb,   "dtb",        PART_FLAG_NONE),
	DEF_PART(mbr1,   "sdmmc",      PART_FLAG_NONE),
	DEF_PART(mbr2,   "system",     PART_FLAG_NONE),
	DEF_PART(mbr3,   "data",       PART_FLAG_NONE),
	DEF_PART(mbr4,   "cache",      PART_FLAG_NONE),
};

static void show_parts_info(void)
{
	struct _part_info *part;
	int i;

	printf("\n");
	printf("  partion          block start              block count          flag \n\n");
	for (i = uboot; i < PART_MAX; i++)
	{
		part = &parts[i];
		printf("  %-10s     %7ld 0x%-8lX      %8ld 0x%-8lX     0x%.2X \n",
			part->name, part->blk_start, part->blk_start,
			part->blk_count, part->blk_count, part->flag);
	}
	printf("\n");
}

static void calc_parts_start(void)
{
	int i;
	unsigned long start;

	start = parts[env].blk_start + parts[env].blk_count;

	for (i = env + 1; i < mbr1; i++) {
		parts[i].blk_start = start;
		start += parts[i].blk_count;
	}
}

static void read_mbr_parts_info(void)
{
	int i, ret;
	struct _part_info *part;
	int block_start, block_count;
	unsigned char part_id;
	char dev[2] = {"0"};
	for(i = mbr1; i < mbr_end; i++)
	{
		ret = get_mmc_part_info(dev, i-mbr1+1, &block_start, &block_count, &part_id);

		part = &parts[i];
		part->blk_start = (unsigned long)block_start;
		part->blk_count = (unsigned long)block_count;
		if (part_id == PART_EXT4) {
			part->flag |= PART_FLAG_EXT4;
			part->part_id = part_id;
		}
		else if (part_id == PART_FAT) {
			part->flag |= PART_FLAG_FAT;
			part->part_id = part_id;
		}
	}
}

int itop4412_partition_init(void)
{
	parts[uboot].blk_start  = UBOOT_BLK_START;
	parts[uboot].blk_count  = UBOOT_BLK_COUNT;

	parts[env].blk_start    = ENV_BLK_START;
	parts[env].blk_count    = 
		(CONFIG_ENV_SIZE >> BLK_SHIFT) + !!(CONFIG_ENV_SIZE & (BLK_SIZE - 1));

	parts[dtb].blk_count   = KB_BLK(CONFIG_PART_DTB_SIZE);

	parts[kernel].blk_count = MB_BLK(CONFIG_PART_KERNEL_SIZE);

	read_mbr_parts_info();
	calc_parts_start();
	show_parts_info();
	return 0;
}

//==================================================================
#define ext4_printf(args, ...)

static int check_compress_ext4(const void *img_base, const unsigned long parti_blk)
{
	const ext4_file_header *file_header;

	file_header = (const ext4_file_header *)img_base;

	if (file_header->magic != EXT4_FILE_HEADER_MAGIC) {
		printf("Invalid EXT4 Magic! 0x%2x\n", file_header->magic);
		return CMD_RET_FAILURE;
	}

	if (file_header->major != EXT4_FILE_HEADER_MAJOR) {
		printf("Invalid Version Info! 0x%2x\n", file_header->major);
		return CMD_RET_FAILURE;
	}

	if (file_header->file_header_size != EXT4_FILE_HEADER_SIZE) {
		printf("Invalid File Header Size! 0x%8x\n", file_header->file_header_size);
		return CMD_RET_FAILURE;
	}

	if (file_header->chunk_header_size != EXT4_CHUNK_HEADER_SIZE) {
		printf("Invalid Chunk Header Size! 0x%8x\n", file_header->chunk_header_size);
		return CMD_RET_FAILURE;
	}

	if (file_header->block_size != EXT4_FILE_BLOCK_SIZE) {
		printf("Invalid Block Size! 0x%8x\n", file_header->block_size);
		return CMD_RET_FAILURE;
	}

	if (parti_blk/(file_header->block_size/(1<<BLK_SHIFT))  < file_header->total_blocks) {
		printf("Invalid Volume Size! Image is bigger than partition size!\n");
		printf("partion size %lu ,image size %u \n",
			parti_blk/(file_header->block_size/(1<<BLK_SHIFT)), file_header->total_blocks);
		return CMD_RET_FAILURE;
	}

	/* image is compressed ext4 */
	return 0;
}

int write_raw_chunk(const void* data, const unsigned long sector,
	const unsigned long sector_size)
{
	char run_cmd[64];
	ext4_printf("write raw data in %d size %d \n", sector, sector_size);
	sprintf(run_cmd, "mmc write 0x%lx 0x%lx 0x%lx",
		(const unsigned long)data, sector, sector_size);
	return run_command(run_cmd, 0);
}

int write_compressed_ext4(const void* img_base, unsigned long sector_base)
{
	unsigned int sector_size;
	int total_chunks;
	const ext4_chunk_header *chunk_header;
	const ext4_file_header *file_header;
	int err, retry;

	file_header = (const ext4_file_header*)img_base;
	total_chunks = file_header->total_chunks;

	ext4_printf("total chunk = %d\n", total_chunks);

	img_base += EXT4_FILE_HEADER_SIZE;

	while(total_chunks) {
		chunk_header = (const ext4_chunk_header*)img_base;
		sector_size = (chunk_header->chunk_size * file_header->block_size) >> BLK_SHIFT;

		switch(chunk_header->type)
		{
			case EXT4_CHUNK_TYPE_RAW:
				ext4_printf("raw_chunk\n");
				retry = 5;
				RETRY:
				err = write_raw_chunk(img_base + EXT4_CHUNK_HEADER_SIZE,
					sector_base, sector_size);
				if (err) {
					printf("[retry=%d] ext4 fs write fail !\n", retry--);
					if (retry > 0) goto RETRY;
					else return 1;
				}
				sector_base += sector_size;
				break;

			case EXT4_CHUNK_TYPE_FILL:
				ext4_printf("fill_chunk\n");
				sector_base += sector_size;
				break;

			case EXT4_CHUNK_TYPE_NONE:
				ext4_printf("none chunk\n");
				sector_base += sector_size;
				break;

			default:
				ext4_printf("unknown chunk type\n");
				sector_base += sector_size;
				break;
		}
		total_chunks--;
		ext4_printf("remain chunks = %d\n", total_chunks);

		img_base += chunk_header->total_size;
	};

	ext4_printf("write done\n");
	return 0;
}

//===========================
static int do_write_part_normal(const struct _part_info *part,
	const unsigned long buf, const unsigned long blk_size)
{
	char run_cmd[64];

	sprintf(run_cmd,"mmc write 0x%lx 0x%lx 0x%lx", buf,
		part->blk_start, blk_size);
	return run_command(run_cmd, 0);
}

static int do_write_part_boot(const struct _part_info *part,
	const unsigned long buf, const unsigned long blk_size)
{
	int ret;
	if(run_command("mmc lock open", 0)) return 1;
	ret = do_write_part_normal(part, buf, blk_size);
	run_command("mmc lock close", 0);
	return ret;
}

static int do_write_part(const struct _part_info *part,
	const unsigned long buf, const unsigned long blk_size)
{
	unsigned char flag = part->flag;

	if (!flag){
		return do_write_part_normal(part, buf, blk_size);
	}else if (flag & PART_FLAG_BOOTABLE){
		for(int i=0;i<20;i++)
		{
			printf("%02x",*((unsigned char *)buf+i));
		}
		return do_write_part_boot(part, buf, blk_size);
	}
	else if ((flag & PART_FLAG_EXT4) && !(flag & PART_FLAG_FAT)) {
		if (check_compress_ext4((void *)buf, part->blk_count))
			return CMD_RET_FAILURE;
		return write_compressed_ext4((void *)buf, part->blk_start);
	}

	return CMD_RET_FAILURE;
}

static int do_mmcpart_show(struct cmd_tbl *cmdtp,
int flag, int argc, char *const argv[])
{
	read_mbr_parts_info();
	show_parts_info();
	return CMD_RET_SUCCESS;
}

static const struct _part_info *mmcpart_find(const char* name)
{
	int i;
	for(i = uboot; i < PART_MAX; i++)
		if(!strcmp(parts[i].name, name)) {
			return &parts[i];
		}
	return NULL;
}

static int mmcpart_check_input(struct _input_argv *out,
	const int argc, char *const argv[])
{
	unsigned long size;

	if (argc < 2)
		return CMD_RET_USAGE;

	out->part = mmcpart_find(argv[1]);
	if (!out->part) {
		printf("can't find partition: %s\n"
			"please run \"mmcpart show\" ..\n", argv[1]);
		return CMD_RET_FAILURE;
	}

	if (argc > 2) {
		out->addr = simple_strtoul(argv[2], NULL, 16);
		if (!out->addr) {
			printf("addr can't be 0 !\n");
			return CMD_RET_FAILURE;
		}
	}

	out->blk_size = out->part->blk_count;
	if (argc > 3) {
		size = simple_strtoul(argv[3], NULL, 16);
		out->blk_size = (size >> BLK_SHIFT) + !!(size & (BLK_SIZE - 1));
		if (out->blk_size > out->part->blk_count) {
			printf("image size is larger than partition[%s]\n", out->part->name);
			return CMD_RET_FAILURE;
		}
		out->blk_size += (out->part->blk_count - out->blk_size) >> 1;
	}

	if (run_command("mmc dev 0", 0)) {
		printf("\ncannot switch to emmc dev !!\n");
		return -CMD_RET_FAILURE;
	}

	return CMD_RET_SUCCESS;
}

static int do_mmcpart_read(struct cmd_tbl *cmdtp,
	int flag, int argc, char *const argv[])
{
	struct _input_argv input;
	char cmd_read[64];
	int ret;

	if(argc < 3)
		return CMD_RET_USAGE;

	ret = mmcpart_check_input(&input, argc, argv);
	if (ret)
		return ret;

	sprintf(cmd_read, "mmc read %lx %lx %lx", input.addr,
		input.part->blk_start, input.blk_size);

	return run_command(cmd_read, 0);
}

static int do_mmcpart_write(struct cmd_tbl *cmdtp,
	int flag, int argc, char *const argv[])
{
	struct _input_argv input;
	int ret;

	if(argc < 3)
		return CMD_RET_USAGE;

	ret = mmcpart_check_input(&input, argc, argv);
	if (ret)
		return ret;

	return do_write_part(input.part, input.addr, input.blk_size);
}

static int do_mmcpart_erase(struct cmd_tbl *cmdtp,
	int flag, int argc, char *const argv[])
{
	struct _input_argv input;
	char cmd_erase[64];
	int ret;

	ret = mmcpart_check_input(&input, argc, argv);
	if (ret)
		return ret;

	sprintf(cmd_erase,"mmc erase %lx %lx",
		input.part->blk_start, input.part->blk_count);

	return run_command(cmd_erase, 0);
}

static struct cmd_tbl cmd_mmcpart[] = {
	U_BOOT_CMD_MKENT(show,  1, 0, do_mmcpart_show,  "", ""),
	U_BOOT_CMD_MKENT(read,  4, 0, do_mmcpart_read,  "", ""),
	U_BOOT_CMD_MKENT(write, 4, 0, do_mmcpart_write, "", ""),
	U_BOOT_CMD_MKENT(erase, 2, 0, do_mmcpart_erase, "", ""),
};

static int do_mmcpart(struct cmd_tbl *cmdtp,
	int flag, int argc, char *const argv[])
{
	struct cmd_tbl *cp;
	cp = find_cmd_tbl(argv[1], cmd_mmcpart, ARRAY_SIZE(cmd_mmcpart));

	/* Drop the mmcpart command */
	argc--;
	argv++;

	if (cp == NULL || argc > cp->maxargs)
		return CMD_RET_USAGE;
	if (flag == CMD_FLAG_REPEAT && !cmd_is_repeatable(cp))
		return CMD_RET_SUCCESS;

	return cp->cmd(cmdtp, flag, argc, argv);
}

U_BOOT_CMD(
	mmcpart, 5, 0, do_mmcpart,
	"mmcpart, itop4412 emmc partitions sub system",
	"mmcpart show - display all partitions of the itop4412 emmc\n"
	"mmcpart read  <part> <addr> [len] - read part's data to addr\n"
	"mmcpart write <part> <addr> [len] - write addr(len) to part\n"
	"mmcpart erase <part> - erase part's data"
);

