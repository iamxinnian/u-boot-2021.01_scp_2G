#include <common.h>
#include <command.h>
#include <mmc.h>
#include <env.h>
#include <stdlib.h>
#include <fastboot.h>

#define CFG_UPDATE_FILE_SRC_DEV		1

int do_write_mmc(char *mmcpart,char *filename)
{
	int ret = 1;
	ulong FileSize=0;
	char run_cmd[60]={0};

	sprintf(run_cmd,"fatsize mmc 1 %s",filename);
	ret=run_command(run_cmd,0);
	FileSize=env_get_hex("filesize",0);
	sprintf(run_cmd,"fatload mmc 1 0x60000000 %s",filename);
	ret |=run_command(run_cmd,0);
	if(FileSize == 0 || ret)
	{
		printf("file:%s not exist!\n",filename);
		return -1;
	}
	sprintf(run_cmd,"mmcpart write %s %lx %lx",mmcpart,(unsigned long)0x60000000,(unsigned long)FileSize);
	ret = run_command(run_cmd, 0);
	if(!ret) {
		printf("\npartition:%s write OK !!!\n",mmcpart);
	}
	return 0;
}

int do_sdfuse (struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	int ret = 1;
	struct mmc *mmc = NULL;
	char run_cmd[60]={0};

	mmc = find_mmc_device(CFG_UPDATE_FILE_SRC_DEV);
	if (mmc == NULL)
	{
		printf("*****************************\n");
		printf("NO Updating SD Card!\n");
		printf("*****************************\n");
		return -1;
	}else{
		ret = mmc_init(mmc);
		if (ret) {
			printf("*****************************\n");
			printf("NO Updating SD Card!\n");
			printf("*****************************\n");
			return ret;
		}
	}

	if (argc < 2)
	{
		return -1;
	}
	
	if ((argc == 2) && !strcmp(argv[1], "flashall"))
	{
		sprintf(run_cmd,"fdisk -c 0 10240 1024 512");
		ret |=run_command(run_cmd,0);
		sprintf(run_cmd,"fatformat mmc 0:1");
		ret |=run_command(run_cmd,0);
		for(int i=2;i<=4;i++)
		{
			sprintf(run_cmd,"ext4format mmc 0:%d",i);
			ret |=run_command(run_cmd,0);
		}

		do_write_mmc("bootloader","u-boot-iTOP-4412.bin");	
		do_write_mmc("kernel","uImage");
		do_write_mmc("dtb","exynos4412-itop-elite.dtb");
		do_write_mmc("system","system.img");
		return 0;
	}else if((argc == 4) && !strcmp(argv[1], "flash")){
		if((!strcmp(argv[2], "bootloader")) || (!strcmp(argv[2], "kernel")) || (!strcmp(argv[2], "dtb")) )
		{
			do_write_mmc(argv[2],argv[3]);
			return 0;
		}else if(!strcmp(argv[2], "system")){
			sprintf(run_cmd,"fatformat mmc 0:1");
			ret |=run_command(run_cmd,0);
			for(int i=2;i<=4;i++)
			{
				sprintf(run_cmd,"ext4format mmc 0:%d",i);
				ret |=run_command(run_cmd,0);
			}
			do_write_mmc(argv[2],argv[3]);
			return 0;
		}else{
			printf("command error!\n");
			return -1;
		}
	}else if((argc == 3) && !strcmp(argv[1], "erase"))
	{
		if(!strcmp(argv[2], "fat"))
		{
			sprintf(run_cmd,"ext4format mmc 0:1");
			run_command(run_cmd,0);
			return 0;
		}else if(!strcmp(argv[2], "system"))
		{
			sprintf(run_cmd,"ext4format mmc 0:2");
			run_command(run_cmd,0);
			return 0;
		}else if(!strcmp(argv[2], "userdata"))
		{
			sprintf(run_cmd,"ext4format mmc 0:3");
			run_command(run_cmd,0);
			return 0;
		}else if(!strcmp(argv[2], "cache"))
		{
			sprintf(run_cmd,"ext4format mmc 0:4");
			run_command(run_cmd,0);
			return 0;
		}else{
			printf("command error!\n");
			return -1;
		}
	}else{
		printf("command error!\n");
		return -1;
	}
	return -1;	
}

U_BOOT_CMD(
	sdfuse,	4,	1,	do_sdfuse,
	"sdfuse  - read images from FAT partition of SD card and write them to booting device.",
	"sdfuse flashall                         - flash boot.img, system.img,\n"
	"                                          erase userdata, cache, and reboot.\n"
	"sdfuse flash <partition> [ <filename> ] - write a file to a partition.\n"
	"sdfuse erase <partition>                - erase (format) a partition."
);
