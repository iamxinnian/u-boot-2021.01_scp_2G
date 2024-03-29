/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2011 Samsung Electronics
 *
 * Configuration settings for the SAMSUNG ITOP4412 (EXYNOS4210) board.
 */

#ifndef __CONFIG_ITOP4412_H
#define __CONFIG_ITOP4412_H

#include <configs/exynos4-common.h>

/* High Level Configuration Options */
#define CONFIG_EXYNOS4412		1	/* which is a EXYNOS4412 SoC */
#define CONFIG_ITOP4412			1	/* working with ITOP4412*/
#define CONFIG_SUPPORT_EMMC_BOOT	1
#define CONFIG_CPU_FREQ_1400KHZ		1

/* ITOP4412 has 4 bank of DRAM */
#define CONFIG_SYS_SDRAM_BASE		0x40000000
#define PHYS_SDRAM_1			CONFIG_SYS_SDRAM_BASE
#define SDRAM_BANK_SIZE			(512 << 20)	/* 512 MB */

/* memtest works on */
#define CONFIG_SYS_LOAD_ADDR		(CONFIG_SYS_SDRAM_BASE + 0x3E00000)

#define CONFIG_MACH_TYPE		MACH_TYPE_ITOP4412

#define CONFIG_SYS_MEM_TOP_HIDE	(1 << 20)	/* ram console */

#define CONFIG_SYS_MONITOR_BASE	0x00000000
#define CONFIG_SYS_BOOTMAPSZ    0x10000000

/* Power Down Modes */
#define S5P_CHECK_SLEEP			0x00000BAD
#define S5P_CHECK_DIDLE			0xBAD00000
#define S5P_CHECK_LPA			0xABAD0000

/* MMC SPL */
#define COPY_BL2_FNPTR_ADDR	0x02020030

#define CONFIG_EXTRA_ENV_SETTINGS \
	"loadaddr=0x40007000\0" \
	"rdaddr=0x48000000\0" \
	"kerneladdr=0x40007000\0" \
	"dtbaddr=0x41000000\0" \
	"ramdiskaddr=0x48000000\0" \
	"fastbootbuf=" __stringify(CONFIG_FASTBOOT_BUF_ADDR) "\0" \
	"console=ttySAC2,115200n8\0" \
	"mmcdev=0\0" \
	"bootenv=uEnv.txt\0" \
	"loadbootenv=load mmc ${mmcdev} ${loadaddr} ${bootenv}\0" \
	"importbootenv=echo Importing environment from mmc ...; " \
		"env import -t $loadaddr $filesize\0" \
        	"loadbootscript=load mmc ${mmcdev} ${loadaddr} boot.scr\0" \
        	"bootscript=echo Running bootscript from mmc${mmcdev} ...; " \
                	"source ${loadaddr}\0"
#define CONFIG_BOOTCOMMAND \
	"if mmc rescan; then " \
		"echo SD/MMC found on device ${mmcdev};" \
		"if run loadbootenv; then " \
			"echo Loaded environment from ${bootenv};" \
			"run importbootenv;" \
		"fi;" \
		"if test -n $uenvcmd; then " \
			"echo Running uenvcmd ...;" \
			"run uenvcmd;" \
		"fi;" \
		"if run loadbootscript; then " \
			"run bootscript; " \
		"fi; " \
	"fi;" \
	"mmcpart read kernel ${kerneladdr};" \
	"mmcpart read dtb ${dtbaddr}; bootm ${kerneladdr} - ${dtbaddr}"
#define CONFIG_CLK_1000_400_200

/* MIU (Memory Interleaving Unit) */
#define CONFIG_MIU_2BIT_21_7_INTERLEAVED

#define RESERVE_BLOCK_SIZE		(512)
#define BL1_SIZE			(15 << 10) /*15 K reserved for BL1*/
#define SPL_SIZE			(16 << 10) /*16 K reserved for BL2*/

#define CONFIG_SPL_MAX_FOOTPRINT	(14 * 1024)

#define CONFIG_SPL_STACK			0x02060000
#define CONFIG_SYS_INIT_SP_ADDR	(CONFIG_SYS_LOAD_ADDR \
					- GENERATED_GBL_DATA_SIZE)

/* U-Boot copy size from boot Media to DRAM.*/
#define COPY_BL2_SIZE		0x52000
#define BL2_START_OFFSET	((RESERVE_BLOCK_SIZE+BL1_SIZE+SPL_SIZE)/512)
#define BL2_SIZE_BLOC_COUNT	(COPY_BL2_SIZE/512)

/* USB HUB */
#define CONFIG_USB_EHCI_EXYNOS
#endif	/* __CONFIG_H */
