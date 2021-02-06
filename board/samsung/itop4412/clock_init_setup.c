/*
 * Clock Initialization for board based on EXYNOS4210
 *
 * Copyright (C) 2013 Samsung Electronics
 * Rajeshwari Shinde <rajeshwari.s@samsung.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <config.h>
#include <asm/io.h>
#include <asm/arch/cpu.h>
#include <asm/arch/clk.h>
#include <asm/arch/clock.h>
#include "clock_init_setup.h"

unsigned char MUX_STAT_Check(unsigned int *addr,unsigned char n)
{
	unsigned int value=readl(addr);
	if(value & (0x1 << n))
	{
		return 1;
	}else{
		return 0;
	}
}
/*
 * system_clock_init: Initialize core clock and bus clock.
 * void system_clock_init(void)
 */
void system_clock_init(void)
{
	struct exynos4x12_clock *clk =
		(struct exynos4x12_clock *)samsung_get_base_clock();
	/*Step1 Set PLL locktime */
	writel(APLL_LOCKTIME, &clk->apll_lock);
	writel(MPLL_LOCKTIME, &clk->mpll_lock);
	writel(EPLL_LOCKTIME, &clk->epll_lock);
	writel(VPLL_LOCKTIME, &clk->vpll_lock);

	/*Step2 Set PLL value */
	writel(APLL_CON1_VAL, &clk->apll_con1);
	writel(APLL_CON0_VAL, &clk->apll_con0);
	writel(MPLL_CON1_VAL, &clk->mpll_con1);
	writel(MPLL_CON0_VAL, &clk->mpll_con0);
	writel(EPLL_CON1_VAL, &clk->epll_con1);
	writel(EPLL_CON0_VAL, &clk->epll_con0);
	writel(VPLL_CON1_VAL, &clk->vpll_con1);
	writel(VPLL_CON0_VAL, &clk->vpll_con0);

	/*Step3 Wait locktime */
	while(!(readl(&clk->apll_con0) & (0x1<<29))) continue;
	while(!(readl(&clk->mpll_con0) & (0x1<<29))) continue;
	while(!(readl(&clk->epll_con0) & (0x1<<29))) continue;
	while(!(readl(&clk->vpll_con0) & (0x1<<29))) continue;

	/*Step4 Set DIV value */
	writel(CLK_DIV_CPU0_VAL, &clk->div_cpu0);
	writel(CLK_DIV_CPU1_VAL, &clk->div_cpu1);
	writel(CLK_DIV_DMC0_VAL, &clk->div_dmc0);
	writel(CLK_DIV_DMC1_VAL, &clk->div_dmc1);
	writel(CLK_DIV_LEFTBUS_VAL, &clk->div_leftbus);
	writel(CLK_DIV_RIGHTBUS_VAL, &clk->div_rightbus);
	writel(CLK_DIV_TOP_VAL, &clk->div_top);
	writel(CLK_DIV_FSYS0_VAL, &clk->div_fsys0);
	writel(CLK_DIV_FSYS1_VAL, &clk->div_fsys1);
	writel(CLK_DIV_FSYS2_VAL, &clk->div_fsys2);
	writel(CLK_DIV_FSYS3_VAL, &clk->div_fsys3);
	writel(CLK_DIV_PERIL0_VAL, &clk->div_peril0);
	writel(CLK_DIV_PERIL1_VAL, &clk->div_peril1);
	writel(CLK_DIV_PERIL2_VAL, &clk->div_peril2);
	writel(CLK_DIV_PERIL3_VAL, &clk->div_peril3);
	writel(CLK_DIV_PERIL4_VAL, &clk->div_peril4);
	writel(CLK_DIV_PERIL5_VAL, &clk->div_peril5);
	writel(CLK_DIV_CAM_VAL, &clk->div_cam);
	writel(CLK_DIV_CAM1_VAL, &clk->div_cam1);
	writel(CLK_DIV_MFC_VAL, &clk->div_mfc);
	writel(CLK_DIV_G3D_VAL, &clk->div_g3d);

	/*Step5 Wait DIV stable */
	while(readl(&clk->div_stat_cpu0) != 0) continue;
	while(readl(&clk->div_stat_cpu1) != 0) continue;
	while(readl(&clk->div_stat_dmc0)!= 0) continue;
	while(readl(&clk->div_stat_dmc1) != 0) continue;
	while(readl(&clk->div_stat_leftbus) != 0) continue;
	while(readl(&clk->div_stat_rightbus) != 0) continue;
	while(readl(&clk->div_stat_top) != 0) continue;
	while(readl(&clk->div_stat_fsys0) != 0) continue;
	while(readl(&clk->div_stat_fsys1) != 0) continue;
	while(readl(&clk->div_stat_fsys2) != 0) continue;
	while(readl(&clk->div_stat_fsys3) != 0) continue;
	while(readl(&clk->div_stat_peril0) != 0) continue;
	while(readl(&clk->div_stat_peril1) != 0) continue;
	while(readl(&clk->div_stat_peril2) != 0) continue;
	while(readl(&clk->div_stat_peril3) != 0) continue;
	while(readl(&clk->div_stat_peril4) != 0) continue;
	while(readl(&clk->div_stat_peril5) != 0) continue;
	while(readl(&clk->div_stat_cam) != 0) continue;
	while(readl(&clk->div_stat_cam1) != 0) continue;
	while(readl(&clk->div_stat_mfc) != 0) continue;
	while(readl(&clk->div_stat_g3d) != 0) continue;
	while(readl(&clk->div_stat_lcd) != 0) continue;
	while(readl(&clk->div_stat_tv) != 0) continue;
	while(readl(&clk->div_stat_maudio) != 0) continue;
	while(readl(&clk->div_stat_isp) != 0) continue;
	while(readl(&clk->div_stat_isp0) != 0) continue;
	while(readl(&clk->div_stat_isp1) != 0) continue;

	/*Step6 Set MUX value */
	writel(CLK_SRC_CPU_VAL, &clk->src_cpu);
	writel(CLK_SRC_TOP0_VAL, &clk->src_top0);
	writel(CLK_SRC_TOP1_VAL, &clk->src_top1);
	writel(CLK_SRC_DMC_VAL, &clk->src_dmc);
	writel(CLK_SRC_LEFTBUS_VAL, &clk->src_leftbus);
	writel(CLK_SRC_RIGHTBUS_VAL, &clk->src_rightbus);
	writel(CLK_SRC_FSYS_VAL, &clk->src_fsys);
	writel(CLK_SRC_PERIL0_VAL, &clk->src_peril0);
	writel(CLK_SRC_PERIL1_VAL, &clk->src_peril1);
	writel(CLK_SRC_CAM_VAL, &clk->src_cam);
	writel(CLK_SRC_CAM1_VAL, &clk->src_cam1);
	writel(CLK_SRC_MFC_VAL, &clk->src_mfc);
	writel(CLK_SRC_G3D_VAL, &clk->src_g3d);
	writel(CLK_SRC_LCD_VAL, &clk->src_lcd);
	writel(CLK_SRC_TV_VAL, &clk->src_tv);
	writel(CLK_SRC_MAUDIO_VAL, &clk->src_maudio);
	writel(CLK_SRC_ISP_VAL, &clk->src_isp);

	/* Step7 Wait MUX stable */
	while(MUX_STAT_Check(&clk->mux_stat_cpu,2)
			| MUX_STAT_Check(&clk->mux_stat_cpu,18)
			| MUX_STAT_Check(&clk->mux_stat_cpu,22)
			| MUX_STAT_Check(&clk->mux_stat_cpu,26))
		continue;
	while(MUX_STAT_Check(&clk->mux_stat_top0,2)
			| MUX_STAT_Check(&clk->mux_stat_top0,6)
			| MUX_STAT_Check(&clk->mux_stat_top0,10)
			| MUX_STAT_Check(&clk->mux_stat_top0,14)
			| MUX_STAT_Check(&clk->mux_stat_top0,18)
			| MUX_STAT_Check(&clk->mux_stat_top0,22)
			| MUX_STAT_Check(&clk->mux_stat_top0,26)
			| MUX_STAT_Check(&clk->mux_stat_top0,30))
		continue;
	while(MUX_STAT_Check(&clk->mux_stat_top1,6)
			| MUX_STAT_Check(&clk->mux_stat_top1,10)
			| MUX_STAT_Check(&clk->mux_stat_top1,14)
			| MUX_STAT_Check(&clk->mux_stat_top1,18)
			| MUX_STAT_Check(&clk->mux_stat_top1,22)
			| MUX_STAT_Check(&clk->mux_stat_top1,26))
		continue;
	while(MUX_STAT_Check(&clk->mux_stat_top0,2)
			| MUX_STAT_Check(&clk->mux_stat_dmc,6)
			| MUX_STAT_Check(&clk->mux_stat_dmc,10)
			| MUX_STAT_Check(&clk->mux_stat_dmc,14)
			| MUX_STAT_Check(&clk->mux_stat_dmc,22)
			| MUX_STAT_Check(&clk->mux_stat_dmc,26)
			| MUX_STAT_Check(&clk->mux_stat_dmc,30))
		continue;
	while(MUX_STAT_Check(&clk->mux_stat_leftbus,2)
			| MUX_STAT_Check(&clk->mux_stat_leftbus,6))
		continue;
	while(MUX_STAT_Check(&clk->mux_stat_rightbus,2)
			| MUX_STAT_Check(&clk->mux_stat_rightbus,6))
		continue;
	while(MUX_STAT_Check(&clk->mux_stat_cam1,2)
			| MUX_STAT_Check(&clk->mux_stat_cam1,6)
			| MUX_STAT_Check(&clk->mux_stat_cam1,10))
		continue;
	while(MUX_STAT_Check(&clk->mux_stat_mfc,2)
			| MUX_STAT_Check(&clk->mux_stat_mfc,6)
			| MUX_STAT_Check(&clk->mux_stat_mfc,10))
		continue;
	while(MUX_STAT_Check(&clk->mux_stat_g3d,2)
			| MUX_STAT_Check(&clk->mux_stat_g3d,6)
			| MUX_STAT_Check(&clk->mux_stat_g3d,10))
		continue;
}
