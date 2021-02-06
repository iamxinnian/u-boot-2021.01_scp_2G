// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2011 Samsung Electronics
 */

#include <common.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <asm/arch/cpu.h>
#include <asm/arch/mmc.h>
#include <asm/arch/periph.h>
#include <asm/arch/pinmux.h>
#include <usb.h>
#include <usb/dwc2_udc.h>
#include <asm/arch/clock.h>
#include <asm/arch/power.h>

void system_clock_init(void);
u32 get_board_rev(void)
{
	return 0;
}

static void exynos4412_set_ps_hold_ctrl(void)
{
	unsigned int *ONO=(unsigned int *)0x11000c08;
	struct exynos4x12_power *power =
	(struct exynos4x12_power *)samsung_get_base_power();

	/* Set PS-Hold high */
	unsigned int value = readl(&power->ps_hold_control);
	value |= (0x1<<8);
	writel(value, &power->ps_hold_control);
	value = readl(ONO);
	value |=(0x3<<4);
	writel(value, ONO);
	writel(0, &power->mask_wdt_reset_request);
	return;
}

int exynos_init(void)
{
#ifdef CONFIG_CMD_USB
	/* USB3503A Connect */
	gpio_request(EXYNOS4X12_GPIO_M33, "USB3503A Connect");

	/* USB3503A Reset */
	gpio_request(EXYNOS4X12_GPIO_M24, "USB3503A Reset");

	/* USB3503A Reference frequency */
	gpio_request(EXYNOS4X12_GPIO_C01, "DM9621");
#endif
	return 0;
}

static int s5pc210_phy_control(int on)
{
	return 0;
}

struct dwc2_plat_otg_data s5pc210_otg_data = {
	.phy_control	= s5pc210_phy_control,
	.regs_phy	= EXYNOS4X12_USBPHY_BASE,
	.regs_otg	= EXYNOS4X12_USBOTG_BASE,
	.usb_phy_ctrl	= EXYNOS4X12_USBPHY_CONTROL,
	.usb_flags	= PHY0_SLEEP,
};

int board_usb_init(int index, enum usb_init_type init)
{
	gpio_direction_output(EXYNOS4X12_GPIO_M33, 0);
	gpio_direction_output(EXYNOS4X12_GPIO_C01, 0);
	gpio_direction_output(EXYNOS4X12_GPIO_M24, 0);
	gpio_direction_output(EXYNOS4X12_GPIO_M24, 1);
	gpio_direction_output(EXYNOS4X12_GPIO_C01, 1);
	gpio_direction_output(EXYNOS4X12_GPIO_M33, 1);
	return dwc2_udc_probe(&s5pc210_otg_data);
}

#ifdef CONFIG_BOARD_EARLY_INIT_F
int exynos_early_init_f(void)
{
	exynos4412_set_ps_hold_ctrl();
	system_clock_init();
	return 0;
}
#endif
