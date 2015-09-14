/* linux/arch/arm/mach-s5pv210/mach-smdkv210.c
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/serial_core.h>
#include <linux/device.h>
#include <linux/dm9000.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/pwm_backlight.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/leds.h>
#include <linux/w1-gpio.h>

#include <asm/hardware/vic.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/setup.h>
#include <asm/mach-types.h>

#include <video/platform_lcd.h>

#include <mach/map.h>
#include <mach/regs-clock.h>

#include <plat/regs-serial.h>
#include <plat/regs-srom.h>
#include <plat/gpio-cfg.h>
#include <plat/devs.h>
#include <plat/cpu.h>
#include <plat/adc.h>
#include <plat/ts.h>
#include <plat/ata.h>
#include <plat/iic.h>
#include <plat/keypad.h>
#include <plat/pm.h>
#include <plat/fb.h>
#include <plat/s5p-time.h>
#include <plat/backlight.h>
#include <plat/regs-fb-v4.h>
#include <plat/mfc.h>
#include <plat/sdhci.h>
#include <plat/ehci.h>

#include "common.h"

/* Following are default values for UCON, ULCON and UFCON UART registers */
#define SMDKV210_UCON_DEFAULT	(S3C2410_UCON_TXILEVEL |	\
				 S3C2410_UCON_RXILEVEL |	\
				 S3C2410_UCON_TXIRQMODE |	\
				 S3C2410_UCON_RXIRQMODE |	\
				 S3C2410_UCON_RXFIFO_TOI |	\
				 S3C2443_UCON_RXERR_IRQEN)

#define SMDKV210_ULCON_DEFAULT	S3C2410_LCON_CS8

#define SMDKV210_UFCON_DEFAULT	(S3C2410_UFCON_FIFOMODE |	\
				 S5PV210_UFCON_TXTRIG4 |	\
				 S5PV210_UFCON_RXTRIG4)

static struct s3c2410_uartcfg smdkv210_uartcfgs[] __initdata = {
	[0] = {
		.hwport		= 0,
		.flags		= 0,
		.ucon		= SMDKV210_UCON_DEFAULT,
		.ulcon		= SMDKV210_ULCON_DEFAULT,
		.ufcon		= SMDKV210_UFCON_DEFAULT,
	},
	[1] = {
		.hwport		= 1,
		.flags		= 0,
		.ucon		= SMDKV210_UCON_DEFAULT,
		.ulcon		= SMDKV210_ULCON_DEFAULT,
		.ufcon		= SMDKV210_UFCON_DEFAULT,
	},
	[2] = {
		.hwport		= 2,
		.flags		= 0,
		.ucon		= SMDKV210_UCON_DEFAULT,
		.ulcon		= SMDKV210_ULCON_DEFAULT,
		.ufcon		= SMDKV210_UFCON_DEFAULT,
	},
	[3] = {
		.hwport		= 3,
		.flags		= 0,
		.ucon		= SMDKV210_UCON_DEFAULT,
		.ulcon		= SMDKV210_ULCON_DEFAULT,
		.ufcon		= SMDKV210_UFCON_DEFAULT,
	},
};

static struct resource smdkv210_dm9000_resources[] = {
	[0] = {
		.start	= S5PV210_PA_SROM_BANK1 + 0x300,
		.end	= S5PV210_PA_SROM_BANK1 + 0x300,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= S5PV210_PA_SROM_BANK1 + 0x300 + 4,
		.end	= S5PV210_PA_SROM_BANK1 + 0x300 + 4,
		.flags	= IORESOURCE_MEM,
	},
	[2] = {
		.start	= IRQ_EINT(10),
		.end	= IRQ_EINT(10),
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL,
	},
};

static struct dm9000_plat_data smdkv210_dm9000_platdata = {
	.flags		= DM9000_PLATF_16BITONLY | DM9000_PLATF_NO_EEPROM,
	.dev_addr	= { 0x00, 0x09, 0xc0, 0xff, 0xec, 0x48 },
};

static struct platform_device smdkv210_dm9000 = {
	.name		= "dm9000",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(smdkv210_dm9000_resources),
	.resource	= smdkv210_dm9000_resources,
	.dev		= {
		.platform_data	= &smdkv210_dm9000_platdata,
	},
};

static struct gpio_keys_button gpio_buttons[] = {
	{
		.gpio		= S5PV210_GPH0(0),
		.code		= KEY_UP,
		.desc		= "up",
		.active_low	= 1,
	},
	{
		.gpio		= S5PV210_GPH0(1),
		.code		= KEY_DOWN,
		.desc		= "down",
		.active_low	= 1,
	},	
	{
		.gpio		= S5PV210_GPH0(2),
		.code		= KEY_LEFT,
		.desc		= "left",
		.active_low	= 1,
	},
	{
		.gpio		= S5PV210_GPH0(3),
		.code		= KEY_RIGHT,
		.desc		= "right",
		.active_low	= 1,
	},
	{
		.gpio		= S5PV210_GPH0(4),
		.code		= KEY_ENTER,
		.desc		= "enter",
		.active_low	= 1,
	},		
	{
		.gpio		= S5PV210_GPH0(5),
		.code		= KEY_ESC,
		.desc		= "esc",
		.active_low	= 1,
	},
	{
		.gpio		= S5PV210_GPH2(6),
		.code		= KEY_VOLUMEUP,
		.desc		= "vol+",
		.active_low	= 1,
	},
	{
		.gpio		= S5PV210_GPH2(7),
		.code		= KEY_VOLUMEDOWN,
		.desc		= "vol-",
		.active_low	= 1,
	},
};
static struct gpio_keys_platform_data gpio_button_data = {
	.buttons	= gpio_buttons,
	.nbuttons	= ARRAY_SIZE(gpio_buttons),
};
static struct platform_device s3c_device_gpio_button = {
	.name		= "gpio-keys",
	.id		= -1,
	.dev		= {
		.platform_data	= &gpio_button_data,
	}
};

static struct gpio_led tq210_leds[] = {
	[0] = {
		.name			= "led1",
		.default_trigger	= "heartbeat",
		.gpio			= S5PV210_GPC0(3),
		.active_low		= 0,
		.default_state	= LEDS_GPIO_DEFSTATE_OFF,
	},
	[1] = {
		.name			= "led2",
		.default_trigger	= "timer",
		.gpio			= S5PV210_GPC0(4),
		.active_low		= 0,
		.default_state	= LEDS_GPIO_DEFSTATE_OFF,
	},
};
static struct gpio_led_platform_data tq210_gpio_led_data = {
	.leds		= tq210_leds,
	.num_leds	= ARRAY_SIZE(tq210_leds),
};
static struct platform_device tq210_device_led= {
       .name	= "leds-gpio",
       .id		= -1,
       .dev		= {
		.platform_data = &tq210_gpio_led_data,
       },
};

/* DS18B20 */
static void w1_enable_external_pullup(int enable)
{
	if (enable)
		s3c_gpio_setpull(S5PV210_GPH1(0), S3C_GPIO_PULL_UP);
	else
		s3c_gpio_setpull(S5PV210_GPH1(0), S3C_GPIO_PULL_NONE);
}
static struct w1_gpio_platform_data ds18b20_w1_gpio = {
	.pin = S5PV210_GPH1(0),
	.is_open_drain = 0,
	.enable_external_pullup = w1_enable_external_pullup,
};
static struct platform_device smdkv210_ds18b20_device = {
        .name     = "w1-gpio",
        .id       = -1,
        .dev      = {
			.platform_data  = &ds18b20_w1_gpio,
        },
};

static void smdkv210_lte480wv_set_power(struct plat_lcd_data *pd,
					unsigned int power)
{
	if (power) {
#if !defined(CONFIG_BACKLIGHT_PWM)
		gpio_request_one(S5PV210_GPD0(0), GPIOF_OUT_INIT_HIGH, "GPD0");
		gpio_free(S5PV210_GPD0(0));
#endif
	} else {
#if !defined(CONFIG_BACKLIGHT_PWM)
		gpio_request_one(S5PV210_GPD0(0), GPIOF_OUT_INIT_LOW, "GPD0");
		gpio_free(S5PV210_GPD0(0));
#endif
	}
}

static struct plat_lcd_data smdkv210_lcd_lte480wv_data = {
	.set_power	= smdkv210_lte480wv_set_power,
};

static struct platform_device smdkv210_lcd_lte480wv = {
	.name			= "platform-lcd",
	.dev.parent		= &s3c_device_fb.dev,
	.dev.platform_data	= &smdkv210_lcd_lte480wv_data,
};

/* Buzzer */
static struct platform_device xc2440_beeper_device = {
	.name		= "pwm-beeper",
	.dev		= {
		.parent 		= &s3c_device_timer[1].dev,
		.platform_data 	= (void *)1,	//pwm通道
	},
	.id = 1,
};

static struct platform_device *smdkv210_devices[] __initdata = {
	&smdkv210_dm9000,
	&s3c_device_hsmmc0,
	&s3c_device_i2c0,
	&s3c_device_gpio_button,
	&tq210_device_led,
	&s5p_device_ehci,
	&smdkv210_ds18b20_device,
	&s3c_device_fb,
	&smdkv210_lcd_lte480wv,
	&s3c_device_timer[1],
	&xc2440_beeper_device,
};

static void __init smdkv210_map_io(void)
{
	s5pv210_init_io(NULL, 0);
	s3c24xx_init_clocks(24000000);
	s3c24xx_init_uarts(smdkv210_uartcfgs, ARRAY_SIZE(smdkv210_uartcfgs));
	s5p_set_timer_source(S5P_PWM2, S5P_PWM4);
}

static void __init smdkv210_reserve(void)
{
	s5p_mfc_reserve_mem(0x43000000, 8 << 20, 0x51000000, 8 << 20);
}

static void __init smdkv210_dm9000_init(void)
{
	unsigned int tmp;

	gpio_request(S5PV210_MP01(1), "nCS1");
	s3c_gpio_cfgpin(S5PV210_MP01(1), S3C_GPIO_SFN(2));
	gpio_free(S5PV210_MP01(1));

	tmp = (5 << S5P_SROM_BCX__TACC__SHIFT);
	__raw_writel(tmp, S5P_SROM_BC1);

	tmp = __raw_readl(S5P_SROM_BW);
	tmp &= (S5P_SROM_BW__CS_MASK << S5P_SROM_BW__NCS1__SHIFT);
	tmp |= (1 << S5P_SROM_BW__NCS1__SHIFT);
	__raw_writel(tmp, S5P_SROM_BW);
}

static struct s3c_sdhci_platdata smdkv210_hsmmc0_data __initdata = {
	.max_width		= 4,
	.cd_type		= S3C_SDHCI_CD_INTERNAL,
};

static struct i2c_board_info smdkv210_i2c_devs0[] __initdata = {
	{ I2C_BOARD_INFO("24c02", 0x50), },
};

static struct s5p_ehci_platdata smdkv210_ehci_pdata;

static struct s3c_fb_pd_win smdkv210_fb_win0 = {
	.win_mode = {
		.left_margin	= 26,	//h_bp
		.right_margin	= 210,//h_fp
		.upper_margin	= 13,	//v_bp
		.lower_margin	= 22,	//v_fp
		.hsync_len	= 20,
		.vsync_len	= 10,
		.xres		= 800,
		.yres		= 480,
	},
	.max_bpp	= 32,
	.default_bpp	= 24,
};
static struct s3c_fb_pd_win smdkv210_fb_win1 = {
	.win_mode = {
		.left_margin	= 26,	//h_bp
		.right_margin	= 210,//h_fp
		.upper_margin	= 13,	//v_bp
		.lower_margin	= 22,	//v_fp
		.hsync_len	= 20,
		.vsync_len	= 10,
		.xres		= 800,
		.yres		= 480,
	},
	.max_bpp	= 32,
	.default_bpp	= 24,
};
static struct s3c_fb_pd_win smdkv210_fb_win2 = {
	.win_mode = {
		.left_margin	= 26,	//h_bp
		.right_margin	= 210,//h_fp
		.upper_margin	= 13,	//v_bp
		.lower_margin	= 22,	//v_fp
		.hsync_len	= 20,
		.vsync_len	= 10,
		.xres		= 800,
		.yres		= 480,
	},
	.max_bpp	= 32,
	.default_bpp	= 24,
};
static struct s3c_fb_pd_win smdkv210_fb_win3 = {
	.win_mode = {
		.left_margin	= 26,	//h_bp
		.right_margin	= 210,//h_fp
		.upper_margin	= 13,	//v_bp
		.lower_margin	= 22,	//v_fp
		.hsync_len	= 20,
		.vsync_len	= 10,
		.xres		= 800,
		.yres		= 480,
	},
	.max_bpp	= 32,
	.default_bpp	= 24,
};
static struct s3c_fb_pd_win smdkv210_fb_win4 = {
	.win_mode = {
		.left_margin	= 26,	//h_bp
		.right_margin	= 210,//h_fp
		.upper_margin	= 13,	//v_bp
		.lower_margin	= 22,	//v_fp
		.hsync_len	= 20,
		.vsync_len	= 10,
		.xres		= 800,
		.yres		= 480,
	},
	.max_bpp	= 32,
	.default_bpp	= 24,
};

static struct s3c_fb_platdata smdkv210_lcd0_pdata __initdata = {
	.win[0]		= &smdkv210_fb_win0,
	.win[1]		= &smdkv210_fb_win1,
	.win[2]		= &smdkv210_fb_win2,
	.win[3]		= &smdkv210_fb_win3,
	.win[4]		= &smdkv210_fb_win4,
	.vidcon0	= (4<<6)|(1<<4),//33.35MHz
	.vidcon1	= (1<<6)|(1<<5),
	.setup_gpio	= s5pv210_fb_gpio_setup_24bpp,
};

/* LCD Backlight data */
static struct samsung_bl_gpio_info smdkv210_bl_gpio_info = {
	.no = S5PV210_GPD0(0),
	.func = S3C_GPIO_SFN(2),
};

static struct platform_pwm_backlight_data smdkv210_bl_data = {
	.pwm_id = 0,
	.pwm_period_ns	= 1000,
};

static int __init smdkv210_pwm_beeper_init(void)
{
	int ret;

	ret = gpio_request(S5PV210_GPD0(1), "beeper");
	if (ret) {
		printk(KERN_ERR "failed to request GPD for PWM-OUT 1\n");
		return ret;
	}

	// Configure GPIO pin with S5PV210_GPD_0_1_TOUT_3 
	s3c_gpio_cfgpin(S5PV210_GPD0(1), S3C_GPIO_SFN(2));

	gpio_free(S5PV210_GPD0(1));

	return 0;
}

static void __init smdkv210_machine_init(void)
{
	smdkv210_dm9000_init();
	s3c_sdhci0_set_platdata(&smdkv210_hsmmc0_data);
	s3c_i2c0_set_platdata(NULL);
	i2c_register_board_info(0, smdkv210_i2c_devs0,
			ARRAY_SIZE(smdkv210_i2c_devs0));
	s5p_ehci_set_platdata(&smdkv210_ehci_pdata);
	s3c_fb_set_platdata(&smdkv210_lcd0_pdata);
	samsung_bl_set(&smdkv210_bl_gpio_info, &smdkv210_bl_data);
	smdkv210_pwm_beeper_init();
	
	platform_add_devices(smdkv210_devices, ARRAY_SIZE(smdkv210_devices));
}

MACHINE_START(SMDKV210, "SMDKV210")
	/* Maintainer: Kukjin Kim <kgene.kim@samsung.com> */
	.atag_offset	= 0x100,
	.init_irq	= s5pv210_init_irq,
	.handle_irq	= vic_handle_irq,
	.map_io		= smdkv210_map_io,
	.init_machine	= smdkv210_machine_init,
	.timer		= &s5p_timer,
	.restart	= s5pv210_restart,
	.reserve	= &smdkv210_reserve,
MACHINE_END
