/*
 * linux/arch/arm/mach-exynos4/mach-smdk4x12.c
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/lcd.h>
#include <linux/mfd/max8997.h>
#include <linux/mfd/max77686.h>
#include <linux/mmc/host.h>
#include <linux/platform_device.h>
#include <linux/pwm_backlight.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/mfd/wm8994/pdata.h>
#include <linux/serial_core.h>
#include <linux/lcd.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_gpio.h>
#include <linux/input/pixcir_ts.h>
#include <linux/gpio_event.h>
#include <linux/platform_data/s3c-hsotg.h>
#include <linux/platform_data/exynos_thermal.h>
#include <linux/mfd/samsung/s5m8767.h>
#include <linux/mfd/samsung/core.h>

#include <asm/mach/arch.h>
#include <asm/hardware/gic.h>
#include <asm/mach-types.h>

#include <plat/backlight.h>
#include <plat/clock.h>
#include <plat/cpu.h>
#include <plat/devs.h>
#include <plat/gpio-cfg.h>
#include <plat/adc.h>
#include <plat/adc-core.h>
#include <plat/iic.h>
#include <plat/keypad.h>
#include <plat/mfc.h>
#include <plat/regs-serial.h>
#include <plat/sdhci.h>
#include <plat/regs-fb-v4.h>
#include <plat/fb.h>
#include <plat/pm.h>
#include <plat/hdmi.h>
#include <plat/ehci.h>
#include <plat/camport.h>
#include <plat/s3c64xx-spi.h>
#include <plat/fimg2d.h>

#ifdef CONFIG_EXYNOS4_DEV_DWMCI
#include <mach/dwmci.h>
#endif

#include <mach/map.h>
#include <mach/exynos-ion.h>
#include <mach/regs-pmu.h>
#include <mach/ohci.h>
#include <mach/ppmu.h>
#include <mach/dev.h>
#include <mach/s3cfb.h>

#include <media/v4l2-mediabus.h>
#include <media/s5p_fimc.h>
#include <media/m5mols.h>
#include <plat/mipi_csis.h>
#include <plat/camport.h>
#include <media/exynos_fimc_is.h>
#include "common.h"
#include <media/exynos_flite.h>

#ifdef CONFIG_KEYBOARD_GPIO
#include <linux/gpio_keys.h>
#endif

#ifdef CONFIG_SAMSUNG_DEV_KEYPAD
#include <plat/keypad.h>
#endif

/* add by cym 20121114 */
#ifdef CONFIG_VIDEO_SR200PC20
#include <media/sr200pc20_platform.h>
#endif
/* end add */

/* add by cym 20130605 */
//#ifdef CONFIG_VIDEO_OV5640 CONFIG_SOC_CAMERA_GC2035
#if defined(CONFIG_VIDEO_OV5640)
#include <media/soc_camera.h>
#include <media/ov5640.h>
//#define temp_width 800
//#define temp_height 600
#endif
//end add

/* add by cym 20140313 */
#include <linux/mpu.h>

#ifdef CONFIG_MPU_SENSORS_MPU6050B1
void sensor_hw_init(void)
{
	printk("%s: line = %d\n", __FUNCTION__, __LINE__);

        if (gpio_request(EXYNOS4_GPX3(3), "MPU6050 INT"))
                printk(KERN_WARNING "MPU6050 INT(GPX3.3) Port request error!!!\n");
        else    {
                s3c_gpio_setpull(EXYNOS4_GPX3(3), S3C_GPIO_PULL_NONE);
                s3c_gpio_cfgpin(EXYNOS4_GPX3(3), S3C_GPIO_SFN(0));
                gpio_direction_input(EXYNOS4_GPX3(3));
                gpio_free(EXYNOS4_GPX3(3));
        }

        /* Sensor AK8975 DRDY */
/*      if (gpio_request(EXYNOS4_GPX1(4), "AK8975 RDY"))
                printk(KERN_WARNING "AK8975 RDY(GPX1.4) Port request error!!!\n");
        else    {
                s3c_gpio_setpull(EXYNOS4_GPX1(4), S3C_GPIO_PULL_NONE);
                s3c_gpio_cfgpin(EXYNOS4_GPX1(4), S3C_GPIO_SFN(0));
                gpio_direction_input(EXYNOS4_GPX1(4));
                gpio_free(EXYNOS4_GPX1(4));
        }       
        //enable_irq(IRQ_EINT(27));
*/
}

static struct mpu_platform_data mpu6050_data = {
        .int_config = 0x10,
        .orientation = {
                0, -1, 0,
                -1, 0, 0,
                0, 0, -1},

        .level_shifter = 0,
};
static struct ext_slave_platform_data inv_mpu_compass_data = {
        .bus = EXT_SLAVE_BUS_PRIMARY,
        .orientation = {
                -1, 0, 0,
                0, 1, 0,
                0, 0, -1},
};
#else
#if 1
static struct mpu_platform_data mpu3050_data = {
	.int_config  = 0x10,
#ifdef CONFIG_TC4_PORTRAIT_MODE
	.orientation = {  0,  1,  0, 
		1,  0,  0, 
		0,  0, -1 },
#else
	.orientation = {  -1,  0,  0,
			   0,  1,  0,
			   0,  0, -1 },
#endif
};

/* accel */
static struct ext_slave_platform_data inv_mpu_bma250_data = {
	.bus         = EXT_SLAVE_BUS_SECONDARY,
#ifdef CONFIG_TC4_PORTRAIT_MODE
	.orientation = {  1,  0,  0, 
		0,  -1,  0, 
		0,  0, -1 },
#else
	.orientation = {  0,  1,  0,
			  1,  0,  0,
			  0,  0, -1 },
#endif
};
#endif
#endif

/* end add */

/* add by cym 20140527 */
#ifdef CONFIG_USB_NET_DM9620
static void __init dm9620_reset(void)
{
	int err = 0;

	err = gpio_request_one(EXYNOS4_GPC0(1), GPIOF_OUT_INIT_HIGH, "DM9620_RESET");
	if (err)
		pr_err("failed to request GPC0_1 for DM9620 reset control\n");

	s3c_gpio_setpull(EXYNOS4_GPC0(1), S3C_GPIO_PULL_UP);
	gpio_set_value(EXYNOS4_GPC0(1), 0);
	
	mdelay(5);

	gpio_set_value(EXYNOS4_GPC0(1), 1);
	gpio_free(EXYNOS4_GPC0(1));
}
#endif
/* end add */

/* Following are default values for UCON, ULCON and UFCON UART registers */
#define SMDK4X12_UCON_DEFAULT	(S3C2410_UCON_TXILEVEL |	\
				 S3C2410_UCON_RXILEVEL |	\
				 S3C2410_UCON_TXIRQMODE |	\
				 S3C2410_UCON_RXIRQMODE |	\
				 S3C2410_UCON_RXFIFO_TOI |	\
				 S3C2443_UCON_RXERR_IRQEN)

#define SMDK4X12_ULCON_DEFAULT	S3C2410_LCON_CS8

#define SMDK4X12_UFCON_DEFAULT	(S3C2410_UFCON_FIFOMODE |	\
				 S5PV210_UFCON_TXTRIG4 |	\
				 S5PV210_UFCON_RXTRIG4)


static struct max77686_regulator_data max77686_regulators[MAX77686_REG_MAX];

static struct s3c2410_uartcfg smdk4x12_uartcfgs[] __initdata = {
	[0] = {
		.hwport		= 0,
		.flags		= 0,
		.ucon		= SMDK4X12_UCON_DEFAULT,
		.ulcon		= SMDK4X12_ULCON_DEFAULT,
		.ufcon		= SMDK4X12_UFCON_DEFAULT,
	},
	[1] = {
		.hwport		= 1,
		.flags		= 0,
		.ucon		= SMDK4X12_UCON_DEFAULT,
		.ulcon		= SMDK4X12_ULCON_DEFAULT,
		.ufcon		= SMDK4X12_UFCON_DEFAULT,
	},
	[2] = {
		.hwport		= 2,
		.flags		= 0,
		.ucon		= SMDK4X12_UCON_DEFAULT,
		.ulcon		= SMDK4X12_ULCON_DEFAULT,
		.ufcon		= SMDK4X12_UFCON_DEFAULT,
	},
	[3] = {
		.hwport		= 3,
		.flags		= 0,
		.ucon		= SMDK4X12_UCON_DEFAULT,
		.ulcon		= SMDK4X12_ULCON_DEFAULT,
		.ufcon		= SMDK4X12_UFCON_DEFAULT,
	},
};

/* add by cym 20140318 */
#ifdef CONFIG_SMSC911X
#include <linux/smsc911x.h>
#include <plat/regs-srom.h>

static struct resource smdk4x12_smsc911x_resources[] = {
	[0] = {
		.start	= EXYNOS4_PA_SROM_BANK(1),
		.end	= EXYNOS4_PA_SROM_BANK(1) + SZ_64K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		/* modify by cym 20140318 */
#if 0
		.start	= IRQ_EINT(5),
		.end	= IRQ_EINT(5),
#else
		.start	= IRQ_EINT(6),
		.end	= IRQ_EINT(6),
#endif
		.flags	= IORESOURCE_IRQ | IRQF_TRIGGER_LOW,
	},
};

static struct smsc911x_platform_config smsc9215_config = {
	.irq_polarity	= SMSC911X_IRQ_POLARITY_ACTIVE_LOW,
	.irq_type	= SMSC911X_IRQ_TYPE_PUSH_PULL,
	.flags		= SMSC911X_USE_16BIT | SMSC911X_FORCE_INTERNAL_PHY,
	.phy_interface	= PHY_INTERFACE_MODE_MII,
	.mac		= {0x00, 0x80, 0x00, 0x23, 0x45, 0x67},
};

static struct platform_device smdk4x12_smsc911x = {
	.name		= "smsc911x",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(smdk4x12_smsc911x_resources),
	.resource	= smdk4x12_smsc911x_resources,
	.dev		= {
		.platform_data	= &smsc9215_config,
	},
};

static void __init smdk4x12_smsc911x_init(void)
{
	u32 cs1;

	int i = 0;
	static char *addr[] = { "addr0" , "addr1" , "addr2", "addr3", "addr4", "addr5", "addr6", "addr7"};
	static char *data_low[] = { "data0" , "data1" , "data2", "data3", "data4", "data5", "data6", "data7"};
	static char *data_high[] = { "data8" , "data9" , "data10", "data11", "data12", "data13", "data14", "data15"};

	//Xm0WEn
	if (!gpio_request(EXYNOS4_GPY0(5), "GPY0_5")) {
		s3c_gpio_cfgpin(EXYNOS4_GPY0(5), S3C_GPIO_SFN(2));
	}

	//Xm0OEn
	if (!gpio_request(EXYNOS4_GPY0(4), "GPY0_4")) {
		s3c_gpio_cfgpin(EXYNOS4_GPY0(4), S3C_GPIO_SFN(2));
	}

	//Xm0CSn1
	if (!gpio_request(EXYNOS4_GPY0(1), "GPY0_1")) {
		s3c_gpio_cfgpin(EXYNOS4_GPY0(1), S3C_GPIO_SFN(2));
	}

	//Xm0DATA_RDn
	if (!gpio_request(EXYNOS4_GPY1(3), "GPY1_3")) {
		s3c_gpio_cfgpin(EXYNOS4_GPY1(3), S3C_GPIO_SFN(2));
	}
	
	gpio_free(EXYNOS4_GPY0(5));
	gpio_free(EXYNOS4_GPY0(4));
	gpio_free(EXYNOS4_GPY0(1));
	gpio_free(EXYNOS4_GPY1(3));

	//Xm0ADDR
	for(i=0; i<8; i++)
	{
		if (!gpio_request(EXYNOS4_GPY3(i), addr[i])) {
			s3c_gpio_cfgpin(EXYNOS4_GPY3(i), S3C_GPIO_SFN(2));
		}
		gpio_free(EXYNOS4_GPY3(i));
	}

	//Xm0DATA0-7
	for(i=0; i<8; i++)
	{
		if (!gpio_request(EXYNOS4_GPY5(i), data_low[i])) {
			s3c_gpio_cfgpin(EXYNOS4_GPY5(i), S3C_GPIO_SFN(2));
		}
		
		gpio_free(EXYNOS4_GPY5(i));
	}

	//Xm0DATA8-15
	for(i=0; i<8; i++)
	{
		if (!gpio_request(EXYNOS4_GPY6(i), data_high[i])) {
			s3c_gpio_cfgpin(EXYNOS4_GPY6(i), S3C_GPIO_SFN(2));
		}
		
		gpio_free(EXYNOS4_GPY6(i));
	}

	/* configure nCS1 width to 16 bits */
	cs1 = __raw_readl(S5P_SROM_BW) &
		~(S5P_SROM_BW__CS_MASK << S5P_SROM_BW__NCS1__SHIFT);
	cs1 |= ((1 << S5P_SROM_BW__DATAWIDTH__SHIFT) |
		(1 << S5P_SROM_BW__WAITENABLE__SHIFT) |
		(1 << S5P_SROM_BW__BYTEENABLE__SHIFT)) <<
		S5P_SROM_BW__NCS1__SHIFT;
	__raw_writel(cs1, S5P_SROM_BW);

	/* set timing for nCS1 suitable for ethernet chip */
	__raw_writel((0x1 << S5P_SROM_BCX__PMC__SHIFT) |
		     (0x9 << S5P_SROM_BCX__TACP__SHIFT) |
		     (0xc << S5P_SROM_BCX__TCAH__SHIFT) |
		     (0x1 << S5P_SROM_BCX__TCOH__SHIFT) |
		     (0x6 << S5P_SROM_BCX__TACC__SHIFT) |
		     (0x1 << S5P_SROM_BCX__TCOS__SHIFT) |
		     (0x1 << S5P_SROM_BCX__TACS__SHIFT), S5P_SROM_BC1);
}
#endif
/* end add */

static struct s3c_sdhci_platdata smdk4x12_hsmmc2_pdata __initdata = {
	.cd_type		= S3C_SDHCI_CD_GPIO,
	/* modify by cym 20140311 */
#if 0
	.ext_cd_gpio		= EXYNOS4_GPK2(2),
#else
	.ext_cd_gpio          = EXYNOS4_GPX0(7),
#endif
	/* end modify */
	.ext_cd_gpio_invert	= 1,
#ifdef CONFIG_EXYNOS4_SDHCI_CH2_8BIT
	.max_width		= 8,
	.host_caps		= MMC_CAP_8_BIT_DATA,
#endif
};

static struct s3c_sdhci_platdata smdk4x12_hsmmc3_pdata __initdata = {
	.cd_type		= S3C_SDHCI_CD_PERMANENT,
};

static struct regulator_consumer_supply max8997_buck1 =
	REGULATOR_SUPPLY("vdd_arm", NULL);

static struct regulator_consumer_supply max8997_buck2 =
	REGULATOR_SUPPLY("vdd_int", NULL);

static struct regulator_consumer_supply max8997_buck3 =
	REGULATOR_SUPPLY("vdd_g3d", NULL);

static struct regulator_init_data max8997_buck1_data = {
	.constraints	= {
		.name		= "VDD_ARM_SMDK4X12",
		.min_uV		= 925000,
		.max_uV		= 1350000,
		.always_on	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max8997_buck1,
};

static struct regulator_init_data max8997_buck2_data = {
	.constraints	= {
		.name		= "VDD_INT_SMDK4X12",
		.min_uV		= 950000,
		.max_uV		= 1150000,
		.always_on	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max8997_buck2,
};

static struct regulator_init_data max8997_buck3_data = {
	.constraints	= {
		.name		= "VDD_G3D_SMDK4X12",
		.min_uV		= 950000,
		.max_uV		= 1150000,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				  REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max8997_buck3,
};

static struct max8997_regulator_data smdk4x12_max8997_regulators[] = {
	{ MAX8997_BUCK1, &max8997_buck1_data },
	{ MAX8997_BUCK2, &max8997_buck2_data },
	{ MAX8997_BUCK3, &max8997_buck3_data },
};

static struct max8997_platform_data smdk4x12_max8997_pdata = {
	.num_regulators	= ARRAY_SIZE(smdk4x12_max8997_regulators),
	.regulators	= smdk4x12_max8997_regulators,

	.buck1_voltage[0] = 1100000,	/* 1.1V */
	.buck1_voltage[1] = 1100000,	/* 1.1V */
	.buck1_voltage[2] = 1100000,	/* 1.1V */
	.buck1_voltage[3] = 1100000,	/* 1.1V */
	.buck1_voltage[4] = 1100000,	/* 1.1V */
	.buck1_voltage[5] = 1100000,	/* 1.1V */
	.buck1_voltage[6] = 1000000,	/* 1.0V */
	.buck1_voltage[7] = 950000,	/* 0.95V */

	.buck2_voltage[0] = 1100000,	/* 1.1V */
	.buck2_voltage[1] = 1000000,	/* 1.0V */
	.buck2_voltage[2] = 950000,	/* 0.95V */
	.buck2_voltage[3] = 900000,	/* 0.9V */
	.buck2_voltage[4] = 1100000,	/* 1.1V */
	.buck2_voltage[5] = 1000000,	/* 1.0V */
	.buck2_voltage[6] = 950000,	/* 0.95V */
	.buck2_voltage[7] = 900000,	/* 0.9V */

	.buck5_voltage[0] = 1100000,	/* 1.1V */
	.buck5_voltage[1] = 1100000,	/* 1.1V */
	.buck5_voltage[2] = 1100000,	/* 1.1V */
	.buck5_voltage[3] = 1100000,	/* 1.1V */
	.buck5_voltage[4] = 1100000,	/* 1.1V */
	.buck5_voltage[5] = 1100000,	/* 1.1V */
	.buck5_voltage[6] = 1100000,	/* 1.1V */
	.buck5_voltage[7] = 1100000,	/* 1.1V */
};
#ifdef CONFIG_REGULATOR_MAX77686
static struct regulator_consumer_supply max77686_buck1 =
REGULATOR_SUPPLY("vdd_mif", NULL);

static struct regulator_consumer_supply max77686_buck2 =
REGULATOR_SUPPLY("vdd_arm", NULL);

static struct regulator_consumer_supply max77686_buck3 =
REGULATOR_SUPPLY("vdd_int", NULL);

static struct regulator_consumer_supply max77686_buck4 =
REGULATOR_SUPPLY("vdd_g3d", NULL);

static struct regulator_consumer_supply max77686_ldo11_consumer =
REGULATOR_SUPPLY("vdd_ldo11", NULL);

static struct regulator_consumer_supply max77686_ldo14_consumer =
REGULATOR_SUPPLY("vdd_ldo14", NULL);

static struct regulator_consumer_supply max77686_ldo12_consumer =
REGULATOR_SUPPLY("vusb_a", NULL);

static struct regulator_consumer_supply max77686_ldo15_consumer =
REGULATOR_SUPPLY("vusb_d", NULL);

static struct regulator_consumer_supply max77686_ldo16_consumer =
REGULATOR_SUPPLY("vdd_hsic", NULL);

static struct regulator_consumer_supply max77686_ldo8_consumer[] = {
REGULATOR_SUPPLY("vdd", "exynos4-hdmi"),
REGULATOR_SUPPLY("vdd_pll", "exynos4-hdmi"),
REGULATOR_SUPPLY("vdd8_mipi", NULL),
};

static struct regulator_consumer_supply max77686_ldo10_consumer[] = {
REGULATOR_SUPPLY("vdd_osc", "exynos4-hdmi"),
REGULATOR_SUPPLY("vdd10_mipi", NULL),
REGULATOR_SUPPLY("vdd_tmu", NULL),
};

static struct regulator_init_data max77686_ldo8_data = {
	.constraints = {
		.name		= "vdd_ldo8 range",
		.min_uV		= 1000000,
		.max_uV		= 1000000,
		.boot_on	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
			.mode		= REGULATOR_MODE_STANDBY,
		},
		.initial_state = PM_SUSPEND_MEM, 
	},
	.num_consumer_supplies	= ARRAY_SIZE(max77686_ldo8_consumer),
	.consumer_supplies	= max77686_ldo8_consumer,
};

static struct regulator_init_data max77686_ldo10_data = {
	.constraints = {
		.name		= "vdd_ldo10 range",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.boot_on	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
			.mode		= REGULATOR_MODE_STANDBY,
		},
		.initial_state = PM_SUSPEND_MEM, 
	},
	.num_consumer_supplies	= ARRAY_SIZE(max77686_ldo10_consumer),
	.consumer_supplies	= max77686_ldo10_consumer,
};

static struct regulator_init_data max77686_ldo12_data = {
	.constraints = {
		.name		= "vdd_ldo12 range",
		.min_uV		= 3000000,
		.max_uV		= 3300000,
		.boot_on	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.always_on	= 1,
		.state_mem	= {
			.disabled	= 1,
			.mode		= REGULATOR_MODE_STANDBY,
		},
		.initial_state = PM_SUSPEND_MEM, 
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77686_ldo12_consumer,
};

static struct regulator_init_data max77686_ldo15_data = {
	.constraints = {
		.name		= "vdd_ldo15 range",
		.min_uV		= 1000000,
		.max_uV		= 1000000,
		.boot_on	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.always_on	= 1,
		.state_mem	= {
			.disabled	= 1,
			.mode		= REGULATOR_MODE_STANDBY,
		},
		.initial_state = PM_SUSPEND_MEM, 
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77686_ldo15_consumer,
};

static struct regulator_init_data max77686_ldo16_data = {
	.constraints = {
		.name		= "vdd_ldo16 range",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.boot_on	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.always_on	= 1,
		.state_mem	= {
			.disabled	= 1,
			.mode		= REGULATOR_MODE_STANDBY,
		},
		.initial_state = PM_SUSPEND_MEM, 
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77686_ldo16_consumer,
};

static struct regulator_init_data max77686_buck1_data = {
	.constraints = {
		.name		= "vdd_mif range",
		.min_uV		= 800000,
		.max_uV		= 1300000,
		.always_on	= 1,
		.boot_on	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
				  REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
			.mode		= REGULATOR_MODE_STANDBY,
		},
		.initial_state = PM_SUSPEND_MEM, 
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &max77686_buck1,
};

static struct regulator_init_data max77686_buck2_data = {
	.constraints = {
		.name		= "vdd_arm range",
		.min_uV		= 800000,
		.max_uV		= 1350000,
		.always_on	= 1,
		.boot_on	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
				  REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
			.mode		= REGULATOR_MODE_STANDBY,
		},
		.initial_state = PM_SUSPEND_MEM, 
	},
	.num_consumer_supplies 	= 1,
	.consumer_supplies	= &max77686_buck2,
};

static struct regulator_init_data max77686_buck3_data = {
	.constraints = {
		.name		= "vdd_int range",
		.min_uV		= 800000,
		.max_uV		= 1150000,
		.always_on	= 1,
		.boot_on	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
				  REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
			.mode		= REGULATOR_MODE_STANDBY,
		},
		.initial_state = PM_SUSPEND_MEM, 
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &max77686_buck3,
};

static struct regulator_init_data max77686_buck4_data = {
	.constraints = {
		.name		= "vdd_g3d range",
		.min_uV		= 850000,
		.max_uV		= 1200000,
		.boot_on	= 1,
		.always_on	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
				  REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
			.mode		= REGULATOR_MODE_STANDBY,
		},
		.initial_state = PM_SUSPEND_MEM, 
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &max77686_buck4,
};

static struct regulator_init_data __initdata max77686_ldo11_data = {
	.constraints	= {
		.name		= "vdd_ldo11 range",
		.min_uV		= 1900000,
		.max_uV		= 1900000,
		.apply_uV	= 1,
		.state_mem	= {
			.disabled	= 1,
			.mode		= REGULATOR_MODE_STANDBY,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77686_ldo11_consumer,
};

static struct regulator_init_data __initdata max77686_ldo14_data = {
	.constraints	= {
		.name		= "vdd_ldo14 range",
		.min_uV		= 1900000,
		.max_uV		= 1900000,
		.apply_uV	= 1,
		.always_on	= 1,
		.state_mem	= {
			.disabled	= 1,
			.mode		= REGULATOR_MODE_STANDBY,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77686_ldo14_consumer,
};

static struct regulator_init_data __initdata max77686_always_on = {
	.constraints = {
		.always_on 	= 1,
		.state_mem 	= {
			.disabled	= 1,
			.mode		= REGULATOR_MODE_STANDBY,
		},
	},
};

static void max77686_populate_pdata (void)
{
	int i;

	/* LDOs[0-7] and BUCKs[5-7] are not initialized yet but required to
	 * be always enabled for stability */
	for (i = 0; i <= MAX77686_LDO7; i++)
		max77686_regulators[i].initdata = &max77686_always_on;
	max77686_regulators[MAX77686_BUCK5].initdata = &max77686_always_on;
	max77686_regulators[MAX77686_BUCK6].initdata = &max77686_always_on;
	max77686_regulators[MAX77686_BUCK7].initdata = &max77686_always_on;

	max77686_regulators[MAX77686_BUCK1].initdata = &max77686_buck1_data;
	max77686_regulators[MAX77686_BUCK2].initdata = &max77686_buck2_data;
	max77686_regulators[MAX77686_BUCK3].initdata = &max77686_buck3_data;
	max77686_regulators[MAX77686_BUCK4].initdata = &max77686_buck4_data;
	max77686_regulators[MAX77686_LDO8].initdata  = &max77686_ldo8_data;
	max77686_regulators[MAX77686_LDO10].initdata = &max77686_ldo10_data;
	max77686_regulators[MAX77686_LDO11].initdata = &max77686_ldo11_data;
	max77686_regulators[MAX77686_LDO12].initdata = &max77686_ldo12_data;
	max77686_regulators[MAX77686_LDO14].initdata = &max77686_ldo14_data;
	max77686_regulators[MAX77686_LDO15].initdata = &max77686_ldo15_data;
	max77686_regulators[MAX77686_LDO16].initdata = &max77686_ldo16_data;

	regulator_has_full_constraints();
}

static struct max77686_platform_data smdk4412_max77686_pdata = {
	.num_regulators = ARRAY_SIZE(max77686_regulators),
	.regulators = max77686_regulators,
};
#endif
#ifdef CONFIG_REGULATOR_S5M8767
/* S5M8767 Regulator */
static int s5m_cfg_irq(void)
{
        /* AP_PMIC_IRQ: EINT26 */
	s3c_gpio_cfgpin(EXYNOS4_GPX1(7), S3C_GPIO_SFN(0xF));
	s3c_gpio_setpull(EXYNOS4_GPX1(7), S3C_GPIO_PULL_UP);
        return 0;
}
#if 0
static struct regulator_consumer_supply s5m8767_buck1_consumer =
REGULATOR_SUPPLY("vdd_mif", NULL);

static struct regulator_consumer_supply s5m8767_buck2_consumer =
REGULATOR_SUPPLY("vdd_arm", NULL);

static struct regulator_consumer_supply s5m8767_buck3_consumer =
REGULATOR_SUPPLY("vdd_int", NULL);

static struct regulator_consumer_supply s5m8767_buck4_consumer =
REGULATOR_SUPPLY("vdd_g3d", NULL);

static struct regulator_consumer_supply s5m8767_ldo11_consumer =
REGULATOR_SUPPLY("vdd_ldo11", NULL);

static struct regulator_consumer_supply s5m8767_ldo14_consumer =
REGULATOR_SUPPLY("vdd_ldo14", NULL);

static struct regulator_consumer_supply s5m8767_ldo12_consumer =
REGULATOR_SUPPLY("vusb_a", NULL);

static struct regulator_consumer_supply s5m8767_ldo15_consumer =
REGULATOR_SUPPLY("vusb_d", NULL);

static struct regulator_consumer_supply s5m8767_ldo16_consumer =
REGULATOR_SUPPLY("vdd_hsic", NULL);

static struct regulator_consumer_supply s5m8767_ldo8_consumer[] = {
REGULATOR_SUPPLY("vdd", "exynos4-hdmi"),
REGULATOR_SUPPLY("vdd_pll", "exynos4-hdmi"),
REGULATOR_SUPPLY("vdd8_mipi", NULL),
};
static struct regulator_consumer_supply s5m8767_ldo10_consumer[] = {
REGULATOR_SUPPLY("vdd_osc", "exynos4-hdmi"),
REGULATOR_SUPPLY("vdd10_mipi", NULL),
REGULATOR_SUPPLY("vdd_tmu", NULL),
};
#else
/* add by cym 20140311 */
static struct regulator_consumer_supply s5m8767_ldo1_supply[] = {
	REGULATOR_SUPPLY("vdd_alive", NULL),
};

static struct regulator_consumer_supply s5m8767_ldo2_supply[] = {
	REGULATOR_SUPPLY("vddq_m12", NULL),
};

static struct regulator_consumer_supply s5m8767_ldo3_supply[] = {
	REGULATOR_SUPPLY("vddioap_18", NULL),
};

static struct regulator_consumer_supply s5m8767_ldo4_supply[] = {
	REGULATOR_SUPPLY("vddq_pre", NULL),
};

static struct regulator_consumer_supply s5m8767_ldo5_supply[] = {
	REGULATOR_SUPPLY("vdd18_2m", NULL),
};

static struct regulator_consumer_supply s5m8767_ldo6_supply[] = {
	REGULATOR_SUPPLY("vdd10_mpll", NULL),
};

static struct regulator_consumer_supply s5m8767_ldo7_supply[] = {
	REGULATOR_SUPPLY("vdd10_xpll", NULL),
};

static struct regulator_consumer_supply s5m8767_ldo8_supply[] = {
	REGULATOR_SUPPLY("vdd10_mipi", NULL),
};

static struct regulator_consumer_supply s5m8767_ldo9_supply[] = {
	REGULATOR_SUPPLY("vdd33_lcd", NULL),
};

static struct regulator_consumer_supply s5m8767_ldo10_supply[] = {
	REGULATOR_SUPPLY("vdd18_mipi", NULL),
};

static struct regulator_consumer_supply s5m8767_ldo11_supply[] = {
	REGULATOR_SUPPLY("vdd18_abb1", NULL),
};

static struct regulator_consumer_supply s5m8767_ldo12_supply[] = {
	REGULATOR_SUPPLY("vdd33_uotg", NULL),
};

static struct regulator_consumer_supply s5m8767_ldo13_supply[] = {
	REGULATOR_SUPPLY("vddioperi_18", NULL),
};

static struct regulator_consumer_supply s5m8767_ldo14_supply[] = {
	REGULATOR_SUPPLY("vdd18_abb02", NULL),
};

static struct regulator_consumer_supply s5m8767_ldo15_supply[] = {
	REGULATOR_SUPPLY("vdd10_ush", NULL),
};

static struct regulator_consumer_supply s5m8767_ldo16_supply[] = {
	REGULATOR_SUPPLY("vdd18_hsic", NULL),
};

static struct regulator_consumer_supply s5m8767_ldo17_supply[] = {
	REGULATOR_SUPPLY("vddioap_mmc012_28", NULL),
};
static struct regulator_consumer_supply s5m8767_ldo18_supply[] = {
	REGULATOR_SUPPLY("vddioperi_28", NULL),
};

static struct regulator_consumer_supply s5m8767_ldo19_supply[] = {
	REGULATOR_SUPPLY("dc33v_tp", NULL),		//cym 20130227
};


static struct regulator_consumer_supply s5m8767_ldo20_supply[] = {
	REGULATOR_SUPPLY("vdd28_cam", NULL),
};
static struct regulator_consumer_supply s5m8767_ldo21_supply[] = {
	REGULATOR_SUPPLY("vdd28_af", NULL),
};

static struct regulator_consumer_supply s5m8767_ldo22_supply[] = {
	REGULATOR_SUPPLY("vdda28_2m", NULL),
};

static struct regulator_consumer_supply s5m8767_ldo23_supply[] = {
	REGULATOR_SUPPLY("vdd_tf", NULL),
};


static struct regulator_consumer_supply s5m8767_ldo24_supply[] = {
	REGULATOR_SUPPLY("vdd33_a31", NULL),
};

static struct regulator_consumer_supply s5m8767_ldo25_supply[] = {
	REGULATOR_SUPPLY("vdd18_cam", NULL),
};

static struct regulator_consumer_supply s5m8767_ldo26_supply[] = {
	REGULATOR_SUPPLY("vdd18_a31", NULL),
};
static struct regulator_consumer_supply s5m8767_ldo27_supply[] = {
	REGULATOR_SUPPLY("gps_1v8", NULL),
};
static struct regulator_consumer_supply s5m8767_ldo28_supply[] = {
	REGULATOR_SUPPLY("dvdd12", NULL),
};

/* end add */

static struct regulator_consumer_supply s5m8767_buck1_consumer =
REGULATOR_SUPPLY("vdd_mif", NULL);

static struct regulator_consumer_supply s5m8767_buck2_consumer =
REGULATOR_SUPPLY("vdd_arm", NULL);

static struct regulator_consumer_supply s5m8767_buck3_consumer =
REGULATOR_SUPPLY("vdd_int", NULL);

static struct regulator_consumer_supply s5m8767_buck4_consumer =
REGULATOR_SUPPLY("vdd_g3d", NULL);

/* add by cym 20140311 */
static struct regulator_consumer_supply s5m8767_buck5_consumer =
	REGULATOR_SUPPLY("vdd_m12", NULL);
static struct regulator_consumer_supply s5m8767_buck6_consumer =
	REGULATOR_SUPPLY("vdd12_5m", NULL);

static struct regulator_consumer_supply s5m8767_buck9_consumer =
	REGULATOR_SUPPLY("vddf28_emmc", NULL);
/* end add */
#endif

#if 0
static struct regulator_init_data s5m8767_ldo8_data = {
	.constraints	={
		.name		= "vdd_ldo8 range",
		.min_uV		= 1000000,
		.max_uV         = 1000000,
		.boot_on        = 1,
		.always_on      = 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem      = {
			.disabled       = 1,
			.mode  	        = REGULATOR_MODE_STANDBY,
		},
		.initial_state = PM_SUSPEND_MEM,
	},
	.num_consumer_supplies  = ARRAY_SIZE(s5m8767_ldo8_consumer),
	.consumer_supplies      = s5m8767_ldo8_consumer,
};

static struct regulator_init_data s5m8767_ldo10_data = {
	.constraints = {
		.name           = "vdd_ldo10 range",
		.min_uV         = 1800000,
		.max_uV         = 1800000,
		.boot_on        = 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem      = {
			.disabled       = 1,
			.mode           = REGULATOR_MODE_STANDBY,
		},
		.initial_state = PM_SUSPEND_MEM,
	},
	.num_consumer_supplies  = ARRAY_SIZE(s5m8767_ldo10_consumer),
	.consumer_supplies      = s5m8767_ldo10_consumer,
};

static struct regulator_init_data s5m8767_ldo12_data = {
	.constraints = {
		.name           = "vdd_ldo12 range",
		.min_uV         = 3000000,
		.max_uV         = 3000000,
		.boot_on        = 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_VOLTAGE,
		.always_on      = 1,
		.state_mem      = {
			.disabled       = 1,
			.mode           = REGULATOR_MODE_STANDBY,
		},
	.initial_state = PM_SUSPEND_MEM,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &s5m8767_ldo12_consumer,
};

static struct regulator_init_data s5m8767_ldo15_data = {
	.constraints = {
		.name           = "vdd_ldo15 range",
		.min_uV         = 1000000,
		.max_uV         = 1000000,
		.boot_on        = 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.always_on      = 1,
		.state_mem      = {
			.disabled       = 1,
			.mode           = REGULATOR_MODE_STANDBY,
		},
		.initial_state = PM_SUSPEND_MEM,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &s5m8767_ldo15_consumer,
};

static struct regulator_init_data s5m8767_ldo16_data = {
	.constraints = {
		.name           = "vdd_ldo16 range",
		.min_uV         = 1800000,
		.max_uV         = 1800000,
		.boot_on        = 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.always_on      = 1,
		.state_mem      = {
			.disabled       = 1,
			.mode           = REGULATOR_MODE_STANDBY,
		},
		.initial_state = PM_SUSPEND_MEM,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &s5m8767_ldo16_consumer,
};

static struct regulator_init_data __initdata s5m8767_ldo11_data = {
	.constraints    = {
		.name           = "vdd_ldo11 range",
		.min_uV         = 1800000,
		.max_uV         = 1800000,
		.always_on      = 1,
		.apply_uV       = 1,
		.state_mem      = {
			.disabled       = 1,
			.mode           = REGULATOR_MODE_STANDBY,
		},
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &s5m8767_ldo11_consumer,
};

static struct regulator_init_data __initdata s5m8767_ldo14_data = {
	.constraints    = {
		.name           = "vdd_ldo14 range",
		.min_uV         = 1800000,
		.max_uV         = 1800000,
		.apply_uV       = 1,
		.always_on      = 1,
		.state_mem      = {
			.disabled       = 1,
			.mode           = REGULATOR_MODE_STANDBY,
		},
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &s5m8767_ldo14_consumer,
};

static struct regulator_init_data s5m8767_buck1_data = {
	.constraints    = {
		.name           = "vdd_mif range",
		.min_uV         = 800000,
		.max_uV         = 1100000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS,
		.always_on = 1,
		.boot_on = 1,
		.state_mem      = {
			.disabled       = 1,
		},
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &s5m8767_buck1_consumer,
};

static struct regulator_init_data s5m8767_buck2_data = {
	.constraints    = {
		.name           = "vdd_arm range",
		.min_uV         =  800000,
		.max_uV         = 1350000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS,
		.always_on = 1,
		.boot_on = 1,
		.state_mem      = {
				.disabled       = 1,
		},
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &s5m8767_buck2_consumer,
};

static struct regulator_init_data s5m8767_buck3_data = {
	.constraints    = {
		.name           = "vdd_int range",
		.min_uV         =  800000,
		.max_uV         = 1150000,
		.apply_uV       = 1,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS,
		.always_on = 1,
		.boot_on = 1,
		.state_mem      = {
			.uV             = 1100000,
			.mode           = REGULATOR_MODE_NORMAL,
			.disabled       = 1,
		},
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &s5m8767_buck3_consumer,
};
static struct regulator_init_data s5m8767_buck4_data = {
	.constraints    = {
		.name           = "vdd_g3d range",
		.min_uV         = 850000,
		.max_uV         = 1200000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS,
		.always_on = 1,
		.boot_on = 1,
		.state_mem      = {
			.disabled       = 1,
			},
		},
	.num_consumer_supplies = 1,
	.consumer_supplies = &s5m8767_buck4_consumer,
};
#else
#define REGULATOR_INIT(_ldo, _name, _min_uV, _max_uV, _always_on, _ops_mask,\
		_disabled) \
	static struct regulator_init_data s5m8767_##_ldo##_init_data = {		\
		.constraints = {					\
			.name	= _name,				\
			.min_uV = _min_uV,				\
			.max_uV = _max_uV,				\
			.always_on	= _always_on,			\
			.boot_on	= _always_on,			\
			.apply_uV	= 1,				\
			.valid_ops_mask = _ops_mask,			\
			.state_mem	= {				\
				.disabled	= _disabled,		\
				.enabled	= !(_disabled),		\
			}						\
		},							\
		.num_consumer_supplies = ARRAY_SIZE(s5m8767_##_ldo##_supply),	\
		.consumer_supplies = &s5m8767_##_ldo##_supply[0],			\
	};

REGULATOR_INIT(ldo1, "VDD_ALIVE", 1100000, 1100000, 1,
		REGULATOR_CHANGE_STATUS, 0);
/* modify by cym 20130709 */
#ifdef CONFIG_CPU_TYPE_SCP
REGULATOR_INIT(ldo2, "VDDQ_M12", 1500000, 1500000, 1,
		REGULATOR_CHANGE_STATUS, 1)
#else
REGULATOR_INIT(ldo2, "VDDQ_M12", 1200000, 1200000, 1,
		REGULATOR_CHANGE_STATUS, 1);//sleep controlled by pwren
#endif
/* end modify */
REGULATOR_INIT(ldo3, "VDDIOAP_18", 1800000, 1800000, 1,
		REGULATOR_CHANGE_STATUS, 0);
REGULATOR_INIT(ldo4, "VDDQ_PRE", 1800000, 1800000, 1,
		REGULATOR_CHANGE_STATUS, 1); //sleep controlled by pwren

REGULATOR_INIT(ldo5, "VDD18_2M", 1800000, 1800000, 0,
		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo6, "VDD10_MPLL", 1000000, 1000000, 1,
		REGULATOR_CHANGE_STATUS, 1);//sleep controlled by pwren
REGULATOR_INIT(ldo7, "VDD10_XPLL", 1000000, 1000000, 1,
		REGULATOR_CHANGE_STATUS, 1);//sleep controlled by pwren
REGULATOR_INIT(ldo8, "VDD10_MIPI", 1000000, 1000000, 0,
		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo9, "VDD33_LCD", 3300000, 3300000, 1,
		REGULATOR_CHANGE_STATUS, 1);


REGULATOR_INIT(ldo10, "VDD18_MIPI", 1800000, 1800000, 0,
		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo11, "VDD18_ABB1", 1800000, 1800000, 1,
		REGULATOR_CHANGE_STATUS, 0); //???
REGULATOR_INIT(ldo12, "VDD33_UOTG", 3300000, 3300000, 1,
		REGULATOR_CHANGE_STATUS, 0);
REGULATOR_INIT(ldo13, "VDDIOPERI_18", 1800000, 1800000, 1,
		REGULATOR_CHANGE_STATUS, 0);//???
REGULATOR_INIT(ldo14, "VDD18_ABB02", 1800000, 1800000, 1,
		REGULATOR_CHANGE_STATUS, 0); //???
REGULATOR_INIT(ldo15, "VDD10_USH", 1000000, 1000000, 1,
		REGULATOR_CHANGE_STATUS, 1);

//liang, VDD18_HSIC must be 1.8V, otherwise USB HUB 3503A can't be recognized
REGULATOR_INIT(ldo16, "VDD18_HSIC", 1800000, 1800000, 1,
		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo17, "VDDIOAP_MMC012_28", 2800000, 2800000, 1,
		REGULATOR_CHANGE_STATUS, 0); //???
REGULATOR_INIT(ldo18, "VDDIOPERI_28", 2800000, 2800000, 1,
		REGULATOR_CHANGE_STATUS, 0);//???
REGULATOR_INIT(ldo19, "DC33V_TP", 3300000, 3300000, 0,
		REGULATOR_CHANGE_STATUS, 1); //??
REGULATOR_INIT(ldo20, "VDD28_CAM", 2800000, 2800000, 0,
		REGULATOR_CHANGE_STATUS, 1);

/* modify by cym 20130417 for MT6260 VCC */
#ifdef CONFIG_TOUCHSCREEN_TSC2007
REGULATOR_INIT(ldo21, "VDD28_AF", 2800000, 2800000, 1,
		REGULATOR_CHANGE_STATUS, 0);
#else
REGULATOR_INIT(ldo21, "VDD28_AF", 2800000, 2800000, 0,
		REGULATOR_CHANGE_STATUS, 1);
#endif
/* end modify */
REGULATOR_INIT(ldo22, "VDDA28_2M", 2800000, 2800000, 0,
		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo23, "VDD28_TF", 2800000, 2800000, 1,
		REGULATOR_CHANGE_STATUS, 1);//sleep controlled by pwren

/* modify by cym 20130318 for 4412 SCP */
#if 0
REGULATOR_INIT(ldo24, "VDD33_A31", 3300000, 3300000, 1,
		REGULATOR_CHANGE_STATUS, 0);
#else
//#ifdef CONFIG_CPU_TYPE_SCP
#if defined(CONFIG_MTK_COMBO_COMM) || defined(CONFIG_MTK_COMBO_COMM_MODULE)
REGULATOR_INIT(ldo24, "VDD33_A31", 2800000, 2800000, 1,
		REGULATOR_CHANGE_STATUS, 0);
#else
REGULATOR_INIT(ldo24, "VDD33_A31", 3300000, 3300000, 1,
		REGULATOR_CHANGE_STATUS, 0);
#endif
#endif
/* end modify */

REGULATOR_INIT(ldo25, "VDD18_CAM", 1800000, 1800000, 0,
		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo26, "VDD18_A31", 1800000, 1800000, 1,
		REGULATOR_CHANGE_STATUS, 0);
REGULATOR_INIT(ldo27, "GPS_1V8", 1800000, 1800000, 1,
		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo28, "DVDD12", 1200000, 1200000, 0,
		REGULATOR_CHANGE_STATUS, 1);

static struct regulator_init_data s5m8767_buck1_data = {
	.constraints	= {
		.name		= "vdd_mif range",
		.min_uV		= 900000,
		.max_uV		= 1100000,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				  REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s5m8767_buck1_consumer,
};

static struct regulator_init_data s5m8767_buck2_data = {
	.constraints	= {
		.name		= "vdd_arm range",
		.min_uV		=  850000,
		.max_uV		= 1450000,
		.apply_uV	= 1,
		.boot_on	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				  REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s5m8767_buck2_consumer,
};

static struct regulator_init_data s5m8767_buck3_data = {
	.constraints	= {
		.name		= "vdd_int range",
		.min_uV		=  875000,
		.max_uV		= 1200000,
		.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			//.uV		= 1100000,
			.mode		= REGULATOR_MODE_NORMAL,
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s5m8767_buck3_consumer,
};

static struct regulator_init_data s5m8767_buck4_data = {
	.constraints	= {
		.name		= "vdd_g3d range",
		.min_uV		= 750000,
		.max_uV		= 1500000,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &s5m8767_buck4_consumer,
};

static struct regulator_init_data s5m8767_buck5_data = {
	.constraints	= {
		.name		= "vdd_m12 range",
		.min_uV		= 750000,
		.max_uV		= 1500000,
		.apply_uV	= 1,
		.boot_on	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &s5m8767_buck5_consumer,
};
static struct regulator_init_data s5m8767_buck6_data = {
	.constraints	= {
		.name		= "vdd12_5m range",
		.min_uV		= 750000,
		.max_uV		= 1500000,
		.boot_on	= 0,
		.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &s5m8767_buck6_consumer,
};
static struct regulator_init_data s5m8767_buck9_data = {
	.constraints	= {
		.name		= "vddf28_emmc range",
		.min_uV		= 750000,
		.max_uV		= 3000000,
		.boot_on	= 1,
		.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &s5m8767_buck9_consumer,
};
#endif

#if 0
static struct sec_regulator_data pegasus_regulators[] = {
	{ S5M8767_LDO8,  &s5m8767_ldo8_data},
	{ S5M8767_LDO10, &s5m8767_ldo10_data},
	{ S5M8767_LDO11, &s5m8767_ldo11_data},
	{ S5M8767_LDO12, &s5m8767_ldo12_data},
	{ S5M8767_LDO14, &s5m8767_ldo14_data},
	{ S5M8767_LDO15, &s5m8767_ldo15_data},
	{ S5M8767_LDO16, &s5m8767_ldo16_data},
	{ S5M8767_BUCK1, &s5m8767_buck1_data },
	{ S5M8767_BUCK2, &s5m8767_buck2_data },
	{ S5M8767_BUCK3, &s5m8767_buck3_data },
	{ S5M8767_BUCK4, &s5m8767_buck4_data },
};

struct sec_opmode_data s5m8767_opmode_data[S5M8767_REG_MAX] = {
	[S5M8767_BUCK1] = {S5M8767_BUCK1, S5M_OPMODE_SUSPEND},
	[S5M8767_BUCK2] = {S5M8767_BUCK2, S5M_OPMODE_SUSPEND},
	[S5M8767_BUCK3] = {S5M8767_BUCK3, S5M_OPMODE_SUSPEND},
	[S5M8767_BUCK4] = {S5M8767_BUCK4, S5M_OPMODE_SUSPEND},
	[S5M8767_LDO8]  = {S5M8767_LDO8,  S5M_OPMODE_SUSPEND},
	[S5M8767_LDO10] = {S5M8767_LDO10, S5M_OPMODE_SUSPEND},
	[S5M8767_LDO11] = {S5M8767_LDO11, S5M_OPMODE_SUSPEND},
	[S5M8767_LDO12] = {S5M8767_LDO12, S5M_OPMODE_SUSPEND},
	[S5M8767_LDO14] = {S5M8767_LDO14, S5M_OPMODE_SUSPEND},
	[S5M8767_LDO15] = {S5M8767_LDO15, S5M_OPMODE_SUSPEND},
	[S5M8767_LDO16] = {S5M8767_LDO16, S5M_OPMODE_SUSPEND},
};
#else
static struct sec_regulator_data pegasus_regulators[] = {
	{ S5M8767_BUCK1, &s5m8767_buck1_data },
	{ S5M8767_BUCK2, &s5m8767_buck2_data },
	{ S5M8767_BUCK3, &s5m8767_buck3_data },
	{ S5M8767_BUCK4, &s5m8767_buck4_data },
	{ S5M8767_BUCK5, &s5m8767_buck5_data },
	{ S5M8767_BUCK6, &s5m8767_buck6_data },
	{ S5M8767_BUCK9, &s5m8767_buck9_data },

	{ S5M8767_LDO1, &s5m8767_ldo1_init_data },
	{ S5M8767_LDO2, &s5m8767_ldo2_init_data },
	{ S5M8767_LDO3, &s5m8767_ldo3_init_data },
	{ S5M8767_LDO4, &s5m8767_ldo4_init_data },
	{ S5M8767_LDO5, &s5m8767_ldo5_init_data },
	{ S5M8767_LDO6, &s5m8767_ldo6_init_data },
	{ S5M8767_LDO7, &s5m8767_ldo7_init_data },
	{ S5M8767_LDO8, &s5m8767_ldo8_init_data },
	{ S5M8767_LDO9, &s5m8767_ldo9_init_data },
	{ S5M8767_LDO10, &s5m8767_ldo10_init_data },

	{ S5M8767_LDO11, &s5m8767_ldo11_init_data },
	{ S5M8767_LDO12, &s5m8767_ldo12_init_data },
	{ S5M8767_LDO13, &s5m8767_ldo13_init_data },
	{ S5M8767_LDO14, &s5m8767_ldo14_init_data },
	{ S5M8767_LDO15, &s5m8767_ldo15_init_data },
	{ S5M8767_LDO16, &s5m8767_ldo16_init_data },
	{ S5M8767_LDO17, &s5m8767_ldo17_init_data },
	{ S5M8767_LDO18, &s5m8767_ldo18_init_data },
	{ S5M8767_LDO19, &s5m8767_ldo19_init_data },
	{ S5M8767_LDO20, &s5m8767_ldo20_init_data },

	{ S5M8767_LDO21, &s5m8767_ldo21_init_data },
	{ S5M8767_LDO22, &s5m8767_ldo22_init_data },
	{ S5M8767_LDO23, &s5m8767_ldo23_init_data },
	{ S5M8767_LDO24, &s5m8767_ldo24_init_data },
	{ S5M8767_LDO25, &s5m8767_ldo25_init_data },
	{ S5M8767_LDO26, &s5m8767_ldo26_init_data },
	{ S5M8767_LDO27, &s5m8767_ldo27_init_data },
	{ S5M8767_LDO28, &s5m8767_ldo28_init_data },
	
	
};

static struct sec_platform_data exynos4_s5m8767_pdata = {
	.device_type		= S5M8767X,
	.irq_base		= IRQ_BOARD_START,
	.num_regulators		= ARRAY_SIZE(pegasus_regulators),
	.regulators		= pegasus_regulators,
	.cfg_pmic_irq		= s5m_cfg_irq,

	.buck2_voltage[0]	= 1250000,
	.buck2_voltage[1]	= 1200000,
	.buck2_voltage[2]	= 1200000,
	.buck2_voltage[3]	= 1200000,
	.buck2_voltage[4]	= 1200000,
	.buck2_voltage[5]	= 1200000,
	.buck2_voltage[6]	=  1200000,
	.buck2_voltage[7]	=  1200000,

	.buck3_voltage[0]	= 1100000,
	.buck3_voltage[1]	= 1100000,
	.buck3_voltage[2]	= 1100000,
	.buck3_voltage[3]	= 1100000,
	.buck3_voltage[4]	= 1100000,
	.buck3_voltage[5]	= 1100000,
	.buck3_voltage[6]	= 1100000,
	.buck3_voltage[7]	= 1100000,

	.buck4_voltage[0]	= 1200000,
	.buck4_voltage[1]	= 1200000,
	.buck4_voltage[2]	= 1200000,
	.buck4_voltage[3]	= 1200000,
	.buck4_voltage[4]	= 1200000,
	.buck4_voltage[5]	= 1200000,
	.buck4_voltage[6]	= 1200000,
	.buck4_voltage[7]	= 1200000,

	.buck_default_idx	= 3,
	.buck_gpios[0]		= EXYNOS4_GPX2(5),
	.buck_gpios[1]		= EXYNOS4_GPX2(6),
	.buck_gpios[2]		= EXYNOS4_GPX2(7),

	.buck_ramp_delay        = 10,
	.buck2_ramp_enable      = true,
	.buck3_ramp_enable      = true,
	.buck4_ramp_enable      = true,
};
#endif
#if 0
static struct sec_platform_data exynos4_s5m8767_pdata = {
	.device_type            = S5M8767X,
	.irq_base               = IRQ_BOARD_START,
	.num_regulators         = ARRAY_SIZE(pegasus_regulators),
	.regulators             = pegasus_regulators,
	.cfg_pmic_irq           = s5m_cfg_irq,
	.wakeup                 = 1,
	.opmode            = s5m8767_opmode_data,
	//        .wtsr_smpl              = 1,

	.buck_default_idx       = 1,
	.buck_gpios[0]          = EXYNOS4_GPL0(3),
	.buck_gpios[1]          = EXYNOS4_GPL0(4),
	.buck_gpios[2]          = EXYNOS4_GPL0(6),

	.buck_ramp_delay        = 25,
	.buck2_ramp_enable      = true,
	.buck3_ramp_enable      = true,
	.buck4_ramp_enable      = true,
#if 0
	.buck_ds[0]             = EXYNOS4_GPL0(0),
	.buck_ds[1]             = EXYNOS4_GPL0(1),
	.buck_ds[2]             = EXYNOS4_GPL0(2),
	.buck2_init             = 1100000,
	.buck3_init             = 1000000,
	.buck4_init             = 1000000,
#endif
};
#endif
/* End of S5M8767 */
#endif

#ifdef CONFIG_SND_SOC_SAMSUNG_SMDK_WM8994
static struct regulator_consumer_supply wm8994_fixed_voltage0_supplies[] = {
        REGULATOR_SUPPLY("AVDD2", "1-001a"),
        REGULATOR_SUPPLY("CPVDD", "1-001a"),
};

static struct regulator_consumer_supply wm8994_fixed_voltage1_supplies[] = {
        REGULATOR_SUPPLY("SPKVDD1", "1-001a"),
        REGULATOR_SUPPLY("SPKVDD2", "1-001a"),
};

static struct regulator_consumer_supply wm8994_fixed_voltage2_supplies =
        REGULATOR_SUPPLY("DBVDD", "1-001a");

static struct regulator_init_data wm8994_fixed_voltage0_init_data = {
        .constraints = {
                .always_on = 1,
        },
        .num_consumer_supplies  = ARRAY_SIZE(wm8994_fixed_voltage0_supplies),
        .consumer_supplies      = wm8994_fixed_voltage0_supplies,
};

static struct regulator_init_data wm8994_fixed_voltage1_init_data = {
        .constraints = {
                .always_on = 1,
        },
        .num_consumer_supplies  = ARRAY_SIZE(wm8994_fixed_voltage1_supplies),
        .consumer_supplies      = wm8994_fixed_voltage1_supplies,
};

static struct regulator_init_data wm8994_fixed_voltage2_init_data = {
        .constraints = {
                .always_on = 1,
        },
        .num_consumer_supplies  = 1,
        .consumer_supplies      = &wm8994_fixed_voltage2_supplies,
};

static struct fixed_voltage_config wm8994_fixed_voltage0_config = {
        .supply_name    = "VDD_1.8V",
        .microvolts     = 1800000,
        .gpio           = -EINVAL,
        .init_data      = &wm8994_fixed_voltage0_init_data,
};

static struct fixed_voltage_config wm8994_fixed_voltage1_config = {
        .supply_name    = "DC_5V",
        .microvolts     = 5000000,
        .gpio           = -EINVAL,
        .init_data      = &wm8994_fixed_voltage1_init_data,
};

static struct fixed_voltage_config wm8994_fixed_voltage2_config = {
        .supply_name    = "VDD_3.3V",
        .microvolts     = 3300000,
        .gpio           = -EINVAL,
        .init_data      = &wm8994_fixed_voltage2_init_data,
};

static struct platform_device wm8994_fixed_voltage0 = {
        .name           = "reg-fixed-voltage",
        .id             = 0,
        .dev            = {
                .platform_data  = &wm8994_fixed_voltage0_config,
        },
};

static struct platform_device wm8994_fixed_voltage1 = {
        .name           = "reg-fixed-voltage",
        .id             = 1,
        .dev            = {
                .platform_data  = &wm8994_fixed_voltage1_config,
        },
};

static struct platform_device wm8994_fixed_voltage2 = {
        .name           = "reg-fixed-voltage",
        .id             = 2,
        .dev            = {
                .platform_data  = &wm8994_fixed_voltage2_config,
        },
};

static struct regulator_consumer_supply wm8994_avdd1_supply =
        REGULATOR_SUPPLY("AVDD1", "1-001a");

static struct regulator_consumer_supply wm8994_dcvdd_supply =
        REGULATOR_SUPPLY("DCVDD", "1-001a");

static struct regulator_init_data wm8994_ldo1_data = {
        .constraints    = {
                .name           = "AVDD1",
        },
        .num_consumer_supplies  = 1,
        .consumer_supplies      = &wm8994_avdd1_supply,
};

static struct regulator_init_data wm8994_ldo2_data = {
        .constraints    = {
                .name           = "DCVDD",
        },
        .num_consumer_supplies  = 1,
        .consumer_supplies      = &wm8994_dcvdd_supply,
};

static struct wm8994_pdata wm8994_platform_data = {
        /* configure gpio1 function: 0x0001(Logic level input/output) */
        .gpio_defaults[0] = 0x0001,
        /* If the i2s0 and i2s2 is enabled simultaneously */
        .gpio_defaults[7] = 0x8100, /* GPIO8  DACDAT3 in */
        .gpio_defaults[8] = 0x0100, /* GPIO9  ADCDAT3 out */
        .gpio_defaults[9] = 0x0100, /* GPIO10 LRCLK3  out */
        .gpio_defaults[10] = 0x0100,/* GPIO11 BCLK3   out */
        .ldo[0] = { 0, &wm8994_ldo1_data },
        .ldo[1] = { 0, &wm8994_ldo2_data },
};
#endif

static struct i2c_board_info smdk4x12_i2c_devs0[] __initdata = {
	{
		I2C_BOARD_INFO("s5p_ddc", (0x74 >> 1)),    
	},
};

#ifdef CONFIG_TOUCHSCREEN_FT5X0X
#include <plat/ft5x0x_touch.h>
static struct ft5x0x_i2c_platform_data ft5x0x_pdata = {
	.gpio_irq		= EXYNOS4_GPX0(4),
	.irq_cfg		= S3C_GPIO_SFN(0xf),
	.screen_max_x	= 1024,
	.screen_max_y	= 768,
	.pressure_max	= 255,
};
#endif

static struct i2c_board_info smdk4x12_i2c_devs1[] __initdata = {
	{
#ifdef CONFIG_REGULATOR_S5M8767
		I2C_BOARD_INFO("sec_pmic", 0xCC >> 1),
		.platform_data = &exynos4_s5m8767_pdata,
		.irq		= IRQ_EINT(15),

#else
#ifdef CONFIG_REGULATOR_MAX8997
		.platform_data = &exynos4_max8997_info,
#endif
#endif
	}
};

static struct i2c_board_info smdk4x12_i2c_devs2[] __initdata = {
	/* nothing here yet */
};


static struct s5p_platform_mipi_csis mipi_csis_platdata = {
#ifdef CONFIG_VIDEO_S5K6A3
	.clk_rate	= 160000000UL,
	.lanes		= 1,
	.alignment	= 24,
	.hs_settle	= 12,
	.phy_enable	= s5p_csis_phy_enable,
#endif
#ifdef CONFIG_VIDEO_M5MOLS
	.clk_rate	= 166000000UL,
	.lanes		= 2,
	.alignment	= 32,
	.hs_settle	= 12,
	.phy_enable	= s5p_csis_phy_enable,
#endif
};
#define GPIO_CAM_LEVEL_EN(n)	EXYNOS4_GPX1(2)
#define GPIO_CAM_8M_ISP_INT	EXYNOS4_GPX3(3)	/* XEINT_27 */
#define GPIO_CAM_MEGA_nRST	EXYNOS4_GPX1(2) 
static int m5mols_set_power(struct device *dev, int on)
{
	gpio_set_value(EXYNOS4_GPX1(2), !on);
	gpio_set_value(EXYNOS4_GPX1(2), !!on);
	return 0;
}
static struct m5mols_platform_data m5mols_platdata = {
	.gpio_reset	= GPIO_CAM_MEGA_nRST,
	.reset_polarity	= 0,
	.set_power	= m5mols_set_power,
};
static struct i2c_board_info m5mols_board_info = {
	I2C_BOARD_INFO("M5MOLS", 0x1F),
	.platform_data = &m5mols_platdata,
};

#ifdef CONFIG_VIDEO_S5K6A3
static struct i2c_board_info s5k6a3_sensor_info = {
        .type = "S5K6A3",
};
#endif

//#ifdef CONFIG_VIDEO_S5K6A3
#if defined(CONFIG_VIDEO_S5K6A3) || defined(CONFIG_VIDEO_SR200PC20) || defined(CONFIG_VIDEO_OV5640)
static int smdk4x12_cam1_reset(int dummy) 
{
	/* remove by cym 20140602 */
#if 0
        int err;

        /* Camera B */

        err = gpio_request(EXYNOS4_GPX1(0), "GPX1");
        if (err)
                printk(KERN_ERR "#### failed to request GPX1_0 ####\n");

        s3c_gpio_setpull(EXYNOS4_GPX1(0), S3C_GPIO_PULL_NONE);
        gpio_direction_output(EXYNOS4_GPX1(0), 0);
        gpio_direction_output(EXYNOS4_GPX1(0), 1);
        gpio_free(EXYNOS4_GPX1(0));
#endif
	/* end remove */

        return 0;
}

static struct s3c_platform_camera s5k6a3 = {
        .id             = CAMERA_CSI_D,
        .clk_name       = "sclk_cam1",
        .cam_power      = smdk4x12_cam1_reset,
        .type           = CAM_TYPE_MIPI,
        .fmt            = MIPI_CSI_RAW10,
        .order422       = CAM_ORDER422_8BIT_YCBYCR,
        .pixelformat    = V4L2_PIX_FMT_UYVY,
        .line_length    = 1920,
        .width          = 1920,
        .height         = 1080,
        .window         = {
                .left   = 0,
                .top    = 0,
                .width  = 1920,
                .height = 1080,
        },
        .srclk_name     = "xusbxti",
        .clk_rate       = 24000000,
        .mipi_lanes     = 1,
        .mipi_settle    = 12,
        .mipi_align     = 24,

        .initialized    = 0,
        .flite_id       = FLITE_IDX_B,
        .use_isp        = true,
        .sensor_index   = 102,
	.type  		= CAM_TYPE_MIPI,
        .use_isp 	= true,
        .inv_pclk 	= 0,
        .inv_vsync 	= 0,
        .inv_href 	= 0,
        .inv_hsync 	= 0,
};

/* add by cym 20140124 */
#ifdef CONFIG_VIDEO_OV5640
struct soc_camera_device ov5640_plat = {
		.user_width = 640,
		.user_height = 480,
};
static struct i2c_board_info  ov5640_i2c_info = {
	I2C_BOARD_INFO("ov5640", 0x78>>1),
	.platform_data = &ov5640_plat,
};

static struct s3c_platform_camera ov5640 = {
			.id 	= CAMERA_PAR_A,
			.clk_name	= "sclk_cam0",
			.i2c_busnum = 7,
			.cam_power	= smdk4x12_cam1_reset,

		.type		= CAM_TYPE_ITU,
		.fmt		= ITU_601_YCBCR422_8BIT,
		.order422	= CAM_ORDER422_8BIT_CBYCRY,
		.info		= &ov5640_i2c_info,
		.pixelformat	= V4L2_PIX_FMT_UYVY, //modify by cym V4L2_PIX_FMT_UYVY,
		.srclk_name = "xusbxti",
		.clk_rate	= 24000000,
		.line_length	= 1920,
		.width		= 640,
		.height 	= 480,
		.window 	= {
			.left	= 0,
			.top	= 0,
			.width	= 640,
			.height = 480,
		},
	
		/* Polarity */
		.inv_pclk	= 0,
		.inv_vsync	= 1,
		.inv_href	= 0,
		.inv_hsync	= 0,
		.reset_camera	= 1,
		.initialized	= 0,
              //.layout_rotate = 0 //for shuping, //180, 
};

#endif
/* end add */

static struct s3c_platform_fimc fimc_plat = {
	.default_cam    = CAMERA_CSI_D,
	.camera         = {
			//&s5k6a3, //remove by cym 
#ifdef CONFIG_VIDEO_OV5640
		&ov5640,
#endif
	},
};

#endif
static struct s5p_fimc_isp_info smdk4x12_camera_sensors[] = {
#ifdef CONFIG_VIDEO_S5K6A3
	{
                .board_info     = &s5k6a3_sensor_info,
                .clk_frequency  = 24000000UL,
                .bus_type       = FIMC_MIPI_CSI2,
		.i2c_bus_num    = 1,
                .mux_id         = 1, /* A-Port : 0, B-Port : 1 */
                .flite_id       = FLITE_IDX_B,
                .cam_power      = smdk4x12_cam1_reset,
		.flags          = 0,
                .csi_data_align = 24,
                .use_isp        = true,
        },
#endif
#ifdef CONFIG_VIDEO_M5MOLS
	{
		.mux_id		= 0,
		.flags		= V4L2_MBUS_PCLK_SAMPLE_FALLING |
				  V4L2_MBUS_VSYNC_ACTIVE_LOW,
		.bus_type	= FIMC_MIPI_CSI2,
		.board_info	= &m5mols_board_info,
		.i2c_bus_num	= 4,
		.clk_frequency	= 24000000UL,
		.csi_data_align	= 32,
	},
#endif
};
static struct s5p_platform_fimc fimc_md_platdata = {
	.isp_info	= smdk4x12_camera_sensors,
	.num_clients	= ARRAY_SIZE(smdk4x12_camera_sensors),
//#ifdef CONFIG_VIDEO_S5K6A3
#if defined(CONFIG_VIDEO_S5K6A3) || defined(CONFIG_VIDEO_OV5640)
	.fimc_plat	= &fimc_plat,
#endif
};

static struct gpio smdk4x12_camera_gpios[] = {
	{ GPIO_CAM_8M_ISP_INT,	GPIOF_IN,            "8M_ISP_INT"  },
	{ GPIO_CAM_MEGA_nRST,	GPIOF_OUT_INIT_LOW,  "CAM_8M_NRST" },
};
static void __init smdk4x12_camera_init(void)
{
	s3c_set_platdata(&mipi_csis_platdata, sizeof(mipi_csis_platdata),
			 &s5p_device_mipi_csis0);
	s3c_set_platdata(&mipi_csis_platdata, sizeof(mipi_csis_platdata),
                         &s5p_device_mipi_csis1);
	s3c_set_platdata(&fimc_md_platdata,  sizeof(fimc_md_platdata),
			 &s5p_device_fimc_md);
	
#if 0
	if (gpio_request_array(smdk4x12_camera_gpios,
			       ARRAY_SIZE(smdk4x12_camera_gpios))) {
		pr_err("%s: GPIO request failed\n", __func__);
		return;
	}

	if (!s3c_gpio_cfgpin(GPIO_CAM_8M_ISP_INT, S3C_GPIO_SFN(0xf)))
	{
        	s3c_gpio_setpull(GPIO_CAM_8M_ISP_INT, S3C_GPIO_PULL_NONE);
		m5mols_board_info.irq = gpio_to_irq(GPIO_CAM_8M_ISP_INT);
	}
	else
		pr_err("Failed to configure 8M_ISP_INT GPIO\n");

	/* Free GPIOs controlled directly by the sensor drivers. */
	gpio_free(GPIO_CAM_MEGA_nRST);
	gpio_free(GPIO_CAM_8M_ISP_INT);
#endif
	if (exynos4_fimc_setup_gpio(S5P_CAMPORT_A))
		pr_err("Camera port A setup failed\n");
}

#ifdef CONFIG_MXC_MMA845X
#include <linux/mma845x.h>

static struct mxc_mma845x_platform_data mma845x_data = {
	.gpio_pin_get = NULL,
	.gpio_pin_put = NULL,
	.int1 = IRQ_EINT(25),	// ACCL_INT1 is gpio for MMA845X INT1
	.int2 = 0,				// ACCL_INT2 is gpio for MMA845X INT2
};
#endif

static struct i2c_board_info smdk4x12_i2c_devs3[] __initdata = {
#ifdef CONFIG_MXC_MMA845X
	{
		.type = "mma845x",
		.addr = 0x1D,		/*mma845x i2c slave address*/
		.platform_data = (void *)&mma845x_data,
	},
#endif

#if defined(CONFIG_TOUCHSCREEN_FT5X0X)
        {
                I2C_BOARD_INFO("ft5x0x_ts", 0x70>>1),
                .irq = IRQ_EINT(4),
                .platform_data = &ft5x0x_pdata,
        },
#endif
};

#ifdef CONFIG_SND_SOC_WM8960
#include <sound/wm8960.h>
static struct wm8960_data wm8960_pdata = {
	.capless		= 0,
	.dres			= WM8960_DRES_400R,
};
#endif

/* I2C4 */
static struct i2c_board_info smdk4x12_i2c_devs4[] __initdata = {
#ifdef CONFIG_SND_SOC_WM8960
	{
		I2C_BOARD_INFO("wm8960", 0x1a),
		.platform_data	= &wm8960_pdata,
	},
#endif
};

/* I2C5 */
static struct i2c_board_info smdk4x12_i2c_devs5[] __initdata = {
#ifdef CONFIG_MPU_SENSORS_MPU6050B1
	{
                I2C_BOARD_INFO("mpu6050", 0x68),
                .platform_data = &mpu6050_data,
                .irq = EXYNOS4_GPX3(3),
        },
/*
        {
                I2C_BOARD_INFO("ak8975", 0x0C),
                .platform_data = &inv_mpu_compass_data,
                .irq = EXYNOS4_GPX1(4),
        }
*/
#else
#if 1
	// gyro
	{
		I2C_BOARD_INFO(MPU_NAME, 0x68),
		.irq = IRQ_EINT(27),
		.platform_data = &mpu3050_data,
	},
	// accel
	{
		I2C_BOARD_INFO("bma250", (0x30>>1)),
		//.irq = IRQ_EINT(24),// 25?
		.platform_data = &inv_mpu_bma250_data,
	},
#endif
#endif
};
/* end add */

/* I2C module and id for HDMIPHY */
static struct i2c_board_info smdk4x12_i2c_hdmiphy[] __initdata = {
	{ I2C_BOARD_INFO("hdmiphy-exynos4412", 0x38), }
};

static void s5p_tv_setup(void)
{
	/* direct HPD to External Interrupt */
	WARN_ON(gpio_request_one(EXYNOS4_GPX3(7), GPIOF_IN, "hpd-plug"));
	s3c_gpio_cfgpin(EXYNOS4_GPX3(7), S3C_GPIO_SFN(0xf));
	s3c_gpio_setpull(EXYNOS4_GPX3(7), S3C_GPIO_PULL_NONE);
	gpio_free(EXYNOS4_GPX3(7));
}

static struct i2c_board_info smdk4x12_i2c_devs7[] __initdata = {
};

static struct samsung_bl_gpio_info smdk4x12_bl_gpio_info = {
	.no = EXYNOS4_GPD0(1),
	.func = S3C_GPIO_SFN(2),
};

static struct platform_pwm_backlight_data smdk4x12_bl_data = {
	.pwm_id = 1,
	.pwm_period_ns  = 1000,
};

static struct s3c_fb_pd_win smdk4x12_fb_win0 = {
	.xres			= 480,
	.yres			= 800,
	.virtual_x		= 480,
	.virtual_y		= 800 * CONFIG_FB_S3C_NR_BUFFERS,
	.max_bpp		= 32,
	.default_bpp	= 24,
	.width			= 66,
	.height			= 109,
};

static struct s3c_fb_pd_win smdk4x12_fb_win1 = {
	.xres			= 480,
	.yres			= 800,
	.virtual_x		= 480,
	.virtual_y		= 800 * CONFIG_FB_S3C_NR_BUFFERS,
	.max_bpp		= 32,
	.default_bpp	= 24,
	.width			= 66,
	.height			= 109,
};

static struct s3c_fb_pd_win smdk4x12_fb_win2 = {
	.xres			= 480,
	.yres			= 800,
	.virtual_x		= 480,
	.virtual_y		= 800 * CONFIG_FB_S3C_NR_BUFFERS,
	.max_bpp		= 32,
	.default_bpp	= 24,
	.width			= 66,
	.height			= 109,
};

static struct s3c_fb_pd_win smdk4x12_fb_win3 = {
	.xres			= 480,
	.yres			= 800,
	.virtual_x		= 480,
	.virtual_y		= 800 * CONFIG_FB_S3C_NR_BUFFERS,
	.max_bpp		= 32,
	.default_bpp	= 24,
	.width			= 66,
	.height			= 109,
};

static struct s3c_fb_pd_win smdk4x12_fb_win4 = {
	.xres			= 480,
	.yres			= 800,
	.virtual_x		= 480,
	.virtual_y		= 800 * CONFIG_FB_S3C_NR_BUFFERS,
	.max_bpp		= 32,
	.default_bpp	= 24,
	.width			= 66,
	.height			= 109,
};

static struct fb_videomode smdk4x12_lcd_timing = {
	.left_margin	= 9,
	.right_margin	= 9,
	.upper_margin	= 5,
	.lower_margin	= 5,
	.hsync_len	= 2,
	.vsync_len	= 2,
	.xres		= 480,
	.yres		= 800,
};

static struct s3c_fb_platdata smdk4x12_lcd0_pdata __initdata = {
	.win[0]		= &smdk4x12_fb_win0,
	.win[1]		= &smdk4x12_fb_win1,
	.win[2]		= &smdk4x12_fb_win2,
	.win[3]		= &smdk4x12_fb_win3,
	.win[4]		= &smdk4x12_fb_win4,
	.vtiming	= &smdk4x12_lcd_timing,
	.vidcon0	= VIDCON0_VIDOUT_RGB | VIDCON0_PNRMODE_RGB,
	.vidcon1	= VIDCON1_INV_HSYNC | VIDCON1_INV_VSYNC,
	.setup_gpio	= exynos4_fimd0_gpio_setup_24bpp,
};

static struct s3cfb_lcd wal101 = {
	.width = 1024,
	.height = 768,
	.p_width = 200,
	.p_height = 150,
	.bpp = 24,//32,
	.freq = 60,//70,//61,
	
	.timing = {
		.h_fp	= 48,
		.h_bp	= 80,
		.h_sw	= 32,
		.v_fp	= 3,
		.v_fpe	= 0,
		.v_bp	= 14,
		.v_bpe	= 0,
		.v_sw	= 5,
	},
	.polarity = {
		.rise_vclk	= 1,
		.inv_hsync	= 0,
		.inv_vsync	= 1,
		.inv_vden	= 0,
	},
};

static int __init s3cfb_setup_lcd(char *str)
{
        //int i;

        if (!strncasecmp("9.7", str, 3)) {
                //printk("000000000000000000000000\n");
                wal101.width = 1024;
                wal101.height = 768;
                wal101.bpp       = 24;
                wal101.freq = 60;//70;

		ft5x0x_pdata.screen_max_x = 1024;
                ft5x0x_pdata.screen_max_y = 768;
        }
        else if(!strncasecmp("7.0", str, 3))
        {
                //printk("1111111111111111111111111\n");
                wal101.width = 800;
                wal101.height = 1280;
                wal101.bpp       = 24;
                wal101.freq = 50;//70;

		ft5x0x_pdata.screen_max_x = 1280;//1280;
                ft5x0x_pdata.screen_max_y = 800;//800;
        }

        printk("%s selected\n", __FUNCTION__);
        return 0;
}

early_param("lcd", s3cfb_setup_lcd);

static void __init fb_init_pdata(struct s3c_fb_platdata *pd) {
	struct s3cfb_lcd *lcd;
	struct s3c_fb_pd_win *win;
	struct fb_videomode *mode = pd->vtiming;
	unsigned long val = 0;
	u64 pixclk = 1000000000000ULL;
	u32 div;
	int i;

	lcd = &wal101;

	for (i = 0; i < S3C_FB_MAX_WIN; i++) {
		if (pd->win[i] == NULL)
			continue;

		win = pd->win[i];
		win->xres		= lcd->width;
		win->yres		= lcd->height;
		win->default_bpp= lcd->bpp ? : 24;
		win->virtual_x	= win->xres;
		win->virtual_y	= win->yres * CONFIG_FB_S3C_NR_BUFFERS;
		win->width		= lcd->p_width;
		win->height		= lcd->p_height;
	}

	mode->left_margin	= lcd->timing.h_bp;
	mode->right_margin	= lcd->timing.h_fp;
	mode->upper_margin	= lcd->timing.v_bp;
	mode->lower_margin	= lcd->timing.v_fp;
	mode->hsync_len		= lcd->timing.h_sw;
	mode->vsync_len		= lcd->timing.v_sw;
	mode->xres			= lcd->width;
	mode->yres			= lcd->height;

	/* calculates pixel clock */
	div  = mode->left_margin + mode->hsync_len + mode->right_margin +
		mode->xres;
	div *= mode->upper_margin + mode->vsync_len + mode->lower_margin +
		mode->yres;
	div *= lcd->freq ? : 60;

	do_div(pixclk, div);

	mode->pixclock		= pixclk + 386;

	/* initialize signal polarity of RGB interface */
	if (lcd->polarity.rise_vclk)
		val |= VIDCON1_INV_VCLK;
	if (lcd->polarity.inv_hsync)
		val |= VIDCON1_INV_HSYNC;
	if (lcd->polarity.inv_vsync)
		val |= VIDCON1_INV_VSYNC;
	if (lcd->polarity.inv_vden)
		val |= VIDCON1_INV_VDEN;

	pd->vidcon1 = val;

	/* add by cym */
#if 0
        {
                int err;

                err = gpio_request(EXYNOS4_GPL1(0), "GPL1_0");
                if (err) {
                        printk(KERN_ERR "failed to request GPL1 for "
                                "lcd power control\n");
                        return;
                }
                gpio_direction_output(EXYNOS4_GPL1(0), 1);

                s3c_gpio_cfgpin(EXYNOS4_GPL1(0), S3C_GPIO_OUTPUT);
                gpio_free(EXYNOS4_GPL1(0));

                mdelay(5);
#if 0
                err = gpio_request(EXYNOS4_GPD0(1), "GPD0_1");
                if (err) {
                        printk(KERN_ERR "failed to request GPD0_1 for "
                                "lcd power control\n");
                        return;
                }
                gpio_direction_output(EXYNOS4_GPD0(1), 1);
                s3c_gpio_cfgpin(EXYNOS4_GPD0(1), S3C_GPIO_OUTPUT);
                gpio_free(EXYNOS4_GPD0(1));
#endif
        }
#endif
        /* end add */
}

#ifdef CONFIG_S3C64XX_DEV_SPI0
static struct s3c64xx_spi_csinfo spi0_csi[] = {
	[0] = {
		.line = EXYNOS4_GPB(1),
		.fb_delay = 0x0,
	},
};

static struct spi_board_info spi0_board_info[] __initdata = {
	{
		.modalias = "spidev",
		.platform_data = NULL,
		.max_speed_hz = 10*1000*1000,
		.bus_num = 0,
		.chip_select = 0,
		.mode = SPI_MODE_0,
		.controller_data = &spi0_csi[0],
	}
};
#endif

#ifdef CONFIG_S3C64XX_DEV_SPI1
static struct s3c64xx_spi_csinfo spi1_csi[] = {
	[0] = {
		.line = EXYNOS4_GPB(5),
		.fb_delay = 0x0,
	},
};

static struct spi_board_info spi1_board_info[] __initdata = {
	{
		.modalias = "spidev",
		.platform_data = NULL,
		.max_speed_hz = 10*1000*1000,
		.bus_num = 1,
		.chip_select = 0,
		.mode = SPI_MODE_0,
		.controller_data = &spi1_csi[0],
	}
};
#endif

#ifdef CONFIG_S3C64XX_DEV_SPI2
static struct s3c64xx_spi_csinfo spi2_csi[] = {
	[0] = {
		.line = EXYNOS4_GPC1(2),
		.fb_delay = 0x0,
	},
};

static struct spi_board_info spi2_board_info[] __initdata = {
	{
		.modalias = "spidev",
		.platform_data = NULL,
		.max_speed_hz = 10*1000*1000,
		.bus_num = 2,
		.chip_select = 0,
		.mode = SPI_MODE_0,
		.controller_data = &spi2_csi[0],
	}
};
#endif

#ifdef CONFIG_LCD_LMS501KF03
static int lcd_power_on(struct lcd_device *ld, int enable)
{
	return 1;
}

static int reset_lcd(struct lcd_device *ld)
{
	int err = 0;
	err = gpio_request_one(EXYNOS4X12_GPM3(6),
			GPIOF_OUT_INIT_HIGH, "GPM3");
	if (err) {
		pr_err("failed to request GPM3 for lcd reset control\n");
		return err;
	}
	gpio_set_value(EXYNOS4X12_GPM3(6), 0);
	mdelay(1);
	gpio_set_value(EXYNOS4X12_GPM3(6), 1);
	gpio_free(EXYNOS4X12_GPM3(6));

	return 1;
}

static struct lcd_platform_data lms501kf03_platform_data = {
	.reset			= reset_lcd,
	.power_on		= lcd_power_on,
	.lcd_enabled	= 0,
	.reset_delay	= 100,  /* 100ms */
};

#define LCD_BUS_NUM		3
static struct spi_board_info spi_board_info[] __initdata = {
	{
		.modalias		= "lms501kf03",
		.platform_data	= (void *)&lms501kf03_platform_data,
		.max_speed_hz	= 1200000,
		.bus_num		= LCD_BUS_NUM,
		.chip_select	= 0,
		.mode			= SPI_MODE_3,
		.controller_data	= (void *)EXYNOS4_GPB(5),
	}
};

static struct spi_gpio_platform_data lms501kf03_spi_gpio_data = {
	.sck	= EXYNOS4_GPB(4), /* DISPLAY_CLK */
	.mosi	= EXYNOS4_GPB(7), /* DISPLAY_SI */
	.miso	= SPI_GPIO_NO_MISO,
	.num_chipselect = 1,
};

static struct platform_device s3c_device_spi_gpio = {
	.name	= "spi_gpio",
	.id		= LCD_BUS_NUM,
	.dev	= {
		.parent			= &s5p_device_fimd0.dev,
		.platform_data	= &lms501kf03_spi_gpio_data,
	},
};
#endif

#ifdef CONFIG_EXYNOS4_DEV_DWMCI
static void exynos_dwmci_cfg_gpio(int width)
{
	unsigned int gpio;

	for (gpio = EXYNOS4_GPK0(0); gpio < EXYNOS4_GPK0(2); gpio++) {
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(3));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
		s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV2);
	}

	switch (width) {
	case MMC_BUS_WIDTH_8:
		for (gpio = EXYNOS4_GPK1(3); gpio <= EXYNOS4_GPK1(6); gpio++) {
			s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(4));
			s3c_gpio_setpull(gpio, S3C_GPIO_PULL_UP);
			s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV2);
		}
	case MMC_BUS_WIDTH_4:
		for (gpio = EXYNOS4_GPK0(3); gpio <= EXYNOS4_GPK0(6); gpio++) {
			s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(3));
			s3c_gpio_setpull(gpio, S3C_GPIO_PULL_UP);
			s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV2);
		}
		break;
	case MMC_BUS_WIDTH_1:
		gpio = EXYNOS4_GPK0(3);
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(3));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_UP);
		s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV2);
	default:
		break;
	}
}

static struct dw_mci_board exynos_dwmci_pdata __initdata = {
	.num_slots		= 1,
	.quirks			= DW_MCI_QUIRK_BROKEN_CARD_DETECTION | DW_MCI_QUIRK_HIGHSPEED,
	.bus_hz			= 100 * 1000 * 1000,
	.caps			= MMC_CAP_UHS_DDR50 | MMC_CAP_1_8V_DDR |
				MMC_CAP_8_BIT_DATA | MMC_CAP_CMD23,
	.fifo_depth		= 0x80,
	.detect_delay_ms	= 200,
	.hclk_name		= "dwmci",
	.cclk_name		= "sclk_dwmci",
	.cfg_gpio		= exynos_dwmci_cfg_gpio,
};
#endif

static int exynos_boot_dev;

#define is_bootfrom_emmc()	\
	((exynos_boot_dev == 0x6) || (exynos_boot_dev == 0x7))
#define is_bootfrom_sd()	\
	 (exynos_boot_dev == 0x3)

static void __init exynos_bootdev_init(void)
{
	u32 capboot = MMC_CAP2_BOOT_DEVICE;

	exynos_boot_dev = __raw_readl(S5P_INFORM3);

	if (is_bootfrom_emmc()) {
#if defined(CONFIG_EXYNOS4_DEV_DWMCI)
		exynos_dwmci_pdata.caps2 |= capboot;
#endif
	} else if (is_bootfrom_sd()) {
		smdk4x12_hsmmc2_pdata.host_caps2 |= capboot;
	} else {
		/* oops...should never fly to here */
		printk(KERN_ERR "Unknown boot device\n");
	}
}

static struct platform_device itop4412_device_adc = {
	.name			= "itop4412_adc",
	.id				= -1,
	.num_resources	= 0,
};

#ifdef CONFIG_KEYBOARD_GPIO
static struct gpio_keys_button gpio_buttons[] = {
	{
		.gpio		= EXYNOS4_GPX1(1),
		//.code		= 10,
		.code		= KEY_HOMEPAGE,
		.desc		= "BUTTON1",
		.active_low	= 1,
		.wakeup		= 0,
	},
	{
		.gpio		= EXYNOS4_GPX1(2),
		//.code		= 24,
		.code		= KEY_BACK,
		.desc		= "BUTTON2",
		.active_low	= 1,
		.wakeup		= 0,
	},
	{
		.gpio		= EXYNOS4_GPX3(2),
		//.code		= 38,
		.code		= KEY_POWER,
		.desc		= "BUTTON3",
		.active_low	= 1,
		.wakeup		= 0,
	}
};

static struct gpio_keys_platform_data gpio_button_data = {
	.buttons	= gpio_buttons,
	.nbuttons	= ARRAY_SIZE(gpio_buttons),
};

static struct platform_device gpio_button_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.num_resources	= 0,
	.dev		= {
		.platform_data	= &gpio_button_data,
	}
};
#endif


#ifdef CONFIG_SAMSUNG_DEV_KEYPAD
static uint32_t smdk4x12_keymap[] __initdata = {
	/* KEY(row, col, keycode) */
	KEY(0, 0, KEY_VOLUMEUP), KEY(1, 0, KEY_VOLUMEDOWN), KEY(0, 2, KEY_HOME), KEY(0, 3, KEY_MENU), KEY(0, 4, KEY_BACK),KEY(0, 5, KEY_POWER)	//volume up ---volume down
};

static struct matrix_keymap_data smdk4x12_keymap_data __initdata = {
	.keymap			= smdk4x12_keymap,
	.keymap_size	= ARRAY_SIZE(smdk4x12_keymap),
};

static struct samsung_keypad_platdata smdk4x12_keypad_data __initdata = {
	.keymap_data	= &smdk4x12_keymap_data,
	.rows			= 2,
	.cols			= 6,
};
#endif

/* USB OTG */
static struct s3c_hsotg_plat smdk4x12_hsotg_pdata;

static struct platform_device exynos4_bus_devfreq = {
	.name		= "exynos4412-busfreq",
	.id			= 1,
};

/* USB EHCI */
static struct s5p_ehci_platdata smdk4x12_ehci_pdata;

static void __init smdk4x12_ehci_init(void)
{
	struct s5p_ehci_platdata *pdata = &smdk4x12_ehci_pdata;
	int err;

	s5p_ehci_set_platdata(pdata);

#define GPIO_USBH_RESET		EXYNOS4X12_GPM2(4)
#define GPIO_HUB_CONNECT EXYNOS4X12_GPM3(3)
	err = gpio_request_one(GPIO_USBH_RESET,
			GPIOF_OUT_INIT_HIGH, "USBH_RESET");
	if (err)
		pr_err("failed to request GPM2_4 for USB reset control\n");

	s3c_gpio_setpull(GPIO_USBH_RESET, S3C_GPIO_PULL_UP);
	gpio_set_value(GPIO_USBH_RESET, 0);
	mdelay(1);
	gpio_set_value(GPIO_USBH_RESET, 1);
	gpio_free(GPIO_USBH_RESET);

	// HUB_CONNECT
	gpio_request(GPIO_HUB_CONNECT, "GPIO_HUB_CONNECT");
	gpio_direction_output(GPIO_HUB_CONNECT, 1);
	s3c_gpio_setpull(GPIO_HUB_CONNECT, S3C_GPIO_PULL_NONE);
	gpio_free(GPIO_HUB_CONNECT);

/* add by cym 20140115 for USI 3G Power On*/
#ifdef CONFIG_UNAPLUS
#if 1
        if(gpio_request(EXYNOS4_GPK1(0), "GPK1_0"))
        {
                printk(KERN_ERR "failed to request GPK1_0 for "
                        "USI control\n");
                //return err;
        }
        gpio_direction_output(EXYNOS4_GPK1(0), 1);

        s3c_gpio_cfgpin(EXYNOS4_GPK1(0), S3C_GPIO_OUTPUT);
        gpio_free(EXYNOS4_GPK1(0));
        mdelay(5);
#endif
#endif
/* end add */
}

/* USB OHCI */
static struct exynos4_ohci_platdata smdk4x12_ohci_pdata;

static void __init smdk4x12_ohci_init(void)
{
	struct exynos4_ohci_platdata *pdata = &smdk4x12_ohci_pdata;

	exynos4_ohci_set_platdata(pdata);
}

#ifdef CONFIG_BUSFREQ_OPP
/* BUSFREQ to control memory/bus*/
static struct device_domain busfreq;
#endif
static struct platform_device exynos4_busfreq = {
	.id = -1,
	.name = "exynos-busfreq",
};

#ifdef CONFIG_BATTERY_SAMSUNG
static struct platform_device samsung_device_battery = {
	.name	= "samsung-fake-battery",
	.id = -1,
};
#endif

/* add by cym 20140527 */
struct platform_device s3c_device_gps = {
        .name   = "si_gps",
        .id             = -1,
};

#ifdef CONFIG_MAX485_CTL
struct platform_device s3c_device_max485_ctl = {
        .name   = "max485_ctl",
        .id             = -1,
};
#endif

#ifdef CONFIG_LEDS_CTL
struct platform_device s3c_device_leds_ctl = {
        .name   = "leds",
        .id             = -1,
};
#endif

#ifdef CONFIG_BUZZER_CTL
struct platform_device s3c_device_buzzer_ctl = {
        .name   = "buzzer_ctl",
        .id             = -1,
};
#endif

#ifdef CONFIG_ADC_CTL
struct platform_device s3c_device_adc_ctl = {
	.name			= "adc_ctl",
	.id				= -1,
};
#endif

/* end add */

static struct platform_device *smdk4x12_devices[] __initdata = {
#ifdef CONFIG_EXYNOS4_DEV_DWMCI
	&exynos_device_dwmci,
#endif
	&s3c_device_hsmmc2,
	&s3c_device_hsmmc3,
#ifdef CONFIG_SND_SOC_SAMSUNG_SMDK_WM8994
	&wm8994_fixed_voltage0,
	&wm8994_fixed_voltage1,
	&wm8994_fixed_voltage2,
#endif
	&s3c_device_i2c0,
	&s3c_device_i2c1,
	/* remove by cym 20140602 */
#if 0
	&s3c_device_i2c2,
#endif
	/* end remove */
	&s3c_device_i2c3,
/* modify by cym 20140318 */
#if 0
#ifdef CONFIG_VIDEO_M5MOLS
	&s3c_device_i2c4,
#endif
#else
	&s3c_device_i2c4,
#endif
/* end modify */
	
	&s3c_device_i2c5,

	&s3c_device_i2c7,
	&s3c_device_adc,
	&s3c_device_rtc,
	&s3c_device_wdt,
#ifdef CONFIG_ITOP4412_BUZZER
	&s3c_device_timer[0],
#endif
#ifdef CONFIG_VIDEO_EXYNOS_FIMC_LITE
	&exynos_device_flite0,
	&exynos_device_flite1,
#endif
	&s5p_device_mipi_csis0,
	&s5p_device_mipi_csis1,
	&s5p_device_fimc0,
	&s5p_device_fimc1,
	&s5p_device_fimc2,
	&s5p_device_fimc3,
	&s5p_device_fimc_md,
	&s5p_device_fimd0,
	&mali_gpu_device,
	&s5p_device_mfc,
	&s5p_device_mfc_l,
	&s5p_device_mfc_r,
	&s5p_device_jpeg,
#ifdef CONFIG_SAMSUNG_DEV_KEYPAD
	&samsung_device_keypad,
#endif
	&itop4412_device_adc,
#ifdef CONFIG_VIDEO_EXYNOS_FIMC_IS
	&exynos4_device_fimc_is,
#endif
#ifdef CONFIG_LCD_LMS501KF03
	&s3c_device_spi_gpio,
#endif
#ifdef CONFIG_S3C64XX_DEV_SPI0
	&s3c64xx_device_spi0,
#endif
#ifdef CONFIG_S3C64XX_DEV_SPI1
	&s3c64xx_device_spi1,
#endif
#ifdef CONFIG_S3C64XX_DEV_SPI2
	&s3c64xx_device_spi2,
#endif
#ifdef CONFIG_ION_EXYNOS
	&exynos_device_ion,
#endif
	&s5p_device_i2c_hdmiphy,
	&s5p_device_hdmi,
	&s5p_device_mixer,
	&exynos4_bus_devfreq,
	&samsung_asoc_dma,
	&samsung_asoc_idma,
#ifdef CONFIG_SND_SAMSUNG_I2S
	&exynos4_device_i2s0,
#endif
#ifdef CONFIG_SND_SAMSUNG_PCM
	&exynos4_device_pcm0,
#endif
#ifdef CONFIG_SND_SAMSUNG_SPDIF
	&exynos4_device_spdif,
#endif
#ifdef CONFIG_VIDEO_EXYNOS_FIMG2D
	&s5p_device_fimg2d,
#endif
#ifdef CONFIG_EXYNOS_THERMAL
	&exynos_device_tmu,
#endif
	&s5p_device_ehci,
	&exynos4_device_ohci,
	&s5p_device_usbswitch,
#if defined CONFIG_SND_SAMSUNG_ALP
	&exynos_device_srp,
#endif
#ifdef CONFIG_BUSFREQ_OPP
	&exynos4_busfreq,
#endif
#ifdef CONFIG_BATTERY_SAMSUNG
	&samsung_device_battery,
#endif

	/* add by cym 20140318 */
#ifdef CONFIG_SMSC911X
	&smdk4x12_smsc911x,
#endif
	/* end add */

#ifdef CONFIG_KEYBOARD_GPIO
	&gpio_button_device,
#endif

	/* add by cym 20140527 */
	&s3c_device_gps,

#ifdef CONFIG_MAX485_CTL
	&s3c_device_max485_ctl ,
#endif

#ifdef CONFIG_LEDS_CTL
        &s3c_device_leds_ctl,
#endif

#ifdef CONFIG_BUZZER_CTL
        &s3c_device_buzzer_ctl,
#endif

#ifdef CONFIG_ADC_CTL
        &s3c_device_adc_ctl,
#endif

	/* end add */
};

static void __init smdk4x12_map_io(void)
{
	clk_xusbxti.rate = 24000000;

	exynos_init_io(NULL, 0);
	s3c24xx_init_clocks(clk_xusbxti.rate);
	s3c24xx_init_uarts(smdk4x12_uartcfgs, ARRAY_SIZE(smdk4x12_uartcfgs));
}

static void __init smdk4x12_reserve(void)
{
	// HACK: This reserved memory will be used for FIMC-IS
	s5p_mfc_reserve_mem(0x58000000, 32<< 20, 0x43000000, 0 << 20);
}

static void smdk4x12_pmu_wdt_init(void)
{
	unsigned int value;

	if (soc_is_exynos4212() || soc_is_exynos4412()) {
		value = __raw_readl(S5P_AUTOMATIC_WDT_RESET_DISABLE);
		value &= ~S5P_SYS_WDTRESET;
		__raw_writel(value, S5P_AUTOMATIC_WDT_RESET_DISABLE);
		value = __raw_readl(S5P_MASK_WDT_RESET_REQUEST);
		value &= ~S5P_SYS_WDTRESET;
		__raw_writel(value, S5P_MASK_WDT_RESET_REQUEST);
	}
}

static void smdk4x12_rtc_wake_init(void)
{
#ifdef CONFIG_PM
	gic_arch_extn.irq_set_wake = s3c_irq_wake;
#endif
}

static struct s3c2410_platform_i2c universal_i2c4_platdata __initdata = {
	.frequency	= 300 * 1000,
	.sda_delay	= 200,
};
#ifdef CONFIG_VIDEO_EXYNOS_FIMC_LITE
static void __set_flite_camera_config(struct exynos_platform_flite *data,
                                        u32 active_index, u32 max_cam)
{       
        data->active_cam_index = active_index;
        data->num_clients = max_cam;
}

static void __init smdk4x12_set_camera_flite_platdata(void)
{
        int flite0_cam_index = 0;
        int flite1_cam_index = 0;
#ifdef CONFIG_VIDEO_S5K6A3
#ifdef CONFIG_S5K6A3_CSI_C
        exynos_flite0_default_data.cam[flite0_cam_index++] = &s5k6a3;
#endif
#ifdef CONFIG_S5K6A3_CSI_D
        exynos_flite1_default_data.cam[flite1_cam_index++] = &s5k6a3;
#endif
#endif
        __set_flite_camera_config(&exynos_flite0_default_data, 0, flite0_cam_index);
        __set_flite_camera_config(&exynos_flite1_default_data, 0, flite1_cam_index);
}
#endif

/* usb phy0 mode */
static int uhost0 = 0;

static int __init setup_uhost(char *str)
{
	if (!strcasecmp(str, "y") || !strcmp(str, "1") ||
		!strcasecmp(str, "yes")) {
		printk("USB PHY0 configured as HOST mode\n");
		uhost0 = 1;
	}

	return 0;
}
early_param("uhost0", setup_uhost);

#ifdef CONFIG_USB_EXYNOS_SWITCH
static struct s5p_usbswitch_platdata smdk4x12_usbswitch_pdata;

static void __init smdk4x12_usbswitch_init(void)
{
	struct s5p_usbswitch_platdata *pdata = &smdk4x12_usbswitch_pdata;
	int err;

	pdata->gpio_host_detect = EXYNOS4_GPX3(5); /* low active */
	err = gpio_request_one(pdata->gpio_host_detect, GPIOF_IN,
							"HOST_DETECT");
	if (err) {
		pr_err("failed to request gpio_host_detect\n");
		return;
	}

	s3c_gpio_cfgpin(pdata->gpio_host_detect, S3C_GPIO_SFN(0xF));
	s3c_gpio_setpull(pdata->gpio_host_detect, S3C_GPIO_PULL_NONE);
	gpio_free(pdata->gpio_host_detect);

	pdata->gpio_device_detect = EXYNOS4_GPX3(4); /* high active */
	err = gpio_request_one(pdata->gpio_device_detect, GPIOF_IN,
							"DEVICE_DETECT");
	if (err) {
		pr_err("failed to request gpio_host_detect for\n");
		return;
	}

	s3c_gpio_cfgpin(pdata->gpio_device_detect, S3C_GPIO_SFN(0xF));
	s3c_gpio_setpull(pdata->gpio_device_detect, S3C_GPIO_PULL_NONE);
	gpio_free(pdata->gpio_device_detect);

	pdata->gpio_host_vbus = EXYNOS4_GPL2(0);
	err = gpio_request_one(pdata->gpio_host_vbus, GPIOF_OUT_INIT_LOW,
							"HOST_VBUS_CONTROL");
	if (err) {
		pr_err("failed to request gpio_host_vbus\n");
		return;
	}

	s3c_gpio_setpull(pdata->gpio_host_vbus, S3C_GPIO_PULL_NONE);
	gpio_free(pdata->gpio_host_vbus);

	s5p_usbswitch_set_platdata(pdata);
}
#endif

#ifdef CONFIG_VIDEO_EXYNOS_FIMG2D
static struct fimg2d_platdata fimg2d_data __initdata = {
	.ip_ver			= IP_VER_G2D_4P,
	.hw_ver			= 0x41,
	.parent_clkname	= "mout_g2d0",
	.clkname		= "sclk_fimg2d",
	.gate_clkname	= "fimg2d",
	.clkrate		= 200 * MHZ,
};
#endif

static int __init exynos4_setup_clock(struct device *dev,
						const char *clock,
						const char *parent,
						unsigned long clk_rate)
{
	struct clk *clk_parent;
	struct clk *sclk;

	sclk = clk_get(dev, clock);
	if (IS_ERR(sclk)) {
		pr_err("Unable to get clock:%s.\n", clock);
		return PTR_ERR(sclk);
	}

	clk_parent = clk_get(NULL, parent);
	if (IS_ERR(clk_parent)) {
		clk_put(sclk);
		pr_err("Unable to get parent clock:%s of clock:%s.\n",
				parent, sclk->name);
		return PTR_ERR(clk_parent);
	}

	if (clk_set_parent(sclk, clk_parent)) {
		pr_err("Unable to set parent %s of clock %s.\n", parent, clock);
		clk_put(sclk);
		clk_put(clk_parent);
		return PTR_ERR(sclk);
	}

	if (clk_rate)
		if (clk_set_rate(sclk, clk_rate)) {
			pr_err("%s rate change failed: %lu\n", sclk->name,
				clk_rate);
			clk_put(sclk);
			clk_put(clk_parent);
			return PTR_ERR(sclk);
		}

	clk_put(sclk);
	clk_put(clk_parent);

	return 0;
}

static void initialize_prime_clocks(void)
{
	exynos4_setup_clock(&s5p_device_fimd0.dev, "sclk_fimd",
                                        "mout_mpll_user", 176 * MHZ);

	exynos4_setup_clock(&s5p_device_fimc0.dev, "sclk_fimc",
					"mout_mpll_user", 176 * MHZ);
	exynos4_setup_clock(&s5p_device_fimc1.dev, "sclk_fimc",
					"mout_mpll_user", 176 * MHZ);
	exynos4_setup_clock(&s5p_device_fimc2.dev, "sclk_fimc",
					"mout_mpll_user", 176 * MHZ);
	exynos4_setup_clock(&s5p_device_fimc3.dev, "sclk_fimc",
					"mout_mpll_user", 176 * MHZ);

	exynos4_setup_clock(&s5p_device_mipi_csis0.dev, "sclk_csis",
					"mout_mpll_user", 176 * MHZ);
	exynos4_setup_clock(&s5p_device_mipi_csis0.dev, "sclk_csis",
					"mout_mpll_user", 176 * MHZ);

	exynos4_setup_clock(NULL, "mout_mfc0", "mout_mpll", 0);
	exynos4_setup_clock(&s5p_device_mfc.dev, "sclk_mfc",
					"mout_mfc0", 220 * MHZ);

	exynos4_setup_clock(NULL, "mout_jpeg0", "mout_mpll", 0);
	exynos4_setup_clock(&s5p_device_mfc.dev, "sclk_jpeg",
					"mout_jpeg0", 176 * MHZ);

	exynos4_setup_clock(&s3c_device_hsmmc2.dev, "dout_mmc2",
					"mout_mpll_user", 100 * MHZ);
#ifdef CONFIG_SND_SAMSUNG_I2S_MASTER
	exynos4_setup_clock(&exynos_device_dwmci.dev, "dout_mmc4",
					"mout_epll", 400 * MHZ);
#else
#ifdef CONFIG_EXYNOS4_DEV_DWMCI
	exynos4_setup_clock(&exynos_device_dwmci.dev, "dout_mmc4",
					"mout_mpll_user", 440 * MHZ);
#endif
#endif
}

static void initialize_non_prime_clocks(void)
{
	exynos4_setup_clock(&s5p_device_fimd0.dev, "sclk_fimd",
                                        "mout_mpll_user", 800 * MHZ);

	exynos4_setup_clock(&s5p_device_fimc0.dev, "sclk_fimc",
					"mout_mpll_user", 160 * MHZ);
	exynos4_setup_clock(&s5p_device_fimc1.dev, "sclk_fimc",
					"mout_mpll_user", 160 * MHZ);
	exynos4_setup_clock(&s5p_device_fimc2.dev, "sclk_fimc",
					"mout_mpll_user", 160 * MHZ);
	exynos4_setup_clock(&s5p_device_fimc3.dev, "sclk_fimc",
					"mout_mpll_user", 160 * MHZ);

	exynos4_setup_clock(&s5p_device_mipi_csis0.dev, "sclk_csis",
					"mout_mpll_user", 160 * MHZ);
	exynos4_setup_clock(&s5p_device_mipi_csis0.dev, "sclk_csis",
					"mout_mpll_user", 160 * MHZ);

	exynos4_setup_clock(NULL, "mout_mfc0", "mout_mpll", 0);
	exynos4_setup_clock(&s5p_device_mfc.dev, "sclk_mfc",
					"mout_mfc0", 200 * MHZ);

	exynos4_setup_clock(NULL, "mout_jpeg0", "mout_mpll", 0);
	exynos4_setup_clock(&s5p_device_mfc.dev, "sclk_jpeg",
					"mout_jpeg0", 160 * MHZ);

	exynos4_setup_clock(&s3c_device_hsmmc2.dev, "dout_mmc2",
					"mout_mpll_user", 100 * MHZ);
#ifdef CONFIG_EXYNOS4_DEV_DWMCI
	exynos4_setup_clock(&exynos_device_dwmci.dev, "dout_mmc4",
					"mout_mpll_user", 400 * MHZ);
#endif
}

static void __init smdk4x12_machine_init(void)
{
	exynos_bootdev_init();

#ifdef CONFIG_S3C64XX_DEV_SPI0
	spi_register_board_info(spi0_board_info, ARRAY_SIZE(spi0_board_info));
#endif
#ifdef CONFIG_S3C64XX_DEV_SPI1
	spi_register_board_info(spi1_board_info, ARRAY_SIZE(spi1_board_info));
#endif
#ifdef CONFIG_S3C64XX_DEV_SPI2
	spi_register_board_info(spi2_board_info, ARRAY_SIZE(spi2_board_info));
#endif

	if (samsung_pack() != EXYNOS4412_PACK_SCP) {
#ifdef CONFIG_REGULATOR_MAX77686
		max77686_populate_pdata();
#endif
	}

	s3c_adc_set_platdata(NULL);
	s3c_adc_setname("samsung-adc-v4");

	s3c_i2c0_set_platdata(NULL);
	i2c_register_board_info(0, smdk4x12_i2c_devs0,
			ARRAY_SIZE(smdk4x12_i2c_devs0));

	s3c_i2c1_set_platdata(NULL);
	i2c_register_board_info(1, smdk4x12_i2c_devs1,
			ARRAY_SIZE(smdk4x12_i2c_devs1));

	s3c_i2c2_set_platdata(NULL);
	i2c_register_board_info(2, smdk4x12_i2c_devs2,
			ARRAY_SIZE(smdk4x12_i2c_devs2));

	s3c_i2c3_set_platdata(NULL);
	i2c_register_board_info(3, smdk4x12_i2c_devs3,
			ARRAY_SIZE(smdk4x12_i2c_devs3));

	s3c_i2c4_set_platdata(NULL);
	i2c_register_board_info(4, smdk4x12_i2c_devs4, ARRAY_SIZE(smdk4x12_i2c_devs4));

#ifdef CONFIG_MPU_SENSORS_MPU6050B1
	sensor_hw_init();
#endif
	s3c_i2c5_set_platdata(NULL);
	i2c_register_board_info(5, smdk4x12_i2c_devs5, ARRAY_SIZE(smdk4x12_i2c_devs5));

	smdk4x12_rtc_wake_init();
	smdk4x12_pmu_wdt_init();

	s3c_i2c7_set_platdata(NULL);
	i2c_register_board_info(7, smdk4x12_i2c_devs7,
			ARRAY_SIZE(smdk4x12_i2c_devs7));

	s3c_hsotg_set_platdata(&smdk4x12_hsotg_pdata);
#ifdef CONFIG_USB_EXYNOS_SWITCH
	smdk4x12_usbswitch_init();
#endif
	samsung_bl_set(&smdk4x12_bl_gpio_info, &smdk4x12_bl_data);

	fb_init_pdata(&smdk4x12_lcd0_pdata);
	s5p_fimd0_set_platdata(&smdk4x12_lcd0_pdata);
#ifdef CONFIG_LCD_LMS501KF03
	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
#endif

#ifdef CONFIG_SAMSUNG_DEV_KEYPAD
	samsung_keypad_set_platdata(&smdk4x12_keypad_data);
#endif

#ifdef CONFIG_EXYNOS4_DEV_DWMCI
	exynos_dwmci_set_platdata(&exynos_dwmci_pdata);
#endif

	s3c_sdhci2_set_platdata(&smdk4x12_hsmmc2_pdata);
	s3c_sdhci3_set_platdata(&smdk4x12_hsmmc3_pdata);

#ifdef CONFIG_ION_EXYNOS
	exynos_ion_set_platdata();
#endif
	s5p_tv_setup();
	s5p_i2c_hdmiphy_set_platdata(NULL);
	s5p_hdmi_set_platdata(smdk4x12_i2c_hdmiphy, NULL, 0);

#ifdef CONFIG_VIDEO_EXYNOS_FIMG2D
	s5p_fimg2d_set_platdata(&fimg2d_data);
#endif
#if defined(CONFIG_VIDEO_M5MOLS) || defined(CONFIG_VIDEO_S5K6A3) || defined(CONFIG_VIDEO_OV5640)
	smdk4x12_camera_init();
#endif
#ifdef CONFIG_VIDEO_EXYNOS_FIMC_LITE
	smdk4x12_set_camera_flite_platdata();
	s3c_set_platdata(&exynos_flite0_default_data,
			sizeof(exynos_flite0_default_data), &exynos_device_flite0);
	s3c_set_platdata(&exynos_flite1_default_data,
			sizeof(exynos_flite1_default_data), &exynos_device_flite1);
#endif
	smdk4x12_ehci_init();

#ifdef CONFIG_S3C64XX_DEV_SPI0
	s3c64xx_spi0_set_platdata(NULL, 0, 1);
#endif
#ifdef CONFIG_S3C64XX_DEV_SPI1
	s3c64xx_spi1_set_platdata(NULL, 0, 1);
#endif
#ifdef CONFIG_S3C64XX_DEV_SPI2
	s3c64xx_spi2_set_platdata(NULL, 0, 1);
#endif

	smdk4x12_ohci_init();
	platform_add_devices(smdk4x12_devices, ARRAY_SIZE(smdk4x12_devices));
	if (!uhost0)
		platform_device_register(&s3c_device_usb_hsotg);

#ifdef CONFIG_VIDEO_EXYNOS_FIMC_IS
	exynos4_fimc_is_set_platdata(NULL);
#endif

	if (soc_is_exynos4412()) {
		if ((samsung_rev() >= EXYNOS4412_REV_2_0))
			initialize_prime_clocks();
		else
			initialize_non_prime_clocks();
	}
#ifdef CONFIG_BUSFREQ_OPP
	dev_add(&busfreq, &exynos4_busfreq.dev);
	ppmu_init(&exynos_ppmu[PPMU_DMC0], &exynos4_busfreq.dev);
	ppmu_init(&exynos_ppmu[PPMU_DMC1], &exynos4_busfreq.dev);
	ppmu_init(&exynos_ppmu[PPMU_CPU], &exynos4_busfreq.dev);
#endif
	set_tmu_platdata();

	/* add by cym 20140318 */
#ifdef CONFIG_SMSC911X
	smdk4x12_smsc911x_init();
#endif
/* end add */

/* add by cym 20140527 */
#ifdef CONFIG_USB_NET_DM9620
	dm9620_reset();
#endif
/* end add */
}

MACHINE_START(ITOP4412, "SMDK4412")
	/* Maintainer: Kukjin Kim <kgene.kim@samsung.com> */
	/* Maintainer: Changhwan Youn <chaos.youn@samsung.com> */
	.atag_offset	= 0x100,
	.init_irq	= exynos4_init_irq,
	.map_io		= smdk4x12_map_io,
	.handle_irq	= gic_handle_irq,
	.init_machine	= smdk4x12_machine_init,
	.init_late	= exynos_init_late,
	.timer		= &exynos4_timer,
	.restart	= exynos4_restart,
	.reserve	= &smdk4x12_reserve,
MACHINE_END

