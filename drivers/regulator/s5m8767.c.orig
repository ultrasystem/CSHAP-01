/*
 * s5m8767.c
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd
 *              http://www.samsung.com
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/bug.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/mfd/s5m87xx/s5m-core.h>
#include <linux/mfd/s5m87xx/s5m-pmic.h>

struct s5m8767_info {
	struct device *dev;
	struct s5m87xx_dev *iodev;
	int num_regulators;
	struct regulator_dev **rdev;
	struct s5m_opmode_data *opmode;

	int ramp_delay;
	bool buck2_ramp;
	bool buck3_ramp;
	bool buck4_ramp;

	bool buck2_gpiodvs;
	bool buck3_gpiodvs;
	bool buck4_gpiodvs;
	u8 buck2_vol[8];
	u8 buck3_vol[8];
	u8 buck4_vol[8];
	int buck_gpios[3];
	int buck_gpioindex;
};

struct s5m_voltage_desc {
	int max;
	int min;
	int step;
};

static const struct s5m_voltage_desc buck_voltage_val1 = {
	.max = 2225000,
	.min =  650000,
	.step =   6250,
};

static const struct s5m_voltage_desc buck_voltage_val2 = {
	.max = 1600000,
	.min =  600000,
	.step =   6250,
};

static const struct s5m_voltage_desc buck_voltage_val3 = {
	.max = 3000000,
	.min =  750000,
	.step =  12500,
};

static const struct s5m_voltage_desc ldo_voltage_val1 = {
/* modify by cym 20140311 */
#if 0
	.max = 1800000,
	.min = 1800000,
	.step =  50000,
#else
	.max = 3950000,
	.min =  800000,
	.step =  50000,
#endif
/* end modify */
};

static const struct s5m_voltage_desc ldo_voltage_val4 = {
	.max = 3300000,
	.min = 3000000,
	.step =  25000,
};
static const struct s5m_voltage_desc ldo_voltage_val2 = {
/* modify by cym 20140311 */
#if 0
	.max = 1000000,
	.min =  800000,
	.step =  25000,
#else
	.max = 2375000,
	.min =  800000,
	.step =  25000,
#endif
/* end modify */
};

static const struct s5m_voltage_desc *reg_voltage_map[] = {
	[S5M8767_LDO1] = &ldo_voltage_val2,
	[S5M8767_LDO2] = &ldo_voltage_val2,
	[S5M8767_LDO3] = &ldo_voltage_val1,
	[S5M8767_LDO4] = &ldo_voltage_val1,
	[S5M8767_LDO5] = &ldo_voltage_val1,
	[S5M8767_LDO6] = &ldo_voltage_val2,
	[S5M8767_LDO7] = &ldo_voltage_val2,
	[S5M8767_LDO8] = &ldo_voltage_val2,
	[S5M8767_LDO9] = &ldo_voltage_val1,
	[S5M8767_LDO10] = &ldo_voltage_val1,
	[S5M8767_LDO11] = &ldo_voltage_val1,
	/* modify by cym 20140311 */
#if 0
	[S5M8767_LDO12] = &ldo_voltage_val4,
#else
	[S5M8767_LDO12] = &ldo_voltage_val1,
#endif
	/* end modify */
	[S5M8767_LDO13] = &ldo_voltage_val1,
	[S5M8767_LDO14] = &ldo_voltage_val1,
	[S5M8767_LDO15] = &ldo_voltage_val2,
	[S5M8767_LDO16] = &ldo_voltage_val1,
	[S5M8767_LDO17] = &ldo_voltage_val1,
	[S5M8767_LDO18] = &ldo_voltage_val1,
	[S5M8767_LDO19] = &ldo_voltage_val1,
	[S5M8767_LDO20] = &ldo_voltage_val1,
	[S5M8767_LDO21] = &ldo_voltage_val1,
	[S5M8767_LDO22] = &ldo_voltage_val1,
	[S5M8767_LDO23] = &ldo_voltage_val1,
	[S5M8767_LDO24] = &ldo_voltage_val1,
	[S5M8767_LDO25] = &ldo_voltage_val1,
	[S5M8767_LDO26] = &ldo_voltage_val1,
	[S5M8767_LDO27] = &ldo_voltage_val1,
	[S5M8767_LDO28] = &ldo_voltage_val1,
	[S5M8767_BUCK1] = &buck_voltage_val1,
	[S5M8767_BUCK2] = &buck_voltage_val2,
	[S5M8767_BUCK3] = &buck_voltage_val2,
	[S5M8767_BUCK4] = &buck_voltage_val2,
	[S5M8767_BUCK5] = &buck_voltage_val1,
	[S5M8767_BUCK6] = &buck_voltage_val1,
	[S5M8767_BUCK7] = NULL,
	[S5M8767_BUCK8] = NULL,
	[S5M8767_BUCK9] = &buck_voltage_val3,
};

static int s5m8767_list_voltage(struct regulator_dev *rdev,
				unsigned int selector)
{
	const struct s5m_voltage_desc *desc;
	int reg_id = rdev_get_id(rdev);
	int val;

	if (reg_id >= ARRAY_SIZE(reg_voltage_map) || reg_id < 0)
		return -EINVAL;

	desc = reg_voltage_map[reg_id];
	if (desc == NULL)
		return -EINVAL;

	val = desc->min + desc->step * selector;
	if (val > desc->max)
		return -EINVAL;

	return val;
}

static unsigned int s5m8767_opmode_reg[][4] = {
	/* {OFF, ON, LOWPOWER, SUSPEND} */
	/* LDO1 ... LDO28 */
	{0x0, 0x3, 0x2, 0x1}, /* LDO1 */
	{0x0, 0x3, 0x2, 0x1},
	{0x0, 0x3, 0x2, 0x1},
	{0x0, 0x0, 0x0, 0x0},
	{0x0, 0x3, 0x2, 0x1}, /* LDO5 */
	{0x0, 0x3, 0x2, 0x1},
	{0x0, 0x3, 0x2, 0x1},
	{0x0, 0x3, 0x2, 0x1},
	{0x0, 0x3, 0x2, 0x1},
	{0x0, 0x3, 0x2, 0x1}, /* LDO10 */
	{0x0, 0x3, 0x2, 0x1},
	{0x0, 0x3, 0x2, 0x1},
	{0x0, 0x3, 0x2, 0x1},
	{0x0, 0x3, 0x2, 0x1},
	{0x0, 0x3, 0x2, 0x1}, /* LDO15 */
	{0x0, 0x3, 0x2, 0x1},
	{0x0, 0x3, 0x2, 0x1},
	{0x0, 0x0, 0x0, 0x0},
	{0x0, 0x3, 0x2, 0x1},
	{0x0, 0x3, 0x2, 0x1}, /* LDO20 */
	{0x0, 0x3, 0x2, 0x1},
	{0x0, 0x3, 0x2, 0x1},
	{0x0, 0x0, 0x0, 0x0},
	{0x0, 0x3, 0x2, 0x1},
	{0x0, 0x3, 0x2, 0x1}, /* LDO25 */
	{0x0, 0x3, 0x2, 0x1},
	{0x0, 0x3, 0x2, 0x1},
	{0x0, 0x3, 0x2, 0x1}, /* LDO28 */

	/* BUCK1 ... BUCK9 */
	{0x0, 0x3, 0x1, 0x1}, /* BUCK1 */
	{0x0, 0x3, 0x1, 0x1},
	{0x0, 0x3, 0x1, 0x1},
	{0x0, 0x3, 0x1, 0x1},
	{0x0, 0x3, 0x2, 0x1}, /* BUCK5 */
	{0x0, 0x3, 0x1, 0x1},
	{0x0, 0x3, 0x1, 0x1},
	{0x0, 0x3, 0x1, 0x1},
	{0x0, 0x3, 0x1, 0x1}, /* BUCK9 */
};

/* add by cym 20140311 */
static int s5m8767_get_disable_val(struct regulator_dev *rdev)
{

	int reg_id = rdev_get_id(rdev);
	int ret = 0;
	switch (reg_id) {
	case S5M8767_LDO2:
	
	case S5M8767_LDO6:
	case S5M8767_LDO7:
	//case S5M8767_LDO9: //Robin, ldo9 is for lcd,it's better to power on/off in lcd driver..
	//case S5M8767_LDO13:
	case S5M8767_LDO11:// ... S5M8767_LDO12://zhangdong, reduce sleep current
	case S5M8767_LDO14 ... S5M8767_LDO15:
	case S5M8767_LDO17:
		 ret = 1;
		break;
        case S5M8767_LDO1:
        case S5M8767_BUCK5: 
	case S5M8767_LDO13:
	case S5M8767_LDO18:	
		ret = 3;
		break;
	case S5M8767_LDO4:
	//case S5M8767_LDO18:
	case S5M8767_LDO23:
		 ret = 2;		
		break;
	case S5M8767_BUCK1 ... S5M8767_BUCK4:
		ret = 1;
		break;
	case S5M8767_BUCK9:
		ret = 1;
		break;
	default:
		return ret;
	}

	return ret;

}
/* end add */

/* remove by cym 20140311 */
#if 0
static int s5m8767_get_register(struct regulator_dev *rdev, int *reg,
				int *enable_ctrl)
#else
static int s5m8767_get_register(struct regulator_dev *rdev, int *reg)
#endif
{
	int reg_id = rdev_get_id(rdev);
	unsigned int mode;
	struct s5m8767_info *s5m8767 = rdev_get_drvdata(rdev);

	switch (reg_id) {
	case S5M8767_LDO1 ... S5M8767_LDO2:
		*reg = S5M8767_REG_LDO1CTRL + (reg_id - S5M8767_LDO1);
		break;
	case S5M8767_LDO3 ... S5M8767_LDO28:
		*reg = S5M8767_REG_LDO3CTRL + (reg_id - S5M8767_LDO3);
		break;
	case S5M8767_BUCK1:
		*reg = S5M8767_REG_BUCK1CTRL1;
		break;
	case S5M8767_BUCK2 ... S5M8767_BUCK4:
		*reg = S5M8767_REG_BUCK2CTRL + (reg_id - S5M8767_BUCK2) * 9;
		break;
	case S5M8767_BUCK5:
		*reg = S5M8767_REG_BUCK5CTRL1;
		break;
	case S5M8767_BUCK6 ... S5M8767_BUCK9:
		*reg = S5M8767_REG_BUCK6CTRL1 + (reg_id - S5M8767_BUCK6) * 2;
		break;
	default:
		return -EINVAL;
	}

	/* remove by cym 20140311 */
#if 0
	mode = s5m8767->opmode[reg_id].mode;
	*enable_ctrl = s5m8767_opmode_reg[reg_id][mode] << S5M8767_ENCTRL_SHIFT;
#endif
	return 0;
}

static int s5m8767_reg_is_enabled(struct regulator_dev *rdev)
{
	struct s5m8767_info *s5m8767 = rdev_get_drvdata(rdev);
	int ret, reg;
	int mask = 0xc0, pattern=0xc0;
	u8 val;

	/* modify by cym 20140311 */
#if 0	
	ret = s5m8767_get_register(rdev, &reg, &enable_ctrl);
#else
	ret = s5m8767_get_register(rdev, &reg);
#endif
	/* end modify */

	if (ret == -EINVAL)
		return 1;
	else if (ret)
		return ret;

	ret = s5m_reg_read(s5m8767->iodev, reg, &val);
	if (ret)
		return ret;
	//return  (val & mask) == enable_ctrl;
	return (val & mask) == pattern;
}

static int s5m8767_reg_enable(struct regulator_dev *rdev)
{
	struct s5m8767_info *s5m8767 = rdev_get_drvdata(rdev);
	int ret, reg;
	
	int mask = 0xc0, pattern=0xc0;

	/* modify by cym 20140311 */
#if 0
	ret = s5m8767_get_register(rdev, &reg, &enable_ctrl);
#else
	ret = s5m8767_get_register(rdev, &reg);
#endif
	/* end modify */

	if (ret)
		return ret;

	ret = s5m8767_get_disable_val(rdev);

	if(ret == 1)
		pattern = 0x40;
	else if(ret == 2)
		pattern = 0x0;
	else if(ret == 3)
		pattern = 0x80;

	return s5m_reg_update(s5m8767->iodev, reg, pattern, mask);
}

static int s5m8767_reg_disable(struct regulator_dev *rdev)
{
	struct s5m8767_info *s5m8767 = rdev_get_drvdata(rdev);
	int ret, reg;
	int  mask = 0xc0, pattern=0xc0;

	/* modify by cym 20140311 */
#if 0
	ret = s5m8767_get_register(rdev, &reg, &enable_ctrl);
#else
	ret = s5m8767_get_register(rdev, &reg);
#endif
	/* end modify */

	if (ret)
		return ret;

	ret = s5m8767_get_disable_val(rdev);

	if(ret == 2)
		ret = 0;

	//return s5m_reg_update(s5m8767->iodev, reg, ~mask, mask);
	return s5m_reg_update(s5m8767->iodev, reg, (~pattern) | (ret <<6), mask);
}

int s5m8767_get_voltage_register(struct regulator_dev *rdev, int *_reg)
{
	struct s5m8767_info *s5m8767 = rdev_get_drvdata(rdev);
	int reg_id = rdev_get_id(rdev);
	int reg;

	switch (reg_id) {
	case S5M8767_LDO1 ... S5M8767_LDO2:
		reg = S5M8767_REG_LDO1CTRL + (reg_id - S5M8767_LDO1);
		break;
	case S5M8767_LDO3 ... S5M8767_LDO28:
		reg = S5M8767_REG_LDO3CTRL + (reg_id - S5M8767_LDO3);
		break;
	case S5M8767_BUCK1:
		reg = S5M8767_REG_BUCK1CTRL2;
		break;
	case S5M8767_BUCK2:
		reg = S5M8767_REG_BUCK2DVS1;
		if (s5m8767->buck2_gpiodvs)
			reg += s5m8767->buck_gpioindex;
		break;
	case S5M8767_BUCK3:
		reg = S5M8767_REG_BUCK3DVS1;
		if (s5m8767->buck3_gpiodvs)
			reg += s5m8767->buck_gpioindex;
		break;
	case S5M8767_BUCK4:
		reg = S5M8767_REG_BUCK4DVS1;
		if (s5m8767->buck4_gpiodvs)
			reg += s5m8767->buck_gpioindex;
		break;
	case S5M8767_BUCK5:
		reg = S5M8767_REG_BUCK5CTRL2;
		break;
	case S5M8767_BUCK6 ... S5M8767_BUCK9:
		reg = S5M8767_REG_BUCK6CTRL2 + (reg_id - S5M8767_BUCK6) * 2;
		break;
	default:
		return -EINVAL;
	}
	//printk("reg = %d\n", reg);
	*_reg = reg;

	return reg;
}

static int s5m8767_get_voltage_sel(struct regulator_dev *rdev)
{
	struct s5m8767_info *s5m8767 = rdev_get_drvdata(rdev);
	int reg, mask, ret;
	int reg_id = rdev_get_id(rdev);
	u8 val;

	ret = s5m8767_get_voltage_register(rdev, &reg);
	if (-EINVAL == ret)
		return ret;
	//printk("**************** ret = %d\n", ret);
	reg = ret;

	mask = (reg_id < S5M8767_BUCK1) ? 0x3f : 0xff;

	ret = s5m_reg_read(s5m8767->iodev, reg, &val);
	if (ret)
		return ret;

	val &= mask;

	return val;
}

static int s5m8767_convert_voltage_to_sel(
		const struct s5m_voltage_desc *desc,
		int min_vol, int max_vol)
{
	int selector = 0;

	if (desc == NULL)
		return -EINVAL;

	if (max_vol < desc->min || min_vol > desc->max)
		return -EINVAL;

	if (min_vol < desc->min)
		min_vol = desc->min;

	selector = DIV_ROUND_UP(min_vol - desc->min, desc->step);

	if (desc->min + desc->step * selector > max_vol)
		return -EINVAL;

	return selector;
}

static inline void s5m8767_set_high(struct s5m8767_info *s5m8767)
{
	int temp_index = s5m8767->buck_gpioindex;

	gpio_set_value(s5m8767->buck_gpios[0], (temp_index >> 2) & 0x1);
	gpio_set_value(s5m8767->buck_gpios[1], (temp_index >> 1) & 0x1);
	gpio_set_value(s5m8767->buck_gpios[2], temp_index & 0x1);
}

static inline void s5m8767_set_low(struct s5m8767_info *s5m8767)
{
	int temp_index = s5m8767->buck_gpioindex;

	gpio_set_value(s5m8767->buck_gpios[2], temp_index & 0x1);
	gpio_set_value(s5m8767->buck_gpios[1], (temp_index >> 1) & 0x1);
	gpio_set_value(s5m8767->buck_gpios[0], (temp_index >> 2) & 0x1);
}

static int s5m8767_set_voltage(struct regulator_dev *rdev,
				int min_uV, int max_uV, unsigned *selector)
{
	struct s5m8767_info *s5m8767 = rdev_get_drvdata(rdev);
	const struct s5m_voltage_desc *desc;
	int reg_id = rdev_get_id(rdev);
	int sel, reg_buf, mask, ret = 0, old_index, index = 0;
	u8 val;
	u8 *buck234_vol = NULL;

	switch (reg_id) {
	case S5M8767_LDO1 ... S5M8767_LDO28:
		mask = 0x3f;
		break;
	case S5M8767_BUCK1 ... S5M8767_BUCK6:
		mask = 0xff;
		if (reg_id == S5M8767_BUCK2 && s5m8767->buck2_gpiodvs)
			buck234_vol = &s5m8767->buck2_vol[0];
		else if (reg_id == S5M8767_BUCK3 && s5m8767->buck3_gpiodvs)
			buck234_vol = &s5m8767->buck3_vol[0];
		else if (reg_id == S5M8767_BUCK4 && s5m8767->buck4_gpiodvs)
			buck234_vol = &s5m8767->buck4_vol[0];
		break;
	case S5M8767_BUCK7 ... S5M8767_BUCK8:
		return -EINVAL;
	case S5M8767_BUCK9:
		mask = 0xff;
		break;
	default:
		return -EINVAL;
	}

	desc = reg_voltage_map[reg_id];

	sel = s5m8767_convert_voltage_to_sel(desc, min_uV, max_uV);
	if (sel < 0)
		return sel;

	/* buck234_vol != NULL means to control buck234 voltage via DVS GPIO */
	if (buck234_vol) {
		while (*buck234_vol != sel) {
			buck234_vol++;
			index++;
		}
		old_index = s5m8767->buck_gpioindex;
		s5m8767->buck_gpioindex = index;

		if (index > old_index)
			s5m8767_set_high(s5m8767);
		else
			s5m8767_set_low(s5m8767);
	} else {
		ret = s5m8767_get_voltage_register(rdev, &reg_buf);
		if (-EINVAL == ret)
			return ret;
		//printk("****************cym ret = %d\n", ret);
		//reg = ret;
		//printk("****************cym reg = %d\n", reg_buf);
		s5m_reg_read(s5m8767->iodev, reg_buf, &val);
		//printk("****************cym1 reg = %d\n", reg_buf);
		val = (val & ~mask) | sel;
		//printk("reg_id = %d, reg_bug = %d, val = 0x%x\n", reg_id, reg_buf, val);
		ret = s5m_reg_write(s5m8767->iodev, ret, val);
	}

	*selector = sel;
	return ret;
}

static int s5m8767_set_voltage_time_sel(struct regulator_dev *rdev,
					     unsigned int old_sel,
					     unsigned int new_sel)
{
	struct s5m8767_info *s5m8767 = rdev_get_drvdata(rdev);
	const struct s5m_voltage_desc *desc;
	int reg_id = rdev_get_id(rdev);

	desc = reg_voltage_map[reg_id];

	if ((old_sel < new_sel) && s5m8767->ramp_delay)
		return DIV_ROUND_UP(desc->step * (new_sel - old_sel),
					s5m8767->ramp_delay * 1000);
	return 0;
}

static struct regulator_ops s5m8767_ops = {
	.list_voltage		= s5m8767_list_voltage,
//	.is_enabled		= s5m8767_reg_is_enabled,
	.enable			= s5m8767_reg_enable,
	.disable		= s5m8767_reg_disable,
	.get_voltage_sel	= s5m8767_get_voltage_sel,
	.set_voltage		= s5m8767_set_voltage,
	.set_voltage_time_sel	= s5m8767_set_voltage_time_sel,
};

#define s5m8767_regulator_desc(_name) {		\
	.name		= #_name,		\
	.id		= S5M8767_##_name,	\
	.ops		= &s5m8767_ops,		\
	.type		= REGULATOR_VOLTAGE,	\
	.owner		= THIS_MODULE,		\
}

static struct regulator_desc regulators[] = {
	s5m8767_regulator_desc(LDO1),
	s5m8767_regulator_desc(LDO2),
	s5m8767_regulator_desc(LDO3),
	s5m8767_regulator_desc(LDO4),
	s5m8767_regulator_desc(LDO5),
	s5m8767_regulator_desc(LDO6),
	s5m8767_regulator_desc(LDO7),
	s5m8767_regulator_desc(LDO8),
	s5m8767_regulator_desc(LDO9),
	s5m8767_regulator_desc(LDO10),
	s5m8767_regulator_desc(LDO11),
	s5m8767_regulator_desc(LDO12),
	s5m8767_regulator_desc(LDO13),
	s5m8767_regulator_desc(LDO14),
	s5m8767_regulator_desc(LDO15),
	s5m8767_regulator_desc(LDO16),
	s5m8767_regulator_desc(LDO17),
	s5m8767_regulator_desc(LDO18),
	s5m8767_regulator_desc(LDO19),
	s5m8767_regulator_desc(LDO20),
	s5m8767_regulator_desc(LDO21),
	s5m8767_regulator_desc(LDO22),
	s5m8767_regulator_desc(LDO23),
	s5m8767_regulator_desc(LDO24),
	s5m8767_regulator_desc(LDO25),
	s5m8767_regulator_desc(LDO26),
	s5m8767_regulator_desc(LDO27),
	s5m8767_regulator_desc(LDO28),
	s5m8767_regulator_desc(BUCK1),
	s5m8767_regulator_desc(BUCK2),
	s5m8767_regulator_desc(BUCK3),
	s5m8767_regulator_desc(BUCK4),
	s5m8767_regulator_desc(BUCK5),
	s5m8767_regulator_desc(BUCK6),
	s5m8767_regulator_desc(BUCK7),
	s5m8767_regulator_desc(BUCK8),
	s5m8767_regulator_desc(BUCK9),
};

static __devinit int s5m8767_pmic_probe(struct platform_device *pdev)
{
	struct s5m87xx_dev *iodev = dev_get_drvdata(pdev->dev.parent);
	struct s5m_platform_data *pdata = dev_get_platdata(iodev->dev);
	struct regulator_config config = { };
	struct regulator_dev **rdev;
	struct s5m8767_info *s5m8767;
	int i, ret, size, reg;
	u8 val;
	
	if (!pdata) {
		dev_err(pdev->dev.parent, "Platform data not supplied\n");
		return -ENODEV;
	}

	if (pdata->buck2_gpiodvs) {
		if (pdata->buck3_gpiodvs || pdata->buck4_gpiodvs) {
			dev_err(&pdev->dev, "S5M8767 GPIO DVS NOT VALID\n");
			return -EINVAL;
		}
	}

	if (pdata->buck3_gpiodvs) {
		if (pdata->buck2_gpiodvs || pdata->buck4_gpiodvs) {
			dev_err(&pdev->dev, "S5M8767 GPIO DVS NOT VALID\n");
			return -EINVAL;
		}
	}

	if (pdata->buck4_gpiodvs) {
		if (pdata->buck2_gpiodvs || pdata->buck3_gpiodvs) {
			dev_err(&pdev->dev, "S5M8767 GPIO DVS NOT VALID\n");
			return -EINVAL;
		}
	}

	s5m8767 = devm_kzalloc(&pdev->dev, sizeof(struct s5m8767_info),
				GFP_KERNEL);
	if (!s5m8767)
		return -ENOMEM;

	size = sizeof(struct regulator_dev *) * (S5M8767_REG_MAX - 2);
	s5m8767->rdev = devm_kzalloc(&pdev->dev, size, GFP_KERNEL);
	if (!s5m8767->rdev)
		return -ENOMEM;

	rdev = s5m8767->rdev;
	s5m8767->dev = &pdev->dev;
	s5m8767->iodev = iodev;
	s5m8767->num_regulators = pdata->num_regulators; /* S5M8767_REG_MAX - 2; */
	platform_set_drvdata(pdev, s5m8767);

	s5m8767->buck_gpioindex = pdata->buck_default_idx;
	s5m8767->buck2_gpiodvs = pdata->buck2_gpiodvs;
	s5m8767->buck3_gpiodvs = pdata->buck3_gpiodvs;
	s5m8767->buck4_gpiodvs = pdata->buck4_gpiodvs;
	s5m8767->buck_gpios[0] = pdata->buck_gpios[0];
	s5m8767->buck_gpios[1] = pdata->buck_gpios[1];
	s5m8767->buck_gpios[2] = pdata->buck_gpios[2];
	s5m8767->ramp_delay = pdata->buck_ramp_delay;
	s5m8767->buck2_ramp = pdata->buck2_ramp_enable;
	s5m8767->buck3_ramp = pdata->buck3_ramp_enable;
	s5m8767->buck4_ramp = pdata->buck4_ramp_enable;
	s5m8767->opmode = pdata->opmode;

	for (i = 0; i < 8; i++) {
		if (s5m8767->buck2_gpiodvs) {
			s5m8767->buck2_vol[i] =
				s5m8767_convert_voltage_to_sel(
						&buck_voltage_val2,
						pdata->buck2_voltage[i],
						pdata->buck2_voltage[i] +
						buck_voltage_val2.step);
		}

		if (s5m8767->buck3_gpiodvs) {
			s5m8767->buck3_vol[i] =
				s5m8767_convert_voltage_to_sel(
						&buck_voltage_val2,
						pdata->buck3_voltage[i],
						pdata->buck3_voltage[i] +
						buck_voltage_val2.step);
		}

		if (s5m8767->buck4_gpiodvs) {
			s5m8767->buck4_vol[i] =
				s5m8767_convert_voltage_to_sel(
						&buck_voltage_val2,
						pdata->buck4_voltage[i],
						pdata->buck4_voltage[i] +
						buck_voltage_val2.step);
		}
	}

	if (pdata->buck2_gpiodvs || pdata->buck3_gpiodvs ||
		pdata->buck4_gpiodvs) {
		if (gpio_is_valid(pdata->buck_gpios[0]) &&
			gpio_is_valid(pdata->buck_gpios[1]) &&
			gpio_is_valid(pdata->buck_gpios[2])) {
			ret = gpio_request(pdata->buck_gpios[0],
						"S5M8767 SET1");
			if (ret == -EBUSY)
				dev_warn(&pdev->dev, "Duplicated gpio request for SET1\n");

			ret = gpio_request(pdata->buck_gpios[1],
					   "S5M8767 SET2");
			if (ret == -EBUSY)
				dev_warn(&pdev->dev, "Duplicated gpio request for SET2\n");

			ret = gpio_request(pdata->buck_gpios[2],
					   "S5M8767 SET3");
			if (ret == -EBUSY)
				dev_warn(&pdev->dev, "Duplicated gpio request for SET3\n");
			/* SET1 GPIO */
			gpio_direction_output(pdata->buck_gpios[0],
					(s5m8767->buck_gpioindex >> 2) & 0x1);
			/* SET2 GPIO */
			gpio_direction_output(pdata->buck_gpios[1],
					(s5m8767->buck_gpioindex >> 1) & 0x1);
			/* SET3 GPIO */
			gpio_direction_output(pdata->buck_gpios[2],
					(s5m8767->buck_gpioindex >> 0) & 0x1);
			ret = 0;
		} else {
			dev_err(&pdev->dev, "GPIO NOT VALID\n");
			ret = -EINVAL;
			return ret;
		}
	}

	s5m_reg_update(s5m8767->iodev, S5M8767_REG_BUCK2CTRL,
			(pdata->buck2_gpiodvs) ? (1 << 1) : (0 << 1), 1 << 1);
	s5m_reg_update(s5m8767->iodev, S5M8767_REG_BUCK3CTRL,
			(pdata->buck3_gpiodvs) ? (1 << 1) : (0 << 1), 1 << 1);
	s5m_reg_update(s5m8767->iodev, S5M8767_REG_BUCK4CTRL,
			(pdata->buck4_gpiodvs) ? (1 << 1) : (0 << 1), 1 << 1);

	/* Initialize GPIO DVS registers */
	for (i = 0; i < 8; i++) {
		if (s5m8767->buck2_gpiodvs) {
			s5m_reg_write(s5m8767->iodev, S5M8767_REG_BUCK2DVS1 + i,
					   s5m8767->buck2_vol[i]);
		}

		if (s5m8767->buck3_gpiodvs) {
			s5m_reg_write(s5m8767->iodev, S5M8767_REG_BUCK3DVS1 + i,
					   s5m8767->buck3_vol[i]);
		}

		if (s5m8767->buck4_gpiodvs) {
			s5m_reg_write(s5m8767->iodev, S5M8767_REG_BUCK4DVS1 + i,
					   s5m8767->buck4_vol[i]);
		}
	}
	s5m_reg_update(s5m8767->iodev, S5M8767_REG_BUCK2CTRL, 0x78, 0xff);
	s5m_reg_update(s5m8767->iodev, S5M8767_REG_BUCK3CTRL, 0x58, 0xff);
	s5m_reg_update(s5m8767->iodev, S5M8767_REG_BUCK4CTRL, 0x78, 0xff);

	if (s5m8767->buck2_ramp)
		s5m_reg_update(s5m8767->iodev, S5M8767_REG_DVSRAMP, 0x08, 0x08);

	if (s5m8767->buck3_ramp)
		s5m_reg_update(s5m8767->iodev, S5M8767_REG_DVSRAMP, 0x04, 0x04);

	if (s5m8767->buck4_ramp)
		s5m_reg_update(s5m8767->iodev, S5M8767_REG_DVSRAMP, 0x02, 0x02);

	if (s5m8767->buck2_ramp || s5m8767->buck3_ramp
		|| s5m8767->buck4_ramp) {
		switch (s5m8767->ramp_delay) {
		case 15:
			s5m_reg_update(s5m8767->iodev, S5M8767_REG_DVSRAMP,
					0xc0, 0xf0);
			break;
		case 25:
			s5m_reg_update(s5m8767->iodev, S5M8767_REG_DVSRAMP,
					0xd0, 0xf0);
			break;
		case 50:
			s5m_reg_update(s5m8767->iodev, S5M8767_REG_DVSRAMP,
					0xe0, 0xf0);
			break;
		case 100:
			s5m_reg_update(s5m8767->iodev, S5M8767_REG_DVSRAMP,
					0xf0, 0xf0);
			break;
		default:
			s5m_reg_update(s5m8767->iodev, S5M8767_REG_DVSRAMP,
					0x90, 0xf0);
		}
	}
	//printk("********************* %d\n", pdata->num_regulators);
	for (i = 0; i < pdata->num_regulators; i++) {
		const struct s5m_voltage_desc *desc;
		int id = pdata->regulators[i].id;

		desc = reg_voltage_map[id];
		if (desc)
			regulators[id].n_voltages =
				(desc->max - desc->min) / desc->step + 1;

		config.dev = s5m8767->dev;
		config.init_data = pdata->regulators[i].initdata;
		config.driver_data = s5m8767;

		rdev[i] = regulator_register(&regulators[id], &config);
		if (IS_ERR(rdev[i])) {
			ret = PTR_ERR(rdev[i]);
			dev_err(s5m8767->dev, "regulator init failed for %d\n",
					id);
			rdev[i] = NULL;
			goto err;
		}
		//if(i == 4)
		//	return 0;
	}
#if 0
	reg = S5M8767_REG_CTRL2;
	ret = s5m_reg_read(s5m8767->iodev, reg, &val);
	val &= 0x8;
	val |= 0x2;
	s5m_reg_write(s5m8767->iodev, reg, val);
#endif

#if 1
#if 0
	//buck1 vdd_mif
        reg = S5M8767_REG_BUCK1CTRL1;//S5M8767_REG_CTRL2;
        val = 0x48;//1.1v
        s5m_reg_write(s5m8767->iodev, reg, val);
#endif
	//printk("****** cym buck2 vdd_arm reg = %d\n", S5M8767_REG_BUCK2DVS1);
#if 0
        //buck2 vdd_arm
        reg = S5M8767_REG_BUCK2DVS1;//S5M8767_REG_CTRL2;
        val = 0x68;//0x68;//1.25v //0x70;//1.3v
        s5m_reg_write(s5m8767->iodev, reg, val);
#endif
#if 0
        //buck3 vdd_int
        reg = S5M8767_REG_BUCK3DVS1;//S5M8767_RRG_CTRL2;
        val = 0x50;//1.1v //0x60;//1.2v
        s5m_reg_write(s5m8767->iodev, reg, val);
#endif
#if 0
        //buck4 vdd_g3d
        reg = S5M8767_REG_BUCK4DVS1;//S5M8767_RRG_CTRL2;
        val = 0x70;//0x70;//1.3v //0x88;//1.45v
        s5m_reg_write(s5m8767->iodev, reg, val);
#endif
#endif
#if 1
	reg = S5M8767_REG_BUCK1CTRL1;
	ret = s5m_reg_read(s5m8767->iodev, reg, &val);
	val &= ~(0x3<<6);
	val |= 0x1<<6;
	s5m_reg_write(s5m8767->iodev, reg, val);

	reg = S5M8767_REG_BUCK2CTRL;
	ret = s5m_reg_read(s5m8767->iodev, reg, &val);
	val &= ~(0x3<<6);
	val |= 0x1<<6;
	s5m_reg_write(s5m8767->iodev, reg, val);
#if 1
	reg = S5M8767_REG_LDO4CTRL;
	ret = s5m_reg_read(s5m8767->iodev, reg, &val);
	val &= ~(0x3<<6);
	val |= 0x1;
	s5m_reg_write(s5m8767->iodev, reg, val);
#endif
	reg = S5M8767_REG_LDO5CTRL;
	ret = s5m_reg_read(s5m8767->iodev, reg, &val);
	val &= ~(0x3<<6);
	val |= 0x1<<6;
	s5m_reg_write(s5m8767->iodev, reg, val);
#if 1
	reg = S5M8767_REG_LDO6CTRL;
	ret = s5m_reg_read(s5m8767->iodev, reg, &val);
	val &= ~(0x3<<6);
	val |= 0x1<<6;
	s5m_reg_write(s5m8767->iodev, reg, val);
#endif
#if 1
	reg = S5M8767_REG_LDO7CTRL;
	ret = s5m_reg_read(s5m8767->iodev, reg, &val);
	val &= ~(0x3<<6);
	val |= 0x1<<6;
	s5m_reg_write(s5m8767->iodev, reg, val);
#endif
	reg = S5M8767_REG_LDO8CTRL;
	ret = s5m_reg_read(s5m8767->iodev, reg, &val);
	val &= ~(0x3<<6);
	val |= 0x1<<6;
	s5m_reg_write(s5m8767->iodev, reg, val);

	reg = S5M8767_REG_LDO9CTRL;
	ret = s5m_reg_read(s5m8767->iodev, reg, &val);
	val &= ~(0x3<<6);
	val |= 0x1<<6;
	s5m_reg_write(s5m8767->iodev, reg, val);

	reg = S5M8767_REG_LDO10CTRL;
	ret = s5m_reg_read(s5m8767->iodev, reg, &val);
	val &= ~(0x3<<6);
	val |= 0x1<<6;
	s5m_reg_write(s5m8767->iodev, reg, val);
#if 1
	reg = S5M8767_REG_LDO11CTRL;
	ret = s5m_reg_read(s5m8767->iodev, reg, &val);
	val &= ~(0x3<<6);
	val |= 0x1<<6;
	s5m_reg_write(s5m8767->iodev, reg, val);
#endif
	reg = S5M8767_REG_LDO12CTRL;
	ret = s5m_reg_read(s5m8767->iodev, reg, &val);
	val &= ~(0x3<<6);
	val |= 0x1<<6;
	s5m_reg_write(s5m8767->iodev, reg, val);
#if 1
	reg = S5M8767_REG_LDO13CTRL;
	ret = s5m_reg_read(s5m8767->iodev, reg, &val);
	val &= ~(0x3<<6);
	val |= 0x1<<6;
	s5m_reg_write(s5m8767->iodev, reg, val);
#endif
	reg = S5M8767_REG_LDO14CTRL;
	ret = s5m_reg_read(s5m8767->iodev, reg, &val);
	val &= ~(0x3<<6);
	val |= 0x1<<6;
	s5m_reg_write(s5m8767->iodev, reg, val);

	reg = S5M8767_REG_LDO15CTRL;
	ret = s5m_reg_read(s5m8767->iodev, reg, &val);
	val &= ~(0x3<<6);
	val |= 0x1<<6;
	s5m_reg_write(s5m8767->iodev, reg, val);

	reg = S5M8767_REG_LDO16CTRL;
	ret = s5m_reg_read(s5m8767->iodev, reg, &val);
	val &= ~(0x3<<6);
	val |= 0x1<<6;
	s5m_reg_write(s5m8767->iodev, reg, val);

	reg = S5M8767_REG_LDO17CTRL;
	ret = s5m_reg_read(s5m8767->iodev, reg, &val);
	val &= ~(0x3<<6);
	val |= 0x1<<6;
	s5m_reg_write(s5m8767->iodev, reg, val);

	reg = S5M8767_REG_LDO18CTRL;
	ret = s5m_reg_read(s5m8767->iodev, reg, &val);
	val &= ~(0x3<<6);
	val |= 0x1<<6;
	s5m_reg_write(s5m8767->iodev, reg, val);

	reg = S5M8767_REG_LDO19CTRL;
	ret = s5m_reg_read(s5m8767->iodev, reg, &val);
	val &= ~(0x3<<6);
	val |= 0x1<<6;
	s5m_reg_write(s5m8767->iodev, reg, val);

	reg = S5M8767_REG_LDO20CTRL;
	ret = s5m_reg_read(s5m8767->iodev, reg, &val);
	val &= ~(0x3<<6);
	val |= 0x1<<6;
	s5m_reg_write(s5m8767->iodev, reg, val);

	reg = S5M8767_REG_LDO21CTRL;
	ret = s5m_reg_read(s5m8767->iodev, reg, &val);
	val &= ~(0x3<<6);
	val |= 0x1<<6;
	s5m_reg_write(s5m8767->iodev, reg, val);

	reg = S5M8767_REG_LDO22CTRL;
	ret = s5m_reg_read(s5m8767->iodev, reg, &val);
	val &= ~(0x3<<6);
	val |= 0x1<<6;
	s5m_reg_write(s5m8767->iodev, reg, val);

	reg = S5M8767_REG_LDO23CTRL;
	ret = s5m_reg_read(s5m8767->iodev, reg, &val);
	val &= ~(0x3<<6);
	val |= 0x1<<6;
	s5m_reg_write(s5m8767->iodev, reg, val);

	reg = S5M8767_REG_LDO24CTRL;
	ret = s5m_reg_read(s5m8767->iodev, reg, &val);
	val &= ~(0x3<<6);
	val |= 0x1<<6;
	s5m_reg_write(s5m8767->iodev, reg, val);

	reg = S5M8767_REG_LDO25CTRL;
	ret = s5m_reg_read(s5m8767->iodev, reg, &val);
	val &= ~(0x3<<6);
	val |= 0x1<<6;
	s5m_reg_write(s5m8767->iodev, reg, val);

	reg = S5M8767_REG_LDO26CTRL;
	ret = s5m_reg_read(s5m8767->iodev, reg, &val);
	val &= ~(0x3<<6);
	val |= 0x1<<6;
	s5m_reg_write(s5m8767->iodev, reg, val);

	reg = S5M8767_REG_LDO27CTRL;
	ret = s5m_reg_read(s5m8767->iodev, reg, &val);
	val &= ~(0x3<<6);
	val |= 0x1<<6;
	s5m_reg_write(s5m8767->iodev, reg, val);
#endif
#if 0
	{
		int i;
		for(i=S5M8767_REG_LDO1CTRL; i<=S5M8767_REG_LDO28CTRL; i++)
		{
			s5m_reg_read(s5m8767->iodev, i, &val);
			
			printk("reg:0x%x, val:0x%x\n", i, val);
		}
	}
#endif
	return 0;
err:
	for (i = 0; i < s5m8767->num_regulators; i++)
		if (rdev[i])
			regulator_unregister(rdev[i]);

	return ret;
}

static int __devexit s5m8767_pmic_remove(struct platform_device *pdev)
{
	struct s5m8767_info *s5m8767 = platform_get_drvdata(pdev);
	struct regulator_dev **rdev = s5m8767->rdev;
	int i;

	for (i = 0; i < s5m8767->num_regulators; i++)
		if (rdev[i])
			regulator_unregister(rdev[i]);

	return 0;
}

static const struct platform_device_id s5m8767_pmic_id[] = {
	{ "s5m8767-pmic", 0},
	{ },
};
MODULE_DEVICE_TABLE(platform, s5m8767_pmic_id);

static struct platform_driver s5m8767_pmic_driver = {
	.driver = {
		.name = "s5m8767-pmic",
		.owner = THIS_MODULE,
	},
	.probe = s5m8767_pmic_probe,
	.remove = __devexit_p(s5m8767_pmic_remove),
	.id_table = s5m8767_pmic_id,
};

static int __init s5m8767_pmic_init(void)
{
	return platform_driver_register(&s5m8767_pmic_driver);
}
subsys_initcall(s5m8767_pmic_init);

static void __exit s5m8767_pmic_exit(void)
{
	platform_driver_unregister(&s5m8767_pmic_driver);
}
module_exit(s5m8767_pmic_exit);

/* Module information */
MODULE_AUTHOR("Sangbeom Kim <sbkim73@samsung.com>");
MODULE_DESCRIPTION("SAMSUNG S5M8767 Regulator Driver");
MODULE_LICENSE("GPL");
