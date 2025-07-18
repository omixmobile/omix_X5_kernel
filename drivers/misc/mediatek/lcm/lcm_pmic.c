// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/regulator/consumer.h>
#include <linux/string.h>
#include <linux/kernel.h>
//Antaiui <AI_BSP_LCD> <hehl> <20200826> for 1907 start
#include <linux/delay.h>
//Antaiui <AI_BSP_LCD> <hehl> <20200826> for 1907 end
#if defined(CONFIG_RT5081_PMU_DSV) || defined(CONFIG_MT6370_PMU_DSV)
static struct regulator *disp_bias_pos;
static struct regulator *disp_bias_neg;
static int regulator_inited;

int display_bias_regulator_init(void)
{
	int ret = 0;

	if (regulator_inited)
		return ret;

	/* please only get regulator once in a driver */
	disp_bias_pos = regulator_get(NULL, "dsv_pos");
	if (IS_ERR(disp_bias_pos)) { /* handle return value */
		ret = PTR_ERR(disp_bias_pos);
		pr_info("get dsv_pos fail, error: %d\n", ret);
		return ret;
	}

	disp_bias_neg = regulator_get(NULL, "dsv_neg");
	if (IS_ERR(disp_bias_neg)) { /* handle return value */
		ret = PTR_ERR(disp_bias_neg);
		pr_info("get dsv_neg fail, error: %d\n", ret);
		return ret;
	}

	regulator_inited = 1;
	return ret; /* must be 0 */

}
EXPORT_SYMBOL(display_bias_regulator_init);

int disp_late_bias_enable(void)
{
	int ret = 0;
	int retval = 0;

	display_bias_regulator_init();

	ret = regulator_enable(disp_bias_pos);
	if (ret < 0)
		pr_info("enable regulator disp_bias_pos fail, ret = %d\n",
		ret);
	retval |= ret;

	ret = regulator_enable(disp_bias_neg);
	if (ret < 0)
		pr_info("enable regulator disp_bias_neg fail, ret = %d\n",
		ret);
	retval |= ret;

	return retval;
}
EXPORT_SYMBOL(disp_late_bias_enable);

int display_bias_enable(void)
{
	int ret = 0;
	int retval = 0;

	display_bias_regulator_init();

	/* set voltage with min & max*/
//Antaiui <AI_BSP_TP> <chenht> <2023-02-04> modify for 2206 begin	
#if defined(AI_TD4160_HUAXIAN_60hz) || defined(AI_TD4160_HUAXIAN_90hz) || defined(AI_ILI9883A_TRULY_60HZ)	
	ret = regulator_set_voltage(disp_bias_pos, 6000000, 6000000);
#elif defined(AI_FT8057_YIKUAILAI_60HZ) || defined(AI_FT8057_YIKUAILAI_90HZ) || defined(AI_ICNL9916_YIKUAILAI_90HZ)	|| defined(AI_ICNL9916_YIKUAILAI_60HZ)
	ret = regulator_set_voltage(disp_bias_pos, 5800000, 5800000);
#elif defined(AI_TRULY_NT36672C)	
	ret = regulator_set_voltage(disp_bias_pos, 5600000, 5600000);
#else
	ret = regulator_set_voltage(disp_bias_pos, 5400000, 5400000);
#endif
//Antaiui <AI_BSP_TP> <chenht> <2023-02-04> modify for 2206 end
	if (ret < 0)
		pr_info("set voltage disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;

//Antaiui <AI_BSP_TP> <chenht> <2023-02-04> modify for 2206 begin	
#if defined(AI_TD4160_HUAXIAN_60hz) || defined(AI_TD4160_HUAXIAN_90hz) || defined(AI_ILI9883A_TRULY_60HZ)
	ret = regulator_set_voltage(disp_bias_neg, 6000000, 6000000);
#elif defined(AI_FT8057_YIKUAILAI_60HZ) || defined(AI_FT8057_YIKUAILAI_90HZ) || defined(AI_ICNL9916_YIKUAILAI_90HZ)	|| defined(AI_ICNL9916_YIKUAILAI_60HZ)
	ret = regulator_set_voltage(disp_bias_neg, 5800000, 5800000);
#elif defined(AI_TRULY_NT36672C)	
	ret = regulator_set_voltage(disp_bias_neg, 5600000, 5600000);
#else
	ret = regulator_set_voltage(disp_bias_neg, 5400000, 5400000);
#endif
//Antaiui <AI_BSP_TP> <chenht> <2023-02-04> modify for 2206 end
	if (ret < 0)
		pr_info("set voltage disp_bias_neg fail, ret = %d\n", ret);
	retval |= ret;

	/* enable regulator */
	ret = regulator_enable(disp_bias_pos);
	if (ret < 0)
		pr_info("enable regulator disp_bias_pos fail, ret = %d\n",
			ret);
	retval |= ret;

	ret = regulator_enable(disp_bias_neg);
	if (ret < 0)
		pr_info("enable regulator disp_bias_neg fail, ret = %d\n",
			ret);
	retval |= ret;

	return retval;
}
EXPORT_SYMBOL(display_bias_enable);

int display_bias_disable(void)
{
	int ret = 0;
	int retval = 0;

	display_bias_regulator_init();

	ret = regulator_disable(disp_bias_neg);
	if (ret < 0)
		pr_info("disable regulator disp_bias_neg fail, ret = %d\n",
			ret);
	retval |= ret;

	ret = regulator_disable(disp_bias_pos);
	if (ret < 0)
		pr_info("disable regulator disp_bias_pos fail, ret = %d\n",
			ret);
	retval |= ret;

	return retval;
}
EXPORT_SYMBOL(display_bias_disable);
//Antaiui <AI_BSP_LCD> <hehl> <20200826> for 1907 start
int PMU_db_pos_neg_setting_delay_hct(int ms, int vol)
{
	int ret = 0;
	int retval = 0;
	vol = vol * 100000;		//convert
	display_bias_regulator_init();
	ret = regulator_set_voltage(disp_bias_pos, vol, vol);
	if (ret < 0)
		pr_err("set voltage disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;
	ret = regulator_set_voltage(disp_bias_neg, vol, vol);
	if (ret < 0)
		pr_err("set voltage disp_bias_neg fail, ret = %d\n", ret);
	retval |= ret;
	ret = regulator_enable(disp_bias_pos);
	if (ret < 0)
		pr_err("enable regulator disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;
	if (ms > 0)
		mdelay(ms);
	ret = regulator_enable(disp_bias_neg);
	if (ret < 0)
		pr_err("enable regulator disp_bias_neg fail, ret = %d\n", ret);
	retval |= ret;
	return retval;
}
EXPORT_SYMBOL(PMU_db_pos_neg_setting_delay_hct);
int PMU_db_pos_neg_setting_delay(int ms)
{
	int ret = 0;
	int retval = 0;
	display_bias_regulator_init();
//Antaiui <AI_BSP_TP> <chenht> <2023-02-04> modify for 2206 begin
#if defined(AI_TD4160_HUAXIAN_60hz) || defined(AI_TD4160_HUAXIAN_90hz) || defined(AI_ILI9883A_TRULY_60HZ)
	ret = regulator_set_voltage(disp_bias_pos, 6000000, 6000000);
#elif defined(AI_FT8057_YIKUAILAI_60HZ) || defined(AI_FT8057_YIKUAILAI_90HZ) || defined(AI_ICNL9916_YIKUAILAI_90HZ) || defined(AI_ICNL9916_YIKUAILAI_60HZ)
	ret = regulator_set_voltage(disp_bias_pos, 5800000, 5800000);
#elif defined(AI_TRULY_NT36672C)	
	ret = regulator_set_voltage(disp_bias_pos, 5600000, 5600000);
#else
	ret = regulator_set_voltage(disp_bias_pos, 5400000, 5400000);
#endif	
//Antaiui <AI_BSP_TP> <chenht> <2023-02-04> modify for 2206 end
	if (ret < 0)
		pr_err("set voltage disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;
//Antaiui <AI_BSP_TP> <chenht> <2023-02-04> modify for 2206 begin
#if defined(AI_TD4160_HUAXIAN_60hz) || defined(AI_TD4160_HUAXIAN_90hz) || defined(AI_ILI9883A_TRULY_60HZ)	
	ret = regulator_set_voltage(disp_bias_neg, 6000000, 6000000);
#elif defined(AI_FT8057_YIKUAILAI_60HZ) || defined(AI_FT8057_YIKUAILAI_90HZ) || defined(AI_ICNL9916_YIKUAILAI_90HZ)	|| defined(AI_ICNL9916_YIKUAILAI_60HZ)
	ret = regulator_set_voltage(disp_bias_neg, 5800000, 5800000);
#elif defined(AI_TRULY_NT36672C)	
	ret = regulator_set_voltage(disp_bias_neg, 5600000, 5600000);	
#else
	ret = regulator_set_voltage(disp_bias_neg, 5400000, 5400000);	
#endif
//Antaiui <AI_BSP_TP> <chenht> <2023-02-04> modify for 2206 end
	if (ret < 0)
		pr_err("set voltage disp_bias_neg fail, ret = %d\n", ret);
	retval |= ret;
#if 0
	ret = mtk_regulator_get_voltage(&disp_bias_pos);
	if (ret < 0)
		pr_err("get voltage disp_bias_pos fail\n");
	pr_debug("pos voltage = %d\n", ret);
	ret = mtk_regulator_get_voltage(&disp_bias_neg);
	if (ret < 0)
		pr_err("get voltage disp_bias_neg fail\n");
	pr_debug("neg voltage = %d\n", ret);
#endif
	ret = regulator_enable(disp_bias_pos);
	if (ret < 0)
		pr_err("enable regulator disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;
	if (ms > 0)
		mdelay(ms);
	ret = regulator_enable(disp_bias_neg);
	if (ret < 0)
		pr_err("enable regulator disp_bias_neg fail, ret = %d\n", ret);
	retval |= ret;
	return retval;
}
EXPORT_SYMBOL(PMU_db_pos_neg_setting_delay);
int PMU_db_pos_neg_disable_delay(int ms)
{
	int ret = 0;
	int retval = 0;
	display_bias_regulator_init();
	ret = regulator_disable(disp_bias_neg);
	if (ret < 0)
		pr_err("disable regulator disp_bias_neg fail, ret = %d\n", ret);
	retval |= ret;
	if (ms > 0)
		mdelay(ms);
	ret = regulator_disable(disp_bias_pos);
	if (ret < 0)
		pr_err("disable regulator disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;
	return retval;
}
EXPORT_SYMBOL(PMU_db_pos_neg_disable_delay);


//ctrl by ext bin, single pin ctrl enn + enp
int RT5081_db_pos_neg_setting(void)
{	
	int ret = 0;
	ret = display_bias_regulator_init();
	if(regulator_is_enabled(disp_bias_neg))
	{
		printk("%s: regulator is enabled,return\n",__func__);
		return 1;
	}
	display_bias_enable();
	return ret;
}

//ctrl by i2c
int RT5081_db_pos_neg_setting_by_i2c(int mdelay)
{	
	int ret = 0;

	ret = display_bias_regulator_init();
	if(regulator_is_enabled(disp_bias_neg))
	{
		printk("%s: regulator is enabled,return\n",__func__);
		return 1;
	}

	PMU_db_pos_neg_setting_delay(mdelay);
	return ret;
}

//ctrl by ext bin, single pin ctrl enn + enp
int RT5081_db_pos_neg_disable(void)
{
	int ret = 0;
	display_bias_disable();
	return ret;
}

//ctrl by i2c
int RT5081_db_pos_neg_disable_by_i2c(int ms)
{
	int ret = 0;
	ret = PMU_db_pos_neg_disable_delay(ms);
	return ret;
}
//Antaiui <AI_BSP_LCD> <hehl> <20200826> for 1907 endif
#else
int display_bias_regulator_init(void)
{
	return 0;
}
EXPORT_SYMBOL(display_bias_regulator_init);

int display_bias_enable(void)
{
	return 0;
}
EXPORT_SYMBOL(display_bias_enable);

int disp_late_bias_enable(void)
{
	return 0;
}
EXPORT_SYMBOL(disp_late_bias_enable);

int display_bias_disable(void)
{
	return 0;
}
EXPORT_SYMBOL(display_bias_disable);
#endif

