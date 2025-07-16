/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/leds.h>

#include "ai_side_torch.h"

#define AW36515_DTNAME_I2C  "mediatek,side_torch"
#define AW36515_DTNAME      "awinic,aw36515"
#define AW36515_NAME        "aw36515"

/* define registers */
#define AW36515_REG_ENABLE              0x01
#define AW36515_REG_IVFM                0x02
#define AW36515_REG_FLASH_LEVEL_LED1    0x03
#define AW36515_REG_FLASH_LEVEL_LED2    0x04
#define AW36515_REG_TORCH_LEVEL_LED1    0x05
#define AW36515_REG_TORCH_LEVEL_LED2    0x06
#define AW36515_REG_BOOST_CONFIG        0x07
#define AW36515_REG_TIMING_CONFIG       0x08
#define AW36515_REG_TEMP                0x09
#define AW36515_REG_FLAG1               0x0A
#define AW36515_REG_FLAG2               0x0B
#define AW36515_REG_DEVICE_ID           0x0C

#define AW36515_LEVEL_TORCH 10
#define AW36515_LEVEL_FLASH 31

/* define channel, level */
#define AW36515_CHANNEL_NUM 2
#define AW36515_CHANNEL_CH1 0
#define AW36515_CHANNEL_CH2 1

#define AW36515_NONE (-1)
#define AW36515_DISABLE 0

#define AW36515_ENABLE_TORCH 0x0B
#define AW36515_ENABLE_FLASH 0x0F

#define AW36515_ENABLE_LED1_TORCH 0x09	// 1001
#define AW36515_ENABLE_LED2_TORCH 0x0A	// 1010
#define AW36515_ENABLE_LED1_FLASH 0x0D	// 1101
#define AW36515_ENABLE_LED2_FLASH 0x0E	// 1110
#define AW36515_ENABLE_INFRARED   0x07	// 0111

#define AW36515_BOOST_CONF_RESET 0x09	// 1001
#define AW36515_TORCH_RAMP_TIME  0x01	// 0001 // 1ms
#define AW36515_FLASH_TIMEOUT    0x10	// 1010 // 600ms

#define AW36515_WAIT_TIME 3
#define AW36515_RETRY_TIMES 3

/* TODO: define register */

/* define mutex, work queue and timer */
static DEFINE_MUTEX(aw36515_mutex);
static struct work_struct aw36515_work_ch1;
static struct work_struct aw36515_work_ch2;
static struct hrtimer aw36515_timer_ch1;
static struct hrtimer aw36515_timer_ch2;
static unsigned int aw36515_timeout_ms[AW36515_CHANNEL_NUM];

/* define i2c */
static struct i2c_client *AW36515_i2c_client;

/* define torch level */
static volatile int AW36515_torch_level = -1;
static volatile unsigned char AW36515_reg_enable;

/* platform data */
struct aw36515_platform_data {
	u8 torch_pin_enable;
	u8 pam_sync_pin_enable;
	u8 thermal_comp_mode_enable;
	u8 strobe_pin_disable;
	u8 vout_mode_enable;
};

/* aw36515 chip data */
struct aw36515_chip_data {
	struct i2c_client *client;
	struct aw36515_platform_data *pdata;
	struct mutex lock;
	u8 last_flag;
	u8 no_pdata;
};

// Flash Brightness Levels
// current = val * 7.83 + 3.91
static const unsigned char aw36515_flash_level[AW36515_LEVEL_FLASH] = {
	0x00, 0x08, 0x11, 0x19, 0x22, 0x2A, 0x33, 0x3B, 0x44, 0x4C,
	0x55, 0x5D, 0x66, 0x6E, 0x77, 0x7F, 0x88, 0x90, 0x99, 0xA1,
	0xAA, 0xB2, 0xBB, 0xC3, 0xCC, 0xD4, 0xDD, 0xE5, 0xEE, 0xF6,
	0xFF
};

// Antaiui <AI_BSP_CAM> <xieht> <2021-08-18> torch level begin
// Torch Brightness Levels
// current = val * 1.96 + 0.98
static const unsigned char aw36515_torch_level[AW36515_LEVEL_TORCH] = {
	// 0x19, 0x33, 0x4C, 0x66, 0x7F, 0x99, 0xb2, 0xCC, 0xE5, 0xFF
	0x10, 0x1E, 0x2C, 0x3A, 0x48, 0x56, 0x64, 0x72, 0x80, 0x8E
};
// Antaiui <AI_BSP_CAM> <xieht> <2021-08-18> torch level end

/* i2c wrapper function */
static inline int aw36515_flash_write(struct i2c_client *client, u8 reg, u8 val)
{
	int ret;
	struct aw36515_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	ret = i2c_smbus_write_byte_data(client, reg, val);
	mutex_unlock(&chip->lock);

	if (ret < 0)
		pr_err("[%s] failed writing at 0x%02x\n", __func__, reg);

	return ret;
}

static inline int aw36515_flash_read(struct i2c_client *client, u8 reg)
{
	int ret;
	struct aw36515_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	ret = i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&chip->lock);

	pr_info("%s reg:0x%x val:0x%x\n", __func__, reg, ret);
	if (ret < 0)
		pr_err("[%s] failed reading at 0x%02x\n", __func__, reg);

	return ret;
}

int aw36515_is_torch(int level)
{
	if (level > 7)	// 350mA // AW36515_LEVEL_TORCH
		return -1;
	return 0;
}

int aw36515_verify_level(int level)
{
	if (level < 0)
		level = 0;
	if (level >= AW36515_LEVEL_FLASH)
		level = AW36515_LEVEL_FLASH - 1;
	AW36515_torch_level = level;
	return level;
}

int aw36515_set_level(int level)
{
	unsigned char reg, val;
	int ret = 0;

    pr_debug("[%s] level: %d\n", __func__, level);
	level = aw36515_verify_level(level);
	if (!aw36515_is_torch(level)) {
		reg = AW36515_REG_TORCH_LEVEL_LED1;
		val = aw36515_torch_level[level];
		ret = aw36515_flash_write(AW36515_i2c_client, reg, val);

		reg = AW36515_REG_TORCH_LEVEL_LED2;
		ret = aw36515_flash_write(AW36515_i2c_client, reg, val);
	} else {
		reg = AW36515_REG_FLASH_LEVEL_LED1;
		val = aw36515_flash_level[level];
		ret = aw36515_flash_write(AW36515_i2c_client, reg, val);

		reg = AW36515_REG_FLASH_LEVEL_LED2;
		ret = aw36515_flash_write(AW36515_i2c_client, reg, val);
	}

	return ret;
}

int aw36515_enable_ch1(void)
{
	unsigned char reg, val;

	reg = AW36515_REG_ENABLE;
	if (!aw36515_is_torch(AW36515_torch_level))
		AW36515_reg_enable |= AW36515_ENABLE_LED1_TORCH;
	else
		AW36515_reg_enable |= AW36515_ENABLE_LED1_FLASH;
	val = AW36515_reg_enable;

	return aw36515_flash_write(AW36515_i2c_client, reg, val);
}

int aw36515_disable_ch1(void)
{
	unsigned char reg, val;

	reg = AW36515_REG_ENABLE;
	AW36515_reg_enable &= ~AW36515_ENABLE_LED1_FLASH;
	val = AW36515_reg_enable;

	return aw36515_flash_write(AW36515_i2c_client, reg, val);
}

int aw36515_enable_ch2(void)
{
	unsigned char reg, val;

	reg = AW36515_REG_ENABLE;
	if (!aw36515_is_torch(AW36515_torch_level))
		AW36515_reg_enable |= AW36515_ENABLE_LED2_TORCH;
	else
		AW36515_reg_enable |= AW36515_ENABLE_LED2_FLASH;
	val = AW36515_reg_enable;

	return aw36515_flash_write(AW36515_i2c_client, reg, val);
}

int aw36515_disable_ch2(void)
{
	unsigned char reg, val;

	reg = AW36515_REG_ENABLE;
	AW36515_reg_enable &= ~AW36515_ENABLE_LED2_FLASH;
	val = AW36515_reg_enable;

	return aw36515_flash_write(AW36515_i2c_client, reg, val);
}

int aw36515_enable(void)
{
	unsigned char reg, val;

	reg = AW36515_REG_ENABLE;
	if (!aw36515_is_torch(AW36515_torch_level))
		AW36515_reg_enable |= (AW36515_ENABLE_LED1_TORCH | AW36515_ENABLE_LED2_TORCH);
	else
		AW36515_reg_enable |= (AW36515_ENABLE_LED1_FLASH | AW36515_ENABLE_LED2_FLASH);
	val = AW36515_reg_enable;

	return aw36515_flash_write(AW36515_i2c_client, reg, val);
}

int aw36515_disable(void)
{
	unsigned char reg, val;

	reg = AW36515_REG_ENABLE;
	AW36515_reg_enable = AW36515_DISABLE;
	val = AW36515_reg_enable;

	return aw36515_flash_write(AW36515_i2c_client, reg, val);
}

int aw36515_chip_init(struct aw36515_chip_data *chip)
{
	unsigned char reg, val;
	int ret;

	pr_info("%s Entry!\n", __func__);

	AW36515_reg_enable = AW36515_DISABLE;
	aw36515_disable();

	reg = AW36515_REG_BOOST_CONFIG;
	val = AW36515_BOOST_CONF_RESET;
	ret = aw36515_flash_write(AW36515_i2c_client, reg, val);

	reg = AW36515_REG_TIMING_CONFIG;
	val = AW36515_TORCH_RAMP_TIME | AW36515_FLASH_TIMEOUT;
	ret = aw36515_flash_write(AW36515_i2c_client, reg, val);

	return ret;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//AW36515 Debug file
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static ssize_t aw36515_get_reg(struct device* cd,struct device_attribute *attr, char* buf);
static ssize_t aw36515_set_reg(struct device* cd, struct device_attribute *attr, const char* buf, size_t len);
static DEVICE_ATTR(reg, 0660, aw36515_get_reg,  aw36515_set_reg);

static ssize_t aw36515_get_reg(struct device* cd,struct device_attribute *attr, char* buf)
{
	unsigned char reg_val;
	unsigned char i;
	ssize_t len = 0;
	for(i=0;i<=0x0D;i++)
	{
		reg_val = aw36515_flash_read(AW36515_i2c_client, i);
		len += snprintf(buf+len, PAGE_SIZE-len, "reg0x%2X = 0x%2X \n", i,reg_val);
	}
	len += snprintf(buf+len, PAGE_SIZE-len, "\r\n");
	return len;
}

static ssize_t aw36515_set_reg(struct device* cd, struct device_attribute *attr, const char* buf, size_t len)
{
	unsigned int databuf[2];
	if(2 == sscanf(buf,"%x %x",&databuf[0], &databuf[1]))
	{
		aw36515_flash_write(AW36515_i2c_client, databuf[0], databuf[1]);
	}
	return len;
}

static void aw36515_create_sysfs(struct i2c_client *client)
{
	struct device *dev = &(client->dev);
	device_create_file(dev, &dev_attr_reg);
}

static void aw36515_delete_sysfs(struct i2c_client *client)
{
	struct device *dev = &(client->dev);
	device_remove_file(dev, &dev_attr_reg);
}

int set_side_torch(int level)
{
	int ret;

	if (level <= 0) {
		pr_info("[%s] OFF!\n", __func__, level);
		ret = aw36515_disable();
		return ret;
	}
	//Antaiui <AI_BSP_CAM> <yaoyc> <2022-06-21> add for ASW2200 clear OVP flag begin
	ret = aw36515_flash_read(AW36515_i2c_client, AW36515_REG_FLAG1); 
	ret = aw36515_flash_read(AW36515_i2c_client, AW36515_REG_FLAG2);
	//Antaiui <AI_BSP_CAM> <yaoyc> <2022-06-21> add for ASW2200 clear OVP flag end
	
	aw36515_set_level(level - 1);
	ret = aw36515_enable();

	pr_info("[%s] ON!\n", __func__, level);
	return ret;
}

int get_torch_level(void)
{
	float curr;
	#if 0
	pr_debug("[%s] level: %d.\n", __func__, AW36515_torch_level);
	if (!aw36515_is_torch(AW36515_torch_level)) {
		// torch current = val * 1.96 + 0.98
		curr = aw36515_torch_level[AW36515_torch_level] * 1.96 + 0.98;
	} else {
		// flash current = val * 7.83 + 3.91
		curr = aw36515_flash_level[AW36515_torch_level] * 7.83 + 3.91;
	}
	#endif
	return (int)curr;
}

/******************************************************************************
 * Timer and work queue
 *****************************************************************************/
static void aw36515_work_disable_ch1(struct work_struct *data)
{
	pr_debug("ht work queue callback\n");
	aw36515_disable_ch1();
}

static void aw36515_work_disable_ch2(struct work_struct *data)
{
	pr_debug("ht work queue callback\n");
	aw36515_disable_ch2();
}

static enum hrtimer_restart aw36515_timer_func_ch1(struct hrtimer *timer)
{
	schedule_work(&aw36515_work_ch1);
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart aw36515_timer_func_ch2(struct hrtimer *timer)
{
	schedule_work(&aw36515_work_ch2);
	return HRTIMER_NORESTART;
}

static int aw36515_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct aw36515_chip_data *chip;
	struct aw36515_platform_data *pdata = client->dev.platform_data;
	int err;

	pr_info("%s start.\n", __func__);
	/* check i2c */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_info("Failed to check i2c functionality.\n");
		err = -ENODEV;
		goto err_out;
	}
	/* init chip private data */
	chip = kzalloc(sizeof(struct aw36515_chip_data), GFP_KERNEL);
	if (!chip) {
		err = -ENOMEM;
		goto err_out;
	}
	chip->client = client;

	/* init platform data */
	if (!pdata) {
		pr_debug("Platform data does not exist\n");
		pdata = kzalloc(sizeof(struct aw36515_platform_data), GFP_KERNEL);
        if (!pdata) {
			err = -ENOMEM;
			goto err_init_pdata;
		}
		chip->no_pdata = 1;
	}
	chip->pdata = pdata;
	i2c_set_clientdata(client, chip);
	AW36515_i2c_client = client;

	/* init mutex and spinlock */
	mutex_init(&chip->lock);

	/* init work queue */
	INIT_WORK(&aw36515_work_ch1, aw36515_work_disable_ch1);
	INIT_WORK(&aw36515_work_ch2, aw36515_work_disable_ch2);

	/* init timer */
	hrtimer_init(&aw36515_timer_ch1, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	hrtimer_init(&aw36515_timer_ch2, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aw36515_timer_ch1.function = aw36515_timer_func_ch1;
	aw36515_timer_ch2.function = aw36515_timer_func_ch2;
	aw36515_timeout_ms[AW36515_CHANNEL_CH1] = 100;
	aw36515_timeout_ms[AW36515_CHANNEL_CH2] = 100;

	/* init chip hw */
	aw36515_chip_init(chip);

	aw36515_create_sysfs(client);
    side_torch_init(&(client->dev));

	pr_info("Probe done.\n");
	return 0;

err_init_pdata:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
err_out:
	return err;
}

static int aw36515_i2c_remove(struct i2c_client *client)
{
	struct aw36515_chip_data *chip = i2c_get_clientdata(client);

	pr_info("Remove start.\n");

    aw36515_delete_sysfs(client);
    side_torch_exit(&(client->dev));

	/* flush work queue */
	flush_work(&aw36515_work_ch1);
	flush_work(&aw36515_work_ch2);

	/* free resource */
	if (chip->no_pdata)
		kfree(chip->pdata);
	kfree(chip);

	pr_info("Remove done.\n");

	return 0;
}

static const struct i2c_device_id aw36515_i2c_id[] = {
	{AW36515_NAME, 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, aw36515_i2c_id);

#ifdef CONFIG_OF
static const struct of_device_id aw36515_i2c_of_match[] = {
	{.compatible = AW36515_DTNAME_I2C},
	{},
};
MODULE_DEVICE_TABLE(of, aw36515_i2c_of_match);
#endif

static struct i2c_driver aw36515_i2c_driver = {
	.driver = {
		   .name = AW36515_NAME,
#ifdef CONFIG_OF
		   .of_match_table = aw36515_i2c_of_match,
#endif
		   },
	.probe = aw36515_i2c_probe,
	.remove = aw36515_i2c_remove,
	.id_table = aw36515_i2c_id,
};

/******************************************************************************
 * Platform device and driver
 *****************************************************************************/
static int aw36515_probe(struct platform_device *dev)
{
	pr_info("aw36515_platform_probe start.\n");

	if (i2c_add_driver(&aw36515_i2c_driver)) {
		pr_debug("Failed to add i2c driver.\n");
		return -1;
	}

	pr_info("%s done.\n", __func__);

	return 0;
}

static int aw36515_remove(struct platform_device *dev)
{
	pr_debug("Remove start.\n");

	i2c_del_driver(&aw36515_i2c_driver);

	pr_debug("Remove done.\n");

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id aw36515_of_match[] = {
	{.compatible = AW36515_DTNAME},
	{},
};
MODULE_DEVICE_TABLE(of, aw36515_of_match);
#endif

static struct platform_device aw36515_platform_device = {
		.name = AW36515_NAME,
		.id = 0,
};
MODULE_DEVICE_TABLE(platform, aw36515_platform_device);


static struct platform_driver aw36515_platform_driver = {
	.probe = aw36515_probe,
	.remove = aw36515_remove,
	.driver = {
		.name = AW36515_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = aw36515_of_match,
#endif
	},
};

static int __init flashlight_aw36515_init(void)
{
	int ret;

	pr_info("flashlight_aw36515_initInit start.\n");

#if 1//ndef CONFIG_OF
	ret = platform_device_register(&aw36515_platform_device);
	if (ret) {
		pr_info("Failed to register platform device\n");
		return ret;
	}
#endif

	ret = platform_driver_register(&aw36515_platform_driver);
	if (ret) {
		pr_info("Failed to register platform driver\n");
		return ret;
	}

	pr_debug("Init done.\n");

	return 0;
}

static void __exit flashlight_aw36515_exit(void)
{
	pr_debug("Exit start.\n");

	platform_driver_unregister(&aw36515_platform_driver);

	pr_debug("Exit done.\n");
}

module_init(flashlight_aw36515_init);
module_exit(flashlight_aw36515_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Simon Wang <Simon-TCH.Wang@mediatek.com>");
MODULE_DESCRIPTION("MTK Flashlight AW36515 Driver");
