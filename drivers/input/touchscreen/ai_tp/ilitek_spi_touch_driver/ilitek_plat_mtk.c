/*
 * ILITEK Touch IC driver
 *
 * Copyright (C) 2011 ILI Technology Corporation.
 *
 * Author: Dicky Chiang <dicky_chiang@ilitek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include "ilitek.h"
#include "../tpd.h"

#define DTS_INT_GPIO	"touch,irq-gpio"
#define DTS_RESET_GPIO	"touch,reset-gpio"
#define DTS_OF_NAME	"mediatek,cap_touch_ilitek_spi"
#define MTK_RST_GPIO	GTP_RST_PORT
#define MTK_INT_GPIO	GTP_INT_PORT

extern struct tpd_device *tpd;

//hct-drv add for fwvno  begin
static struct tpd_driver_t tpd_device_driver;
extern u32 fwver;
//hct-drv add for fwvno  end

//Antaiui <AI_BSP_CTP> <hehl> <2020-5-28> add gesture  start
#ifdef CONFIG_AI_BSP_GESTURE_SWITCH
int tpgesture_status=0;
int ilitek_wake_switch = 0;
int ilitek_gesture_switch = 0;
struct ilitek_data {
    bool glove_mode;
    bool cover_mode;
    bool charger_mode;
    bool gesture_mode;  
	bool landscape_mode;
};
struct ilitek_data *itk_data;
struct mutex ilitek_mutex;
unsigned int gesture_cfg;
ssize_t ilitek_double_wake_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", ilitek_wake_switch);
}

ssize_t ilitek_double_wake_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int rt;
    unsigned long val;
    mutex_lock(&ilitek_mutex);

    rt = kstrtoul(buf, 10, &val);
    if(rt != 0){
        ipio_err("%s, invalid value\n", __func__);
        return rt;
    }
    ilitek_wake_switch = val;
    tpgesture_status= ilitek_gesture_switch || ilitek_wake_switch;

    ipio_info("%s, tpgesture_status=%d\n", __func__, tpgesture_status);
     mutex_unlock(&ilitek_mutex);

    return count;
}

ssize_t ilitek_gesture_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", ilitek_gesture_switch);
}

ssize_t ilitek_gesture_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int rt;
    unsigned long val;
	mutex_lock(&ilitek_mutex);

    rt = kstrtoul(buf, 10, &val);
    if(rt != 0){
        ipio_err("%s, invalid value\n", __func__);
        return rt;
    }
    ilitek_gesture_switch = val;
    tpgesture_status= ilitek_gesture_switch || ilitek_wake_switch;

    ipio_info("%s, tpgesture_status=%d\n", __func__, tpgesture_status);
    mutex_unlock(&ilitek_mutex);

    return count;
}

static ssize_t ilitek_gesture_config_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf,"%u\n", gesture_cfg);
}

static ssize_t ilitek_gesture_config_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int rt;
	unsigned long val;

	mutex_lock(&ilitek_mutex);
	rt = kstrtoul(buf, 10, &val);
	if(rt != 0) {
		ipio_err("invalid value \n");
	}

	gesture_cfg = val & 0xffff;
	ipio_info(" %s val=%lu \n",__func__,val);
	mutex_unlock(&ilitek_mutex);

	return count;
}

static DEVICE_ATTR(gesture_config, S_IRUGO|S_IWUSR, ilitek_gesture_config_show, ilitek_gesture_config_store);
static DEVICE_ATTR (double_wake, S_IRUGO|S_IWUSR, ilitek_double_wake_show, ilitek_double_wake_store);
static DEVICE_ATTR (gesture_wake, S_IRUGO|S_IWUSR, ilitek_gesture_show, ilitek_gesture_store);
extern ssize_t ilitek_factory_test_show(struct device *dev, struct device_attribute *attr, char *buf);
static DEVICE_ATTR(factory_check, S_IRUGO, ilitek_factory_test_show, NULL);
#endif
//Antaiui <AI_BSP_CTP> <hehl> <2020-5-28> add gesture  end

void ilitek_plat_tp_reset(void)
{
	ipio_info("edge delay = %d\n", idev->rst_edge_delay);

	/* Need accurate power sequence, do not change it to msleep */
	tpd_gpio_output(idev->tp_rst, 1);
	mdelay(1);
	tpd_gpio_output(idev->tp_rst, 0);
	mdelay(5);
	tpd_gpio_output(idev->tp_rst, 1);
	mdelay(idev->rst_edge_delay);
}

void ilitek_plat_input_register(void)
{
	int i;

	idev->input = tpd->dev;

	if (tpd_dts_data.use_tpd_button) {
		for (i = 0; i < tpd_dts_data.tpd_key_num; i++)
			input_set_capability(idev->input, EV_KEY, tpd_dts_data.tpd_key_local[i]);
	}

	/* set the supported event type for input device */
	set_bit(EV_ABS, idev->input->evbit);
	set_bit(EV_SYN, idev->input->evbit);
	set_bit(EV_KEY, idev->input->evbit);
	set_bit(BTN_TOUCH, idev->input->keybit);
	set_bit(BTN_TOOL_FINGER, idev->input->keybit);
	set_bit(INPUT_PROP_DIRECT, idev->input->propbit);

	if (MT_PRESSURE)
		input_set_abs_params(idev->input, ABS_MT_PRESSURE, 0, 255, 0, 0);

	if (MT_B_TYPE) {
#if KERNEL_VERSION(3, 7, 0) <= LINUX_VERSION_CODE
		input_mt_init_slots(idev->input, MAX_TOUCH_NUM, INPUT_MT_DIRECT);
#else
		input_mt_init_slots(idev->input, MAX_TOUCH_NUM);
#endif /* LINUX_VERSION_CODE */
	} else {
		input_set_abs_params(idev->input, ABS_MT_TRACKING_ID, 0, MAX_TOUCH_NUM, 0, 0);
	}

	/* Gesture keys register */
	input_set_capability(idev->input, EV_KEY, KEY_POWER);
	input_set_capability(idev->input, EV_KEY, KEY_GESTURE_UP);
	input_set_capability(idev->input, EV_KEY, KEY_GESTURE_DOWN);
	input_set_capability(idev->input, EV_KEY, KEY_GESTURE_O);
	input_set_capability(idev->input, EV_KEY, KEY_GESTURE_E);
	input_set_capability(idev->input, EV_KEY, KEY_GESTURE_M);
	input_set_capability(idev->input, EV_KEY, KEY_GESTURE_W);
	input_set_capability(idev->input, EV_KEY, KEY_GESTURE_S);
	input_set_capability(idev->input, EV_KEY, KEY_GESTURE_V);
	input_set_capability(idev->input, EV_KEY, KEY_GESTURE_Z);
	input_set_capability(idev->input, EV_KEY, KEY_GESTURE_C);
//Antaiui <AI_BSP_CTP> <hehl> <2020-5-28> add gesture  start
#ifdef CONFIG_AI_BSP_GESTURE_SWITCH	
	input_set_capability(idev->input, EV_KEY, KEY_DOUBLE_WAKE);
#endif	
//Antaiui <AI_BSP_CTP> <hehl> <2020-5-28> add gesture  end	

	__set_bit(KEY_GESTURE_POWER, idev->input->keybit);
	__set_bit(KEY_GESTURE_UP, idev->input->keybit);
	__set_bit(KEY_GESTURE_DOWN, idev->input->keybit);
	__set_bit(KEY_GESTURE_O, idev->input->keybit);
	__set_bit(KEY_GESTURE_E, idev->input->keybit);
	__set_bit(KEY_GESTURE_M, idev->input->keybit);
	__set_bit(KEY_GESTURE_W, idev->input->keybit);
	__set_bit(KEY_GESTURE_S, idev->input->keybit);
	__set_bit(KEY_GESTURE_V, idev->input->keybit);
	__set_bit(KEY_GESTURE_Z, idev->input->keybit);
	__set_bit(KEY_GESTURE_C, idev->input->keybit);
//Antaiui <AI_BSP_CTP> <hehl> <2020-5-28> add gesture  start	
#ifdef CONFIG_AI_BSP_GESTURE_SWITCH
	__set_bit(KEY_DOUBLE_WAKE, idev->input->keybit);
#endif
//Antaiui <AI_BSP_CTP> <hehl> <2020-5-28> add gesture  end	
}

#if REGULATOR_POWER
void ilitek_plat_regulator_power_on(bool status)
{
	ipio_info("%s\n", status ? "POWER ON" : "POWER OFF");

	if (status) {
		if (idev->vdd) {
			if (regulator_enable(idev->vdd) < 0)
				ipio_err("regulator_enable VDD fail\n");
		}
		if (idev->vcc) {
			if (regulator_enable(idev->vcc) < 0)
				ipio_err("regulator_enable VCC fail\n");
		}
	} else {
		if (idev->vdd) {
			if (regulator_disable(idev->vdd) < 0)
				ipio_err("regulator_enable VDD fail\n");
		}
		if (idev->vcc) {
			if (regulator_disable(idev->vcc) < 0)
				ipio_err("regulator_enable VCC fail\n");
		}
	}
	atomic_set(&idev->ice_stat, DISABLE);
	mdelay(5);
}
#endif

#if REGULATOR_POWER
static void ilitek_plat_regulator_power_init(void)
{
	const char *vdd_name = "vdd";
	const char *vcc_name = "vcc";

	idev->vdd = regulator_get(tpd->tpd_dev, vdd_name);
	if (ERR_ALLOC_MEM(idev->vdd)) {
		ipio_err("regulator_get VDD fail\n");
		idev->vdd = NULL;
	}

	tpd->reg = idev->vdd;

	if (regulator_set_voltage(idev->vdd, VDD_VOLTAGE, VDD_VOLTAGE) < 0)
		ipio_err("Failed to set VDD %d\n", VDD_VOLTAGE);

	idev->vcc = regulator_get(idev->dev, vcc_name);
	if (ERR_ALLOC_MEM(idev->vcc)) {
		ipio_err("regulator_get VCC fail.\n");
		idev->vcc = NULL;
	}
	if (regulator_set_voltage(idev->vcc, VCC_VOLTAGE, VCC_VOLTAGE) < 0)
		ipio_err("Failed to set VCC %d\n", VCC_VOLTAGE);

	ilitek_plat_regulator_power_on(true);
}
#endif

static int ilitek_plat_gpio_register(void)
{
	int ret = 0;

	idev->tp_int = MTK_INT_GPIO;
	idev->tp_rst = MTK_RST_GPIO;

	ipio_info("TP INT: %d\n", idev->tp_int);
	ipio_info("TP RESET: %d\n", idev->tp_rst);

	if (!gpio_is_valid(idev->tp_int)) {
		ipio_err("Invalid INT gpio: %d\n", idev->tp_int);
		return -EBADR;
	}

	if (!gpio_is_valid(idev->tp_rst)) {
		ipio_err("Invalid RESET gpio: %d\n", idev->tp_rst);
		return -EBADR;
	}

	ret = gpio_request(idev->tp_int, "TP_INT");
	if (ret < 0) {
		ipio_err("Request IRQ GPIO failed, ret = %d\n", ret);
		gpio_free(idev->tp_int);
		ret = gpio_request(idev->tp_int, "TP_INT");
		if (ret < 0) {
			ipio_err("Retrying request INT GPIO still failed , ret = %d\n", ret);
			goto out;
		}
	}

	ret = gpio_request(idev->tp_rst, "TP_RESET");
	if (ret < 0) {
		ipio_err("Request RESET GPIO failed, ret = %d\n", ret);
		gpio_free(idev->tp_rst);
		ret = gpio_request(idev->tp_rst, "TP_RESET");
		if (ret < 0) {
			ipio_err("Retrying request RESET GPIO still failed , ret = %d\n", ret);
			goto out;
		}
	}

out:
	gpio_direction_input(idev->tp_int);
	return ret;
}

void ilitek_plat_irq_disable(void)
{
	unsigned long flag;

	spin_lock_irqsave(&idev->irq_spin, flag);

	if (atomic_read(&idev->irq_stat) == DISABLE)
		goto out;

	if (!idev->irq_num) {
		ipio_err("gpio_to_irq (%d) is incorrect\n", idev->irq_num);
		goto out;
	}

	disable_irq_nosync(idev->irq_num);
	atomic_set(&idev->irq_stat, DISABLE);
	ipio_debug("Disable irq success\n");

out:
	spin_unlock_irqrestore(&idev->irq_spin, flag);
}

void ilitek_plat_irq_enable(void)
{
	unsigned long flag;

	spin_lock_irqsave(&idev->irq_spin, flag);

	if (atomic_read(&idev->irq_stat) == ENABLE)
		goto out;

	if (!idev->irq_num) {
		ipio_err("gpio_to_irq (%d) is incorrect\n", idev->irq_num);
		goto out;
	}

	enable_irq(idev->irq_num);
	atomic_set(&idev->irq_stat, ENABLE);
	ipio_debug("Enable irq success\n");

out:
	spin_unlock_irqrestore(&idev->irq_spin, flag);
}

static irqreturn_t ilitek_plat_isr_top_half(int irq, void *dev_id)
{
	if (irq != idev->irq_num) {
		ipio_err("Incorrect irq number (%d)\n", irq);
		return IRQ_NONE;
	}

	if (atomic_read(&idev->mp_int_check) == ENABLE) {
		atomic_set(&idev->mp_int_check, DISABLE);
		ipio_debug("interrupt for mp test, ignore\n");
		wake_up(&(idev->inq));
		return IRQ_HANDLED;
	}

	if (idev->prox_near) {
		ipio_info("Proximity event, ignore interrupt!\n");
		return IRQ_HANDLED;
	}
//Antaiui <AI_BSP_CTP> <hehl> <2020-10-27> modify start
	/*ipio_debug("report: %d, rst: %d, fw: %d, switch: %d, mp: %d, sleep: %d, esd: %d\n",
			idev->report,
			atomic_read(&idev->tp_reset),
			atomic_read(&idev->fw_stat),
			atomic_read(&idev->tp_sw_mode),
			atomic_read(&idev->mp_stat),
			atomic_read(&idev->tp_sleep),
			atomic_read(&idev->esd_stat));*/
//Antaiui <AI_BSP_CTP> <hehl> <2020-10-27> modify end
	if (!idev->report || atomic_read(&idev->tp_reset) ||
		atomic_read(&idev->fw_stat) || atomic_read(&idev->tp_sw_mode) ||
		atomic_read(&idev->mp_stat) || atomic_read(&idev->tp_sleep) ||
		atomic_read(&idev->esd_stat)) {
			ipio_debug("ignore interrupt !\n");
			return IRQ_HANDLED;
	}

	return IRQ_WAKE_THREAD;
}

static irqreturn_t ilitek_plat_isr_bottom_half(int irq, void *dev_id)
{
	if (mutex_is_locked(&idev->touch_mutex)) {
		ipio_debug("touch is locked, ignore\n");
		return IRQ_HANDLED;
	}
	mutex_lock(&idev->touch_mutex);
	ilitek_tddi_report_handler();
	mutex_unlock(&idev->touch_mutex);
	return IRQ_HANDLED;
}

void ilitek_plat_irq_unregister(void)
{
	devm_free_irq(idev->dev, idev->irq_num, NULL);
}

int ilitek_plat_irq_register(int type)
{
	int ret = 0;
	static bool get_irq_pin;
	struct device_node *node;

	atomic_set(&idev->irq_stat, DISABLE);

	if (get_irq_pin == false) {
		node = of_find_matching_node(NULL, touch_of_match);
		if (node)
			idev->irq_num = irq_of_parse_and_map(node, 0);

		ipio_info("idev->irq_num = %d\n", idev->irq_num);
		get_irq_pin = true;
	}

	ret = devm_request_threaded_irq(idev->dev, idev->irq_num,
				ilitek_plat_isr_top_half,
				ilitek_plat_isr_bottom_half,
				type | IRQF_ONESHOT, "ilitek", NULL);
	enable_irq_wake(idev->irq_num);// modify by mtk for fix bug(deep sleep cannot resume) 
	if (type == IRQF_TRIGGER_FALLING)
		ipio_info("IRQ TYPE = IRQF_TRIGGER_FALLING\n");
	if (type == IRQF_TRIGGER_RISING)
		ipio_info("IRQ TYPE = IRQF_TRIGGER_RISING\n");

	if (ret != 0)
		ipio_err("Failed to register irq handler, irq = %d, ret = %d\n", idev->irq_num, ret);

	atomic_set(&idev->irq_stat, ENABLE);

	return ret;
}

static void tpd_resume(struct device *h)
{
	if (ilitek_tddi_sleep_handler(TP_RESUME) < 0)
		ipio_err("TP resume failed\n");
}

static void tpd_suspend(struct device *h)
{
	if (ilitek_tddi_sleep_handler(TP_SUSPEND) < 0)
		ipio_err("TP suspend failed\n");
}

//hct-drv add start
#ifdef ILITEK_ENABLE_GESTURE
char tpgesture_status_value[5] = {};

static ssize_t show_tpgesture_value(struct device* dev, struct device_attribute *attr, char *buf)
{
	printk("show tp gesture value is %s \n",tpgesture_value);
	return scnprintf(buf, PAGE_SIZE, "%s\n", tpgesture_value);
}
static ssize_t show_tpgesture_status_value(struct device* dev, struct device_attribute *attr, char *buf)
{
	printk("show tp gesture status is %s \n",tpgesture_status_value);
	return scnprintf(buf, PAGE_SIZE, "%s\n", tpgesture_status_value);
}
static ssize_t store_tpgesture_status_value(struct device* dev, struct device_attribute *attr, const char *buf, size_t count)
{
	if(!strncmp(buf, "on", 2))
	{
		sprintf(tpgesture_status_value,"on");
		tpgesture_status = 1;//status --- on
	}
	else
	{
		sprintf(tpgesture_status_value,"off");
		tpgesture_status = 0;//status --- off
	}
	printk("store_tpgesture_status_value status is %s \n",tpgesture_status_value);
	return count;
}
static DEVICE_ATTR(tpgesture,  0664, show_tpgesture_value, NULL);
static DEVICE_ATTR(tpgesture_status,  0664, show_tpgesture_status_value, store_tpgesture_status_value);
static struct device_attribute *tpd_attr_list[] = {
	&dev_attr_tpgesture,
	&dev_attr_tpgesture_status,
};
#endif
//hct-drv add end

static int ilitek_plat_probe(void)
{
	ipio_info("platform probe\n");
//Antaiui <AI_BSP_CTP> <hehl> <2020-5-28> add gesture  start
#ifdef CONFIG_AI_BSP_GESTURE_SWITCH
    itk_data = (struct ilitek_data *)kzalloc(sizeof(*itk_data), GFP_KERNEL);
    if (!itk_data) {
        ipio_err("allocate memory for ilitek_data fail");
        return -ENOMEM;
    }
	mutex_init(&ilitek_mutex);
#endif
//Antaiui <AI_BSP_CTP> <hehl> <2020-5-28> add gesture  start
#if REGULATOR_POWER
	ilitek_plat_regulator_power_init();
#endif

	if (ilitek_plat_gpio_register() < 0)
		ipio_err("Register gpio failed\n");

	if (ilitek_tddi_init() < 0) {
		ipio_err("platform probe failed\n");
		return -ENODEV;
	}
	ilitek_plat_irq_register(idev->irq_tirgger_type);
	tpd_load_status = 1;
	return 0;
}

static int ilitek_plat_remove(void)
{
	ipio_info();
	ilitek_tddi_dev_remove();
	return 0;
}

static const struct of_device_id tp_match_table[] = {
	{.compatible = DTS_OF_NAME},
	{},
};

static struct ilitek_hwif_info hwif = {
	.bus_type = TDDI_INTERFACE,
	.plat_type = TP_PLAT_MTK,
	.owner = THIS_MODULE,
	.name = TDDI_DEV_ID,
	.of_match_table = of_match_ptr(tp_match_table),
	.plat_probe = ilitek_plat_probe,
	.plat_remove = ilitek_plat_remove,
};

static int tpd_local_init(void)
{
	ipio_info("TPD init device driver\n");

	if (ilitek_tddi_dev_init(&hwif) < 0) {
		ipio_err("Failed to register i2c/spi bus driver\n");
		return -ENODEV;
	}
	if (tpd_load_status == 0) {
		ipio_err("Add error touch panel driver\n");
		return -1;
	}
	if (tpd_dts_data.use_tpd_button) {
		tpd_button_setting(tpd_dts_data.tpd_key_num, tpd_dts_data.tpd_key_local,
				   tpd_dts_data.tpd_key_dim_local);
	}

	tpd_type_cap = 1;
//hct-drv add for fwvno  begin
	ipio_info(" fwver = %d\n",__func__, fwver);
//	msleep(10);
//	sprintf(tpd_device_driver.descript,"Vno:%d.%d.%d.%d", fwver >> 24, (fwver >> 16) & 0x00ff, (fwver >> 8) & 0x0000ff, fwver & 0x000000ff);
//hct-drv add for fwvno  end
	ipio_info("TPD init device driver [end]\n");
	return 0;
}

static struct tpd_driver_t tpd_device_driver = {
	.tpd_device_name = TDDI_DEV_ID,
	.tpd_local_init = tpd_local_init,
	.suspend = tpd_suspend,
	.resume = tpd_resume,
//hct-drv add start
#ifdef ILITEK_ENABLE_GESTURE
	.attrs = {
		.attr = tpd_attr_list,
		.num = (int)(sizeof(tpd_attr_list)/sizeof(tpd_attr_list[0])),
	},
#endif
//hct-drv add end
};

//Antaiui <AI_BSP_CTP> <hehl> <2020-5-28> add gesture  start
#ifdef CONFIG_AI_BSP_GESTURE_SWITCH
static struct device_attribute *tp_feature_attr_list[] = {

    &dev_attr_double_wake,
    &dev_attr_gesture_wake,
  // &dev_attr_gesture_coordition,
   &dev_attr_gesture_config,
   &dev_attr_factory_check,
}; 

static struct platform_device ai_tp_wake_device = {
    .name   = "tp_wake_switch",
    .id     = -1,
};

static int tpd_create_attr(struct device *dev)
{
    int idx, err = 0;
    int num = (int)(sizeof(tp_feature_attr_list)/sizeof(tp_feature_attr_list[0]));

    if (dev == NULL)
    {
        return -EINVAL;
    }
    
    for(idx = 0; idx < num; idx++)
    {
        if(err != device_create_file(dev, tp_feature_attr_list[idx]))
        {
            ipio_err("[GESTURE]driver_create_file failed.\n");
            break;
        }
    }

    return err;
}

static int tpd_delete_attr(struct device *dev)
{
    int idx ,err = 0;
    int num = (int)(sizeof(tp_feature_attr_list)/sizeof(tp_feature_attr_list[0]));

    if (dev == NULL)
    {
        return -EINVAL;
    }

    for(idx = 0; idx < num; idx++)
    {
        device_remove_file(dev,tp_feature_attr_list[idx]);
    }
    return err;
}

int ilitek_create_gesture_sysfs(void)
{
    int ret = 0;

    ret = platform_device_register(&ai_tp_wake_device);
    if (ret)
    {
        ipio_err("[GESTURE]create ai_tp_wake_device failed\n");
    }
    ret = tpd_create_attr(&(ai_tp_wake_device.dev));
    if(ret)
    {
        ipio_err("[GESTURE]create ai_tp_feature attr failed \n");
    }

    return 0;
}
#endif
//Antaiui <AI_BSP_CTP> <hehl> <2020-5-28> add gesture  end

static int __init ilitek_plat_dev_init(void)
{
	int ret = 0;

	ipio_info("ILITEK TP driver init for MTK\n");
	tpd_get_dts_info();
	ret = tpd_driver_add(&tpd_device_driver);
	if (ret < 0) {
		ipio_err("ILITEK add TP driver failed\n");
		tpd_driver_remove(&tpd_device_driver);
		return -ENODEV;
	}

//Antaiui <AI_BSP_CTP> <hehl> <2020-5-28> add gesture  start
#ifdef CONFIG_AI_BSP_GESTURE_SWITCH
    ret = ilitek_create_gesture_sysfs();
    if (0 != ret) {
        ipio_err("[Mode]create ai_tp_wake_device failed.");
        tpd_delete_attr(&(ai_tp_wake_device.dev));
        platform_device_unregister(&ai_tp_wake_device);
        return -EIO;
    } else {
        ipio_info("[Mode]create ai_tp_wake_device succeeded");
    }
#endif	
//Antaiui <AI_BSP_CTP> <hehl> <2020-5-28> add gesture  end

	return 0;
}

static void __exit ilitek_plat_dev_exit(void)
{
	ipio_info("ilitek driver has been removed\n");
	tpd_driver_remove(&tpd_device_driver);
}

module_init(ilitek_plat_dev_init);
module_exit(ilitek_plat_dev_exit);
MODULE_AUTHOR("ILITEK");
MODULE_LICENSE("GPL");
