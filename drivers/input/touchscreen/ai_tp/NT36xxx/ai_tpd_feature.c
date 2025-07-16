#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/ioctl.h>
#include <linux/compat.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include "tpd.h"
//#include "gn_tpd_feature.h"
#include <linux/gpio.h>
#include "nt36xxx.h"
//Antai <AI_BSP_TP> <hehl> <2021-04-12> add gesture start
#include "ai_tpd_feature.h"
#ifdef WAKEUP_GESTURE
unsigned int nvt_gesture_cfg;
int nvt_wake_switch = 0;
int nvt_gesture_switch = 0;
struct tpd_ges_data *nvt_tpd_ges_devp = NULL;
#endif
//Antai <AI_BSP_TP> <hehl> <2021-04-12> add gesture end

//Antai <AI_BSP_TP> <hehl> <2021-02-20> add for TP mmi start

#ifdef CONFIG_AI_BSP_MTK_DEVICE_CHECK
extern int ai_set_device_info(struct ai_device_info ai_dev_info);

 int nvt_register_hardware_info(void)
{
	u8 fw_ver = 0;
	u8 vendor_id = 0;
	int ret = 0;
	u8 buf[3] = {0};

	struct ai_device_info ai_tp_hw_info;

	buf[0] = EVENT_MAP_FWINFO;
	buf[1] = 0x00;
	buf[2] = 0x00;
	ret = CTP_I2C_READ(ts->client, I2C_BLDR_Address, buf, 3);
	if (ret < 0) {
		NVT_ERR("i2c read error!(%d)\n", ret);
		return ret;
	}
	fw_ver = buf[1];

    //read vendor id
    buf[0] = EVENT_MAP_VENDORID;
    buf[1] = 0x00;
    buf[2] = 0x00;
    ret = CTP_I2C_READ(ts->client, I2C_BLDR_Address, buf, 3);
    if (ret < 0) {
        NVT_ERR("i2c read vendor id error!(%d)\n", ret);
        return ret;
    }
    vendor_id = buf[1];

	ai_tp_hw_info.ai_dev_type = AI_DEVICE_TYPE_TP;
//Antai <AI_BSP_TP> <hehl> <2021-02-22> add for TP mmi start
	snprintf(ai_tp_hw_info.name, AI_DEVICE_NAME_LEN, "truly-nt36525b_fw[0x%x]",fw_ver);
//Antai <AI_BSP_TP> <hehl> <2021-02-22> add for TP mmi end
	snprintf(ai_tp_hw_info.vendor, AI_DEVICE_VENDOR_LEN, "aiui");
	snprintf(ai_tp_hw_info.version, AI_DEVICE_VERSION_LEN, "00");

	ai_set_device_info(ai_tp_hw_info);

	return ret;
}
#endif
//Antai <AI_BSP_TP> <hehl> <2021-02-20> add for TP mmi end
extern int nvt_test_result;
static ssize_t nt36xxx_factory_check(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	
	struct file *pfile = NULL;
		if (NULL == pfile)
		pfile = filp_open("/proc/nvt_selftest", O_RDONLY, 0);
	if (IS_ERR(pfile)) {
		pr_err("error occured while opening file /proc/nvt_selftest.\n");
		return -EIO;
	}
	filp_close(pfile, NULL);
	return sprintf(buf, "%d\n", nvt_test_result);
}

static DEVICE_ATTR(factory_check, 0444, nt36xxx_factory_check, NULL);

//Antai <AI_BSP_TP> <hehl> <2021-03-16> add landscape mode start
int ai_nvt_screen_switch = 0;
#define NVT_SCREEN_VERTICAL    0xBA
#define NVT_SCREEN_HORIZONTAL  0xBB
void nvt_horizontal_vertical_setting(uint8_t is_horizontal)
{
    uint8_t buf[10] = {0};
    uint32_t retry = 0;
    uint32_t ret = 0;

    mutex_lock(&ts->lock);

    if (is_horizontal == 1) {
        NVT_LOG("Enter horizontal mode\n");
        buf[0] = EVENT_MAP_HOST_CMD;
        buf[1] = NVT_SCREEN_HORIZONTAL;
    } else if (is_horizontal == 0) {
        NVT_LOG("Enter vertical mode, is_horizontal = %d\n", is_horizontal);
        buf[0] = EVENT_MAP_HOST_CMD;
        buf[1] = NVT_SCREEN_VERTICAL;
    } else {
        NVT_ERR("set horizontal_vertical (%d) failed, invalid parameter\n", is_horizontal);
        return;
    }

    ret = CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);
    if (ret < 0)
        NVT_ERR("set horizontal_vertical (%d) failed:%d\n", is_horizontal, ret);

    /* read back fw EVENT_MAP_HOST_CMD */
    while (1) {
        msleep(18); //wait for 1 frame (~16.66ms)

        buf[0] = EVENT_MAP_HOST_CMD;
        buf[1] = 0xFF;
        CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 2);

        if (buf[1] == 0x00) {
            ret = 0;
            break;
        }

        retry++;
        if(unlikely(retry > 5)) {
            NVT_ERR("error, retry=%d, buf[1]=0x%02X\n", retry, buf[1]);
            ret = -1;
            break;
        }
    }

    mutex_unlock(&ts->lock);

    return;
}

static ssize_t nvt_landscape_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", ai_nvt_screen_switch);
}

static ssize_t nvt_landscape_mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int ret = -1;
    long val = -1;

    ret = kstrtoul(buf, 10, &val); 
    if(ret != 0) {
        printk(KERN_ERR "%s, invalid value \n",__func__);
    }

    ai_nvt_screen_switch = (val > 0)?0x01:0x00;

    if(ai_nvt_screen_switch == 0x01)
        nvt_horizontal_vertical_setting(1);
    else
        nvt_horizontal_vertical_setting(0);

    return count;
}

static DEVICE_ATTR(landscape_mode, 0664, nvt_landscape_mode_show, nvt_landscape_mode_store);

//Antai <AI_BSP_TP> <hehl> <2021-04-12> add gesture start
#ifdef	WAKEUP_GESTURE
static ssize_t tp_double_wake_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%d\n", nvt_wake_switch);
}
static ssize_t tp_double_wake_write(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	int rt;
	unsigned long val;
	//rt = strict_strtoul(buf, 10, &val); 
	rt = kstrtoul(buf, 10, &val); 
	if(rt != 0){
		pr_err("%s, invalid value\n", __func__);
		return rt;
	}
	nvt_wake_switch = val;
	NVT_LOG("%s, %d\n", __func__, nvt_wake_switch);
	return size;
}
static DEVICE_ATTR(double_wake, 0664, tp_double_wake_show, tp_double_wake_write);

static ssize_t tp_gesture_switch_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%d\n", nvt_gesture_switch);
}
static ssize_t tp_gesture_switch_write(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	int rt;
	unsigned long val;
	//rt = strict_strtoul(buf, 10, &val); 
	rt = kstrtoul(buf, 10, &val); 
	if(rt != 0){
		pr_err("%s, invalid value\n", __func__);
		return rt;
	}
	nvt_gesture_switch = val;
	NVT_LOG("%s, %d\n", __func__, nvt_gesture_switch);
	return size;
}
static DEVICE_ATTR(gesture_wake, 0664, tp_gesture_switch_show, tp_gesture_switch_write);

static ssize_t tp_gesture_coordition_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int i,count;
	int len = 0;

	if(nvt_tpd_ges_devp == NULL)
		return 0;
	count = sizeof(nvt_tpd_ges_devp->f_point)/sizeof(nvt_tpd_ges_devp->f_point.data[0]);
	NVT_LOG(" %s count=%d \n",__func__,count);
	for(i=0;i<count;i++) {
		if(i==count-1)
			len += sprintf(buf+len,"%d",nvt_tpd_ges_devp->f_point.data[i]);
		else
			len += sprintf(buf+len,"%d,",nvt_tpd_ges_devp->f_point.data[i]);
	}
	
	return len;

}
static DEVICE_ATTR(gesture_coordition, 0444, tp_gesture_coordition_show, NULL);

static ssize_t tp_gesture_config_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf,"%u\n",nvt_gesture_cfg);
}
static ssize_t tp_gesture_config_store(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	int rt;
	unsigned long val;
	//rt = strict_strtoul(buf, 10,&val);
	rt = kstrtoul(buf, 10, &val); 
	if(rt != 0) {
		printk(KERN_ERR "%s, invalid value \n",__func__);
	}
	nvt_gesture_cfg = val & 0xffff;

	NVT_LOG(" %s gesture_cfg val=0x%04lX \n",__func__,val);

	return count;
}
static DEVICE_ATTR(gesture_config, 0664, tp_gesture_config_show, tp_gesture_config_store);
#endif
//Antai <AI_BSP_TP> <hehl> <2021-04-12> add gesture end

//Antai <AI_BSP_TP> <hehl> <2021-03-16> add landscape mode end
static struct device_attribute *tp_feature_attr_list[] = {
    &dev_attr_factory_check,
//Antai <AI_BSP_TP> <hehl> <2021-03-16> add landscape mode start	
	&dev_attr_landscape_mode,
//Antai <AI_BSP_TP> <hehl> <2021-04-12> add gesture start	
#ifdef	WAKEUP_GESTURE
	&dev_attr_double_wake,
	&dev_attr_gesture_wake,
	&dev_attr_gesture_coordition,
	&dev_attr_gesture_config,
#endif
//Antai <AI_BSP_TP> <hehl> <2021-04-12> add gesture end
//Antai <AI_BSP_TP> <hehl> <2021-03-16> add landscape mode end
};

static int tpd_create_attr(struct device *dev)
{
    int idx, err = 0;
    int num = (int)(sizeof(tp_feature_attr_list)/sizeof(tp_feature_attr_list[0]));
    if (dev == NULL)
    {
        return -EINVAL;
    }
    TPD_DMESG("tpd_create_attr ----0 \n");
    for(idx = 0; idx < num; idx++)
    {
        err = device_create_file(dev, tp_feature_attr_list[idx]);
        if(err)
        {
            TPD_DMESG("TPD  driver_create_file failed");
            break;
        }
    }
    TPD_DMESG("TPD  driver_create_file success\n");
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
static struct platform_device ai_tp_wake_device = {
       .name   = "tp_wake_switch",
       .id     = -1,
};

int ai_nvt_tpd_feature_init_data(struct i2c_client *client)
{
	int ret;
//Antai <AI_BSP_TP> <hehl> <2021-04-12> add gesture start	
#ifdef	WAKEUP_GESTURE
	nvt_tpd_ges_devp = kzalloc(sizeof(struct tpd_ges_data),GFP_KERNEL);
	if(!nvt_tpd_ges_devp) {
		NVT_LOG(KERN_INFO "tpd_gesture: char kzalloc tpd_ges_data fail\n");
	}
#endif	
//Antai <AI_BSP_TP> <hehl> <2021-04-12> add gesture end
	ret = platform_device_register(&ai_tp_wake_device);
	if (ret)
	{
		printk("tpd: create ai_tp_wake_device failed\n");
	}
	ret = tpd_create_attr(&(ai_tp_wake_device.dev));
	if(ret)
	{
		printk("tpd: create ai_tp_feature attr failed \n");
	}
//Antai <AI_BSP_TP> <hehl> <2021-02-20> add for TP mmi start	
#ifdef CONFIG_AI_BSP_MTK_DEVICE_CHECK
	ret = nvt_register_hardware_info();
	if (ret < 0) {
		return -1;
	}
#endif
//Antai <AI_BSP_TP> <hehl> <2021-02-20> add for TP mmi start
	return ret;

}

int ai_nvt_tpd_feature_reinit(void)
{
	NVT_LOG(KERN_INFO "tpd_gesture: char dev clean up \n");
	tpd_delete_attr(&(ai_tp_wake_device.dev));
	platform_device_unregister(&ai_tp_wake_device);

    return 0;
}
