/************************************************************************
*
* File Name: ai_tpd_feature.c
*
* Author: antai_driver_team
*
* Created: 2022-12-01
*
* Modify:
*
* Abstract: create char device and proc node for  the comm between APK and TP
*
************************************************************************/

/*****************************************************************************
* Included header files
*****************************************************************************/
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
//#include "tpd.h"
#include <linux/gpio.h>


//#include "focaltech_core.h"
//#include "focaltech_config.h"
//#include "focaltech_test/focaltech_test.h"

#include "omnivision_tcm_core.h"
#include "ai_tpd_feature.h"

struct ovt_tcm_hcd *tcm_hcd;


/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/
//Antai <AI_BSP_TP> <chenht> <2022-12-21> modify for 2206 gesture begin
#if WAKEUP_GESTURE
extern ssize_t ovt_double_wake_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t ovt_double_wake_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t ovt_gesture_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t ovt_gesture_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
//extern ssize_t fts_gesture_config_show(struct device *dev,struct device_attribute *attr, char *buf);
//extern ssize_t fts_gesture_config_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t ovt_gesture_buf_show(struct device *dev,struct device_attribute *attr, char *buf);
extern ssize_t ovt_gesture_buf_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count);
static DEVICE_ATTR (double_wake, S_IRUGO|S_IWUSR, ovt_double_wake_show, ovt_double_wake_store);
static DEVICE_ATTR (gesture_wake, S_IRUGO|S_IWUSR, ovt_gesture_show, ovt_gesture_store);
//static DEVICE_ATTR (double_wake, S_IRUGO|S_IWUSR, fts_double_wake_show, fts_double_wake_store);
//static DEVICE_ATTR (gesture_wake, S_IRUGO|S_IWUSR, fts_gesture_show, fts_gesture_store);
static DEVICE_ATTR (gesture_coordition, S_IRUGO|S_IWUSR, ovt_gesture_buf_show, ovt_gesture_buf_store);
//static DEVICE_ATTR(gesture_config, S_IRUGO|S_IWUSR, fts_gesture_config_show, fts_gesture_config_store);
#endif	
//Antai <AI_BSP_TP> <chenht> <2022-12-21> modify for 2206 gesture end
/*****************************************************************************
* Static function prototypes
*****************************************************************************/
//static DEVICE_ATTR (gesture_coordition, S_IRUGO|S_IWUSR, fts_gesture_buf_show, fts_gesture_buf_store);
//static DEVICE_ATTR(gesture_config, S_IRUGO|S_IWUSR, fts_gesture_config_show, fts_gesture_config_store);



extern ssize_t ovt_test_show(struct device *dev,struct device_attribute *attr,char *buf);
static DEVICE_ATTR(factory_check, 0444, ovt_test_show, NULL);

static struct device_attribute *tp_feature_attr_list[] = {
    &dev_attr_factory_check,
//Antai <AI_BSP_TP> <chenht> <2022-12-21> modify for 2206 gesture begin
#if WAKEUP_GESTURE
    &dev_attr_double_wake,
    &dev_attr_gesture_wake,
    &dev_attr_gesture_coordition,
    //&dev_attr_gesture_config,
#endif
//Antai <AI_BSP_TP> <chenht> <2022-12-21> modify for 2206 gesture end
};

static int tpd_create_attr(struct device *dev)
{
    int idx, err = 0;
    int num = (int)(sizeof(tp_feature_attr_list)/sizeof(tp_feature_attr_list[0]));
    if (dev == NULL)
    {
        return -EINVAL;
    }
    LOGD(tcm_hcd->pdev->dev.parent,"tpd_create_attr ----0 \n");
    pr_info("tpd_create_attr ----0\n");
    for(idx = 0; idx < num; idx++)
    {
        err = device_create_file(dev, tp_feature_attr_list[idx]);
        if(err)
        {
            LOGD(tcm_hcd->pdev->dev.parent,"TPD  driver_create_file failed");
            break;
        }
    }
    LOGD(tcm_hcd->pdev->dev.parent,"TPD  driver_create_file success\n");
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

int ai_ovt_tpd_feature_init_data(void)
{
	int ret = 0;

	ret = platform_device_register(&ai_tp_wake_device);
	if (ret)
	{
		LOGD(tcm_hcd->pdev->dev.parent,"tpd: create ai_tp_wake_device failed\n");
		return -1;
	}
	ret = tpd_create_attr(&(ai_tp_wake_device.dev));
	if(ret)
	{
		LOGD(tcm_hcd->pdev->dev.parent,"tpd: create ai_tp_feature attr failed \n");
		return -1;
	}
	return ret;

}

int ai_ovt_tpd_feature_reinit(void)
{
	LOGD(tcm_hcd->pdev->dev.parent,KERN_INFO "tpd_gesture: char dev clean up \n");
	tpd_delete_attr(&(ai_tp_wake_device.dev));
	platform_device_unregister(&ai_tp_wake_device);

    return 0;
}
//Antai <AI_BSP_TP> <chenht> <2022-11-30> modify for 2206 gesture end
