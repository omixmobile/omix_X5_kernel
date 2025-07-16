/* Copyright Statement:
 * antaiui liuxinhua 2017-09-12 add begin
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <asm/atomic.h>
#include <asm/uaccess.h>

#include <linux/ai_device_check.h>

// Antaiui <AI_BSP_MMI> <xieht> <2021-03-26> mmi camera about begin
#define AI_DEVICE_MAIN_CAM
#define AI_DEVICE_SUB_CAM

#if CONFIG_ANT_CAM_MAX_NUMBER_OF_CAMERA >= 3
#define AI_DEVICE_MAIN2_CAM
#endif

#if CONFIG_ANT_CAM_MAX_NUMBER_OF_CAMERA >= 4
#define AI_DEVICE_SUB2_CAM
#endif

#if CONFIG_ANT_CAM_MAX_NUMBER_OF_CAMERA >= 5
#define AI_DEVICE_MAIN3_CAM
#endif

#if CONFIG_ANT_CAM_MAX_NUMBER_OF_CAMERA >= 6
#define AI_DEVICE_SUB3_CAM
#endif
// Antaiui <AI_BSP_MMI> <xieht> <2021-03-26> mmi camera about end

#define AI_DEVICE_NAME	"ai_device_check"

struct ai_device_info dev_info[AI_DEVICE_TYPE_TOTAL];

extern u32 get_devinfo_with_index(u32 index);//chiva

int ai_set_device_info(struct ai_device_info ai_dev_info)
{
	strcpy(dev_info[ai_dev_info.ai_dev_type].name, ai_dev_info.name);
	strcpy(dev_info[ai_dev_info.ai_dev_type].vendor, ai_dev_info.vendor);
	strcpy(dev_info[ai_dev_info.ai_dev_type].version, ai_dev_info.version);

	return 0;
}
/*
static void dump_info()
{
	int i;
	for(i=0;i<AI_DEVICE_TYPE_TOTAL;i++)
	{
		AI_DEVICE_DEBUG("type=%d,name=%s,vendor=%s,version=%s\n",i,dev_info[i].name,dev_info[i].vendor,dev_info[i].version);
	}
}
*/
//Antaiui <AI_BSP_LCD> <shenjie> <20181012> add for ASW1102A-434 change \n for ",",start
static ssize_t ai_device_name(struct device *dev,struct device_attribute *attr, char *buf_name)
{
	int len=0;
	#ifdef AI_DEVICE_LCD
	len+=snprintf(buf_name+len, AI_DEVICE_NAME_LEN,"LCD: %s,", dev_info[AI_DEVICE_TYPE_LCD].name);
	#endif
	#ifdef AI_DEVICE_TOUCHPANEL
	len+=snprintf(buf_name+len, AI_DEVICE_NAME_LEN,"TP: %s,", dev_info[AI_DEVICE_TYPE_TP].name);
    #endif
    #ifdef AI_DEVICE_FINGER
    len+=snprintf(buf_name+len, AI_DEVICE_NAME_LEN,"Finger:%s,", dev_info[AI_DEVICE_TYPE_FINGER].name);
    #endif
	#ifdef AI_DEVICE_ACCELEROMETER
	len+=snprintf(buf_name+len,AI_DEVICE_NAME_LEN, "G-sensor: %s,", dev_info[AI_DEVICE_TYPE_ACCELEROMETER].name);
	#endif	
	#ifdef AI_DEVICE_MAGNETIC_FIELD
	len+=snprintf(buf_name+len,AI_DEVICE_NAME_LEN, "M-sensor: %s,", dev_info[AI_DEVICE_TYPE_MAGNETIC_FIELD].name);
	#endif
	#ifdef AI_DEVICE_GYROSCOPE
	len+=snprintf(buf_name+len, AI_DEVICE_NAME_LEN,"GYRO: %s,", dev_info[AI_DEVICE_TYPE_GYROSCOPE].name);
	#endif
	#ifdef AI_DEVICE_LIGHT
	len+=snprintf(buf_name+len,AI_DEVICE_NAME_LEN, "L-sensor:%s,", dev_info[AI_DEVICE_TYPE_LIGHT].name);
	#endif
	#ifdef AI_DEVICE_PROXIMITY
	len+=snprintf(buf_name+len, AI_DEVICE_NAME_LEN,"P-sensor:%s,", dev_info[AI_DEVICE_TYPE_PROXIMITY].name);
	#endif
	#ifdef AI_DEVICE_ALSPS
	len+=snprintf(buf_name+len, AI_DEVICE_NAME_LEN,"alsps-sensor:%s,", dev_info[AI_DEVICE_TYPE_ALSPS].name);
	#endif
	#ifdef AI_DEVICE_CAP_KEY
	len+=snprintf(buf_name+len, AI_DEVICE_NAME_LEN,"%s,", dev_info[AI_DEVICE_TYPE_CAP_KEY].name);
	#endif
	#ifdef AI_DEVICE_MAIN_CAM
	len+=snprintf(buf_name+len,AI_DEVICE_NAME_LEN, "Main_CAM: %s,",dev_info[AI_DEVICE_TYPE_MAIN_CAM].name);
	#endif
	#ifdef AI_DEVICE_SUB_CAM 
	len+=snprintf(buf_name+len,AI_DEVICE_NAME_LEN, "SUB_CAM: %s,",dev_info[AI_DEVICE_TYPE_SUB_CAM].name);
	#endif
	#ifdef AI_DEVICE_MAIN2_CAM
	len+=snprintf(buf_name+len,AI_DEVICE_NAME_LEN, "Main2_CAM: %s,",dev_info[AI_DEVICE_TYPE_MAIN2_CAM].name);
	#endif
	//Antaiui <AI_BSP_CAMERA> <xionggh> <2018-02-23> add for ASW1102A-8 begin
	#ifdef AI_DEVICE_SUB2_CAM 
	len+=snprintf(buf_name+len,AI_DEVICE_NAME_LEN, "SUB2_CAM: %s,",dev_info[AI_DEVICE_TYPE_SUB2_CAM].name);
	#endif
	//Antaiui <AI_BSP_CAMERA> <xionggh> <2018-02-23> add for ASW1102A-8 end
/* <ANTITEK> <ANT_BSP_CAM> <xuel> <2019-05-17> added begin */
#ifdef AI_DEVICE_MAIN3_CAM
	len+=snprintf(buf_name+len,AI_DEVICE_NAME_LEN, "Main3_CAM: %s,",dev_info[AI_DEVICE_TYPE_MAIN3_CAM].name);
#endif
/* <ANTITEK> <ANT_BSP_CAM> <xuel> <2019-05-17> added end */
/* Antaiui <ANT_BSP_CAM> <xieht> <2020-07-24> add sub3 begin */
#ifdef AI_DEVICE_SUB3_CAM
	len+=snprintf(buf_name+len,AI_DEVICE_NAME_LEN, "Sub3_CAM: %s,",dev_info[AI_DEVICE_TYPE_SUB3_CAM].name);
#endif
/* Antaiui <ANT_BSP_CAM> <xieht> <2020-07-24> add sub3 begin */
	#ifdef AI_DEVICE_RF
	len+=snprintf(buf_name+len, AI_DEVICE_NAME_LEN,"RF: %s,", dev_info[AI_DEVICE_TYPE_RF].name);
	#endif
//Gionee <GN_BSP_DEVICECHECK> <lihl> <20170511> add for 49184 start
        #ifdef AI_DEVICE_MEMORY
        len+=snprintf(buf_name+len,AI_DEVICE_NAME_LEN, "MEMORY:%s,",dev_info[AI_DEVICE_TYPE_MEMORY].name);
        #endif
//Gionee <GN_BSP_DEVICECHECK> <lihl> <20170511> add for 49184 end  

//Antai <AI_BSP_MEMORY> <luobowen> <2021-06-28> add for ddr start
    #ifdef AI_DEVICE_DDR
        len+=snprintf(buf_name+len,AI_DEVICE_NAME_LEN, "DDR(ram):%s,",dev_info[AI_DEVICE_TYPE_DDR].name);
    #endif
//Antai <AI_BSP_MEMORY> <luobowen> <2021-06-28> add for ddr end 

	#ifdef AI_DEVICE_BATTERY
	len+=snprintf(buf_name+len,AI_DEVICE_NAME_LEN, "BATTERY: %s,",dev_info[AI_DEVICE_TYPE_BATTERY].name);
	#endif
	//Antaiui <AI_BSP_CHG> <zhouli> <20180321> add for ASW1102A-813 start
	#ifdef AI_DEVICE_CHARGER
	len+=snprintf(buf_name+len,AI_DEVICE_NAME_LEN, "CHG:%s,",dev_info[AI_DEVICE_TYPE_CHARGER].name);
	#endif
	//Antaiui <AI_BSP_CHG> <zhouli> <20180321> add for ASW1102A-813 end
	
	#ifdef AI_DEVICE_NFC
	len+=snprintf(buf_name+len,AI_DEVICE_NAME_LEN, "NFC:%s,",dev_info[AI_DEVICE_TYPE_NFC].name);
	#endif
	return len;
}
//Antaiui <AI_BSP_LCD> <shenjie> <20181012> add for ASW1102A-434 change \n for ",",end

static ssize_t ai_device_vendor(struct device *dev,struct device_attribute *attr, char *buf_vendor)
{
	int len=0;
	#ifdef AI_DEVICE_LCD
	len+=snprintf(buf_vendor+len, AI_DEVICE_VENDOR_LEN,"%s\n", dev_info[AI_DEVICE_TYPE_LCD].vendor);
	#endif
	#ifdef AI_DEVICE_TOUCHPANEL
	len+=snprintf(buf_vendor+len, AI_DEVICE_VENDOR_LEN,"%s\n", dev_info[AI_DEVICE_TYPE_TP].vendor);
	#endif
	#ifdef AI_DEVICE_FINGER
    len+=snprintf(buf_vendor+len, AI_DEVICE_VENDOR_LEN,"%s\n", dev_info[AI_DEVICE_TYPE_FINGER].vendor);
    #endif
	#ifdef AI_DEVICE_ACCELEROMETER
	len+=snprintf(buf_vendor+len,AI_DEVICE_VENDOR_LEN, "%s\n", dev_info[AI_DEVICE_TYPE_ACCELEROMETER].vendor);
	#endif

	#ifdef AI_DEVICE_MAGNETIC_FIELD
	len+=snprintf(buf_vendor+len, AI_DEVICE_VENDOR_LEN,"%s\n", dev_info[AI_DEVICE_TYPE_MAGNETIC_FIELD].vendor);
	#endif
	#ifdef AI_DEVICE_GYROSCOPE
	len+=snprintf(buf_vendor+len, AI_DEVICE_VENDOR_LEN,"%s\n", dev_info[AI_DEVICE_TYPE_GYROSCOPE].vendor);
	#endif
	#ifdef AI_DEVICE_LIGHT
	len+=snprintf(buf_vendor+len,AI_DEVICE_VENDOR_LEN, "%s\n", dev_info[AI_DEVICE_TYPE_LIGHT].vendor);
	#endif
	#ifdef AI_DEVICE_PROXIMITY
	len+=snprintf(buf_vendor+len, AI_DEVICE_VENDOR_LEN,"%s\n", dev_info[AI_DEVICE_TYPE_PROXIMITY].vendor);
	#endif
	#ifdef AI_DEVICE_ALSPS
	len+=snprintf(buf_vendor+len, AI_DEVICE_VENDOR_LEN,"%s,", dev_info[AI_DEVICE_TYPE_ALSPS].vendor);
	#endif
	#ifdef AI_DEVICE_CAP_KEY
	len+=snprintf(buf_vendor+len, AI_DEVICE_VENDOR_LEN,"%s\n", dev_info[AI_DEVICE_TYPE_CAP_KEY].vendor);
	#endif
	#ifdef AI_DEVICE_MAIN_CAM 
	len+=snprintf(buf_vendor+len, AI_DEVICE_VENDOR_LEN,"%s\n", dev_info[AI_DEVICE_TYPE_MAIN_CAM].vendor);
	#endif
	#ifdef AI_DEVICE_SUB_CAM 
	len+=snprintf(buf_vendor+len, AI_DEVICE_VENDOR_LEN,"%s\n", dev_info[AI_DEVICE_TYPE_SUB_CAM].vendor);
	#endif
	#ifdef AI_DEVICE_MAIN2_CAM
	len+=snprintf(buf_vendor+len,AI_DEVICE_VENDOR_LEN, "%s\n",dev_info[AI_DEVICE_TYPE_MAIN2_CAM].vendor);
	#endif
/* <ANTITEK> <ANT_BSP_CAM> <xuel> <2019-05-17> added begin */
#ifdef AI_DEVICE_MAIN3_CAM
	len+=snprintf(buf_vendor+len,AI_DEVICE_VENDOR_LEN, "%s\n",dev_info[AI_DEVICE_TYPE_MAIN3_CAM].vendor);
#endif
/* <ANTITEK> <ANT_BSP_CAM> <xuel> <2019-05-17> added end */
/* Antaiui <ANT_BSP_CAM> <xieht> <2020-07-24> add sub3 begin */
#ifdef AI_DEVICE_SUB2_CAM
	len+=snprintf(buf_vendor+len,AI_DEVICE_VENDOR_LEN, "%s\n",dev_info[AI_DEVICE_TYPE_SUB2_CAM].vendor);
#endif
#ifdef AI_DEVICE_SUB3_CAM
	len+=snprintf(buf_vendor+len,AI_DEVICE_VENDOR_LEN, "%s\n",dev_info[AI_DEVICE_TYPE_SUB3_CAM].vendor);
#endif
/* Antaiui <ANT_BSP_CAM> <xieht> <2020-07-24> add sub3  end  */

	#ifdef AI_DEVICE_RF
	len+=snprintf(buf_vendor+len, AI_DEVICE_VENDOR_LEN,"%s\n", dev_info[AI_DEVICE_TYPE_RF].vendor);
	#endif
	#ifdef AI_DEVICE_BATTERY 
	len+=snprintf(buf_vendor+len, AI_DEVICE_VENDOR_LEN,"%s\n", dev_info[AI_DEVICE_TYPE_BATTERY].vendor);
	#endif
	//Antaiui <AI_BSP_CHG> <zhouli> <20180321> add for ASW1102A-813 start
	#ifdef AI_DEVICE_CHARGER
	len+=snprintf(buf_vendor+len,AI_DEVICE_VENDOR_LEN, "CHG:%s\n",dev_info[AI_DEVICE_TYPE_CHARGER].vendor);
	#endif
	//Antaiui <AI_BSP_CHG> <zhouli> <20180321> add for ASW1102A-813 end
	
	#ifdef AI_DEVICE_NFC
	len+=snprintf(buf_vendor+len,AI_DEVICE_VENDOR_LEN, "NFC:%s\n",dev_info[AI_DEVICE_TYPE_NFC].vendor);
	#endif
	return len;	
}

static ssize_t ai_device_version(struct device *dev,struct device_attribute *attr, char *buf_version)
{	
	int len=0;
	#ifdef AI_DEVICE_LCD
	len+=snprintf(buf_version+len, AI_DEVICE_VERSION_LEN,"%s\n", dev_info[AI_DEVICE_TYPE_LCD].version);
	#endif
	#ifdef AI_DEVICE_TOUCHPANEL
	len+=snprintf(buf_version+len,AI_DEVICE_VERSION_LEN, "%s\n", dev_info[AI_DEVICE_TYPE_TP].version);
	#endif
	#ifdef AI_DEVICE_FINGER
    len+=snprintf(buf_version+len, AI_DEVICE_VERSION_LEN,"%s\n", dev_info[AI_DEVICE_TYPE_FINGER].version);
    #endif
	#ifdef AI_DEVICE_ACCELEROMETER
	len+=snprintf(buf_version+len,AI_DEVICE_VERSION_LEN, "%s\n", dev_info[AI_DEVICE_TYPE_ACCELEROMETER].version);
	#endif
	#ifdef AI_DEVICE_MAGNETIC_FIELD
	len+=snprintf(buf_version+len, AI_DEVICE_VERSION_LEN,"%s\n", dev_info[AI_DEVICE_TYPE_MAGNETIC_FIELD].version);
	#endif
	#ifdef AI_DEVICE_GYROSCOPE
	len+=snprintf(buf_version+len,AI_DEVICE_VERSION_LEN, "%s\n", dev_info[AI_DEVICE_TYPE_GYROSCOPE].version);
	#endif
	#ifdef AI_DEVICE_LIGHT
	len+=snprintf(buf_version+len, AI_DEVICE_VERSION_LEN,"%s\n", dev_info[AI_DEVICE_TYPE_LIGHT].version);
	#endif
	#ifdef AI_DEVICE_PROXIMITY
	len+=snprintf(buf_version+len, AI_DEVICE_VERSION_LEN,"%s\n", dev_info[AI_DEVICE_TYPE_PROXIMITY].version);
	#endif
	#ifdef AI_DEVICE_ALSPS
	len+=snprintf(buf_version+len, AI_DEVICE_VERSION_LEN,"%s\n", dev_info[AI_DEVICE_TYPE_ALSPS].version);
	#endif
	#ifdef AI_DEVICE_CAP_KEY
	len+=snprintf(buf_version+len, AI_DEVICE_VERSION_LEN,"%s\n", dev_info[AI_DEVICE_TYPE_CAP_KEY].version);
	#endif 
	#ifdef AI_DEVICE_MAIN_CAM 
	len+=snprintf(buf_version+len, AI_DEVICE_VERSION_LEN,"%s\n", dev_info[AI_DEVICE_TYPE_MAIN_CAM].version);
	#endif
	#ifdef AI_DEVICE_SUB_CAM 
	len+=snprintf(buf_version+len, AI_DEVICE_VERSION_LEN,"%s\n", dev_info[AI_DEVICE_TYPE_SUB_CAM].version);
	#endif
	#ifdef AI_DEVICE_MAIN2_CAM 
	len+=snprintf(buf_version+len, AI_DEVICE_VERSION_LEN,"%s\n", dev_info[AI_DEVICE_TYPE_MAIN2_CAM].version);
	#endif
/* <ANTITEK> <ANT_BSP_CAM> <xuel> <2019-05-17> added begin */
#ifdef AI_DEVICE_MAIN3_CAM
	len+=snprintf(buf_version+len, AI_DEVICE_VERSION_LEN,"%s\n", dev_info[AI_DEVICE_TYPE_MAIN3_CAM].version);
#endif
/* <ANTITEK> <ANT_BSP_CAM> <xuel> <2019-05-17> added end */
/* Antaiui <ANT_BSP_CAM> <xieht> <2020-07-24> add sub3 begin */
#ifdef AI_DEVICE_SUB2_CAM
	len+=snprintf(buf_version+len, AI_DEVICE_VERSION_LEN,"%s\n", dev_info[AI_DEVICE_TYPE_SUB2_CAM].version);
#endif
#ifdef AI_DEVICE_SUB3_CAM
	len+=snprintf(buf_version+len, AI_DEVICE_VERSION_LEN,"%s\n", dev_info[AI_DEVICE_TYPE_SUB3_CAM].version);
#endif
/* Antaiui <ANT_BSP_CAM> <xieht> <2020-07-24> add sub3  end  */

	#ifdef AI_DEVICE_RF
	len+=snprintf(buf_version+len, AI_DEVICE_VERSION_LEN,"%s\n", dev_info[AI_DEVICE_TYPE_RF].version);
	#endif

	#ifdef AI_DEVICE_BATTERY 
	len+=snprintf(buf_version+len, AI_DEVICE_VERSION_LEN,"%s\n", dev_info[AI_DEVICE_TYPE_BATTERY].version);
	#endif

	//Antaiui <AI_BSP_CHG> <zhouli> <20180321> add for ASW1102A-813 start
	#ifdef AI_DEVICE_CHARGER
	len+=snprintf(buf_version+len,AI_DEVICE_VERSION_LEN, "CHG:%s\n",dev_info[AI_DEVICE_TYPE_CHARGER].version);
	#endif
	//Antaiui <AI_BSP_CHG> <zhouli> <20180321> add for ASW1102A-813 end
	
	#ifdef AI_DEVICE_NFC
	len+=snprintf(buf_version+len,AI_DEVICE_VERSION_LEN, "CHG:%s\n",dev_info[AI_DEVICE_TYPE_NFC].version);
	#endif
	return len;
}

DEVICE_ATTR(name, S_IRUGO, ai_device_name, NULL);
DEVICE_ATTR(vendor, S_IRUGO, ai_device_vendor, NULL);
DEVICE_ATTR(version, S_IRUGO, ai_device_version, NULL);

static struct device_attribute *ai_device_attr_list[] =
{
	&dev_attr_name,
	&dev_attr_vendor,
	&dev_attr_version,
};

static int ai_device_create_attr(struct device *dev) 
{
	int idx, err = 0;
	int num = (int)(sizeof(ai_device_attr_list)/sizeof(ai_device_attr_list[0]));
	AI_DEVICE_DEBUG("%s\n", __func__);
	if(!dev)
	{
		return -EINVAL;
	}	

	for(idx = 0; idx < num; idx++)
	{
		if((err = device_create_file(dev, ai_device_attr_list[idx])))
		{            
			AI_DEVICE_DEBUG("device_create_file (%s) = %d\n", ai_device_attr_list[idx]->attr.name, err);        
			break;
		}
	}

	return err;
}


static int ai_device_pdrv_probe(struct platform_device *pdev)
{
	int err=0;
	AI_DEVICE_DEBUG("%s\n", __func__);
	if(ai_device_create_attr(&pdev->dev) != 0)
	{
		AI_DEVICE_DEBUG("%s:unable to create attributes!!\n",__func__);
		goto exit_create_attr_failed;
	}

	return 0;
exit_create_attr_failed:
	AI_DEVICE_DEBUG("%s: err = %d\n", __func__, err);        
	return err;
}

/* should never be called */
static int ai_device_pdrv_remove(struct platform_device *pdev)
{
	AI_DEVICE_DEBUG("%s\n", __func__);
	return 0;
}

static struct platform_device ai_device_check_pdev = {
	.name	= "ai_device_check",
	.id	= -1,
};

#ifdef CONFIG_OF
static const struct of_device_id ai_device_check[] = {
	{ .compatible = "mediatek,ai_dev_check", },
	{},
};
#endif

static struct platform_driver ai_device_pdrv = {
	.probe		= ai_device_pdrv_probe,
	.remove		= ai_device_pdrv_remove,
	.driver		= {
		.name	= AI_DEVICE_NAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = ai_device_check,
#endif
	},
};

static int __init ai_device_mod_init(void)
{
	int ret = 0;
	AI_DEVICE_DEBUG("%s\n", __func__);

	ret = platform_device_register(&ai_device_check_pdev);
	if (ret != 0){
		AI_DEVICE_DEBUG("register device failed (%d)\n", ret);
		return ret;
	}	

	ret = platform_driver_register(&ai_device_pdrv);
	if (ret) {
		AI_DEVICE_DEBUG("register driver failed (%d)\n", ret);
		return ret;
	}

	return ret;
}

/* should never be called */
static void __exit ai_device_mod_exit(void)
{
	AI_DEVICE_DEBUG("%s\n", __func__);
}

//module_init core_initcall
module_init(ai_device_mod_init);
module_exit(ai_device_mod_exit);

MODULE_AUTHOR("liuxinhua <liuxinhua@antaiui.com>");
MODULE_DESCRIPTION("ai_device_check Driver v0.1");
MODULE_LICENSE("GPL");
// antaiui liuxinhua 2017-09-12 add end
