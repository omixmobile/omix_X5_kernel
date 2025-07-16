#define LOG_TAG         "Ant-Test"

#include "cts_platform.h"
#include "cts_test.h"
//Antai <AI_BSP_TP> <chenht> <2023-05-09> add edge restain for 2206 begin
#include "cts_tcs.h"
//Antai <AI_BSP_TP> <chenht> <2023-05-09> add edge restain for 2206 end

struct device *MMI_test_dev;

//Antai <AI_BSP_TP> <chenht> <2023-05-09> add edge restain for 2206 begin
u8 direction = 0;
//Antai <AI_BSP_TP> <chenht> <2023-05-09> add edge restain for 2206 end

enum test_type {
    TEST_PIN_INT = 0,
    TEST_PIN_RESET,
    TEST_RAWDATA,
    TEST_NOISE,
    TEST_OPEN,
    TEST_SHORT,
    TEST_COMPENSATE_CAP,
};

static struct cts_test_param init_environment(int test)
{
    int min, max;
    struct cts_rawdata_test_priv_param priv_param = {
        .frames = 16,
    };
    struct cts_test_param test_param = {
        .flags = 0,
        .min = &min,
        .max = &max,
    };

    switch (test) {
    case TEST_RAWDATA:
        // min = 100;
        // max = 200;
        // priv_param.frames = 16;
        test_param.flags = CTS_TEST_FLAG_VALIDATE_DATA |
                    CTS_TEST_FLAG_VALIDATE_MIN |
                    CTS_TEST_FLAG_VALIDATE_MAX |
                    CTS_TEST_FLAG_STOP_TEST_IF_VALIDATE_FAILED |
                    CTS_TEST_FLAG_DUMP_TEST_DATA_TO_CONSOLE;
        test_param.num_invalid_node = 0;
        test_param.invalid_nodes = NULL;
        test_param.priv_param = &priv_param;
        test_param.priv_param_size = sizeof(priv_param);
        break;
    case TEST_NOISE:
        // max = 100;
        // priv_param.frames = 16;
        test_param.flags = CTS_TEST_FLAG_VALIDATE_DATA |
                    CTS_TEST_FLAG_VALIDATE_MAX |
                    CTS_TEST_FLAG_STOP_TEST_IF_VALIDATE_FAILED |
                    CTS_TEST_FLAG_DUMP_TEST_DATA_TO_CONSOLE;
        test_param.num_invalid_node = 0;
        test_param.invalid_nodes = NULL;
        test_param.priv_param = &priv_param;
        test_param.priv_param_size = sizeof(priv_param);
        break;
    case TEST_OPEN:
        min = 200;
        test_param.flags = CTS_TEST_FLAG_VALIDATE_DATA |
                    CTS_TEST_FLAG_VALIDATE_MIN |
                    CTS_TEST_FLAG_STOP_TEST_IF_VALIDATE_FAILED |
                    CTS_TEST_FLAG_DUMP_TEST_DATA_TO_CONSOLE;
        test_param.num_invalid_node = 0;
        test_param.invalid_nodes = NULL;
        break;
    case TEST_SHORT:
        min = 500;
        test_param.flags = CTS_TEST_FLAG_VALIDATE_DATA |
                    CTS_TEST_FLAG_VALIDATE_MIN |
                    CTS_TEST_FLAG_STOP_TEST_IF_VALIDATE_FAILED |
                    CTS_TEST_FLAG_DUMP_TEST_DATA_TO_CONSOLE;
        test_param.num_invalid_node = 0;
        test_param.invalid_nodes = NULL;
        break;
    case TEST_COMPENSATE_CAP:
        // min = 100;
        // max = 200;
        test_param.flags = CTS_TEST_FLAG_VALIDATE_DATA |
                    CTS_TEST_FLAG_VALIDATE_MIN |
                    CTS_TEST_FLAG_VALIDATE_MAX |
                    CTS_TEST_FLAG_STOP_TEST_IF_VALIDATE_FAILED |
                    CTS_TEST_FLAG_DUMP_TEST_DATA_TO_CONSOLE;
        test_param.num_invalid_node = 0;
        test_param.invalid_nodes = NULL;
        break;
    case TEST_PIN_INT:
    case TEST_PIN_RESET:
    default:
        break;
    }
    return test_param;
}

static ssize_t cts_factory_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int err = 1;
    struct chipone_ts_data *cts_data = dev_get_drvdata(MMI_test_dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;

    struct cts_test_param test_param = init_environment(TEST_PIN_INT);

    err = cts_test_int_pin(cts_dev, &test_param);
#if 0
    struct chipone_ts_data *cts_data = dev_get_drvdata(MMI_test_dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;

    struct cts_test_param test_param = init_environment(TEST_PIN_INT);

    err = cts_test_int_pin(cts_dev, &test_param);
    if (err) {
        cts_info("ant ctp mmi test: init pin fail!");
        sprintf(buf, "init pin fail: %d\n", err);
        return -1;
    }
    mdelay(5);

    test_param = init_environment(TEST_PIN_RESET);
    err = cts_test_reset_pin(cts_dev, &test_param);
    if (err) {
        cts_info("ant ctp mmi test: reset pin fail!");
        sprintf(buf, "reset pin fail: %d\n", err);
        return -1;
    }
    mdelay(5);
#endif
#if 0
    test_param = init_environment(TEST_RAWDATA);
    err = cts_test_rawdata(cts_dev, &test_param);
    if (err) {
        cts_info("ant ctp mmi test: init pin fail!");
        sprintf(buf, "init pin fail: %d\n", err);
        return -1;
    }
    mdelay(5);

    test_param = init_environment(TEST_NOISE);
    err = cts_test_noise(cts_dev, &test_param);
    if (err) {
        cts_info("ant ctp mmi test: noise fail!");
        sprintf(buf, "noise fail: %d\n", err);
        return -1;
    }
    mdelay(5);

    test_param = init_environment(TEST_OPEN);
    err = cts_test_open(cts_dev, &test_param);
    if (err) {
        cts_info("ant ctp mmi test: open fail!");
        sprintf(buf, "open fail: %d\n", err);
        return -1;
    }
    mdelay(5);

    test_param = init_environment(TEST_SHORT);
    err = cts_test_short(cts_dev, &test_param);
    if (err) {
        cts_info("ant ctp mmi test: short fail!");
        sprintf(buf, "short fail: %d\n", err);
        return -1;
    }
    mdelay(5);

    test_param = init_environment(TEST_COMPENSATE_CAP);
    err = cts_test_compensate_cap(cts_dev, &test_param);
    if (err) {
        cts_info("ant ctp mmi test: compensate cap fail!");
        sprintf(buf, "compensate cap fail: %d\n", err);
        return -1;
    }
#endif

    // temp
    mdelay(2000);

    err = 1;
    return sprintf(buf, "%d\n", err);
}


//Antai <AI_BSP_TP> <chenht> <2023-05-09> add edge restain for 2206 begin
static ssize_t cts_edge_restain_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(MMI_test_dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    int ret;

    cts_info("Read sysfs '/%s'", attr->attr.name);

    cts_lock_device(cts_dev);
    ret = cts_tcs_get_panel_direction(cts_dev, &direction);
    cts_unlock_device(cts_dev);
    if (ret) {
        cts_err("get panel direction failed!");
        return scnprintf(buf, PAGE_SIZE, "Read error\n");
    }

    return scnprintf(buf, PAGE_SIZE, "direction: 0x%02x\n", direction);
}

static ssize_t cts_edge_restain_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(MMI_test_dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    //u8 direction;
    int ret;
    long val = -1;

    cts_info("Write sysfs '/%s' size %zu", attr->attr.name, count);

    ret = kstrtoul(buf, 10, &val); 
    if(ret != 0) {
        printk(KERN_ERR "%s, invalid value \n",__func__);
    }

    if ( val >= 2)
        val = 2;

    /************************************************************************************
    "val" is the value form frameworks, "direction" is the value write into TP firmware
    *************************************************************************************/
    switch (val) {
        case 0 : direction = 1; break;//normal
        case 1 : direction = 0; break;//notch left
        case 2 : direction = 2; break;//notch right
        default:
            cts_err("Invalid arg for mode");
            return -EINVAL;
    }
    cts_info("direction = %d", direction);

    cts_lock_device(cts_dev);
    ret = cts_tcs_set_panel_direction(cts_dev, direction);
    cts_unlock_device(cts_dev);

    if (ret) {
        cts_err("Set edge restain failed");
        return -EIO;
    }

    return count;
}
static DEVICE_ATTR(landscape_mode, S_IWUSR | S_IRUGO,
        cts_edge_restain_show, cts_edge_restain_store);

//Antai <AI_BSP_TP> <chenht> <2023-05-09> add edge restain for 2206 end
/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/
//Antai <AI_BSP_TP> <chenht> <2023-02-14> add gesture mode for 2206 begin
#ifdef CFG_CTS_GESTURE
extern ssize_t cts_double_wake_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t cts_double_wake_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t cts_gesture_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t cts_gesture_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
//extern ssize_t cts_gesture_config_show(struct device *dev,struct device_attribute *attr, char *buf);
//extern ssize_t cts_gesture_config_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t cts_gesture_buf_show(struct device *dev,struct device_attribute *attr, char *buf);
extern ssize_t cts_gesture_buf_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count);

//Antai <AI_BSP_TP> <chenht> <2023-02-14> add gesture mode for 2206 end
/*****************************************************************************
* Static function prototypes
*****************************************************************************/

static DEVICE_ATTR (double_wake, S_IRUGO|S_IWUSR, cts_double_wake_show, cts_double_wake_store);
static DEVICE_ATTR (gesture_wake, S_IRUGO|S_IWUSR, cts_gesture_show, cts_gesture_store);
static DEVICE_ATTR (gesture_coordition, S_IRUGO|S_IWUSR, cts_gesture_buf_show, cts_gesture_buf_store);
//static DEVICE_ATTR(gesture_config, S_IRUGO|S_IWUSR, cts_gesture_config_show, cts_gesture_config_store);
#endif /* CFG_CTS_GESTURE */


static DEVICE_ATTR(factory_check, S_IRUGO, cts_factory_show, NULL);


static struct device_attribute *tp_feature_attr_list[] = {
    &dev_attr_factory_check,
//Antai <AI_BSP_TP> <chenht> <2023-02-14> add gesture mode for 2206 begin	
#ifdef CFG_CTS_GESTURE
    &dev_attr_double_wake,
    &dev_attr_gesture_wake,
    &dev_attr_gesture_coordition,
    //&dev_attr_gesture_config,
#endif /* CFG_CTS_GESTURE */
    //Antai <AI_BSP_TP> <chenht> <2023-05-09> add edge restain for 2206 begin
    &dev_attr_landscape_mode,
    //Antai <AI_BSP_TP> <chenht> <2023-05-09> add edge restain for 2206 end
//Antai <AI_BSP_TP> <chenht> <2023-02-14> add gesture mode for 2206 end
};

static int tpd_create_attr(struct device *dev)
{
    int idx, err = 0;
    int num = (int)(sizeof(tp_feature_attr_list)/sizeof(tp_feature_attr_list[0]));
    if (dev == NULL)
    {
        return -EINVAL;
    }
    cts_dbg("tpd_create_attr ----0 \n");
    for(idx = 0; idx < num; idx++)
    {
        err = device_create_file(dev, tp_feature_attr_list[idx]);
        if(err)
        {
            cts_dbg("TPD  driver_create_file failed");
            break;
        }
    }
    cts_dbg("TPD  driver_create_file success\n");
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

int ai_cts_tpd_feature_init_data(struct device *dev)
{
	int ret = 0;
    MMI_test_dev = dev;

	ret = platform_device_register(&ai_tp_wake_device);
	if (ret)
	{
		cts_dbg("tpd: create ai_tp_wake_device failed\n");
		return -1;
	}
	ret = tpd_create_attr(&(ai_tp_wake_device.dev));
	if(ret)
	{
		cts_dbg("tpd: create ai_tp_feature attr failed \n");
		return -1;
	}
	return ret;

}

int ai_cts_tpd_feature_exit(struct device *dev)
{
	cts_dbg(KERN_INFO "tpd_gesture: char dev clean up \n");
	tpd_delete_attr(&(ai_tp_wake_device.dev));
	platform_device_unregister(&ai_tp_wake_device);

    return 0;
}


/*
static int ant_ctp_mmi_test_remove_attr(struct device *dev)
{
    int num = (int)(sizeof(ant_ctp_mmi_test_attr_list) / sizeof(ant_ctp_mmi_test_attr_list[0]));
    int i;

    for (i = 0; i < num; i++) {
        device_remove_file(dev, ant_ctp_mmi_test_attr_list[i]);
    }

    return 0;
}

static int ant_ctp_mmi_test_create_attr(struct device *dev) {
    int num = (int)(sizeof(ant_ctp_mmi_test_attr_list) / sizeof(ant_ctp_mmi_test_attr_list[0]));
    int i, err;

    for (i = 0; i < num; i++) {
        err = device_create_file(dev, ant_ctp_mmi_test_attr_list[i]);
        if (err) {
            goto create_attr_fail;
        }
    }

    return 0;

create_attr_fail:
    ant_ctp_mmi_test_remove_attr(dev);
    return err;
}

int ant_cts_test_mode_init(struct device *dev) {

    int ret = 0;
    MMI_test_dev = dev;

    ret = platform_device_register(&ant_ctp_mmi_test_device);
    if (ret) {
        cts_info("ant ctp mmi test device create fail!");
        return -ENODEV;
    }

    ant_ctp_mmi_test_create_attr(&(ant_ctp_mmi_test_device.dev));
    if (ret) {
        cts_info("ant ctp mmi test attr create fail!");
        platform_device_unregister(&ant_ctp_mmi_test_device);
        return ret;
    }
    return ret;
}

int ant_ctp_test_mode_exit(struct device *dev)
{
    ant_ctp_mmi_test_remove_attr(&(ant_ctp_mmi_test_device.dev));
    return 0;
}
*/