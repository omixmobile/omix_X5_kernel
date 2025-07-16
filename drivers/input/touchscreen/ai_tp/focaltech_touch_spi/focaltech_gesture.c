/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2012-2020, Focaltech Ltd. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*****************************************************************************
*
* File Name: focaltech_gestrue.c
*
* Author: Focaltech Driver Team
*
* Created: 2016-08-08
*
* Abstract:
*
* Reference:
*
*****************************************************************************/

/*****************************************************************************
* 1.Included header files
*****************************************************************************/
#include "focaltech_core.h"

//Antaiui <AI_BSP_CTP> <chenht> <2022-11-30> add for gesture mode begin
#include <linux/of_device.h>
//Antaiui <AI_BSP_CTP> <chenht> <2022-11-30> add for gesture mode end
/******************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
//Antai <AI_BSP_TP> <chenht> <2022-11-25> modify for 2206 gesture begin
#define KEY_GESTURE_U                           KEY_F17     //KEY_U
#define KEY_GESTURE_UP                          KEY_UP
#define KEY_GESTURE_DOWN                        KEY_DOWN
#define KEY_GESTURE_LEFT                        KEY_LEFT
#define KEY_GESTURE_RIGHT                       KEY_RIGHT
#define KEY_GESTURE_O                           195         //KEY_O
#define KEY_GESTURE_E                           KEY_F18     //KEY_E
#define KEY_GESTURE_M                           KEY_F19     //KEY_M
#define KEY_GESTURE_L                           KEY_L
#define KEY_GESTURE_W                           KEY_F23     //KEY_W
#define KEY_GESTURE_S                           KEY_S
#define KEY_GESTURE_V                           KEY_F13     //KEY_V
#define KEY_GESTURE_C                           KEY_C
#define KEY_GESTURE_Z                           KEY_Z
//Antai <AI_BSP_TP> <chenht> <2022-11-25> modify for 2206 gesture end

#define GESTURE_LEFT                            0x20
#define GESTURE_RIGHT                           0x21
#define GESTURE_UP                              0x22
#define GESTURE_DOWN                            0x23
#define GESTURE_DOUBLECLICK                     0x24
#define GESTURE_O                               0x30
#define GESTURE_W                               0x31
#define GESTURE_M                               0x32
#define GESTURE_E                               0x33
#define GESTURE_L                               0x44
#define GESTURE_S                               0x46
#define GESTURE_V                               0x54
#define GESTURE_Z                               0x41
#define GESTURE_C                               0x34

/*****************************************************************************
* Private enumerations, structures and unions using typedef
*****************************************************************************/
/*
* gesture_id    - mean which gesture is recognised
* point_num     - points number of this gesture
* coordinate_x  - All gesture point x coordinate
* coordinate_y  - All gesture point y coordinate
* mode          - gesture enable/disable, need enable by host
*               - 1:enable gesture function(default)  0:disable
* active        - gesture work flag,
*                 always set 1 when suspend, set 0 when resume
*/
struct fts_gesture_st {
    u8 gesture_id;
    u8 point_num;
    u16 coordinate_x[FTS_GESTURE_POINTS_MAX];
    u16 coordinate_y[FTS_GESTURE_POINTS_MAX];
};

//Antai <AI_BSP_TP> <chenht> <2022-12-29> add for 2206 gesture animation begin
struct ges_feature_info
{
    union {
        struct{
            u16 start_x;
            u16 start_y;
            u16 end_x;
            u16 end_y;
            u16 width;
            u16 height;
            u16 mid_x;
            u16 mid_y;
            u16 top_x;
            u16 top_y;
            u16 bottom_x;
            u16 bottom_y;
            u16 left_x;
            u16 left_y;
            u16 right_x;
            u16 right_y;
        };
        u16 data[16];
    };
};

struct ft_gesture_st {
    struct ges_feature_info f_point;
};

static struct ft_gesture_st ft_gesture_data;
//Antai <AI_BSP_TP> <chenht> <2022-12-29> add for 2206 gesture animation end
/*****************************************************************************
* Static variables
*****************************************************************************/
static struct fts_gesture_st fts_gesture_data;

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/
//Antai <AI_BSP_TP> <chenht> <2022-11-30> modify for 2206 gesture begin
int fts_wake_switch = 0;
int fts_gesture_switch = 0;
unsigned int gesture_cfg;
//Antai <AI_BSP_TP> <chenht> <2022-11-30> modify for 2206 gesture end
/*****************************************************************************
* Static function prototypes
*****************************************************************************/

ssize_t fts_gesture_config_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    return sprintf(buf,"%u\n",gesture_cfg);
}
	
ssize_t fts_gesture_config_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    int rt;
    unsigned long val;

	
    rt = kstrtoul(buf, 10, &val);
    if(rt != 0) {
		FTS_DEBUG("%s, invalid value", __func__);
    }
    gesture_cfg = val & 0xffff;

	FTS_DEBUG("%s gesture_cfg=%lu\n", __func__, gesture_cfg);

    return count;
}

ssize_t fts_double_wake_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", fts_wake_switch);
}

ssize_t fts_double_wake_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int rt;
	unsigned long val;

    
    rt = kstrtoul(buf, 10, &val);
    if(rt != 0){
        FTS_DEBUG("invalid value\n");
        return rt;
    }
	
	fts_wake_switch = val;

	FTS_DEBUG("fts_double_wake_store value : %d\n", __func__, fts_wake_switch);


    return count;
}


ssize_t fts_gesture_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", fts_gesture_switch);
}

ssize_t fts_gesture_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    
	int rt;
	unsigned long val;

    rt = kstrtoul(buf, 10, &val);
    if(rt != 0){
        FTS_DEBUG("invalid value\n");
        return rt;
    }
	
	fts_gesture_switch = val;

    
	FTS_DEBUG("fts_gesture_store value : %d\n", __func__, fts_gesture_switch);


    return count;
}




//Antai <AI_BSP_TP> <chenht> <2022-12-29> add for 2206 gesture animation begin
void set_location_area_point()
{
		ft_gesture_data.f_point.start_x = 60;
		ft_gesture_data.f_point.start_y = 500;
		ft_gesture_data.f_point.end_x = 660;
		ft_gesture_data.f_point.end_y = 1100;
		ft_gesture_data.f_point.width = 600;
		ft_gesture_data.f_point.height = 600;
		ft_gesture_data.f_point.mid_x = 360;
		ft_gesture_data.f_point.mid_y = 800;
		ft_gesture_data.f_point.top_x = 360;
		ft_gesture_data.f_point.top_y = 500;
		ft_gesture_data.f_point.bottom_x = 360;
		ft_gesture_data.f_point.bottom_y = 1100;
		ft_gesture_data.f_point.left_x = 60;
		ft_gesture_data.f_point.left_y = 800;
		ft_gesture_data.f_point.right_x = 660;
		ft_gesture_data.f_point.right_y = 800;
}
//Antai <AI_BSP_TP> <chenht> <2022-12-29> add for 2206 gesture animation end

//Antai <AI_BSP_TP> <chenht> <2022-12-29> modify for 2206 gesture animation begin
ssize_t fts_gesture_buf_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int count;
    int i = 0;
    int len = 0;
    set_location_area_point();
    count = sizeof(ft_gesture_data.f_point)/sizeof(ft_gesture_data.f_point.data[0]);

    for (i = 0; i < count; i++)
    {
        if(i==count-1)
            len += sprintf(buf+len,"%d",ft_gesture_data.f_point.data[i]);
        else
            len += sprintf(buf+len,"%d,",ft_gesture_data.f_point.data[i]);
    }
    return len;
}
//Antai <AI_BSP_TP> <chenht> <2022-12-29> modify for 2206 gesture animation end


ssize_t fts_gesture_buf_store(
    struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    return -EPERM;
}


static void fts_gesture_report(struct input_dev *input_dev, int gesture_id)
{
    int gesture;

    FTS_DEBUG("gesture_id:0x%x", gesture_id);
    if((gesture_id == GESTURE_DOUBLECLICK) && (fts_wake_switch == 1)) {
        input_report_key(input_dev, KEY_GESTURE_U, 1);
        input_sync(input_dev);
        input_report_key(input_dev, KEY_GESTURE_U, 0);
        input_sync(input_dev);
        return;
    }

    if(fts_gesture_switch == 1) {
        switch (gesture_id) {
            case GESTURE_LEFT:
                gesture = KEY_GESTURE_LEFT;
                break;
            case GESTURE_RIGHT:
                gesture = KEY_GESTURE_RIGHT;
                break;
            case GESTURE_UP:
                gesture = KEY_GESTURE_UP;
                break;
            case GESTURE_DOWN:
                gesture = KEY_GESTURE_DOWN;
                break;
            //case GESTURE_DOUBLECLICK:
            //    gesture = KEY_GESTURE_U;
            //    break;
            case GESTURE_O:
                gesture = KEY_GESTURE_O;
                break;
            case GESTURE_W:
                gesture = KEY_GESTURE_W;
                break;
            case GESTURE_M:
                gesture = KEY_GESTURE_M;
                break;
            case GESTURE_E:
                gesture = KEY_GESTURE_E;
                break;
            case GESTURE_L:
                gesture = KEY_GESTURE_L;
                break;
            case GESTURE_S:
                gesture = KEY_GESTURE_S;
                break;
            case GESTURE_V:
                gesture = KEY_GESTURE_V;
                break;
            case GESTURE_Z:
                gesture = KEY_GESTURE_Z;
                break;
            case  GESTURE_C:
                gesture = KEY_GESTURE_C;
                break;
            default:
                gesture = -1;
                break;
        }
    }
    /* report event key */
    if (gesture != -1) {
        FTS_DEBUG("Gesture Code=%d", gesture);
        input_report_key(input_dev, gesture, 1);
        input_sync(input_dev);
        input_report_key(input_dev, gesture, 0);
        input_sync(input_dev);
    }
}

/*****************************************************************************
* Name: fts_gesture_readdata
* Brief: Read information about gesture: enable flag/gesture points..., if ges-
*        ture enable, save gesture points' information, and report to OS.
*        It will be called this function every intrrupt when FTS_GESTURE_EN = 1
*
*        gesture data length: 1(enable) + 1(reserve) + 2(header) + 6 * 4
* Input: ts_data - global struct data
*        data    - gesture data buffer
* Output:
* Return: 0 - read gesture data successfully, the report data is gesture data
*         1 - tp not in suspend/gesture not enable in TP FW
*         -Exx - error
*****************************************************************************/
int fts_gesture_readdata(struct fts_ts_data *ts_data, u8 *touch_buf)
{
    int ret = 0;
    int i = 0;
    int index = 0;
    u8 buf[FTS_GESTURE_DATA_LEN] = { 0 };
    struct input_dev *input_dev = ts_data->input_dev;
    struct fts_gesture_st *gesture = &fts_gesture_data;

    if (!ts_data->gesture_support) {
        FTS_ERROR("gesture no support");
        return -EINVAL;
    }

    if (ts_data->gesture_bmode == GESTURE_BM_TOUCH) {
        memcpy(buf, touch_buf + FTS_TOUCH_DATA_LEN, FTS_GESTURE_DATA_LEN);
    } else {
        buf[2] = FTS_REG_GESTURE_OUTPUT_ADDRESS;
        ret = fts_read(&buf[2], 1, &buf[2], FTS_GESTURE_DATA_LEN - 2);
        if (ret < 0) {
            FTS_ERROR("read gesture header data fail");
            return ret;
        }
    }

    /* init variable before read gesture point */
    memset(gesture->coordinate_x, 0, FTS_GESTURE_POINTS_MAX * sizeof(u16));
    memset(gesture->coordinate_y, 0, FTS_GESTURE_POINTS_MAX * sizeof(u16));
    gesture->gesture_id = buf[2];
    gesture->point_num = buf[3];
    FTS_DEBUG("gesture_id=%d, point_num=%d",
              gesture->gesture_id, gesture->point_num);

    /* save point data,max:6 */
    for (i = 0; i < FTS_GESTURE_POINTS_MAX; i++) {
        index = 4 * i + 4;
        gesture->coordinate_x[i] = (u16)(((buf[0 + index] & 0x0F) << 8)
                                         + buf[1 + index]);
        gesture->coordinate_y[i] = (u16)(((buf[2 + index] & 0x0F) << 8)
                                         + buf[3 + index]);
    }

    /* report gesture to OS */
    fts_gesture_report(input_dev, gesture->gesture_id);
    return 0;
}

void fts_gesture_recovery(struct fts_ts_data *ts_data)
{
    if (ts_data->gesture_support && ts_data->suspended) {
        FTS_DEBUG("gesture recovery...");
        fts_write_reg(0xD1, 0xFF);
        fts_write_reg(0xD2, 0xFF);
        fts_write_reg(0xD5, 0xFF);
        fts_write_reg(0xD6, 0xFF);
        fts_write_reg(0xD7, 0xFF);
        fts_write_reg(0xD8, 0xFF);
        fts_write_reg(FTS_REG_GESTURE_EN, ENABLE);
    }
}

int fts_gesture_suspend(struct fts_ts_data *ts_data)
{
    int i = 0;
    u8 state = 0xFF;

    FTS_FUNC_ENTER();
    if (enable_irq_wake(ts_data->irq)) {
        FTS_DEBUG("enable_irq_wake(irq:%d) fail", ts_data->irq);
    }

    for (i = 0; i < 5; i++) {
        fts_write_reg(0xD1, 0xFF);
        fts_write_reg(0xD2, 0xFF);
        fts_write_reg(0xD5, 0xFF);
        fts_write_reg(0xD6, 0xFF);
        fts_write_reg(0xD7, 0xFF);
        fts_write_reg(0xD8, 0xFF);
        fts_write_reg(FTS_REG_GESTURE_EN, ENABLE);
        msleep(1);
        fts_read_reg(FTS_REG_GESTURE_EN, &state);
        if (state == ENABLE)
            break;
    }

    if (i >= 5)
        FTS_ERROR("make IC enter into gesture(suspend) fail,state:%x", state);
    else
        FTS_INFO("Enter into gesture(suspend) successfully");

    FTS_FUNC_EXIT();
    return 0;
}

int fts_gesture_resume(struct fts_ts_data *ts_data)
{
    int i = 0;
    u8 state = 0xFF;

    FTS_FUNC_ENTER();
    if (disable_irq_wake(ts_data->irq)) {
        FTS_DEBUG("disable_irq_wake(irq:%d) fail", ts_data->irq);
    }

    for (i = 0; i < 5; i++) {
        fts_write_reg(FTS_REG_GESTURE_EN, DISABLE);
        msleep(1);
        fts_read_reg(FTS_REG_GESTURE_EN, &state);
        if (state == DISABLE)
            break;
    }

    if (i >= 5)
        FTS_ERROR("make IC exit gesture(resume) fail,state:%x", state);
    else
        FTS_INFO("resume from gesture successfully");

    FTS_FUNC_EXIT();
    return 0;
}

int fts_gesture_init(struct fts_ts_data *ts_data)
{
    struct input_dev *input_dev = ts_data->input_dev;

    FTS_FUNC_ENTER();
    input_set_capability(input_dev, EV_KEY, KEY_POWER);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_U);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_UP);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_DOWN);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_LEFT);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_RIGHT);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_O);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_E);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_M);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_L);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_W);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_S);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_V);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_Z);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_C);

    __set_bit(KEY_GESTURE_RIGHT, input_dev->keybit);
    __set_bit(KEY_GESTURE_LEFT, input_dev->keybit);
    __set_bit(KEY_GESTURE_UP, input_dev->keybit);
    __set_bit(KEY_GESTURE_DOWN, input_dev->keybit);
    __set_bit(KEY_GESTURE_U, input_dev->keybit);
    __set_bit(KEY_GESTURE_O, input_dev->keybit);
    __set_bit(KEY_GESTURE_E, input_dev->keybit);
    __set_bit(KEY_GESTURE_M, input_dev->keybit);
    __set_bit(KEY_GESTURE_W, input_dev->keybit);
    __set_bit(KEY_GESTURE_L, input_dev->keybit);
    __set_bit(KEY_GESTURE_S, input_dev->keybit);
    __set_bit(KEY_GESTURE_V, input_dev->keybit);
    __set_bit(KEY_GESTURE_C, input_dev->keybit);
    __set_bit(KEY_GESTURE_Z, input_dev->keybit);

    //fts_create_gesture_sysfs(ts_data->dev);

    memset(&fts_gesture_data, 0, sizeof(struct fts_gesture_st));
    ts_data->gesture_bmode = GESTURE_BM_REG;
    ts_data->gesture_support = FTS_GESTURE_EN;

    if ((ts_data->ic_info.ids.type <= 0x25)
        || (ts_data->ic_info.ids.type == 0x87)
        || (ts_data->ic_info.ids.type == 0x88)) {
        FTS_INFO("ic type:0x%02x,GESTURE_BM_TOUCH", ts_data->ic_info.ids.type);
        ts_data->touch_size += FTS_GESTURE_DATA_LEN;
        ts_data->gesture_bmode = GESTURE_BM_TOUCH;
    }

    FTS_FUNC_EXIT();
    return 0;
}

int fts_gesture_exit(struct fts_ts_data *ts_data)
{
    FTS_FUNC_ENTER();
    //sysfs_remove_group(&ts_data->dev->kobj, &fts_gesture_group);
    FTS_FUNC_EXIT();
    return 0;
}
