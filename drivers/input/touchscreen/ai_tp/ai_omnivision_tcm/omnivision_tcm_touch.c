/*
 * omnivision TCM touchscreen driver
 *
 * Copyright (C) 2017-2018 omnivision Incorporated. All rights reserved.
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
 * INFORMATION CONTAINED IN THIS DOCUMENT IS PROVIDED "AS-IS," AND omnivision
 * EXPRESSLY DISCLAIMS ALL EXPRESS AND IMPLIED WARRANTIES, INCLUDING ANY
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE,
 * AND ANY WARRANTIES OF NON-INFRINGEMENT OF ANY INTELLECTUAL PROPERTY RIGHTS.
 * IN NO EVENT SHALL omnivision BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, PUNITIVE, OR CONSEQUENTIAL DAMAGES ARISING OUT OF OR IN CONNECTION
 * WITH THE USE OF THE INFORMATION CONTAINED IN THIS DOCUMENT, HOWEVER CAUSED
 * AND BASED ON ANY THEORY OF LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, AND EVEN IF omnivision WAS ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE. IF A TRIBUNAL OF COMPETENT JURISDICTION DOES
 * NOT PERMIT THE DISCLAIMER OF DIRECT DAMAGES OR ANY OTHER DAMAGES, omnivision'
 * TOTAL CUMULATIVE LIABILITY TO ANY PARTY SHALL NOT EXCEED ONE HUNDRED U.S.
 * DOLLARS.
 */

#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include "omnivision_tcm_core.h"
//Antaiui <AI_BSP_CTP> <chenht> <2022-12-21> add for gesture mode begin
#include <linux/of_device.h>
//Antaiui <AI_BSP_CTP> <chenht> <2022-12-21> add for gesture mode end
#define TYPE_B_PROTOCOL
//Antai <AI_BSP_TP> <chenht> <2022-12-21> modify for 2206 gesture begin
#include "ai_tpd_feature.h"
//Antai <AI_BSP_TP> <chenht> <2022-12-21> modify for 2206 gesture end


//Antai <AI_BSP_TP> <chenht> <2022-12-21> add for 2206 gesture begin
#define KEY_DOUBLE_CLICK_WAKEUP                 KEY_F17 //187    //KEY_U
#define KEY_GESTURE_O                           195         //KEY_O
#define KEY_GESTURE_E                           KEY_F18     //KEY_E
#define KEY_GESTURE_M                           KEY_F19     //KEY_M
#define KEY_GESTURE_W                           KEY_F23     //KEY_W
#define KEY_GESTURE_V                           KEY_F13     //KEY_V
//Antai <AI_BSP_TP> <chenht> <2022-12-21> add for 2206 gesture end

//Antai <AI_BSP_TP> <chenht> <2022-12-21> add for 2206 gesture begin
#if WAKEUP_GESTURE 
int ovt_wake_switch = 0;
int ovt_gesture_switch = 0;
#endif
//Antai <AI_BSP_TP> <chenht> <2022-12-21> add for 2206 gesture end

//#define USE_DEFAULT_TOUCH_REPORT_CONFIG

#define TOUCH_REPORT_CONFIG_SIZE 128
#include "../tpd.h"
extern struct tpd_device *tpd;

enum touch_status {
	LIFT = 0,
	FINGER = 1,
	GLOVED_FINGER = 2,
	NOP = -1,
};

//Antai <AI_BSP_TP> <chenht> <2022-12-21> modify for 2206 gesture begin
enum gesture_id {
	NO_GESTURE_DETECTED = 0,
	GESTURE_DOUBLE_TAP = 0X01,
	GESTURE_V = 0X05,
	GESTURE_O = 0X10,
	GESTURE_E = 0X65,
	GESTURE_M = 0X6d,
	GESTURE_W = 0X77,
};

enum touch_report_code {
	TOUCH_END = 0,
	TOUCH_FOREACH_ACTIVE_OBJECT,
	TOUCH_FOREACH_OBJECT,
	TOUCH_FOREACH_END,
	TOUCH_PAD_TO_NEXT_BYTE,
	TOUCH_TIMESTAMP,
	TOUCH_OBJECT_N_INDEX,
	TOUCH_OBJECT_N_CLASSIFICATION,
	TOUCH_OBJECT_N_X_POSITION,
	TOUCH_OBJECT_N_Y_POSITION,
	TOUCH_OBJECT_N_Z,
	TOUCH_OBJECT_N_X_WIDTH,
	TOUCH_OBJECT_N_Y_WIDTH,
	TOUCH_OBJECT_N_TX_POSITION_TIXELS,
	TOUCH_OBJECT_N_RX_POSITION_TIXELS,
	TOUCH_0D_BUTTONS_STATE,
	TOUCH_GESTURE_ID,
	TOUCH_FRAME_RATE,
	TOUCH_POWER_IM,
	TOUCH_CID_IM,
	TOUCH_RAIL_IM,
	TOUCH_CID_VARIANCE_IM,
	TOUCH_NSM_FREQUENCY,
	TOUCH_NSM_STATE,
	TOUCH_NUM_OF_ACTIVE_OBJECTS,
	TOUCH_NUM_OF_CPU_CYCLES_USED_SINCE_LAST_FRAME,
	TOUCH_FACE_DETECT,
	TOUCH_GESTURE_DATA,
	TOUCH_OBJECT_N_FORCE,
	TOUCH_FINGERPRINT_AREA_MEET,
	TOUCH_TUNING_GAUSSIAN_WIDTHS = 0x80,
	TOUCH_TUNING_SMALL_OBJECT_PARAMS,
	TOUCH_TUNING_0D_BUTTONS_VARIANCE,
};

struct object_data {
	unsigned char status;
	unsigned int x_pos;
	unsigned int y_pos;
	unsigned int x_width;
	unsigned int y_width;
	unsigned int z;
	unsigned int tx_pos;
	unsigned int rx_pos;
};

struct input_params {
	unsigned int max_x;
	unsigned int max_y;
	unsigned int max_objects;
};

struct touch_data {
	struct object_data *object_data;
	unsigned int timestamp;
	unsigned int buttons_state;
	unsigned int gesture_id;
	unsigned int frame_rate;
	unsigned int power_im;
	unsigned int cid_im;
	unsigned int rail_im;
	unsigned int cid_variance_im;
	unsigned int nsm_frequency;
	unsigned int nsm_state;
	unsigned int num_of_active_objects;
	unsigned int num_of_cpu_cycles;
	unsigned int fd_data;
	unsigned int force_data;
	unsigned int fingerprint_area_meet;
};

struct touch_hcd {
	bool irq_wake;
	bool init_touch_ok;
	bool suspend_touch;
	unsigned char *prev_status;
	unsigned int max_x;
	unsigned int max_y;
	unsigned int max_objects;
	struct mutex report_mutex;
	struct input_dev *input_dev;
	struct touch_data touch_data;
	struct input_params input_params;
	struct ovt_tcm_buffer out;
	struct ovt_tcm_buffer resp;
	struct ovt_tcm_hcd *tcm_hcd;
};

static struct touch_hcd *touch_hcd;
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

struct ovt_gesture_st {
    struct ges_feature_info f_point;
};

static struct ovt_gesture_st ovt_gesture_data;
//Antai <AI_BSP_TP> <chenht> <2022-12-29> add for 2206 gesture animation end

//Antai <AI_BSP_TP> <chenht> <2022-12-21> modify for 2206 gesture begin

#if WAKEUP_GESTURE

ssize_t ovt_double_wake_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", ovt_wake_switch);
}

ssize_t ovt_double_wake_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int rt;
	unsigned long val;
	struct ovt_tcm_hcd *tcm_hcd = touch_hcd->tcm_hcd;
    rt = kstrtoul(buf, 10, &val);
    if(rt != 0){
        LOGD(tcm_hcd->pdev->dev.parent,"invalid value\n");
        return rt;
    }
	
	ovt_wake_switch = val;
	LOGD(tcm_hcd->pdev->dev.parent,"ovt_double_wake_store value : %d\n", ovt_wake_switch);

    return count;
}


ssize_t ovt_gesture_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", ovt_gesture_switch);
}

ssize_t ovt_gesture_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    
	int rt;
	unsigned long val;
	struct ovt_tcm_hcd *tcm_hcd = touch_hcd->tcm_hcd;
	
    rt = kstrtoul(buf, 10, &val);
    if(rt != 0){
        LOGD(tcm_hcd->pdev->dev.parent,"invalid value\n");
        return rt;
    }
	
	ovt_gesture_switch = val;
	LOGD(tcm_hcd->pdev->dev.parent,"ovt_gesture_store value : %d\n", ovt_gesture_switch);
    return count;
}

//Antai <AI_BSP_TP> <chenht> <2022-12-29> add for 2206 gesture animation begin
void ovt_set_location_area_point()
{
		ovt_gesture_data.f_point.start_x = 60;
		ovt_gesture_data.f_point.start_y = 500;
		ovt_gesture_data.f_point.end_x = 660;
		ovt_gesture_data.f_point.end_y = 1100;
		ovt_gesture_data.f_point.width = 600;
		ovt_gesture_data.f_point.height = 600;
		ovt_gesture_data.f_point.mid_x = 360;
		ovt_gesture_data.f_point.mid_y = 800;
		ovt_gesture_data.f_point.top_x = 360;
		ovt_gesture_data.f_point.top_y = 500;
		ovt_gesture_data.f_point.bottom_x = 360;
		ovt_gesture_data.f_point.bottom_y = 1100;
		ovt_gesture_data.f_point.left_x = 60;
		ovt_gesture_data.f_point.left_y = 800;
		ovt_gesture_data.f_point.right_x = 660;
		ovt_gesture_data.f_point.right_y = 800;
}
//Antai <AI_BSP_TP> <chenht> <2022-12-29> add for 2206 gesture animation end

//Antai <AI_BSP_TP> <chenht> <2022-12-29> modify for 2206 gesture animation begin
ssize_t ovt_gesture_buf_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int count;
    int i = 0;
    int len = 0;
    ovt_set_location_area_point();
    count = sizeof(ovt_gesture_data.f_point)/sizeof(ovt_gesture_data.f_point.data[0]);

    for (i = 0; i < count; i++)
    {
        if(i==count-1)
            len += sprintf(buf+len,"%d",ovt_gesture_data.f_point.data[i]);
        else
            len += sprintf(buf+len,"%d,",ovt_gesture_data.f_point.data[i]);
    }
    

    return len;
}

ssize_t ovt_gesture_buf_store(
    struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    return -EPERM;
}
//Antai <AI_BSP_TP> <chenht> <2022-12-29> modify for 2206 gesture animation end
#endif
//Antai <AI_BSP_TP> <chenht> <2022-12-21> modify for 2206 gesture end

/**
 * touch_free_objects() - Free all touch objects
 *
 * Report finger lift events to the input subsystem for all touch objects.
 */
static void touch_free_objects(void)
{
#ifdef TYPE_B_PROTOCOL
	unsigned int idx;
#endif

	if (touch_hcd->input_dev == NULL)
		return;

	mutex_lock(&touch_hcd->report_mutex);

#ifdef TYPE_B_PROTOCOL
	for (idx = 0; idx < touch_hcd->max_objects; idx++) {
		input_mt_slot(touch_hcd->input_dev, idx);
		input_mt_report_slot_state(touch_hcd->input_dev,
				MT_TOOL_FINGER, 0);
	}
#endif
	input_report_key(touch_hcd->input_dev,
			BTN_TOUCH, 0);
	input_report_key(touch_hcd->input_dev,
			BTN_TOOL_FINGER, 0);
#ifndef TYPE_B_PROTOCOL
	input_mt_sync(touch_hcd->input_dev);
#endif
	input_sync(touch_hcd->input_dev);

	mutex_unlock(&touch_hcd->report_mutex);

	return;
}

/**
 * touch_get_report_data() - Retrieve data from touch report
 *
 * Retrieve data from the touch report based on the bit offset and bit length
 * information from the touch report configuration.
 */
static int touch_get_report_data(unsigned int offset,
		unsigned int bits, unsigned int *data)
{
	unsigned char mask;
	unsigned char byte_data;
	unsigned int output_data;
	unsigned int bit_offset;
	unsigned int byte_offset;
	unsigned int data_bits;
	unsigned int available_bits;
	unsigned int remaining_bits;
	unsigned char *touch_report;
	struct ovt_tcm_hcd *tcm_hcd = touch_hcd->tcm_hcd;

	if (bits == 0 || bits > 32) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Invalid number of bits\n");
		return -EINVAL;
	}

	if (offset + bits > tcm_hcd->report.buffer.data_length * 8) {
		*data = 0;
		return 0;
	}

	touch_report = tcm_hcd->report.buffer.buf;

	output_data = 0;
	remaining_bits = bits;

	bit_offset = offset % 8;
	byte_offset = offset / 8;

	while (remaining_bits) {
		byte_data = touch_report[byte_offset];
		byte_data >>= bit_offset;

		available_bits = 8 - bit_offset;
		data_bits = MIN(available_bits, remaining_bits);
		mask = 0xff >> (8 - data_bits);

		byte_data &= mask;

		output_data |= byte_data << (bits - remaining_bits);

		bit_offset = 0;
		byte_offset += 1;
		remaining_bits -= data_bits;
	}

	*data = output_data;

	return 0;
}

/**
 * touch_parse_report() - Parse touch report
 *
 * Traverse through the touch report configuration and parse the touch report
 * generated by the device accordingly to retrieve the touch data.
 */
static int touch_parse_report(void)
{
	int retval;
	bool active_only;
	bool num_of_active_objects;
	unsigned char code;
	unsigned int size;
	unsigned int idx;
	unsigned int obj;
	unsigned int next;
	unsigned int data;
	unsigned int bits;
	unsigned int offset;
	unsigned int objects;
	unsigned int active_objects;
	unsigned int report_size;
	unsigned int config_size;
	unsigned char *config_data;
	struct touch_data *touch_data;
	struct object_data *object_data;
	struct ovt_tcm_hcd *tcm_hcd = touch_hcd->tcm_hcd;
	static unsigned int end_of_foreach;

	touch_data = &touch_hcd->touch_data;
	object_data = touch_hcd->touch_data.object_data;

	config_data = tcm_hcd->config.buf;
	config_size = tcm_hcd->config.data_length;

	report_size = tcm_hcd->report.buffer.data_length;

	size = sizeof(*object_data) * touch_hcd->max_objects;
	memset(touch_hcd->touch_data.object_data, 0x00, size);

	num_of_active_objects = false;

	idx = 0;
	offset = 0;
	objects = 0;
	active_objects = 0;
	active_only = false;

	while (idx < config_size) {
		code = config_data[idx++];
		switch (code) {
		case TOUCH_END:
			goto exit;
		case TOUCH_FOREACH_ACTIVE_OBJECT:
			obj = 0;
			next = idx;
			active_only = true;
			break;
		case TOUCH_FOREACH_OBJECT:
			obj = 0;
			next = idx;
			active_only = false;
			break;
		case TOUCH_FOREACH_END:
			end_of_foreach = idx;
			if (active_only) {
				if (num_of_active_objects) {
					objects++;
					if (objects < active_objects)
						idx = next;
				} else if (offset < report_size * 8) {
					idx = next;
				}
			} else {
				obj++;
				if (obj < touch_hcd->max_objects)
					idx = next;
			}
			break;
		case TOUCH_PAD_TO_NEXT_BYTE:
			offset = ceil_div(offset, 8) * 8;
			break;
		case TOUCH_TIMESTAMP:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get timestamp\n");
				return retval;
			}
			touch_data->timestamp = data;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_INDEX:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &obj);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get object index\n");
				return retval;
			}
			offset += bits;
			break;
		case TOUCH_OBJECT_N_CLASSIFICATION:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get object classification\n");
				return retval;
			}
			object_data[obj].status = data;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_X_POSITION:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get object x position\n");
				return retval;
			}
			object_data[obj].x_pos = data;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_Y_POSITION:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get object y position\n");
				return retval;
			}
			object_data[obj].y_pos = data;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_Z:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get object z\n");
				return retval;
			}
			object_data[obj].z = data;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_X_WIDTH:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get object x width\n");
				return retval;
			}
			object_data[obj].x_width = data;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_Y_WIDTH:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get object y width\n");
				return retval;
			}
			object_data[obj].y_width = data;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_TX_POSITION_TIXELS:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get object tx position\n");
				return retval;
			}
			object_data[obj].tx_pos = data;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_RX_POSITION_TIXELS:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get object rx position\n");
				return retval;
			}
			object_data[obj].rx_pos = data;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_FORCE:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get object force\n");
				return retval;
			}
			touch_data->force_data = data;
			offset += bits;
			break;
		case TOUCH_FINGERPRINT_AREA_MEET:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get object force\n");
				return retval;
			}
			touch_data->fingerprint_area_meet = data;
			LOGN(tcm_hcd->pdev->dev.parent,
					"fingerprint_area_meet = %x\n",
					touch_data->fingerprint_area_meet);
			offset += bits;
			break;
		case TOUCH_0D_BUTTONS_STATE:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get 0D buttons state\n");
				return retval;
			}
			touch_data->buttons_state = data;
			offset += bits;
			break;
		case TOUCH_GESTURE_ID:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get gesture double tap\n");
				return retval;
			}
			touch_data->gesture_id = data;
			offset += bits;
			break;
		case TOUCH_FRAME_RATE:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get frame rate\n");
				return retval;
			}
			touch_data->frame_rate = data;
			offset += bits;
			break;
		case TOUCH_POWER_IM:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get power IM\n");
				return retval;
			}
			touch_data->power_im = data;
			offset += bits;
			break;
		case TOUCH_CID_IM:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get CID IM\n");
				return retval;
			}
			touch_data->cid_im = data;
			offset += bits;
			break;
		case TOUCH_RAIL_IM:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get rail IM\n");
				return retval;
			}
			touch_data->rail_im = data;
			offset += bits;
			break;
		case TOUCH_CID_VARIANCE_IM:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get CID variance IM\n");
				return retval;
			}
			touch_data->cid_variance_im = data;
			offset += bits;
			break;
		case TOUCH_NSM_FREQUENCY:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get NSM frequency\n");
				return retval;
			}
			touch_data->nsm_frequency = data;
			offset += bits;
			break;
		case TOUCH_NSM_STATE:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get NSM state\n");
				return retval;
			}
			touch_data->nsm_state = data;
			offset += bits;
			break;
		case TOUCH_GESTURE_DATA:
			bits = config_data[idx++];
			offset += bits;
			break;
		case TOUCH_NUM_OF_ACTIVE_OBJECTS:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get number of active objects\n");
				return retval;
			}
			active_objects = data;
			num_of_active_objects = true;
			touch_data->num_of_active_objects = data;
			offset += bits;
			if (touch_data->num_of_active_objects == 0) {
				if (0 == end_of_foreach) {
					LOGE(tcm_hcd->pdev->dev.parent,
						"Invalid report, num_active and end_foreach are 0\n");
					return 0;
				}
				idx = end_of_foreach;
			}
			break;
		case TOUCH_NUM_OF_CPU_CYCLES_USED_SINCE_LAST_FRAME:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get num CPU cycles used since last frame\n");
				return retval;
			}
			touch_data->num_of_cpu_cycles = data;
			offset += bits;
			break;
		case TOUCH_FACE_DETECT:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to detect face\n");
				return retval;
			}
			touch_data->fd_data = data;
			offset += bits;
			break;
		case TOUCH_TUNING_GAUSSIAN_WIDTHS:
			bits = config_data[idx++];
			offset += bits;
			break;
		case TOUCH_TUNING_SMALL_OBJECT_PARAMS:
			bits = config_data[idx++];
			offset += bits;
			break;
		case TOUCH_TUNING_0D_BUTTONS_VARIANCE:
			bits = config_data[idx++];
			offset += bits;
			break;
		default:
			bits = config_data[idx++];
			offset += bits;
			break;
		}
	}

exit:
	return 0;
}

/**
 * touch_report() - Report touch events
 *
 * Retrieve data from the touch report generated by the device and report touch
 * events to the input subsystem.
 */
static void touch_report(void)
{
	int retval;
//Antai <AI_BSP_TP> <chenht> <2022-12-21> add for 2206 gesture begin
#if WAKEUP_GESTURE
	int gesture;
#endif
//Antai <AI_BSP_TP> <chenht> <2022-12-21> add for 2206 gesture end
	unsigned int idx;
	unsigned int x;
	unsigned int y;
	unsigned int temp;
	unsigned int status;
	unsigned int touch_count;
	struct touch_data *touch_data;
	struct object_data *object_data;
	struct ovt_tcm_hcd *tcm_hcd = touch_hcd->tcm_hcd;
	const struct ovt_tcm_board_data *bdata = tcm_hcd->hw_if->bdata;

	if (!touch_hcd->init_touch_ok)
		return;

	if (touch_hcd->input_dev == NULL)
		return;

	if (touch_hcd->suspend_touch)
		return;

	mutex_lock(&touch_hcd->report_mutex);

	retval = touch_parse_report();
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to parse touch report\n");
		goto exit;
	}

	touch_data = &touch_hcd->touch_data;
	object_data = touch_hcd->touch_data.object_data;

//Antai <AI_BSP_TP> <chenht> <2022-12-21> modify for 2206 gesture begin
#if WAKEUP_GESTURE
	if (touch_data->gesture_id == GESTURE_DOUBLE_TAP &&
			 tcm_hcd->in_suspend &&
			 tcm_hcd->wakeup_gesture_enabled &&
			 ovt_wake_switch == 1) {

		input_report_key(touch_hcd->input_dev, KEY_DOUBLE_CLICK_WAKEUP, 1);
		input_sync(touch_hcd->input_dev);
		input_report_key(touch_hcd->input_dev, KEY_DOUBLE_CLICK_WAKEUP, 0);
		input_sync(touch_hcd->input_dev);
	}

	if(ovt_gesture_switch == 1){
		switch (touch_data->gesture_id){
			case GESTURE_V:
				gesture = KEY_GESTURE_V;
				break;
			case GESTURE_O:
				gesture = KEY_GESTURE_O;
				break;
			case GESTURE_E:
				gesture = KEY_GESTURE_E;
				break;
			case GESTURE_M:
				gesture = KEY_GESTURE_M;
				break;
			case GESTURE_W:
				gesture = KEY_GESTURE_W;
				break;
			default:
				gesture = -1;
				break;
		}
	}

	/* report event key */
    if (gesture != -1) {
        LOGI(tcm_hcd->pdev->dev.parent,"Gesture Code=%d", gesture);
        input_report_key(touch_hcd->input_dev, gesture, 1);
        input_sync(touch_hcd->input_dev);
        input_report_key(touch_hcd->input_dev, gesture, 0);
        input_sync(touch_hcd->input_dev);
    }


	/* debug info*/
	LOGI(tcm_hcd->pdev->dev.parent,"touch_data->gesture_id = %d",touch_data->gesture_id);
	LOGI(tcm_hcd->pdev->dev.parent,"tcm_hcd->in_suspend = %d",tcm_hcd->in_suspend);
	LOGI(tcm_hcd->pdev->dev.parent,"tcm_hcd->wakeup_gesture_enabled = %d",tcm_hcd->wakeup_gesture_enabled);
	LOGI(tcm_hcd->pdev->dev.parent,"ovt_wake_switch = %d",ovt_wake_switch);
	LOGI(tcm_hcd->pdev->dev.parent,"ovt_gesture_switch = %d",ovt_gesture_switch);
#endif
//Antai <AI_BSP_TP> <chenht> <2022-12-21> modify for 2206 gesture end
	if (tcm_hcd->in_suspend)
		goto exit;

	touch_count = 0;

	for (idx = 0; idx < touch_hcd->max_objects; idx++) {
		if (touch_hcd->prev_status[idx] == LIFT &&
				object_data[idx].status == LIFT)
			status = NOP;
		else
			status = object_data[idx].status;

		switch (status) {
		case LIFT:
#ifdef TYPE_B_PROTOCOL
			input_mt_slot(touch_hcd->input_dev, idx);
			input_mt_report_slot_state(touch_hcd->input_dev,
					MT_TOOL_FINGER, 0);
#endif
			break;
		case FINGER:
		case GLOVED_FINGER:
			x = object_data[idx].x_pos;
			y = object_data[idx].y_pos;
			if (bdata->swap_axes) {
				temp = x;
				x = y;
				y = temp;
			}
			if (bdata->x_flip)
				x = touch_hcd->input_params.max_x - x;
			if (bdata->y_flip)
				y = touch_hcd->input_params.max_y - y;
#ifdef TYPE_B_PROTOCOL
			input_mt_slot(touch_hcd->input_dev, idx);
			input_mt_report_slot_state(touch_hcd->input_dev,
					MT_TOOL_FINGER, 1);
#endif
			input_report_key(touch_hcd->input_dev,
					BTN_TOUCH, 1);
			input_report_key(touch_hcd->input_dev,
					BTN_TOOL_FINGER, 1);
			input_report_abs(touch_hcd->input_dev,
					ABS_MT_POSITION_X, x);
			input_report_abs(touch_hcd->input_dev,
					ABS_MT_POSITION_Y, y);
#ifndef TYPE_B_PROTOCOL
			input_mt_sync(touch_hcd->input_dev);
#endif
			LOGD(tcm_hcd->pdev->dev.parent,
					"Finger %d: x = %d, y = %d\n",
					idx, x, y);
			touch_count++;
			break;
		default:
			break;
		}

		touch_hcd->prev_status[idx] = object_data[idx].status;
	}

	if (touch_count == 0) {
		input_report_key(touch_hcd->input_dev,
				BTN_TOUCH, 0);
		input_report_key(touch_hcd->input_dev,
				BTN_TOOL_FINGER, 0);
#ifndef TYPE_B_PROTOCOL
		input_mt_sync(touch_hcd->input_dev);
#endif
	}

	input_sync(touch_hcd->input_dev);

exit:
	mutex_unlock(&touch_hcd->report_mutex);

	return;
}

/**
 * touch_set_input_params() - Set input parameters
 *
 * Set the input parameters of the input device based on the information
 * retrieved from the application information packet. In addition, set up an
 * array for tracking the status of touch objects.
 */
extern struct tpd_device *tpd;
extern int tpd_register_flag;
static int touch_set_input_params(void)
{
	struct ovt_tcm_hcd *tcm_hcd = touch_hcd->tcm_hcd;
	
	touch_hcd->input_dev = tpd->dev;
	input_set_abs_params(touch_hcd->input_dev,
			ABS_MT_POSITION_X, 0, touch_hcd->max_x, 0, 0);
	input_set_abs_params(touch_hcd->input_dev,
			ABS_MT_POSITION_Y, 0, touch_hcd->max_y, 0, 0);
	input_set_abs_params(touch_hcd->input_dev,
			ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(touch_hcd->input_dev,
			ABS_MT_TOUCH_MINOR, 0, 255, 0, 0);

	input_mt_init_slots(touch_hcd->input_dev, touch_hcd->max_objects,
			INPUT_MT_DIRECT);

//Antai <AI_BSP_TP> <chenht> <2022-12-21> add for 2206 gesture begin
#if WAKEUP_GESTURE
	set_bit(KEY_DOUBLE_CLICK_WAKEUP, touch_hcd->input_dev->keybit);
	set_bit(KEY_GESTURE_V, touch_hcd->input_dev->keybit);
	set_bit(KEY_GESTURE_O, touch_hcd->input_dev->keybit);
	set_bit(KEY_GESTURE_E, touch_hcd->input_dev->keybit);
	set_bit(KEY_GESTURE_M, touch_hcd->input_dev->keybit);
	set_bit(KEY_GESTURE_W, touch_hcd->input_dev->keybit);
	
	input_set_capability(touch_hcd->input_dev, EV_KEY, KEY_DOUBLE_CLICK_WAKEUP);
	input_set_capability(touch_hcd->input_dev, EV_KEY, KEY_GESTURE_V);
	input_set_capability(touch_hcd->input_dev, EV_KEY, KEY_GESTURE_O);
	input_set_capability(touch_hcd->input_dev, EV_KEY, KEY_GESTURE_E);
	input_set_capability(touch_hcd->input_dev, EV_KEY, KEY_GESTURE_M);
	input_set_capability(touch_hcd->input_dev, EV_KEY, KEY_GESTURE_W);
#endif
//Antai <AI_BSP_TP> <chenht> <2022-12-21> add for 2206 gesture end
	touch_hcd->input_params.max_x = touch_hcd->max_x;
	touch_hcd->input_params.max_y = touch_hcd->max_y;
	touch_hcd->input_params.max_objects = touch_hcd->max_objects;

	if (touch_hcd->max_objects == 0)
		return 0;

	kfree(touch_hcd->prev_status);
	touch_hcd->prev_status = kzalloc(touch_hcd->max_objects, GFP_KERNEL);
	if (!touch_hcd->prev_status) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to allocate memory for touch_hcd->prev_status\n");
		return -ENOMEM;
	}

	return 0;
}

/**
 * touch_get_input_params() - Get input parameters
 *
 * Retrieve the input parameters to register with the input subsystem for
 * the input device from the application information packet. In addition,
 * the touch report configuration is retrieved and stored.
 */
static int touch_get_input_params(void)
{
	int retval;
	unsigned int temp;
	struct ovt_tcm_app_info *app_info;
	struct ovt_tcm_hcd *tcm_hcd = touch_hcd->tcm_hcd;
	const struct ovt_tcm_board_data *bdata = tcm_hcd->hw_if->bdata;

	app_info = &tcm_hcd->app_info;
	touch_hcd->max_x = le2_to_uint(app_info->max_x);
	touch_hcd->max_y = le2_to_uint(app_info->max_y);
	touch_hcd->max_objects = le2_to_uint(app_info->max_objects);

	if (bdata->swap_axes) {
		temp = touch_hcd->max_x;
		touch_hcd->max_x = touch_hcd->max_y;
		touch_hcd->max_y = temp;
	}

	LOCK_BUFFER(tcm_hcd->config);

	retval = tcm_hcd->write_message(tcm_hcd,
			CMD_GET_TOUCH_REPORT_CONFIG,
			NULL,
			0,
			&tcm_hcd->config.buf,
			&tcm_hcd->config.buf_size,
			&tcm_hcd->config.data_length,
			NULL,
			0);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to write command %s\n",
				STR(CMD_GET_TOUCH_REPORT_CONFIG));
		UNLOCK_BUFFER(tcm_hcd->config);
		return retval;
	}

	UNLOCK_BUFFER(tcm_hcd->config);

	return 0;
}

/**
 * touch_set_input_dev() - Set up input device
 *
 * Allocate an input device, configure the input device based on the particular
 * input events to be reported, and register the input device with the input
 * subsystem.
 */
static int touch_set_input_dev(void)
{
	int retval;
	struct ovt_tcm_hcd *tcm_hcd = touch_hcd->tcm_hcd;

	touch_hcd->input_dev = input_allocate_device();
	if (touch_hcd->input_dev == NULL) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to allocate input device\n");
		return -ENODEV;
	}

	touch_hcd->input_dev->name = TOUCH_INPUT_NAME;
	touch_hcd->input_dev->phys = TOUCH_INPUT_PHYS_PATH;
	touch_hcd->input_dev->id.product = OMNIVISION_TCM_ID_PRODUCT;
	touch_hcd->input_dev->id.version = OMNIVISION_TCM_ID_VERSION;
	touch_hcd->input_dev->dev.parent = tcm_hcd->pdev->dev.parent;
	input_set_drvdata(touch_hcd->input_dev, tcm_hcd);

	set_bit(EV_SYN, touch_hcd->input_dev->evbit);
	set_bit(EV_KEY, touch_hcd->input_dev->evbit);
	set_bit(EV_ABS, touch_hcd->input_dev->evbit);
	set_bit(BTN_TOUCH, touch_hcd->input_dev->keybit);
	set_bit(BTN_TOOL_FINGER, touch_hcd->input_dev->keybit);
#ifdef INPUT_PROP_DIRECT
	set_bit(INPUT_PROP_DIRECT, touch_hcd->input_dev->propbit);
#endif
//Antai <AI_BSP_TP> <chenht> <2022-12-21> modify for 2206 gesture begin
/*
#if WAKEUP_GESTURE
	set_bit(KEY_DOUBLE_CLICK_WAKEUP, touch_hcd->input_dev->keybit);
	input_set_capability(touch_hcd->input_dev, EV_KEY, KEY_DOUBLE_CLICK_WAKEUP);
#endif
*/
//Antai <AI_BSP_TP> <chenht> <2022-12-21> modify for 2206 gesture end
	retval = touch_set_input_params();
#if 0
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to set input parameters\n");
		input_free_device(touch_hcd->input_dev);
		touch_hcd->input_dev = NULL;
		return retval;
	}

	retval = input_register_device(touch_hcd->input_dev);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to register input device\n");
		input_free_device(touch_hcd->input_dev);
		touch_hcd->input_dev = NULL;
		return retval;
	}
#endif
	return 0;
}

/**
 * touch_set_report_config() - Set touch report configuration
 *
 * Send the SET_TOUCH_REPORT_CONFIG command to configure the format and content
 * of the touch report.
 */
static int touch_set_report_config(void)
{
	int retval;
	unsigned int idx;
	unsigned int length;
	struct ovt_tcm_app_info *app_info;
	struct ovt_tcm_hcd *tcm_hcd = touch_hcd->tcm_hcd;

#ifdef USE_DEFAULT_TOUCH_REPORT_CONFIG
	return 0;
#endif

	app_info = &tcm_hcd->app_info;
	length = le2_to_uint(app_info->max_touch_report_config_size);

	if (length < TOUCH_REPORT_CONFIG_SIZE) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Invalid maximum touch report config size\n");
		return -EINVAL;
	}

	LOCK_BUFFER(touch_hcd->out);

	retval = ovt_tcm_alloc_mem(tcm_hcd,
			&touch_hcd->out,
			length);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to allocate memory for touch_hcd->out.buf\n");
		UNLOCK_BUFFER(touch_hcd->out);
		return retval;
	}

	idx = 0;
#if WAKEUP_GESTURE
	touch_hcd->out.buf[idx++] = TOUCH_GESTURE_ID;
	touch_hcd->out.buf[idx++] = 8;
#endif
	touch_hcd->out.buf[idx++] = TOUCH_FOREACH_ACTIVE_OBJECT;
	touch_hcd->out.buf[idx++] = TOUCH_OBJECT_N_INDEX;
	touch_hcd->out.buf[idx++] = 4;
	touch_hcd->out.buf[idx++] = TOUCH_OBJECT_N_CLASSIFICATION;
	touch_hcd->out.buf[idx++] = 4;
	touch_hcd->out.buf[idx++] = TOUCH_OBJECT_N_X_POSITION;
	touch_hcd->out.buf[idx++] = 12;
	touch_hcd->out.buf[idx++] = TOUCH_OBJECT_N_Y_POSITION;
	touch_hcd->out.buf[idx++] = 12;
	touch_hcd->out.buf[idx++] = TOUCH_OBJECT_N_X_WIDTH;
	touch_hcd->out.buf[idx++] = 16;
	touch_hcd->out.buf[idx++] = TOUCH_OBJECT_N_Y_WIDTH;
	touch_hcd->out.buf[idx++] = 16;
	touch_hcd->out.buf[idx++] = TOUCH_OBJECT_N_Z;
	touch_hcd->out.buf[idx++] = 16;	
	touch_hcd->out.buf[idx++] = TOUCH_FOREACH_END;
	touch_hcd->out.buf[idx++] = TOUCH_END;

	LOCK_BUFFER(touch_hcd->resp);

	retval = tcm_hcd->write_message(tcm_hcd,
			CMD_SET_TOUCH_REPORT_CONFIG,
			touch_hcd->out.buf,
			length,
			&touch_hcd->resp.buf,
			&touch_hcd->resp.buf_size,
			&touch_hcd->resp.data_length,
			NULL,
			0);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to write command %s\n",
				STR(CMD_SET_TOUCH_REPORT_CONFIG));
		UNLOCK_BUFFER(touch_hcd->resp);
		UNLOCK_BUFFER(touch_hcd->out);
		return retval;
	}

	UNLOCK_BUFFER(touch_hcd->resp);
	UNLOCK_BUFFER(touch_hcd->out);

	LOGN(tcm_hcd->pdev->dev.parent,
			"Set touch config done\n");

	return 0;
}

/**
 * touch_check_input_params() - Check input parameters
 *
 * Check if any of the input parameters registered with the input subsystem for
 * the input device has changed.
 */
static int touch_check_input_params(void)
{
	unsigned int size;
	struct ovt_tcm_hcd *tcm_hcd = touch_hcd->tcm_hcd;

	if (touch_hcd->max_x == 0 && touch_hcd->max_y == 0)
		return 0;

	if (touch_hcd->input_params.max_objects != touch_hcd->max_objects) {
		kfree(touch_hcd->touch_data.object_data);
		size = sizeof(*touch_hcd->touch_data.object_data);
		size *= touch_hcd->max_objects;
		touch_hcd->touch_data.object_data = kzalloc(size, GFP_KERNEL);
		if (!touch_hcd->touch_data.object_data) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to allocate memory for touch_hcd->touch_data.object_data\n");
			return -ENOMEM;
		}
		return 1;
	}

	if (touch_hcd->input_params.max_x != touch_hcd->max_x)
		return 1;

	if (touch_hcd->input_params.max_y != touch_hcd->max_y)
		return 1;

	return 0;
}

/**
 * touch_set_input_reporting() - Configure touch report and set up new input
 * device if necessary
 *
 * After a device reset event, configure the touch report and set up a new input
 * device if any of the input parameters has changed after the device reset.
 */
static int touch_set_input_reporting(void)
{
	int retval;
	struct ovt_tcm_hcd *tcm_hcd = touch_hcd->tcm_hcd;

	if (IS_NOT_FW_MODE(tcm_hcd->id_info.mode) ||
			tcm_hcd->app_status != APP_STATUS_OK) {
		LOGN(tcm_hcd->pdev->dev.parent,
				"Identifying mode = 0x%02x\n",
				tcm_hcd->id_info.mode);

		return 0;
	}

	touch_hcd->init_touch_ok = false;

	touch_free_objects();

	mutex_lock(&touch_hcd->report_mutex);

	retval = touch_set_report_config();
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to set report config\n");
		goto exit;
	}

	retval = touch_get_input_params();
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to get input parameters\n");
		goto exit;
	}

	retval = touch_check_input_params();
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to check input parameters\n");
		goto exit;
	} else if (retval == 0) {
		LOGN(tcm_hcd->pdev->dev.parent,
				"Input parameters unchanged\n");
		goto exit;
	}
#if 0
	if (touch_hcd->input_dev != NULL) {
		input_unregister_device(touch_hcd->input_dev);
		touch_hcd->input_dev = NULL;
	}
#endif
	retval = touch_set_input_dev();
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to set up input device\n");
		goto exit;
	}

exit:
	mutex_unlock(&touch_hcd->report_mutex);

	touch_hcd->init_touch_ok = retval < 0 ? false : true;

	return retval;
}


int touch_init(struct ovt_tcm_hcd *tcm_hcd)
{
	int retval;

	touch_hcd = kzalloc(sizeof(*touch_hcd), GFP_KERNEL);
	if (!touch_hcd) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to allocate memory for touch_hcd\n");
		return -ENOMEM;
	}

	touch_hcd->tcm_hcd = tcm_hcd;

	mutex_init(&touch_hcd->report_mutex);

	INIT_BUFFER(touch_hcd->out, false);
	INIT_BUFFER(touch_hcd->resp, false);

	retval = touch_set_input_reporting();
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to set up input reporting\n");
		goto err_set_input_reporting;
	}

	tcm_hcd->report_touch = touch_report;

	return 0;

err_set_input_reporting:
	kfree(touch_hcd->touch_data.object_data);
	kfree(touch_hcd->prev_status);

	RELEASE_BUFFER(touch_hcd->resp);
	RELEASE_BUFFER(touch_hcd->out);

	kfree(touch_hcd);
	touch_hcd = NULL;

	return retval;
}

int touch_remove(struct ovt_tcm_hcd *tcm_hcd)
{
	if (!touch_hcd)
		goto exit;

	tcm_hcd->report_touch = NULL;

	if (touch_hcd->input_dev)
		input_unregister_device(touch_hcd->input_dev);

	kfree(touch_hcd->touch_data.object_data);
	kfree(touch_hcd->prev_status);

	RELEASE_BUFFER(touch_hcd->resp);
	RELEASE_BUFFER(touch_hcd->out);

	kfree(touch_hcd);
	touch_hcd = NULL;

exit:

	return 0;
}

int touch_reinit(struct ovt_tcm_hcd *tcm_hcd)
{
	int retval = 0;

	if (!touch_hcd) {
		retval = touch_init(tcm_hcd);
		return retval;
	}

	touch_free_objects();

	if (IS_NOT_FW_MODE(tcm_hcd->id_info.mode)) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Application mode is not running (firmware mode = %d)\n",
				tcm_hcd->id_info.mode);
		return 0;
	}

	if (!tcm_hcd->in_hdl_mode) {
		retval = tcm_hcd->identify(tcm_hcd, false);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to do identification\n");
			return retval;
		}
	}

	retval = touch_set_input_reporting();
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to set up input reporting\n");
	}

	return retval;
}

int touch_early_suspend(struct ovt_tcm_hcd *tcm_hcd)
{
	if (!touch_hcd)
		return 0;

	if (tcm_hcd->wakeup_gesture_enabled)
		touch_hcd->suspend_touch = false;
	else
		touch_hcd->suspend_touch = true;

	touch_free_objects();

	return 0;
}

int touch_suspend(struct ovt_tcm_hcd *tcm_hcd)
{
	int retval;

	if (!touch_hcd)
		return 0;

	touch_hcd->suspend_touch = true;

	touch_free_objects();

	if (tcm_hcd->wakeup_gesture_enabled) {
		if (!touch_hcd->irq_wake) {
			enable_irq_wake(tcm_hcd->irq);
			touch_hcd->irq_wake = true;
		}

		touch_hcd->suspend_touch = false;

		retval = tcm_hcd->set_dynamic_config(tcm_hcd,
				DC_IN_WAKEUP_GESTURE_MODE,
				1);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to enable wakeup gesture mode\n");
			return retval;
		}
	}

	return 0;
}

int touch_resume(struct ovt_tcm_hcd *tcm_hcd)
{
	int retval;

	if (!touch_hcd)
		return 0;

	touch_hcd->suspend_touch = false;

	if (tcm_hcd->wakeup_gesture_enabled) {
		if (touch_hcd->irq_wake) {
			disable_irq_wake(tcm_hcd->irq);
			touch_hcd->irq_wake = false;
		}

		retval = tcm_hcd->set_dynamic_config(tcm_hcd,
				DC_IN_WAKEUP_GESTURE_MODE,
				0);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to disable wakeup gesture mode\n");
			return retval;
		}
	}

	return 0;
}




