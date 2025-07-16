 /*
 * Copyright (C) 2010 - 2018 Novatek, Inc.
 *
 * $Revision: 64113 $
 * $Date: 2020-06-10 16:39:49 +0800 (Wed, 10 Jun 2020) $
 * To XH 20210129 lsf
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */
#ifndef 	_LINUX_NVT_TOUCH_H
#define		_LINUX_NVT_TOUCH_H

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/uaccess.h>

#include "nt36xxx_mem_map.h"

#define NVT_DEBUG 1

#define NVT_GPIO_AS_INT(pin) tpd_gpio_as_int(pin)
#define NVT_GPIO_OUTPUT(pin, level) tpd_gpio_output(pin, level)

//---INT trigger mode---
//#define EINTF_TRIGGER_RISING     0x00000001
//#define EINTF_TRIGGER_FALLING    0x00000002
#define INT_TRIGGER_TYPE IRQF_TRIGGER_RISING

//---I2C driver info.---
#define NVT_I2C_NAME "NVT-ts"
#define I2C_BLDR_Address 0x01
#define I2C_FW_Address 0x01
#define I2C_HW_Address 0x62

#if NVT_DEBUG
#define NVT_LOG(fmt, args...)    pr_err("[%s] %s %d: " fmt, NVT_I2C_NAME, __func__, __LINE__, ##args)
#else
#define NVT_LOG(fmt, args...)    pr_info("[%s] %s %d: " fmt, NVT_I2C_NAME, __func__, __LINE__, ##args)
#endif
#define NVT_ERR(fmt, args...)    pr_err("[%s] %s %d: " fmt, NVT_I2C_NAME, __func__, __LINE__, ##args)

//---Input device info.---
#define NVT_TS_NAME "NVTCapacitiveTouchScreen"

//---Touch info.---
#define TOUCH_DEFAULT_MAX_WIDTH 1080
#define TOUCH_DEFAULT_MAX_HEIGHT 2460
#define TOUCH_MAX_FINGER_NUM 10
#define TOUCH_KEY_NUM 0
#if TOUCH_KEY_NUM > 0
extern const uint16_t touch_key_array[TOUCH_KEY_NUM];
#endif
#define TOUCH_FORCE_NUM 1000

/* Enable only when module have tp reset pin and connected to host */
#define NVT_TOUCH_SUPPORT_HW_RST 0

//---Customerized func.---
#define NVT_TOUCH_PROC 1
#define NVT_TOUCH_EXT_PROC 1
#define NVT_TOUCH_MP 1
#define MT_PROTOCOL_B 1
//antai <AI_BSP_TP> <luobowen> <2021-02-25> add for tp begin
#define WAKEUP_GESTURE 1
//antai <AI_BSP_TP> <luobowen> <2021-02-25> add for tp end
#if WAKEUP_GESTURE
extern const uint16_t gesture_key_array[];
#endif

//Antaiui <AI_BSP_TP> <yaoyc> <2021-03-19> modify for update tp fw to v05 begin
#define BOOT_UPDATE_FIRMWARE 1
#define BOOT_UPDATE_FIRMWARE_NAME "nt36672_firmware_update.bin"
//Antaiui <AI_BSP_TP> <yaoyc> <2021-03-19> modify for update tp fw to v05 end

//---ESD Protect.---
//Antai <AI_BSP_TP> <luobowen> <2021-02-02> add for tp begin
#define NVT_TOUCH_ESD_PROTECT 1
//Antai <AI_BSP_TP> <luobowen> <2021-02-02> add for tp end
#define NVT_TOUCH_ESD_CHECK_PERIOD 1500	/* ms */

//--I2C DMA info.---
#define I2C_DMA_SUPPORT 1
#define DMA_MAX_TRANSACTION_LENGTH        255   // for DMA mode
#define DMA_MAX_I2C_TRANSFER_SIZE        (DMA_MAX_TRANSACTION_LENGTH - 1)
#define MAX_TRANSACTION_LENGTH            8
#define MAX_I2C_TRANSFER_SIZE            (MAX_TRANSACTION_LENGTH - 1)

struct nvt_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct delayed_work nvt_fwu_work;
	uint16_t addr;
	int8_t phys[32];
	uint8_t fw_ver;
	uint8_t x_num;
	uint8_t y_num;
	uint16_t abs_x_max;
	uint16_t abs_y_max;
	uint8_t max_touch_num;
	uint8_t max_button_num;
	uint32_t int_trigger_type;
	int32_t irq_gpio;
	uint32_t irq_flags;
	int32_t reset_gpio;
	uint32_t reset_flags;
	struct mutex lock;
	const struct nvt_ts_mem_map *mmap;
	uint8_t carrier_system;
	uint16_t nvt_pid;
	uint8_t *xbuf;
	struct mutex xbuf_lock;
	bool irq_enabled;
	spinlock_t irq_lock;
};

#if NVT_TOUCH_PROC
struct nvt_flash_data{
	rwlock_t lock;
	struct i2c_client *client;
};
#endif

typedef enum {
	RESET_STATE_INIT = 0xA0,// IC reset
	RESET_STATE_REK,		// ReK baseline
	RESET_STATE_REK_FINISH,	// baseline is ready
	RESET_STATE_NORMAL_RUN,	// normal run
	RESET_STATE_MAX  = 0xAF
} RST_COMPLETE_STATE;

typedef enum {
    EVENT_MAP_HOST_CMD                      = 0x50,
    EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE   = 0x51,
    EVENT_MAP_RESET_COMPLETE                = 0x60,
    EVENT_MAP_FWINFO                        = 0x78,
    EVENT_MAP_PROJECTID                     = 0x9A,
//Antai <AI_BSP_TP> <hehl> <2021-02-20> add for TP mmi start
    EVENT_MAP_VENDORID                      = 0x4E,
//Antai <AI_BSP_TP> <hehl> <2021-02-20> add for TP mmi end
} I2C_EVENT_MAP;

#define NVT_XBUF_LEN	(1025)

//---extern structures---
extern struct nvt_ts_data *ts;

//---extern functions---
extern int32_t CTP_I2C_READ(struct i2c_client *client, uint16_t address, uint8_t *buf, uint16_t len);
extern int32_t CTP_I2C_WRITE(struct i2c_client *client, uint16_t address, uint8_t *buf, uint16_t len);
extern void nvt_bootloader_reset(void);
extern void nvt_sw_reset_idle(void);
extern int32_t nvt_check_fw_reset_state(RST_COMPLETE_STATE check_reset_state);
extern int32_t nvt_get_fw_info(void);
extern int32_t nvt_clear_fw_status(void);
extern int32_t nvt_check_fw_status(void);
extern int32_t nvt_set_page(uint16_t i2c_addr, uint32_t addr);
#if NVT_TOUCH_ESD_PROTECT
extern void nvt_esd_check_enable(uint8_t enable);
#endif /* #if NVT_TOUCH_ESD_PROTECT */
extern void nvt_stop_crc_reboot(void);
//Antai <AI_BSP_TP> <hehl> <2021-04-12> add gesture start
#ifdef WAKEUP_GESTURE
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

struct tpd_ges_data 
{
	u8 length;
	struct ges_feature_info f_point;
};

#define TST_BIT(flag,bit)	(flag & (0x1 << bit))
#define CLR_BIT(flag,bit)	(flag &= (~(0x1 << bit)))
#define SET_BIT(flag,bit)	(flag |= (0x1 << bit))
enum {
	TYPE_E = 0,
    TYPE_C ,
    TYPE_M,
    TYPE_TRIANGLE,
    TYPE_2F_UP,
    TYPE_2F_DOWN,
    TYPE_W,
    TYPE_Z,
    TYPE_CIRCLE,
    TYPE_S,
    TYPE_V_LEFT,
    TYPE_V_UP ,
    TYPE_V_DOWN ,
    TYPE_2F_LEFT,
    TYPE_2F_RIGHT,
};

//Antai <AI_BSP_TP> <hehl> <2021-07-12> add gesture start
#define AI_GESTURE_U_UP                 KEY_F13
#define AI_GESTURE_U_DOWN               KEY_F14
#define AI_GESTURE_U_LEFT               KEY_F15
#define AI_GESTURE_U_RIGHT              186
#define AI_GESTURE_DOUBLE_TAP           KEY_F17
#define AI_GESTURE_E                    KEY_F18
#define AI_GESTURE_M                    KEY_F19
#define AI_GESTURE_TRIANGLE             190
#define AI_GESTURE_2F_UP                KEY_F21
#define AI_GESTURE_2F_DOWN              KEY_F22
#define AI_GESTURE_W                    KEY_F23
#define AI_GESTURE_Z                    KEY_F24
//Antai <AI_BSP_TP> <penggy> <2021-07-20> add gesture start
#define AI_GESTURE_O                    195
//Antai <AI_BSP_TP> <penggy> <2021-07-20> add gesture end
#define AI_GESTURE_S                    196
#define AI_GESTURE_2F_LEFT              197
#define AI_GESTURE_2F_RIGHT             198
#define AI_GESTURE_1F_SWIPE             199
#define AI_GESTURE_V_DOWN				KEY_F14
#define AI_GESTURE_V_RIGHT				KEY_F15
//Antai <AI_BSP_TP> <hehl> <2021-07-12> add gesture end
	
extern struct tpd_ges_data *nvt_tpd_ges_devp;
extern unsigned int nvt_gesture_cfg;
extern int nvt_wake_switch;
extern int nvt_gesture_switch;
#endif
//Antai <AI_BSP_TP> <hehl> <2021-04-12> add gesture end
#endif /* _LINUX_NVT_TOUCH_H */
