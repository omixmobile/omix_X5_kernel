/*
 * Copyright (C) 2018 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 s5kgm1stmipi_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "s5kgm1stmipiraw_Sensor.h"

#define PFX "S5KGM1ST_camera_sensor"
#define LOG_INF(format,  args...)	pr_info(PFX "[%s] " format,  __func__,  ##args)

static DEFINE_SPINLOCK(imgsensor_drv_lock);
static kal_uint8 chipID;
static kal_uint16 hdr_le, hdr_me, hdr_se;

static struct imgsensor_info_struct imgsensor_info = {
    .sensor_id = S5KGM1ST_SENSOR_ID,
    .checksum_value = 0x6e9db31e,
    .pre = {	// sync with cap
        .pclk = 482000000,
        .linelength = 5024,
        .framelength = 3194,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4000,
        .grabwindow_height = 3000,
        .mipi_data_lp2hs_settle_dc = 85,
        .max_framerate = 300,
        .mipi_pixel_rate = 460800000,//479700000
    },
    .cap = {
        .pclk = 482000000,
        .linelength = 5024,
        .framelength = 3194,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4000,
        .grabwindow_height = 3000,
        .mipi_data_lp2hs_settle_dc = 85,
        .max_framerate = 300,
        .mipi_pixel_rate = 460800000,//479700000
    },
    .normal_video = {
        .pclk = 482000000,
        .linelength = 5024,
        .framelength = 3194,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4000,
        .grabwindow_height = 3000,
        .mipi_data_lp2hs_settle_dc = 85,
        .max_framerate = 300,
        .mipi_pixel_rate = 460800000,//479700000
    },
    .hs_video = {
        .pclk = 482000000,
        .linelength = 2512,
        .framelength = 1600,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 2000,
        .grabwindow_height = 1500,
        .mipi_data_lp2hs_settle_dc = 85,
        .max_framerate = 1200,
        .mipi_pixel_rate = 480000000, //479700000
    },
    .slim_video = {
        .pclk = 482000000,
        .linelength = 2512,
        .framelength = 6388,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 2000,
        .grabwindow_height = 1500,
        .mipi_data_lp2hs_settle_dc = 85,
        .max_framerate = 300,
        .mipi_pixel_rate = 480000000, //479700000
    },

    .margin = 5,
    .min_shutter = 4,
    .max_frame_length = 0xffff,
    .ae_shut_delay_frame = 0,
    .ae_sensor_gain_delay_frame = 0,
    .ae_ispGain_delay_frame = 2,
    // Antaiui <AI_BSP_CAM> <xieht> <2021-03-12> frame time delay frame begin
    .frame_time_delay_frame = 1,
    // Antaiui <AI_BSP_CAM> <xieht> <2021-03-12> frame time delay frame end
    .ihdr_support = 0,	  /*1, support; 0,not support*/
    .ihdr_le_firstline = 0,  /*1,le first; 0, se first*/
    .sensor_mode_num = 5,	  /*support sensor mode num*/

    .cap_delay_frame = 2,/*3 guanjd modify for cts*/
    .pre_delay_frame = 2,/*3 guanjd modify for cts*/
    .video_delay_frame = 3,
    .hs_video_delay_frame = 3,
    .slim_video_delay_frame = 3,

    .isp_driving_current = ISP_DRIVING_4MA,
    .sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
    .mipi_sensor_type = MIPI_OPHY_NCSI2, /*0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2*/
    .mipi_settle_delay_mode = 1, /*0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL*/
    .sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gb,
    .mclk = 24,
    .mipi_lane_num = SENSOR_MIPI_4_LANE,
    .i2c_addr_table = {0x20, 0xff},
    .i2c_speed = 400,
};

static struct imgsensor_struct imgsensor = {
    .mirror = IMAGE_HV_MIRROR,
    .sensor_mode = IMGSENSOR_MODE_INIT,
    .shutter = 0x3D0,
    .gain = 0x100,
    .dummy_pixel = 0,
    .dummy_line = 0,
    .current_fps = 0,
    .autoflicker_en = KAL_FALSE,
    .test_pattern = KAL_FALSE,
    .current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,
    .ihdr_mode = 0,
    .hdr_mode = 0,
    .i2c_write_id = 0x20,
    .current_ae_effective_frame = 1,
};

/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] = {
    { 4000, 3000, 0, 0, 4000, 3000, 4000, 3000, 0, 0, 4000, 3000, 0, 0, 4000, 3000 }, /* preview sync with capture */
    { 4000, 3000, 0, 0, 4000, 3000, 4000, 3000, 0, 0, 4000, 3000, 0, 0, 4000, 3000 }, /* capture */
    { 4000, 3000, 0, 0, 4000, 3000, 4000, 3000, 0, 0, 4000, 3000, 0, 0, 4000, 3000 }, /* normal video sync with capture */
    { 4000, 3000, 0, 0, 4000, 3000, 2000, 1500, 0, 0, 2000, 1500, 0, 0, 2000, 1500 }, /* high  speed video */
    { 4000, 3000, 0, 0, 4000, 3000, 2000, 1500, 0, 0, 2000, 1500, 0, 0, 2000, 1500 }, /* slim video */
}; /*cpy from preview*/

#define PDAF_TEST

#ifdef PDAF_TEST
// Antaiui <AI_BSP_CAM> <xieht> <2021-03-01> pdaf porting begin
static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info =
{
    .i4OffsetX = 16,
    .i4OffsetY = 60,
    .i4PitchX  = 32,
    .i4PitchY  = 32,
    .i4PairNum  = 16,
    .i4SubBlkW  = 8,
    .i4SubBlkH  = 8,
    .i4BlockNumX = 124,
    .i4BlockNumY = 90,
    .i4PosR = {
        {18, 65}, {26, 65}, {34, 65}, {42, 65},
        {22, 69}, {30, 69}, {38, 69}, {46, 69},
        {18, 77}, {26, 77}, {34, 77}, {42, 77},
        {22, 89}, {30, 89}, {38, 89}, {46, 89}
    },
    .i4PosL = {
        {18, 61}, {26, 61}, {34, 61}, {42, 61},
        {22, 73}, {30, 73}, {38, 73}, {46, 73},
        {18, 81}, {26, 81}, {34, 81}, {42, 81},
        {22, 85}, {30, 85}, {38, 85}, {46, 85}
    },
    .iMirrorFlip = 3,
};
#endif

// all vc2 sync with capture
static struct SENSOR_VC_INFO_STRUCT SENSOR_VC_INFO[1] = {
    {
        0x02, 0x0a,  0x00,  0x08, 0x40, 0x00,
        0x00, 0x2b, 0x7E0, 0x5E4, 0x01, 0x00, 0x0000, 0x0000,
        0x01, 0x30, 0x26C, 0x2D0, 0x03, 0x00, 0x0000, 0x0000
    },
};
// Antaiui <AI_BSP_CAM> <xieht> <2021-03-01> pdaf porting end

static kal_uint16 read_cmos_sensor_16_16(kal_uint32 addr)
{
    kal_uint16 get_byte = 0;
    char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
    /*kdSetI2CSpeed(imgsensor_info.i2c_speed); Add this func to set i2c speed by each sensor*/
    iReadRegI2C(pusendcmd, 2, (u8 *)&get_byte, 2, imgsensor.i2c_write_id);
    return ((get_byte<<8)&0xff00)|((get_byte>>8)&0x00ff);
}

static void write_cmos_sensor_16_16(kal_uint16 addr, kal_uint16 para)
{
    char pusendcmd[4] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para >> 8), (char)(para & 0xFF)};
    /* kdSetI2CSpeed(imgsensor_info.i2c_speed); Add this func to set i2c speed by each sensor*/
    iWriteRegI2C(pusendcmd, 4, imgsensor.i2c_write_id);
}

static kal_uint16 read_cmos_sensor_16_8(kal_uint16 addr)
{
    kal_uint16 get_byte = 0;
    char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
    /*kdSetI2CSpeed(imgsensor_info.i2c_speed);  Add this func to set i2c speed by each sensor*/
    iReadRegI2C(pusendcmd, 2, (u8 *)&get_byte, 1, imgsensor.i2c_write_id);
    return get_byte;
}

static void write_cmos_sensor_16_8(kal_uint16 addr, kal_uint8 para)
{
    char pusendcmd[4] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
    /* kdSetI2CSpeed(imgsensor_info.i2c_speed);Add this func to set i2c speed by each sensor*/
    iWriteRegI2C(pusendcmd, 3, imgsensor.i2c_write_id);
}

#define EEPROM_SLAVE_ADDRES 0xA0
/*
static kal_uint16 read_eeprom_16_8(kal_uint16 addr)
{
    kal_uint16 get_byte= 0;
    char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    //kdSetI2CSpeed(imgsensor_info.i2c_speed);  Add this func to set i2c speed by each sensor
    iReadRegI2C(pusendcmd , 2, (u8*)&get_byte,1,EEPROM_SLAVE_ADDRES);
    return get_byte;
}
*/
#define MULTI_WRITE 1

#if MULTI_WRITE
#define I2C_BUFFER_LEN 225	/* trans# max is 255, each 3 bytes */
#else
#define I2C_BUFFER_LEN 4
#endif

static kal_uint16 table_write_cmos_sensor(kal_uint16 *para, kal_uint32 len)
{
    char puSendCmd[I2C_BUFFER_LEN];
    kal_uint32 tosend, IDX;
    kal_uint16 addr = 0, addr_last = 0, data;

    tosend = 0;
    IDX = 0;
    while (len > IDX) {
        addr = para[IDX];

        {
            puSendCmd[tosend++] = (char)(addr >> 8);
            puSendCmd[tosend++] = (char)(addr & 0xFF);
            data = para[IDX + 1];
            puSendCmd[tosend++] = (char)(data >> 8);
            puSendCmd[tosend++] = (char)(data & 0xFF);
            IDX += 2;
            addr_last = addr;

        }
#if MULTI_WRITE
        /* Write when remain buffer size is less than 4 bytes or reach end of data */
        if ((I2C_BUFFER_LEN - tosend) < 4 || IDX == len || addr != addr_last) {
            iBurstWriteReg_multi(puSendCmd, tosend, imgsensor.i2c_write_id,
                    4, imgsensor_info.i2c_speed);
            tosend = 0;
        }
#else
        iWriteRegI2CTiming(puSendCmd, 4, imgsensor.i2c_write_id, imgsensor_info.i2c_speed);
        tosend = 0;

#endif
    }
    return 0;
}

// Antaiui <AI_BSP_CAM> <xieht> <2021-06-04> add check streaming off begin
static void check_output_stream_off(void) {
    int timeout = (10000 / imgsensor.current_fps) + 1;
    int i = 0;
    int framecnt = 0;
    for (i = 0; i < timeout; i++) {
        mdelay(5);
        framecnt = read_cmos_sensor_16_8(0x0005);
        if (framecnt == 0xFF) {
            LOG_INF("Stream Off OK at i = %d.\n", i);
            return;
        }
    }
    LOG_INF("Stream Off Fail! framecnt = %d.\n", framecnt);
}   /*  check_output_stream_off  */

static kal_uint32 streaming_control(kal_bool enable)
{
    LOG_INF("streaming_enable(0= Sw Standby,1= streaming): %d\n", enable);
    if (enable) {
        write_cmos_sensor_16_8(0x0100, 0x01);
        mdelay(10);
    } else {
        write_cmos_sensor_16_8(0x0100, 0x00);
        check_output_stream_off();
    }
    return ERROR_NONE;
}
// Antaiui <AI_BSP_CAM> <xieht> <2021-06-04> add check streaming off end

static void set_dummy(void)
{
    LOG_INF("dummyline = %d, dummypixels = %d\n", imgsensor.dummy_line, imgsensor.dummy_pixel);
    write_cmos_sensor_16_16(0x0340, imgsensor.frame_length);
    write_cmos_sensor_16_16(0x0342, imgsensor.line_length);
}   /*  set_dummy  */

static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{

    kal_uint32 frame_length = imgsensor.frame_length;

    LOG_INF("framerate = %d, min framelength should enable %d\n", framerate, min_framelength_en);

    frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
    spin_lock(&imgsensor_drv_lock);
    if (frame_length >= imgsensor.min_frame_length)
        imgsensor.frame_length = frame_length;
    else
        imgsensor.frame_length = imgsensor.min_frame_length;
    imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;

    if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
        imgsensor.frame_length = imgsensor_info.max_frame_length;
        imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
    }
    if (min_framelength_en)
        imgsensor.min_frame_length = imgsensor.frame_length;
    spin_unlock(&imgsensor_drv_lock);
    set_dummy();
}   /*  set_max_framerate  */

// Antaiui <AI_BSP_CAM> <xieht> <2021-03-12> modify for long exposure begin
static bool bNeedSetNormalMode = KAL_FALSE;
#define ORI_TIME    95932

static void write_shutter(kal_uint32 shutter)
{
    kal_uint16 realtime_fps = 0;
    int framecnt = 0;

    spin_lock(&imgsensor_drv_lock);
    if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
        imgsensor.frame_length = shutter + imgsensor_info.margin;
    else
        imgsensor.frame_length = imgsensor.min_frame_length;
    if (imgsensor.frame_length > imgsensor_info.max_frame_length)
        imgsensor.frame_length = imgsensor_info.max_frame_length;
    spin_unlock(&imgsensor_drv_lock);
    if (shutter < imgsensor_info.min_shutter)
        shutter = imgsensor_info.min_shutter;

    if (imgsensor.autoflicker_en) {
        realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
        if (realtime_fps >= 297 && realtime_fps <= 305) {
            set_max_framerate(296, 0);
        } else if (realtime_fps >= 147 && realtime_fps <= 150) {
            set_max_framerate(146, 0);
        } else {
            /* Extend frame length*/
            write_cmos_sensor_16_16(0x0340, imgsensor.frame_length);
        }
    } else {
        /* Extend frame length*/
        write_cmos_sensor_16_16(0x0340, imgsensor.frame_length);
    }

    if (shutter > 65530) {
        kal_uint16 exposure_time;
        kal_uint16 long_shutter;
        kal_uint16 new_framelength;

        LOG_INF("enter long exposure mode!");
        bNeedSetNormalMode = KAL_TRUE;

        LOG_INF("long exposure mode: shutter(%d), linelength(%d)", shutter, imgsensor.line_length);
        exposure_time = (shutter + 64) * 157 / 15062500; // shutter * imgsensor.line_length / imgsensor.pclk;
        long_shutter = (shutter + 64) / 64; // exposure_time * 0x05DB;
        new_framelength = long_shutter + 4;
        LOG_INF("long exposure mode: exposure_time(%f), shutter(%#x), framelength(%#x)", exposure_time, long_shutter, new_framelength);

        write_cmos_sensor_16_16(0x6028, 0x4000);
        write_cmos_sensor_16_8(0x0100, 0x00);
        while (1) {
            mdelay(5);
            framecnt = read_cmos_sensor_16_8(0x0005);
            LOG_INF("Stream Off oning at framecnt = 0x%x.\n", framecnt);
            if ( framecnt == 0xFF) {
                LOG_INF("Stream Off OK at framecnt = 0x%x.\n", framecnt);
                break;
            }
        }
        write_cmos_sensor_16_16(0x0340, new_framelength);
        write_cmos_sensor_16_16(0x0202, long_shutter);
        write_cmos_sensor_16_16(0x0702, 0x0600);
        write_cmos_sensor_16_16(0x0704, 0x0600);
        write_cmos_sensor_16_8(0x0100, 0x01);
        /* Frame exposure mode customization for LE */
        imgsensor.ae_frm_mode.frame_mode_1 = IMGSENSOR_AE_MODE_SE;
        imgsensor.ae_frm_mode.frame_mode_2 = IMGSENSOR_AE_MODE_SE;
        imgsensor.current_ae_effective_frame = 1;
        LOG_INF("exit long exposure mode!");
    } else {
        imgsensor.current_ae_effective_frame = 1;
        if (bNeedSetNormalMode) {
            LOG_INF("long exposure mode to normal mode!");
            write_cmos_sensor_16_16(0x6028, 0x4000);
            write_cmos_sensor_16_8(0x0100, 0x00);
            while (1) {
                mdelay(5);
                framecnt = read_cmos_sensor_16_8(0x0005);
                LOG_INF("Stream Off oning at framecnt = 0x%x.\n", framecnt);
                if ( framecnt == 0xFF) {
                    LOG_INF("Stream Off OK at framecnt = 0x%x.\n", framecnt);
                    break;
                }
            }
            write_cmos_sensor_16_16(0x0340, imgsensor.frame_length);
            write_cmos_sensor_16_16(0x0202, 0x0100);
            write_cmos_sensor_16_16(0x0702, 0x0000);
            write_cmos_sensor_16_16(0x0704, 0x0000);
            write_cmos_sensor_16_8(0x0100, 0x01);
            bNeedSetNormalMode = KAL_FALSE;
        }
        write_cmos_sensor_16_16(0x0202, shutter);
        LOG_INF("shutter: %d, framelength: %d", shutter, imgsensor.frame_length);
    }
}
// Antaiui <AI_BSP_CAM> <xieht> <2021-03-12> modify for long exposure end

/*************************************************************************
 * FUNCTION
 *	set_shutter
 *
 * DESCRIPTION
 *	This function set e-shutter of sensor to change exposure time.
 *
 * PARAMETERS
 *	iShutter : exposured lines
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static void set_shutter(kal_uint32 shutter)
{
    unsigned long flags;

    spin_lock_irqsave(&imgsensor_drv_lock, flags);
    imgsensor.shutter = shutter;
    spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

    write_shutter(shutter);
}   /*  set_shutter  */

// Antaiui <AI_BSP_CAM> <xieht> <2021-03-12> modify for long exposure begin
/*	write_shutter  */
static void set_shutter_frame_length(kal_uint16 shutter, kal_uint16 frame_length)
{
    unsigned long flags;
    kal_int32 dummy_line = 0;

    spin_lock_irqsave(&imgsensor_drv_lock, flags);
    imgsensor.shutter = shutter;
    spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

    spin_lock(&imgsensor_drv_lock);
    /* Change frame time */
    if (frame_length > 1)
        dummy_line = frame_length - imgsensor.frame_length;
    imgsensor.frame_length = imgsensor.frame_length + dummy_line;

    /*  */
    if (shutter > imgsensor.frame_length - imgsensor_info.margin)
        imgsensor.frame_length = shutter + imgsensor_info.margin;

    if (imgsensor.frame_length > imgsensor_info.max_frame_length)
        imgsensor.frame_length = imgsensor_info.max_frame_length;

    spin_unlock(&imgsensor_drv_lock);
    shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
    shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin))
        ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

    /* Extend frame length */
    write_cmos_sensor_16_16(0x0340, imgsensor.frame_length & 0xFFFF);

    /* Update Shutter */
    write_cmos_sensor_16_16(0X0202, shutter & 0xFFFF);

    LOG_INF("shutter = %d, framelength = %d/%d, dummy_line= %d\n", shutter, imgsensor.frame_length,
            frame_length, dummy_line);

}  /* write_shutter */
// Antaiui <AI_BSP_CAM> <xieht> <2021-03-12> modify for long exposure end


static kal_uint16 gain2reg(const kal_uint16 gain)
{
    kal_uint16 reg_gain = 0x0;

    reg_gain = gain/2;
    return (kal_uint16)reg_gain;
}

/*************************************************************************
 * FUNCTION
 *	set_gain
 *
 * DESCRIPTION
 *	This function is to set global gain to sensor.
 *
 * PARAMETERS
 *	iGain : sensor global gain(base: 0x40)
 *
 * RETURNS
 *	the actually gain set to sensor.
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
    kal_uint16 reg_gain;

    /*gain= 1024;for test*/
    /*return; for test*/

    if (gain < BASEGAIN || gain > 32 * BASEGAIN) {
        LOG_INF("Error gain setting");

        if (gain < BASEGAIN)
            gain = BASEGAIN;
        else if (gain > 32 * BASEGAIN)
            gain = 32 * BASEGAIN;
    }

    reg_gain = gain2reg(gain);
    spin_lock(&imgsensor_drv_lock);
    imgsensor.gain = reg_gain;
    spin_unlock(&imgsensor_drv_lock);
    LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

    write_cmos_sensor_16_16(0x0204, reg_gain);
    /*write_cmos_sensor_16_8(0x0204,(reg_gain>>8));*/
    /*write_cmos_sensor_16_8(0x0205,(reg_gain&0xff));*/

    return gain;
}   /*  set_gain  */

static void set_mirror_flip(kal_uint8 image_mirror)
{
    switch (image_mirror) {
    case IMAGE_NORMAL:
        write_cmos_sensor_16_8(0x0101, 0x00);   /* Gr*/
        break;
    case IMAGE_H_MIRROR:
        write_cmos_sensor_16_8(0x0101, 0x01);
        break;
    case IMAGE_V_MIRROR:
        write_cmos_sensor_16_8(0x0101, 0x02);
        break;
    case IMAGE_HV_MIRROR:
        write_cmos_sensor_16_8(0x0101, 0x03);/*Gb*/
        break;
    default:
        LOG_INF("Error image_mirror setting\n");
    }
}

/*************************************************************************
 * FUNCTION
 *	night_mode
 *
 * DESCRIPTION
 *	This function night mode of sensor.
 *
 * PARAMETERS
 *	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
#if 0
static void night_mode(kal_bool enable)
{
    /*No Need to implement this function*/
}	/*	night_mode	*/
#endif

static kal_uint16 addr_data_pair_init_new[] = {
    0x6028, 0x2000,
    0x602A, 0x3F5C,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x00F0,
    0x6F12, 0x9EBB,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x2DE9,
    0x6F12, 0xFF5F,
    0x6F12, 0xFF48,
    0x6F12, 0x8B46,
    0x6F12, 0x1746,
    0x6F12, 0x0068,
    0x6F12, 0x9A46,
    0x6F12, 0x4FEA,
    0x6F12, 0x1049,
    0x6F12, 0x80B2,
    0x6F12, 0x8046,
    0x6F12, 0x0146,
    0x6F12, 0x0022,
    0x6F12, 0x4846,
    0x6F12, 0x00F0,
    0x6F12, 0x02FB,
    0x6F12, 0xF94D,
    0x6F12, 0x95F8,
    0x6F12, 0x6D00,
    0x6F12, 0x0228,
    0x6F12, 0x35D0,
    0x6F12, 0x0224,
    0x6F12, 0xF74E,
    0x6F12, 0x5346,
    0x6F12, 0xB6F8,
    0x6F12, 0xB802,
    0x6F12, 0xB0FB,
    0x6F12, 0xF4F0,
    0x6F12, 0xA6F8,
    0x6F12, 0xB802,
    0x6F12, 0xD5F8,
    0x6F12, 0x1411,
    0x6F12, 0x06F5,
    0x6F12, 0x2E76,
    0x6F12, 0x6143,
    0x6F12, 0xC5F8,
    0x6F12, 0x1411,
    0x6F12, 0xB5F8,
    0x6F12, 0x8C11,
    0x6F12, 0x411A,
    0x6F12, 0x89B2,
    0x6F12, 0x25F8,
    0x6F12, 0x981B,
    0x6F12, 0x35F8,
    0x6F12, 0x142C,
    0x6F12, 0x6243,
    0x6F12, 0x521E,
    0x6F12, 0x00FB,
    0x6F12, 0x0210,
    0x6F12, 0xB5F8,
    0x6F12, 0xF210,
    0x6F12, 0x07FB,
    0x6F12, 0x04F2,
    0x6F12, 0x0844,
    0x6F12, 0xC5F8,
    0x6F12, 0xF800,
    0x6F12, 0x5946,
    0x6F12, 0x0098,
    0x6F12, 0x00F0,
    0x6F12, 0xDBFA,
    0x6F12, 0x3088,
    0x6F12, 0x4146,
    0x6F12, 0x6043,
    0x6F12, 0x3080,
    0x6F12, 0xE86F,
    0x6F12, 0x0122,
    0x6F12, 0xB0FB,
    0x6F12, 0xF4F0,
    0x6F12, 0xE867,
    0x6F12, 0x04B0,
    0x6F12, 0x4846,
    0x6F12, 0xBDE8,
    0x6F12, 0xF05F,
    0x6F12, 0x00F0,
    0x6F12, 0xC7BA,
    0x6F12, 0x0124,
    0x6F12, 0xC8E7,
    0x6F12, 0x2DE9,
    0x6F12, 0xF041,
    0x6F12, 0x8046,
    0x6F12, 0xD848,
    0x6F12, 0x0022,
    0x6F12, 0x4168,
    0x6F12, 0x0D0C,
    0x6F12, 0x8EB2,
    0x6F12, 0x3146,
    0x6F12, 0x2846,
    0x6F12, 0x00F0,
    0x6F12, 0xB9FA,
    0x6F12, 0xD74C,
    0x6F12, 0xD54F,
    0x6F12, 0x2078,
    0x6F12, 0x97F8,
    0x6F12, 0x8B12,
    0x6F12, 0x10FB,
    0x6F12, 0x01F0,
    0x6F12, 0x2070,
    0x6F12, 0x4046,
    0x6F12, 0x00F0,
    0x6F12, 0xB8FA,
    0x6F12, 0x2078,
    0x6F12, 0x97F8,
    0x6F12, 0x8B12,
    0x6F12, 0x0122,
    0x6F12, 0xB0FB,
    0x6F12, 0xF1F0,
    0x6F12, 0x2070,
    0x6F12, 0x3146,
    0x6F12, 0x2846,
    0x6F12, 0xBDE8,
    0x6F12, 0xF041,
    0x6F12, 0x00F0,
    0x6F12, 0xA1BA,
    0x6F12, 0x2DE9,
    0x6F12, 0xFF47,
    0x6F12, 0x8146,
    0x6F12, 0xC648,
    0x6F12, 0x1746,
    0x6F12, 0x8846,
    0x6F12, 0x8068,
    0x6F12, 0x1C46,
    0x6F12, 0x85B2,
    0x6F12, 0x060C,
    0x6F12, 0x0022,
    0x6F12, 0x2946,
    0x6F12, 0x3046,
    0x6F12, 0x00F0,
    0x6F12, 0x92FA,
    0x6F12, 0x2346,
    0x6F12, 0x3A46,
    0x6F12, 0x4146,
    0x6F12, 0x4846,
    0x6F12, 0x00F0,
    0x6F12, 0x9BFA,
    0x6F12, 0xC14A,
    0x6F12, 0x9088,
    0x6F12, 0xF0B3,
    0x6F12, 0xBE48,
    0x6F12, 0x90F8,
    0x6F12, 0xBA10,
    0x6F12, 0xD1B3,
    0x6F12, 0xD0F8,
    0x6F12, 0x2801,
    0x6F12, 0x1168,
    0x6F12, 0x8842,
    0x6F12, 0x00D3,
    0x6F12, 0x0846,
    0x6F12, 0x010A,
    0x6F12, 0xB1FA,
    0x6F12, 0x81F0,
    0x6F12, 0xC0F1,
    0x6F12, 0x1700,
    0x6F12, 0xC140,
    0x6F12, 0x02EB,
    0x6F12, 0x4000,
    0x6F12, 0xC9B2,
    0x6F12, 0x0389,
    0x6F12, 0xC288,
    0x6F12, 0x9B1A,
    0x6F12, 0x4B43,
    0x6F12, 0x8033,
    0x6F12, 0x02EB,
    0x6F12, 0x2322,
    0x6F12, 0x0092,
    0x6F12, 0x438A,
    0x6F12, 0x028A,
    0x6F12, 0x9B1A,
    0x6F12, 0x4B43,
    0x6F12, 0x8033,
    0x6F12, 0x02EB,
    0x6F12, 0x2322,
    0x6F12, 0x0192,
    0x6F12, 0x838B,
    0x6F12, 0x428B,
    0x6F12, 0x9B1A,
    0x6F12, 0x4B43,
    0x6F12, 0x8033,
    0x6F12, 0x02EB,
    0x6F12, 0x2322,
    0x6F12, 0x0292,
    0x6F12, 0xC28C,
    0x6F12, 0x808C,
    0x6F12, 0x121A,
    0x6F12, 0x4A43,
    0x6F12, 0x8032,
    0x6F12, 0x00EB,
    0x6F12, 0x2220,
    0x6F12, 0x0390,
    0x6F12, 0x0022,
    0x6F12, 0x6846,
    0x6F12, 0x54F8,
    0x6F12, 0x2210,
    0x6F12, 0x50F8,
    0x6F12, 0x2230,
    0x6F12, 0x5943,
    0x6F12, 0x090B,
    0x6F12, 0x44F8,
    0x6F12, 0x2210,
    0x6F12, 0x521C,
    0x6F12, 0x00E0,
    0x6F12, 0x01E0,
    0x6F12, 0x042A,
    0x6F12, 0xF2D3,
    0x6F12, 0x04B0,
    0x6F12, 0x2946,
    0x6F12, 0x3046,
    0x6F12, 0xBDE8,
    0x6F12, 0xF047,
    0x6F12, 0x0122,
    0x6F12, 0x00F0,
    0x6F12, 0x3FBA,
    0x6F12, 0x2DE9,
    0x6F12, 0xF041,
    0x6F12, 0x4FF4,
    0x6F12, 0x7A71,
    0x6F12, 0xB0FB,
    0x6F12, 0xF1F2,
    0x6F12, 0xB0FB,
    0x6F12, 0xF1F5,
    0x6F12, 0x01FB,
    0x6F12, 0x1207,
    0x6F12, 0x0024,
    0x6F12, 0x934E,
    0x6F12, 0x06E0,
    0x6F12, 0x48F2,
    0x6F12, 0xE801,
    0x6F12, 0x4843,
    0x6F12, 0x400B,
    0x6F12, 0x00F0,
    0x6F12, 0x40FA,
    0x6F12, 0x641C,
    0x6F12, 0x706B,
    0x6F12, 0xAC42,
    0x6F12, 0x4FEA,
    0x6F12, 0x9000,
    0x6F12, 0xF3D3,
    0x6F12, 0x7843,
    0x6F12, 0xBDE8,
    0x6F12, 0xF041,
    0x6F12, 0x00EB,
    0x6F12, 0x4010,
    0x6F12, 0x400B,
    0x6F12, 0x00F0,
    0x6F12, 0x32BA,
    0x6F12, 0x70B5,
    0x6F12, 0x0024,
    0x6F12, 0x8A4D,
    0x6F12, 0x0CE0,
    0x6F12, 0xA0F5,
    0x6F12, 0x7F42,
    0x6F12, 0xFE3A,
    0x6F12, 0x13D0,
    0x6F12, 0x521E,
    0x6F12, 0x14D0,
    0x6F12, 0x91F8,
    0x6F12, 0x0E11,
    0x6F12, 0x00F0,
    0x6F12, 0x29FA,
    0x6F12, 0x641C,
    0x6F12, 0x142C,
    0x6F12, 0x05D2,
    0x6F12, 0x05EB,
    0x6F12, 0x8401,
    0x6F12, 0xB1F8,
    0x6F12, 0x0C01,
    0x6F12, 0x0028,
    0x6F12, 0xECD1,
    0x6F12, 0x7D49,
    0x6F12, 0x0420,
    0x6F12, 0xA1F8,
    0x6F12, 0xCA06,
    0x6F12, 0x70BD,
    0x6F12, 0x91F8,
    0x6F12, 0x0E01,
    0x6F12, 0x05E0,
    0x6F12, 0x91F8,
    0x6F12, 0x0E01,
    0x6F12, 0x4FF4,
    0x6F12, 0x7A71,
    0x6F12, 0x10FB,
    0x6F12, 0x01F0,
    0x6F12, 0x00F0,
    0x6F12, 0x15FA,
    0x6F12, 0xE5E7,
    0x6F12, 0x70B5,
    0x6F12, 0x0024,
    0x6F12, 0x764D,
    0x6F12, 0x0CE0,
    0x6F12, 0xA0F5,
    0x6F12, 0x7F42,
    0x6F12, 0xFE3A,
    0x6F12, 0x13D0,
    0x6F12, 0x521E,
    0x6F12, 0x14D0,
    0x6F12, 0x91F8,
    0x6F12, 0x5E11,
    0x6F12, 0x00F0,
    0x6F12, 0x01FA,
    0x6F12, 0x641C,
    0x6F12, 0x142C,
    0x6F12, 0x05D2,
    0x6F12, 0x05EB,
    0x6F12, 0x8401,
    0x6F12, 0xB1F8,
    0x6F12, 0x5C01,
    0x6F12, 0x0028,
    0x6F12, 0xECD1,
    0x6F12, 0x6949,
    0x6F12, 0x0220,
    0x6F12, 0xA1F8,
    0x6F12, 0xCA06,
    0x6F12, 0x70BD,
    0x6F12, 0x91F8,
    0x6F12, 0x5E01,
    0x6F12, 0x05E0,
    0x6F12, 0x91F8,
    0x6F12, 0x5E01,
    0x6F12, 0x4FF4,
    0x6F12, 0x7A71,
    0x6F12, 0x10FB,
    0x6F12, 0x01F0,
    0x6F12, 0x00F0,
    0x6F12, 0xEDF9,
    0x6F12, 0xE5E7,
    0x6F12, 0xF8B5,
    0x6F12, 0x604D,
    0x6F12, 0xB5F8,
    0x6F12, 0xCA06,
    0x6F12, 0x0328,
    0x6F12, 0x5BD1,
    0x6F12, 0x5F4E,
    0x6F12, 0x96F8,
    0x6F12, 0x3800,
    0x6F12, 0x90B1,
    0x6F12, 0x96F8,
    0x6F12, 0x3900,
    0x6F12, 0x0A28,
    0x6F12, 0x0ED8,
    0x6F12, 0x0024,
    0x6F12, 0x08E0,
    0x6F12, 0x06EB,
    0x6F12, 0x4400,
    0x6F12, 0x3219,
    0x6F12, 0x408F,
    0x6F12, 0x0121,
    0x6F12, 0x4E32,
    0x6F12, 0x00F0,
    0x6F12, 0xD9F9,
    0x6F12, 0x641C,
    0x6F12, 0x96F8,
    0x6F12, 0x3900,
    0x6F12, 0xA042,
    0x6F12, 0xF2D8,
    0x6F12, 0xD5F8,
    0x6F12, 0xDC06,
    0x6F12, 0x8047,
    0x6F12, 0x96F8,
    0x6F12, 0x2E00,
    0x6F12, 0x4FF4,
    0x6F12, 0x7A71,
    0x6F12, 0x10FB,
    0x6F12, 0x01F0,
    0x6F12, 0x00F0,
    0x6F12, 0xC4F9,
    0x6F12, 0x0220,
    0x6F12, 0x00F0,
    0x6F12, 0xCBF9,
    0x6F12, 0x0120,
    0x6F12, 0x00F0,
    0x6F12, 0xCDF9,
    0x6F12, 0x4D49,
    0x6F12, 0x0020,
    0x6F12, 0x4883,
    0x6F12, 0x4B4C,
    0x6F12, 0x94F8,
    0x6F12, 0xFB20,
    0x6F12, 0x4A83,
    0x6F12, 0x95F8,
    0x6F12, 0xAC10,
    0x6F12, 0xB1B1,
    0x6F12, 0x85F8,
    0x6F12, 0x4807,
    0x6F12, 0x00F0,
    0x6F12, 0xC4F9,
    0x6F12, 0x0646,
    0x6F12, 0x3046,
    0x6F12, 0x00F0,
    0x6F12, 0xC5F9,
    0x6F12, 0xD4F8,
    0x6F12, 0x6412,
    0x6F12, 0x8142,
    0x6F12, 0x01D2,
    0x6F12, 0x0121,
    0x6F12, 0x00E0,
    0x6F12, 0x0021,
    0x6F12, 0x95F8,
    0x6F12, 0x4807,
    0x6F12, 0x8DF8,
    0x6F12, 0x0000,
    0x6F12, 0x9DF8,
    0x6F12, 0x0000,
    0x6F12, 0x0843,
    0x6F12, 0xEDD0,
    0x6F12, 0x95F8,
    0x6F12, 0x9806,
    0x6F12, 0x0028,
    0x6F12, 0x0ED0,
    0x6F12, 0x0122,
    0x6F12, 0x1402,
    0x6F12, 0x48F6,
    0x6F12, 0xF825,
    0x6F12, 0x2146,
    0x6F12, 0x2846,
    0x6F12, 0x00F0,
    0x6F12, 0xAFF9,
    0x6F12, 0x2146,
    0x6F12, 0x2846,
    0x6F12, 0xBDE8,
    0x6F12, 0xF840,
    0x6F12, 0x0022,
    0x6F12, 0x00F0,
    0x6F12, 0xA8B9,
    0x6F12, 0xF8BD,
    0x6F12, 0x2DE9,
    0x6F12, 0xF041,
    0x6F12, 0x344C,
    0x6F12, 0x3249,
    0x6F12, 0x0646,
    0x6F12, 0x94F8,
    0x6F12, 0x6970,
    0x6F12, 0x8988,
    0x6F12, 0x94F8,
    0x6F12, 0x8120,
    0x6F12, 0x0020,
    0x6F12, 0xC1B1,
    0x6F12, 0x2146,
    0x6F12, 0xD1F8,
    0x6F12, 0x9410,
    0x6F12, 0x72B1,
    0x6F12, 0x8FB1,
    0x6F12, 0x0846,
    0x6F12, 0x00F0,
    0x6F12, 0x98F9,
    0x6F12, 0x0546,
    0x6F12, 0xE06F,
    0x6F12, 0x00F0,
    0x6F12, 0x94F9,
    0x6F12, 0x8542,
    0x6F12, 0x02D2,
    0x6F12, 0xD4F8,
    0x6F12, 0x9400,
    0x6F12, 0x26E0,
    0x6F12, 0xE06F,
    0x6F12, 0x24E0,
    0x6F12, 0x002F,
    0x6F12, 0xFBD1,
    0x6F12, 0x002A,
    0x6F12, 0x24D0,
    0x6F12, 0x0846,
    0x6F12, 0x1EE0,
    0x6F12, 0x1E49,
    0x6F12, 0x0D8E,
    0x6F12, 0x496B,
    0x6F12, 0x4B42,
    0x6F12, 0x77B1,
    0x6F12, 0x2048,
    0x6F12, 0x806F,
    0x6F12, 0x10E0,
    0x6F12, 0x4242,
    0x6F12, 0x00E0,
    0x6F12, 0x0246,
    0x6F12, 0x0029,
    0x6F12, 0x0FDB,
    0x6F12, 0x8A42,
    0x6F12, 0x0FDD,
    0x6F12, 0x3046,
    0x6F12, 0xBDE8,
    0x6F12, 0xF041,
    0x6F12, 0x00F0,
    0x6F12, 0x78B9,
    0x6F12, 0x002A,
    0x6F12, 0x0CD0,
    0x6F12, 0x1748,
    0x6F12, 0xD0F8,
    0x6F12, 0x8C00,
    0x6F12, 0x25B1,
    0x6F12, 0x0028,
    0x6F12, 0xEDDA,
    0x6F12, 0xEAE7,
    0x6F12, 0x1946,
    0x6F12, 0xEDE7,
    0x6F12, 0x00F0,
    0x6F12, 0x70F9,
    0x6F12, 0xE060,
    0x6F12, 0x0120,
    0x6F12, 0xBDE8,
    0x6F12, 0xF081,
    0x6F12, 0x2DE9,
    0x6F12, 0xF35F,
    0x6F12, 0xDFF8,
    0x6F12, 0x24A0,
    0x6F12, 0x0C46,
    0x6F12, 0xBAF8,
    0x6F12, 0xBE04,
    0x6F12, 0x08B1,
    0x6F12, 0x00F0,
    0x6F12, 0x67F9,
    0x6F12, 0x0B4E,
    0x6F12, 0x3088,
    0x6F12, 0x0128,
    0x6F12, 0x19D1,
    0x6F12, 0x002C,
    0x6F12, 0x17D1,
    0x6F12, 0x11E0,
    0x6F12, 0x2000,
    0x6F12, 0x4690,
    0x6F12, 0x2000,
    0x6F12, 0x2C30,
    0x6F12, 0x2000,
    0x6F12, 0x2E30,
    0x6F12, 0x2000,
    0x6F12, 0x2580,
    0x6F12, 0x2000,
    0x6F12, 0x6000,
    0x6F12, 0x2000,
    0x6F12, 0x0DE0,
    0x6F12, 0x4000,
    0x6F12, 0x7000,
    0x6F12, 0x2000,
    0x6F12, 0x2BA0,
    0x6F12, 0x2000,
    0x6F12, 0x3600,
    0x6F12, 0x6F4D,
    0x6F12, 0x2889,
    0x6F12, 0x18B1,
    0x6F12, 0x401E,
    0x6F12, 0x2881,
    0x6F12, 0xBDE8,
    0x6F12, 0xFC9F,
    0x6F12, 0xDFF8,
    0x6F12, 0xB491,
    0x6F12, 0xD9F8,
    0x6F12, 0x0000,
    0x6F12, 0xB0F8,
    0x6F12, 0xD602,
    0x6F12, 0x38B1,
    0x6F12, 0x3089,
    0x6F12, 0x401C,
    0x6F12, 0x80B2,
    0x6F12, 0x3081,
    0x6F12, 0xFF28,
    0x6F12, 0x01D9,
    0x6F12, 0xE889,
    0x6F12, 0x3081,
    0x6F12, 0x6648,
    0x6F12, 0x4FF0,
    0x6F12, 0x0008,
    0x6F12, 0xC6F8,
    0x6F12, 0x0C80,
    0x6F12, 0xB0F8,
    0x6F12, 0x5EB0,
    0x6F12, 0x40F2,
    0x6F12, 0xFF31,
    0x6F12, 0x0B20,
    0x6F12, 0x00F0,
    0x6F12, 0x31F9,
    0x6F12, 0xD9F8,
    0x6F12, 0x0000,
    0x6F12, 0x0027,
    0x6F12, 0x3C46,
    0x6F12, 0xB0F8,
    0x6F12, 0xD412,
    0x6F12, 0x21B1,
    0x6F12, 0x0098,
    0x6F12, 0x00F0,
    0x6F12, 0x18F9,
    0x6F12, 0x0746,
    0x6F12, 0x0BE0,
    0x6F12, 0xB0F8,
    0x6F12, 0xD602,
    0x6F12, 0x40B1,
    0x6F12, 0x3089,
    0x6F12, 0xE989,
    0x6F12, 0x8842,
    0x6F12, 0x04D3,
    0x6F12, 0x0098,
    0x6F12, 0xFFF7,
    0x6F12, 0x5BFF,
    0x6F12, 0x0746,
    0x6F12, 0x0124,
    0x6F12, 0x3846,
    0x6F12, 0x00F0,
    0x6F12, 0x1BF9,
    0x6F12, 0xD9F8,
    0x6F12, 0x0000,
    0x6F12, 0xB0F8,
    0x6F12, 0xD602,
    0x6F12, 0x08B9,
    0x6F12, 0xA6F8,
    0x6F12, 0x0280,
    0x6F12, 0xC7B3,
    0x6F12, 0x4746,
    0x6F12, 0xA6F8,
    0x6F12, 0x0880,
    0x6F12, 0x00F0,
    0x6F12, 0x13F9,
    0x6F12, 0xF068,
    0x6F12, 0x3061,
    0x6F12, 0x688D,
    0x6F12, 0x50B3,
    0x6F12, 0xA88D,
    0x6F12, 0x50BB,
    0x6F12, 0x00F0,
    0x6F12, 0x10F9,
    0x6F12, 0xA889,
    0x6F12, 0x20B3,
    0x6F12, 0x1CB3,
    0x6F12, 0x706B,
    0x6F12, 0xAA88,
    0x6F12, 0xDAF8,
    0x6F12, 0x0815,
    0x6F12, 0xCAB1,
    0x6F12, 0x8842,
    0x6F12, 0x0CDB,
    0x6F12, 0x90FB,
    0x6F12, 0xF1F3,
    0x6F12, 0x90FB,
    0x6F12, 0xF1F2,
    0x6F12, 0x01FB,
    0x6F12, 0x1303,
    0x6F12, 0xB3EB,
    0x6F12, 0x610F,
    0x6F12, 0x00DD,
    0x6F12, 0x521C,
    0x6F12, 0x01FB,
    0x6F12, 0x1200,
    0x6F12, 0x0BE0,
    0x6F12, 0x91FB,
    0x6F12, 0xF0F3,
    0x6F12, 0x91FB,
    0x6F12, 0xF0F2,
    0x6F12, 0x00FB,
    0x6F12, 0x1313,
    0x6F12, 0xB3EB,
    0x6F12, 0x600F,
    0x6F12, 0x00DD,
    0x6F12, 0x521C,
    0x6F12, 0x5043,
    0x6F12, 0x401A,
    0x6F12, 0xF168,
    0x6F12, 0x01EB,
    0x6F12, 0x4000,
    0x6F12, 0xF060,
    0x6F12, 0xA88D,
    0x6F12, 0x10B1,
    0x6F12, 0xF089,
    0x6F12, 0x3087,
    0x6F12, 0xAF85,
    0x6F12, 0x5846,
    0x6F12, 0xBDE8,
    0x6F12, 0xFC5F,
    0x6F12, 0x00F0,
    0x6F12, 0xE4B8,
    0x6F12, 0x70B5,
    0x6F12, 0x3049,
    0x6F12, 0x0446,
    0x6F12, 0x0020,
    0x6F12, 0xC1F8,
    0x6F12, 0x3005,
    0x6F12, 0x2F48,
    0x6F12, 0x0022,
    0x6F12, 0xC168,
    0x6F12, 0x0D0C,
    0x6F12, 0x8EB2,
    0x6F12, 0x3146,
    0x6F12, 0x2846,
    0x6F12, 0x00F0,
    0x6F12, 0x6CF8,
    0x6F12, 0x2046,
    0x6F12, 0x00F0,
    0x6F12, 0xD7F8,
    0x6F12, 0x3146,
    0x6F12, 0x2846,
    0x6F12, 0xBDE8,
    0x6F12, 0x7040,
    0x6F12, 0x0122,
    0x6F12, 0x00F0,
    0x6F12, 0x62B8,
    0x6F12, 0x10B5,
    0x6F12, 0x0022,
    0x6F12, 0xAFF2,
    0x6F12, 0x6751,
    0x6F12, 0x2448,
    0x6F12, 0x00F0,
    0x6F12, 0xCEF8,
    0x6F12, 0x224C,
    0x6F12, 0x0122,
    0x6F12, 0xAFF2,
    0x6F12, 0xD941,
    0x6F12, 0x2060,
    0x6F12, 0x2148,
    0x6F12, 0x00F0,
    0x6F12, 0xC6F8,
    0x6F12, 0x0022,
    0x6F12, 0xAFF2,
    0x6F12, 0xA141,
    0x6F12, 0x6060,
    0x6F12, 0x1F48,
    0x6F12, 0x00F0,
    0x6F12, 0xBFF8,
    0x6F12, 0x0022,
    0x6F12, 0xAFF2,
    0x6F12, 0xE931,
    0x6F12, 0xA060,
    0x6F12, 0x1C48,
    0x6F12, 0x00F0,
    0x6F12, 0xB8F8,
    0x6F12, 0x0022,
    0x6F12, 0xAFF2,
    0x6F12, 0xB731,
    0x6F12, 0x1A48,
    0x6F12, 0x00F0,
    0x6F12, 0xB2F8,
    0x6F12, 0x0022,
    0x6F12, 0xAFF2,
    0x6F12, 0x7331,
    0x6F12, 0x1848,
    0x6F12, 0x00F0,
    0x6F12, 0xACF8,
    0x6F12, 0x0022,
    0x6F12, 0xAFF2,
    0x6F12, 0x2F31,
    0x6F12, 0x1648,
    0x6F12, 0x00F0,
    0x6F12, 0xA6F8,
    0x6F12, 0x0022,
    0x6F12, 0xAFF2,
    0x6F12, 0x7521,
    0x6F12, 0x1448,
    0x6F12, 0x00F0,
    0x6F12, 0xA0F8,
    0x6F12, 0x0022,
    0x6F12, 0xAFF2,
    0x6F12, 0xED11,
    0x6F12, 0x1248,
    0x6F12, 0x00F0,
    0x6F12, 0x9AF8,
    0x6F12, 0x0022,
    0x6F12, 0xAFF2,
    0x6F12, 0xAD01,
    0x6F12, 0x1048,
    0x6F12, 0x00F0,
    0x6F12, 0x94F8,
    0x6F12, 0xE060,
    0x6F12, 0x10BD,
    0x6F12, 0x0000,
    0x6F12, 0x2000,
    0x6F12, 0x2BA0,
    0x6F12, 0x2000,
    0x6F12, 0x0890,
    0x6F12, 0x4000,
    0x6F12, 0x7000,
    0x6F12, 0x2000,
    0x6F12, 0x2E30,
    0x6F12, 0x2000,
    0x6F12, 0x4690,
    0x6F12, 0x0000,
    0x6F12, 0x24A7,
    0x6F12, 0x0001,
    0x6F12, 0x1AF3,
    0x6F12, 0x0001,
    0x6F12, 0x09BD,
    0x6F12, 0x0000,
    0x6F12, 0xA943,
    0x6F12, 0x0000,
    0x6F12, 0x71F1,
    0x6F12, 0x0000,
    0x6F12, 0x7239,
    0x6F12, 0x0000,
    0x6F12, 0x5D87,
    0x6F12, 0x0000,
    0x6F12, 0x576B,
    0x6F12, 0x0000,
    0x6F12, 0x57ED,
    0x6F12, 0x0000,
    0x6F12, 0xBF8D,
    0x6F12, 0x4AF6,
    0x6F12, 0x293C,
    0x6F12, 0xC0F2,
    0x6F12, 0x000C,
    0x6F12, 0x6047,
    0x6F12, 0x42F2,
    0x6F12, 0xA74C,
    0x6F12, 0xC0F2,
    0x6F12, 0x000C,
    0x6F12, 0x6047,
    0x6F12, 0x41F6,
    0x6F12, 0xF32C,
    0x6F12, 0xC0F2,
    0x6F12, 0x010C,
    0x6F12, 0x6047,
    0x6F12, 0x40F6,
    0x6F12, 0xBD1C,
    0x6F12, 0xC0F2,
    0x6F12, 0x010C,
    0x6F12, 0x6047,
    0x6F12, 0x4AF6,
    0x6F12, 0x2D1C,
    0x6F12, 0xC0F2,
    0x6F12, 0x000C,
    0x6F12, 0x6047,
    0x6F12, 0x48F2,
    0x6F12, 0x0B3C,
    0x6F12, 0xC0F2,
    0x6F12, 0x000C,
    0x6F12, 0x6047,
    0x6F12, 0x4AF6,
    0x6F12, 0x431C,
    0x6F12, 0xC0F2,
    0x6F12, 0x000C,
    0x6F12, 0x6047,
    0x6F12, 0x48F2,
    0x6F12, 0x6F2C,
    0x6F12, 0xC0F2,
    0x6F12, 0x000C,
    0x6F12, 0x6047,
    0x6F12, 0x47F6,
    0x6F12, 0xA57C,
    0x6F12, 0xC0F2,
    0x6F12, 0x000C,
    0x6F12, 0x6047,
    0x6F12, 0x45F6,
    0x6F12, 0x815C,
    0x6F12, 0xC0F2,
    0x6F12, 0x000C,
    0x6F12, 0x6047,
    0x6F12, 0x4AF6,
    0x6F12, 0xE70C,
    0x6F12, 0xC0F2,
    0x6F12, 0x000C,
    0x6F12, 0x6047,
    0x6F12, 0x4AF6,
    0x6F12, 0x171C,
    0x6F12, 0xC0F2,
    0x6F12, 0x000C,
    0x6F12, 0x6047,
    0x6F12, 0x4AF6,
    0x6F12, 0x453C,
    0x6F12, 0xC0F2,
    0x6F12, 0x000C,
    0x6F12, 0x6047,
    0x6F12, 0x4AF6,
    0x6F12, 0x532C,
    0x6F12, 0xC0F2,
    0x6F12, 0x000C,
    0x6F12, 0x6047,
    0x6F12, 0x45F2,
    0x6F12, 0x377C,
    0x6F12, 0xC0F2,
    0x6F12, 0x000C,
    0x6F12, 0x6047,
    0x6F12, 0x45F2,
    0x6F12, 0xD56C,
    0x6F12, 0xC0F2,
    0x6F12, 0x000C,
    0x6F12, 0x6047,
    0x6F12, 0x45F2,
    0x6F12, 0xC91C,
    0x6F12, 0xC0F2,
    0x6F12, 0x000C,
    0x6F12, 0x6047,
    0x6F12, 0x40F2,
    0x6F12, 0xAB2C,
    0x6F12, 0xC0F2,
    0x6F12, 0x000C,
    0x6F12, 0x6047,
    0x6F12, 0x44F6,
    0x6F12, 0x897C,
    0x6F12, 0xC0F2,
    0x6F12, 0x000C,
    0x6F12, 0x6047,
    0x6F12, 0x45F2,
    0x6F12, 0xA56C,
    0x6F12, 0xC0F2,
    0x6F12, 0x000C,
    0x6F12, 0x6047,
    0x6F12, 0x45F2,
    0x6F12, 0xEF6C,
    0x6F12, 0xC0F2,
    0x6F12, 0x000C,
    0x6F12, 0x6047,
    0x6F12, 0x40F2,
    0x6F12, 0x6D7C,
    0x6F12, 0xC0F2,
    0x6F12, 0x000C,
    0x6F12, 0x6047,
    0x6F12, 0x4BF6,
    0x6F12, 0x8D7C,
    0x6F12, 0xC0F2,
    0x6F12, 0x000C,
    0x6F12, 0x6047,
    0x6F12, 0x4BF2,
    0x6F12, 0xAB4C,
    0x6F12, 0xC0F2,
    0x6F12, 0x000C,
    0x6F12, 0x6047,
    0x602A, 0x46A0,
    0x6F12, 0x0549,
    0x6F12, 0x0448,
    0x6F12, 0x054A,
    0x6F12, 0xC1F8,
    0x6F12, 0x5005,
    0x6F12, 0x101A,
    0x6F12, 0xA1F8,
    0x6F12, 0x5405,
    0x6F12, 0xFFF7,
    0x6F12, 0x0EBF,
    0x6F12, 0x2000,
    0x6F12, 0x46D8,
    0x6F12, 0x2000,
    0x6F12, 0x2E30,
    0x6F12, 0x2000,
    0x6F12, 0x6000,
    0x6F12, 0x7047,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x08D1,
    0x6F12, 0x00A6,
    0x6F12, 0x0000,
    0x6F12, 0x00FF,
};

static kal_uint16 addr_data_pair_capture[] = {
    0x6028, 0x4000,
    0x6214, 0x7971,
    0x6218, 0x7150,
    0x0344, 0x0008,
    0x0346, 0x0008,
    0x0348, 0x0FA7,
    0x034A, 0x0BBF,
    0x034C, 0x0FA0,
    0x034E, 0x0BB8,
    0x0350, 0x0000,
    0x0352, 0x0000,
    0x0340, 0x0C7A,
    0x0342, 0x13A0,
    0x0900, 0x0111,
    0x0380, 0x0001,
    0x0382, 0x0001,
    0x0384, 0x0001,
    0x0386, 0x0001,
    0x0404, 0x1000,
    0x0402, 0x1010,
    0x0136, 0x1800,
    0x0304, 0x0006,
    0x030C, 0x0000,
    0x0306, 0x00F1,
    0x0302, 0x0001,
    0x0300, 0x0008,
    0x030E, 0x0003,
    0x0312, 0x0001,
    0x0310, 0x0090,
    0x6028, 0x2000,
    0x602A, 0x1492,
    0x6F12, 0x0078,
    0x602A, 0x0E4E,
    0x6F12, 0x007A,
    0x6028, 0x4000,
    0x0118, 0x0004,
    0x021E, 0x0000,
    0x6028, 0x2000,
    0x602A, 0x2126,
    0x6F12, 0x0100,
    0x602A, 0x1168,
    0x6F12, 0x0020,
    0x602A, 0x2DB6,
    0x6F12, 0x0001,
    0x602A, 0x1668,
    0x6F12, 0xF0F0,
    0x602A, 0x166A,
    0x6F12, 0xF0F0,
    0x602A, 0x118A,
    0x6F12, 0x0802,
    0x602A, 0x151E,
    0x6F12, 0x0001,
    0x602A, 0x217E,
    0x6F12, 0x0001,
    0x602A, 0x1520,
    0x6F12, 0x0008,
    0x602A, 0x2522,
    0x6F12, 0x0804,
    0x602A, 0x2524,
    0x6F12, 0x0400,
    0x602A, 0x2568,
    0x6F12, 0x5500,
    0x602A, 0x2588,
    0x6F12, 0x1111,
    0x602A, 0x258C,
    0x6F12, 0x1111,
    0x602A, 0x25A6,
    0x6F12, 0x0000,
    0x602A, 0x252C,
    0x6F12, 0x0601,
    0x602A, 0x252E,
    0x6F12, 0x0605,
    0x602A, 0x25A8,
    0x6F12, 0x1100,
    0x602A, 0x25AC,
    0x6F12, 0x0011,
    0x602A, 0x25B0,
    0x6F12, 0x1100,
    0x602A, 0x25B4,
    0x6F12, 0x0011,
    0x602A, 0x15A4,
    0x6F12, 0x0141,
    0x602A, 0x15A6,
    0x6F12, 0x0545,
    0x602A, 0x15A8,
    0x6F12, 0x0649,
    0x602A, 0x15AA,
    0x6F12, 0x024D,
    0x602A, 0x15AC,
    0x6F12, 0x0151,
    0x602A, 0x15AE,
    0x6F12, 0x0555,
    0x602A, 0x15B0,
    0x6F12, 0x0659,
    0x602A, 0x15B2,
    0x6F12, 0x025D,
    0x602A, 0x15B4,
    0x6F12, 0x0161,
    0x602A, 0x15B6,
    0x6F12, 0x0565,
    0x602A, 0x15B8,
    0x6F12, 0x0669,
    0x602A, 0x15BA,
    0x6F12, 0x026D,
    0x602A, 0x15BC,
    0x6F12, 0x0171,
    0x602A, 0x15BE,
    0x6F12, 0x0575,
    0x602A, 0x15C0,
    0x6F12, 0x0679,
    0x602A, 0x15C2,
    0x6F12, 0x027D,
    0x602A, 0x15C4,
    0x6F12, 0x0141,
    0x602A, 0x15C6,
    0x6F12, 0x0545,
    0x602A, 0x15C8,
    0x6F12, 0x0649,
    0x602A, 0x15CA,
    0x6F12, 0x024D,
    0x602A, 0x15CC,
    0x6F12, 0x0151,
    0x602A, 0x15CE,
    0x6F12, 0x0555,
    0x602A, 0x15D0,
    0x6F12, 0x0659,
    0x602A, 0x15D2,
    0x6F12, 0x025D,
    0x602A, 0x15D4,
    0x6F12, 0x0161,
    0x602A, 0x15D6,
    0x6F12, 0x0565,
    0x602A, 0x15D8,
    0x6F12, 0x0669,
    0x602A, 0x15DA,
    0x6F12, 0x026D,
    0x602A, 0x15DC,
    0x6F12, 0x0171,
    0x602A, 0x15DE,
    0x6F12, 0x0575,
    0x602A, 0x15E0,
    0x6F12, 0x0679,
    0x602A, 0x15E2,
    0x6F12, 0x027D,
    0x602A, 0x1A50,
    0x6F12, 0x0001,
    0x602A, 0x1A54,
    0x6F12, 0x0100,
    0x6028, 0x4000,
    0x0D00, 0x0101,
    0x0D02, 0x0101,
    0x0114, 0x0301,
    0xF486, 0x0000,
    0xF488, 0x0000,
    0xF48A, 0x0000,
    0xF48C, 0x0000,
    0xF48E, 0x0000,
    0xF490, 0x0000,
    0xF492, 0x0000,
    0xF494, 0x0000,
    0xF496, 0x0000,
    0xF498, 0x0000,
    0xF49A, 0x0000,
    0xF49C, 0x0000,
    0xF49E, 0x0000,
    0xF4A0, 0x0000,
    0xF4A2, 0x0000,
    0xF4A4, 0x0000,
    0xF4A6, 0x0000,
    0xF4A8, 0x0000,
    0xF4AA, 0x0000,
    0xF4AC, 0x0000,
    0xF4AE, 0x0000,
    0xF4B0, 0x0000,
    0xF4B2, 0x0000,
    0xF4B4, 0x0000,
    0xF4B6, 0x0000,
    0xF4B8, 0x0000,
    0xF4BA, 0x0000,
    0xF4BC, 0x0000,
    0xF4BE, 0x0000,
    0xF4C0, 0x0000,
    0xF4C2, 0x0000,
    0xF4C4, 0x0000,
    0x0202, 0x0010,
    0x0226, 0x0010,
    0x0204, 0x0020,
    0x0B06, 0x0101,
    0x6028, 0x2000,
    0x602A, 0x107A,
    0x6F12, 0x1D00,
    0x602A, 0x1074,
    0x6F12, 0x1D00,
    0x602A, 0x0E7C,
    0x6F12, 0x0000,
    0x602A, 0x1120,
    0x6F12, 0x0200,
    0x602A, 0x1122,
    0x6F12, 0x0028,
    0x602A, 0x1128,
    0x6F12, 0x0604,
    0x602A, 0x1AC0,
    0x6F12, 0x0200,
    0x602A, 0x1AC2,
    0x6F12, 0x0002,
    0x602A, 0x1494,
    0x6F12, 0x3D68,
    0x602A, 0x1498,
    0x6F12, 0xF10D,
    0x602A, 0x1488,
    0x6F12, 0x0F04,
    0x602A, 0x148A,
    0x6F12, 0x170B,
    0x602A, 0x150E,
    0x6F12, 0x40C2,
    0x602A, 0x1510,
    0x6F12, 0x80AF,
    0x602A, 0x1512,
    0x6F12, 0x00A0,
    0x602A, 0x1486,
    0x6F12, 0x1430,
    0x602A, 0x1490,
    0x6F12, 0x5009,
    0x602A, 0x149E,
    0x6F12, 0x01C4,
    0x602A, 0x11CC,
    0x6F12, 0x0008,
    0x602A, 0x11CE,
    0x6F12, 0x000B,
    0x602A, 0x11D0,
    0x6F12, 0x0006,
    0x602A, 0x11DA,
    0x6F12, 0x0012,
    0x602A, 0x11E6,
    0x6F12, 0x002A,
    0x602A, 0x125E,
    0x6F12, 0x0048,
    0x602A, 0x11F4,
    0x6F12, 0x0000,
    0x602A, 0x11F8,
    0x6F12, 0x0016,
    0x6028, 0x4000,
    0xF444, 0x05BF,
    0xF44A, 0x0016,
    0xF44C, 0x1414,
    0xF44E, 0x0014,
    0xF458, 0x0008,
    0xF46E, 0xC040,
    0xF470, 0x0008,
    0x6028, 0x2000,
    0x602A, 0x1CAA,
    0x6F12, 0x0000,
    0x602A, 0x1CAC,
    0x6F12, 0x0000,
    0x602A, 0x1CAE,
    0x6F12, 0x0000,
    0x602A, 0x1CB0,
    0x6F12, 0x0000,
    0x602A, 0x1CB2,
    0x6F12, 0x0000,
    0x602A, 0x1CB4,
    0x6F12, 0x0000,
    0x602A, 0x1CB6,
    0x6F12, 0x0000,
    0x602A, 0x1CB8,
    0x6F12, 0x0000,
    0x602A, 0x1CBA,
    0x6F12, 0x0000,
    0x602A, 0x1CBC,
    0x6F12, 0x0000,
    0x602A, 0x1CBE,
    0x6F12, 0x0000,
    0x602A, 0x1CC0,
    0x6F12, 0x0000,
    0x602A, 0x1CC2,
    0x6F12, 0x0000,
    0x602A, 0x1CC4,
    0x6F12, 0x0000,
    0x602A, 0x1CC6,
    0x6F12, 0x0000,
    0x602A, 0x1CC8,
    0x6F12, 0x0000,
    0x602A, 0x6000,
    0x6F12, 0x000F,
    0x602A, 0x6002,
    0x6F12, 0xFFFF,
    0x602A, 0x6004,
    0x6F12, 0x0000,
    0x602A, 0x6006,
    0x6F12, 0x1000,
    0x602A, 0x6008,
    0x6F12, 0x1000,
    0x602A, 0x600A,
    0x6F12, 0x1000,
    0x602A, 0x600C,
    0x6F12, 0x1000,
    0x602A, 0x600E,
    0x6F12, 0x1000,
    0x602A, 0x6010,
    0x6F12, 0x1000,
    0x602A, 0x6012,
    0x6F12, 0x1000,
    0x602A, 0x6014,
    0x6F12, 0x1000,
    0x602A, 0x6016,
    0x6F12, 0x1000,
    0x602A, 0x6018,
    0x6F12, 0x1000,
    0x602A, 0x601A,
    0x6F12, 0x1000,
    0x602A, 0x601C,
    0x6F12, 0x1000,
    0x602A, 0x601E,
    0x6F12, 0x1000,
    0x602A, 0x6020,
    0x6F12, 0x1000,
    0x602A, 0x6022,
    0x6F12, 0x1000,
    0x602A, 0x6024,
    0x6F12, 0x1000,
    0x602A, 0x6026,
    0x6F12, 0x1000,
    0x602A, 0x6028,
    0x6F12, 0x1000,
    0x602A, 0x602A,
    0x6F12, 0x1000,
    0x602A, 0x602C,
    0x6F12, 0x1000,
    0x602A, 0x1144,
    0x6F12, 0x0100,
    0x602A, 0x1146,
    0x6F12, 0x1B00,
    0x602A, 0x1080,
    0x6F12, 0x0100,
    0x602A, 0x1084,
    0x6F12, 0x00C0,
    0x602A, 0x108A,
    0x6F12, 0x00C0,
    0x602A, 0x1090,
    0x6F12, 0x0001,
    0x602A, 0x1092,
    0x6F12, 0x0000,
    0x602A, 0x1094,
    0x6F12, 0xA32E,
    0x602A, 0x602E,
    0x6F12, 0x0000,
    0x602A, 0x6038,
    0x6F12, 0x0003,
    0x602A, 0x603A,
    0x6F12, 0x005F,
    0x602A, 0x603C,
    0x6F12, 0x0060,
    0x602A, 0x603E,
    0x6F12, 0x0061,
    0x602A, 0x25D4,
    0x6F12, 0x0020,
    0x602A, 0x25D6,
    0x6F12, 0x0020,
};

static kal_uint16 addr_data_pair_hs_video[] = {
    0x6028, 0x4000,
    0x6214, 0x7971,
    0x6218, 0x7150,
    0x0344, 0x0008,
    0x0346, 0x0008,
    0x0348, 0x0FA7,
    0x034A, 0x0BBF,
    0x034C, 0x07D0,
    0x034E, 0x05DC,
    0x0350, 0x0000,
    0x0352, 0x0000,
    0x0340, 0x0640,
    0x0342, 0x09D0,
    0x0900, 0x0122,
    0x0380, 0x0001,
    0x0382, 0x0003,
    0x0384, 0x0001,
    0x0386, 0x0003,
    0x0404, 0x1000,
    0x0402, 0x1010,
    0x0136, 0x1800,
    0x0304, 0x0006,
    0x030C, 0x0000,
    0x0306, 0x00F1,
    0x0302, 0x0001,
    0x0300, 0x0008,
    0x030E, 0x0003,
    0x0312, 0x0001,
    0x0310, 0x0095,
    0x6028, 0x2000,
    0x602A, 0x1492,
    0x6F12, 0x0078,
    0x602A, 0x0E4E,
    0x6F12, 0x0069,
    0x6028, 0x4000,
    0x0118, 0x0004,
    0x021E, 0x0000,
    0x6028, 0x2000,
    0x602A, 0x2126,
    0x6F12, 0x0000,
    0x602A, 0x1168,
    0x6F12, 0x0020,
    0x602A, 0x2DB6,
    0x6F12, 0x0001,
    0x602A, 0x1668,
    0x6F12, 0xFF00,
    0x602A, 0x166A,
    0x6F12, 0xFF00,
    0x602A, 0x118A,
    0x6F12, 0x0402,
    0x602A, 0x151E,
    0x6F12, 0x0002,
    0x602A, 0x217E,
    0x6F12, 0x0001,
    0x602A, 0x1520,
    0x6F12, 0x0000,
    0x602A, 0x2522,
    0x6F12, 0x1004,
    0x602A, 0x2524,
    0x6F12, 0x0200,
    0x602A, 0x2568,
    0x6F12, 0x0000,
    0x602A, 0x2588,
    0x6F12, 0x0000,
    0x602A, 0x258C,
    0x6F12, 0x0000,
    0x602A, 0x25A6,
    0x6F12, 0x0000,
    0x602A, 0x252C,
    0x6F12, 0x0601,
    0x602A, 0x252E,
    0x6F12, 0x0605,
    0x602A, 0x25A8,
    0x6F12, 0x1100,
    0x602A, 0x25AC,
    0x6F12, 0x0011,
    0x602A, 0x25B0,
    0x6F12, 0x1100,
    0x602A, 0x25B4,
    0x6F12, 0x0011,
    0x602A, 0x15A4,
    0x6F12, 0x0641,
    0x602A, 0x15A6,
    0x6F12, 0x0145,
    0x602A, 0x15A8,
    0x6F12, 0x0149,
    0x602A, 0x15AA,
    0x6F12, 0x064D,
    0x602A, 0x15AC,
    0x6F12, 0x0651,
    0x602A, 0x15AE,
    0x6F12, 0x0155,
    0x602A, 0x15B0,
    0x6F12, 0x0159,
    0x602A, 0x15B2,
    0x6F12, 0x065D,
    0x602A, 0x15B4,
    0x6F12, 0x0661,
    0x602A, 0x15B6,
    0x6F12, 0x0165,
    0x602A, 0x15B8,
    0x6F12, 0x0169,
    0x602A, 0x15BA,
    0x6F12, 0x066D,
    0x602A, 0x15BC,
    0x6F12, 0x0671,
    0x602A, 0x15BE,
    0x6F12, 0x0175,
    0x602A, 0x15C0,
    0x6F12, 0x0179,
    0x602A, 0x15C2,
    0x6F12, 0x067D,
    0x602A, 0x15C4,
    0x6F12, 0x0641,
    0x602A, 0x15C6,
    0x6F12, 0x0145,
    0x602A, 0x15C8,
    0x6F12, 0x0149,
    0x602A, 0x15CA,
    0x6F12, 0x064D,
    0x602A, 0x15CC,
    0x6F12, 0x0651,
    0x602A, 0x15CE,
    0x6F12, 0x0155,
    0x602A, 0x15D0,
    0x6F12, 0x0159,
    0x602A, 0x15D2,
    0x6F12, 0x065D,
    0x602A, 0x15D4,
    0x6F12, 0x0661,
    0x602A, 0x15D6,
    0x6F12, 0x0165,
    0x602A, 0x15D8,
    0x6F12, 0x0169,
    0x602A, 0x15DA,
    0x6F12, 0x066D,
    0x602A, 0x15DC,
    0x6F12, 0x0671,
    0x602A, 0x15DE,
    0x6F12, 0x0175,
    0x602A, 0x15E0,
    0x6F12, 0x0179,
    0x602A, 0x15E2,
    0x6F12, 0x067D,
    0x602A, 0x1A50,
    0x6F12, 0x0001,
    0x602A, 0x1A54,
    0x6F12, 0x0100,
    0x6028, 0x4000,
    0x0D00, 0x0100,
    0x0D02, 0x0001,
    0x0114, 0x0300,
    0xF486, 0x0000,
    0xF488, 0x0000,
    0xF48A, 0x0000,
    0xF48C, 0x0000,
    0xF48E, 0x0000,
    0xF490, 0x0000,
    0xF492, 0x0000,
    0xF494, 0x0000,
    0xF496, 0x0000,
    0xF498, 0x0000,
    0xF49A, 0x0000,
    0xF49C, 0x0000,
    0xF49E, 0x0000,
    0xF4A0, 0x0000,
    0xF4A2, 0x0000,
    0xF4A4, 0x0000,
    0xF4A6, 0x0000,
    0xF4A8, 0x0000,
    0xF4AA, 0x0000,
    0xF4AC, 0x0000,
    0xF4AE, 0x0000,
    0xF4B0, 0x0000,
    0xF4B2, 0x0000,
    0xF4B4, 0x0000,
    0xF4B6, 0x0000,
    0xF4B8, 0x0000,
    0xF4BA, 0x0000,
    0xF4BC, 0x0000,
    0xF4BE, 0x0000,
    0xF4C0, 0x0000,
    0xF4C2, 0x0000,
    0xF4C4, 0x0000,
    0x0202, 0x0010,
    0x0226, 0x0010,
    0x0204, 0x0020,
    0x0B06, 0x0101,
    0x6028, 0x2000,
    0x602A, 0x107A,
    0x6F12, 0x1D00,
    0x602A, 0x1074,
    0x6F12, 0x1D00,
    0x602A, 0x0E7C,
    0x6F12, 0x0000,
    0x602A, 0x1120,
    0x6F12, 0x0200,
    0x602A, 0x1122,
    0x6F12, 0x0028,
    0x602A, 0x1128,
    0x6F12, 0x0604,
    0x602A, 0x1AC0,
    0x6F12, 0x0200,
    0x602A, 0x1AC2,
    0x6F12, 0x0002,
    0x602A, 0x1494,
    0x6F12, 0x3D68,
    0x602A, 0x1498,
    0x6F12, 0xF10D,
    0x602A, 0x1488,
    0x6F12, 0x0904,
    0x602A, 0x148A,
    0x6F12, 0x170B,
    0x602A, 0x150E,
    0x6F12, 0x00C2,
    0x602A, 0x1510,
    0x6F12, 0xC0AF,
    0x602A, 0x1512,
    0x6F12, 0x0080,
    0x602A, 0x1486,
    0x6F12, 0x1430,
    0x602A, 0x1490,
    0x6F12, 0x4D09,
    0x602A, 0x149E,
    0x6F12, 0x01C4,
    0x602A, 0x11CC,
    0x6F12, 0x0008,
    0x602A, 0x11CE,
    0x6F12, 0x000B,
    0x602A, 0x11D0,
    0x6F12, 0x0003,
    0x602A, 0x11DA,
    0x6F12, 0x0012,
    0x602A, 0x11E6,
    0x6F12, 0x002A,
    0x602A, 0x125E,
    0x6F12, 0x0048,
    0x602A, 0x11F4,
    0x6F12, 0x0000,
    0x602A, 0x11F8,
    0x6F12, 0x0016,
    0x6028, 0x4000,
    0xF444, 0x05BF,
    0xF44A, 0x0008,
    0xF44E, 0x0012,
    0xF46E, 0x90C0,
    0xF470, 0x2809,
    0x6028, 0x2000,
    0x602A, 0x1CAA,
    0x6F12, 0x0000,
    0x602A, 0x1CAC,
    0x6F12, 0x0000,
    0x602A, 0x1CAE,
    0x6F12, 0x0000,
    0x602A, 0x1CB0,
    0x6F12, 0x0000,
    0x602A, 0x1CB2,
    0x6F12, 0x0000,
    0x602A, 0x1CB4,
    0x6F12, 0x0000,
    0x602A, 0x1CB6,
    0x6F12, 0x0000,
    0x602A, 0x1CB8,
    0x6F12, 0x0000,
    0x602A, 0x1CBA,
    0x6F12, 0x0000,
    0x602A, 0x1CBC,
    0x6F12, 0x0000,
    0x602A, 0x1CBE,
    0x6F12, 0x0000,
    0x602A, 0x1CC0,
    0x6F12, 0x0000,
    0x602A, 0x1CC2,
    0x6F12, 0x0000,
    0x602A, 0x1CC4,
    0x6F12, 0x0000,
    0x602A, 0x1CC6,
    0x6F12, 0x0000,
    0x602A, 0x1CC8,
    0x6F12, 0x0000,
    0x602A, 0x6000,
    0x6F12, 0x000F,
    0x602A, 0x6002,
    0x6F12, 0xFFFF,
    0x602A, 0x6004,
    0x6F12, 0x0000,
    0x602A, 0x6006,
    0x6F12, 0x1000,
    0x602A, 0x6008,
    0x6F12, 0x1000,
    0x602A, 0x600A,
    0x6F12, 0x1000,
    0x602A, 0x600C,
    0x6F12, 0x1000,
    0x602A, 0x600E,
    0x6F12, 0x1000,
    0x602A, 0x6010,
    0x6F12, 0x1000,
    0x602A, 0x6012,
    0x6F12, 0x1000,
    0x602A, 0x6014,
    0x6F12, 0x1000,
    0x602A, 0x6016,
    0x6F12, 0x1000,
    0x602A, 0x6018,
    0x6F12, 0x1000,
    0x602A, 0x601A,
    0x6F12, 0x1000,
    0x602A, 0x601C,
    0x6F12, 0x1000,
    0x602A, 0x601E,
    0x6F12, 0x1000,
    0x602A, 0x6020,
    0x6F12, 0x1000,
    0x602A, 0x6022,
    0x6F12, 0x1000,
    0x602A, 0x6024,
    0x6F12, 0x1000,
    0x602A, 0x6026,
    0x6F12, 0x1000,
    0x602A, 0x6028,
    0x6F12, 0x1000,
    0x602A, 0x602A,
    0x6F12, 0x1000,
    0x602A, 0x602C,
    0x6F12, 0x1000,
    0x602A, 0x1144,
    0x6F12, 0x0100,
    0x602A, 0x1146,
    0x6F12, 0x1B00,
    0x602A, 0x1080,
    0x6F12, 0x0100,
    0x602A, 0x1084,
    0x6F12, 0x00C0,
    0x602A, 0x108A,
    0x6F12, 0x00C0,
    0x602A, 0x1090,
    0x6F12, 0x0001,
    0x602A, 0x1092,
    0x6F12, 0x0000,
    0x602A, 0x1094,
    0x6F12, 0xA32E,
};

static kal_uint16 addr_data_pair_2000x1500_30fps[] = {
    0x6028, 0x4000,
    0x6214, 0x7971,
    0x6218, 0x7150,
    0x0344, 0x0008,
    0x0346, 0x0008,
    0x0348, 0x0FA7,
    0x034A, 0x0BBF,
    0x034C, 0x07D0,
    0x034E, 0x05DC,
    0x0350, 0x0000,
    0x0352, 0x0000,
    0x0340, 0x18F8,
    0x0342, 0x09D0,
    0x0900, 0x0121,
    0x0380, 0x0001,
    0x0382, 0x0003,
    0x0384, 0x0001,
    0x0386, 0x0001,
    0x0404, 0x1000,
    0x0402, 0x1020,
    0x0136, 0x1800,
    0x0304, 0x0006,
    0x030C, 0x0000,
    0x0306, 0x00F1,
    0x0302, 0x0001,
    0x0300, 0x0008,
    0x030E, 0x0004,
    0x0312, 0x0000,
    0x0310, 0x0064,
    0x6028, 0x2000,
    0x602A, 0x1492,
    0x6F12, 0x0078,
    0x602A, 0x0E4E,
    0x6F12, 0x0074,
    0x6028, 0x4000,
    0x0118, 0x0004,
    0x021E, 0x0000,
    0x6028, 0x2000,
    0x602A, 0x2126,
    0x6F12, 0x0000,
    0x602A, 0x1168,
    0x6F12, 0x0020,
    0x602A, 0x2DB6,
    0x6F12, 0x0001,
    0x602A, 0x1668,
    0x6F12, 0xFF00,
    0x602A, 0x166A,
    0x6F12, 0xFF00,
    0x602A, 0x118A,
    0x6F12, 0x0402,
    0x602A, 0x151E,
    0x6F12, 0x0001,
    0x602A, 0x217E,
    0x6F12, 0x0001,
    0x602A, 0x1520,
    0x6F12, 0x0100,
    0x602A, 0x2522,
    0x6F12, 0x0804,
    0x602A, 0x2524,
    0x6F12, 0x0400,
    0x602A, 0x2568,
    0x6F12, 0x5500,
    0x602A, 0x2588,
    0x6F12, 0x1111,
    0x602A, 0x258C,
    0x6F12, 0x1111,
    0x602A, 0x25A6,
    0x6F12, 0x0000,
    0x602A, 0x252C,
    0x6F12, 0x0601,
    0x602A, 0x252E,
    0x6F12, 0x0605,
    0x602A, 0x25A8,
    0x6F12, 0x1100,
    0x602A, 0x25AC,
    0x6F12, 0x0011,
    0x602A, 0x25B0,
    0x6F12, 0x1100,
    0x602A, 0x25B4,
    0x6F12, 0x0011,
    0x602A, 0x15A4,
    0x6F12, 0x0141,
    0x602A, 0x15A6,
    0x6F12, 0x0545,
    0x602A, 0x15A8,
    0x6F12, 0x0649,
    0x602A, 0x15AA,
    0x6F12, 0x024D,
    0x602A, 0x15AC,
    0x6F12, 0x0151,
    0x602A, 0x15AE,
    0x6F12, 0x0555,
    0x602A, 0x15B0,
    0x6F12, 0x0659,
    0x602A, 0x15B2,
    0x6F12, 0x025D,
    0x602A, 0x15B4,
    0x6F12, 0x0161,
    0x602A, 0x15B6,
    0x6F12, 0x0565,
    0x602A, 0x15B8,
    0x6F12, 0x0669,
    0x602A, 0x15BA,
    0x6F12, 0x026D,
    0x602A, 0x15BC,
    0x6F12, 0x0171,
    0x602A, 0x15BE,
    0x6F12, 0x0575,
    0x602A, 0x15C0,
    0x6F12, 0x0679,
    0x602A, 0x15C2,
    0x6F12, 0x027D,
    0x602A, 0x15C4,
    0x6F12, 0x0141,
    0x602A, 0x15C6,
    0x6F12, 0x0545,
    0x602A, 0x15C8,
    0x6F12, 0x0649,
    0x602A, 0x15CA,
    0x6F12, 0x024D,
    0x602A, 0x15CC,
    0x6F12, 0x0151,
    0x602A, 0x15CE,
    0x6F12, 0x0555,
    0x602A, 0x15D0,
    0x6F12, 0x0659,
    0x602A, 0x15D2,
    0x6F12, 0x025D,
    0x602A, 0x15D4,
    0x6F12, 0x0161,
    0x602A, 0x15D6,
    0x6F12, 0x0565,
    0x602A, 0x15D8,
    0x6F12, 0x0669,
    0x602A, 0x15DA,
    0x6F12, 0x026D,
    0x602A, 0x15DC,
    0x6F12, 0x0171,
    0x602A, 0x15DE,
    0x6F12, 0x0575,
    0x602A, 0x15E0,
    0x6F12, 0x0679,
    0x602A, 0x15E2,
    0x6F12, 0x027D,
    0x602A, 0x1A50,
    0x6F12, 0x0001,
    0x602A, 0x1A54,
    0x6F12, 0x0100,
    0x6028, 0x4000,
    0x0D00, 0x0100,
    0x0D02, 0x0001,
    0x0114, 0x0300,
    0xF486, 0x0641,
    0xF488, 0x0A45,
    0xF48A, 0x0A49,
    0xF48C, 0x064D,
    0xF48E, 0x0651,
    0xF490, 0x0A55,
    0xF492, 0x0A59,
    0xF494, 0x065D,
    0xF496, 0x0661,
    0xF498, 0x0A65,
    0xF49A, 0x0A69,
    0xF49C, 0x066D,
    0xF49E, 0x0671,
    0xF4A0, 0x0A75,
    0xF4A2, 0x0A79,
    0xF4A4, 0x067D,
    0xF4A6, 0x0641,
    0xF4A8, 0x0A45,
    0xF4AA, 0x0A49,
    0xF4AC, 0x064D,
    0xF4AE, 0x0651,
    0xF4B0, 0x0A55,
    0xF4B2, 0x0A59,
    0xF4B4, 0x065D,
    0xF4B6, 0x0661,
    0xF4B8, 0x0A65,
    0xF4BA, 0x0A69,
    0xF4BC, 0x066D,
    0xF4BE, 0x0671,
    0xF4C0, 0x0A75,
    0xF4C2, 0x0A79,
    0xF4C4, 0x067D,
    0x0202, 0x0010,
    0x0226, 0x0010,
    0x0204, 0x0020,
    0x0B06, 0x0101,
    0x6028, 0x2000,
    0x602A, 0x107A,
    0x6F12, 0x1D00,
    0x602A, 0x1074,
    0x6F12, 0x1D00,
    0x602A, 0x0E7C,
    0x6F12, 0x0000,
    0x602A, 0x1120,
    0x6F12, 0x0000,
    0x602A, 0x1122,
    0x6F12, 0x0028,
    0x602A, 0x1128,
    0x6F12, 0x0601,
    0x602A, 0x1AC0,
    0x6F12, 0x0200,
    0x602A, 0x1AC2,
    0x6F12, 0x0002,
    0x602A, 0x1494,
    0x6F12, 0x3D68,
    0x602A, 0x1498,
    0x6F12, 0xF10D,
    0x602A, 0x1488,
    0x6F12, 0x0F04,
    0x602A, 0x148A,
    0x6F12, 0x170B,
    0x602A, 0x150E,
    0x6F12, 0x40C2,
    0x602A, 0x1510,
    0x6F12, 0x80AF,
    0x602A, 0x1512,
    0x6F12, 0x00A0,
    0x602A, 0x1486,
    0x6F12, 0x1430,
    0x602A, 0x1490,
    0x6F12, 0x5009,
    0x602A, 0x149E,
    0x6F12, 0x01C4,
    0x602A, 0x11CC,
    0x6F12, 0x0008,
    0x602A, 0x11CE,
    0x6F12, 0x000B,
    0x602A, 0x11D0,
    0x6F12, 0x0006,
    0x602A, 0x11DA,
    0x6F12, 0x0012,
    0x602A, 0x11E6,
    0x6F12, 0x002A,
    0x602A, 0x125E,
    0x6F12, 0x0048,
    0x602A, 0x11F4,
    0x6F12, 0x0000,
    0x602A, 0x11F8,
    0x6F12, 0x0016,
    0x6028, 0x4000,
    0xF444, 0x05BF,
    0xF44A, 0x0016,
    0xF44C, 0x1414,
    0xF44E, 0x0014,
    0xF458, 0x0008,
    0xF46E, 0xD040,
    0xF470, 0x0008,
    0x6028, 0x2000,
    0x602A, 0x1CAA,
    0x6F12, 0x0000,
    0x602A, 0x1CAC,
    0x6F12, 0x0000,
    0x602A, 0x1CAE,
    0x6F12, 0x0000,
    0x602A, 0x1CB0,
    0x6F12, 0x0000,
    0x602A, 0x1CB2,
    0x6F12, 0x0000,
    0x602A, 0x1CB4,
    0x6F12, 0x0000,
    0x602A, 0x1CB6,
    0x6F12, 0x0000,
    0x602A, 0x1CB8,
    0x6F12, 0x0000,
    0x602A, 0x1CBA,
    0x6F12, 0x0000,
    0x602A, 0x1CBC,
    0x6F12, 0x0000,
    0x602A, 0x1CBE,
    0x6F12, 0x0000,
    0x602A, 0x1CC0,
    0x6F12, 0x0000,
    0x602A, 0x1CC2,
    0x6F12, 0x0000,
    0x602A, 0x1CC4,
    0x6F12, 0x0000,
    0x602A, 0x1CC6,
    0x6F12, 0x0000,
    0x602A, 0x1CC8,
    0x6F12, 0x0000,
    0x602A, 0x6000,
    0x6F12, 0x000F,
    0x602A, 0x6002,
    0x6F12, 0xFFFF,
    0x602A, 0x6004,
    0x6F12, 0x0000,
    0x602A, 0x6006,
    0x6F12, 0x1000,
    0x602A, 0x6008,
    0x6F12, 0x1000,
    0x602A, 0x600A,
    0x6F12, 0x1000,
    0x602A, 0x600C,
    0x6F12, 0x1000,
    0x602A, 0x600E,
    0x6F12, 0x1000,
    0x602A, 0x6010,
    0x6F12, 0x1000,
    0x602A, 0x6012,
    0x6F12, 0x1000,
    0x602A, 0x6014,
    0x6F12, 0x1000,
    0x602A, 0x6016,
    0x6F12, 0x1000,
    0x602A, 0x6018,
    0x6F12, 0x1000,
    0x602A, 0x601A,
    0x6F12, 0x1000,
    0x602A, 0x601C,
    0x6F12, 0x1000,
    0x602A, 0x601E,
    0x6F12, 0x1000,
    0x602A, 0x6020,
    0x6F12, 0x1000,
    0x602A, 0x6022,
    0x6F12, 0x1000,
    0x602A, 0x6024,
    0x6F12, 0x1000,
    0x602A, 0x6026,
    0x6F12, 0x1000,
    0x602A, 0x6028,
    0x6F12, 0x1000,
    0x602A, 0x602A,
    0x6F12, 0x1000,
    0x602A, 0x602C,
    0x6F12, 0x1000,
    0x602A, 0x1144,
    0x6F12, 0x0100,
    0x602A, 0x1146,
    0x6F12, 0x1B00,
    0x602A, 0x1080,
    0x6F12, 0x0100,
    0x602A, 0x1084,
    0x6F12, 0x00C0,
    0x602A, 0x108A,
    0x6F12, 0x00C0,
    0x602A, 0x1090,
    0x6F12, 0x0001,
    0x602A, 0x1092,
    0x6F12, 0x0000,
    0x602A, 0x1094,
    0x6F12, 0xA32E,
    0x602A, 0x602E,
    0x6F12, 0x0000,
    0x602A, 0x6038,
    0x6F12, 0x0003,
    0x602A, 0x603A,
    0x6F12, 0x005F,
    0x602A, 0x603C,
    0x6F12, 0x0060,
    0x602A, 0x603E,
    0x6F12, 0x0061,
    0x602A, 0x25D4,
    0x6F12, 0x0020,
    0x602A, 0x25D6,
    0x6F12, 0x0020,
};

static void sensor_init(void)
{
    /*Global setting */
    write_cmos_sensor_16_16(0x6028, 0x4000);
    write_cmos_sensor_16_16(0x0000, 0x0000);
    write_cmos_sensor_16_16(0x0000, 0xF8D1);
    write_cmos_sensor_16_16(0x6010, 0x0001);
    mdelay(6);
    write_cmos_sensor_16_16(0x6214, 0x7971);
    write_cmos_sensor_16_16(0x6218, 0x7150); /*open clk */
    write_cmos_sensor_16_16(0x0A02, 0x0074);
    LOG_INF(" use new chip init setting\n");
    table_write_cmos_sensor(addr_data_pair_init_new,
            sizeof(addr_data_pair_init_new) / sizeof(kal_uint16));
}	/*	sensor_init  */

/* Pll Setting - VCO = 280Mhz*/
static void capture_setting()
{
    LOG_INF("start\n");
    // Antaiui <AI_BSP_CAM> <xieht> <2021-06-04> add check streaming off begin
    write_cmos_sensor_16_8(0x0100, 0x00);
    check_output_stream_off();
    // Antaiui <AI_BSP_CAM> <xieht> <2021-06-04> add check streaming off end
    table_write_cmos_sensor(addr_data_pair_capture,
            sizeof(addr_data_pair_capture) / sizeof(kal_uint16));
    LOG_INF("end\n");
}

static void preview_setting(void)
{
    LOG_INF("start\n");
    capture_setting();
    LOG_INF("end\n");
}	/*	preview_setting  */

static void normal_video_setting(void)
{
    LOG_INF("start\n");
    capture_setting();
    LOG_INF("end\n");
}

static void hs_video_setting(void)
{
    LOG_INF("start\n");
    // Antaiui <AI_BSP_CAM> <xieht> <2021-06-04> add check streaming off begin
    write_cmos_sensor_16_8(0x0100, 0x00);
    check_output_stream_off();
    // Antaiui <AI_BSP_CAM> <xieht> <2021-06-04> add check streaming off end
    table_write_cmos_sensor(addr_data_pair_hs_video,
            sizeof(addr_data_pair_hs_video) / sizeof(kal_uint16));
    LOG_INF("end\n");
}

static void slim_video_setting(void)
{
    LOG_INF("start\n");
    // Antaiui <AI_BSP_CAM> <xieht> <2021-06-04> add check streaming off begin
    write_cmos_sensor_16_8(0x0100, 0x00);
    check_output_stream_off();
    // Antaiui <AI_BSP_CAM> <xieht> <2021-06-04> add check streaming off end
    table_write_cmos_sensor(addr_data_pair_2000x1500_30fps,
            sizeof(addr_data_pair_2000x1500_30fps) / sizeof(kal_uint16));
    LOG_INF("end\n");
}

/*************************************************************************
 * FUNCTION
 *	get_imgsensor_id
 *
 * DESCRIPTION
 *	This function get the sensor ID
 *
 * PARAMETERS
 *	*sensorID : return the sensor ID
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
    kal_uint8 i = 0;
    kal_uint8 retry = 2;

    /*sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address*/
    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do {
            *sensor_id = ((read_cmos_sensor_16_8(0x0000) << 8) | read_cmos_sensor_16_8(0x0001));
            LOG_INF("read out sensor id 0x%x\n", *sensor_id);
            if (*sensor_id == imgsensor_info.sensor_id) {
                LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id, *sensor_id);
                chipID = read_cmos_sensor_16_8(0x000F);
                LOG_INF("chipID: %d\n", chipID);
                return ERROR_NONE;
            }
            LOG_INF("Read sensor id fail, id: 0x%x\n", imgsensor.i2c_write_id);
            retry--;
        } while (retry > 0);
        i++;
        retry = 2;
    }
    if (*sensor_id !=  imgsensor_info.sensor_id) {
        /* if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF*/
        *sensor_id = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
    }
    return ERROR_NONE;
}

/*************************************************************************
 * FUNCTION
 *	open
 *
 * DESCRIPTION
 *	This function initialize the registers of CMOS sensor
 *
 * PARAMETERS
 *	None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 open(void)
{
    kal_uint8 i = 0;
    kal_uint8 retry = 2;
    kal_uint16 sensor_id = 0;

    LOG_INF("PLATFORM:MT6771,MIPI 4LANE\n");

    /*sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address*/
    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do {
            sensor_id = ((read_cmos_sensor_16_8(0x0000) << 8) | read_cmos_sensor_16_8(0x0001));
            if (sensor_id == imgsensor_info.sensor_id) {
                LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id, sensor_id);
                break;
            }
            LOG_INF("Read sensor id fail, id: 0x%x\n", imgsensor.i2c_write_id);
            retry--;
        } while (retry > 0);
        i++;
        if (sensor_id == imgsensor_info.sensor_id)
            break;
        retry = 2;
    }
    if (imgsensor_info.sensor_id !=  sensor_id)
        return ERROR_SENSOR_CONNECT_FAIL;

    /* initail sequence write in  */
    sensor_init();
    spin_lock(&imgsensor_drv_lock);

    imgsensor.autoflicker_en = KAL_FALSE;
    imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
    imgsensor.shutter = 0x3D0;
    imgsensor.gain = 0x100;
    imgsensor.pclk = imgsensor_info.pre.pclk;
    imgsensor.frame_length = imgsensor_info.pre.framelength;
    imgsensor.line_length = imgsensor_info.pre.linelength;
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.dummy_pixel = 0;
    imgsensor.dummy_line = 0;
    imgsensor.ihdr_mode = 0;
    imgsensor.test_pattern = KAL_FALSE;
    imgsensor.current_fps = imgsensor_info.pre.max_framerate;
    spin_unlock(&imgsensor_drv_lock);
    bNeedSetNormalMode = KAL_FALSE;

    return ERROR_NONE;
}   /*  open  */

/*************************************************************************
 * FUNCTION
 *	close
 *
 * DESCRIPTION
 *
 *
 * PARAMETERS
 *	None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 close(void)
{
    LOG_INF("E\n");

    /*No Need to implement this function*/

    return ERROR_NONE;
}   /*  close  */

/*************************************************************************
 * FUNCTION
 * preview
 *
 * DESCRIPTION
 *	This function start the sensor preview.
 *
 * PARAMETERS
 *	*image_window : address pointer of pixel numbers in one period of HSYNC
 *  *sensor_config_data : address pointer of line numbers in one period of VSYNC
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
        MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
    imgsensor.pclk = imgsensor_info.pre.pclk;
    imgsensor.line_length = imgsensor_info.pre.linelength;
    imgsensor.frame_length = imgsensor_info.pre.framelength;
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);

    preview_setting();  /* 4:3 normal binning size */
    set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}   /*  preview  */

/*************************************************************************
 * FUNCTION
 *	capture
 *
 * DESCRIPTION
 *	This function setup the CMOS sensor in capture MY_OUTPUT mode
 *
 * PARAMETERS
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
        MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
    if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
        LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
                imgsensor.current_fps, imgsensor_info.cap.max_framerate/10);
    imgsensor.pclk = imgsensor_info.cap.pclk;
    imgsensor.line_length = imgsensor_info.cap.linelength;
    imgsensor.frame_length = imgsensor_info.cap.framelength;
    imgsensor.min_frame_length = imgsensor_info.cap.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);

    capture_setting();
    set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}   /*  capture()  */

static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
        MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
    imgsensor.pclk = imgsensor_info.normal_video.pclk;
    imgsensor.line_length = imgsensor_info.normal_video.linelength;
    imgsensor.frame_length = imgsensor_info.normal_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);

    normal_video_setting();
    set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}   /*  normal_video  */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
        MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
    imgsensor.pclk = imgsensor_info.hs_video.pclk;
    imgsensor.line_length = imgsensor_info.hs_video.linelength;
    imgsensor.frame_length = imgsensor_info.hs_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);

    hs_video_setting();
    set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}   /*  hs_video  */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
        MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
    imgsensor.pclk = imgsensor_info.slim_video.pclk;
    imgsensor.line_length = imgsensor_info.slim_video.linelength;
    imgsensor.frame_length = imgsensor_info.slim_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);

    slim_video_setting();
    set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}   /*  slim_video  */

static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
    LOG_INF("E\n");
    sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
    sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

    sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
    sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

    sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
    sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;


    sensor_resolution->SensorHighSpeedVideoWidth	 = imgsensor_info.hs_video.grabwindow_width;
    sensor_resolution->SensorHighSpeedVideoHeight	 = imgsensor_info.hs_video.grabwindow_height;

    sensor_resolution->SensorSlimVideoWidth	 = imgsensor_info.slim_video.grabwindow_width;
    sensor_resolution->SensorSlimVideoHeight	 = imgsensor_info.slim_video.grabwindow_height;

    return ERROR_NONE;
}   /* get_resolution  */

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
        MSDK_SENSOR_INFO_STRUCT *sensor_info,
        MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("scenario_id = %d\n", scenario_id);
    sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
    sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; /* inverse with datasheet*/
    sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorInterruptDelayLines = 4; /* not use */
    sensor_info->SensorResetActiveHigh = FALSE; /* not use */
    sensor_info->SensorResetDelayCount = 5; /* not use */

    sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
    sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
    sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
    sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

    sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
    sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
    sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
    sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
    sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;

    sensor_info->SensorMasterClockSwitch = 0; /* not use */
    sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

    sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;
    sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;
    // Antaiui <AI_BSP_CAM> <xieht> <2021-03-12> frame time delay frame begin
    sensor_info->FrameTimeDelayFrame = imgsensor_info.frame_time_delay_frame;
    // Antaiui <AI_BSP_CAM> <xieht> <2021-03-12> frame time delay frame end
    sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
    sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
    sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
    sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
#ifdef PDAF_TEST
    sensor_info->PDAF_Support = 2;  // 0 NO PDAF, 1 raw,  2 vc
#else
    sensor_info->PDAF_Support = 0;
#endif
    sensor_info->HDR_Support = 0; /*0: NO HDR, 1: iHDR, 2:mvHDR, 3:zHDR, 4:four-cell mVHDR*/
    sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
    sensor_info->SensorClockFreq = imgsensor_info.mclk;
    sensor_info->SensorClockDividCount = 3; /* not use */
    sensor_info->SensorClockRisingCount = 0;
    sensor_info->SensorClockFallingCount = 2; /* not use */
    sensor_info->SensorPixelClockCount = 3; /* not use */
    sensor_info->SensorDataLatchCount = 2; /* not use */

    sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
    sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
    sensor_info->SensorWidthSampling = 0;  /* 0 is default 1x*/
    sensor_info->SensorHightSampling = 0;	/* 0 is default 1x*/
    sensor_info->SensorPacketECCOrder = 1;

    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
                imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
                imgsensor_info.cap.mipi_data_lp2hs_settle_dc;
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
                imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
                imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
                imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;
            break;
        default:
            sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
            break;
    }
    return ERROR_NONE;
}   /*  get_info  */

static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
        MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("scenario_id = %d\n", scenario_id);
    spin_lock(&imgsensor_drv_lock);
    imgsensor.current_scenario_id = scenario_id;
    spin_unlock(&imgsensor_drv_lock);
    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            preview(image_window, sensor_config_data);
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            capture(image_window, sensor_config_data);
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            normal_video(image_window, sensor_config_data);
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            hs_video(image_window, sensor_config_data);
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            slim_video(image_window, sensor_config_data);
            break;
        default:
            LOG_INF("Error ScenarioId setting");
            preview(image_window, sensor_config_data);
            return ERROR_INVALID_SCENARIO_ID;
    }
    return ERROR_NONE;
}   /*  control()  */

static kal_uint32 set_video_mode(UINT16 framerate)
{
    LOG_INF("framerate = %d\n ", framerate);
    /* SetVideoMode Function should fix framerate*/
    if (framerate == 0)
        /* Dynamic frame rate*/
        return ERROR_NONE;
    spin_lock(&imgsensor_drv_lock);
    if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
        imgsensor.current_fps = 296;
    else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
        imgsensor.current_fps = 146;
    else
        imgsensor.current_fps = framerate;
    spin_unlock(&imgsensor_drv_lock);
    set_max_framerate(imgsensor.current_fps, 1);

    return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
    LOG_INF("enable = %d, framerate = %d\n", enable, framerate);
    spin_lock(&imgsensor_drv_lock);
    if (enable) /*enable auto flicker*/
        imgsensor.autoflicker_en = KAL_TRUE;
    else /*Cancel Auto flick*/
        imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    return ERROR_NONE;
}

static kal_uint32 set_max_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
    kal_uint32 frame_length;

    LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ?
                       (frame_length - imgsensor_info.pre.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            set_dummy();
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            if (framerate == 0)
                return ERROR_NONE;
            frame_length = imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength)
                       ? (frame_length - imgsensor_info.normal_video.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            set_dummy();
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
                LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
                           framerate, imgsensor_info.cap.max_framerate/10);
            frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ?
                (frame_length - imgsensor_info.cap.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            set_dummy();
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ?
                       (frame_length - imgsensor_info.hs_video.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            set_dummy();
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ?
                       (frame_length - imgsensor_info.slim_video.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            set_dummy();
            break;
        default:
            frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ?
                       (frame_length - imgsensor_info.pre.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            set_dummy();
            LOG_INF("error scenario_id = %d, we use preview scenario\n", scenario_id);
            break;
    }
    return ERROR_NONE;
}

static kal_uint32 get_default_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
    LOG_INF("scenario_id = %d\n", scenario_id);

    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            *framerate = imgsensor_info.pre.max_framerate;
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            *framerate = imgsensor_info.normal_video.max_framerate;
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            *framerate = imgsensor_info.cap.max_framerate;
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            *framerate = imgsensor_info.hs_video.max_framerate;
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            *framerate = imgsensor_info.slim_video.max_framerate;
            break;
        default:
            break;
    }

    return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
    LOG_INF("enable: %d\n", enable);

    if (enable) {
        /* 0x5E00[8]: 1 enable,  0 disable*/
        /* 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK*/
        write_cmos_sensor_16_16(0x0600, 0x0002);
    } else {
        /* 0x5E00[8]: 1 enable,  0 disable*/
        /* 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK*/
        write_cmos_sensor_16_16(0x0600, 0x0000);
    }
    spin_lock(&imgsensor_drv_lock);
    imgsensor.test_pattern = enable;
    spin_unlock(&imgsensor_drv_lock);
    return ERROR_NONE;
}

static void hdr_write_tri_shutter(kal_uint16 le, kal_uint16 me, kal_uint16 se)
{
    kal_uint16 realtime_fps = 0;

    LOG_INF("E! le:0x%x, me:0x%x, se:0x%x\n", le, me, se);
    spin_lock(&imgsensor_drv_lock);
    if (le > imgsensor.min_frame_length - imgsensor_info.margin)
        imgsensor.frame_length = le + imgsensor_info.margin;
    else
        imgsensor.frame_length = imgsensor.min_frame_length;
    if (imgsensor.frame_length > imgsensor_info.max_frame_length)
        imgsensor.frame_length = imgsensor_info.max_frame_length;
    spin_unlock(&imgsensor_drv_lock);
    if (le < imgsensor_info.min_shutter)
        le = imgsensor_info.min_shutter;

    if (imgsensor.autoflicker_en) {
        realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
        if (realtime_fps >= 297 && realtime_fps <= 305)
            set_max_framerate(296, 0);
        else if (realtime_fps >= 147 && realtime_fps <= 150)
            set_max_framerate(146, 0);
        else
            write_cmos_sensor_16_16(0x0340, imgsensor.frame_length & 0xFFFF);
    } else {
        write_cmos_sensor_16_16(0x0340, imgsensor.frame_length & 0xFFFF);
    }

    hdr_le = le;
    hdr_me = me;
    hdr_se = se;

    /* Long exposure */
    /*write_cmos_sensor_16_16(0x0226, le);*/
    /* Middle exposure */
    /*write_cmos_sensor_16_16(0x022c, me);*/
    /* Short exposure */
    /*write_cmos_sensor_16_16(0x0202, se);*/

    LOG_INF("imgsensor.frame_length:0x%x\n", imgsensor.frame_length);
    LOG_INF("L! le:0x%x, me:0x%x, se:0x%x\n", le, me, se);

}

static kal_uint16 gain2reg_dig(kal_uint16 gain, kal_uint16 g_gain)
{
    kal_uint16 dig_gain = 0x0;

    /* gain's base is 64, dig_gain's base is 256 */
    dig_gain = ((kal_uint32)gain*256/g_gain)&0xffff;
    LOG_INF("gain:%d,g_gain:%d,dig_gain=%d\n", gain, g_gain, dig_gain);
    return (kal_uint16)dig_gain;
}

static void hdr_write_tri_gain(kal_uint16 lg, kal_uint16 mg, kal_uint16 sg)
{
    kal_uint16 reg_lg_dig, reg_mg_dig, reg_sg_dig;
    kal_uint16 global_gain, reg_global_gain;

    /* should use analog(global) gain first */
    /* nosie would be obviously if totally use digital gain */
    global_gain = lg < mg?lg:mg;
    global_gain = global_gain < sg?global_gain:sg;

    if (global_gain < BASEGAIN || global_gain > 16 * BASEGAIN) {
        LOG_INF("Error gain setting");

        if (global_gain < BASEGAIN)
            global_gain = BASEGAIN;
        else if (global_gain > 16 * BASEGAIN)
            global_gain = 16 * BASEGAIN;
    }

    reg_global_gain = gain2reg(global_gain);
    reg_lg_dig = gain2reg_dig(lg, global_gain);
    reg_mg_dig = gain2reg_dig(mg, global_gain);
    reg_sg_dig = gain2reg_dig(sg, global_gain);

    spin_lock(&imgsensor_drv_lock);
    imgsensor.gain = reg_global_gain;
    spin_unlock(&imgsensor_drv_lock);

    write_cmos_sensor_16_8(0x0104, 0x01);
    /* Long expo Gian- digital gain, step=256 */
    write_cmos_sensor_16_16(0x0230, reg_lg_dig);
    /* Medium expo Gian */
    write_cmos_sensor_16_16(0x0238, reg_mg_dig);
    /* Short expo Gian */
    write_cmos_sensor_16_16(0x020E, reg_sg_dig);
    /* global gain - step=32 */
    write_cmos_sensor_16_16(0x0204, reg_global_gain);

    write_cmos_sensor_16_16(0x0226, hdr_le);
    write_cmos_sensor_16_16(0x022c, hdr_me);
    write_cmos_sensor_16_16(0x0202, hdr_se);
    write_cmos_sensor_16_8(0x0104, 0x00);

    LOG_INF("lg:0x%x, reg_lg_dig:0x%x, mg:0x%x, reg_mg_dig:0x%x, sg:0x%x, reg_sg_dig:0x%x\n",
            lg, reg_lg_dig, mg, reg_mg_dig, sg, reg_sg_dig);
    LOG_INF("reg_global_gain:0x%x\n", reg_global_gain);

}

static kal_uint32 set_awb_gain(struct SET_SENSOR_AWB_GAIN *pSetSensorAWB)
{
    UINT32 rgain_32, grgain_32, gbgain_32, bgain_32, ggain_32;
    UINT32 BASE_HDR = 2184;
    UINT32 BASE_WB = 256;
    UINT16 reg_rgain, reg_ggain, reg_bgain;

    grgain_32 = pSetSensorAWB->ABS_GAIN_GR;
    rgain_32  = pSetSensorAWB->ABS_GAIN_R;
    bgain_32  = pSetSensorAWB->ABS_GAIN_B;
    gbgain_32 = pSetSensorAWB->ABS_GAIN_GB;
    ggain_32  = (grgain_32 + gbgain_32) >> 1; /*Gr_gain = Gb_gain */
    LOG_INF("[set_awb_gain] rgain:%d, ggain:%d, bgain:%d\n",
            rgain_32, ggain_32, bgain_32);

    /* set WB gain when HDR/remosaic*/
    reg_rgain = (rgain_32*BASE_WB/512)&0xffff;
    reg_ggain = (ggain_32*BASE_WB/512)&0xffff;
    reg_bgain = (bgain_32*BASE_WB/512)&0xffff;
    LOG_INF("[BASE_WB=256] reg_rgain:%d, reg_ggain:%d, reg_bgain:%d\n",
            reg_rgain, reg_ggain, reg_bgain);

    write_cmos_sensor_16_16(0x0D12, reg_rgain);
    write_cmos_sensor_16_16(0x0D14, reg_ggain);
    write_cmos_sensor_16_16(0x0D16, reg_bgain);

    /*set weight gain when HDR*/
    if (imgsensor.sensor_mode != IMGSENSOR_MODE_CAPTURE) {
        reg_rgain = (rgain_32*BASE_HDR*5/16/512)&0x1fff;  /*max value is 8192 */
        reg_ggain = (ggain_32*BASE_HDR*9/16/512)&0x1fff;
        reg_bgain = (bgain_32*BASE_HDR*2/16/512)&0x1fff;
        LOG_INF("[BASE_HDR=2184] reg_rgain:%d, reg_ggain:%d, reg_bgain:%d\n",
                reg_rgain, reg_ggain, reg_bgain);

        write_cmos_sensor_16_16(0x6028, 0x2000);
        write_cmos_sensor_16_16(0x602A, 0x4B9C);
        write_cmos_sensor_16_16(0x6F12, reg_rgain);
        write_cmos_sensor_16_16(0x6F12, reg_ggain);
        write_cmos_sensor_16_16(0x6F12, reg_bgain); /* short expo's gain */
        write_cmos_sensor_16_16(0x602A, 0x4BAA);
        write_cmos_sensor_16_16(0x6F12, reg_rgain);
        write_cmos_sensor_16_16(0x6F12, reg_ggain);
        write_cmos_sensor_16_16(0x6F12, reg_bgain); /* long expo's gain */
        write_cmos_sensor_16_16(0x602A, 0x4BB8);
        write_cmos_sensor_16_16(0x6F12, reg_rgain);
        write_cmos_sensor_16_16(0x6F12, reg_ggain);
        write_cmos_sensor_16_16(0x6F12, reg_bgain); /* medium expo's gain */
        write_cmos_sensor_16_16(0x602A, 0x4BC6);
        write_cmos_sensor_16_16(0x6F12, reg_rgain);
        write_cmos_sensor_16_16(0x6F12, reg_ggain);
        write_cmos_sensor_16_16(0x6F12, reg_bgain); /* mixed expo's gain */
    }

    return ERROR_NONE;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
        UINT8 *feature_para, UINT32 *feature_para_len)
{
    UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
    UINT16 *feature_data_16 = (UINT16 *) feature_para;
    UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
    UINT32 *feature_data_32 = (UINT32 *) feature_para;
    unsigned long long *feature_data = (unsigned long long *) feature_para;
    struct SET_SENSOR_AWB_GAIN *pSetSensorAWB = (struct SET_SENSOR_AWB_GAIN *)feature_para;

    struct SET_PD_BLOCK_INFO_T *PDAFinfo;
    struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
    struct SENSOR_VC_INFO_STRUCT *pvcinfo;

    MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data = (MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

    LOG_INF("feature_id = %d\n", feature_id);
    switch (feature_id) {
        case SENSOR_FEATURE_GET_PERIOD:
            *feature_return_para_16++ = imgsensor.line_length;
            *feature_return_para_16 = imgsensor.frame_length;
            *feature_para_len = 4;
            break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
            *feature_return_para_32 = imgsensor.pclk;
            *feature_para_len = 4;
            break;
        case SENSOR_FEATURE_SET_ESHUTTER:
            set_shutter(*feature_data);
            break;
        case SENSOR_FEATURE_SET_NIGHTMODE:
            /*night_mode((BOOL) *feature_data);*/
            break;
        case SENSOR_FEATURE_SET_GAIN:
            set_gain((UINT16) *feature_data);
            break;
        case SENSOR_FEATURE_SET_FLASHLIGHT:
            break;
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
            break;
        case SENSOR_FEATURE_SET_REGISTER:
            write_cmos_sensor_16_16(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
            break;
        case SENSOR_FEATURE_GET_REGISTER:
            sensor_reg_data->RegData = read_cmos_sensor_16_16(sensor_reg_data->RegAddr);
            break;
        case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
            /* get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE*/
            /* if EEPROM does not exist in camera module.*/
            *feature_return_para_32 = LENS_DRIVER_ID_DO_NOT_CARE;
            *feature_para_len = 4;
            break;
        case SENSOR_FEATURE_SET_VIDEO_MODE:
            set_video_mode(*feature_data);
            break;
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            get_imgsensor_id(feature_return_para_32);
            break;
        case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
            set_auto_flicker_mode((BOOL)*feature_data_16, *(feature_data_16+1));
            break;
        case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
            set_max_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM) * feature_data, *(feature_data + 1));
            break;
        case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
            get_default_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM) * (feature_data),
                    (MUINT32 *)(uintptr_t)(*(feature_data+1)));
            break;
        case SENSOR_FEATURE_SET_TEST_PATTERN:
            set_test_pattern_mode((BOOL)*feature_data);
            break;
        case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: /*for factory mode auto testing*/
            *feature_return_para_32 = imgsensor_info.checksum_value;
            *feature_para_len = 4;
            break;
        case SENSOR_FEATURE_SET_FRAMERATE:
            spin_lock(&imgsensor_drv_lock);
            imgsensor.current_fps = *feature_data_32;
            spin_unlock(&imgsensor_drv_lock);
            break;
        case SENSOR_FEATURE_GET_CROP_INFO:
            LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", *feature_data_32);
            wininfo = (struct SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data + 1));

            switch (*feature_data_32) {
                case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                    memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[1], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                    memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[2], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
                    memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[3], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_SLIM_VIDEO:
                    memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[4], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
                default:
                    memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[0], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
                    break;
            }
            break;
        case SENSOR_FEATURE_GET_PDAF_INFO:
            LOG_INF("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%lld\n", *feature_data);
            PDAFinfo = (struct SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data + 1));

            switch (*feature_data) {
                case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
                #ifdef PDAF_TEST
                    memcpy((void *)PDAFinfo, (void *)&imgsensor_pd_info, sizeof(struct SET_PD_BLOCK_INFO_T));
                #endif
                    break;
                case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
                case MSDK_SCENARIO_ID_SLIM_VIDEO:
                default:
                    break;
            }
            break;
        case SENSOR_FEATURE_SET_PDAF:
            LOG_INF("PDAF mode :%d\n", *feature_data_16);
            #ifdef PDAF_TEST
            imgsensor.pdaf_mode = *feature_data_16;
            #endif
            break;
            // Antaiui <AI_BSP_CAM> <xieht> <2021-03-01> pdaf porting begin
        case SENSOR_FEATURE_GET_PDAF_DATA:
            /* no use, get pdaf data with DoCamCalPDAFAnt() in camera_calibration_cam_cal.cpp */
            LOG_INF("SENSOR_FEATURE_GET_PDAF_DATA\n");
            break;
            // Antaiui <AI_BSP_CAM> <xieht> <2021-03-01> pdaf porting end
        case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
            LOG_INF("SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%lld\n", *feature_data);
            /*PDAF capacity enable or not, 2p8 only full size support PDAF*/
            switch (*feature_data) {
                case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
                #ifdef PDAF_TEST
                    *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
                    break;
                #endif

                case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
                case MSDK_SCENARIO_ID_SLIM_VIDEO:
                default:
                    *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
                    break;
            }
            break;
        case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
            set_shutter_frame_length((UINT16) *feature_data, (UINT16) *(feature_data + 1));
            break;
        case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
            LOG_INF("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
            streaming_control(KAL_FALSE);
            break;
        case SENSOR_FEATURE_SET_STREAMING_RESUME:
            LOG_INF("SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n", *feature_data);
            if (*feature_data != 0)
                set_shutter(*feature_data);
            streaming_control(KAL_TRUE);
            break;
        case SENSOR_FEATURE_GET_AE_FRAME_MODE_FOR_LE:
            LOG_INF("SENSOR_FEATURE_GET_AE_FRAME_MODE_FOR_LE\n");
            memcpy(feature_return_para_32, &imgsensor.ae_frm_mode, sizeof(struct IMGSENSOR_AE_FRM_MODE));
            break;
        case SENSOR_FEATURE_GET_AE_EFFECTIVE_FRAME_FOR_LE:
            LOG_INF("SENSOR_FEATURE_GET_AE_EFFECTIVE_FRAME_FOR_LE\n");
            *feature_return_para_32 =  imgsensor.current_ae_effective_frame;
            break;
        case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
            {
                kal_uint32 rate;

                switch (*feature_data) {
                    case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                        rate = imgsensor_info.cap.mipi_pixel_rate;
                        break;
                    case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                        rate = imgsensor_info.normal_video.mipi_pixel_rate;
                        break;
                    case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
                        rate = imgsensor_info.hs_video.mipi_pixel_rate;
                        break;
                    case MSDK_SCENARIO_ID_SLIM_VIDEO:
                        rate = imgsensor_info.slim_video.mipi_pixel_rate;
                        break;
                    case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
                        rate = imgsensor_info.pre.mipi_pixel_rate;
                        break;
                    default:
                        rate = 0;
                        break;
                }
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = rate;
            }
            break;
        case SENSOR_FEATURE_SET_HDR:
            LOG_INF("hdr enable :%d\n", *feature_data_32);
            spin_lock(&imgsensor_drv_lock);
            imgsensor.hdr_mode = (UINT8)*feature_data_32;
            spin_unlock(&imgsensor_drv_lock);
            break;
        case SENSOR_FEATURE_SET_HDR_TRI_SHUTTER:
            LOG_INF("SENSOR_FEATURE_SET_HDR_TRI_SHUTTER LE=%d, SE=%d, ME=%d\n",
                    (UINT16) *feature_data,
                    (UINT16) *(feature_data + 1),
                    (UINT16) *(feature_data + 2));
            hdr_write_tri_shutter((UINT16)*feature_data,
                    (UINT16)*(feature_data+1),
                    (UINT16)*(feature_data+2));
            break;
        case SENSOR_FEATURE_SET_HDR_TRI_GAIN:
            LOG_INF("SENSOR_FEATURE_SET_HDR_TRI_GAIN LGain=%d, SGain=%d, MGain=%d\n",
                    (UINT16) *feature_data,
                    (UINT16) *(feature_data + 1),
                    (UINT16) *(feature_data + 2));
            hdr_write_tri_gain((UINT16)*feature_data, (UINT16)*(feature_data+1), (UINT16)*(feature_data+2));
            break;
            break;
        case SENSOR_FEATURE_SET_AWB_GAIN:
            set_awb_gain(pSetSensorAWB);
            break;
        case SENSOR_FEATURE_GET_VC_INFO:
            LOG_INF("SENSOR_FEATURE_GET_VC_INFO %d\n", (UINT16) *feature_data);
            pvcinfo = (struct SENSOR_VC_INFO_STRUCT *)(uintptr_t)(*(feature_data + 1));
            switch (*feature_data_32) {
                case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
                case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                    memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[0],
                            sizeof(struct SENSOR_VC_INFO_STRUCT));
                    break;
                default:
                    break;
            }
            break;
        case SENSOR_FEATURE_GET_SENSOR_HDR_CAPACITY:
            /*
             * SENSOR_VHDR_MODE_NONE  = 0x0,
             * SENSOR_VHDR_MODE_IVHDR = 0x01,
             * SENSOR_VHDR_MODE_MVHDR = 0x02,
             * SENSOR_VHDR_MODE_ZVHDR = 0x09
             * SENSOR_VHDR_MODE_4CELL_MVHDR = 0x0A
             */
            switch (*feature_data) {
                case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
                case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                    *(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0x02;
                    break;
                case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
                case MSDK_SCENARIO_ID_SLIM_VIDEO:
                default:
                    *(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0x0;
                    break;
            }
            LOG_INF("SENSOR_FEATURE_GET_SENSOR_HDR_CAPACITY scenarioId:%llu, HDR:%llu\n"
                    , *feature_data, *(feature_data+1));
            break;

        default:
            break;
    }

    return ERROR_NONE;
}   /*  feature_control()  */

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
    open,
    get_info,
    get_resolution,
    feature_control,
    control,
    close
};


UINT32 S5KGM1ST_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc != NULL)
        *pfFunc = &sensor_func;
    return ERROR_NONE;
}   /*  S5K2X5_MIPI_RAW_SensorInit  */
