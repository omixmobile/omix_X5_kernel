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
 *	 s5kjn1mipi_Sensor.c
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
#include "s5kjn1sq03mipiraw_Sensor.h"

#define PFX "S5KJN1_camera_sensor"
#define LOG_INF(format,  args...)	pr_info(PFX "[%s] " format,  __func__,  ##args)

static DEFINE_SPINLOCK(imgsensor_drv_lock);
static kal_uint8 chipID;
static kal_uint16 hdr_le, hdr_me, hdr_se;

static struct imgsensor_info_struct imgsensor_info = {
    .sensor_id = S5KJN1SQ03_SENSOR_ID,
    .checksum_value = 0xf5055885,
    .pre = {	// sync with cap
        .pclk = 560000000,
        .linelength = 5910,
        .framelength = 3156,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4080,
        .grabwindow_height = 3072,
        .mipi_data_lp2hs_settle_dc = 85,
        .max_framerate = 300,
        .mipi_pixel_rate = 494400000,
    },
    .cap = {	// sync with cap
        .pclk = 560000000,
        .linelength = 5910,
        .framelength = 3156,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4080,
        .grabwindow_height = 3072,
        .mipi_data_lp2hs_settle_dc = 85,
        .max_framerate = 300,
        .mipi_pixel_rate = 494400000,
    },
    .normal_video =  {	// sync with cap
        .pclk = 560000000,
        .linelength = 5910,
        .framelength = 3156,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4080,
        .grabwindow_height = 3072,
        .mipi_data_lp2hs_settle_dc = 85,
        .max_framerate = 300,
        .mipi_pixel_rate = 494400000,
    },
    .hs_video = {
        .pclk = 600000000,
        .linelength = 2096,
        .framelength = 2380,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 1280,
        .grabwindow_height = 720,
        .mipi_data_lp2hs_settle_dc = 85,
        .max_framerate = 1200,
        .mipi_pixel_rate = 640000000,
    },
    .slim_video =  {
        .pclk = 560000000,
        .linelength = 5910,
        .framelength = 3156,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4080,
        .grabwindow_height = 3072,
        .mipi_data_lp2hs_settle_dc = 85,
        .max_framerate = 300,
        .mipi_pixel_rate = 480000000,
    },
    .custom1 =  {   // 3840x2880@30fps for vsdof
        .pclk = 560000000,
        .linelength = 7712,
        .framelength = 5792,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 3840,
        .grabwindow_height = 2880,
        .mipi_data_lp2hs_settle_dc = 85,
        .max_framerate = 300,
        .mipi_pixel_rate = 480000000,
    },

    .margin = 5,
    .min_shutter = 4,
    //Antaiui <AI_BSP_CAM> <yaoyc> <2020-12-01> add for 25571 android R camera hal request begin
    .min_gain = 64,
    .max_gain = 1024,
    .min_gain_iso = 100,
    //.exp_step = 2,
    .gain_step = 2,
    .gain_type = 2,
    //Antaiui <AI_BSP_CAM> <yaoyc> <2020-12-01> add for 25571 android R camera hal request end
    .max_frame_length = 0xffff,
    .ae_shut_delay_frame = 0,
    .ae_sensor_gain_delay_frame = 0,
    .ae_ispGain_delay_frame = 2,
    // Antaiui <AI_BSP_CAM> <xieht> <2021-03-12> frame time delay frame begin
    .frame_time_delay_frame = 1,
    // Antaiui <AI_BSP_CAM> <xieht> <2021-03-12> frame time delay frame end
    .ihdr_support = 0,	  /*1, support; 0,not support*/
    .ihdr_le_firstline = 0,  /*1,le first; 0, se first*/
    .sensor_mode_num = 6,	  /*support sensor mode num*/

    .cap_delay_frame = 2,/*3 guanjd modify for cts*/
    .pre_delay_frame = 2,/*3 guanjd modify for cts*/
    .video_delay_frame = 3,
    .hs_video_delay_frame = 3,
    .slim_video_delay_frame = 3,
    .custom1_delay_frame = 3,

    .isp_driving_current = ISP_DRIVING_4MA,
    .sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
    .mipi_sensor_type = MIPI_OPHY_NCSI2, /*0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2*/
    .mipi_settle_delay_mode = 1, /*0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL*/
    .sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gr,//SENSOR_OUTPUT_FORMAT_RAW_Gr,//SENSOR_OUTPUT_FORMAT_RAW_R,//SENSOR_OUTPUT_FORMAT_RAW_B,
    .mclk = 24,
    .mipi_lane_num = SENSOR_MIPI_4_LANE,
    .i2c_addr_table = {0x20,0xff},
    .i2c_speed = 1000,
};

static struct imgsensor_struct imgsensor = {
    .mirror = IMAGE_NORMAL,				//mirrorflip information
    .sensor_mode = IMGSENSOR_MODE_INIT,
    .shutter = 0x200,					//current shutter
    .gain = 0x200,						//current gain
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
// Antaiui <AI_BSP_CAM> <xieht> <2022-03-09> modify for its fov begin
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[6] = {
    { 4080, 3072,    0,    0, 4080, 3072, 4080, 3072,   0,   0, 4080, 3072,  0,  0, 4080, 3072}, /* preview sync with capture */
    { 4080, 3072,    0,    0, 4080, 3072, 4080, 3072,   0,   0, 4080, 3072,  0,  0, 4080, 3072}, /* capture */
    { 4080, 3072,    0,    0, 4080, 3072, 4080, 3072,   0,   0, 4080, 3072,  0,  0, 4080, 3072}, /* normal video sync with capture */
    { 4080, 3072,  760,  816, 2560, 1440, 1280,  720,   0,   0, 1280,  720,  0,  0, 1280,  720}, /* hs video */
    { 4080, 3072,    0,    0, 4080, 3072, 4080, 3072,   0,   0, 4080, 3072,  0,  0, 4080, 3072}, /* slim video */
    { 4080, 3072,  120,   96, 3840, 2880, 3840, 2880,   0,   0, 3840, 2880,  0,  0, 3840, 2880}, /* custom1 */
};
// Antaiui <AI_BSP_CAM> <xieht> <2022-03-09> modify for its fov end

#define PDAF_TEST

#ifdef PDAF_TEST
// Antaiui <AI_BSP_CAM> <xieht> <2021-03-01> pdaf porting begin
static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info =
{
    .i4OffsetX = 8,
    .i4OffsetY = 8,
    .i4PitchX  = 8,
    .i4PitchY  = 8,
    .i4PairNum  = 4,
    .i4SubBlkW  = 8,
    .i4SubBlkH  = 2,
    .i4BlockNumX = 508,
    .i4BlockNumY = 382,
    .i4PosR = {
        {10, 8}, {8, 11}, {12, 12}, {14, 15}
    },
    .i4PosL = {
        {11, 8}, {9, 11}, {13, 12}, {15, 15}
    },
    .iMirrorFlip = 0,
};
#endif

// all vc2 sync with capture
static struct SENSOR_VC_INFO_STRUCT SENSOR_VC_INFO[3] = {
    /* Preview mode setting */
    /* Capture mode setting */
    {0x02, 0x0A,   0x00,   0x08, 0x40, 0x00,
        0x00, 0x2B, 0x0FF0, 0x0C00, 0x01, 0x00, 0x0000, 0x0000,
        0x01, 0x30, 0x027C, 0x0BF0, 0x03, 0x00, 0x0000, 0x0000},
    /* Video mode setting */
    {0x02, 0x0A,   0x00,   0x08, 0x40, 0x00,
        0x00, 0x2B, 0x0FF0, 0x0C00, 0x01, 0x00, 0x0000, 0x0000,
        0x01, 0x30, 0x027C, 0x08E8, 0x03, 0x00, 0x0000, 0x0000},
    /* Custom1 mode setting */
    {0x02, 0x0A,   0x00,   0x08, 0x40, 0x00,
        0x00, 0x2B, 0x0FF0, 0x0C00, 0x01, 0x00, 0x0000, 0x0000,
        0x01, 0x30, 0x0200, 0x0980, 0x03, 0x00, 0x0000, 0x0000}
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
static void set_shutter_frame_length(kal_uint16 shutter, kal_uint16 frame_length)
{
    unsigned long flags;
    kal_uint16 realtime_fps = 0;
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

    /* Update Shutter */
    write_cmos_sensor_16_16(0X0202, shutter & 0xFFFF);

    LOG_INF("shutter = %d, framelength = %d/%d, dummy_line= %d\n", shutter, imgsensor.frame_length, frame_length, dummy_line);
}  /* set_shutter_frame_length */
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
        write_cmos_sensor_16_8(0x0101, 0x00); /* Gr */
        break;
    case IMAGE_H_MIRROR:
        write_cmos_sensor_16_8(0x0101, 0x01);
        break;
    case IMAGE_V_MIRROR:
        write_cmos_sensor_16_8(0x0101, 0x02);
        break;
    case IMAGE_HV_MIRROR:
        write_cmos_sensor_16_8(0x0101, 0x03); /* Gb */
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
    0x6028,0x2400,
    0x602A,0x7700,
    0x6F12,0x1753,
    0x6F12,0x01FC,
    0x6F12,0xE702,
    0x6F12,0x6385,
    0x6F12,0xB787,
    0x6F12,0x0024,
    0x6F12,0x9386,
    0x6F12,0xC701,
    0x6F12,0xB7B7,
    0x6F12,0x0024,
    0x6F12,0x3777,
    0x6F12,0x0024,
    0x6F12,0x9387,
    0x6F12,0xC77F,
    0x6F12,0x1307,
    0x6F12,0x07C7,
    0x6F12,0x958F,
    0x6F12,0x2328,
    0x6F12,0xD774,
    0x6F12,0x231A,
    0x6F12,0xF774,
    0x6F12,0x012F,
    0x6F12,0xB777,
    0x6F12,0x0024,
    0x6F12,0x1307,
    0x6F12,0xD003,
    0x6F12,0x23A0,
    0x6F12,0xE774,
    0x6F12,0x1753,
    0x6F12,0x01FC,
    0x6F12,0x6700,
    0x6F12,0x2384,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x8547,
    0x6F12,0x6310,
    0x6F12,0xF506,
    0x6F12,0x3786,
    0x6F12,0x0024,
    0x6F12,0x9306,
    0x6F12,0x4601,
    0x6F12,0x83C7,
    0x6F12,0x4600,
    0x6F12,0xA1CB,
    0x6F12,0xB737,
    0x6F12,0x0024,
    0x6F12,0x83A7,
    0x6F12,0x875C,
    0x6F12,0x83D6,
    0x6F12,0x2600,
    0x6F12,0x83D7,
    0x6F12,0x271E,
    0x6F12,0x13D7,
    0x6F12,0x2700,
    0x6F12,0xB707,
    0x6F12,0x0140,
    0x6F12,0x83D5,
    0x6F12,0x27F0,
    0x6F12,0x8357,
    0x6F12,0x4601,
    0x6F12,0x1306,
    0x6F12,0xE7FF,
    0x6F12,0xB697,
    0x6F12,0x8D8F,
    0x6F12,0xC207,
    0x6F12,0xC183,
    0x6F12,0x9396,
    0x6F12,0x0701,
    0x6F12,0xC186,
    0x6F12,0x635F,
    0x6F12,0xD600,
    0x6F12,0x8907,
    0x6F12,0x998F,
    0x6F12,0x9396,
    0x6F12,0x0701,
    0x6F12,0xC186,
    0x6F12,0x9397,
    0x6F12,0x0601,
    0x6F12,0xC183,
    0x6F12,0x37B7,
    0x6F12,0x0040,
    0x6F12,0x2311,
    0x6F12,0xF7A0,
    0x6F12,0x8280,
    0x6F12,0xE3D8,
    0x6F12,0x06FE,
    0x6F12,0xBA97,
    0x6F12,0xF917,
    0x6F12,0xCDB7,
    0x6F12,0xB717,
    0x6F12,0x0024,
    0x6F12,0x9387,
    0x6F12,0x07CA,
    0x6F12,0xAA97,
    0x6F12,0x3387,
    0x6F12,0xB700,
    0x6F12,0x8D8F,
    0x6F12,0x83C5,
    0x6F12,0xD705,
    0x6F12,0xB747,
    0x6F12,0x0024,
    0x6F12,0x9386,
    0x6F12,0x078F,
    0x6F12,0x83D7,
    0x6F12,0xE670,
    0x6F12,0x0905,
    0x6F12,0x0347,
    0x6F12,0xC705,
    0x6F12,0x630B,
    0x6F12,0xF500,
    0x6F12,0x8567,
    0x6F12,0xB697,
    0x6F12,0x03A6,
    0x6F12,0x8794,
    0x6F12,0xB306,
    0x6F12,0xB700,
    0x6F12,0xB296,
    0x6F12,0x23A4,
    0x6F12,0xD794,
    0x6F12,0x2207,
    0x6F12,0x3305,
    0x6F12,0xB700,
    0x6F12,0x4205,
    0x6F12,0x4181,
    0x6F12,0x8280,
    0x6F12,0x5D71,
    0x6F12,0xA2C6,
    0x6F12,0xA6C4,
    0x6F12,0x7324,
    0x6F12,0x2034,
    0x6F12,0xF324,
    0x6F12,0x1034,
    0x6F12,0x7360,
    0x6F12,0x0430,
    0x6F12,0x2AD8,
    0x6F12,0x2ED6,
    0x6F12,0x3545,
    0x6F12,0x9305,
    0x6F12,0x8008,
    0x6F12,0x22DA,
    0x6F12,0x3ECE,
    0x6F12,0x86C2,
    0x6F12,0x96C0,
    0x6F12,0x1ADE,
    0x6F12,0x1EDC,
    0x6F12,0x32D4,
    0x6F12,0x36D2,
    0x6F12,0x3AD0,
    0x6F12,0x42CC,
    0x6F12,0x46CA,
    0x6F12,0x72C8,
    0x6F12,0x76C6,
    0x6F12,0x7AC4,
    0x6F12,0x7EC2,
    0x6F12,0x9710,
    0x6F12,0x00FC,
    0x6F12,0xE780,
    0x6F12,0x40F5,
    0x6F12,0x9377,
    0x6F12,0x8500,
    0x6F12,0x2A84,
    0x6F12,0x85C3,
    0x6F12,0xB737,
    0x6F12,0x0024,
    0x6F12,0x9387,
    0x6F12,0x077C,
    0x6F12,0x03D7,
    0x6F12,0x6702,
    0x6F12,0x0507,
    0x6F12,0x2393,
    0x6F12,0xE702,
    0x6F12,0x9710,
    0x6F12,0x00FC,
    0x6F12,0xE780,
    0x6F12,0x00ED,
    0x6F12,0x0545,
    0x6F12,0xD535,
    0x6F12,0x1374,
    0x6F12,0x0408,
    0x6F12,0x11CC,
    0x6F12,0xB737,
    0x6F12,0x0024,
    0x6F12,0x9387,
    0x6F12,0x077C,
    0x6F12,0x03D7,
    0x6F12,0x8705,
    0x6F12,0x0507,
    0x6F12,0x239C,
    0x6F12,0xE704,
    0x6F12,0x9710,
    0x6F12,0x00FC,
    0x6F12,0xE780,
    0x6F12,0x6065,
    0x6F12,0x9640,
    0x6F12,0x8642,
    0x6F12,0x7253,
    0x6F12,0xE253,
    0x6F12,0x5254,
    0x6F12,0x4255,
    0x6F12,0xB255,
    0x6F12,0x2256,
    0x6F12,0x9256,
    0x6F12,0x0257,
    0x6F12,0xF247,
    0x6F12,0x6248,
    0x6F12,0xD248,
    0x6F12,0x424E,
    0x6F12,0xB24E,
    0x6F12,0x224F,
    0x6F12,0x924F,
    0x6F12,0x7370,
    0x6F12,0x0430,
    0x6F12,0x7390,
    0x6F12,0x1434,
    0x6F12,0x7310,
    0x6F12,0x2434,
    0x6F12,0x3644,
    0x6F12,0xA644,
    0x6F12,0x6161,
    0x6F12,0x7300,
    0x6F12,0x2030,
    0x6F12,0x1743,
    0x6F12,0x01FC,
    0x6F12,0xE702,
    0x6F12,0xC369,
    0x6F12,0xB787,
    0x6F12,0x0024,
    0x6F12,0x83A4,
    0x6F12,0xC700,
    0x6F12,0x2A84,
    0x6F12,0x0146,
    0x6F12,0xA685,
    0x6F12,0x1145,
    0x6F12,0x9790,
    0x6F12,0xFFFB,
    0x6F12,0xE780,
    0x6F12,0x60AD,
    0x6F12,0x2285,
    0x6F12,0x97E0,
    0x6F12,0xFFFB,
    0x6F12,0xE780,
    0x6F12,0x40A9,
    0x6F12,0xB787,
    0x6F12,0x0024,
    0x6F12,0x9387,
    0x6F12,0x87F5,
    0x6F12,0x03C7,
    0x6F12,0x0700,
    0x6F12,0x8546,
    0x6F12,0x6315,
    0x6F12,0xD706,
    0x6F12,0xB776,
    0x6F12,0x0024,
    0x6F12,0x03A6,
    0x6F12,0x468D,
    0x6F12,0x8945,
    0x6F12,0x9386,
    0x6F12,0xA700,
    0x6F12,0x630F,
    0x6F12,0xB600,
    0x6F12,0x9386,
    0x6F12,0x2700,
    0x6F12,0x630B,
    0x6F12,0xE600,
    0x6F12,0x3717,
    0x6F12,0x0024,
    0x6F12,0x0356,
    0x6F12,0x6738,
    0x6F12,0x2D47,
    0x6F12,0x6314,
    0x6F12,0xE600,
    0x6F12,0x9386,
    0x6F12,0xA700,
    0x6F12,0xB747,
    0x6F12,0x0024,
    0x6F12,0x83A5,
    0x6F12,0xC786,
    0x6F12,0x2946,
    0x6F12,0x1305,
    0x6F12,0x0404,
    0x6F12,0x83C7,
    0x6F12,0xC52F,
    0x6F12,0x2148,
    0x6F12,0x1D8E,
    0x6F12,0x8147,
    0x6F12,0x3387,
    0x6F12,0xF500,
    0x6F12,0xB388,
    0x6F12,0xF600,
    0x6F12,0x0317,
    0x6F12,0xE73C,
    0x6F12,0x8398,
    0x6F12,0x0800,
    0x6F12,0x8907,
    0x6F12,0x1105,
    0x6F12,0x4697,
    0x6F12,0x3317,
    0x6F12,0xC700,
    0x6F12,0x232E,
    0x6F12,0xE5FE,
    0x6F12,0xE391,
    0x6F12,0x07FF,
    0x6F12,0x0546,
    0x6F12,0xA685,
    0x6F12,0x1145,
    0x6F12,0x9790,
    0x6F12,0xFFFB,
    0x6F12,0xE780,
    0x6F12,0x60A4,
    0x6F12,0x1743,
    0x6F12,0x01FC,
    0x6F12,0x6700,
    0x6F12,0x0361,
    0x6F12,0x1743,
    0x6F12,0x01FC,
    0x6F12,0xE702,
    0x6F12,0xA35C,
    0x6F12,0xB787,
    0x6F12,0x0024,
    0x6F12,0x83A4,
    0x6F12,0x8700,
    0x6F12,0x4111,
    0x6F12,0xAA89,
    0x6F12,0x2E8A,
    0x6F12,0xB28A,
    0x6F12,0xA685,
    0x6F12,0x0146,
    0x6F12,0x1145,
    0x6F12,0x36C6,
    0x6F12,0x9790,
    0x6F12,0xFFFB,
    0x6F12,0xE780,
    0x6F12,0x60A1,
    0x6F12,0xB787,
    0x6F12,0x0024,
    0x6F12,0x1387,
    0x6F12,0x87F5,
    0x6F12,0x0347,
    0x6F12,0x2701,
    0x6F12,0x1384,
    0x6F12,0x87F5,
    0x6F12,0xB246,
    0x6F12,0x0149,
    0x6F12,0x11CF,
    0x6F12,0x3767,
    0x6F12,0x0024,
    0x6F12,0x0357,
    0x6F12,0x2777,
    0x6F12,0xB777,
    0x6F12,0x0024,
    0x6F12,0x9387,
    0x6F12,0x07C7,
    0x6F12,0x0E07,
    0x6F12,0x03D9,
    0x6F12,0x871C,
    0x6F12,0x2394,
    0x6F12,0xE71C,
    0x6F12,0x5686,
    0x6F12,0xD285,
    0x6F12,0x4E85,
    0x6F12,0x97E0,
    0x6F12,0xFFFB,
    0x6F12,0xE780,
    0x6F12,0x0088,
    0x6F12,0x8347,
    0x6F12,0x2401,
    0x6F12,0x89C7,
    0x6F12,0xB777,
    0x6F12,0x0024,
    0x6F12,0x239C,
    0x6F12,0x27E3,
    0x6F12,0x0546,
    0x6F12,0xA685,
    0x6F12,0x1145,
    0x6F12,0x9790,
    0x6F12,0xFFFB,
    0x6F12,0xE780,
    0x6F12,0xC09B,
    0x6F12,0x4101,
    0x6F12,0x1743,
    0x6F12,0x01FC,
    0x6F12,0x6700,
    0x6F12,0xA357,
    0x6F12,0x1743,
    0x6F12,0x01FC,
    0x6F12,0xE702,
    0x6F12,0x0353,
    0x6F12,0x3784,
    0x6F12,0x0024,
    0x6F12,0x9307,
    0x6F12,0x04F4,
    0x6F12,0x83C7,
    0x6F12,0x4701,
    0x6F12,0x5D71,
    0x6F12,0x2A89,
    0x6F12,0x8DEF,
    0x6F12,0xB787,
    0x6F12,0x0024,
    0x6F12,0x03A4,
    0x6F12,0x4700,
    0x6F12,0x0146,
    0x6F12,0x1145,
    0x6F12,0xA285,
    0x6F12,0x9790,
    0x6F12,0xFFFB,
    0x6F12,0xE780,
    0x6F12,0x2098,
    0x6F12,0x4A85,
    0x6F12,0x97E0,
    0x6F12,0xFFFB,
    0x6F12,0xE780,
    0x6F12,0x400F,
    0x6F12,0x0546,
    0x6F12,0xA285,
    0x6F12,0x1145,
    0x6F12,0x9790,
    0x6F12,0xFFFB,
    0x6F12,0xE780,
    0x6F12,0xA096,
    0x6F12,0x6161,
    0x6F12,0x1743,
    0x6F12,0x01FC,
    0x6F12,0x6700,
    0x6F12,0xE351,
    0x6F12,0x1304,
    0x6F12,0x04F4,
    0x6F12,0x0347,
    0x6F12,0x5401,
    0x6F12,0x8547,
    0x6F12,0x6310,
    0x6F12,0xF716,
    0x6F12,0xB785,
    0x6F12,0x0024,
    0x6F12,0x9384,
    0x6F12,0x05F8,
    0x6F12,0x4146,
    0x6F12,0x9385,
    0x6F12,0x05F8,
    0x6F12,0x0A85,
    0x6F12,0x9740,
    0x6F12,0x01FC,
    0x6F12,0xE780,
    0x6F12,0x6058,
    0x6F12,0x4146,
    0x6F12,0x9385,
    0x6F12,0x0401,
    0x6F12,0x0808,
    0x6F12,0x9740,
    0x6F12,0x01FC,
    0x6F12,0xE780,
    0x6F12,0x6057,
    0x6F12,0x4146,
    0x6F12,0x9385,
    0x6F12,0x0402,
    0x6F12,0x0810,
    0x6F12,0x9740,
    0x6F12,0x01FC,
    0x6F12,0xE780,
    0x6F12,0x6056,
    0x6F12,0x4146,
    0x6F12,0x9385,
    0x6F12,0x0403,
    0x6F12,0x0818,
    0x6F12,0x9740,
    0x6F12,0x01FC,
    0x6F12,0xE780,
    0x6F12,0x6055,
    0x6F12,0x1C40,
    0x6F12,0xB7DB,
    0x6F12,0x0040,
    0x6F12,0x014B,
    0x6F12,0xBEC0,
    0x6F12,0x5C40,
    0x6F12,0x8149,
    0x6F12,0x854A,
    0x6F12,0xBEC2,
    0x6F12,0x5C44,
    0x6F12,0x096D,
    0x6F12,0x370C,
    0x6F12,0x0040,
    0x6F12,0xBEC4,
    0x6F12,0x1C48,
    0x6F12,0x938B,
    0x6F12,0x0B03,
    0x6F12,0x930C,
    0x6F12,0x0004,
    0x6F12,0xBEC6,
    0x6F12,0x0347,
    0x6F12,0x7401,
    0x6F12,0x8967,
    0x6F12,0x631B,
    0x6F12,0x5701,
    0x6F12,0x93F7,
    0x6F12,0xF900,
    0x6F12,0x9808,
    0x6F12,0xBA97,
    0x6F12,0x83C7,
    0x6F12,0x07FB,
    0x6F12,0x8A07,
    0x6F12,0xCA97,
    0x6F12,0x9C43,
    0x6F12,0x63DE,
    0x6F12,0x3A01,
    0x6F12,0x1387,
    0x6F12,0x69FE,
    0x6F12,0x63FA,
    0x6F12,0xEA00,
    0x6F12,0x1387,
    0x6F12,0xA9FD,
    0x6F12,0x63F6,
    0x6F12,0xEA00,
    0x6F12,0x1387,
    0x6F12,0x49FC,
    0x6F12,0x63E3,
    0x6F12,0xEA0A,
    0x6F12,0x131A,
    0x6F12,0x1B00,
    0x6F12,0x9808,
    0x6F12,0x5297,
    0x6F12,0x8354,
    0x6F12,0x07FF,
    0x6F12,0x0145,
    0x6F12,0xB384,
    0x6F12,0xF402,
    0x6F12,0xB580,
    0x6F12,0x637A,
    0x6F12,0x9D00,
    0x6F12,0x2685,
    0x6F12,0x9780,
    0x6F12,0xFFFB,
    0x6F12,0xE780,
    0x6F12,0x6052,
    0x6F12,0x4915,
    0x6F12,0x1375,
    0x6F12,0xF50F,
    0x6F12,0x9C08,
    0x6F12,0xD297,
    0x6F12,0x03D7,
    0x6F12,0x07FE,
    0x6F12,0xB3D4,
    0x6F12,0xA400,
    0x6F12,0xC204,
    0x6F12,0x6297,
    0x6F12,0xC180,
    0x6F12,0x2310,
    0x6F12,0x9700,
    0x6F12,0x03D7,
    0x6F12,0x07FC,
    0x6F12,0x83D7,
    0x6F12,0x07FD,
    0x6F12,0x050B,
    0x6F12,0x6297,
    0x6F12,0x8356,
    0x6F12,0x0700,
    0x6F12,0x3315,
    0x6F12,0xF500,
    0x6F12,0x558D,
    0x6F12,0x4205,
    0x6F12,0x4181,
    0x6F12,0x2310,
    0x6F12,0xA700,
    0x6F12,0x8509,
    0x6F12,0xE395,
    0x6F12,0x99F7,
    0x6F12,0x8D47,
    0x6F12,0x37D4,
    0x6F12,0x0040,
    0x6F12,0x2319,
    0x6F12,0xF40A,
    0x6F12,0x2316,
    0x6F12,0x040C,
    0x6F12,0x9790,
    0x6F12,0x00FC,
    0x6F12,0xE780,
    0x6F12,0x6093,
    0x6F12,0xAA84,
    0x6F12,0x9790,
    0x6F12,0x00FC,
    0x6F12,0xE780,
    0x6F12,0x2092,
    0x6F12,0x9307,
    0x6F12,0x0008,
    0x6F12,0x33D5,
    0x6F12,0xA700,
    0x6F12,0x9307,
    0x6F12,0x0004,
    0x6F12,0x1205,
    0x6F12,0xB3D7,
    0x6F12,0x9700,
    0x6F12,0x1375,
    0x6F12,0x0503,
    0x6F12,0x8D8B,
    0x6F12,0x5D8D,
    0x6F12,0x2319,
    0x6F12,0xA40C,
    0x6F12,0x45B5,
    0x6F12,0x1397,
    0x6F12,0x1900,
    0x6F12,0x9394,
    0x6F12,0x0701,
    0x6F12,0x5E97,
    0x6F12,0xC180,
    0x6F12,0x2310,
    0x6F12,0x9700,
    0x6F12,0x6DB7,
    0x6F12,0x0347,
    0x6F12,0x6401,
    0x6F12,0xE315,
    0x6F12,0xF7FA,
    0x6F12,0xB784,
    0x6F12,0x0024,
    0x6F12,0x9384,
    0x6F12,0x04F8,
    0x6F12,0x4146,
    0x6F12,0x9385,
    0x6F12,0x0404,
    0x6F12,0x0A85,
    0x6F12,0x9740,
    0x6F12,0x01FC,
    0x6F12,0xE780,
    0x6F12,0x2042,
    0x6F12,0x4146,
    0x6F12,0x9385,
    0x6F12,0x0405,
    0x6F12,0x0808,
    0x6F12,0x9740,
    0x6F12,0x01FC,
    0x6F12,0xE780,
    0x6F12,0x2041,
    0x6F12,0x4146,
    0x6F12,0x9385,
    0x6F12,0x0406,
    0x6F12,0x0810,
    0x6F12,0x9740,
    0x6F12,0x01FC,
    0x6F12,0xE780,
    0x6F12,0x2040,
    0x6F12,0x4146,
    0x6F12,0x9385,
    0x6F12,0x0407,
    0x6F12,0x0818,
    0x6F12,0x9740,
    0x6F12,0x01FC,
    0x6F12,0xE780,
    0x6F12,0x203F,
    0x6F12,0x0357,
    0x6F12,0x0400,
    0x6F12,0x8357,
    0x6F12,0x2400,
    0x6F12,0x37DB,
    0x6F12,0x0040,
    0x6F12,0x2310,
    0x6F12,0xE104,
    0x6F12,0x2311,
    0x6F12,0xF104,
    0x6F12,0x2312,
    0x6F12,0xE104,
    0x6F12,0x2313,
    0x6F12,0xF104,
    0x6F12,0x0357,
    0x6F12,0x8400,
    0x6F12,0x8357,
    0x6F12,0xA400,
    0x6F12,0x814A,
    0x6F12,0x2314,
    0x6F12,0xE104,
    0x6F12,0x2315,
    0x6F12,0xF104,
    0x6F12,0x2316,
    0x6F12,0xE104,
    0x6F12,0x2317,
    0x6F12,0xF104,
    0x6F12,0x8149,
    0x6F12,0x854B,
    0x6F12,0x096D,
    0x6F12,0x130B,
    0x6F12,0x0B03,
    0x6F12,0x370C,
    0x6F12,0x0040,
    0x6F12,0x930C,
    0x6F12,0x0004,
    0x6F12,0x0347,
    0x6F12,0x7401,
    0x6F12,0x8967,
    0x6F12,0x631B,
    0x6F12,0x7701,
    0x6F12,0x93F7,
    0x6F12,0xF900,
    0x6F12,0x9808,
    0x6F12,0xBA97,
    0x6F12,0x83C7,
    0x6F12,0x07FB,
    0x6F12,0x8A07,
    0x6F12,0xCA97,
    0x6F12,0x9C43,
    0x6F12,0x63C4,
    0x6F12,0x3B07,
    0x6F12,0x139A,
    0x6F12,0x1A00,
    0x6F12,0x9808,
    0x6F12,0x5297,
    0x6F12,0x8354,
    0x6F12,0x07FF,
    0x6F12,0x0145,
    0x6F12,0xB384,
    0x6F12,0xF402,
    0x6F12,0xB580,
    0x6F12,0x637A,
    0x6F12,0x9D00,
    0x6F12,0x2685,
    0x6F12,0x9780,
    0x6F12,0xFFFB,
    0x6F12,0xE780,
    0x6F12,0xA03B,
    0x6F12,0x4915,
    0x6F12,0x1375,
    0x6F12,0xF50F,
    0x6F12,0x9C08,
    0x6F12,0xD297,
    0x6F12,0x03D7,
    0x6F12,0x07FE,
    0x6F12,0xB3D4,
    0x6F12,0xA400,
    0x6F12,0xC204,
    0x6F12,0x6297,
    0x6F12,0xC180,
    0x6F12,0x2310,
    0x6F12,0x9700,
    0x6F12,0x03D7,
    0x6F12,0x07FC,
    0x6F12,0x83D7,
    0x6F12,0x07FD,
    0x6F12,0x850A,
    0x6F12,0x6297,
    0x6F12,0x8356,
    0x6F12,0x0700,
    0x6F12,0x3315,
    0x6F12,0xF500,
    0x6F12,0x558D,
    0x6F12,0x4205,
    0x6F12,0x4181,
    0x6F12,0x2310,
    0x6F12,0xA700,
    0x6F12,0x8509,
    0x6F12,0xE391,
    0x6F12,0x99F9,
    0x6F12,0x51BD,
    0x6F12,0x1397,
    0x6F12,0x1900,
    0x6F12,0x9394,
    0x6F12,0x0701,
    0x6F12,0x5A97,
    0x6F12,0xC180,
    0x6F12,0x2310,
    0x6F12,0x9700,
    0x6F12,0xE5B7,
    0x6F12,0x1743,
    0x6F12,0x01FC,
    0x6F12,0xE702,
    0x6F12,0xE326,
    0x6F12,0xB737,
    0x6F12,0x0024,
    0x6F12,0x83A7,
    0x6F12,0x0761,
    0x6F12,0xAA84,
    0x6F12,0x2E89,
    0x6F12,0x8297,
    0x6F12,0xB787,
    0x6F12,0x0024,
    0x6F12,0x03A4,
    0x6F12,0x0700,
    0x6F12,0x0146,
    0x6F12,0x1145,
    0x6F12,0xA285,
    0x6F12,0x9780,
    0x6F12,0xFFFB,
    0x6F12,0xE780,
    0x6F12,0xC069,
    0x6F12,0xCA85,
    0x6F12,0x2685,
    0x6F12,0x9730,
    0x6F12,0x00FC,
    0x6F12,0xE780,
    0x6F12,0x20F5,
    0x6F12,0x0546,
    0x6F12,0xA285,
    0x6F12,0x1145,
    0x6F12,0x9780,
    0x6F12,0xFFFB,
    0x6F12,0xE780,
    0x6F12,0x2068,
    0x6F12,0x1743,
    0x6F12,0x01FC,
    0x6F12,0x6700,
    0x6F12,0xC324,
    0x6F12,0xB717,
    0x6F12,0x0024,
    0x6F12,0x83C7,
    0x6F12,0x0734,
    0x6F12,0xEDCF,
    0x6F12,0x1743,
    0x6F12,0x01FC,
    0x6F12,0xE702,
    0x6F12,0x6321,
    0x6F12,0x9780,
    0x6F12,0x00FC,
    0x6F12,0xE780,
    0x6F12,0x803E,
    0x6F12,0xB787,
    0x6F12,0x0024,
    0x6F12,0x1387,
    0x6F12,0x87F5,
    0x6F12,0x0347,
    0x6F12,0xF701,
    0x6F12,0x9387,
    0x6F12,0x87F5,
    0x6F12,0x19EB,
    0x6F12,0x37F7,
    0x6F12,0x0040,
    0x6F12,0x8356,
    0x6F12,0x6772,
    0x6F12,0x2391,
    0x6F12,0xD702,
    0x6F12,0x0357,
    0x6F12,0xA772,
    0x6F12,0x2392,
    0x6F12,0xE702,
    0x6F12,0xB776,
    0x6F12,0x0024,
    0x6F12,0x83C6,
    0x6F12,0xB6F1,
    0x6F12,0x0547,
    0x6F12,0xA38F,
    0x6F12,0xE700,
    0x6F12,0x99C6,
    0x6F12,0x83D6,
    0x6F12,0x4701,
    0x6F12,0x238F,
    0x6F12,0xE700,
    0x6F12,0x2380,
    0x6F12,0xD702,
    0x6F12,0x83C6,
    0x6F12,0x0702,
    0x6F12,0x03C7,
    0x6F12,0xE701,
    0x6F12,0xB9CE,
    0x6F12,0x0DC3,
    0x6F12,0x03D7,
    0x6F12,0x6701,
    0x6F12,0x0DCF,
    0x6F12,0xB7F6,
    0x6F12,0x0040,
    0x6F12,0x2393,
    0x6F12,0xE672,
    0x6F12,0x03D7,
    0x6F12,0x8701,
    0x6F12,0x0DCF,
    0x6F12,0xB7F6,
    0x6F12,0x0040,
    0x6F12,0x2395,
    0x6F12,0xE672,
    0x6F12,0x238F,
    0x6F12,0x0700,
    0x6F12,0x03C7,
    0x6F12,0x0702,
    0x6F12,0x7D17,
    0x6F12,0x1377,
    0x6F12,0xF70F,
    0x6F12,0x2380,
    0x6F12,0xE702,
    0x6F12,0x01E7,
    0x6F12,0x0547,
    0x6F12,0x238F,
    0x6F12,0xE700,
    0x6F12,0x1743,
    0x6F12,0x01FC,
    0x6F12,0x6700,
    0x6F12,0x631A,
    0x6F12,0x83D6,
    0x6F12,0x2702,
    0x6F12,0x37F7,
    0x6F12,0x0040,
    0x6F12,0x2313,
    0x6F12,0xD772,
    0x6F12,0xD1B7,
    0x6F12,0x83D6,
    0x6F12,0x4702,
    0x6F12,0x37F7,
    0x6F12,0x0040,
    0x6F12,0x2315,
    0x6F12,0xD772,
    0x6F12,0xD1B7,
    0x6F12,0x71DF,
    0x6F12,0x03D7,
    0x6F12,0xA701,
    0x6F12,0x19CF,
    0x6F12,0xB7F6,
    0x6F12,0x0040,
    0x6F12,0x2393,
    0x6F12,0xE672,
    0x6F12,0x03D7,
    0x6F12,0xC701,
    0x6F12,0x19CF,
    0x6F12,0xB7F6,
    0x6F12,0x0040,
    0x6F12,0x2395,
    0x6F12,0xE672,
    0x6F12,0x238F,
    0x6F12,0x0700,
    0x6F12,0x6DBF,
    0x6F12,0x83D6,
    0x6F12,0x2702,
    0x6F12,0x37F7,
    0x6F12,0x0040,
    0x6F12,0x2313,
    0x6F12,0xD772,
    0x6F12,0xC5B7,
    0x6F12,0x83D6,
    0x6F12,0x4702,
    0x6F12,0x37F7,
    0x6F12,0x0040,
    0x6F12,0x2315,
    0x6F12,0xD772,
    0x6F12,0xC5B7,
    0x6F12,0x8280,
    0x6F12,0x1743,
    0x6F12,0x01FC,
    0x6F12,0xE702,
    0x6F12,0xC311,
    0x6F12,0xB787,
    0x6F12,0x0024,
    0x6F12,0xA166,
    0x6F12,0xB775,
    0x6F12,0x0024,
    0x6F12,0x9386,
    0x6F12,0x76F7,
    0x6F12,0x3777,
    0x6F12,0x0024,
    0x6F12,0x9387,
    0x6F12,0xC701,
    0x6F12,0x2946,
    0x6F12,0x9385,
    0x6F12,0xA57F,
    0x6F12,0x3545,
    0x6F12,0x2320,
    0x6F12,0xF73C,
    0x6F12,0x9710,
    0x6F12,0x00FC,
    0x6F12,0xE780,
    0x6F12,0xA01F,
    0x6F12,0xB777,
    0x6F12,0x0024,
    0x6F12,0xB785,
    0x6F12,0x0024,
    0x6F12,0x3755,
    0x6F12,0x0020,
    0x6F12,0x3737,
    0x6F12,0x0024,
    0x6F12,0x9387,
    0x6F12,0x4774,
    0x6F12,0x0146,
    0x6F12,0x9385,
    0x6F12,0xA58B,
    0x6F12,0x1305,
    0x6F12,0x0537,
    0x6F12,0x232C,
    0x6F12,0xF75E,
    0x6F12,0x9710,
    0x6F12,0x00FC,
    0x6F12,0xE780,
    0x6F12,0x4058,
    0x6F12,0xB787,
    0x6F12,0x0024,
    0x6F12,0x23A6,
    0x6F12,0xA700,
    0x6F12,0xB785,
    0x6F12,0x0024,
    0x6F12,0x3765,
    0x6F12,0x0020,
    0x6F12,0x0146,
    0x6F12,0x9385,
    0x6F12,0xE59F,
    0x6F12,0x1305,
    0x6F12,0x45B2,
    0x6F12,0x9710,
    0x6F12,0x00FC,
    0x6F12,0xE780,
    0x6F12,0x2056,
    0x6F12,0xB787,
    0x6F12,0x0024,
    0x6F12,0x23A2,
    0x6F12,0xA700,
    0x6F12,0xB785,
    0x6F12,0x0024,
    0x6F12,0x3755,
    0x6F12,0x0020,
    0x6F12,0x0146,
    0x6F12,0x9385,
    0x6F12,0x2597,
    0x6F12,0x1305,
    0x6F12,0x0525,
    0x6F12,0x9710,
    0x6F12,0x00FC,
    0x6F12,0xE780,
    0x6F12,0x0054,
    0x6F12,0xB787,
    0x6F12,0x0024,
    0x6F12,0x23A4,
    0x6F12,0xA700,
    0x6F12,0xB775,
    0x6F12,0x0024,
    0x6F12,0x3775,
    0x6F12,0x0020,
    0x6F12,0x0146,
    0x6F12,0x9385,
    0x6F12,0x257B,
    0x6F12,0x1305,
    0x6F12,0x85D3,
    0x6F12,0x9710,
    0x6F12,0x00FC,
    0x6F12,0xE780,
    0x6F12,0xE051,
    0x6F12,0xB787,
    0x6F12,0x0024,
    0x6F12,0x23A8,
    0x6F12,0xA700,
    0x6F12,0xB785,
    0x6F12,0x0024,
    0x6F12,0x37B5,
    0x6F12,0x0020,
    0x6F12,0x0146,
    0x6F12,0x9385,
    0x6F12,0x85CE,
    0x6F12,0x1305,
    0x6F12,0xA5C6,
    0x6F12,0x9710,
    0x6F12,0x00FC,
    0x6F12,0xE780,
    0x6F12,0xC04F,
    0x6F12,0xB787,
    0x6F12,0x0024,
    0x6F12,0x23A0,
    0x6F12,0xA700,
    0x6F12,0xB787,
    0x6F12,0x0024,
    0x6F12,0x3737,
    0x6F12,0x0024,
    0x6F12,0x9387,
    0x6F12,0x67D3,
    0x6F12,0x2320,
    0x6F12,0xF768,
    0x6F12,0x1743,
    0x6F12,0x01FC,
    0x6F12,0x6700,
    0x6F12,0x4304,
    0x6F12,0x0000,
    0x6F12,0x0020,
    0x6F12,0x0020,
    0x6F12,0x0020,
    0x6F12,0x0020,
    0x6F12,0x0020,
    0x6F12,0x0020,
    0x6F12,0x0020,
    0x6F12,0x0020,
    0x6F12,0x0020,
    0x6F12,0x0020,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0001,
    0x6F12,0x0203,
    0x6F12,0x0001,
    0x6F12,0x0203,
    0x6F12,0x0405,
    0x6F12,0x0607,
    0x6F12,0x0405,
    0x6F12,0x0607,
    0x6F12,0x10D0,
    0x6F12,0x10D0,
    0x6F12,0x1CD0,
    0x6F12,0x1CD0,
    0x6F12,0x22D0,
    0x6F12,0x22D0,
    0x6F12,0x2ED0,
    0x6F12,0x2ED0,
    0x6F12,0x0000,
    0x6F12,0x0400,
    0x6F12,0x0800,
    0x6F12,0x0C00,
    0x6F12,0x0800,
    0x6F12,0x0C00,
    0x6F12,0x0000,
    0x6F12,0x0400,
    0x6F12,0x30D0,
    0x6F12,0x32D0,
    0x6F12,0x64D0,
    0x6F12,0x66D0,
    0x6F12,0x7CD0,
    0x6F12,0x7ED0,
    0x6F12,0xA8D0,
    0x6F12,0xAAD0,
    0x6F12,0x0001,
    0x6F12,0x0001,
    0x6F12,0x0001,
    0x6F12,0x0001,
    0x6F12,0x0405,
    0x6F12,0x0405,
    0x6F12,0x0405,
    0x6F12,0x0405,
    0x6F12,0x10D0,
    0x6F12,0x10D0,
    0x6F12,0x12D0,
    0x6F12,0x12D0,
    0x6F12,0x20D0,
    0x6F12,0x20D0,
    0x6F12,0x22D0,
    0x6F12,0x22D0,
    0x6F12,0x0000,
    0x6F12,0x0400,
    0x6F12,0x0000,
    0x6F12,0x0400,
    0x6F12,0x0000,
    0x6F12,0x0400,
    0x6F12,0x0000,
    0x6F12,0x0400,
    0x6F12,0x30D0,
    0x6F12,0x32D0,
    0x6F12,0x38D0,
    0x6F12,0x3AD0,
    0x6F12,0x70D0,
    0x6F12,0x72D0,
    0x6F12,0x78D0,
    0x6F12,0x7AD0,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x1743,
    0x6F12,0x01FC,
    0x6F12,0xE702,
    0x6F12,0xA3F3,
    0x6F12,0xB787,
    0x6F12,0x0024,
    0x6F12,0x1307,
    0x6F12,0xC00E,
    0x6F12,0x9387,
    0x6F12,0xC701,
    0x6F12,0xB785,
    0x6F12,0x0024,
    0x6F12,0x3755,
    0x6F12,0x0020,
    0x6F12,0xBA97,
    0x6F12,0x0146,
    0x6F12,0x3777,
    0x6F12,0x0024,
    0x6F12,0x9385,
    0x6F12,0x4507,
    0x6F12,0x1305,
    0x6F12,0xC504,
    0x6F12,0x2320,
    0x6F12,0xF73C,
    0x6F12,0x9710,
    0x6F12,0x00FC,
    0x6F12,0xE780,
    0x6F12,0x603C,
    0x6F12,0xB787,
    0x6F12,0x0024,
    0x6F12,0x23A0,
    0x6F12,0xA710,
    0x6F12,0x4928,
    0x6F12,0xE177,
    0x6F12,0x3747,
    0x6F12,0x0024,
    0x6F12,0x9387,
    0x6F12,0x5776,
    0x6F12,0x2317,
    0x6F12,0xF782,
    0x6F12,0x1743,
    0x6F12,0x01FC,
    0x6F12,0x6700,
    0x6F12,0xE3F0,
    0x6F12,0x1743,
    0x6F12,0x01FC,
    0x6F12,0xE702,
    0x6F12,0x83EC,
    0x6F12,0xB787,
    0x6F12,0x0024,
    0x6F12,0x83A4,
    0x6F12,0x0710,
    0x6F12,0xAA89,
    0x6F12,0x2E8A,
    0x6F12,0x0146,
    0x6F12,0xA685,
    0x6F12,0x1145,
    0x6F12,0x9780,
    0x6F12,0xFFFB,
    0x6F12,0xE780,
    0x6F12,0xA031,
    0x6F12,0xB787,
    0x6F12,0x0024,
    0x6F12,0x03C7,
    0x6F12,0x4710,
    0x6F12,0x3E84,
    0x6F12,0x0149,
    0x6F12,0x11CF,
    0x6F12,0x3767,
    0x6F12,0x0024,
    0x6F12,0x0357,
    0x6F12,0x2777,
    0x6F12,0xB777,
    0x6F12,0x0024,
    0x6F12,0x9387,
    0x6F12,0x07C7,
    0x6F12,0x0E07,
    0x6F12,0x03D9,
    0x6F12,0x871C,
    0x6F12,0x2394,
    0x6F12,0xE71C,
    0x6F12,0xD285,
    0x6F12,0x4E85,
    0x6F12,0x97D0,
    0x6F12,0xFFFB,
    0x6F12,0xE780,
    0x6F12,0xA0F8,
    0x6F12,0x8347,
    0x6F12,0x4410,
    0x6F12,0x89C7,
    0x6F12,0xB777,
    0x6F12,0x0024,
    0x6F12,0x239C,
    0x6F12,0x27E3,
    0x6F12,0x0546,
    0x6F12,0xA685,
    0x6F12,0x1145,
    0x6F12,0x9780,
    0x6F12,0xFFFB,
    0x6F12,0xE780,
    0x6F12,0xA02C,
    0x6F12,0x1743,
    0x6F12,0x01FC,
    0x6F12,0x6700,
    0x6F12,0xA3E8,
    0x6F12,0xE177,
    0x6F12,0x3747,
    0x6F12,0x0024,
    0x6F12,0x9387,
    0x6F12,0x5776,
    0x6F12,0x2318,
    0x6F12,0xF782,
    0x6F12,0x8280,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x602A,0x35CC,
    0x6F12,0x1C80,
    0x6F12,0x0024,
    0x6028,0x2400,
    0x602A,0x1354,
    0x6F12,0x0100,
    0x6F12,0x7017,
    0x602A,0x13B2,
    0x6F12,0x0000,
    0x602A,0x1236,
    0x6F12,0x0000,
    0x602A,0x1A0A,
    0x6F12,0x4C0A,
    0x602A,0x2210,
    0x6F12,0x3401,
    0x602A,0x2176,
    0x6F12,0x6400,
    0x602A,0x222E,
    0x6F12,0x0001,
    0x602A,0x06B6,
    0x6F12,0x0A00,
    0x602A,0x06BC,
    0x6F12,0x1001,
    0x602A,0x2140,
    0x6F12,0x0101,
    0x602A,0x218E,
    0x6F12,0x0000,
    0x602A,0x1A0E,
    0x6F12,0x9600,
    0x6028,0x4000,
    0xF44E,0x0011,
    0xF44C,0x0B0B,
    0xF44A,0x0006,
    0x0118,0x0002,
    0x011A,0x0001,	/*	sensor_init_setting_array  */
};

static kal_uint16 preview_setting_array[] = {
    0x602A,0x1A28,
    0x6028,0x2400,
    0x6F12,0x4C00,
    0x602A,0x065A,
    0x6F12,0x0000,
    0x602A,0x139E,
    0x6F12,0x0100,
    0x602A,0x139C,
    0x6F12,0x0000,
    0x602A,0x13A0,
    0x6F12,0x0A00,
    0x6F12,0x0120,
    0x602A,0x2072,
    0x6F12,0x0000,
    0x602A,0x1A64,
    0x6F12,0x0301,
    0x6F12,0xFF00,
    0x602A,0x19E6,
    0x6F12,0x0200,
    0x602A,0x1A30,
    0x6F12,0x3401,
    0x602A,0x19FC,
    0x6F12,0x0B00,
    0x602A,0x19F4,
    0x6F12,0x0606,
    0x602A,0x19F8,
    0x6F12,0x1010,
    0x602A,0x1B26,
    0x6F12,0x6F80,
    0x6F12,0xA060,
    0x602A,0x1A3C,
    0x6F12,0x6207,
    0x602A,0x1A48,
    0x6F12,0x6207,
    0x602A,0x1444,
    0x6F12,0x2000,
    0x6F12,0x2000,
    0x602A,0x144C,
    0x6F12,0x3F00,
    0x6F12,0x3F00,
    0x602A,0x7F6C,
    0x6F12,0x0100,
    0x6F12,0x2F00,
    0x6F12,0xFA00,
    0x6F12,0x2400,
    0x6F12,0xE500,
    0x602A,0x0650,
    0x6F12,0x0600,
    0x602A,0x0654,
    0x6F12,0x0000,
    0x602A,0x1A46,
    0x6F12,0x8A00,
    0x602A,0x1A52,
    0x6F12,0xBF00,
    0x602A,0x0674,
    0x6F12,0x0500,
    0x6F12,0x0500,
    0x6F12,0x0500,
    0x6F12,0x0500,
    0x602A,0x0668,
    0x6F12,0x0800,
    0x6F12,0x0800,
    0x6F12,0x0800,
    0x6F12,0x0800,
    0x602A,0x0684,
    0x6F12,0x4001,
    0x602A,0x0688,
    0x6F12,0x4001,
    0x602A,0x147C,
    0x6F12,0x1000,
    0x602A,0x1480,
    0x6F12,0x1000,
    0x602A,0x19F6,
    0x6F12,0x0904,
    0x602A,0x0812,
    0x6F12,0x0000,
    0x602A,0x1A02,
    0x6F12,0x1800,
    0x602A,0x2148,
    0x6F12,0x0100,
    0x602A,0x2042,
    0x6F12,0x1A00,
    0x602A,0x0874,
    0x6F12,0x0100,
    0x602A,0x09C0,
    0x6F12,0x2008,
    0x602A,0x09C4,
    0x6F12,0x2000,
    0x602A,0x19FE,
    0x6F12,0x0E1C,
    0x602A,0x4D92,
    0x6F12,0x0100,
    0x602A,0x84C8,
    0x6F12,0x0100,
    0x602A,0x4D94,
    0x6F12,0x0005,
    0x6F12,0x000A,
    0x6F12,0x0010,
    0x6F12,0x0810,
    0x6F12,0x000A,
    0x6F12,0x0040,
    0x6F12,0x0810,
    0x6F12,0x0810,
    0x6F12,0x8002,
    0x6F12,0xFD03,
    0x6F12,0x0010,
    0x6F12,0x1510,
    0x602A,0x3570,
    0x6F12,0x0000,
    0x602A,0x3574,
    0x6F12,0x1201,
    0x602A,0x21E4,
    0x6F12,0x0400,
    0x602A,0x21EC,
    0x6F12,0x1F04,
    0x602A,0x2080,
    0x6F12,0x0101,
    0x6F12,0xFF00,
    0x6F12,0x7F01,
    0x6F12,0x0001,
    0x6F12,0x8001,
    0x6F12,0xD244,
    0x6F12,0xD244,
    0x6F12,0x14F4,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x602A,0x20BA,
    0x6F12,0x121C,
    0x6F12,0x111C,
    0x6F12,0x54F4,
    0x602A,0x120E,
    0x6F12,0x1000,
    0x602A,0x212E,
    0x6F12,0x0200,
    0x602A,0x13AE,
    0x6F12,0x0101,
    0x602A,0x0718,
    0x6F12,0x0001,
    0x602A,0x0710,
    0x6F12,0x0002,
    0x6F12,0x0804,
    0x6F12,0x0100,
    0x602A,0x1B5C,
    0x6F12,0x0000,
    0x602A,0x0786,
    0x6F12,0x7701,
    0x602A,0x2022,
    0x6F12,0x0500,
    0x6F12,0x0500,
    0x602A,0x1360,
    0x6F12,0x0100,
    0x602A,0x1376,
    0x6F12,0x0100,
    0x6F12,0x6038,
    0x6F12,0x7038,
    0x6F12,0x8038,
    0x602A,0x1386,
    0x6F12,0x0B00,
    0x602A,0x06FA,
    0x6F12,0x0000,
    0x602A,0x4A94,
    0x6F12,0x0900,
    0x6F12,0x0000,
    0x6F12,0x0300,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0300,
    0x6F12,0x0000,
    0x6F12,0x0900,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x602A,0x0A76,
    0x6F12,0x1000,
    0x602A,0x0AEE,
    0x6F12,0x1000,
    0x602A,0x0B66,
    0x6F12,0x1000,
    0x602A,0x0BDE,
    0x6F12,0x1000,
    0x602A,0x0BE8,
    0x6F12,0x3000,
    0x6F12,0x3000,
    0x602A,0x0C56,
    0x6F12,0x1000,
    0x602A,0x0C60,
    0x6F12,0x3000,
    0x6F12,0x3000,
    0x602A,0x0CB6,
    0x6F12,0x0100,
    0x602A,0x0CF2,
    0x6F12,0x0001,
    0x602A,0x0CF0,
    0x6F12,0x0101,
    0x602A,0x11B8,
    0x6F12,0x0100,
    0x602A,0x11F6,
    0x6F12,0x0020,
    0x602A,0x4A74,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0xD8FF,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0xD8FF,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x602A,0x218E,
    0x6F12,0x0000,
    0x602A,0x2268,
    0x6F12,0xF279,
    0x602A,0x5006,
    0x6F12,0x0000,
    0x602A,0x500E,
    0x6F12,0x0100,
    0x602A,0x4E70,
    0x6F12,0x2062,
    0x6F12,0x5501,
    0x602A,0x06DC,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6028,0x4000,
    0xF46A,0xAE80,
    0x0344,0x0000,
    0x0346,0x0000,
    0x0348,0x1FFF,
    0x034A,0x181F,
    0x034C,0x0FF0,
    0x034E,0x0C00,
    0x0350,0x0008,
    0x0352,0x0008,
    0x0900,0x0122,
    0x0380,0x0002,
    0x0382,0x0002,
    0x0384,0x0002,
    0x0386,0x0002,
    0x0110,0x1002,
    0x0114,0x0301,
    0x0116,0x3000,
    0x0136,0x1800,
    0x013E,0x0000,
    0x0300,0x0006,
    0x0302,0x0001,
    0x0304,0x0004,
    0x0306,0x008C,
    0x0308,0x0008,
    0x030A,0x0001,
    0x030C,0x0000,
    0x030E,0x0004,
    0x0310,0x0067,
    0x0312,0x0000,
    0x080E,0x0000,
    0x0340,0x0C54,
    0x0342,0x1716,
    0x0702,0x0000,
    0x0202,0x0100,
    0x0200,0x0100,
    #ifdef PDAF_TEST
    0x0D00,0x0101,
    0x0D02,0x0101,
    #else
    0x0D00,0x0100,
    0x0D02,0x0001,
    #endif
    0x0D04,0x0102,
    0x6226,0x0000,
};

static kal_uint16 capture_setting_array[] = {
    0x602A,0x1A28,
    0x6028,0x2400,
    0x6F12,0x4C00,
    0x602A,0x065A,
    0x6F12,0x0000,
    0x602A,0x139E,
    0x6F12,0x0100,
    0x602A,0x139C,
    0x6F12,0x0000,
    0x602A,0x13A0,
    0x6F12,0x0A00,
    0x6F12,0x0120,
    0x602A,0x2072,
    0x6F12,0x0000,
    0x602A,0x1A64,
    0x6F12,0x0301,
    0x6F12,0xFF00,
    0x602A,0x19E6,
    0x6F12,0x0200,
    0x602A,0x1A30,
    0x6F12,0x3401,
    0x602A,0x19FC,
    0x6F12,0x0B00,
    0x602A,0x19F4,
    0x6F12,0x0606,
    0x602A,0x19F8,
    0x6F12,0x1010,
    0x602A,0x1B26,
    0x6F12,0x6F80,
    0x6F12,0xA060,
    0x602A,0x1A3C,
    0x6F12,0x6207,
    0x602A,0x1A48,
    0x6F12,0x6207,
    0x602A,0x1444,
    0x6F12,0x2000,
    0x6F12,0x2000,
    0x602A,0x144C,
    0x6F12,0x3F00,
    0x6F12,0x3F00,
    0x602A,0x7F6C,
    0x6F12,0x0100,
    0x6F12,0x2F00,
    0x6F12,0xFA00,
    0x6F12,0x2400,
    0x6F12,0xE500,
    0x602A,0x0650,
    0x6F12,0x0600,
    0x602A,0x0654,
    0x6F12,0x0000,
    0x602A,0x1A46,
    0x6F12,0x8A00,
    0x602A,0x1A52,
    0x6F12,0xBF00,
    0x602A,0x0674,
    0x6F12,0x0500,
    0x6F12,0x0500,
    0x6F12,0x0500,
    0x6F12,0x0500,
    0x602A,0x0668,
    0x6F12,0x0800,
    0x6F12,0x0800,
    0x6F12,0x0800,
    0x6F12,0x0800,
    0x602A,0x0684,
    0x6F12,0x4001,
    0x602A,0x0688,
    0x6F12,0x4001,
    0x602A,0x147C,
    0x6F12,0x1000,
    0x602A,0x1480,
    0x6F12,0x1000,
    0x602A,0x19F6,
    0x6F12,0x0904,
    0x602A,0x0812,
    0x6F12,0x0000,
    0x602A,0x1A02,
    0x6F12,0x1800,
    0x602A,0x2148,
    0x6F12,0x0100,
    0x602A,0x2042,
    0x6F12,0x1A00,
    0x602A,0x0874,
    0x6F12,0x0100,
    0x602A,0x09C0,
    0x6F12,0x2008,
    0x602A,0x09C4,
    0x6F12,0x2000,
    0x602A,0x19FE,
    0x6F12,0x0E1C,
    0x602A,0x4D92,
    0x6F12,0x0100,
    0x602A,0x84C8,
    0x6F12,0x0100,
    0x602A,0x4D94,
    0x6F12,0x0005,
    0x6F12,0x000A,
    0x6F12,0x0010,
    0x6F12,0x0810,
    0x6F12,0x000A,
    0x6F12,0x0040,
    0x6F12,0x0810,
    0x6F12,0x0810,
    0x6F12,0x8002,
    0x6F12,0xFD03,
    0x6F12,0x0010,
    0x6F12,0x1510,
    0x602A,0x3570,
    0x6F12,0x0000,
    0x602A,0x3574,
    0x6F12,0x1201,
    0x602A,0x21E4,
    0x6F12,0x0400,
    0x602A,0x21EC,
    0x6F12,0x1F04,
    0x602A,0x2080,
    0x6F12,0x0101,
    0x6F12,0xFF00,
    0x6F12,0x7F01,
    0x6F12,0x0001,
    0x6F12,0x8001,
    0x6F12,0xD244,
    0x6F12,0xD244,
    0x6F12,0x14F4,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x602A,0x20BA,
    0x6F12,0x121C,
    0x6F12,0x111C,
    0x6F12,0x54F4,
    0x602A,0x120E,
    0x6F12,0x1000,
    0x602A,0x212E,
    0x6F12,0x0200,
    0x602A,0x13AE,
    0x6F12,0x0101,
    0x602A,0x0718,
    0x6F12,0x0001,
    0x602A,0x0710,
    0x6F12,0x0002,
    0x6F12,0x0804,
    0x6F12,0x0100,
    0x602A,0x1B5C,
    0x6F12,0x0000,
    0x602A,0x0786,
    0x6F12,0x7701,
    0x602A,0x2022,
    0x6F12,0x0500,
    0x6F12,0x0500,
    0x602A,0x1360,
    0x6F12,0x0100,
    0x602A,0x1376,
    0x6F12,0x0100,
    0x6F12,0x6038,
    0x6F12,0x7038,
    0x6F12,0x8038,
    0x602A,0x1386,
    0x6F12,0x0B00,
    0x602A,0x06FA,
    0x6F12,0x0000,
    0x602A,0x4A94,
    0x6F12,0x0900,
    0x6F12,0x0000,
    0x6F12,0x0300,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0300,
    0x6F12,0x0000,
    0x6F12,0x0900,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x602A,0x0A76,
    0x6F12,0x1000,
    0x602A,0x0AEE,
    0x6F12,0x1000,
    0x602A,0x0B66,
    0x6F12,0x1000,
    0x602A,0x0BDE,
    0x6F12,0x1000,
    0x602A,0x0BE8,
    0x6F12,0x3000,
    0x6F12,0x3000,
    0x602A,0x0C56,
    0x6F12,0x1000,
    0x602A,0x0C60,
    0x6F12,0x3000,
    0x6F12,0x3000,
    0x602A,0x0CB6,
    0x6F12,0x0100,
    0x602A,0x0CF2,
    0x6F12,0x0001,
    0x602A,0x0CF0,
    0x6F12,0x0101,
    0x602A,0x11B8,
    0x6F12,0x0100,
    0x602A,0x11F6,
    0x6F12,0x0020,
    0x602A,0x4A74,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0xD8FF,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0xD8FF,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x602A,0x218E,
    0x6F12,0x0000,
    0x602A,0x2268,
    0x6F12,0xF279,
    0x602A,0x5006,
    0x6F12,0x0000,
    0x602A,0x500E,
    0x6F12,0x0100,
    0x602A,0x4E70,
    0x6F12,0x2062,
    0x6F12,0x5501,
    0x602A,0x06DC,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6028,0x4000,
    0xF46A,0xAE80,
    0x0344,0x0000,
    0x0346,0x0000,
    0x0348,0x1FFF,
    0x034A,0x181F,
    0x034C,0x0FF0,
    0x034E,0x0C00,
    0x0350,0x0008,
    0x0352,0x0008,
    0x0900,0x0122,
    0x0380,0x0002,
    0x0382,0x0002,
    0x0384,0x0002,
    0x0386,0x0002,
    0x0110,0x1002,
    0x0114,0x0301,
    0x0116,0x3000,
    0x0136,0x1800,
    0x013E,0x0000,
    0x0300,0x0006,
    0x0302,0x0001,
    0x0304,0x0004,
    0x0306,0x008C,
    0x0308,0x0008,
    0x030A,0x0001,
    0x030C,0x0000,
    0x030E,0x0004,
    0x0310,0x0067,
    0x0312,0x0000,
    0x080E,0x0000,
    0x0340,0x0C54,
    0x0342,0x1716,
    0x0702,0x0000,
    0x0202,0x0100,
    0x0200,0x0100,
    #ifdef PDAF_TEST
    0x0D00,0x0101,
    0x0D02,0x0101,
    #else
    0x0D00,0x0100,
    0x0D02,0x0001,
    #endif
    0x0D04,0x0102,
    0x6226,0x0000,
};

static kal_uint16 normal_video_setting_array[] = {
    0x602A,0x1A28,
    0x6028,0x2400,
    0x6F12,0x4C00,
    0x602A,0x065A,
    0x6F12,0x0000,
    0x602A,0x139E,
    0x6F12,0x0100,
    0x602A,0x139C,
    0x6F12,0x0000,
    0x602A,0x13A0,
    0x6F12,0x0A00,
    0x6F12,0x0120,
    0x602A,0x2072,
    0x6F12,0x0000,
    0x602A,0x1A64,
    0x6F12,0x0301,
    0x6F12,0xFF00,
    0x602A,0x19E6,
    0x6F12,0x0200,
    0x602A,0x1A30,
    0x6F12,0x3401,
    0x602A,0x19FC,
    0x6F12,0x0B00,
    0x602A,0x19F4,
    0x6F12,0x0606,
    0x602A,0x19F8,
    0x6F12,0x1010,
    0x602A,0x1B26,
    0x6F12,0x6F80,
    0x6F12,0xA060,
    0x602A,0x1A3C,
    0x6F12,0x6207,
    0x602A,0x1A48,
    0x6F12,0x6207,
    0x602A,0x1444,
    0x6F12,0x2000,
    0x6F12,0x2000,
    0x602A,0x144C,
    0x6F12,0x3F00,
    0x6F12,0x3F00,
    0x602A,0x7F6C,
    0x6F12,0x0100,
    0x6F12,0x2F00,
    0x6F12,0xFA00,
    0x6F12,0x2400,
    0x6F12,0xE500,
    0x602A,0x0650,
    0x6F12,0x0600,
    0x602A,0x0654,
    0x6F12,0x0000,
    0x602A,0x1A46,
    0x6F12,0x8A00,
    0x602A,0x1A52,
    0x6F12,0xBF00,
    0x602A,0x0674,
    0x6F12,0x0500,
    0x6F12,0x0500,
    0x6F12,0x0500,
    0x6F12,0x0500,
    0x602A,0x0668,
    0x6F12,0x0800,
    0x6F12,0x0800,
    0x6F12,0x0800,
    0x6F12,0x0800,
    0x602A,0x0684,
    0x6F12,0x4001,
    0x602A,0x0688,
    0x6F12,0x4001,
    0x602A,0x147C,
    0x6F12,0x1000,
    0x602A,0x1480,
    0x6F12,0x1000,
    0x602A,0x19F6,
    0x6F12,0x0904,
    0x602A,0x0812,
    0x6F12,0x0000,
    0x602A,0x1A02,
    0x6F12,0x1800,
    0x602A,0x2148,
    0x6F12,0x0100,
    0x602A,0x2042,
    0x6F12,0x1A00,
    0x602A,0x0874,
    0x6F12,0x0100,
    0x602A,0x09C0,
    0x6F12,0x2008,
    0x602A,0x09C4,
    0x6F12,0x2000,
    0x602A,0x19FE,
    0x6F12,0x0E1C,
    0x602A,0x4D92,
    0x6F12,0x0100,
    0x602A,0x84C8,
    0x6F12,0x0100,
    0x602A,0x4D94,
    0x6F12,0x0005,
    0x6F12,0x000A,
    0x6F12,0x0010,
    0x6F12,0x0810,
    0x6F12,0x000A,
    0x6F12,0x0040,
    0x6F12,0x0810,
    0x6F12,0x0810,
    0x6F12,0x8002,
    0x6F12,0xFD03,
    0x6F12,0x0010,
    0x6F12,0x1510,
    0x602A,0x3570,
    0x6F12,0x0000,
    0x602A,0x3574,
    0x6F12,0x1201,
    0x602A,0x21E4,
    0x6F12,0x0400,
    0x602A,0x21EC,
    0x6F12,0x1F04,
    0x602A,0x2080,
    0x6F12,0x0101,
    0x6F12,0xFF00,
    0x6F12,0x7F01,
    0x6F12,0x0001,
    0x6F12,0x8001,
    0x6F12,0xD244,
    0x6F12,0xD244,
    0x6F12,0x14F4,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x602A,0x20BA,
    0x6F12,0x121C,
    0x6F12,0x111C,
    0x6F12,0x54F4,
    0x602A,0x120E,
    0x6F12,0x1000,
    0x602A,0x212E,
    0x6F12,0x0200,
    0x602A,0x13AE,
    0x6F12,0x0101,
    0x602A,0x0718,
    0x6F12,0x0001,
    0x602A,0x0710,
    0x6F12,0x0002,
    0x6F12,0x0804,
    0x6F12,0x0100,
    0x602A,0x1B5C,
    0x6F12,0x0000,
    0x602A,0x0786,
    0x6F12,0x7701,
    0x602A,0x2022,
    0x6F12,0x0500,
    0x6F12,0x0500,
    0x602A,0x1360,
    0x6F12,0x0100,
    0x602A,0x1376,
    0x6F12,0x0100,
    0x6F12,0x6038,
    0x6F12,0x7038,
    0x6F12,0x8038,
    0x602A,0x1386,
    0x6F12,0x0B00,
    0x602A,0x06FA,
    0x6F12,0x0000,
    0x602A,0x4A94,
    0x6F12,0x0900,
    0x6F12,0x0000,
    0x6F12,0x0300,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0300,
    0x6F12,0x0000,
    0x6F12,0x0900,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x602A,0x0A76,
    0x6F12,0x1000,
    0x602A,0x0AEE,
    0x6F12,0x1000,
    0x602A,0x0B66,
    0x6F12,0x1000,
    0x602A,0x0BDE,
    0x6F12,0x1000,
    0x602A,0x0BE8,
    0x6F12,0x3000,
    0x6F12,0x3000,
    0x602A,0x0C56,
    0x6F12,0x1000,
    0x602A,0x0C60,
    0x6F12,0x3000,
    0x6F12,0x3000,
    0x602A,0x0CB6,
    0x6F12,0x0100,
    0x602A,0x0CF2,
    0x6F12,0x0001,
    0x602A,0x0CF0,
    0x6F12,0x0101,
    0x602A,0x11B8,
    0x6F12,0x0100,
    0x602A,0x11F6,
    0x6F12,0x0020,
    0x602A,0x4A74,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0xD8FF,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0xD8FF,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x602A,0x218E,
    0x6F12,0x0000,
    0x602A,0x2268,
    0x6F12,0xF279,
    0x602A,0x5006,
    0x6F12,0x0000,
    0x602A,0x500E,
    0x6F12,0x0100,
    0x602A,0x4E70,
    0x6F12,0x2062,
    0x6F12,0x5501,
    0x602A,0x06DC,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6028,0x4000,
    0xF46A,0xAE80,
    0x0344,0x0000,
    0x0346,0x0000,
    0x0348,0x1FFF,
    0x034A,0x181F,
    0x034C,0x0FF0,
    0x034E,0x0C00,
    0x0350,0x0008,
    0x0352,0x0008,
    0x0900,0x0122,
    0x0380,0x0002,
    0x0382,0x0002,
    0x0384,0x0002,
    0x0386,0x0002,
    0x0110,0x1002,
    0x0114,0x0301,
    0x0116,0x3000,
    0x0136,0x1800,
    0x013E,0x0000,
    0x0300,0x0006,
    0x0302,0x0001,
    0x0304,0x0004,
    0x0306,0x008C,
    0x0308,0x0008,
    0x030A,0x0001,
    0x030C,0x0000,
    0x030E,0x0004,
    0x0310,0x0067,
    0x0312,0x0000,
    0x080E,0x0000,
    0x0340,0x0C54,
    0x0342,0x1716,
    0x0702,0x0000,
    0x0202,0x0100,
    0x0200,0x0100,
    #ifdef PDAF_TEST
    0x0D00,0x0101,
    0x0D02,0x0101,
    #else
    0x0D00,0x0100,
    0x0D02,0x0001,
    #endif
    0x0D04,0x0102,
    0x6226,0x0000,
};

static kal_uint16 hs_video_setting_array[] = {
    0x6028,0x2400,
    0x602A,0x1A28,
    0x6F12,0x4C00,
    0x602A,0x065A,
    0x6F12,0x0000,
    0x602A,0x139E,
    0x6F12,0x0300,
    0x602A,0x139C,
    0x6F12,0x0000,
    0x602A,0x13A0,
    0x6F12,0x0A00,
    0x6F12,0x0020,
    0x602A,0x2072,
    0x6F12,0x0000,
    0x602A,0x1A64,
    0x6F12,0x0301,
    0x6F12,0xFF00,
    0x602A,0x19E6,
    0x6F12,0x0201,
    0x602A,0x1A30,
    0x6F12,0x3401,
    0x602A,0x19FC,
    0x6F12,0x0B00,
    0x602A,0x19F4,
    0x6F12,0x0606,
    0x602A,0x19F8,
    0x6F12,0x1010,
    0x602A,0x1B26,
    0x6F12,0x6F80,
    0x6F12,0xA020,
    0x602A,0x1A3C,
    0x6F12,0x5207,
    0x602A,0x1A48,
    0x6F12,0x5207,
    0x602A,0x1444,
    0x6F12,0x2100,
    0x6F12,0x2100,
    0x602A,0x144C,
    0x6F12,0x4200,
    0x6F12,0x4200,
    0x602A,0x7F6C,
    0x6F12,0x0100,
    0x6F12,0x3100,
    0x6F12,0xF700,
    0x6F12,0x2600,
    0x6F12,0xE100,
    0x602A,0x0650,
    0x6F12,0x0600,
    0x602A,0x0654,
    0x6F12,0x0000,
    0x602A,0x1A46,
    0x6F12,0x8900,
    0x602A,0x1A52,
    0x6F12,0xBF00,
    0x602A,0x0674,
    0x6F12,0x0500,
    0x6F12,0x0500,
    0x6F12,0x0500,
    0x6F12,0x0500,
    0x602A,0x0668,
    0x6F12,0x0800,
    0x6F12,0x0800,
    0x6F12,0x0800,
    0x6F12,0x0800,
    0x602A,0x0684,
    0x6F12,0x4001,
    0x602A,0x0688,
    0x6F12,0x4001,
    0x602A,0x147C,
    0x6F12,0x1000,
    0x602A,0x1480,
    0x6F12,0x1000,
    0x602A,0x19F6,
    0x6F12,0x0904,
    0x602A,0x0812,
    0x6F12,0x0010,
    0x602A,0x2148,
    0x6F12,0x0100,
    0x602A,0x2042,
    0x6F12,0x1A00,
    0x602A,0x0874,
    0x6F12,0x1100,
    0x602A,0x09C0,
    0x6F12,0x1803,
    0x602A,0x09C4,
    0x6F12,0x1803,
    0x602A,0x19FE,
    0x6F12,0x0E1C,
    0x602A,0x4D92,
    0x6F12,0x0000,
    0x602A,0x8104,
    0x6F12,0x0000,
    0x602A,0x4D94,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x602A,0x3570,
    0x6F12,0x0000,
    0x602A,0x3574,
    0x6F12,0x3801,
    0x602A,0x21E4,
    0x6F12,0x0400,
    0x602A,0x21EC,
    0x6F12,0x6801,
    0x602A,0x2080,
    0x6F12,0x0100,
    0x6F12,0xFF01,
    0x602A,0x2086,
    0x6F12,0x0002,
    0x602A,0x208E,
    0x6F12,0x14F4,
    0x602A,0x208A,
    0x6F12,0xC244,
    0x6F12,0xD244,
    0x602A,0x120E,
    0x6F12,0x1000,
    0x602A,0x212E,
    0x6F12,0x0A00,
    0x602A,0x13AE,
    0x6F12,0x0102,
    0x602A,0x0718,
    0x6F12,0x0005,
    0x602A,0x0710,
    0x6F12,0x0004,
    0x6F12,0x0401,
    0x6F12,0x0100,
    0x602A,0x1B5C,
    0x6F12,0x0300,
    0x602A,0x0786,
    0x6F12,0x7701,
    0x602A,0x2022,
    0x6F12,0x0101,
    0x6F12,0x0101,
    0x602A,0x1360,
    0x6F12,0x0100,
    0x602A,0x1376,
    0x6F12,0x0200,
    0x6F12,0x6038,
    0x6F12,0x7038,
    0x6F12,0x8038,
    0x602A,0x1386,
    0x6F12,0x0B00,
    0x602A,0x06FA,
    0x6F12,0x1000,
    0x602A,0x4A94,
    0x6F12,0x0600,
    0x6F12,0x0600,
    0x6F12,0x0600,
    0x6F12,0x0600,
    0x6F12,0x0600,
    0x6F12,0x0600,
    0x6F12,0x0600,
    0x6F12,0x0600,
    0x6F12,0x0600,
    0x6F12,0x0600,
    0x6F12,0x0600,
    0x6F12,0x0600,
    0x6F12,0x0600,
    0x6F12,0x0600,
    0x6F12,0x0600,
    0x6F12,0x0600,
    0x602A,0x0A76,
    0x6F12,0x1000,
    0x602A,0x0AEE,
    0x6F12,0x1000,
    0x602A,0x0B66,
    0x6F12,0x1000,
    0x602A,0x0BDE,
    0x6F12,0x1000,
    0x602A,0x0C56,
    0x6F12,0x1000,
    0x602A,0x0CF2,
    0x6F12,0x0001,
    0x602A,0x0CF0,
    0x6F12,0x0101,
    0x602A,0x11B8,
    0x6F12,0x0000,
    0x602A,0x11F6,
    0x6F12,0x0010,
    0x602A,0x4A74,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6028,0x4000,
    0xF46A,0xAE80,
    0x0344,0x05F0,
    0x0346,0x0660,
    0x0348,0x1A0F,
    0x034A,0x11BF,
    0x034C,0x0500,
    0x034E,0x02D0,
    0x0350,0x0004,
    0x0352,0x0004,
    0x0900,0x0144,
    0x0380,0x0002,
    0x0382,0x0006,
    0x0384,0x0002,
    0x0386,0x0006,
    0x0110,0x1002,
    0x0114,0x0300,
    0x0116,0x3000,
    0x0136,0x1800,
    0x013E,0x0000,
    0x0300,0x0006,
    0x0302,0x0001,
    0x0304,0x0004,
    0x0306,0x0096,
    0x0308,0x0008,
    0x030A,0x0001,
    0x030C,0x0000,
    0x030E,0x0003,
    0x0310,0x0064,
    0x0312,0x0000,
    0x080E,0x0000,
    0x0340,0x094C,
    0x0342,0x0830,
    0x0702,0x0000,
    0x0202,0x0100,
    0x0200,0x0100,
    // #ifdef PDAF_TEST
    // Antaiui <AI_BSP_CAM> <xieht> <2022-04-01> 打开PD补偿 begin
    0x0D00,0x0101, // 0x0d01 0x01 PD补偿
    // Antaiui <AI_BSP_CAM> <xieht> <2022-04-01> 打开PD补偿 end
    0x0D02,0x0001,
    // #endif
    0x0D04,0x0002,
    0x6226,0x0000,
};

static kal_uint16 slim_video_setting_array[] = {
    0x6028,0x2400,
    0x602A,0x1A28,
    0x6F12,0x4C00,
    0x602A,0x065A,
    0x6F12,0x0000,
    0x602A,0x139E,
    0x6F12,0x0100,
    0x602A,0x139C,
    0x6F12,0x0000,
    0x602A,0x13A0,
    0x6F12,0x0A00,
    0x6F12,0x0120,
    0x602A,0x2072,
    0x6F12,0x0000,
    0x602A,0x1A64,
    0x6F12,0x0301,
    0x6F12,0xFF00,
    0x602A,0x19E6,
    0x6F12,0x0200,
    0x602A,0x1A30,
    0x6F12,0x3401,
    0x602A,0x19FC,
    0x6F12,0x0B00,
    0x602A,0x19F4,
    0x6F12,0x0606,
    0x602A,0x19F8,
    0x6F12,0x1010,
    0x602A,0x1B26,
    0x6F12,0x6F80,
    0x6F12,0xA060,
    0x602A,0x1A3C,
    0x6F12,0x6207,
    0x602A,0x1A48,
    0x6F12,0x6207,
    0x602A,0x1444,
    0x6F12,0x2000,
    0x6F12,0x2000,
    0x602A,0x144C,
    0x6F12,0x3F00,
    0x6F12,0x3F00,
    0x602A,0x7F6C,
    0x6F12,0x0100,
    0x6F12,0x2F00,
    0x6F12,0xFA00,
    0x6F12,0x2400,
    0x6F12,0xE500,
    0x602A,0x0650,
    0x6F12,0x0600,
    0x602A,0x0654,
    0x6F12,0x0000,
    0x602A,0x1A46,
    0x6F12,0xB000,
    0x602A,0x1A52,
    0x6F12,0xBF00,
    0x602A,0x0674,
    0x6F12,0x0500,
    0x6F12,0x0500,
    0x6F12,0x0500,
    0x6F12,0x0500,
    0x602A,0x0668,
    0x6F12,0x0800,
    0x6F12,0x0800,
    0x6F12,0x0800,
    0x6F12,0x0800,
    0x602A,0x0684,
    0x6F12,0x4001,
    0x602A,0x0688,
    0x6F12,0x4001,
    0x602A,0x147C,
    0x6F12,0x1000,
    0x602A,0x1480,
    0x6F12,0x1000,
    0x602A,0x19F6,
    0x6F12,0x0904,
    0x602A,0x0812,
    0x6F12,0x0010,
    0x602A,0x2148,
    0x6F12,0x0100,
    0x602A,0x2042,
    0x6F12,0x1A00,
    0x602A,0x0874,
    0x6F12,0x0100,
    0x602A,0x09C0,
    0x6F12,0x2008,
    0x602A,0x09C4,
    0x6F12,0x2000,
    0x602A,0x19FE,
    0x6F12,0x0E1C,
    0x602A,0x4D92,
    0x6F12,0x0100,
    0x602A,0x8104,
    0x6F12,0x0100,
    0x602A,0x4D94,
    0x6F12,0x0005,
    0x6F12,0x000A,
    0x6F12,0x0010,
    0x6F12,0x1510,
    0x6F12,0x000A,
    0x6F12,0x0040,
    0x6F12,0x1510,
    0x6F12,0x1510,
    0x602A,0x3570,
    0x6F12,0x0000,
    0x602A,0x3574,
    0x6F12,0x4700,
    0x602A,0x21E4,
    0x6F12,0x0400,
    0x602A,0x21EC,
    0x6F12,0xC702,
    0x602A,0x2080,
    0x6F12,0x0100,
    0x6F12,0xFF00,
    0x602A,0x2086,
    0x6F12,0x0001,
    0x602A,0x208E,
    0x6F12,0x14F4,
    0x602A,0x208A,
    0x6F12,0xD244,
    0x6F12,0xD244,
    0x602A,0x120E,
    0x6F12,0x1000,
    0x602A,0x212E,
    0x6F12,0x0200,
    0x602A,0x13AE,
    0x6F12,0x0101,
    0x602A,0x0718,
    0x6F12,0x0001,
    0x602A,0x0710,
    0x6F12,0x0002,
    0x6F12,0x0804,
    0x6F12,0x0100,
    0x602A,0x1B5C,
    0x6F12,0x0000,
    0x602A,0x0786,
    0x6F12,0x7701,
    0x602A,0x2022,
    0x6F12,0x0500,
    0x6F12,0x0500,
    0x602A,0x1360,
    0x6F12,0x0100,
    0x602A,0x1376,
    0x6F12,0x0100,
    0x6F12,0x6038,
    0x6F12,0x7038,
    0x6F12,0x8038,
    0x602A,0x1386,
    0x6F12,0x0B00,
    0x602A,0x06FA,
    0x6F12,0x0000,
    0x602A,0x4A94,
    0x6F12,0x0600,
    0x6F12,0x0600,
    0x6F12,0x0600,
    0x6F12,0x0600,
    0x6F12,0x0600,
    0x6F12,0x0600,
    0x6F12,0x0600,
    0x6F12,0x0600,
    0x6F12,0x0600,
    0x6F12,0x0600,
    0x6F12,0x0600,
    0x6F12,0x0600,
    0x6F12,0x0600,
    0x6F12,0x0600,
    0x6F12,0x0600,
    0x6F12,0x0600,
    0x602A,0x0A76,
    0x6F12,0x1000,
    0x602A,0x0AEE,
    0x6F12,0x1000,
    0x602A,0x0B66,
    0x6F12,0x1000,
    0x602A,0x0BDE,
    0x6F12,0x1000,
    0x602A,0x0C56,
    0x6F12,0x1000,
    0x602A,0x0CF2,
    0x6F12,0x0001,
    0x602A,0x0CF0,
    0x6F12,0x0101,
    0x602A,0x11B8,
    0x6F12,0x0100,
    0x602A,0x11F6,
    0x6F12,0x0020,
    0x602A,0x4A74,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0xD8FF,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0xD8FF,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6F12,0x0000,
    0x6028,0x4000,
    0xF46A,0xAE80,
    0x0344,0x0000,
    0x0346,0x0308,
    0x0348,0x1FFF,
    0x034A,0x1517,
    0x034C,0x0FF0,
    0x034E,0x08F8,
    0x0350,0x0008,
    0x0352,0x0008,
    0x0900,0x0122,
    0x0380,0x0002,
    0x0382,0x0002,
    0x0384,0x0002,
    0x0386,0x0002,
    0x0110,0x1002,
    0x0114,0x0300,
    0x0116,0x3000,
    0x0136,0x1800,
    0x013E,0x0000,
    0x0300,0x0006,
    0x0302,0x0001,
    0x0304,0x0004,
    0x0306,0x008C,
    0x0308,0x0008,
    0x030A,0x0001,
    0x030C,0x0000,
    0x030E,0x0004,
    0x0310,0x0064,
    0x0312,0x0000,
    0x080E,0x0000,
    0x0340,0x0C54,
    0x0342,0x1716,
    0x0702,0x0000,
    0x0202,0x0100,
    0x0200,0x0100,
    // #ifdef PDAF_TEST
    // Antaiui <AI_BSP_CAM> <xieht> <2022-04-01> 打开PD补偿 begin
    0x0D00,0x0101, // 0x0d01 0x01 PD补偿
    // Antaiui <AI_BSP_CAM> <xieht> <2022-04-01> 打开PD补偿 end
    0x0D02,0x0001,
    // #endif
    0x0D04,0x0102,
    0x6226,0x0000,
};

static kal_uint16 custom1_setting_array[] = {
	0x6028, 0x2400,
	0x602A, 0x1A28,
	0x6F12, 0x4C00,
	0x602A, 0x065A,
	0x6F12, 0x0000,
	0x602A, 0x139E,
	0x6F12, 0x0100,
	0x602A, 0x139C,
	0x6F12, 0x0000,
	0x602A, 0x13A0,
	0x6F12, 0x0A00,
	0x6F12, 0x0120,
	0x602A, 0x2072,
	0x6F12, 0x0000,
	0x602A, 0x1A64,
	0x6F12, 0x0301,
	0x6F12, 0xFF00,
	0x602A, 0x19E6,
	0x6F12, 0x0200,
	0x602A, 0x1A30,
	0x6F12, 0x3401,
	0x602A, 0x19FC,
	0x6F12, 0x0B00,
	0x602A, 0x19F4,
	0x6F12, 0x0606,
	0x602A, 0x19F8,
	0x6F12, 0x1010,
	0x602A, 0x1B26,
	0x6F12, 0x6F80,
	0x6F12, 0xA060,
	0x602A, 0x1A3C,
	0x6F12, 0x6207,
	0x602A, 0x1A48,
	0x6F12, 0x6207,
	0x602A, 0x1444,
	0x6F12, 0x2000,
	0x6F12, 0x2000,
	0x602A, 0x144C,
	0x6F12, 0x3F00,
	0x6F12, 0x3F00,
	0x602A, 0x7F6C,
	0x6F12, 0x0100,
	0x6F12, 0x2F00,
	0x6F12, 0xFA00,
	0x6F12, 0x2400,
	0x6F12, 0xE500,
	0x602A, 0x0650,
	0x6F12, 0x0600,
	0x602A, 0x0654,
	0x6F12, 0x0000,
	0x602A, 0x1A46,
	0x6F12, 0x8A00,
	0x602A, 0x1A52,
	0x6F12, 0xBF00,
	0x602A, 0x0674,
	0x6F12, 0x0500,
	0x6F12, 0x0500,
	0x6F12, 0x0500,
	0x6F12, 0x0500,
	0x602A, 0x0668,
	0x6F12, 0x0800,
	0x6F12, 0x0800,
	0x6F12, 0x0800,
	0x6F12, 0x0800,
	0x602A, 0x0684,
	0x6F12, 0x4001,
	0x602A, 0x0688,
	0x6F12, 0x4001,
	0x602A, 0x147C,
	0x6F12, 0x1000,
	0x602A, 0x1480,
	0x6F12, 0x1000,
	0x602A, 0x19F6,
	0x6F12, 0x0904,
	0x602A, 0x0812,
	0x6F12, 0x0000,
	0x602A, 0x1A02,
	0x6F12, 0x1800,
	0x602A, 0x2148,
	0x6F12, 0x0100,
	0x602A, 0x2042,
	0x6F12, 0x1A00,
	0x602A, 0x0874,
	0x6F12, 0x0100,
	0x602A, 0x09C0,
	0x6F12, 0x2008,
	0x602A, 0x09C4,
	0x6F12, 0x2000,
	0x602A, 0x19FE,
	0x6F12, 0x0E1C,
	0x602A, 0x4D92,
	0x6F12, 0x0100,
	0x602A, 0x84C8,
	0x6F12, 0x0100,
	0x602A, 0x4D94,
	0x6F12, 0x0005,
	0x6F12, 0x000A,
	0x6F12, 0x0010,
	0x6F12, 0x0810,
	0x6F12, 0x000A,
	0x6F12, 0x0040,
	0x6F12, 0x0810,
	0x6F12, 0x0810,
	0x6F12, 0x8002,
	0x6F12, 0xFD03,
	0x6F12, 0x0010,
	0x6F12, 0x1510,
	0x602A, 0x3570,
	0x6F12, 0x0000,
	0x602A, 0x3574,
	0x6F12, 0x1201,
	0x602A, 0x21E4,
	0x6F12, 0x0400,
	0x602A, 0x21EC,
	0x6F12, 0x1F04,
	0x602A, 0x2080,
	0x6F12, 0x0101,
	0x6F12, 0xFF00,
	0x6F12, 0x7F01,
	0x6F12, 0x0001,
	0x6F12, 0x8001,
	0x6F12, 0xD244,
	0x6F12, 0xD244,
	0x6F12, 0x14F4,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x20BA,
	0x6F12, 0x121C,
	0x6F12, 0x111C,
	0x6F12, 0x54F4,
	0x602A, 0x120E,
	0x6F12, 0x1000,
	0x602A, 0x212E,
	0x6F12, 0x0200,
	0x602A, 0x13AE,
	0x6F12, 0x0101,
	0x602A, 0x0718,
	0x6F12, 0x0001,
	0x602A, 0x0710,
	0x6F12, 0x0002,
	0x6F12, 0x0804,
	0x6F12, 0x0100,
	0x602A, 0x1B5C,
	0x6F12, 0x0000,
	0x602A, 0x0786,
	0x6F12, 0x7701,
	0x602A, 0x2022,
	0x6F12, 0x0500,
	0x6F12, 0x0500,
	0x602A, 0x1360,
	0x6F12, 0x0100,
	0x602A, 0x1376,
	0x6F12, 0x0100,
	0x6F12, 0x6038,
	0x6F12, 0x7038,
	0x6F12, 0x8038,
	0x602A, 0x1386,
	0x6F12, 0x0B00,
	0x602A, 0x06FA,
	0x6F12, 0x1000,
	0x602A, 0x4A94,
	0x6F12, 0x0900,
	0x6F12, 0x0000,
	0x6F12, 0x0300,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0300,
	0x6F12, 0x0000,
	0x6F12, 0x0900,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x0A76,
	0x6F12, 0x1000,
	0x602A, 0x0AEE,
	0x6F12, 0x1000,
	0x602A, 0x0B66,
	0x6F12, 0x1000,
	0x602A, 0x0BDE,
	0x6F12, 0x1000,
	0x602A, 0x0BE8,
	0x6F12, 0x3000,
	0x6F12, 0x3000,
	0x602A, 0x0C56,
	0x6F12, 0x1000,
	0x602A, 0x0C60,
	0x6F12, 0x3000,
	0x6F12, 0x3000,
	0x602A, 0x0CB6,
	0x6F12, 0x0100,
	0x602A, 0x0CF2,
	0x6F12, 0x0001,
	0x602A, 0x0CF0,
	0x6F12, 0x0101,
	0x602A, 0x11B8,
	0x6F12, 0x0100,
	0x602A, 0x11F6,
	0x6F12, 0x0020,
	0x602A, 0x4A74,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0xD8FF,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0xD8FF,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x218E,
	0x6F12, 0x0000,
	0x602A, 0x2268,
	0x6F12, 0xF279,
	0x602A, 0x5006,
	0x6F12, 0x0000,
	0x602A, 0x500E,
	0x6F12, 0x0100,
	0x602A, 0x4E70,
	0x6F12, 0x2062,
	0x6F12, 0x5501,
	0x602A, 0x06DC,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6028, 0x4000,
	0xF46A, 0xAE80,
	0x0344, 0x00F0,
	0x0346, 0x00C0,
	0x0348, 0x1F0F,
	0x034A, 0x175F,
	0x034C, 0x0F00,
	0x034E, 0x0B40,
	0x0350, 0x0008,
	0x0352, 0x0008,
	0x0900, 0x0122,
	0x0380, 0x0002,
	0x0382, 0x0002,
	0x0384, 0x0002,
	0x0386, 0x0002,
	0x0110, 0x1002,
	0x0114, 0x0301,
	0x0116, 0x3000,
	0x0136, 0x1800,
	0x013E, 0x0000,
	0x0300, 0x0006,
	0x0302, 0x0001,
	0x0304, 0x0004,
	0x0306, 0x008C,
	0x0308, 0x0008,
	0x030A, 0x0001,
	0x030C, 0x0000,
	0x030E, 0x0004,
	0x0310, 0x008A,
	0x0312, 0x0000,
	0x080E, 0x0000,
	0x0340, 0x0FD6,
	0x0342, 0x11E8,
	0x0702, 0x0000,
	0x0202, 0x0100,
	0x0200, 0x0100,
    #if 1
    // #ifdef PDAF_TEST
    // Antaiui <AI_BSP_CAM> <xieht> <2022-04-01> 打开PD补偿 begin
    0x0D00,0x0101, // 0x0d01 0x01 PD补偿
    // Antaiui <AI_BSP_CAM> <xieht> <2022-04-01> 打开PD补偿 end
    0x0D02,0x0001,
    // #endif
    0x0D04,0x0102,
    #else
	0x0D00, 0x0101,
	0x0D02, 0x0101,
	0x0D04, 0x0102,
    #endif
	0x6226, 0x0000,
};

static void sensor_init(void)
{
    /*Global setting */

    write_cmos_sensor_16_16(0x6028,0x4000);
    write_cmos_sensor_16_16(0x0000,0x0001);
    write_cmos_sensor_16_16(0x0000,0x38E1);
    write_cmos_sensor_16_16(0x001E,0x0005);
    write_cmos_sensor_16_16(0x6028,0x4000);
    write_cmos_sensor_16_16(0x6010,0x0001);
    mdelay(5);
    write_cmos_sensor_16_16(0x6226, 0x0001);
    mdelay(10);

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
    table_write_cmos_sensor(capture_setting_array,sizeof(capture_setting_array)/sizeof(kal_uint16));
    LOG_INF("end\n");
}

static void preview_setting(void)
{
    LOG_INF("start\n");
    // Antaiui <AI_BSP_CAM> <xieht> <2021-06-04> add check streaming off begin
    write_cmos_sensor_16_8(0x0100, 0x00);
    check_output_stream_off();
    // Antaiui <AI_BSP_CAM> <xieht> <2021-06-04> add check streaming off end
    table_write_cmos_sensor(preview_setting_array,sizeof(preview_setting_array) /sizeof(kal_uint16));
    LOG_INF("end\n");
}	/*	preview_setting  */

static void normal_video_setting(void)
{
    LOG_INF("start\n");
    // Antaiui <AI_BSP_CAM> <xieht> <2021-06-04> add check streaming off begin
    write_cmos_sensor_16_8(0x0100, 0x00);
    check_output_stream_off();
    // Antaiui <AI_BSP_CAM> <xieht> <2021-06-04> add check streaming off end
    table_write_cmos_sensor(normal_video_setting_array,sizeof(normal_video_setting_array) /sizeof(kal_uint16));
    LOG_INF("end\n");
}

static void hs_video_setting(void)
{
    LOG_INF("start\n");
    // Antaiui <AI_BSP_CAM> <xieht> <2021-06-04> add check streaming off begin
    write_cmos_sensor_16_8(0x0100, 0x00);
    check_output_stream_off();
    // Antaiui <AI_BSP_CAM> <xieht> <2021-06-04> add check streaming off end
    table_write_cmos_sensor(hs_video_setting_array,sizeof(hs_video_setting_array) /sizeof(kal_uint16));
    LOG_INF("end\n");
}

static void slim_video_setting(void)
{
    LOG_INF("start\n");
    // Antaiui <AI_BSP_CAM> <xieht> <2021-06-04> add check streaming off begin
    write_cmos_sensor_16_8(0x0100, 0x00);
    check_output_stream_off();
    // Antaiui <AI_BSP_CAM> <xieht> <2021-06-04> add check streaming off end
    table_write_cmos_sensor(slim_video_setting_array,sizeof(slim_video_setting_array) /sizeof(kal_uint16));
    LOG_INF("end\n");
}

static void custom1_setting(void)
{
    LOG_INF("start\n");
    // Antaiui <AI_BSP_CAM> <xieht> <2021-06-04> add check streaming off begin
    write_cmos_sensor_16_8(0x0100, 0x00);
    check_output_stream_off();
    // Antaiui <AI_BSP_CAM> <xieht> <2021-06-04> add check streaming off end
    table_write_cmos_sensor(custom1_setting_array,sizeof(custom1_setting_array) /sizeof(kal_uint16));
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
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
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
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
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
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
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

static kal_uint32 custom1(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
        MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM1;
    imgsensor.pclk = imgsensor_info.custom1.pclk;
    imgsensor.line_length = imgsensor_info.custom1.linelength;
    imgsensor.frame_length = imgsensor_info.custom1.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom1.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);

    custom1_setting();
    set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}   /*  custom1  */

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

	sensor_resolution->SensorCustom1Width = imgsensor_info.custom1.grabwindow_width;
	sensor_resolution->SensorCustom1Height = imgsensor_info.custom1.grabwindow_height;

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
    sensor_info->Custom1DelayFrame = imgsensor_info.custom1_delay_frame;

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
	case MSDK_SCENARIO_ID_CUSTOM1:
		sensor_info->SensorGrabStartX = imgsensor_info.custom1.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.custom1.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
						imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;
		break;
    default:
        sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
        sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;
        sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
            imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
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
	case MSDK_SCENARIO_ID_CUSTOM1:
		custom1(image_window, sensor_config_data);
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
	case MSDK_SCENARIO_ID_CUSTOM1:
		frame_length = imgsensor_info.custom1.pclk / framerate * 10 / imgsensor_info.custom1.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom1.framelength) ?
				   (frame_length - imgsensor_info.custom1.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.custom1.framelength + imgsensor.dummy_line;
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
	case MSDK_SCENARIO_ID_CUSTOM1:
		*framerate = imgsensor_info.custom1.max_framerate;
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

#if 0
static void hdr_set_motion(kal_uint32 motion)
{
    write_cmos_sensor_16_16(0x6028, 0x2001);
    write_cmos_sensor_16_16(0x602A, 0x252A);
    if (motion > 0)
        write_cmos_sensor_16_16(0x6F12, 0x0001);
    else
        write_cmos_sensor_16_16(0x6F12, 0x0000);

    LOG_INF("motion=%d\n", motion);
}
#endif
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
    //Antaiui <AI_BSP_CAM> <yaoyc> <2020-12-01> add for 25571 android R camera hal request begin
    case SENSOR_FEATURE_GET_GAIN_RANGE_BY_SCENARIO:
        *(feature_data + 1) = imgsensor_info.min_gain;
        *(feature_data + 2) = imgsensor_info.max_gain;
        break;
    case SENSOR_FEATURE_GET_BASE_GAIN_ISO_AND_STEP:
        *(feature_data + 0) = imgsensor_info.min_gain_iso;
        *(feature_data + 1) = imgsensor_info.gain_step;
        *(feature_data + 2) = imgsensor_info.gain_type;
        break;
    case SENSOR_FEATURE_GET_MIN_SHUTTER_BY_SCENARIO:
        *(feature_data + 1) = imgsensor_info.min_shutter;
        //*(feature_data + 2) = imgsensor_info.exp_step;
        break;
    case SENSOR_FEATURE_GET_BINNING_TYPE:
        *feature_return_para_32 = 1;
        pr_debug("SENSOR_FEATURE_GET_BINNING_TYPE AE_binning_type:%d,\n",
                *feature_return_para_32);
        *feature_para_len = 4;
        break;
    case SENSOR_FEATURE_GET_FRAME_CTRL_INFO_BY_SCENARIO:
        /*
         * 1, if driver support new sw frame sync
         * set_shutter_frame_length() support third para auto_extend_en
         */
        pr_info("SENSOR_FEATURE_GET_FRAME_CTRL_INFO_BY_SCENARIO \n");
        *(feature_data + 1) = 0;
        /* margin info by scenario */
        *(feature_data + 2) = imgsensor_info.margin;
        break;
    case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ_BY_SCENARIO:
        switch (*feature_data) {
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.cap.pclk;
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.normal_video.pclk;
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.hs_video.pclk;
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.slim_video.pclk;
            break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom1.pclk;
			break;
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        default:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.pre.pclk;
            break;
        }
        break;
    case SENSOR_FEATURE_GET_PERIOD_BY_SCENARIO:
        switch (*feature_data) {
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = (imgsensor_info.cap.framelength << 16)
                + imgsensor_info.cap.linelength;
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = (imgsensor_info.normal_video.framelength << 16)
                + imgsensor_info.normal_video.linelength;
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = (imgsensor_info.hs_video.framelength << 16)
                + imgsensor_info.hs_video.linelength;
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = (imgsensor_info.slim_video.framelength << 16)
                + imgsensor_info.slim_video.linelength;
            break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom1.framelength << 16)
				+ imgsensor_info.custom1.linelength;
			break;
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        default:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = (imgsensor_info.pre.framelength << 16)
                + imgsensor_info.pre.linelength;
            break;
        }
        break;
    //Antaiui <AI_BSP_CAM> <yaoyc> <2020-12-01> add for 25571 android R camera hal request end
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
		case MSDK_SCENARIO_ID_CUSTOM1:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[5], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
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
        case MSDK_SCENARIO_ID_CUSTOM1:
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
        case MSDK_SCENARIO_ID_CUSTOM1:
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
            case MSDK_SCENARIO_ID_CUSTOM1:
                rate = imgsensor_info.custom1.mipi_pixel_rate;
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
//Antai <AI_BSP_CAM> <yaoyc> <2022-07-15> add for mt6761 begin
	case SENSOR_FEATURE_GET_PIXEL_RATE: {
            kal_uint32 rate;
            LOG_INF("pangfei SENSOR_FEATURE_GET_PIXEL_RATE :\n");
            switch (*feature_data) {
            case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                rate = (imgsensor_info.cap.pclk /
                        (imgsensor_info.cap.linelength - 80))*
                    imgsensor_info.cap.grabwindow_width;
                break;
            case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                rate = (imgsensor_info.normal_video.pclk /
                        (imgsensor_info.normal_video.linelength - 80))*
                    imgsensor_info.normal_video.grabwindow_width;
                break;
            case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
                rate = (imgsensor_info.hs_video.pclk /
                        (imgsensor_info.hs_video.linelength - 80))*
                    imgsensor_info.hs_video.grabwindow_width;
                break;
            case MSDK_SCENARIO_ID_SLIM_VIDEO:
                rate = (imgsensor_info.slim_video.pclk /
                        (imgsensor_info.slim_video.linelength - 80))*
                    imgsensor_info.slim_video.grabwindow_width;
                break;
            case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            default:
                rate = (imgsensor_info.pre.pclk /
                        (imgsensor_info.pre.linelength - 80))*
                    imgsensor_info.pre.grabwindow_width;
                break;
                LOG_INF("pangfei SENSOR_FEATURE_GET_PIXEL_RATE :rate %d \n", rate);
            }
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = rate;
            break;
        }
//Antai <AI_BSP_CAM> <yaoyc> <2022-07-15> add for mt6761 end
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
    #if 0
    case SENSOR_FEATURE_SET_HDR_MOTION:
        hdr_set_motion((UINT32)*feature_data);
        break;
    #endif
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


UINT32 S5KJN1SQ03_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc != NULL)
        *pfFunc = &sensor_func;
    return ERROR_NONE;
}   /*  S5K2X5_MIPI_RAW_SensorInit  */
