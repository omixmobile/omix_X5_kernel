/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

/*****************************************************************************
 *
 * Filename:
 * ---------
 *     OV13853mipi_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 * Setting version:
 * ------------
 *   update full pd setting for OV13853EB_03B
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/

#define PFX "OV13853_camera_sensor"
// #define pr_fmt(fmt) PFX "[%s] " fmt, __func__

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/types.h>

#include "ov13853mipiraw_Sensor.h"

#ifdef CONFIG_CAM_PERFORMANCE_IMPROVE
#include <linux/dma-mapping.h>
#endif

#define LOG_INF(format, args...)    pr_info(PFX "[%s] " format, __FUNCTION__, ##args)
#define LOGE(format, args...)   pr_err(PFX, "[%s] " format, __FUNCTION__, ##args)

static DEFINE_SPINLOCK(imgsensor_drv_lock);

static int ov13853_chip_ver = OV13853_R2A;
static int m_bpc_select=0;

static struct imgsensor_info_struct imgsensor_info = {
    .sensor_id = OV13853_SENSOR_ID,        //record sensor id defined in Kd_imgsensor.h

    .checksum_value = 0xa261a12b,        //checksum value for Camera Auto Test
    .pre = {
        .pclk = 240000000,                //record different mode's pclk
        .linelength = 2400,                //record different mode's linelength
        .framelength = 3328,            //record different mode's framelength
        .startx = 0,                    //record different mode's startx of grabwindow
        .starty = 0,                    //record different mode's starty of grabwindow
        .grabwindow_width = 2096,        //record different mode's width of grabwindow
        .grabwindow_height = 1552,        //record different mode's height of grabwindow
        /*     following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario    */
        .mipi_data_lp2hs_settle_dc = 100,//unit , ns
        // Antaiui <AI_BSP_CAM> <xieht> <2022-02-23> mipi pixel rate modify begin
        .mipi_pixel_rate = 255000000,
        // Antaiui <AI_BSP_CAM> <xieht> <2022-02-23> mipi pixel rate modify end
        .max_framerate = 300,
    },
    .cap = {
        .pclk = 480000000,
        .linelength = 4800,
        .framelength = 3328,
        .startx = 0,
        .starty = 0,
        //.grabwindow_width = 4096, //4192,
        //.grabwindow_height = 3008,//3104,
        .grabwindow_width = 4192,
        .grabwindow_height = 3104,
        .mipi_data_lp2hs_settle_dc = 100,//unit , ns
        // Antaiui <AI_BSP_CAM> <xieht> <2022-02-23> mipi pixel rate modify begin
        .mipi_pixel_rate = 476000000,
        // Antaiui <AI_BSP_CAM> <xieht> <2022-02-23> mipi pixel rate modify end
        .max_framerate = 300,

    },
    .cap1 = {	//capture for PIP 24fps relative information, capture1 mode must use same framelength, linelength with Capture mode for shutter calculate
        .pclk = 240000000,
        .linelength = 4800,
        .framelength = 3328,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4192,
        .grabwindow_height = 3104,
        .mipi_data_lp2hs_settle_dc = 100,//unit , ns
        // Antaiui <AI_BSP_CAM> <xieht> <2022-02-23> mipi pixel rate modify begin
        .mipi_pixel_rate = 62400000,
        // Antaiui <AI_BSP_CAM> <xieht> <2022-02-23> mipi pixel rate modify end
        .max_framerate = 150,    //less than 13M(include 13M),cap1 max framerate is 24fps,16M max framerate is 20fps, 20M max framerate is 15fps
    },
    .normal_video = {
        .pclk = 480000000,
        .linelength = 4800,
        .framelength = 3328,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4192,
        .grabwindow_height = 3104,
        .mipi_data_lp2hs_settle_dc = 100,//unit , ns
        // Antaiui <AI_BSP_CAM> <xieht> <2022-02-23> mipi pixel rate modify begin
        .mipi_pixel_rate = 476000000,
        // Antaiui <AI_BSP_CAM> <xieht> <2022-02-23> mipi pixel rate modify end
        .max_framerate = 300,
    },
    .hs_video = {
        .pclk = 240000000,
        .linelength = 2968, //2968,2400
        .framelength = 674, //674,831
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 640,  //640,1280
        .grabwindow_height = 480,  //480,720
        .mipi_data_lp2hs_settle_dc = 100,//unit , ns
        // Antaiui <AI_BSP_CAM> <xieht> <2022-02-23> mipi pixel rate modify begin
        .mipi_pixel_rate = 59000000,
        // Antaiui <AI_BSP_CAM> <xieht> <2022-02-23> mipi pixel rate modify end
        .max_framerate = 1200,
    },
    .slim_video = {
        .pclk = 240000000,
        .linelength = 2400, //9600,//2400,
        .framelength = 3324, //834,//3328,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 1280,
        .grabwindow_height = 720,
        .mipi_data_lp2hs_settle_dc = 100,//unit , ns
        // Antaiui <AI_BSP_CAM> <xieht> <2022-02-23> mipi pixel modify begin
        .mipi_pixel_rate = 256000000,
        // Antaiui <AI_BSP_CAM> <xieht> <2022-02-23> mipi pixel modify end
        .max_framerate = 300,
    },
    .margin = 8,            //sensor framelength & shutter margin
    .min_shutter = 0x3,        //min shutter
    .max_frame_length = 0x7fff,//max framelength by sensor register's limitation
    .ae_shut_delay_frame = 0,    //shutter delay frame for AE cycle, 2 frame with ispGain_delay-shut_delay=2-0=2
    .ae_sensor_gain_delay_frame = 0,//sensor gain delay frame for AE cycle,2 frame with ispGain_delay-sensor_gain_delay=2-0=2
    .ae_ispGain_delay_frame = 2,//isp gain delay frame for AE cycle
    .ihdr_support = 0,      //1, support; 0,not support
    .ihdr_le_firstline = 0,  //1,le first ; 0, se first
    .sensor_mode_num = 5,      //support sensor mode num

    .cap_delay_frame = 3,        //enter capture delay frame num
    .pre_delay_frame = 2,         //enter preview delay frame num
    .video_delay_frame = 2,        //enter video delay frame num
    .hs_video_delay_frame = 2,    //enter high speed video  delay frame num
    .slim_video_delay_frame = 2,//enter slim video delay frame num

    .isp_driving_current = ISP_DRIVING_8MA, //mclk driving current
    .sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,//sensor_interface_type
    .mipi_sensor_type = MIPI_OPHY_NCSI2, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
    .mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO,//0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
    .sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_B,//sensor output first pixel color
    .mclk = 24,//mclk value, suggest 24 or 26 for 24Mhz or 26Mhz
    .mipi_lane_num = SENSOR_MIPI_4_LANE,//mipi lane num
    .i2c_addr_table = { 0x20,0x6c, 0xff},//record sensor support all write id addr, only supprt 4must end with 0xff
};

static struct imgsensor_struct imgsensor = {
    .mirror = IMAGE_V_MIRROR,                //mirrorflip information
    .sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
    .shutter = 0x3D0,                    //current shutter
    .gain = 0x100,                        //current gain
    .dummy_pixel = 0,                    //current dummypixel
    .dummy_line = 0,                    //current dummyline
    .current_fps = 0,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
    .autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
    .test_pattern = KAL_FALSE,        //test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
    .current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
    .ihdr_en = 0, //sensor need support LE, SE with HDR feature
    .i2c_write_id = 0x00,//record current sensor's i2c write id
};

/* Sensor output window information */
// Antaiui <AI_BSP_CAM> <xieht> <2022-02-23> w1size resize begin
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] ={
    { 4192, 3104,   0,   0, 4192, 3104, 2096, 1552,   0,   0, 2096, 1552, 0, 0, 2096, 1552}, // Preview
    { 4192, 3104,   0,   0, 4192, 3104, 4192, 3104,   0,   0, 4192, 3104, 0, 0, 4192, 3104}, // capture
    { 4192, 3104,   0,   0, 4192, 3104, 4192, 3104,   0,   0, 4192, 3104, 0, 0, 4192, 3104}, // video
    // { 4192, 3104,   0,   0, 4192, 3104, 2096, 1552, 728, 536,  640,  480, 0, 0,  640,  480}, // hight speed video
    { 4192, 3104,   0,   0, 4192, 3104, 1048,  776, 204, 148,  640,  480, 0, 0,  640,  480}, // hight speed video
    { 4192, 3104,   0, 372, 4192, 2360, 2096, 1180, 408, 230, 1280,  720, 0, 0, 1280,  720}
};
// Antaiui <AI_BSP_CAM> <xieht> <2022-02-23> w1size resize end

/*PD information update*/
static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info =
{
    .i4OffsetX =  64,
    .i4OffsetY = 64,
    .i4PitchX  = 32,
    .i4PitchY  = 32,
    .i4PairNum  =8,
    .i4SubBlkW  =16,
    .i4SubBlkH  =8,//need check xb.pang
    .i4PosL = {{78,70},{94,70},{70,74},{86,74},{78,86},{94,86},{70,90},{86,90}},
    .i4PosR = {{78,66},{94,66},{70,78},{86,78},{78,82},{94,82},{70,94},{86,94}},
};

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
    kal_uint16 get_byte=0;
    char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    iReadRegI2C(pusendcmd , 2, (u8*)&get_byte,1,imgsensor.i2c_write_id);

    return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
    char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) , (char)(para & 0xFF)};

    iWriteRegI2C(pusendcmd , 3, imgsensor.i2c_write_id);
}

static void set_dummy(void)
{
    LOG_INF("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);
    /* you can set dummy by imgsensor.dummy_line and imgsensor.dummy_pixel, or you can set dummy by imgsensor.frame_length and imgsensor.line_length */
    write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
    write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
    write_cmos_sensor(0x380c, imgsensor.line_length >> 8);
    write_cmos_sensor(0x380d, imgsensor.line_length & 0xFF);
}

static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
    //kal_int16 dummy_line;
    kal_uint32 frame_length = imgsensor.frame_length;
    //unsigned long flags;

    LOG_INF("framerate = %d, min framelength should enable = %d \n", framerate,min_framelength_en);

    frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;

    spin_lock(&imgsensor_drv_lock);
    imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length;
    imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
    //dummy_line = frame_length - imgsensor.min_frame_length;
    //if (dummy_line < 0)
    //    imgsensor.dummy_line = 0;
    //else
    //    imgsensor.dummy_line = dummy_line;
    //imgsensor.frame_length = frame_length + imgsensor.dummy_line;
    if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
        imgsensor.frame_length = imgsensor_info.max_frame_length;
        imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
    }
    if (min_framelength_en)
        imgsensor.min_frame_length = imgsensor.frame_length;
    spin_unlock(&imgsensor_drv_lock);

    set_dummy();
}

static void write_shutter(kal_uint16 shutter)
{
    unsigned long flags;
    kal_uint16 realtime_fps = 0;
    //kal_uint32 frame_length = 0;

    spin_lock_irqsave(&imgsensor_drv_lock, flags);
    imgsensor.shutter = shutter;
    spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
    LOG_INF("Enter! shutter =%d \n", shutter);

    //write_shutter(shutter);
    /* 0x3500, 0x3501, 0x3502 will increase VBLANK to get exposure larger than frame exposure */
    /* AE doesn't update sensor gain at capture mode, thus extra exposure lines must be updated here. */

    // OV Recommend Solution
    // if shutter bigger than frame_length, should extend frame length first
    spin_lock(&imgsensor_drv_lock);
    if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
        imgsensor.frame_length = shutter + imgsensor_info.margin;
    else
        imgsensor.frame_length = imgsensor.min_frame_length;
    if (imgsensor.frame_length > imgsensor_info.max_frame_length)
        imgsensor.frame_length = imgsensor_info.max_frame_length;
    spin_unlock(&imgsensor_drv_lock);

    shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
    shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

    // Framelength should be an even number
    shutter = (shutter >> 1) << 1;
    imgsensor.frame_length = (imgsensor.frame_length >> 1) << 1;

    if (imgsensor.autoflicker_en) {
        realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
        if(realtime_fps >= 297 && realtime_fps <= 305)
            set_max_framerate(296,0);
        else if(realtime_fps >= 147 && realtime_fps <= 150)
            set_max_framerate(146,0);
        else {
            // Extend frame length
            write_cmos_sensor(0x380e, (imgsensor.frame_length >> 8)&0x7f);
            write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
        }
    } else {
        // Extend frame length
        //write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
        //write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
        write_cmos_sensor(0x380e, (imgsensor.frame_length >> 8)&0x7f);
        write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
    }

    // Update Shutter
    // write_cmos_sensor(0x3502, (shutter << 4) & 0xFF);
    // write_cmos_sensor(0x3501, (shutter >> 4) & 0xFF);
    // write_cmos_sensor(0x3500, (shutter >> 12) & 0x0F);

    write_cmos_sensor(0x3500, (shutter>>12) & 0x0F);
    write_cmos_sensor(0x3501, (shutter>>4) & 0xFF);
    write_cmos_sensor(0x3502, (shutter<<4) & 0xF0);
    LOG_INF("Exit! shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);
}

static void set_shutter(kal_uint32 shutter)
{
    unsigned long flags;

    spin_lock_irqsave(&imgsensor_drv_lock, flags);
    imgsensor.shutter = shutter;
    spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

    write_shutter(shutter);
}

static kal_uint16 gain2reg(const kal_uint16 gain)
{
    kal_uint16 iReg = 0x0000;
    //kal_uint16 iGain=gain;

    if (ov13853_chip_ver == OV13853_R1A) {
        iReg = gain * 32 / BASEGAIN;
        if(iReg < 0x20) {
            iReg = 0x20;
        }
        if(iReg > 0xfc) {
            iReg = 0xfc;
        }
        //SENSORDB("[OV13853Gain2Reg]: isp gain:%d,sensor gain:0x%x\n",iGain,iReg);
    } else if(ov13853_chip_ver == OV13853_R2A) {
        iReg = gain * 16 / BASEGAIN;
        if(iReg < 0x10) {
            iReg = 0x10;
        }
        if(iReg > 0xf8) {
            iReg = 0xf8;
        }
    }
    return iReg;//ov13853. sensorGlobalGain
}

static kal_uint16 set_gain(kal_uint16 gain)
{
    kal_uint16 reg_gain;

    reg_gain = gain2reg(gain);

    spin_lock(&imgsensor_drv_lock);
    imgsensor.gain = reg_gain;
    spin_unlock(&imgsensor_drv_lock);

    LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

    write_cmos_sensor(0x350a, reg_gain >> 8);
    write_cmos_sensor(0x350b, reg_gain & 0xFF);

    return gain;
}

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
    LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n",le,se,gain);
    if (imgsensor.ihdr_en) {
        spin_lock(&imgsensor_drv_lock);
        if (le > imgsensor.min_frame_length - imgsensor_info.margin)
            imgsensor.frame_length = le + imgsensor_info.margin;
        else
            imgsensor.frame_length = imgsensor.min_frame_length;
        if (imgsensor.frame_length > imgsensor_info.max_frame_length)
            imgsensor.frame_length = imgsensor_info.max_frame_length;
        spin_unlock(&imgsensor_drv_lock);

        if (le < imgsensor_info.min_shutter) le = imgsensor_info.min_shutter;
        if (se < imgsensor_info.min_shutter) se = imgsensor_info.min_shutter;

        // Extend frame length first
        write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
        write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);

        write_cmos_sensor(0x3502, (le << 4) & 0xFF);
        write_cmos_sensor(0x3501, (le >> 4) & 0xFF);
        write_cmos_sensor(0x3500, (le >> 12) & 0x0F);

        write_cmos_sensor(0x3508, (se << 4) & 0xFF);
        write_cmos_sensor(0x3507, (se >> 4) & 0xFF);
        write_cmos_sensor(0x3506, (se >> 12) & 0x0F);

        set_gain(gain);
    }
}

static void set_mirror_flip(kal_uint8 image_mirror)
{
    LOG_INF("image_mirror = %d\n", image_mirror);

    /********************************************************
     *
     *   0x3820[2] ISP Vertical flip
     *   0x3820[1] Sensor Vertical flip
     *
     *   0x3821[2] ISP Horizontal mirror
     *   0x3821[1] Sensor Horizontal mirror
     *
     *   ISP and Sensor flip or mirror register bit should be the same!!
     *
     ********************************************************/

    switch (image_mirror) {
        case IMAGE_NORMAL:
            write_cmos_sensor(0x3820,((read_cmos_sensor(0x3820) & 0xFB) | 0x00));
            write_cmos_sensor(0x3821,((read_cmos_sensor(0x3821) & 0xFB) | 0x00));
            break;
        case IMAGE_H_MIRROR:
            write_cmos_sensor(0x3820,((read_cmos_sensor(0x3820) & 0xFB) | 0x00));
            write_cmos_sensor(0x3821,((read_cmos_sensor(0x3821) & 0xFF) | 0x04));
            break;
        case IMAGE_V_MIRROR:
            write_cmos_sensor(0x3820,((read_cmos_sensor(0x3820) & 0xFF) | 0x04));
            write_cmos_sensor(0x3821,((read_cmos_sensor(0x3821) & 0xFB) | 0x00));
            break;
        case IMAGE_HV_MIRROR:
            write_cmos_sensor(0x3820,((read_cmos_sensor(0x3820) & 0xFF) | 0x04));
            write_cmos_sensor(0x3821,((read_cmos_sensor(0x3821) & 0xFF) | 0x04));
            break;
        default:
            LOG_INF("Error image_mirror setting\n");
    }

}

static void night_mode(kal_bool enable)
{
    LOG_INF("night_mode do nothing");
}

static void sensor_init(void)
{
    LOG_INF("E\n");

    //0320 setting
    write_cmos_sensor(0x0103, 0x01);//
    write_cmos_sensor(0x0102, 0x01);//
    write_cmos_sensor(0x0300, 0x01);//	;for 640Mbps
    write_cmos_sensor(0x0301, 0x00);//
    write_cmos_sensor(0x0302, 0x28);//
    write_cmos_sensor(0x0303, 0x00);//
    write_cmos_sensor(0x030a, 0x00);//
    write_cmos_sensor(0x300f, 0x11);//
    write_cmos_sensor(0x3010, 0x01);//
    write_cmos_sensor(0x3011, 0x76);//
    write_cmos_sensor(0x3012, 0x41);//
    write_cmos_sensor(0x3013, 0x12);//
    write_cmos_sensor(0x3014, 0x11);//
    write_cmos_sensor(0x301f, 0x03);//
    write_cmos_sensor(0x3106, 0x00);//
    write_cmos_sensor(0x3210, 0x47);//
    write_cmos_sensor(0x3500, 0x00);//
    write_cmos_sensor(0x3501, 0x60);//
    write_cmos_sensor(0x3502, 0x00);//
    write_cmos_sensor(0x3506, 0x00);//
    write_cmos_sensor(0x3507, 0x02);//
    write_cmos_sensor(0x3508, 0x00);//
    write_cmos_sensor(0x350a, 0x00);//
    write_cmos_sensor(0x350b, 0x80);//
    write_cmos_sensor(0x350e, 0x00);//
    write_cmos_sensor(0x350f, 0x10);//
    write_cmos_sensor(0x351a, 0x00);//
    write_cmos_sensor(0x351b, 0x10);//
    write_cmos_sensor(0x351c, 0x00);//
    write_cmos_sensor(0x351d, 0x20);//
    write_cmos_sensor(0x351e, 0x00);//
    write_cmos_sensor(0x351f, 0x40);//
    write_cmos_sensor(0x3520, 0x00);//
    write_cmos_sensor(0x3521, 0x80);//
    write_cmos_sensor(0x3600, 0xc0);//
    write_cmos_sensor(0x3601, 0xfc);//
    write_cmos_sensor(0x3602, 0x02);//
    write_cmos_sensor(0x3603, 0x78);//
    write_cmos_sensor(0x3604, 0xb1);//
    write_cmos_sensor(0x3605, 0x95);//
    write_cmos_sensor(0x3606, 0x73);//
    write_cmos_sensor(0x3607, 0x07);//
    write_cmos_sensor(0x3609, 0x40);//
    write_cmos_sensor(0x360a, 0x30);//
    write_cmos_sensor(0x360b, 0x91);//
    write_cmos_sensor(0x360c, 0x09);//
    write_cmos_sensor(0x360f, 0x02);//
    write_cmos_sensor(0x3611, 0x10);//
    write_cmos_sensor(0x3612, 0x27);//
    write_cmos_sensor(0x3613, 0x33);//
    write_cmos_sensor(0x3615, 0x0c);//
    write_cmos_sensor(0x3616, 0x0e);//
    write_cmos_sensor(0x3641, 0x02);//
    write_cmos_sensor(0x3660, 0x82);//
    write_cmos_sensor(0x3668, 0x54);//
    write_cmos_sensor(0x3669, 0x00);//
    write_cmos_sensor(0x366a, 0x3f);//
    write_cmos_sensor(0x3667, 0xa0);//
    write_cmos_sensor(0x3702, 0x40);//
    write_cmos_sensor(0x3703, 0x44);//
    write_cmos_sensor(0x3704, 0x2c);//
    write_cmos_sensor(0x3705, 0x01);//
    write_cmos_sensor(0x3706, 0x15);//
    write_cmos_sensor(0x3707, 0x44);//
    write_cmos_sensor(0x3708, 0x3c);//
    write_cmos_sensor(0x3709, 0x1f);//
    write_cmos_sensor(0x370a, 0x27);//
    write_cmos_sensor(0x370b, 0x3c);//
    write_cmos_sensor(0x3720, 0x55);//
    write_cmos_sensor(0x3722, 0x84);//
    write_cmos_sensor(0x3729, 0x00);//
    write_cmos_sensor(0x3728, 0x40);//
    write_cmos_sensor(0x372a, 0x00);//
    write_cmos_sensor(0x372b, 0x02);//
    write_cmos_sensor(0x372e, 0x22);//
    write_cmos_sensor(0x372f, 0xa0);//
    write_cmos_sensor(0x3730, 0x00);//
    write_cmos_sensor(0x3731, 0x00);//
    write_cmos_sensor(0x3732, 0x00);//
    write_cmos_sensor(0x3733, 0x00);//
    write_cmos_sensor(0x3710, 0x28);//
    write_cmos_sensor(0x3716, 0x03);//
    write_cmos_sensor(0x3718, 0x1c);//
    write_cmos_sensor(0x3719, 0x0c);//
    write_cmos_sensor(0x371a, 0x08);//
    write_cmos_sensor(0x371c, 0xfc);//
    write_cmos_sensor(0x3748, 0x00);//
    write_cmos_sensor(0x3760, 0x13);//
    write_cmos_sensor(0x3761, 0x33);//
    write_cmos_sensor(0x3762, 0x86);//
    write_cmos_sensor(0x3763, 0x16);//
    write_cmos_sensor(0x3767, 0x24);//
    write_cmos_sensor(0x3768, 0x06);//
    write_cmos_sensor(0x3769, 0x45);//
    write_cmos_sensor(0x376c, 0x23);//
    write_cmos_sensor(0x376f, 0x80);//
    write_cmos_sensor(0x3773, 0x06);//
    write_cmos_sensor(0x3d84, 0x00);//
    write_cmos_sensor(0x3d85, 0x17);//
    write_cmos_sensor(0x3d8c, 0x73);//
    write_cmos_sensor(0x3d8d, 0xbf);//
    write_cmos_sensor(0x3800, 0x00);//
    write_cmos_sensor(0x3801, 0x00);//
    write_cmos_sensor(0x3802, 0x00);//
    write_cmos_sensor(0x3803, 0x08);//
    write_cmos_sensor(0x3804, 0x10);//
    write_cmos_sensor(0x3805, 0x9f);//
    write_cmos_sensor(0x3806, 0x0c);//
    write_cmos_sensor(0x3807, 0x4f);//
    write_cmos_sensor(0x3808, 0x08);//
    write_cmos_sensor(0x3809, 0x40);//
    write_cmos_sensor(0x380a, 0x06);//
    write_cmos_sensor(0x380b, 0x20);//
    write_cmos_sensor(0x380c, 0x09);//
    write_cmos_sensor(0x380d, 0x60);//
    write_cmos_sensor(0x380e, 0x0d);//
    write_cmos_sensor(0x380f, 0x00);//
    write_cmos_sensor(0x3810, 0x00);//
    write_cmos_sensor(0x3811, 0x08);//
    write_cmos_sensor(0x3812, 0x00);//
    write_cmos_sensor(0x3813, 0x00);//
    write_cmos_sensor(0x3814, 0x31);//
    write_cmos_sensor(0x3815, 0x31);//
    write_cmos_sensor(0x3820, 0x01);//
    write_cmos_sensor(0x3821, 0x06);//
    write_cmos_sensor(0x3823, 0x00);//
    write_cmos_sensor(0x3826, 0x00);//
    write_cmos_sensor(0x3827, 0x02);//
    write_cmos_sensor(0x3834, 0x00);//
    write_cmos_sensor(0x3835, 0x1c);//
    write_cmos_sensor(0x3836, 0x08);//
    write_cmos_sensor(0x3837, 0x02);//
    write_cmos_sensor(0x4000, 0xf1);//
    write_cmos_sensor(0x4001, 0x00);//
    write_cmos_sensor(0x4005, 0x40);//  add obtarget=0x40
    write_cmos_sensor(0x400b, 0x0c);//
    write_cmos_sensor(0x4011, 0x00);//
    write_cmos_sensor(0x401a, 0x00);//
    write_cmos_sensor(0x401b, 0x00);//
    write_cmos_sensor(0x401c, 0x00);//
    write_cmos_sensor(0x401d, 0x00);//
    write_cmos_sensor(0x4020, 0x00);//
    write_cmos_sensor(0x4021, 0xe4);//
    write_cmos_sensor(0x4022, 0x04);//
    write_cmos_sensor(0x4023, 0xd7);//
    write_cmos_sensor(0x4024, 0x05);//
    write_cmos_sensor(0x4025, 0xbc);//
    write_cmos_sensor(0x4026, 0x05);//
    write_cmos_sensor(0x4027, 0xbf);//
    write_cmos_sensor(0x4028, 0x00);//
    write_cmos_sensor(0x4029, 0x02);//
    write_cmos_sensor(0x402a, 0x04);//
    write_cmos_sensor(0x402b, 0x08);//
    write_cmos_sensor(0x402c, 0x02);//
    write_cmos_sensor(0x402d, 0x02);//
    write_cmos_sensor(0x402e, 0x0c);//
    write_cmos_sensor(0x402f, 0x08);//
    write_cmos_sensor(0x403d, 0x2c);//
    write_cmos_sensor(0x403f, 0x7f);//
    write_cmos_sensor(0x4041, 0x07);//
    write_cmos_sensor(0x4500, 0x82);//
    write_cmos_sensor(0x4501, 0x78);//	;H digital skip for PD pixel; 3C for H digital bin
    write_cmos_sensor(0x458b, 0x00);//
    write_cmos_sensor(0x459c, 0x00);//
    write_cmos_sensor(0x459d, 0x00);//
    write_cmos_sensor(0x459e, 0x00);//
    write_cmos_sensor(0x4601, 0x83);//
    write_cmos_sensor(0x4602, 0x22);//
    write_cmos_sensor(0x4603, 0x01);//
    write_cmos_sensor(0x4837, 0x19);//
    write_cmos_sensor(0x4d00, 0x04);//
    write_cmos_sensor(0x4d01, 0x42);//
    write_cmos_sensor(0x4d02, 0xd1);//
    write_cmos_sensor(0x4d03, 0x90);//
    write_cmos_sensor(0x4d04, 0x66);//
    write_cmos_sensor(0x4d05, 0x65);//
    write_cmos_sensor(0x4d0b, 0x00);//

    write_cmos_sensor(0x5001, 0x01);//
    write_cmos_sensor(0x5002, 0x07);//
    write_cmos_sensor(0x5003, 0x4f);//
    write_cmos_sensor(0x5013, 0x40);//
    write_cmos_sensor(0x501c, 0x00);//
    write_cmos_sensor(0x501d, 0x10);//
    write_cmos_sensor(0x510f, 0xfc);//
    write_cmos_sensor(0x5110, 0xf0);//
    write_cmos_sensor(0x5111, 0x10);//
    write_cmos_sensor(0x5115, 0x3c);//
    write_cmos_sensor(0x5116, 0x30);//
    write_cmos_sensor(0x5117, 0x10);//
    write_cmos_sensor(0x536d, 0x02);//
    write_cmos_sensor(0x536e, 0x67);//
    write_cmos_sensor(0x536f, 0x01);//
    write_cmos_sensor(0x5370, 0x4c);//
    write_cmos_sensor(0x5400, 0x00);//
    write_cmos_sensor(0x5400, 0x00);//
    write_cmos_sensor(0x5401, 0x61);//
    write_cmos_sensor(0x5402, 0x00);//
    write_cmos_sensor(0x5403, 0x00);//
    write_cmos_sensor(0x5404, 0x00);//
    write_cmos_sensor(0x5405, 0x40);//
    write_cmos_sensor(0x540c, 0x05);//
    write_cmos_sensor(0x5501, 0x00);//
    write_cmos_sensor(0x5b00, 0x00);//
    write_cmos_sensor(0x5b01, 0x00);//
    write_cmos_sensor(0x5b02, 0x01);//
    write_cmos_sensor(0x5b03, 0xff);//
    write_cmos_sensor(0x5b04, 0x02);//
    write_cmos_sensor(0x5b05, 0x6c);//
    write_cmos_sensor(0x5b09, 0x02);//
    write_cmos_sensor(0x5e00, 0x00);//
    write_cmos_sensor(0x5e10, 0x1c);//
    write_cmos_sensor(0x0100, 0x01);//
    write_cmos_sensor(0x5b20, 0x02);//
    write_cmos_sensor(0x5b21, 0x37);//
    write_cmos_sensor(0x5b22, 0x0c);//
    write_cmos_sensor(0x5b23, 0x37);//
    write_cmos_sensor(0x5101, 0x00);//
    write_cmos_sensor(0x5102, 0x00);//
    write_cmos_sensor(0x5103, 0x00);//
    write_cmos_sensor(0x5104, 0x00);//
    write_cmos_sensor(0x5105, 0x00);//
    write_cmos_sensor(0x5106, 0x00);//
    //write_cmos_sensor(0x5107, 0x01);//
    //write_cmos_sensor(0x5108, 0x01);//
    //write_cmos_sensor(0x5109, 0x01);//
    write_cmos_sensor(0x510f, 0xfd);//
    write_cmos_sensor(0x5110, 0xf0);//
    write_cmos_sensor(0x5111, 0x10);//
    if (m_bpc_select == 1) {
        LOG_INF("bpc select 1 \n");
        write_cmos_sensor(0x5107, 0x01);//
        write_cmos_sensor(0x5108, 0x01);//
        write_cmos_sensor(0x5109, 0x01);//
        write_cmos_sensor(0x5000, 0x0e);//
        write_cmos_sensor(0x5100, 0x30);//
        write_cmos_sensor(0x510e, 0x0c);//
    } else if(m_bpc_select == 2) {
        LOG_INF("bpc select 2 \n");
        write_cmos_sensor(0x5107, 0x01);//
        write_cmos_sensor(0x5108, 0x01);//
        write_cmos_sensor(0x5109, 0x01);//
        write_cmos_sensor(0x5000, 0x0e);//
        write_cmos_sensor(0x5100, 0x20);//
        write_cmos_sensor(0x510e, 0x00);//
    } else if(m_bpc_select == 3) {
        LOG_INF("bpc select 3 \n");
        write_cmos_sensor(0x5107, 0x01);//
        write_cmos_sensor(0x5108, 0x01);//
        write_cmos_sensor(0x5109, 0x01);//
        write_cmos_sensor(0x5000, 0x0e);//
        write_cmos_sensor(0x5100, 0x30);//
        write_cmos_sensor(0x510e, 0x0c);//
    } else {
        LOG_INF("bpc select default \n");
        //write_cmos_sensor(0x5000, 0x0e);//
        //write_cmos_sensor(0x5100, 0x30);//
        //write_cmos_sensor(0x510e, 0x00);//
        write_cmos_sensor(0x5000, 0x0e);//
        write_cmos_sensor(0x5100, 0x30);//
        write_cmos_sensor(0x5107, 0x00);//
        write_cmos_sensor(0x5108, 0x00);//
        write_cmos_sensor(0x5109, 0x00);//
        write_cmos_sensor(0x510e, 0x00);//
    }
    write_cmos_sensor(0x0100, 0x0);
}

static void preview_setting(void)
{
    LOG_INF("E\n");

    write_cmos_sensor(0x0100, 0x00); //
    write_cmos_sensor(0x0300, 0x01); //	;for 640Mbps
    write_cmos_sensor(0x0301, 0x00); //
    write_cmos_sensor(0x0302, 0x28); //
    write_cmos_sensor(0x0303, 0x00); //
    write_cmos_sensor(0x3501, 0x60); //
    write_cmos_sensor(0x3612, 0x27); //
    write_cmos_sensor(0x370a, 0x27); //
    write_cmos_sensor(0x3729, 0x00); //
    write_cmos_sensor(0x372a, 0x00); //
    write_cmos_sensor(0x3718, 0x1c); //
    write_cmos_sensor(0x372f, 0xa0); //
    write_cmos_sensor(0x3801, 0x00); //
    write_cmos_sensor(0x3802, 0x00); //
    write_cmos_sensor(0x3803, 0x08); //
    write_cmos_sensor(0x3805, 0x9f); //
    write_cmos_sensor(0x3806, 0x0c); //
    write_cmos_sensor(0x3807, 0x4f); //
    write_cmos_sensor(0x3808, 0x08); //
    write_cmos_sensor(0x3809, 0x40); //
    write_cmos_sensor(0x380a, 0x06); //
    write_cmos_sensor(0x380b, 0x20); //
    //write_cmos_sensor(0x380c, 0x09); //
    //write_cmos_sensor(0x380d, 0x60); //
    //write_cmos_sensor(0x380e, 0x0d); //
    //write_cmos_sensor(0x380f, 0x00); //
    write_cmos_sensor(0x380c, ((imgsensor_info.pre.linelength >> 8) & 0xFF)); // hts = 2688
    write_cmos_sensor(0x380d, (imgsensor_info.pre.linelength & 0xFF));        // hts
    write_cmos_sensor(0x380e, ((imgsensor_info.pre.framelength >> 8) & 0xFF));  // vts = 1984
    write_cmos_sensor(0x380f, (imgsensor_info.pre.framelength & 0xFF));         // vts
    write_cmos_sensor(0x3810, 0x00); //
    write_cmos_sensor(0x3811, 0x08); //
    write_cmos_sensor(0x3813, 0x00); // 0x02
    write_cmos_sensor(0x3814, 0x31); //
    write_cmos_sensor(0x3815, 0x31); //
    write_cmos_sensor(0x3820, 0x01); //
    write_cmos_sensor(0x3821, 0x06); //
    write_cmos_sensor(0x3836, 0x08); //
    write_cmos_sensor(0x3837, 0x02); //
    write_cmos_sensor(0x4020, 0x00); //
    write_cmos_sensor(0x4021, 0xe4); //
    write_cmos_sensor(0x4022, 0x04); //
    write_cmos_sensor(0x4023, 0xd7); //
    write_cmos_sensor(0x4024, 0x05); //
    write_cmos_sensor(0x4025, 0xbc); //
    write_cmos_sensor(0x4026, 0x05); //
    write_cmos_sensor(0x4027, 0xbf); //
    write_cmos_sensor(0x4501, 0x78); //	;H digital skip for PD pixel; 3C for H digital bin
    write_cmos_sensor(0x4601, 0x83); //
    write_cmos_sensor(0x4603, 0x01); //
    write_cmos_sensor(0x4837, 0x19); //
    write_cmos_sensor(0x5401, 0x61); //
    write_cmos_sensor(0x5405, 0x40); //
    write_cmos_sensor(0x5b21, 0x37); //
    write_cmos_sensor(0x5b22, 0x0c); //
    write_cmos_sensor(0x5b23, 0x37); //
    /* Antaiui <AI_BSP_CAM> <xieht> <2022-02-24> use streaming control begin */
    // write_cmos_sensor(0x0100, 0x01); //
    // mdelay(10);
    /* Antaiui <AI_BSP_CAM> <xieht> <2022-02-24> use streaming control end */
}

int capture_first_flag = 0;
int pre_currefps = 0;
static void capture_setting(kal_uint16 currefps)
{
    LOG_INF("E! currefps:%d\n",currefps);
    if (pre_currefps != currefps) {
        capture_first_flag = 0;
        pre_currefps = currefps;
    } else {
        capture_first_flag = 1;
    }
    if (capture_first_flag == 0) {
        if (currefps == 300) {  //30fps for Normal capture & ZSD
            //full size @ 30fps
            write_cmos_sensor(0x0100, 0x00); //
            write_cmos_sensor(0x0300, 0x00); //	;for 1200Mbps
            write_cmos_sensor(0x0301, 0x00); //
            write_cmos_sensor(0x0302, 0x32); //
            write_cmos_sensor(0x0303, 0x00); //;01
            write_cmos_sensor(0x3501, 0xc0); //
            write_cmos_sensor(0x3612, 0x07); //;27
            write_cmos_sensor(0x370a, 0x24); //
            write_cmos_sensor(0x3729, 0x00); //
            write_cmos_sensor(0x372a, 0x04); //
            write_cmos_sensor(0x3718, 0x10); //
            write_cmos_sensor(0x372f, 0xa0); //
            write_cmos_sensor(0x3801, 0x0C); //
            write_cmos_sensor(0x3802, 0x00); //
            write_cmos_sensor(0x3803, 0x00); //
            write_cmos_sensor(0x3805, 0x93); //
            write_cmos_sensor(0x3806, 0x0c); //
            write_cmos_sensor(0x3807, 0x4F); //
            write_cmos_sensor(0x3808, 0x10); //
            write_cmos_sensor(0x3809, 0x80); //
            write_cmos_sensor(0x380a, 0x0c); //
            write_cmos_sensor(0x380b, 0x40); //
            //write_cmos_sensor(0x380c, 0x12); //
            //write_cmos_sensor(0x380d, 0xc0); //
            //write_cmos_sensor(0x380e, 0x0d); //
            //write_cmos_sensor(0x380f, 0x00); //
            write_cmos_sensor(0x380c, ((imgsensor_info.cap.linelength >> 8) & 0xFF)); // hts = 2688
            write_cmos_sensor(0x380d, (imgsensor_info.cap.linelength & 0xFF));        // hts
            write_cmos_sensor(0x380e, ((imgsensor_info.cap.framelength >> 8) & 0xFF));  // vts = 1984
            write_cmos_sensor(0x380f, (imgsensor_info.cap.framelength & 0xFF));         // vts
            write_cmos_sensor(0x3810, 0x00); //
            write_cmos_sensor(0x3811, 0x04); //
            write_cmos_sensor(0x3813, 0x08); //
            write_cmos_sensor(0x3814, 0x11); //
            write_cmos_sensor(0x3815, 0x11); //
            write_cmos_sensor(0x3820, 0x00); //
            write_cmos_sensor(0x3821, 0x04); //
            write_cmos_sensor(0x3836, 0x04); //
            write_cmos_sensor(0x3837, 0x01); //
            write_cmos_sensor(0x4020, 0x02); //
            write_cmos_sensor(0x4021, 0x4C); //
            write_cmos_sensor(0x4022, 0x0E); //
            write_cmos_sensor(0x4023, 0x37); //
            write_cmos_sensor(0x4024, 0x0F); //
            write_cmos_sensor(0x4025, 0x1C); //
            write_cmos_sensor(0x4026, 0x0F); //
            write_cmos_sensor(0x4027, 0x1F); //
            write_cmos_sensor(0x4501, 0x38); //
            write_cmos_sensor(0x4601, 0x04); //
            write_cmos_sensor(0x4603, 0x00); //
            write_cmos_sensor(0x4837, 0x0d); //;1b
            write_cmos_sensor(0x5401, 0x71); //
            write_cmos_sensor(0x5405, 0x80); //
            write_cmos_sensor(0x5b21, 0x91); //
            write_cmos_sensor(0x5b22, 0x02); //
            write_cmos_sensor(0x5b23, 0x91); //
            /* Antaiui <AI_BSP_CAM> <xieht> <2022-02-24> use streaming control begin */
            // write_cmos_sensor(0x0100, 0x01); ////
            /* Antaiui <AI_BSP_CAM> <xieht> <2022-02-24> use streaming control end */
        } else {
            //full size 15fps for PIP
            write_cmos_sensor(0x0100, 0x00); //
            write_cmos_sensor(0x0300, 0x00); // ;for 600Mbps
            write_cmos_sensor(0x0301, 0x00); //
            write_cmos_sensor(0x0302, 0x32); //
            write_cmos_sensor(0x0303, 0x01); //
            write_cmos_sensor(0x3501, 0xc0); //
            write_cmos_sensor(0x3612, 0x27); //
            write_cmos_sensor(0x370a, 0x24); //
            write_cmos_sensor(0x3729, 0x00); //
            write_cmos_sensor(0x372a, 0x04); //
            write_cmos_sensor(0x3718, 0x10); //
            write_cmos_sensor(0x372f, 0xa0); //
            write_cmos_sensor(0x3801, 0x0C); //
            write_cmos_sensor(0x3802, 0x00); //
            write_cmos_sensor(0x3803, 0x00); //
            write_cmos_sensor(0x3805, 0x93); //
            write_cmos_sensor(0x3806, 0x0c); //
            write_cmos_sensor(0x3807, 0x4F); //
            write_cmos_sensor(0x3808, 0x10); //
            write_cmos_sensor(0x3809, 0x80); //
            write_cmos_sensor(0x380a, 0x0c); //
            write_cmos_sensor(0x380b, 0x40); //
            //write_cmos_sensor(0x380c, 0x12); //
            //write_cmos_sensor(0x380d, 0xc0); //
            //write_cmos_sensor(0x380e, 0x0d); //
            //write_cmos_sensor(0x380f, 0x00); //
            write_cmos_sensor(0x380c, ((imgsensor_info.cap1.linelength >> 8) & 0xFF)); // hts = 2688
            write_cmos_sensor(0x380d, (imgsensor_info.cap1.linelength & 0xFF));		// hts
            write_cmos_sensor(0x380e, ((imgsensor_info.cap1.framelength >> 8) & 0xFF));  // vts = 1984
            write_cmos_sensor(0x380f, (imgsensor_info.cap1.framelength & 0xFF));		  // vts
            write_cmos_sensor(0x3810, 0x00); //
            write_cmos_sensor(0x3811, 0x04); //
            write_cmos_sensor(0x3813, 0x08); //
            write_cmos_sensor(0x3814, 0x11); //
            write_cmos_sensor(0x3815, 0x11); //
            write_cmos_sensor(0x3820, 0x00); //
            write_cmos_sensor(0x3821, 0x04); //
            write_cmos_sensor(0x3836, 0x04); //
            write_cmos_sensor(0x3837, 0x01); //
            write_cmos_sensor(0x4020, 0x02); //
            write_cmos_sensor(0x4021, 0x4C); //
            write_cmos_sensor(0x4022, 0x0E); //
            write_cmos_sensor(0x4023, 0x37); //
            write_cmos_sensor(0x4024, 0x0F); //
            write_cmos_sensor(0x4025, 0x1C); //
            write_cmos_sensor(0x4026, 0x0F); //
            write_cmos_sensor(0x4027, 0x1F); //
            write_cmos_sensor(0x4501, 0x38); //
            write_cmos_sensor(0x4601, 0x04); //
            write_cmos_sensor(0x4603, 0x00); //
            write_cmos_sensor(0x4837, 0x1b); //
            write_cmos_sensor(0x5401, 0x71); //
            write_cmos_sensor(0x5405, 0x80); //
            write_cmos_sensor(0x5b21, 0x91); //
            write_cmos_sensor(0x5b22, 0x02); //
            write_cmos_sensor(0x5b23, 0x91); //
            /* Antaiui <AI_BSP_CAM> <xieht> <2022-02-24> use streaming control begin */
            // write_cmos_sensor(0x0100, 0x01);
            /* Antaiui <AI_BSP_CAM> <xieht> <2022-02-24> use streaming control end */
        }
        capture_first_flag = 1;
    }
    // mdelay(20);
}

static void normal_video_setting(kal_uint16 currefps)
{
    LOG_INF("E! currefps:%d\n",currefps);
    //full size @ 30fps
    write_cmos_sensor(0x0100, 0x00); //
    write_cmos_sensor(0x0300, 0x00); //	;for 1200Mbps
    write_cmos_sensor(0x0301, 0x00); //
    write_cmos_sensor(0x0302, 0x32); //
    write_cmos_sensor(0x0303, 0x00); //;01
    write_cmos_sensor(0x3501, 0xc0); //
    write_cmos_sensor(0x3612, 0x07); //;27
    write_cmos_sensor(0x370a, 0x24); //
    write_cmos_sensor(0x3729, 0x00); //
    write_cmos_sensor(0x372a, 0x04); //
    write_cmos_sensor(0x3718, 0x10); //
    write_cmos_sensor(0x372f, 0xa0); //
    write_cmos_sensor(0x3801, 0x0C); //
    write_cmos_sensor(0x3802, 0x00); //
    write_cmos_sensor(0x3803, 0x00); //
    write_cmos_sensor(0x3805, 0x93); //
    write_cmos_sensor(0x3806, 0x0c); //
    write_cmos_sensor(0x3807, 0x4F); //
    write_cmos_sensor(0x3808, 0x10); //
    write_cmos_sensor(0x3809, 0x80); //
    write_cmos_sensor(0x380a, 0x0c); //
    write_cmos_sensor(0x380b, 0x40); //
    //write_cmos_sensor(0x380c, 0x12); //
    //write_cmos_sensor(0x380d, 0xc0); //
    //write_cmos_sensor(0x380e, 0x0d); //
    //write_cmos_sensor(0x380f, 0x00); //
    write_cmos_sensor(0x380c, ((imgsensor_info.normal_video.linelength >> 8) & 0xFF)); // hts = 2688
    write_cmos_sensor(0x380d, (imgsensor_info.normal_video.linelength & 0xFF));        // hts
    write_cmos_sensor(0x380e, ((imgsensor_info.normal_video.framelength >> 8) & 0xFF));  // vts = 1984
    write_cmos_sensor(0x380f, (imgsensor_info.normal_video.framelength & 0xFF));         // vts
    write_cmos_sensor(0x3810, 0x00); //
    write_cmos_sensor(0x3811, 0x04); //
    write_cmos_sensor(0x3813, 0x08); //
    write_cmos_sensor(0x3814, 0x11); //
    write_cmos_sensor(0x3815, 0x11); //
    write_cmos_sensor(0x3820, 0x00); //
    write_cmos_sensor(0x3821, 0x04); //
    write_cmos_sensor(0x3836, 0x04); //
    write_cmos_sensor(0x3837, 0x01); //
    write_cmos_sensor(0x4020, 0x02); //
    write_cmos_sensor(0x4021, 0x4C); //
    write_cmos_sensor(0x4022, 0x0E); //
    write_cmos_sensor(0x4023, 0x37); //
    write_cmos_sensor(0x4024, 0x0F); //
    write_cmos_sensor(0x4025, 0x1C); //
    write_cmos_sensor(0x4026, 0x0F); //
    write_cmos_sensor(0x4027, 0x1F); //
    write_cmos_sensor(0x4501, 0x38); //
    write_cmos_sensor(0x4601, 0x04); //
    write_cmos_sensor(0x4603, 0x00); //
    write_cmos_sensor(0x4837, 0x0d); //;1b
    write_cmos_sensor(0x5401, 0x71); //
    write_cmos_sensor(0x5405, 0x80); //
    write_cmos_sensor(0x5b21, 0x91); //
    write_cmos_sensor(0x5b22, 0x02); //
    write_cmos_sensor(0x5b23, 0x91); //

    /* Antaiui <AI_BSP_CAM> <xieht> <2022-02-24> use streaming control begin */
    // write_cmos_sensor(0x0100, 0x01); //
    /* Antaiui <AI_BSP_CAM> <xieht> <2022-02-24> use streaming control end */

    if (imgsensor.ihdr_en) {
        write_cmos_sensor(0x3821, 0x84);
        write_cmos_sensor(0x3836, 0x08);
        write_cmos_sensor(0x3837, 0x02);
        //write_cmos_sensor(0x3507, 0xc0);
        write_cmos_sensor(0x3767, 0x18);
        write_cmos_sensor(0x4029, 0x04);
        write_cmos_sensor(0x372e, 0x82);
        write_cmos_sensor(0x4001, 0x08);
        write_cmos_sensor(0x3718, 0x1C);
    }

    // mdelay(20);
}

static void hs_video_setting(void)
{
    LOG_INF("E\n");

    write_cmos_sensor(0x0100, 0x00); //
    write_cmos_sensor(0x0300, 0x01); //	;for 640Mbps
    write_cmos_sensor(0x0301, 0x00); //
    write_cmos_sensor(0x0302, 0x28); //
    write_cmos_sensor(0x0303, 0x00); //
    write_cmos_sensor(0x3501, 0x60); //
    write_cmos_sensor(0x3612, 0x27); //
    write_cmos_sensor(0x370a, 0x27); //	;ppchg_offset shift
    write_cmos_sensor(0x3729, 0x00); //	;VFPN line shift
    write_cmos_sensor(0x372a, 0x00); //
    write_cmos_sensor(0x3718, 0x10); //
    write_cmos_sensor(0x372f, 0x90); //
    write_cmos_sensor(0x3801, 0x00); //
    write_cmos_sensor(0x3802, 0x03); //
    write_cmos_sensor(0x3803, 0x58); // ;shift 2 rows in V direction
    write_cmos_sensor(0x3805, 0x9f); //
    write_cmos_sensor(0x3806, 0x09); //
    write_cmos_sensor(0x3807, 0x01); //
    write_cmos_sensor(0x3808, ((imgsensor_info.hs_video.grabwindow_width >> 8) & 0xFF)); // 0x05
    write_cmos_sensor(0x3809, (imgsensor_info.hs_video.grabwindow_width & 0xFF)); // 0x00
    write_cmos_sensor(0x380a, ((imgsensor_info.hs_video.grabwindow_height >> 8) & 0xFF)); // 0x02
    write_cmos_sensor(0x380b, (imgsensor_info.hs_video.grabwindow_height & 0xFF)); // 0xD0
    //write_cmos_sensor(0x380c, 0x09); //
    //write_cmos_sensor(0x380d, 0x60); //
    //write_cmos_sensor(0x380e, 0x03); //
    //write_cmos_sensor(0x380f, 0x3f); //
    write_cmos_sensor(0x380c, ((imgsensor_info.hs_video.linelength >> 8) & 0xFF)); // hts = 2688
    write_cmos_sensor(0x380d, (imgsensor_info.hs_video.linelength & 0xFF));        // hts
    write_cmos_sensor(0x380e, ((imgsensor_info.hs_video.framelength >> 8) & 0xFF));  // vts = 1984
    write_cmos_sensor(0x380f, (imgsensor_info.hs_video.framelength & 0xFF));         // vts
    write_cmos_sensor(0x3810, 0x03); //
    write_cmos_sensor(0x3811, 0x44); //
    write_cmos_sensor(0x3813, 0x02); //
    write_cmos_sensor(0x3814, 0x31); //
    write_cmos_sensor(0x3815, 0x31); //
    write_cmos_sensor(0x3820, 0x00); //
    write_cmos_sensor(0x3821, 0x06); //
    write_cmos_sensor(0x3836, 0x08); //
    write_cmos_sensor(0x3837, 0x02); //
    write_cmos_sensor(0x4020, 0x00); //
    write_cmos_sensor(0x4021, 0xe4); //
    write_cmos_sensor(0x4022, 0x04); //
    write_cmos_sensor(0x4023, 0xd7); //
    write_cmos_sensor(0x4024, 0x05); //
    write_cmos_sensor(0x4025, 0xbc); //
    write_cmos_sensor(0x4026, 0x05); //
    write_cmos_sensor(0x4027, 0xbf); //
    write_cmos_sensor(0x4501, 0x78); //	;H digital skip for PD pixel; 3C for H digital bin
    write_cmos_sensor(0x4601, 0x04); //
    write_cmos_sensor(0x4603, 0x00); //
    write_cmos_sensor(0x4837, 0x19); //
    write_cmos_sensor(0x5401, 0x61); //
    write_cmos_sensor(0x5405, 0x40); //
    write_cmos_sensor(0x5b21, 0x37); //
    write_cmos_sensor(0x5b22, 0x0c); //
    write_cmos_sensor(0x5b23, 0x37); //

    /* Antaiui <AI_BSP_CAM> <xieht> <2022-02-24> use streaming control begin */
    // write_cmos_sensor(0x0100, 0x01);//
    // mdelay(10);
    /* Antaiui <AI_BSP_CAM> <xieht> <2022-02-24> use streaming control end */
}

static void slim_video_setting(void)
{
    LOG_INF("E");
    write_cmos_sensor(0x0100, 0x00); //
    write_cmos_sensor(0x0300, 0x01); // ;for 640Mbps
    write_cmos_sensor(0x0301, 0x00); //
    write_cmos_sensor(0x0302, 0x28); //
    write_cmos_sensor(0x0303, 0x00); //
    write_cmos_sensor(0x3501, 0x60); //
    write_cmos_sensor(0x3612, 0x27); //
    write_cmos_sensor(0x370a, 0x27); // ;ppchg_offset shift
    write_cmos_sensor(0x3729, 0x00); // ;VFPN line shift
    write_cmos_sensor(0x372a, 0x00); //
    write_cmos_sensor(0x3718, 0x10); //
    write_cmos_sensor(0x372f, 0x90); //
    write_cmos_sensor(0x3801, 0x00); //
    write_cmos_sensor(0x3802, 0x03); //
    write_cmos_sensor(0x3803, 0x58); // ;shift 2 rows in V direction
    write_cmos_sensor(0x3805, 0x9f); //
    write_cmos_sensor(0x3806, 0x09); //
    write_cmos_sensor(0x3807, 0x01); //
    write_cmos_sensor(0x3808, 0x05); //
    write_cmos_sensor(0x3809, 0x00); //
    write_cmos_sensor(0x380a, 0x02); //
    write_cmos_sensor(0x380b, 0xD0); //
    //write_cmos_sensor(0x380c, 0x09); //
    //write_cmos_sensor(0x380d, 0x60); //
    //write_cmos_sensor(0x380e, 0x0c); //
    //write_cmos_sensor(0x380f, 0xfc); //
    write_cmos_sensor(0x380C, ((imgsensor_info.slim_video.linelength >> 8) & 0xFF)); // hts = 9600
    write_cmos_sensor(0x380D, (imgsensor_info.slim_video.linelength & 0xFF));        // hts
    write_cmos_sensor(0x380E, ((imgsensor_info.slim_video.framelength >> 8) & 0xFF));  // vts = 834
    write_cmos_sensor(0x380F, (imgsensor_info.slim_video.framelength & 0xFF));         // vts
    write_cmos_sensor(0x3810, 0x03); //
    write_cmos_sensor(0x3811, 0x44); //
    write_cmos_sensor(0x3813, 0x02); //
    write_cmos_sensor(0x3814, 0x31); //
    write_cmos_sensor(0x3815, 0x31); //
    write_cmos_sensor(0x3820, 0x00); //
    write_cmos_sensor(0x3821, 0x06); //
    write_cmos_sensor(0x3836, 0x08); //
    write_cmos_sensor(0x3837, 0x02); //
    write_cmos_sensor(0x4020, 0x00); //
    write_cmos_sensor(0x4021, 0xe4); //
    write_cmos_sensor(0x4022, 0x04); //
    write_cmos_sensor(0x4023, 0xd7); //
    write_cmos_sensor(0x4024, 0x05); //
    write_cmos_sensor(0x4025, 0xbc); //
    write_cmos_sensor(0x4026, 0x05); //
    write_cmos_sensor(0x4027, 0xbf); //
    write_cmos_sensor(0x4501, 0x78); // ;H digital skip for PD pixel; 3C for H digital bin
    write_cmos_sensor(0x4601, 0x04); //
    write_cmos_sensor(0x4603, 0x00); //
    write_cmos_sensor(0x4837, 0x19); //
    write_cmos_sensor(0x5401, 0x61); //
    write_cmos_sensor(0x5405, 0x40); //
    write_cmos_sensor(0x5b21, 0x37); //
    write_cmos_sensor(0x5b22, 0x0c); //
    write_cmos_sensor(0x5b23, 0x37); //

    /* Antaiui <AI_BSP_CAM> <xieht> <2022-02-24> use streaming control begin */
    // write_cmos_sensor(0x0100, 0x01);//
    // mdelay(10);
    /* Antaiui <AI_BSP_CAM> <xieht> <2022-02-24> use streaming control end */
}

static kal_uint32 return_sensor_id(void)
{
    return ((read_cmos_sensor(0x300A) << 8) | read_cmos_sensor(0x300B));
}

static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
    kal_uint8 i = 0;
    kal_uint8 retry = 2;
    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do {
            *sensor_id = return_sensor_id();
            if (*sensor_id == imgsensor_info.sensor_id) {
                LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
                return ERROR_NONE;
            }
            LOG_INF("Read sensor id fail:0x%x, id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
            retry--;
        } while(retry > 0);
        i++;
        retry = 1;
    }
    if (*sensor_id != imgsensor_info.sensor_id) {
        *sensor_id = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
    }

    return ERROR_NONE;
}

static kal_uint32 open(void)
{
    kal_uint8 i = 0;
    kal_uint8 retry = 1;
    kal_uint32 sensor_id = 0;
    LOG_INF("preview 2112*1568@30fps,1080Mbps/lane; video 1024*768@120fps,864Mbps/lane; capture 4224*3136@30fps,1080Mbps/lane\n");

    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do {
            sensor_id = return_sensor_id();
            if (sensor_id == imgsensor_info.sensor_id) {
                LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
                break;
            }
            LOG_INF("Read sensor id fail: 0x%x, id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
            retry--;
        } while(retry > 0);
        i++;
        if (sensor_id == imgsensor_info.sensor_id)
            break;
        retry = 2;
    }
    if (imgsensor_info.sensor_id != sensor_id)
        return ERROR_SENSOR_CONNECT_FAIL;

    if ((read_cmos_sensor(0x302a))==0xb1) {
        LOG_INF("----R1A---- \n");
        ov13853_chip_ver = OV13853_R1A;
    } else if((read_cmos_sensor(0x302a))==0xb2) {
        LOG_INF("----R2A---- \n");
        ov13853_chip_ver = OV13853_R2A;
    }

    sensor_init();

    spin_lock(&imgsensor_drv_lock);
    imgsensor.autoflicker_en= KAL_FALSE;
    imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
    imgsensor.shutter = 0x3D0;
    imgsensor.gain = 0x100;
    imgsensor.pclk = imgsensor_info.pre.pclk;
    imgsensor.frame_length = imgsensor_info.pre.framelength;
    imgsensor.line_length = imgsensor_info.pre.linelength;
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.dummy_pixel = 0;
    imgsensor.dummy_line = 0;
    imgsensor.ihdr_en = 0;
    imgsensor.test_pattern = KAL_FALSE;
    imgsensor.current_fps = imgsensor_info.pre.max_framerate;
    imgsensor.pdaf_mode= 0;

    capture_first_flag = 0;
    pre_currefps = 0;

    spin_unlock(&imgsensor_drv_lock);

    return ERROR_NONE;
}

static kal_uint32 close(void)
{
    LOG_INF("E\n");

    return ERROR_NONE;
}   /*  close  */

static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
        MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
    imgsensor.pclk = imgsensor_info.pre.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.pre.linelength;
    imgsensor.frame_length = imgsensor_info.pre.framelength;
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.current_fps = imgsensor.current_fps;
    //imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);

    preview_setting();
    set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}

static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
        MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
    if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {//PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M
        imgsensor.pclk = imgsensor_info.cap1.pclk;
        imgsensor.line_length = imgsensor_info.cap1.linelength;
        imgsensor.frame_length = imgsensor_info.cap1.framelength;
        imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
        //imgsensor.autoflicker_en = KAL_FALSE;
    } else {
        if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
            LOG_INF("Warning: current_fps %d fps is not support, so use cap1's setting: %d fps!\n",imgsensor.current_fps,imgsensor_info.cap1.max_framerate/10);
        imgsensor.pclk = imgsensor_info.cap.pclk;
        imgsensor.line_length = imgsensor_info.cap.linelength;
        imgsensor.frame_length = imgsensor_info.cap.framelength;
        imgsensor.min_frame_length = imgsensor_info.cap.framelength;
        //imgsensor.autoflicker_en = KAL_FALSE;
    }

    spin_unlock(&imgsensor_drv_lock);

    capture_setting(imgsensor.current_fps);
    set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}   /* capture() */

static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
        MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
    imgsensor.pclk = imgsensor_info.normal_video.pclk;
    imgsensor.line_length = imgsensor_info.normal_video.linelength;
    imgsensor.frame_length = imgsensor_info.normal_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
    //imgsensor.current_fps = 300;
    //imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);

    normal_video_setting(imgsensor.current_fps);
    set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
        MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
    imgsensor.pclk = imgsensor_info.hs_video.pclk;
    //imgsensor.video_mode = KAL_TRUE;
    imgsensor.line_length = imgsensor_info.hs_video.linelength;
    imgsensor.frame_length = imgsensor_info.hs_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
    //imgsensor.current_fps = 300;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);

    hs_video_setting();
    set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
        MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
    imgsensor.pclk = imgsensor_info.slim_video.pclk;
    //imgsensor.video_mode = KAL_TRUE;
    imgsensor.line_length = imgsensor_info.slim_video.linelength;
    imgsensor.frame_length = imgsensor_info.slim_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
    //imgsensor.current_fps = 300;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);

    slim_video_setting();
    set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}

static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
    LOG_INF("E\n");
    sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
    sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

    sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
    sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

    sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
    sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;


    sensor_resolution->SensorHighSpeedVideoWidth = imgsensor_info.hs_video.grabwindow_width;
    sensor_resolution->SensorHighSpeedVideoHeight = imgsensor_info.hs_video.grabwindow_height;

    sensor_resolution->SensorSlimVideoWidth = imgsensor_info.slim_video.grabwindow_width;
    sensor_resolution->SensorSlimVideoHeight = imgsensor_info.slim_video.grabwindow_height;

    return ERROR_NONE;
}   /*  get_resolution  */

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
        MSDK_SENSOR_INFO_STRUCT *sensor_info,
        MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    if(scenario_id == 0)
        LOG_INF("scenario_id = %d\n", scenario_id);

    sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
    sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; // inverse with datasheet
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

    sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;          /* The frame of setting shutter default 0 for TG int */
    sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;    /* The frame of setting sensor gain */
    sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
    sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
    sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
    sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;

    sensor_info->PDAF_Support = 1; /*0: NO PDAF, 1: PDAF Raw Data mode, 2:PDAF VC mode*/

    //sensor_info->HDR_Support = 0; /*0: NO HDR, 1: iHDR, 2:mvHDR, 3:zHDR*/
    sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
    sensor_info->SensorClockFreq = imgsensor_info.mclk;
    sensor_info->SensorClockDividCount = 3; /* not use */
    sensor_info->SensorClockRisingCount = 0;
    sensor_info->SensorClockFallingCount = 2; /* not use */
    sensor_info->SensorPixelClockCount = 3; /* not use */
    sensor_info->SensorDataLatchCount = 2; /* not use */

    sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
    sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
    sensor_info->SensorWidthSampling = 0;  // 0 is default 1x
    sensor_info->SensorHightSampling = 0;   // 0 is default 1x
    sensor_info->SensorPacketECCOrder = 1;

    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.cap.mipi_data_lp2hs_settle_dc;
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;
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
            capture_first_flag = 0;
            pre_currefps = 0;
            preview(image_window, sensor_config_data);
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            capture(image_window, sensor_config_data);
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            capture_first_flag = 0;
            pre_currefps = 0;
            normal_video(image_window, sensor_config_data);
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            capture_first_flag = 0;
            pre_currefps = 0;
            hs_video(image_window, sensor_config_data);
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            capture_first_flag = 0;
            pre_currefps = 0;
            slim_video(image_window, sensor_config_data);
            break;
        default:
            LOG_INF("Error ScenarioId setting");
            capture_first_flag = 0;
            pre_currefps = 0;
            preview(image_window, sensor_config_data);
            return ERROR_INVALID_SCENARIO_ID;
    }

    return ERROR_NONE;
}   /* control() */

static kal_uint32 set_video_mode(UINT16 framerate)
{
    LOG_INF("framerate = %d\n ", framerate);
    // SetVideoMode Function should fix framerate
    if (framerate == 0)
        // Dynamic frame rate
        return ERROR_NONE;

    spin_lock(&imgsensor_drv_lock);
    if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
        imgsensor.current_fps = 296;
    else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
        imgsensor.current_fps = 146;
    else
        imgsensor.current_fps = framerate;
    spin_unlock(&imgsensor_drv_lock);

    set_max_framerate(imgsensor.current_fps,1);

    return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
    LOG_INF("enable = %d, framerate = %d \n", enable, framerate);

    spin_lock(&imgsensor_drv_lock);
    if (enable) //enable auto flicker
        imgsensor.autoflicker_en = KAL_TRUE;
    else //Cancel Auto flick
        imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);

    return ERROR_NONE;
}

static kal_uint32 set_max_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
    //kal_int16 dummyLine;
    kal_uint32 frameHeight;

    LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

    if(framerate == 0)
        return ERROR_NONE;

    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            frameHeight = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
            LOG_INF("frameHeight = %d\n", frameHeight);
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frameHeight > imgsensor_info.pre.framelength)?(frameHeight - imgsensor_info.pre.framelength):0;
            imgsensor.frame_length =imgsensor_info.pre.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            if (imgsensor.frame_length > imgsensor.shutter)
                set_dummy();
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            if(framerate == 0)
                return ERROR_NONE;
            frameHeight = imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frameHeight > imgsensor_info.normal_video.framelength)?(frameHeight - imgsensor_info.normal_video.framelength):0;
            imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            if (imgsensor.frame_length > imgsensor.shutter)
                set_dummy();
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            frameHeight = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
            spin_lock(&imgsensor_drv_lock);

            imgsensor.dummy_line = (frameHeight > imgsensor_info.cap.framelength)?(frameHeight - imgsensor_info.cap.framelength):0;
            imgsensor.frame_length =imgsensor_info.cap.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            if (imgsensor.frame_length > imgsensor.shutter)
                set_dummy();
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            frameHeight = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frameHeight > imgsensor_info.hs_video.framelength)?(frameHeight - imgsensor_info.hs_video.framelength):0;
            imgsensor.frame_length =imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            if (imgsensor.frame_length > imgsensor.shutter)
                set_dummy();
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            frameHeight = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frameHeight > imgsensor_info.slim_video.framelength)?(frameHeight - imgsensor_info.slim_video.framelength):0;
            imgsensor.frame_length =imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            if (imgsensor.frame_length > imgsensor.shutter)
                set_dummy();
            break;
        default:  //coding with  preview scenario by default
            frameHeight = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frameHeight > imgsensor_info.pre.framelength)?(frameHeight - imgsensor_info.pre.framelength):0;
            imgsensor.frame_length =imgsensor_info.pre.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);

            if (imgsensor.frame_length > imgsensor.shutter)
                set_dummy();
            LOG_INF("error scenario_id = %d, we use preview scenario \n", scenario_id);
            break;
    }
    return ERROR_NONE;
}

static kal_uint32 get_default_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
    if(scenario_id == 0)
        LOG_INF("[3058]scenario_id = %d\n", scenario_id);

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

/* Antaiui <AI_BSP_CAM> <xieht> <2022-02-24> streaming control begin */
static void check_output_stream_off(void) {
    int timeout = (10000 / imgsensor.current_fps) + 1;
    int i = 0;
    int framecnt = 0;
    for (i = 0; i < timeout; i++) {
        mdelay(5);
        framecnt = read_cmos_sensor(0x0005);
        if (framecnt == 0x00) {
            LOG_INF("Stream off OK at i = %d.\n", i);
            return;
        }
    }
    LOG_INF("[xieht] Not YET! framecnt: %d", framecnt);
}

static kal_uint32 streaming_control(kal_bool enable)
{
    LOG_INF("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);
    if (enable) {
        write_cmos_sensor(0x0100, 0x01);
        mdelay(5);
    } else {
        write_cmos_sensor(0x0100, 0x00);
        check_output_stream_off();
    }
    return ERROR_NONE;
}
/* Antaiui <AI_BSP_CAM> <xieht> <2022-02-24> streaming control end */

// Antaiui <AI_BSP_CAM> <xieht> <2022-03-23> test_pattern_mode modify begin
static kal_uint32 set_test_pattern_mode(UINT16 mode)
{
    LOG_INF("mode: %d\n", mode);

    if (mode) {
        // 0x5E00[8]: 1 enable,  0 disable
        // 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
        if (mode == 5)
            write_cmos_sensor(0x5E00, 0x83);
        else
            write_cmos_sensor(0x5E00, 0x80);
    } else {
        // 0x5E00[8]: 1 enable,  0 disable
        // 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
        write_cmos_sensor(0x5E00, 0x00);
    }
    spin_lock(&imgsensor_drv_lock);
    imgsensor.test_pattern = (BOOL)mode;
    spin_unlock(&imgsensor_drv_lock);
    return ERROR_NONE;
}
// Antaiui <AI_BSP_CAM> <xieht> <2022-03-23> test_pattern_mode modify end


static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
        UINT8 *feature_para,UINT32 *feature_para_len)
{
    UINT16 *feature_return_para_16=(UINT16 *) feature_para;
    UINT16 *feature_data_16=(UINT16 *) feature_para;
    UINT32 *feature_return_para_32=(UINT32 *) feature_para;
    UINT32 *feature_data_32=(UINT32 *) feature_para;
    unsigned long long *feature_data=(unsigned long long *) feature_para;
    //unsigned long long *feature_return_para=(unsigned long long *) feature_para;
    struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
    MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data=(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;
    struct SET_PD_BLOCK_INFO_T *PDAFinfo;

    if(!((feature_id == 3040) || (feature_id == 3058)))
        LOG_INF("feature_id = %d\n", feature_id);

    switch (feature_id) {
        case SENSOR_FEATURE_GET_PERIOD:
            *feature_return_para_16++ = imgsensor.line_length;
            *feature_return_para_16 = imgsensor.frame_length;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
            *feature_return_para_32 = imgsensor.pclk;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_ESHUTTER:
            set_shutter(*feature_data);
            break;
        case SENSOR_FEATURE_SET_NIGHTMODE:
            night_mode((BOOL) *feature_data);
            break;
        case SENSOR_FEATURE_SET_GAIN:
            set_gain((UINT16) *feature_data);
            break;
        case SENSOR_FEATURE_SET_FLASHLIGHT:
            break;
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
            break;
        case SENSOR_FEATURE_SET_REGISTER:
            write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
            break;
        case SENSOR_FEATURE_GET_REGISTER:
            sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
            break;
        case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
            // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
            // if EEPROM does not exist in camera module.
            *feature_return_para_32=LENS_DRIVER_ID_DO_NOT_CARE;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_VIDEO_MODE:
            set_video_mode(*feature_data);
            break;
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            get_imgsensor_id(feature_return_para_32);
            break;
        case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
            set_auto_flicker_mode((BOOL)*feature_data_16,*(feature_data_16+1));
            break;
        case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
            set_max_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
            break;
        case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
            get_default_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*(feature_data), (MUINT32 *)(uintptr_t)(*(feature_data+1)));
            //get_default_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*feature_data_32, (MUINT32 *)(*(feature_data_32+1)));
            break;
        case SENSOR_FEATURE_GET_PDAF_DATA:
            LOG_INF("SENSOR_FEATURE_GET_PDAF_DATA\n");
            break;
        // Antaiui <AI_BSP_CAM> <xieht> <2022-03-23> test_pattern_mode modify begin
        case SENSOR_FEATURE_SET_TEST_PATTERN:
            LOG_INF("[xieht] SET_TEST_PATTERN_MODE: %llu", *feature_data);
            set_test_pattern_mode((UINT16)*feature_data);
            break;
        // Antaiui <AI_BSP_CAM> <xieht> <2022-03-23> test_pattern_mode modify end
        case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: //for factory mode auto testing
            *feature_return_para_32 = imgsensor_info.checksum_value;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_FRAMERATE:
            spin_lock(&imgsensor_drv_lock);
            imgsensor.current_fps = *feature_data_32;
            spin_unlock(&imgsensor_drv_lock);
            LOG_INF("current fps :%d\n", imgsensor.current_fps);
            break;
        case SENSOR_FEATURE_GET_CROP_INFO:
            LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", *feature_data_32);
            wininfo = (struct SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));
            switch (*feature_data_32) {
                case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[1],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[2],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[3],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_SLIM_VIDEO:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[4],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
                default:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[0],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
                    break;
            }
            break;
        case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
            LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",(UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
            ihdr_write_shutter_gain((UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
            break;
        case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
            {
                kal_uint32 rate;
                LOG_INF("pangfei SENSOR_FEATURE_GET_MIPI_PIXEL_RATE\n");
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
                    default:
                        rate = imgsensor_info.pre.mipi_pixel_rate;
                        break;
                }
                *(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = rate;
                LOG_INF("pangfei SENSOR_FEATURE_GET_MIPI_PIXEL_RATE rate %d\n", rate);
            }
            break;
        case SENSOR_FEATURE_GET_PDAF_INFO:
            LOG_INF("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%lld\n", *feature_data);
            PDAFinfo= (struct SET_PD_BLOCK_INFO_T *)
                (uintptr_t)(*(feature_data+1));

            switch (*feature_data) {
                case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                    memcpy((void *)PDAFinfo,(void *)&imgsensor_pd_info,
                            sizeof(struct SET_PD_BLOCK_INFO_T));
                    break;
                case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
                case MSDK_SCENARIO_ID_SLIM_VIDEO:
                case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
                default:
                    break;
            }
            break;
        case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
            LOG_INF("SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%lld\n", *feature_data);
            //PDAF capacity enable or not, 2p8 only full size support PDAF
            switch (*feature_data) {
                case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                    *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
                    break;
                case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                    *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;// xxx is not supported
                    break;
                case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
                    *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
                    break;
                case MSDK_SCENARIO_ID_SLIM_VIDEO:
                    *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
                    break;
                case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
                    *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
                    break;
                default:
                    *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
                    break;
            }
            break;
        case SENSOR_FEATURE_SET_PDAF:
            LOG_INF("PDAF mode :%d\n", imgsensor.pdaf_mode);
            break;
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
        /* Antaiui <AI_BSP_CAM> <xieht> <2022-02-24> streaming control begin */
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
        /* Antaiui <AI_BSP_CAM> <xieht> <2022-02-24> streaming control end */

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

UINT32 OV13853_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc!=NULL)
        *pfFunc=&sensor_func;
    return ERROR_NONE;
}
