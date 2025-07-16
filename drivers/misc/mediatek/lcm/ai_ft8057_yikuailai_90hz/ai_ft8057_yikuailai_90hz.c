#ifdef BUILD_LK
#include <stdio.h>
#include <string.h>
#else
#include <linux/string.h>
#include <linux/kernel.h>
#endif

#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <platform/mt_pmic.h>
#include "ddp_hal.h"

#include <platform/upmu_common.h>
#include <platform/mt_i2c.h>
#include <string.h>
#include <lcm_pmic.h>
#else
//#include <mach/mt_gpio.h>
//#include <mach/mt_pm_ldo.h>
#endif

#include "lcm_drv.h"
#include "lcm_i2c.h"

#ifdef BUILD_LK

#elif defined(BUILD_UBOOT)

#else
//#include <mt-plat/mt_gpio.h>
#include <mt-plat/upmu_common.h>
//#include <mach/gpio_const.h>

#endif


// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH     (720)
#define FRAME_HEIGHT (1612)
#ifndef BUILD_LK
#define LCM_DENSITY                     (480)
#endif
//Antaiui <AI_BSP_LCM> <chenht> <2022-09-15> modify lcm size begin
#define PHYSICAL_WIDTH  (68330)  
#define PHYSICAL_HEIGHT (152490)
//Antaiui <AI_BSP_LCM> <chenht> <2022-09-15> modify lcm size end
#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifdef BUILD_LK
#define LCM_PRINT printf
#else
#define LCM_PRINT printk
#endif

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static struct LCM_UTIL_FUNCS lcm_util;

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))



// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define SET_RESET_PIN(v)        (lcm_util.set_reset_pin((v)))

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)									lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)				lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)  
#define set_gpio_lcd_enp(cmd)                                   lcm_util.set_gpio_lcd_enp_bias(cmd)
#define set_gpio_lcd_enn(cmd)                                   lcm_util.set_gpio_lcd_enn_bias(cmd)
#define REGFLAG_DELAY                                                                   0XFFE
#define REGFLAG_END_OF_TABLE                                                            0xFFF   // END OF REGISTERS MARKER
#define set_gpio_tp_incell_rst(cmd)                                   lcm_util.set_gpio_tp_incell_rst(cmd)
extern int PMU_db_pos_neg_setting_delay(int ms);
extern int PMU_db_pos_neg_disable_delay(int ms);

struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};
static struct LCM_setting_table lcm_initialization_setting[] = {
{0x00,1,{0x00}},
{0xFF,3,{0x80,0x57,0x01}}, 

{0x00,1,{0x80}},
{0xFF,2,{0x80,0x57}},

//--------------Panel Resolution --------------//
{0x00,1,{0x00}},
{0x2A,4,{0x00,0x00,0x02,0xCF}}, //720

{0x00,1,{0x00}},
{0x2B,4,{0x00,0x00,0x06,0x4B}}, //1612

{0x00,1,{0xA3}},  //Y=1612
{0xB3,4,{0x06,0x4C,0x00,0x18}},

//--------------Voltage set--------------//
{0x00,1,{0x93}},//VGH_N 18V
{0xC5,1,{0x75}},

{0x00,1,{0x97}},//VGH_I 18V
{0xC5,1,{0x75}},

{0x00,1,{0x9A}},//VGL_N -12V 
{0xC5,1,{0x41}},

{0x00,1,{0x9C}},//VGL_I -12V 
{0xC5,1,{0x41}},

{0x00,1,{0xB6}}, //VGHO1_N_I=17v
{0xC5,2,{0x61,0x61}},

{0x00,1,{0xB8}}, //VGLO1_N_I -11V
{0xC5,2,{0x37,0x37}}, 

{0x00,1,{0x00}}, 
{0xD8,2,{0x2F,0x2F}},  //GVDDP/N 5.2V  -5.2V

{0x00,1,{0x00}},
{0xD9,1,{0x00}},

//{0x00,1,{0x07}}, 
//{0xD9,3,{0x00,0xA5,0xA5}},  //VCOM{-1.5V} 

{0x00,1,{0x82}}, 
{0xC5,1,{0x55}},  //LVD

{0x00,1,{0x83}}, 
{0xC5,1,{0x07}},  //LVD Enable

//Vgh_s_sel>>vgh 眏
{0x00,1,{0x96}},
{0xf5,1,{0x0d}},
 
//Vgl_s_sel>>vgl 眏
{0x00,1,{0x86}},
{0xf5,1,{0x0d}},
 
// VGH CLK Line Rate{1 Line}
{0x00,1,{0x94}},
{0xC5,1,{0x15}},
 
// VGL CLK Line Rate{1 Line}
{0x00,1,{0x9B}},
{0xC5,1,{0x51}},

{0x00,1,{0xA3}},  //GVDD_EN
{0xA5,1,{0x04}}, 

{0x00,1,{0x99}},  
{0xCF,1,{0x56}}, 

{0x00,1,{0x90}}, //MIPI 4lane 
{0xFF,1,{0x00}}, 

{0x00,1,{0x86}},  
{0xB7,1,{0x80}}, //I2C EN

{0x00,1,{0xA5}},  
{0xB0,1,{0x1D}}, //


//--------------Gamma setting 2.2--------------//
{0x00,1,{0x00}},
{0xE1,16,{0x0D,0x10,0x1A,0x25,0x2C,0x35,0x43,0x50,0x50,0x5D,0x60,0x75,0x8B,0x76,0x74,0x67}},
{0x00,1,{0x10}},
{0xE1,8,{0x5D,0x51,0x40,0x36,0x2D,0x1E,0x10,0x0F}},

{0x00,1,{0x00}},
{0xE2,16,{0x0D,0x10,0x1A,0x25,0x2C,0x35,0x43,0x50,0x50,0x5D,0x60,0x75,0x8B,0x76,0x74,0x67}},
{0x00,1,{0x10}},
{0xE2,8,{0x5D,0x51,0x40,0x36,0x2D,0x1E,0x10,0x0F}},



//--------------TCON setting--------------//
//TCON
{0x00,1,{0x80}},
{0xC0,6,{0x00,0xD2,0x00,0x2E,0x00,0x1C}},

{0x00,1,{0x90}},
{0xC0,6,{0x00,0x7F,0x00,0x2E,0x00,0x1C}},

{0x00,1,{0xA0}},
{0xC0,6,{0x00,0xD2,0x00,0x2E,0x00,0x1C}},

{0x00,1,{0xB0}},
{0xC0,5,{0x01,0x0F,0x00,0x2E,0x1C}},

{0x00,1,{0xC1}},
{0xC0,8,{0x01,0x33,0x01,0x0A,0x00,0xCD,0x01,0x8F}},

{0x00,1,{0x70}},
{0xC0,6,{0x00,0x7F,0x00,0x2E,0x00,0x1C}},

{0x00,1,{0xA3}},
{0xC1,6,{0x00, 0x33, 0x00,0x3C,0x00,0x02}},

{0x00,1,{0xB7}},
{0xC1,2,{0x00,0x33}},

{0x00,1,{0x73}},
{0xCE,2,{0x09,0x09}},

{0x00,1,{0x80}},
{0xCE,16,{0x01, 0x81,0x09,0x09,0x00,0x78,0x00,0x96,0x00,0x78,0x00,0x96,0x00,0x78,0x00,0x96}},

{0x00,1,{0x90}},
{0xCE,15,{0x00,0xA5,0x16,0x8F,0x00,0xA5,0x80,0x09,0x09,0x00,0x07,0xD0,0x16,0x16,0x17}},

{0x00,1,{0xA0}},
{0xCE,3,{0x20,0x00,0x00}},

{0x00,1,{0xB0}},
{0xCE,3,{0x87,0x00,0x00}},

{0x00,1,{0xD1}},
{0xCE,7,{0x00,0x00,0x01,0x00,0x00,0x00,0x00}},

{0x00,1,{0xE1}},
{0xCE,11,{0x08,0x03,0xC3,0x03,0xC3,0x02,0xB0,0x00,0x00,0x00,0x00}},

{0x00,1,{0xF1}},
{0xCE,9,{0x14,0x14,0x1E,0x01,0x52,0x01,0x52,0x01,0x53}},

{0x00,1,{0xB0}},
{0xCF,4,{0x00,0x00,0x6D,0x71}},

{0x00,1,{0xB5}},
{0xCF,4,{0x03,0x03,0x5B,0x5F}},

{0x00,1,{0xC0}},
{0xCF,4,{0x06,0x06,0x47,0x4B}},

{0x00,1,{0xC5}},
{0xCF,4,{0x06,0x06,0x4B,0x4F}},

{0x00,1,{0x60}},
{0xCF,8,{0x00,0x00,0x6D,0x71,0x03,0x03,0x5B,0x5F}},

{0x00,1,{0x70}},
{0xCF,8,{0x00,0x00,0x65,0x69,0x03,0x03,0x53,0x57}},

{0x00,1,{0xAA}},
{0xCF,4,{0x80,0x80,0x1C,0x18}},

//Qsync Detect
{0x00,1,{0xD1}},
{0xC1,12,{0x03,0xAA,0x05,0x22,0x09,0x59,0x05,0x87,0x08,0x23,0x0F,0xAC}},

{0x00,1,{0xE1}},
{0xC1,2,{0x05,0x22}},

{0x00,1,{0xE2}},
{0xCF,12,{0x06,0xDE,0x06,0xDD,0x06,0xDD,0x06,0xDD,0x06,0xDD,0x06,0xDD}},

//OSC
{0x00,1,{0x80}},
{0xC1,2,{0x00,0x00}},

{0x00,1,{0x90}},
{0xC1,1,{0x01}},

//Line rate for TP
{0x00,1,{0xF5}},
{0xCF,1,{0x01}},

//TP Frame rate
{0x00,1,{0xF6}},
{0xCF,1,{0x5A}},

//TCON Frame rate
{0x00,1,{0xF1}},
{0xCF,1,{0x5A}},

//Mode switch by CMD2
{0x00,1,{0xF7}},
{0xCF,1,{0x71}},

//switch 90Hz
{0x00,1,{0x00}},
{0x1F,1,{0x5A,0x5A}},	//90Hz


//VB Mode 1.2ms
{0x00,1,{0xD1}},
{0xCE,7,{0x00,0x0A,0x01,0x01,0x00,0xCA,0x01}},

{0x00,1,{0xE8}},
{0xCE,4,{0x00,0xCA,0x00,0xCA}},

//--------------CGOUT setting--------------//
//U2D mapping
//CGOUTL
{0x00,1,{0x80}},
{0xCC,16,{0x00,0x07,0x09,0x0B,0x0D,0x0F,0x11,0x00,0x03,0x05,0x23,0x01,0x1D,0x26,0x1C,0x00}},

{0x00,1,{0x90}},
{0xCC,8,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

//CGOUTR
{0x00,1,{0x80}},
{0xCD,16,{0x00,0x06,0x08,0x0A,0x0C,0x0E,0x10,0x00,0x02,0x04,0x22,0x01,0x1D,0x26,0x1C,0x00}},

{0x00,1,{0x90}},
{0xCD,8,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

//D2U mapping
//CGOUTL
{0x00,1,{0xA0}},
{0xCC,16,{0x00,0x08,0x06,0x10,0x0E,0x0C,0x0A,0x00,0x04,0x02,0x22,0x01,0x1D,0x1C,0x26,0x00}},

{0x00,1,{0xB0}},
{0xCC,8,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

//CGOUTR
{0x00,1,{0xA0}},
{0xCD,16,{0x00,0x09,0x07,0x11,0x0F,0x0D,0x0B,0x00,0x05,0x03,0x23,0x01,0x1D,0x1C,0x26,0x00}},

{0x00,1,{0xB0}},
{0xCD,8,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

//--------------GIP setting--------------//
//power off enmode setting
{0x00,1,{0x80}},
{0xCB,16,{0xC1,0xC1,0xC1,0xC1,0xC1,0xC1,0xC1,0xC1,0xC1,0xC1,0xC1,0xC1,0xC1,0xC1,0xC1,0xC1}},

{0x00,1,{0xED}},
{0xCB,1,{0xC1}},

//power on enmode setting
{0x00,1,{0x90}},
{0xCB,16,{0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{0x00,1,{0xEE}},
{0xCB,1,{0x00}},

{0x00,1,{0x90}},
{0xC3,1,{0x00}},

//skip & powr on1 enmode setting
{0x00,1,{0xA0}},
{0xCB,8,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

//power off blank enmode setting
{0x00,1,{0xB0}},
{0xCB,4,{0x55,0x55,0x55,0x55}},
//power on blank enmode setting
{0x00,1,{0xC0}},
{0xCB,4,{0x55,0x55,0x55,0x55}},

//power on blank enmode setting
{0x00,1,{0xD2}},
{0xCB,11,{0x83,0x00,0x83,0x83,0x00,0x83,0x83,0x00,0x83,0x83,0x00}},

{0x00,1,{0xE0}},
{0xCB,13,{0x83,0x83,0x00,0x83,0x83,0x00,0x83,0x83,0x00,0x83,0x83,0x00,0x83}},

{0x00,1,{0xFA}},
{0xCB,2,{0x83,0x00}},

{0x00,1,{0xEF}},
{0xCB,1,{0x00}},

//STV1 
{0x00,1,{0x68}},
{0xC2,4,{0x90,0x05,0x00,0x00}},
//STV2
{0x00,1,{0x6C}},
{0xC2,4,{0x8F,0x05,0x00,0x00}},
//STV3
{0x00,1,{0x70}},
{0xC2,4,{0x8E,0x05,0x00,0x00}},
//STV4
{0x00,1,{0x74}},
{0xC2,4,{0x8D,0x05,0x00,0x00}},

//RST1
{0x00,1,{0xEA}},
{0xC2,8,{0x12,0x00,0x11,0x08,0x00,0x00,0x00,0x00}},

//CKV1
{0x00,1,{0x8C}},
{0xC2,5,{0x8A,0x04,0x15,0x9A,0x9F}},
//CKV2
{0x00,1,{0x91}},
{0xC2,5,{0x89,0x05,0x15,0x9A,0x9F}},
//CKV3
{0x00,1,{0x96}},
{0xC2,5,{0x88,0x06,0x15,0x9A,0x9F}},
//CKV4
{0x00,1,{0x9B}},
{0xC2,5,{0x87,0x07,0x15,0x9A,0x9F}},

//CKV5
{0x00,1,{0xA0}},
{0xC2,5,{0x86,0x00,0x15,0x9A,0x9F}},
//CKV6
{0x00,1,{0xA5}},
{0xC2,5,{0x85,0x00,0x15,0x9A,0x9F}},
//CKV7
{0x00,1,{0xAA}},
{0xC2,5,{0x84,0x00,0x15,0x9A,0x9F}},
//CKV8
{0x00,1,{0xAF}},
{0xC2,5,{0x83,0x00,0x15,0x9A,0x9F}},

//CKV9
{0x00,1,{0xB4}},
{0xC2,5,{0x82,0x01,0x15,0x9A,0x9F}},
//CKV10
{0x00,1,{0xB9}},
{0xC2,5,{0x81,0x02,0x15,0x9A,0x9F}},
//CKV11
{0x00,1,{0xBE}},
{0xC2,5,{0x80,0x03,0x15,0x9A,0x9F}},
//CKV12
{0x00,1,{0xC3}},
{0xC2,5,{0x01,0x04,0x15,0x9A,0x9F}},

//CKV width setting
{0x00,1,{0xdc}},
{0xC2,8,{0xbb,0xbb,0xbb,0xbb,0xbb,0xbb,0x00,0x00}},

//GOFF3 
{0x00,1,{0xE8}},
{0xC3,4,{0x00,0x00,0x00,0x00}},

//GOFF4
{0x00,1,{0xEC}},
{0xC3,4,{0x00,0x00,0x00,0x00}},

{0x00,1,{0xF9}}, //GOFF3 add VST4
{0xCB,1,{0x20}},

{0x00,1,{0xFE}}, //GOFF4 add VST4
{0xCB,1,{0x08}},

{0x00,1,{0xF8}}, //GOFF3/GOFF4 add VST4 shift & width
{0xCD,3,{0x8C,0x55,0x8B}},

{0x00,1,{0xFC}}, //GCH/GCL
{0xCB,2,{0x08,0x40}},

{0x00,1,{0xFB}}, //GCH/GCL start&end point
{0xC3,4,{0x93,0x08,0x93,0x08}},

//--------------Source setting--------------//
{0x00,1,{0x98}},
{0xC4,1,{0x08}},

{0x00,1,{0x91}},
{0xE9,4,{0xFF,0xFF,0xFF,0x00}},

{0x00,1,{0x85}},//Posdmy pch
{0xC4,1,{0x80}},

{0x00,1,{0x86}},//4VB pch
{0xA4,1,{0xB6}},

{0x00,1,{0x95}},//VB pch data
{0xC4,1,{0x80}},

//--------------Power on&off--------------//
{0x00,1,{0xCA}},//Power on 3 
{0xC0,2,{0x90,0x11}},

{0x00,1,{0xB7}},//sd_en_sdpl_sel
{0xF5,1,{0x1D}},

{0x00,1,{0x90}},
{0xC3,1,{0x00}},

{0x00,1,{0xB1}},//VCOM
{0xF5,1,{0x11}},

//--------------sleep in allgateon--------------//
{0x00,1,{0xB0}}, 
{0xC5,1,{0x00}},

{0x00,1,{0xB3}},
{0xC5,1,{0x00}},

{0x00,1,{0xB2}},
{0xC5,1,{0x0D}},

{0x00,1,{0xB5}},
{0xC5,1,{0x02}},

{0x00,1,{0xC2}},
{0xF5,1,{0x42}},
//*********************************************//
//*********************************************//
//LongV mode
{0x00,1,{0x80}},
{0xCE,1,{0x00}},

{0x00,1,{0xD0}},
{0xCE,1,{0x01}},

{0x00,1,{0xE0}},
{0xCE,1,{0x00}},

{0x00,1,{0xA1}},
{0xC1,1,{0xCC}},

{0x00,1,{0xA6}},
{0xC1,1,{0x10}},

{0x00,1,{0x71}},//VFP=217-1
{0xC0,5,{0xA3,0x00,0xD8,0x00,0x22}},

//*********************************************//
//*********************************************//
//--------------CMD WR Disable--------------//
{0x00,1,{0x00}},
{0xFF,3,{0x00,0x00,0x00}}, 

{0x00,1,{0x80}},
{0xFF,2,{0x00,0x00}},

{0x35,1,{0x00}},

{0x11,1,{0x00}},
{REGFLAG_DELAY,120, {}},
{0x29,1,{0x00}},
{REGFLAG_DELAY,20, {}},
    {REGFLAG_END_OF_TABLE,0x00,{}}
};

//Antai <AI_BSP_LCD> <chenht> <2023-01-09> add for lcm leakage current begin
static struct LCM_setting_table lcm_deepstandby_setting[] = {
{0xA5,1,{0x03}}, 

{0x28,1,{0x00}},
{REGFLAG_DELAY,120, {}},
{0x10,1,{0x00}},
{REGFLAG_DELAY,50, {}},

{0x00,1,{0x00}},
{0xFF,3,{0x80,0x57,0x01}},

{0x00,1,{0x80}},
{0xFF,2,{0x80,0x57}},

{0x00,1,{0x00}},
{0xF7,4,{0x5A,0xA5,0x95,0x27}},
{REGFLAG_DELAY,150, {}},
};
//Antai <AI_BSP_LCD> <chenht> <2023-01-09> add for lcm leakage current end

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;

    for(i = 0; i < count; i++) {

        unsigned cmd;
        cmd = table[i].cmd;

        switch (cmd) {

            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;

            case REGFLAG_END_OF_TABLE :
                break;

            default:
                dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
        }
    }
}

static void lcm_set_util_funcs(const struct LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(struct LCM_UTIL_FUNCS));
}

//Antai <AI_BSP_LCD> <chenht> <2022-10-19> add for lcm frame rate switching begin
#ifdef CONFIG_MTK_HIGH_FRAME_RATE
/*DynFPS*/
static void lcm_dfps_int(struct LCM_DSI_PARAMS *dsi)
{
	struct dfps_info *dfps_params = dsi->dfps_params;

	dsi->dfps_enable = 1;
	dsi->dfps_default_fps = 6000;/*real fps * 100, to support float*/
	dsi->dfps_def_vact_tim_fps = 9000;/*real vact timing fps * 100*/

	/*traversing array must less than DFPS_LEVELS*/
	/*DPFS_LEVEL0*/
	dfps_params[0].level = DFPS_LEVEL0;
	dfps_params[0].fps = 6000;/*real fps * 100, to support float*/
	dfps_params[0].vact_timing_fps = 9000;/*real vact timing fps * 100*/
	/*if mipi clock solution*/
	/*dfps_params[0].PLL_CLOCK = xx;*/
	/*dfps_params[0].data_rate = xx; */
	/*if HFP solution*/
	/*dfps_params[0].horizontal_frontporch = xx;*/
	dfps_params[0].vertical_frontporch = 1144; 
	//dfps_params[1].vertical_frontporch_for_low_power = 540;

	/*if need mipi hopping params add here*/
	/*dfps_params[0].PLL_CLOCK_dyn =xx;
	 *dfps_params[0].horizontal_frontporch_dyn =xx ;
	 * dfps_params[0].vertical_frontporch_dyn = 1291;
	 */

	/*DPFS_LEVEL1*/
	dfps_params[1].level = DFPS_LEVEL1;
	dfps_params[1].fps = 9000;/*real fps * 100, to support float*/
	dfps_params[1].vact_timing_fps = 9000;/*real vact timing fps * 100*/
	/*if mipi clock solution*/
	/*dfps_params[1].PLL_CLOCK = xx;*/
	/*dfps_params[1].data_rate = xx; */
	/*if HFP solution*/
	/*dfps_params[1].horizontal_frontporch = xx;*/
	dfps_params[1].vertical_frontporch = 217; //371
	//dfps_params[0].vertical_frontporch_for_low_power = 540;

	/*if need mipi hopping params add here*/
	/*dfps_params[1].PLL_CLOCK_dyn =xx;
	 *dfps_params[1].horizontal_frontporch_dyn =xx ;
	 * dfps_params[1].vertical_frontporch_dyn= 54;
	 * dfps_params[1].vertical_frontporch_for_low_power_dyn =xx;
	 */

	dsi->dfps_num = 2;
}
#endif
//Antai <AI_BSP_LCD> <chenht> <2022-10-19> add for lcm frame rate switching end

static void lcm_get_params(struct LCM_PARAMS *params)
{
	memset(params, 0, sizeof(struct LCM_PARAMS));

	params->type = LCM_TYPE_DSI;
	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;	
	params->physical_width = PHYSICAL_WIDTH/1000;
	params->physical_height = PHYSICAL_HEIGHT/1000;

	params->dsi.mode = SYNC_PULSE_VDO_MODE;
	params->dsi.LANE_NUM = LCM_FOUR_LANE;
	
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;	
	params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;	
	params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;	
	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active = 6;
	params->dsi.vertical_backporch = 28;
	//Antai <AI_BSP_LCD> <chenht> <2022-10-19> modify for lcm frame rate switching begin
	params->dsi.vertical_frontporch = 1144; // PLL 422:vfp 234  416:210 //90hz:264  60hz:1230
	//Antai <AI_BSP_LCD> <chenht> <2022-10-19> modify for lcm frame rate switching end
 	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 16;
	params->dsi.horizontal_backporch = 16;
	params->dsi.horizontal_frontporch = 12;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;
	//Antai <AI_BSP_LCD> <penggy> <2021-07-07> add for lcm esd begin	

	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 1;
	params->dsi.lcm_esd_check_table[0].cmd          = 0x0A;
	params->dsi.lcm_esd_check_table[0].count        = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
	//Antai <AI_BSP_LCD> <penggy> <2021-07-07> add for lcm esd end	

	params->dsi.ssc_disable = 1;
	params->dsi.ssc_range = 4;
	params->dsi.cont_clock = 1;
	//	params->dsi.clk_lp_per_line_enable=1;
	params->dsi.PLL_CLOCK = 418;	//432   /* this value must be in MTK suggested table 1378*/

	//Antai <AI_BSP_LCD> <chenht> <2022-10-19> add for lcm frame rate switching begin
	#ifdef CONFIG_MTK_HIGH_FRAME_RATE
	/****DynFPS start****/
	lcm_dfps_int(&(params->dsi));
	/****DynFPS end****/
	#endif
	//Antai <AI_BSP_LCD> <chenht> <2022-10-19> add for lcm frame rate switching end
}


static void lcm_init_power(void)
{
	LCM_PRINT("yj LCM --- %s \n",__func__);
	//MDELAY(5);
}

static void lcm_suspend_power(void)
{
	LCM_PRINT("yj LCM --- %s \n",__func__);
}

static void lcm_resume_power(void)
{
	LCM_PRINT("yj LCM --- %s \n",__func__);
	lcm_init_power();
}

//Antai <AI_BSP_LCD> <hehl> <20201204> add for lcm id ata start	
#if defined(CONFIG_AI_BSP_LCM_ID_ATA)
static int lcm_id_ata=-1;
unsigned int lcm_id_ata_read(void){
	return lcm_id_ata ;
}

static unsigned int ata_lcm_compare_id(void)
{
	unsigned int id1 = 0;
	unsigned int id2 = 0;
	unsigned int id3 = 0;
	unsigned char buffer[3];
	unsigned int array[16];	

	struct LCM_setting_table switch_table_page1[] = {
		{0x00,1,{0x00}},
		{0xFF,3,{0x87,0x56,0x01}},
		{0x00,1,{0x80}},
		{0xFF,2,{0x87,0x56}},
	};
	
	push_table(switch_table_page1, sizeof(switch_table_page1) / sizeof(struct LCM_setting_table), 1);
	array[0] = 0x00023700;
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0xA1, &buffer[0], 3);
	id1 = buffer[0];
	id2 = buffer[1];
	id3 = buffer[2];
	LCM_PRINT("lcm hx8394f id1=0x%x id2=0x%x id3=0x%x\n",id1,id2,id3);
	if((id1 == 0x01) && (id2 == 0x8B) && (id3 == 0x87))
	{
		LCM_PRINT("lcm read hx8394 id success\n");
		return 1;
	}
	else
	{
		LCM_PRINT("lcm read hx8394 id fail\n");
		return 0;
	}	
}
#endif
//Antai <AI_BSP_LCD> <hehl> <20201204> add for lcm id ata end	
static void lcm_init(void)
{

	MDELAY(5);
	PMU_db_pos_neg_setting_delay(5);
	
	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(120);

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
//Antai <AI_BSP_LCD> <hehl> <20201204> add for lcm id ata start	
#if defined(CONFIG_AI_BSP_LCM_ID_ATA)
	lcm_id_ata=ata_lcm_compare_id();
	LCM_PRINT("lcm hx8394f lcm_id_ata=%d\n",lcm_id_ata);
#endif
//Antai <AI_BSP_LCD> <hehl> <20201204> add for lcm id ata end		
}

//Antai <AI_BSP_LCD> <chenht> <2022-11-25> modify for 2206 gesture begin
extern int is_gesture_mode_fts(void);
//Antai <AI_BSP_LCD> <chenht> <2022-11-25> modify for 2206 gesture end

static void lcm_suspend(void)
{
	//Antai <AI_BSP_LCD> <chenht> <2023-01-09> modify for lcm leakage current begin
	//unsigned int data_array[16];
	//Antai <AI_BSP_LCD> <chenht> <2023-01-09> modify for lcm leakage current end

	LCM_PRINT("yj LCM --- %s \n",__func__);
	
	//Antai <AI_BSP_LCD> <chenht> <2023-01-09> modify for lcm leakage current begin
	push_table(lcm_deepstandby_setting, sizeof(lcm_deepstandby_setting) / sizeof(struct LCM_setting_table), 1);
	/*
	data_array[0] = 0x00280500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(30);

	data_array[0] = 0x00100500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(100);
	*/
	//Antai <AI_BSP_LCD> <chenht> <2023-01-09> modify for lcm leakage current end

	//Antai <AI_BSP_LCD> <chenht> <2022-11-25> modify for 2206 gesture begin
	/*	
	PMU_db_pos_neg_disable_delay(15);

	MDELAY(5);
	SET_RESET_PIN(0);
	MDELAY(10);
	*/
	//Antai <AI_BSP_LCD> <chenht> <2022-11-25> modify for 2206 gesture end
}

static void lcm_resume(void)
{
	LCM_PRINT(" yj LCM --- %s \n",__func__);
//Antai <AI_BSP_LCD> <penggy> <20210608> add for lcm  start	
	MDELAY(5);
	//Antai <AI_BSP_LCD> <chenht> <2022-11-25> modify for 2206 gesture begin
	if(is_gesture_mode_fts() == 0)
		PMU_db_pos_neg_setting_delay(5);
    //Antai <AI_BSP_LCD> <chenht> <2022-11-25> modify for 2206 gesture end
	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(120);
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
	LCM_PRINT(" YKL&FT8057_LCM_RESET_KL %s \n",__func__);
//Antai <AI_BSP_LCD> <penggy> <20210608> add for lcm  end	
}

static unsigned int lcm_compare_id(void)
{
//AIUI <AI_BSP_SYS> <hehl> <201901105> add for lcm compatible start	
#if 0
	unsigned int id1 = 0;
	unsigned int id2 = 0;
	unsigned int id3 = 0;
	unsigned char buffer[3];
	unsigned int array[16];	
	int ret = 0;
	struct LCM_setting_table switch_table_page1[] = {
		{0x00,1,{0x00}},
		{0xFF,3,{0x87,0x19,0x01}},
		{0x00,1,{0x80}},
		{0xFF,2,{0x87,0x19}},
	};

	/*config mt6371 register 0xB2[7:6]=0x3, that is set db_delay=4ms.*/
        ret = PMU_REG_MASK(0xB2, (0x3 << 6), (0x3 << 6));

        /* set AVDD 5.5v, (4v+30*0.05v) */
//Antaiui <AI_BSP_LCD> <gaoxiaoxing> <20180120> add for ASW1102A-2,start
        ret = PMU_REG_MASK(0xB3, 30, (0x3F << 0));
//Antaiui <AI_BSP_LCD> <gaoxiaoxing> <20180120> add for ASW1102A-2,end
        if (ret < 0)
                LCM_PRINT("yj ft8719----mt6370----cmd=%0x--i2c write error----\n", 0xB3);

        MDELAY(10);
        /* set AVEE */
//Antaiui <AI_BSP_LCD> <gaoxiaoxing> <20180120> add for ASW1102A-2,start
        ret = PMU_REG_MASK(0xB4, 30, (0x3F << 0));
//Antaiui <AI_BSP_LCD> <gaoxiaoxing> <20180120> add for ASW1102A-2,end
        if (ret < 0)
                LCM_PRINT("yj ft8719----mt6370----cmd=%0x--i2c write error----\n", 0xB4);

        /* enable AVDD & AVEE */
        /* 0x12--default value; bit3--Vneg; bit6--Vpos; */
       // ret = PMU_REG_MASK(0xB1, (1<<3) | (1<<6), (1<<3) | (1<<6));
        ret = PMU_REG_MASK(0xB1, (1<<6), (1<<6));
        MDELAY(2);
        ret = PMU_REG_MASK(0xB1, (1<<3), (1<<3));
        if (ret < 0)
                LCM_PRINT("yj ft8719----mt6370----cmd=%0x--i2c write error----\n", 0xB1);


        MDELAY(30);

	mt_set_gpio_mode(GPIO_TP_RESET, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_TP_RESET, GPIO_DIR_OUT);
	mt_set_gpio_mode(GPIO_LCM_RESET, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCM_RESET, GPIO_DIR_OUT);

	mt_set_gpio_out(GPIO_TP_RESET, GPIO_OUT_ONE);
	mt_set_gpio_out(GPIO_LCM_RESET, GPIO_OUT_ONE);
   	MDELAY(1);
	mt_set_gpio_out(GPIO_TP_RESET, GPIO_OUT_ZERO);
	mt_set_gpio_out(GPIO_LCM_RESET, GPIO_OUT_ZERO);
	MDELAY(1);
	mt_set_gpio_out(GPIO_TP_RESET, GPIO_OUT_ONE);
	mt_set_gpio_out(GPIO_LCM_RESET, GPIO_OUT_ONE);
	MDELAY(20);
	push_table(switch_table_page1, sizeof(switch_table_page1) / sizeof(struct LCM_setting_table), 1);
	array[0] = 0x00023700;
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0xA1, &buffer[0], 3);
	id1 = buffer[0];
	id2 = buffer[1];
	id3 = buffer[2];
	printf("lk ft8719 id1=0x%x id2=0x%x id3=0x%x\n",id1,id2,id3);
	if((id1 == 0x01) && (id2 == 0x8B) && (id3 == 0x87))
	{
		printf("lk read ft8719 id success\n");
		return 1;
	}
	else
	{
		printf("lk read ft8719 id fail\n");
		return 0;
	}	
#endif 
     return 1;	
//AIUI <AI_BSP_SYS> <hehl> <201901105> add for lcm compatible end	
}

static unsigned int lcm_ata_check(unsigned char *buffer)
{
#ifndef BUILD_LK
#ifdef  CONFIG_AI_BSP_LCM_ID_ATA
	char *ptr;
	char sbc_value[5] = {0};
    int len = 0;
	ptr = strstr(saved_command_line, "lcm_id=");
	 if (ptr != NULL){
        len = strlen("lcm_id=");
        if (strlen(ptr) > 0){
            ptr += len;
            strncpy(sbc_value, ptr, 1);
			if(0==strcmp(sbc_value,"1"))
			{
				LCM_PRINT("lcm ata check success , lcm_id=%s\n", sbc_value);
				return 1;
			}else{
				LCM_PRINT("lcm ata check fail\n");
				return 0;
			}	
        }else{
            LCM_PRINT("lcm ptr len<0 error\n");
			return 0;
        }
    }else{
		LCM_PRINT("lcm get saved_command_line is null\n");
		return 0;
    }
#else
		return 1;
#endif		
#endif
}
//Antai <AI_BSP_LCD> <hehl> <2021-03-15> add for lcm id ata end
// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
struct LCM_DRIVER ai_ft8057_yikuailai_90hz_lcm_drv =
{
	.name           = "ai_ft8057_yikuailai_90hz",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
	.init_power	= lcm_init_power,
	.resume_power = lcm_resume_power,
	.suspend_power = lcm_suspend_power,
	.ata_check = lcm_ata_check,
};





