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

#define FRAME_WIDTH     (1080)
#define FRAME_HEIGHT (2400)
#ifndef BUILD_LK
#define LCM_DENSITY                     (480)
#endif
//Antaiui <AI_BSP_LCM> <hehl> <2022-01-15> modify lcm size begin
#define PHYSICAL_WIDTH  (73710)  
#define PHYSICAL_HEIGHT (148640)
//Antaiui <AI_BSP_LCM> <hehl> <2022-01-15> modify lcm size end
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
{0XFF,1,{0X10}},
{0XFB,1,{0X01}},
{0XB0,1,{0X00}},
{0XC0,1,{0X00}},
{0XC2,2,{0X1B,0XA0}},

{0XFF,1,{0X20}},
{0XFB,1,{0X01}},
{0X01,1,{0X66}},
{0X06,1,{0X40}},
{0X07,1,{0X38}},
{0X18,1,{0X66}},
{0X1B,1,{0X01}},
{0X2F,1,{0X83}},
{0X5C,1,{0X90}},
{0X5E,1,{0XAA}},
{0X69,1,{0X91}},
{0X95,1,{0XD1}},
{0X96,1,{0XD1}},
{0XF2,1,{0X65}},
{0XF3,1,{0X64}},
{0XF4,1,{0X65}},
{0XF5,1,{0X64}},
{0XF6,1,{0X65}},
{0XF7,1,{0X64}},
{0XF8,1,{0X65}},
{0XF9,1,{0X64}},

{0XFF,1,{0X24}},
{0XFB,1,{0X01}},
{0X01,1,{0X0F}},
{0X03,1,{0X0C}},
{0X05,1,{0X1D}},
{0X08,1,{0X2F}},
{0X09,1,{0X2E}},
{0X0A,1,{0X2D}},
{0X0B,1,{0X2C}},
{0X11,1,{0X17}},
{0X12,1,{0X13}},
{0X13,1,{0X15}},
{0X15,1,{0X14}},
{0X16,1,{0X16}},
{0X17,1,{0X18}},
{0X1B,1,{0X01}},
{0X1D,1,{0X1D}},
{0X20,1,{0X2F}},
{0X21,1,{0X2E}},
{0X22,1,{0X2D}},
{0X23,1,{0X2C}},
{0X29,1,{0X17}},
{0X2A,1,{0X13}},
{0X2B,1,{0X15}},
{0X2F,1,{0X14}},
{0X30,1,{0X16}},
{0X31,1,{0X18}},
{0X32,1,{0X04}},
{0X34,1,{0X10}},
{0X35,1,{0X1F}},
{0X36,1,{0X1F}},
{0X37,1,{0X20}},
{0X4D,1,{0X1B}},
{0X4E,1,{0X4B}},
{0X4F,1,{0X4B}},
{0X53,1,{0X4B}},
{0X71,1,{0X30}},
{0X79,1,{0X11}},
{0X7A,1,{0X82}},
{0X7B,1,{0X96}},
{0X7D,1,{0X04}},
{0X80,1,{0X04}},
{0X81,1,{0X04}},
{0X82,1,{0X13}},
{0X84,1,{0X31}},
{0X85,1,{0X00}},
{0X86,1,{0X00}},
{0X87,1,{0X00}},
{0X90,1,{0X13}},
{0X92,1,{0X31}},
{0X93,1,{0X00}},
{0X94,1,{0X00}},
{0X95,1,{0X00}},
{0X9C,1,{0XF4}},
{0X9D,1,{0X01}},
{0XA0,1,{0X16}},
{0XA2,1,{0X16}},
{0XA3,1,{0X02}},
{0XA4,1,{0X04}},
{0XA5,1,{0X04}},
{0XC6,1,{0XC0}},
{0XC9,1,{0X00}},
{0XD9,1,{0X80}},
{0XE9,1,{0X02}},

{0XFF,1,{0X25}},
{0XFB,1,{0X01}},
{0X0F,1,{0X1B}},
{0X18,1,{0X21}},
{0X19,1,{0XE4}},
{0X21,1,{0X40}},
{0X66,1,{0XD8}},
{0X68,1,{0X50}},
{0X69,1,{0X10}},
{0X6B,1,{0X00}},
{0X6D,1,{0X0D}},
{0X6E,1,{0X48}},
{0X72,1,{0X41}},
{0X73,1,{0X4A}},
{0X74,1,{0XD0}},
{0X77,1,{0X62}},
{0X79,1,{0X77}},
{0X7D,1,{0X40}},
{0X7E,1,{0X1D}},
{0X7F,1,{0X00}},
{0X80,1,{0X04}},
{0X84,1,{0X0D}},
{0XCF,1,{0X80}},
{0XD6,1,{0X80}},
{0XD7,1,{0X80}},
{0XEF,1,{0X20}},
{0XF0,1,{0X84}},

{0XFF,1,{0X26}},
{0XFB,1,{0X01}},
{0X15,1,{0X04}},
{0X81,1,{0X16}},
{0X83,1,{0X02}},
{0X84,1,{0X03}},
{0X85,1,{0X01}},
{0X86,1,{0X03}},
{0X87,1,{0X01}},
{0X88,1,{0X00}},
{0X8A,1,{0X1A}},
{0X8B,1,{0X11}},
{0X8C,1,{0X24}},
{0X8E,1,{0X42}},
{0X8F,1,{0X11}},
{0X90,1,{0X11}},
{0X91,1,{0X11}},
{0X9A,1,{0X80}},
{0X9B,1,{0X04}},
{0X9C,1,{0X00}},
{0X9D,1,{0X00}},
{0X9E,1,{0X00}},

{0XFF,1,{0X27}},
{0XFB,1,{0X01}},
{0X01,1,{0X60}},
{0X20,1,{0X81}},
{0X21,1,{0XE7}},
{0X25,1,{0X82}},
{0X26,1,{0X1F}},
{0X6E,1,{0X00}},
{0X6F,1,{0X00}},
{0X70,1,{0X00}},
{0X71,1,{0X00}},
{0X72,1,{0X00}},
{0X75,1,{0X00}},
{0X76,1,{0X00}},
{0X77,1,{0X00}},
{0X7D,1,{0X09}},
{0X7E,1,{0X5F}},
{0X80,1,{0X23}},
{0X82,1,{0X09}},
{0X83,1,{0X5F}},
{0X88,1,{0X01}},
{0X89,1,{0X10}},
{0XA5,1,{0X10}},
{0XA6,1,{0X23}},
{0XA7,1,{0X01}},
{0XB6,1,{0X40}},
{0XE3,1,{0X02}},
{0XE4,1,{0XDA}},
{0XE5,1,{0X01}},
{0XE6,1,{0X6D}},
{0XE9,1,{0X03}},
{0XEA,1,{0X2F}},
{0XEB,1,{0X01}},
{0XEC,1,{0X98}},

{0XFF,1,{0X2A}},
{0XFB,1,{0X01}},
{0X00,1,{0X91}},
{0X03,1,{0X20}},
{0X07,1,{0X52}},
{0X0A,1,{0X70}},
{0X0D,1,{0X40}},
{0X0E,1,{0X02}},
{0X11,1,{0XF0}},
{0X15,1,{0X0E}},
{0X16,1,{0XB6}},
{0X19,1,{0X0E}},
{0X1A,1,{0X8A}},
{0X1B,1,{0X14}},
{0X1D,1,{0X36}},
{0X1E,1,{0X4F}},
{0X1F,1,{0X51}},
{0X20,1,{0X4F}},
{0X28,1,{0XEC}},
{0X29,1,{0X0C}},
{0X2A,1,{0X05}},
{0X2D,1,{0X06}},
{0X2F,1,{0X02}},
{0X30,1,{0X4A}},
{0X33,1,{0X0E}},
{0X34,1,{0XEE}},
{0X35,1,{0X30}},
{0X36,1,{0X06}},
{0X37,1,{0XE9}},
{0X38,1,{0X34}},
{0X39,1,{0X02}},
{0X3A,1,{0X4A}},
{0X45,1,{0X0E}},
{0X46,1,{0X40}},
{0X47,1,{0X03}},
{0X4A,1,{0XA0}},
{0X4E,1,{0X0E}},
{0X4F,1,{0XB6}},
{0X52,1,{0X0E}},
{0X53,1,{0X8A}},
{0X54,1,{0X14}},
{0X56,1,{0X36}},
{0X57,1,{0X76}},
{0X58,1,{0X76}},
{0X59,1,{0X76}},
{0X60,1,{0X80}},
{0X61,1,{0XA0}},
{0X62,1,{0X04}},
{0X63,1,{0X33}},
{0X65,1,{0X04}},
{0X66,1,{0X01}},
{0X67,1,{0X05}},
{0X68,1,{0X40}},
{0X6A,1,{0X4C}},
{0X6B,1,{0XA2}},
{0X6C,1,{0X21}},
{0X6D,1,{0XBB}},
{0X6E,1,{0X9F}},
{0X6F,1,{0X23}},
{0X70,1,{0XB9}},
{0X71,1,{0X05}},
{0X7A,1,{0X07}},
{0X7B,1,{0X40}},
{0X7D,1,{0X01}},
{0X7F,1,{0X2C}},
{0X83,1,{0X0E}},
{0X84,1,{0XB6}},
{0X87,1,{0X0E}},
{0X88,1,{0X8A}},
{0X89,1,{0X14}},
{0X8B,1,{0X36}},
{0X8C,1,{0X3A}},
{0X8D,1,{0X3A}},
{0X8E,1,{0X3A}},
{0X95,1,{0X80}},
{0X96,1,{0XFD}},
{0X97,1,{0X14}},
{0X98,1,{0X17}},
{0X99,1,{0X01}},
{0X9A,1,{0X08}},
{0X9B,1,{0X02}},
{0X9C,1,{0X4C}},
{0X9D,1,{0XAF}},
{0X9F,1,{0X6B}},
{0XA0,1,{0XFF}},
{0XA2,1,{0X40}},
{0XA3,1,{0X6F}},
{0XA4,1,{0XF9}},
{0XA5,1,{0X45}},
{0XA6,1,{0X6A}},
{0XA7,1,{0X4C}},

{0XFF,1,{0X2C}},
{0XFB,1,{0X01}},
{0X00,1,{0X02}},
{0X01,1,{0X02}},
{0X02,1,{0X02}},
{0X03,1,{0X16}},
{0X04,1,{0X16}},
{0X05,1,{0X16}},
{0X0D,1,{0X1F}},
{0X0E,1,{0X1F}},
{0X16,1,{0X1B}},
{0X17,1,{0X4B}},
{0X18,1,{0X4B}},
{0X19,1,{0X4B}},
{0X2A,1,{0X03}},
{0X4D,1,{0X16}},
{0X4E,1,{0X02}},
{0X4F,1,{0X27}},
{0X53,1,{0X02}},
{0X54,1,{0X02}},
{0X55,1,{0X02}},
{0X56,1,{0X0E}},
{0X58,1,{0X0E}},
{0X59,1,{0X0E}},
{0X61,1,{0X1F}},
{0X62,1,{0X1F}},
{0X6A,1,{0X14}},
{0X6B,1,{0X34}},
{0X6C,1,{0X34}},
{0X6D,1,{0X34}},
{0X7E,1,{0X03}},
{0X9D,1,{0X0E}},
{0X9E,1,{0X02}},
{0X9F,1,{0X03}},

{0XFF,1,{0XE0}},
{0XFB,1,{0X01}},
{0X35,1,{0X82}},

{0XFF,1,{0XF0}},
{0XFB,1,{0X01}},
{0X1C,1,{0X01}},
{0X33,1,{0X01}},
{0X5A,1,{0X00}},
{0XD2,1,{0X52}},

{0XFF,1,{0XD0}},
{0XFB,1,{0X01}},
{0X53,1,{0X22}},
{0X54,1,{0X02}},

{0XFF,1,{0XC0}},
{0XFB,1,{0X01}},
{0X9C,1,{0X11}},
{0X9D,1,{0X11}},

{0XFF,1,{0X2B}},
{0XFB,1,{0X01}},
{0XB7,1,{0X08}},
{0XB8,1,{0X0B}},

{0XFF,1,{0X10}},

{0x11,1,{0x00}},
{REGFLAG_DELAY,120, {}},
{0x29,1,{0x00}},
{REGFLAG_DELAY,20, {}},
    {REGFLAG_END_OF_TABLE,0x00,{}}
};

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

	params->dsi.vertical_sync_active = 2;
	params->dsi.vertical_backporch = 14;
	params->dsi.vertical_frontporch = 16;
 	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 10;
	params->dsi.horizontal_backporch = 60;
	params->dsi.horizontal_frontporch = 60;
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
	params->dsi.PLL_CLOCK = 560;	/* this value must be in MTK suggested table 1378*/

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

static void lcm_suspend(void)
{
	unsigned int data_array[16];

	LCM_PRINT("yj LCM --- %s \n",__func__);
	data_array[0] = 0x00280500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(30);

	data_array[0] = 0x00100500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(100);
	PMU_db_pos_neg_disable_delay(15);

	MDELAY(5);
	SET_RESET_PIN(0);
	MDELAY(10);
}

static void lcm_resume(void)
{
	LCM_PRINT(" yj LCM --- %s \n",__func__);
//Antai <AI_BSP_LCD> <penggy> <20210608> add for lcm  start	
	MDELAY(5);
	PMU_db_pos_neg_setting_delay(5);
	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(120);
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
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
struct LCM_DRIVER ai_truly_nt36672c_lcm_drv =
{
	.name           = "ai_truly_nt36672c",
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





