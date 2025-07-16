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
{0xF0,2,{0x5A,0x59}},
{0xF1,2,{0xA5,0xA6}},
{0xD4,1,{0x31}},

//BIST
{0xC0,4,{0x40,0x93,0x00,0x1F}},//41OPEN
{0xC1,32,{0x00,0x20,0x20,0xD2,0x04,0x30,0x30,0x04,0x4C,0x06,0x12,0x70,0x35,0x35,0x07,0x11,0x84,0x4C,0x00,0x93,0x18,0x8F,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
{0xC2,1,{0x00}},
{0xC4,32,{0x04,0x33,0xB8,0x40,0x00,0xBC,0x00,0xE0,0x00,0x00,0x00,0x26,0x48,0x91,0xB3,0x75,0x00,0x04,0xE0,0x20,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
{0xC5,32,{0x03,0x24,0x96,0xC8,0x3E,0x00,0x05,0x02,0x12,0x0C,0x00,0x32,0x3F,0x08,0x01,0x3F,0x10,0x20,0xC8,0x03,0x55,0xFF,0x01,0x14,0x38,0x7F,0x00,0x00,0x00,0x00,0x00,0x00}},
{0xC6,32,{0x5E,0x1F,0x15,0x2B,0x2B,0x28,0x3F,0x03,0x16,0x16,0x00,0x11,0x40,0x00,0x98,0x98,0x60,0x80,0x0C,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
{0xC8,3,{0x46,0x00,0x49}}, //V01 2022.12.6
{0xB2,32,{0x95,0x96,0x05,0x04,0xFF,0x22,0x10,0x0C,0x88,0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x00,0xB2,0xB0,0xB2,0xB0,0x00,0x00,0x00,0x00,0x55,0x55,0x10,0x20,0x11,0x00,0x00}},
{0xB3,32,{0xF7,0x09,0x09,0x09,0x81,0xB7,0x00,0x00,0xAE,0x00,0x00,0x00,0x08,0x44,0x0C,0x00,0x00,0x03,0x00,0x00,0x0C,0x00,0x00,0x03,0x00,0x00,0x55,0x55,0x55,0x55,0x55,0x55}},
{0xB4,32,{0x31,0x01,0x01,0x01,0x81,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x08,0x00,0x0C,0x00,0x00,0x03,0x00,0x00,0x0C,0x00,0x00,0x03,0x00,0x00,0x55,0x55,0x55,0x55,0x55,0x55}},
{0xB5,32,{0x00,0x19,0x0A,0x19,0x0A,0xC6,0x26,0x01,0xA2,0x33,0x44,0x00,0x26,0x00,0x59,0x7C,0x02,0x08,0x30,0x30,0x50,0x20,0x40,0x12,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

//FOWAD SCAN
{0xB6,32,{0x00,0x00,0x33,0x00,0x04,0x08,0x0C,0x0E,0x10,0x12,0x14,0x16,0x18,0x1A,0x32,0x32,0x00,0x00,0x00,0x00,0x00,0x00,0x7F,0xFF,0x00,0x20,0x03,0x00,0x00,0x00,0x00,0x00}},
{0xB7,32,{0x00,0x00,0x33,0x00,0x05,0x09,0x0D,0x0F,0x11,0x13,0x15,0x17,0x19,0x1B,0x32,0x32,0x00,0x00,0x00,0x00,0x00,0x00,0x7F,0xFF,0x00,0x20,0x03,0x00,0x00,0x00,0x00,0x00}},

//BACK SCAN
{0xB8,12,{0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55}},
{0xB9,32,{0x01,0x15,0x55,0x55,0x55,0x00,0x00,0x15,0x55,0x55,0x55,0x00,0x00,0xAA,0xAA,0xAA,0xAA,0xAA,0xA0,0xAA,0xAA,0xAA,0xAA,0xAA,0xA0,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
{0xBB,8,{0x01,0x02,0x03,0x0A,0x04,0x13,0x14,0x00}},  //V01 2022.12.6
{0xBC,32,{0x00,0x00,0x00,0x00,0x04,0x00,0xFF,0xF0,0x0B,0x32,0x50,0x49,0x33,0x33,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
{0xBD,6,{0xA1,0xA2,0x52,0x2E,0x00,0x8F}},
{0xBE,16,{0x0C,0x88,0x43,0x38,0x33,0x00,0x00,0x38,0x00,0xB2,0xAF,0xB2,0xAF,0x00,0x00,0x33}},//V01 2022.12.6
{0xBF,9,{0x0C,0x19,0x0C,0x19,0x00,0x11,0x04,0x18,0x50}},
{0xCA,32,{0x00,0x40,0x00,0x19,0x46,0x94,0x41,0x8F,0x44,0x44,0x50,0x50,0x5A,0x5A,0x64,0x64,0x32,0x32,0x11,0x00,0x01,0x01,0x0A,0x06,0x22,0x00,0x05,0x00,0x00,0x46,0x5A,0x04}},
{0xFB,2,{0x40,0x54}},

//VCOM SETTING//烧录VCOM则需屏蔽
//89 E1 E1 11

//GAMMA
{0xC7,32,{0x73,0x10,0x01,0x23,0x45,0x67,0x77,0x77,0x30,0x73,0x10,0x01,0x23,0x45,0x67,0x77,0x77,0x30,0x31,0x00,0x01,0xFF,0xFF,0x40,0xF0,0x0E,0x41,0x00,0x00,0x00,0x00,0x00}},//VGMP=5.2,VGMN=-5.5V

//GAMMA 2.2
{0x80,32,{0xFF,0xFD,0xF9,0xF6,0xF3,0xF0,0xED,0xEA,0xE7,0xDE,0xD5,0xC5,0xB4,0xA4,0x96,0x8A,0x7E,0x7E,0x72,0x66,0x5A,0x4D,0x40,0x30,0x26,0x18,0x14,0x10,0x0D,0x0A,0x07,0x04}},
{0x81,32,{0xFF,0xFD,0xF9,0xF6,0xF3,0xF0,0xED,0xEA,0xE7,0xDE,0xD5,0xC5,0xB4,0xA4,0x96,0x8A,0x7E,0x7E,0x72,0x66,0x5A,0x4D,0x40,0x30,0x26,0x18,0x14,0x10,0x0D,0x0A,0x07,0x04}},
{0x82,32,{0xFF,0xFD,0xF9,0xF6,0xF3,0xF0,0xED,0xEA,0xE7,0xDE,0xD5,0xC5,0xB4,0xA4,0x96,0x8A,0x7E,0x7E,0x72,0x66,0x5A,0x4D,0x40,0x30,0x26,0x18,0x14,0x10,0x0D,0x0A,0x07,0x04}},
{0x83,13,{0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00}},
{0x84,27,{0x18,0x3E,0xD4,0x3D,0x8B,0x5F,0x3D,0xFF,0xE0,0x18,0x3E,0xD4,0x3D,0x8B,0x5F,0x3D,0xFF,0xE0,0x18,0x3E,0xD4,0x3D,0x8B,0x5F,0x3D,0xFF,0xE0}},

{0xE0,7,{0x0C,0x00,0xB0,0x10,0xC4,0x09,0x00}},//12BIT 20KHZ

{0xF1,2,{0x5A,0x59}},
{0xF0,2,{0xA5,0xA6}},
{0x35,2,{0x00,0x00}},
{0x51,2,{0x0F,0xFF}},
{0x53,2,{0x24,0x00}},

{0x11,1,{0x00}},
{REGFLAG_DELAY,120, {}},
{0x29,1,{0x00}},
{REGFLAG_DELAY,10, {}},
{0x6D,2,{0x02,0x00}},
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

	params->dsi.vertical_sync_active = 4;
	params->dsi.vertical_backporch = 32;
	//Antai <AI_BSP_LCD> <chenht> <2023-02-04> modify for lcm frame rate switching begin
	params->dsi.vertical_frontporch = 210; 
	//Antai <AI_BSP_LCD> <chenht> <2023-02-04> modify for lcm frame rate switching end
 	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 4;
	params->dsi.horizontal_backporch = 48;
	params->dsi.horizontal_frontporch = 48;
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
	params->dsi.PLL_CLOCK = 290;

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
	LCM_PRINT("ai_icnl9916_yikuailai_60hz kernel lcm_init\n");		
}

static void lcm_suspend(void)
{
	unsigned int data_array[16];
	
	LCM_PRINT("yj LCM --- %s \n",__func__);
	
	data_array[0] = 0x00280500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(10);

	data_array[0] = 0x00100500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);
		
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
	LCM_PRINT(" YKL&ICNL9916_LCM_RESET_KL %s \n",__func__);
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
struct LCM_DRIVER ai_icnl9916_yikuailai_60hz_lcm_drv =
{
	.name           = "ai_icnl9916_yikuailai_60hz",
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





