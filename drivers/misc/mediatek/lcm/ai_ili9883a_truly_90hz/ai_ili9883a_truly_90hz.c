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
#define FRAME_HEIGHT (1600)
#ifndef BUILD_LK
#define LCM_DENSITY                     (480)
#endif
//Antai <AI_BSP_LCD> <chenht> <2022-12-15> change lcm display size start    
#define PHYSICAL_WIDTH  (69308)
#define PHYSICAL_HEIGHT (157240) //160640
//Antai <AI_BSP_LCD> <chenht> <2022-12-15> change lcm display size end  
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
//GIP Setting
{0xFF,3,{0x98,0x83,0x01}},
{0x00,1,{0x46}},       
{0x01,1,{0x13}},      //STV 4H
{0x02,1,{0x00}},      //STV rise adjust
{0x03,1,{0x20}},      //STV fall adjust
		   
{0x04,1,{0x01}},     
{0x05,1,{0x13}},      //RST 4H
{0x06,1,{0x00}},      //RST rise adjust
{0x07,1,{0x20}},      //RST fall adjust
		   
{0x08,1,{0x82}},            
{0x09,1,{0x05}},      
{0x0A,1,{0x73}},      //CLK 4H      
{0x0B,1,{0x00}},      
		   
{0x0C,1,{0x05}},      //CLKA rise adjust     sensor mura  0413  4E
{0x0D,1,{0x05}},      //CLKA rise adjust     sensor mura  0413  4E
{0x0E,1,{0x00}},      //CLKA fall adjust 
{0x0F,1,{0x00}},      //CLKA fall adjust
		   
{0x10,1,{0x00}},      //gate EQT1    
{0x11,1,{0x0a}},      //Gate EQT2  0419


//FW ONLY
//FW_R
{0x31,1,{0x02}},     // VGL
{0x32,1,{0x02}},     // VGL
{0x33,1,{0x0C}},     // RST_L
{0x34,1,{0x0C}},     // RST_L
{0x35,1,{0x02}},     // VGL_L
{0x36,1,{0x02}},     // VGL_L
{0x37,1,{0x16}},     // CLK2B_L
{0x38,1,{0x16}},     // CLK2B_L
{0x39,1,{0x14}},     // CLK1B_L
{0x3A,1,{0x14}},     // CLK1B_L
{0x3B,1,{0x12}},     // CLK2_L
{0x3C,1,{0x12}},     // CLK2_L
{0x3D,1,{0x10}},     // CLK1_L
{0x3E,1,{0x10}},     // CLK1_L
{0x3F,1,{0x08}},     // STV_L
{0x40,1,{0x08}},     // STV_L
{0x41,1,{0x07}},     // Hi-Z
{0x42,1,{0x07}},     // Hi-Z
{0x43,1,{0x07}},     // Hi-Z
{0x44,1,{0x07}},     // Hi-Z
{0x45,1,{0x07}},     // Hi-Z
{0x46,1,{0x07}},     // Hi-Z

//FW_L
{0x47,1,{0x02}},     // VGL
{0x48,1,{0x02}},     // VGL
{0x49,1,{0x0D}},     // RST_R
{0x4A,1,{0x0D}},     // RST_R
{0x4B,1,{0x02}},     // VGL_R
{0x4C,1,{0x02}},     // VGL_R
{0x4D,1,{0x17}},     // CLK2B_R
{0x4E,1,{0x17}},     // CLK2B_R
{0x4F,1,{0x15}},     // CLK1B_R
{0x50,1,{0x15}},     // CLK1B_R
{0x51,1,{0x13}},     // CLK2_R
{0x52,1,{0x13}},     // CLK2_R
{0x53,1,{0x11}},     // CLK1_R
{0x54,1,{0x11}},     // CLK1_R
{0x55,1,{0x09}},     // STV_R
{0x56,1,{0x09}},     // STV_R
{0x57,1,{0x07}},     // Hi-Z
{0x58,1,{0x07}},     // Hi-Z
{0x59,1,{0x07}},     // Hi-Z
{0x5A,1,{0x07}},     // Hi-Z
{0x5B,1,{0x07}},     // Hi-Z
{0x5C,1,{0x07}},     // Hi-Z

// RTN. Internal VBP, Internal VFP
{0xFF,3,{0x98,0x83,0x02}},
{0x06,1,{0x57}},     // Internal Line Time (RTN)
{0x0A,1,{0x5B}},     // Internal VFP[9:8]
{0x0C,1,{0x00}},     // Internal VBP[8]
{0x0D,1,{0x20}},     // Internal VBP
{0x0E,1,{0x73}},     // Internal VFP
{0x39,1,{0x05}},     // VBP[8],RTN[8],VFP[9:8]
{0x3A,1,{0x20}},     // BIST VBP
{0x3B,1,{0x73}},     // BIST VFP
{0x3C,1,{0x7C}},     // BIST RTN
{0xF0,1,{0x05}},     // BIST VFP[12:8] (FRM_RAT change)
{0xF1,1,{0x5D}},     // BIST VFP (FRM_RAT change)
{0x40,1,{0x4B}},		//SDT=1.34us
{0x4A,1,{0x00}},     // COMPR disable (Default)
{0x4D,1,{0xCE}},     // Power Saving Disable

{0x4e,1,{0x11}},      //SRC BIAS   0419
{0x29,1,{0x38}},      //vgh pumping freq   0422
{0x2A,1,{0x38}},      //vgl pumping freq    0422

// Power Setting
{0xFF,3,{0x98,0x83,0x05}},
{0x03,1,{0x00}},     // VCOM
{0x04,1,{0xA6}},     // VCOM

{0x60,1,{0x6b}},       //pumping single mode  63 ,  dual mode=0x6b  0419
{0x61,1,{0x01}},	//VGL x2 ratio   0419
{0x65,1,{0x07}},       //VGHO short VGH, VGL short VGLO   0419
{0x88,1,{0x02}},	// VCL=-2.64   0419


{0x69,1,{0x97}},     // GVDDN = -5.5V
{0x6A,1,{0x97}},     // GVDDP = 5.5V
{0x6D,1,{0xA1}},     // VGHO = 15V
{0x73,1,{0xa7}},     // VGH = 16V
{0x79,1,{0x79}},    // VGLO = -10V
{0x7F,1,{0x59}},     // VGL = -10V short to VGLO
{0xa4,1,{0x03}},    // LVD disable 模组段需要移除
// Resolution
{0xFF,3,{0x98,0x83,0x06}},
{0xD9,1,{0x1F}},     // 4Lane
{0xC0,1,{0x40}},     // NL = 1600
{0xC1,1,{0x16}},     // NL = 1600

{0xD6,1,{0x55}},     // FTE SEL
{0xDC,1,{0x68}},     // FTE OUTPUT TSHD

//Gamma Register
//Gamma Register									
{0xFF,3,{0x98,0x83,0x08}},	   //0413								
//{0xE0,29,00,24,51,9E,DB,55,20,53,7C,AB,A9,D1,0D,40,6C,EA,98,C3,F6,14,FF,3B,5D,89,BC,3F,DA,EB,EC									
//{0xE1,29,00,24,51,9E,DB,55,20,53,7C,AB,A9,D1,0D,40,6C,EA,98,C3,F6,14,FF,3B,5D,89,BC,3F,DA,EB,EC									

//Gamma Register									
	
{0xFF,3,{0x98,0x83,0x08}},	 //0414								
{0xE0,29,{0x00,0x24,0x57,0xA8,0xE5,0x55,0x28,0x5A,0x81,0xAD,0xA9,0xD0,0x04,0x2E,0x52,0xAA,0x75,0x98,0xC3,0xDE,0xFF,0x01,0x20,0x49,0x7B,0x3F,0xA6,0xD1,0xEC}},								
{0xE1,29,{0x00,0x24,0x57,0xA8,0xE5,0x55,0x28,0x5A,0x81,0xAD,0xA9,0xD0,0x04,0x2E,0x52,0xAA,0x75,0x98,0xC3,0xDE,0xFF,0x01,0x20,0x49,0x7B,0x3F,0xA6,0xD1,0xEC}},								
								


// OSC Auto Trim Setting
{0xFF,3,{0x98,0x83,0x0B}},
{0x9A,1,{0x42}},
{0x9B,1,{0xC5}},
{0x9C,1,{0x02}},
{0x9D,1,{0x02}},
{0x9E,1,{0x45}},
{0x9F,1,{0x45}},
{0xAB,1,{0xE0}},     // AutoTrimType

{0xFF,3,{0x98,0x83,0x0E}},
{0x11,1,{0x4E}},    // TSVD_LONGV_RISE [4:0]  4B->4E  TSVD to tshd time
{0x12,1,{0x02}},    // TSVD _LongV_Shift
{0x13,1,{0x14}},    // LV mode TSHD Rise position [4:0]
{0x00,1,{0xA0}},    // LV mode

{0xFF,3,{0x98,0x83,0x00}},
{0x35,1,{0x00}},
{0x11,1,{0x00}},
{REGFLAG_DELAY,120, {}},
{0x29,1,{0x00}},
{REGFLAG_DELAY,20, {}},
    {REGFLAG_END_OF_TABLE,0x00,{}}
};

/*******************Dynfps start*************************/
#if 0//def CONFIG_MTK_HIGH_FRAME_RATE

#define DFPS_MAX_CMD_NUM 10

struct LCM_dfps_cmd_table {
	bool need_send_cmd;
	struct LCM_setting_table prev_f_cmd[DFPS_MAX_CMD_NUM];
};

static struct LCM_dfps_cmd_table
	dfps_cmd_table[DFPS_LEVELNUM][DFPS_LEVELNUM] = {

/**********level 0 to 0,1 cmd*********************/
[DFPS_LEVEL0][DFPS_LEVEL0] = {
	false,
	/*prev_frame cmd*/
	{
	{REGFLAG_END_OF_TABLE, 0x00, {} },
	},
},
/*60->90*/
[DFPS_LEVEL0][DFPS_LEVEL1] = {
	true,
	/*prev_frame cmd*/
	{
	{0xFF, 1, {0x25} },
	{0xFB, 1, {0x01} },
	{0x18, 1, {0x20} },
	{REGFLAG_END_OF_TABLE, 0x00, {} },
	},
},

/**********level 1 to 0,1 cmd*********************/
/*90->60*/
[DFPS_LEVEL1][DFPS_LEVEL0] = {
	true,
	/*prev_frame cmd*/
	{
	{0xFF, 1, {0x25} },
	{0xFB, 1, {0x01} },
	{0x18, 1, {0x21} },
	{REGFLAG_END_OF_TABLE, 0x00, {} },
	},
},

[DFPS_LEVEL1][DFPS_LEVEL1] = {
	false,
	/*prev_frame cmd*/
	{
	{REGFLAG_END_OF_TABLE, 0x00, {} },
	},

},

};
#endif
/*******************Dynfps end*************************/
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
//Antaiui <AI_BSP_SNS> <wutj> <2022-12-19> chang 90hz config start
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
	//Antaiui <AI_BSP_SNS> <wutj> <2022-11-24> chang 90hz config start
	dfps_params[0].vertical_frontporch = 1370;
	//Antaiui <AI_BSP_SNS> <wutj> <2022-11-24> chang 90hz config end
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
	dfps_params[1].vertical_frontporch = 371; //371
	//dfps_params[0].vertical_frontporch_for_low_power = 540;

	/*if need mipi hopping params add here*/
	/*dfps_params[1].PLL_CLOCK_dyn =xx;
	 *dfps_params[1].horizontal_frontporch_dyn =xx ;
	 * dfps_params[1].vertical_frontporch_dyn= 54;
	 * dfps_params[1].vertical_frontporch_for_low_power_dyn =xx;
	 */

	dsi->dfps_num = 2;
}
//Antaiui <AI_BSP_SNS> <wutj> <2022-12-19> chang 90hz config start
#endif
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
    params->dsi.vertical_backporch = 30;
	//Antaiui <AI_BSP_SNS> <wutj> <2022-12-19> chang 90hz config start
	params->dsi.vertical_frontporch = 1370; //346
	//Antaiui <AI_BSP_SNS> <wutj> <2022-12-19> chang 90hz config end
 	params->dsi.vertical_active_line = FRAME_HEIGHT;

 	params->dsi.horizontal_sync_active = 11;
	params->dsi.horizontal_backporch = 20;
	params->dsi.horizontal_frontporch = 20;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;

	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 1;
	params->dsi.lcm_esd_check_table[0].cmd          = 0x0A;
	params->dsi.lcm_esd_check_table[0].count        = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;

	params->dsi.ssc_disable = 1;
	params->dsi.ssc_range = 4;
	params->dsi.cont_clock = 1;
//	params->dsi.clk_lp_per_line_enable=1;
//Antaiui <AI_BSP_SNS> <wutj> <2022-12-19> chang 90hz config start
	params->dsi.PLL_CLOCK = 455;
//Antaiui <AI_BSP_SNS> <wutj> <2022-12-19> chang 90hz config end
	
	#ifdef CONFIG_MTK_HIGH_FRAME_RATE
	/****DynFPS start****/
	lcm_dfps_int(&(params->dsi));
	/****DynFPS end****/
#endif

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
extern int is_gesture_mode(void);
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
	//Antai <AI_BSP_LCD> <wutj> <2022-08-22> close pmu suspend function start
	LCM_PRINT("is gesture mode is %d",is_gesture_mode());
	if(is_gesture_mode()==0){
	PMU_db_pos_neg_disable_delay(15);
	MDELAY(5);
	SET_RESET_PIN(0);
	MDELAY(10);
	}
	//Antai <AI_BSP_LCD> <wutj> <2022-08-22> close pmu suspend function end	
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

/*******************Dynfps start*************************/
#if 0//def CONFIG_MTK_HIGH_FRAME_RATE
static void dfps_dsi_push_table(
	void *cmdq, struct LCM_setting_table *table,
	unsigned int count, unsigned char force_update)
{
	unsigned int i;
	unsigned int cmd;

	for (i = 0; i < count; i++) {
		cmd = table[i].cmd;
		switch (cmd) {
		case REGFLAG_END_OF_TABLE:
			return;
		default:
			dfps_dsi_send_cmd(
				cmdq, cmd, table[i].count,
				table[i].para_list, force_update);
			break;
		}
	}

}
static bool lcm_dfps_need_inform_lcm(
	unsigned int from_level, unsigned int to_level)
{
	struct LCM_dfps_cmd_table *p_dfps_cmds = NULL;

	if (from_level == to_level) {
		LCM_LOGI("%s,same level\n", __func__);
		return false;
	}
	p_dfps_cmds =
		&(dfps_cmd_table[from_level][to_level]);

	return p_dfps_cmds->need_send_cmd;
}

static void lcm_dfps_inform_lcm(void *cmdq_handle,
unsigned int from_level, unsigned int to_level)
{
	struct LCM_dfps_cmd_table *p_dfps_cmds = NULL;

	if (from_level == to_level) {
		LCM_LOGI("%s,same level\n", __func__);
		goto done;
	}
	p_dfps_cmds =
		&(dfps_cmd_table[from_level][to_level]);

	if (p_dfps_cmds &&
		!(p_dfps_cmds->need_send_cmd)) {
		LCM_LOGI("%s,no cmd[L%d->L%d]\n",
			__func__, from_level, to_level);
		goto done;
	}

	dfps_dsi_push_table(
		cmdq_handle, p_dfps_cmds->prev_f_cmd,
		ARRAY_SIZE(p_dfps_cmds->prev_f_cmd), 1);
done:
	LCM_LOGI("%s,done %d->%d\n",
		__func__, from_level, to_level);

}
#endif
/*******************Dynfps end*************************/

//Antai <AI_BSP_LCD> <hehl> <2021-03-15> add for lcm id ata end
// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
struct LCM_DRIVER ai_ili9883a_truly_90hz_lcm_drv =
{
	.name           = "ai_ili9883a_truly_90hz",
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
#if 0//def CONFIG_MTK_HIGH_FRAME_RATE
	/*DynFPS*/
	.dfps_send_lcm_cmd = lcm_dfps_inform_lcm,
	.dfps_need_send_cmd = lcm_dfps_need_inform_lcm,
#endif
};





