#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#ifdef CONFIG_OF
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#endif
#include <linux/gpio.h>

#include "ai_sublcd_gc9106.h"
//#include "ai_sublcd_spi.h"
#include <linux/mutex.h>



extern int __init ai_sublcd_display_init(void);
extern void __exit ai_sublcd_display_exit(void);
extern void gc9106_display_create_sysfs(struct platform_device *dev);
//Antaiui <AI_BSP_LCD> <zhuli> <2022-07-04> add for sub lcd test in factory test begin
extern void cancel_display_work(void);
//Antaiui <AI_BSP_LCD> <zhuli> <2022-07-04> add for sub lcd test in factory test end



#define AI_SUBLCD_GC9106_DTS    "mediatek,ai_gc9106"
#define AI_SUBLCD_GC9106_NAME   "ai_gc9106"

#define SPIDelay  ndelay(10)

#define LCD_CtrlWrite(v) ai_gc9106_write_cmd_byte(&g_gc9106_data,(v))
#define LCD_DataWrite(v) ai_gc9106_write_data_byte(&g_gc9106_data,(v))
#define LCD_DataRead()   ai_gc9106_read_byte(&g_gc9106_data)


#define DISPLAY_BUFSIZE (80*160*2)

//Antaiui <AI_BSP_LCD> <zhuli> <2022-07-06> add for sub lcd mutex begin
static DEFINE_MUTEX(wake_lock);
//Antaiui <AI_BSP_LCD> <zhuli> <2022-07-06> add for sub lcd mutex end


static int ai_gc9106_probe(struct platform_device *dev);
static int ai_gc9106_remove(struct platform_device *dev);

 struct gc9106_platform_data {
    unsigned int cs_gpio;
    unsigned int scl_gpio;
    unsigned int sda_gpio;
    unsigned int dcx_gpio;
    unsigned int rst_gpio;
    unsigned int ldo_gpio;
    unsigned int lcd_id;
    unsigned char is_init;
    unsigned char is_suspend;
    unsigned char *d_buf;
    //Antaiui <AI_BSP_LCD> <zhuli> <2022-07-18> add for sub lcd test begin
    bool is_testing;
    //Antaiui <AI_BSP_LCD> <zhuli> <2022-07-18> add for sub lcd test end
    struct delayed_work dwork;
 };


static struct gc9106_platform_data g_gc9106_data = {0,};
#ifdef CONFIG_OF
static const struct of_device_id gc9106_of_match[] = {
	{.compatible = AI_SUBLCD_GC9106_DTS},
	{},
};
MODULE_DEVICE_TABLE(of, gc9106_of_match);
#endif

static struct platform_driver ai_gc9106_platform_driver = {
	.probe = ai_gc9106_probe,
	.remove = ai_gc9106_remove,
	.driver = {
		.name = AI_SUBLCD_GC9106_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = gc9106_of_match,
#endif
	},
};

static __maybe_unused void ai_gc9106_write_data_byte(struct gc9106_platform_data *data, unsigned char byte);
static __maybe_unused void ai_gc9106_write_cmd_byte(struct gc9106_platform_data *data, unsigned char byte);
static __maybe_unused void ai_gc9106_write_data_simulate(struct gc9106_platform_data *data, unsigned char* p_data, int len);
static unsigned char ai_gc9106_read_byte(struct gc9106_platform_data *data );

static void ai_gc9106_cs_is_enable(struct gc9106_platform_data *data, int enable)
{
	if(enable) {
		gpio_direction_output(data->cs_gpio, 0);
	}else {
		gpio_direction_output(data->cs_gpio, 1);
	}
}
static void ai_gc9106_write_byte(struct gc9106_platform_data *data, unsigned char byte)
{
	int i;
	//gpio_direction_output(data->scl_gpio, 0);

	//SPIDelay;
	for(i = 7; i >= 0; i--) {
		gpio_direction_output(data->scl_gpio, 0);
		if(byte & (1 << i)) {
			gpio_direction_output(data->sda_gpio, 1);
		} else {
			gpio_direction_output(data->sda_gpio, 0);
		}
		//ai_ndelay(10);
		gpio_direction_output(data->scl_gpio, 1);
		//ai_ndelay(10);
	}
}

static unsigned char ai_gc9106_read_byte(struct gc9106_platform_data *data )
{
	int i;
	unsigned char rev = 0;
	gpio_direction_input(data->sda_gpio);
	SPIDelay;
	for(i = 0; i < 8; i++) {
		gpio_direction_output(data->scl_gpio, 0);
		SPIDelay;
		gpio_direction_output(data->scl_gpio, 1);
		rev = rev << 1;
		rev |= gpio_get_value(data->sda_gpio);
		SPIDelay;
	}
	pr_info("%s rev:0x%x \n", __func__, rev);
	return rev;
}

/*
  read register value by gpio mode
*/
static unsigned char ai_gc9106_read_reg(struct gc9106_platform_data *data , unsigned char reg)
{
	unsigned char rev = 0;

	ai_gc9106_cs_is_enable(data, 1);

	gpio_direction_output(data->dcx_gpio, 0);
	ai_gc9106_write_byte(data, reg);
	rev = ai_gc9106_read_byte(data);

	ai_gc9106_cs_is_enable(data, 0);

	pr_info("%s rev:0x%x \n", __func__, rev);
	return rev;
}
/*
  write data by gpio mode
*/
static void ai_gc9106_write_data_byte(struct gc9106_platform_data *data, unsigned char byte)
{
	ai_gc9106_cs_is_enable(data, 1);
	/*when dcx pin is high, is a data type*/
	gpio_direction_output(data->dcx_gpio, 1);
	ai_gc9106_write_byte(data, byte);
	ai_gc9106_cs_is_enable(data, 0);
}
/*
  send command by gpio mode
*/
static void ai_gc9106_write_cmd_byte(struct gc9106_platform_data *data, unsigned char byte) {
	ai_gc9106_cs_is_enable(data, 1);
	/*when dcx pin is high, is a command type*/
	gpio_direction_output(data->dcx_gpio, 0);
	ai_gc9106_write_byte(data, byte);
	ai_gc9106_cs_is_enable(data, 0);
}

static void ai_gc9106_write_data_simulate(struct gc9106_platform_data *data, unsigned char* p_data, int len)
{
	int i;
	ai_gc9106_cs_is_enable(data, 1);
	gpio_direction_output(data->dcx_gpio, 1);
	udelay(2);
	for(i = 0; i < len; i++) {
		ai_gc9106_write_byte(data, p_data[i]);
	}
	udelay(2);
	ai_gc9106_cs_is_enable(data, 0);
}

static int ai_gc9106_read_id(void)
{
	int ret = 0;
	unsigned char id[2] = {0,0};

	id[0] = ai_gc9106_read_reg(&g_gc9106_data, 0xdb);
	id[1] = ai_gc9106_read_reg(&g_gc9106_data, 0xdc);

	ret = (id[0] << 8) + id[1];
	pr_info("%s, id[0]:0x%02x id[1]:0x%02x \n", __func__, id[0], id[1]);
	return ret;
}
void ai_gc9106_init_lcd(void)
{
	int i;
	//int j;

    //Antaiui <AI_BSP_LCD> <zhuli> <2022-07-06> add for sub lcd Reentrancy begin
    if(g_gc9106_data.is_init == 1){
        pr_info("sublcd %s \n", __func__);
        return;
    }
    //Antaiui <AI_BSP_LCD> <zhuli> <2022-07-06> add for sub lcd Reentrancy end

	//-------- Reset Sequence-----//
	pr_info("%s, enter \n", __func__);
	gpio_direction_output(g_gc9106_data.rst_gpio, 1); //LCD_nRST = 1;
	mdelay(50); //delayms(50);
	gpio_direction_output(g_gc9106_data.rst_gpio, 0);  //LCD_nRST = 0;
	mdelay(50);
	gpio_direction_output(g_gc9106_data.rst_gpio, 1); //LCD_nRST = 1;
	mdelay(120); //delayms(120);
	//------end Reset Sequence-----//
    //Init_codeStart
#if 1
	for(i = 0; i < ARRAY_SIZE(ai_gc9106_init_parm); i++) {
		if(ai_gc9106_init_parm[i].type == CMD_DELAY) {
			mdelay(ai_gc9106_init_parm[i].num);
			continue;
		} else {
			lcm_spi_interface(ai_gc9106_init_parm[i].data, ai_gc9106_init_parm[i].num, ai_gc9106_init_parm[i].type);
		}
	}
	pr_info("%s, i = %d \n", __func__);
#else
    LCD_CtrlWrite(0xfe);
    LCD_CtrlWrite(0xfe);
    LCD_CtrlWrite(0xef);

    LCD_CtrlWrite(0xb3);
    LCD_DataWrite(0x03);

    LCD_CtrlWrite(0x36);
    LCD_DataWrite(0xd8);

    LCD_CtrlWrite(0x3a);
    LCD_DataWrite(0x05);

    LCD_CtrlWrite(0xb6);
    LCD_DataWrite(0x11);
    LCD_CtrlWrite(0xac);
    LCD_DataWrite(0x0b);

    LCD_CtrlWrite(0xb4);
    LCD_DataWrite(0x21);

    LCD_CtrlWrite(0xb1);
    LCD_DataWrite(0xc0);

    LCD_CtrlWrite(0xe6);
    LCD_DataWrite(0x50);
    LCD_DataWrite(0x43);
    LCD_CtrlWrite(0xe7);
    LCD_DataWrite(0x56);
    LCD_DataWrite(0x43);

    LCD_CtrlWrite(0xF0);
    LCD_DataWrite(0x1f);
    LCD_DataWrite(0x41);
    LCD_DataWrite(0x1B);
    LCD_DataWrite(0x55);
    LCD_DataWrite(0x36);
    LCD_DataWrite(0x3d);
    LCD_DataWrite(0x3e);
    LCD_DataWrite(0x0);
    LCD_DataWrite(0x16);
    LCD_DataWrite(0x08);
    LCD_DataWrite(0x09);
    LCD_DataWrite(0x15);
    LCD_DataWrite(0x14);
    LCD_DataWrite(0xf);

    LCD_CtrlWrite(0xF1);

    LCD_DataWrite(0x1f);
    LCD_DataWrite(0x41);
    LCD_DataWrite(0x1B);
    LCD_DataWrite(0x55);
    LCD_DataWrite(0x36);
    LCD_DataWrite(0x3d);
    LCD_DataWrite(0x3e);
    LCD_DataWrite(0x0);
    LCD_DataWrite(0x16);
    LCD_DataWrite(0x08);
    LCD_DataWrite(0x09);
    LCD_DataWrite(0x15);
    LCD_DataWrite(0x14);
    LCD_DataWrite(0xf);


    LCD_CtrlWrite(0xfe);
    LCD_CtrlWrite(0xff);

	LCD_CtrlWrite(0x35);
	LCD_DataWrite(0x00);
	LCD_CtrlWrite(0x44);
	LCD_DataWrite(0x00);
    LCD_CtrlWrite(0x11);
	mdelay(120);
    LCD_CtrlWrite(0x29);
	LCD_CtrlWrite(0x2a);
	LCD_DataWrite(0x00);
	LCD_DataWrite(0x18);
	LCD_DataWrite(0x00);
	LCD_DataWrite(0x67);
	LCD_CtrlWrite(0x2c);
#endif
	g_gc9106_data.is_init = 1;
	g_gc9106_data.is_suspend = 0;
}

void ai_gc9106_enter_sleep(void)
{
#if 1
	int i;

    //Antaiui <AI_BSP_LCD> <zhuli> <2022-07-06> add for sub lcd Reentrancy begin
	pr_info("sublcd %s, line = %d, suspend = %d \n", __func__, __LINE__, g_gc9106_data.is_suspend);
    if(g_gc9106_data.is_suspend == 1){
        pr_info("sublcd %s \n", __func__);
        return;
    }

	mutex_lock(&wake_lock);
    //Antaiui <AI_BSP_LCD> <zhuli> <2022-07-06> add for sub lcd Reentrancy end

	for(i = 0; i < ARRAY_SIZE(ai_gc9106_suspend_parm); i++) {
		if(ai_gc9106_suspend_parm[i].type == CMD_DELAY) {
			mdelay(ai_gc9106_suspend_parm[i].num);
			continue;
		} else {
			lcm_spi_interface(ai_gc9106_suspend_parm[i].data, ai_gc9106_suspend_parm[i].num, ai_gc9106_suspend_parm[i].type);
		}
	}
	g_gc9106_data.is_suspend = 1;
	pr_info("sublcd %s, i = %d, line = %d, suspend = %d \n", __func__, i, __LINE__, g_gc9106_data.is_suspend);
    mutex_unlock(&wake_lock);
#else
	LCD_CtrlWrite(0xfe);
	LCD_CtrlWrite(0xef);
	LCD_CtrlWrite(0x28);
	mdelay(120);
	LCD_CtrlWrite(0x10);
	mdelay(50);
#endif
}
void ai_gc9106_exit_sleep(void)
{
#if 1
	int i;

    //Antaiui <AI_BSP_LCD> <zhuli> <2022-07-06> add for sub lcd Reentrancy begin
	pr_info("sublcd %s, line = %d, suspend = %d \n", __func__, __LINE__, g_gc9106_data.is_suspend);
    if(g_gc9106_data.is_suspend == 0){
        pr_info("sublcd %s \n", __func__);
        return;
    }

	mutex_lock(&wake_lock);
    //Antaiui <AI_BSP_LCD> <zhuli> <2022-07-06> add for sub lcd Reentrancy end

	for(i = 0; i < ARRAY_SIZE(ai_gc9106_resume_parm); i++) {
		if(ai_gc9106_resume_parm[i].type == CMD_DELAY) {
			mdelay(ai_gc9106_resume_parm[i].num);
			continue;
		} else {
			lcm_spi_interface(ai_gc9106_resume_parm[i].data, ai_gc9106_resume_parm[i].num, ai_gc9106_resume_parm[i].type);
		}
	}
	g_gc9106_data.is_suspend = 0;
	pr_info("sublcd %s, i = %d, line = %d, suspend = %d \n", __func__, i, __LINE__, g_gc9106_data.is_suspend);
    mutex_unlock(&wake_lock);
#else
	LCD_CtrlWrite(0xfe);
	LCD_CtrlWrite(0xef);
	LCD_CtrlWrite(0x11);
	mdelay(120) ;
	LCD_CtrlWrite(0x29);
#endif
}
static void ai_gc9106_display_work(struct work_struct *work)
{
	static int disp_color = 0;
	int i = 0;

	pr_info("%s enter  disp_color:%d \n",__func__, disp_color);
    //Antaiui <AI_BSP_LCD> <zhuli> <2022-07-06> cancel sub lcd display time when mmi test begin
    cancel_display_work();
    //Antaiui <AI_BSP_LCD> <zhuli> <2022-07-06> cancel sub lcd display time when mmi test end

	if(g_gc9106_data.is_init != 1) {
		ai_gc9106_init_lcd();
	}

    //Antaiui <AI_BSP_LCD> <zhuli> <2022-07-13> sub lcd wake up when sleep in hardware test begin
    if(g_gc9106_data.is_suspend == 1) {
        ai_gc9106_exit_sleep();
    }
    //Antaiui <AI_BSP_LCD> <zhuli> <2022-07-13> sub lcd wake up when sleep in hardware test end

	switch(disp_color) {
		case 0:   //black
			memset(g_gc9106_data.d_buf, 0x00, DISPLAY_BUFSIZE);
			lcm_spi_interface(g_gc9106_data.d_buf,DISPLAY_BUFSIZE, 1);
			disp_color = 1;
			break;
		case 1: //red
			while(i < DISPLAY_BUFSIZE) {
				g_gc9106_data.d_buf[i] = 0xf8;
				g_gc9106_data.d_buf[i+1] = 0x00;
				i=i+2;
			}
			lcm_spi_interface(g_gc9106_data.d_buf,DISPLAY_BUFSIZE, 1);
			disp_color = 2;
			break;
		case 2: //green
			while(i < DISPLAY_BUFSIZE) {
				g_gc9106_data.d_buf[i] = 0x07;
				g_gc9106_data.d_buf[i+1] = 0xe0;
				i=i+2;
			}
			lcm_spi_interface(g_gc9106_data.d_buf,DISPLAY_BUFSIZE, 1);
			disp_color = 3;
			break;
		case 3: //blue
			while(i < DISPLAY_BUFSIZE) {
				g_gc9106_data.d_buf[i] = 0x00;
				g_gc9106_data.d_buf[i+1] = 0x1f;
				i=i+2;
			}
			lcm_spi_interface(g_gc9106_data.d_buf,DISPLAY_BUFSIZE, 1);
			disp_color = 4;
			break;
		case 4: //white
			memset(g_gc9106_data.d_buf, 0xff, DISPLAY_BUFSIZE);
			lcm_spi_interface(g_gc9106_data.d_buf,DISPLAY_BUFSIZE, 1);
			disp_color = 0;
			break;
		default:
			break;
	}

	schedule_delayed_work(&(g_gc9106_data.dwork), msecs_to_jiffies(1000));
}

static void ai_gc9106_display_test(struct gc9106_platform_data* data, int type)
{
	int i = 0;
	pr_info("%s enter : type:%d\n", __func__, type);
	switch(type){
		case 1:
			while(i < DISPLAY_BUFSIZE) {
				g_gc9106_data.d_buf[i] = 0xf8;
				g_gc9106_data.d_buf[i+1] = 0x00;
				i=i+2;
			}
			lcm_spi_interface(g_gc9106_data.d_buf,DISPLAY_BUFSIZE, 1);
			break;
		case 2:
			while(i < DISPLAY_BUFSIZE) {
				g_gc9106_data.d_buf[i] = 0x07;
				g_gc9106_data.d_buf[i+1] = 0xe0;
				i=i+2;
			}
			lcm_spi_interface(g_gc9106_data.d_buf,DISPLAY_BUFSIZE, 1);
			break;
		case 3:
			while(i < DISPLAY_BUFSIZE) {
				g_gc9106_data.d_buf[i] = 0x00;
				g_gc9106_data.d_buf[i+1] = 0x1f;
				i=i+2;
			}
			lcm_spi_interface(g_gc9106_data.d_buf,DISPLAY_BUFSIZE, 1);
			break;
		case 4:
			memset(g_gc9106_data.d_buf, 0xff, DISPLAY_BUFSIZE);
			lcm_spi_interface(g_gc9106_data.d_buf,DISPLAY_BUFSIZE, 1);
			break;
		default:
			break;
	}
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//AW36515 Debug file
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static ssize_t gc9106_rgb_test_get(struct device* cd,struct device_attribute *attr, char* buf);
static ssize_t gc9106_rgb_test_set(struct device* cd, struct device_attribute *attr, const char* buf, size_t len);
static ssize_t gc9106_mmi_test_set(struct device* cd, struct device_attribute *attr, const char* buf, size_t len);
static ssize_t gc9106_lcd_id_get(struct device* cd,struct device_attribute *attr, char* buf);

static DEVICE_ATTR(rgb_test, 0660, gc9106_rgb_test_get,  gc9106_rgb_test_set);
static DEVICE_ATTR(mmi_test, 0660, NULL,  gc9106_mmi_test_set);
static DEVICE_ATTR(lcd_id, 0660, gc9106_lcd_id_get,  NULL);

static ssize_t gc9106_rgb_test_get(struct device* cd,struct device_attribute *attr, char* buf)
{

	ssize_t len = 0;
	len = snprintf(buf, PAGE_SIZE-len, "ldo:%d rst:%d cs:%d scl:%d sda:%d dcx:%d\n",
				gpio_get_value(g_gc9106_data.ldo_gpio),gpio_get_value(g_gc9106_data.rst_gpio),gpio_get_value(g_gc9106_data.cs_gpio),
				gpio_get_value(g_gc9106_data.scl_gpio),gpio_get_value(g_gc9106_data.sda_gpio),gpio_get_value(g_gc9106_data.dcx_gpio));
	len += snprintf(buf+len, PAGE_SIZE-len, "\r\n");
	return len;
}

static ssize_t gc9106_rgb_test_set(struct device* cd, struct device_attribute *attr, const char* buf, size_t len)
{
	int ret = 0;
	int cmd = 0;

	ret = kstrtos32(buf,10, &cmd);
	pr_info("%s , cmd:%d ; ret : %d\n",__func__, cmd, ret);

	switch(cmd) {
		case 0:
			if(g_gc9106_data.is_init == 0)
				ai_gc9106_init_lcd();
			break;
		case 1:  //red
		case 2:  //green
		case 3:  //blue
		case 4:  //white
			ai_gc9106_display_test(&g_gc9106_data, cmd);
			break;
		case 5:
			pr_info("%s, gpio test 5\n");
			gpio_direction_output(g_gc9106_data.ldo_gpio, 0);
			gpio_direction_output(g_gc9106_data.rst_gpio, 0);
			gpio_direction_output(g_gc9106_data.cs_gpio,1);
			gpio_direction_output(g_gc9106_data.scl_gpio,1);
			gpio_direction_output(g_gc9106_data.sda_gpio,1);
			gpio_direction_output(g_gc9106_data.dcx_gpio,1);
			g_gc9106_data.is_init = 0;
			break;
		case 6:
			pr_info("%s, gpio test 6\n");
			gpio_direction_output(g_gc9106_data.ldo_gpio, 1);
			gpio_direction_output(g_gc9106_data.rst_gpio, 1);
			gpio_direction_output(g_gc9106_data.cs_gpio,1);
			gpio_direction_output(g_gc9106_data.scl_gpio,0);
			gpio_direction_output(g_gc9106_data.sda_gpio,0);
			gpio_direction_output(g_gc9106_data.dcx_gpio,0);
			break;
		case 7:
			pr_info("%s, suspend test \n");
			if(g_gc9106_data.is_init == 1) {
				cancel_delayed_work(&(g_gc9106_data.dwork));
				ai_gc9106_enter_sleep();
			}
			break;
		case 8:
			pr_info("%s, resume test \n");
			if(g_gc9106_data.is_suspend == 1) {
				ai_gc9106_exit_sleep();
			}
			break;
		default:
			pr_err("%s, invalid command\n ", __func__);
			break;
	}
	return len;
}

static ssize_t gc9106_mmi_test_set(struct device* cd, struct device_attribute *attr, const char* buf, size_t len)
{
    int ret = 0;
    int cmd = 0;

    ret = kstrtos32(buf,10, &cmd);
    pr_info("sublcd func:%s, line:%d, cmd:%d, ret: %d\n",__func__, __LINE__, cmd, ret);
    if(cmd == 1) {
        //Antaiui <AI_BSP_LCD> <zhuli> <2022-07-19> add for sub lcd mmi test begin
        g_gc9106_data.is_testing = true;
        //Antaiui <AI_BSP_LCD> <zhuli> <2022-07-19> add for sub lcd mmi test end

        if(g_gc9106_data.is_init == 0) {
            ai_gc9106_init_lcd();
        }else if(g_gc9106_data.is_suspend == 1) {
            ai_gc9106_exit_sleep();
        }
        //Antaiui <AI_BSP_LCD> <zhuli> <2022-07-08> cancel sub lcd time display in hardware test begin
        cancel_display_work();
        //Antaiui <AI_BSP_LCD> <zhuli> <2022-07-08> cancel sub lcd time display in hardware test end
        schedule_delayed_work(&(g_gc9106_data.dwork), 0);
    //Antaiui <AI_BSP_LCD> <zhuli> <2022-07-04> add for sub lcd test in factory test begin
    }else if(cmd == 2){
        int i = 0;

        //Antaiui <AI_BSP_LCD> <zhuli> <2022-07-19> add for sub lcd mmi test begin
        g_gc9106_data.is_testing = true;
        //Antaiui <AI_BSP_LCD> <zhuli> <2022-07-19> add for sub lcd mmi test end

        if(g_gc9106_data.is_init == 0) {
            ai_gc9106_init_lcd();
        }else if(g_gc9106_data.is_suspend == 1) {
            ai_gc9106_exit_sleep();
        }
        cancel_display_work();

        while(i < DISPLAY_BUFSIZE) {
            g_gc9106_data.d_buf[i] = 0x00;
            g_gc9106_data.d_buf[i+1] = 0x1f;
            i=i+2;
        }
        lcm_spi_interface(g_gc9106_data.d_buf,DISPLAY_BUFSIZE, 1);

        pr_info("%s,  ret: %d \n",__func__,  ret);
    //Antaiui <AI_BSP_LCD> <zhuli> <2022-07-04> add for sub lcd test in factory test end
    }else{
	if(g_gc9106_data.is_init == 1) {
	    cancel_delayed_work(&(g_gc9106_data.dwork));
	    ai_gc9106_enter_sleep();
            //Antaiui <AI_BSP_LCD> <zhuli> <2022-07-18> add for sub lcd test begin
            g_gc9106_data.is_testing = false;
            //Antaiui <AI_BSP_LCD> <zhuli> <2022-07-18> add for sub lcd test end
	}
    }
    return len;
}

static ssize_t gc9106_lcd_id_get(struct device* cd,struct device_attribute *attr, char* buf)
{
	ssize_t len = 0;
    //Antaiui <AI_BSP_LCD> <zhuli> <2022-07-05> add for sub lcd test in factory test begin
    lcm_spi_interface(0, 0, WRITE_CMD);
    g_gc9106_data.lcd_id = ai_gc9106_read_id();
	len = snprintf(buf, PAGE_SIZE-len, "%02x%02x\n", (g_gc9106_data.lcd_id & 0xff00) >> 8, g_gc9106_data.lcd_id & 0x00ff);
	return len;
    //Antaiui <AI_BSP_LCD> <zhuli> <2022-07-05> add for sub lcd test in factory test end
}

static void gc9106_create_sysfs(struct platform_device *dev)
{
	//struct device *dev = &(dev->dev);
	device_create_file(&(dev->dev), &dev_attr_rgb_test);
	device_create_file(&(dev->dev), &dev_attr_mmi_test);
	device_create_file(&(dev->dev), &dev_attr_lcd_id);
}

static int gc9106_parse_dt(struct device *dev, struct gc9106_platform_data *data)
 {
      int ret = 0;
      struct device_node *np = dev->of_node;

      data->rst_gpio = of_get_named_gpio(np, "gpio_rst", 0);
      if ((!gpio_is_valid(data->rst_gpio)))
              return -EINVAL;

      data->ldo_gpio = of_get_named_gpio(np, "gpio_ldo", 0);
      if ((!gpio_is_valid(data->ldo_gpio)))
              return -EINVAL;

      data->scl_gpio = of_get_named_gpio(np, "gpio_scl", 0);
      if ((!gpio_is_valid(data->scl_gpio)))
              return -EINVAL;

      //required for old platform only
      data->cs_gpio = of_get_named_gpio(np, "gpio_cs", 0);
      if ((!gpio_is_valid(data->cs_gpio)))
              return -EINVAL;

	  data->sda_gpio = of_get_named_gpio(np, "gpio_dcx", 0);
      if ((!gpio_is_valid(data->sda_gpio)))
              return -EINVAL;

      data->dcx_gpio = of_get_named_gpio(np, "gpio_sda", 0);
      if ((!gpio_is_valid(data->dcx_gpio)))
              return -EINVAL;

      pr_info("%s: %d, %d, %d, %d, %d, %d error:%d\n", __func__,
                  data->rst_gpio, data->ldo_gpio, data->scl_gpio,
                  data->cs_gpio, data->sda_gpio, data->dcx_gpio, ret);
      return ret;
  }

static int ai_gc9106_gpio_init(struct gc9106_platform_data *data)
{
	int ret = 0;
	pr_info("%s enter\n", __func__);

	if(gpio_is_valid(data->rst_gpio)) {
		ret = gpio_request(data->rst_gpio, "gc9106_rst_gpio");
		if(ret) {
			pr_err("%s, unable request gc9106 rst gpio [%d] \n", __func__, data->rst_gpio);
		}else {
			ret = gpio_direction_output(data->rst_gpio, 1);
			if(ret) {
				pr_err("%s, can not set direction output of gc9106 ldo gpio [%d]\n",__func__,ret);
			}
		}
	}

	if(gpio_is_valid(data->cs_gpio)) {
		ret = gpio_request(data->cs_gpio, "gc9106_cs_gpio");
		if(ret) {
			pr_err("%s, unable request gc9106 cs gpio [%d] \n", __func__, data->cs_gpio);
		}else {
			ret = gpio_direction_output(data->cs_gpio, 1);
			if(ret) {
				pr_err("%s, can not set direction output of gc9106 ldo gpio [%d]\n",__func__,ret);
			}
			gpio_direction_input(data->cs_gpio);
			mdelay(5);
			pr_info("%s, cs_gpio:%d\n", __func__, gpio_get_value(data->cs_gpio));
		}
	}

	if(gpio_is_valid(data->ldo_gpio)) {
		ret = gpio_request(data->ldo_gpio, "gc9106_ldo_gpio");
		if(ret) {
			pr_err("%s, unable request gc9106 ldo gpio [%d] \n", __func__, data->ldo_gpio);
		}else {
			ret = gpio_direction_output(data->ldo_gpio, 1);
			if(ret) {
				pr_err("%s, can not set direction output of gc9106 ldo gpio [%d]\n",__func__,ret);
			}
		}
	}

	if(gpio_is_valid(data->scl_gpio)) {
		ret = gpio_request(data->scl_gpio, "gc9106_scl_gpio");
		if(ret) {
			pr_err("%s, unable request gc9106 scl gpio [%d] \n", __func__, data->scl_gpio);
		}
	}



	if(gpio_is_valid(data->sda_gpio)) {
		ret = gpio_request(data->sda_gpio, "gc9106_sda_gpio");
		if(ret) {
			pr_err("%s, unable request gc9106 sda gpio [%d] \n", __func__, data->sda_gpio);
		}
	}

	if(gpio_is_valid(data->dcx_gpio)) {
		ret = gpio_request(data->dcx_gpio, "gc9106_dcx_gpio");
		if(ret) {
			pr_err("%s, unable request gc9106 dcx gpio [%d] \n", __func__, data->dcx_gpio);
		}
	}

	return ret;

}
static int ai_gc9106_probe(struct platform_device *dev)
{
	int ret;
	struct gc9106_platform_data *data = &g_gc9106_data;

	pr_info("%s start.\n", __func__);

	data->d_buf = (unsigned char*) kzalloc(DISPLAY_BUFSIZE,GFP_KERNEL);
	if(data->d_buf == NULL) {
		pr_err("%s, kzalloc display buffer error \n", __func__);
		return 0;
	}

	ret = gc9106_parse_dt(&(dev->dev), data);
	if(ret < 0) {
		pr_info("%s: %d, %d, %d, %d, %d, %d error:%d\n", __func__,
                  data->rst_gpio, data->ldo_gpio, data->scl_gpio,
                  data->cs_gpio, data->sda_gpio, data->dcx_gpio, ret);
	}

	ret = ai_gc9106_gpio_init(data);
	g_gc9106_data.is_init = 0;

	gc9106_create_sysfs(dev);
	gc9106_display_create_sysfs(dev);

	INIT_DELAYED_WORK(&(g_gc9106_data.dwork),ai_gc9106_display_work);
	//schedule_delayed_work(&(g_gc9106_data.dwork), msecs_to_jiffies(5000));
	g_gc9106_data.lcd_id = ai_gc9106_read_id();
	pr_info("%s done.\n", __func__);

	return 0;
}
static int ai_gc9106_remove(struct platform_device *dev)
{
	pr_info("Remove start.\n");

	platform_driver_unregister(&ai_gc9106_platform_driver);

	pr_info("Remove done.\n");

	return 0;
}


void ai_sublcd_gc9106_frame_interface(unsigned char *sbuf)
{
    pr_info("sublcd %s, testing = %d \n", __func__, g_gc9106_data.is_testing);

    //Antaiui <AI_BSP_LCD> <zhuli> <2022-07-18> add for sub lcd test begin
    if(g_gc9106_data.is_testing)
    {
        return;
    }
    //Antaiui <AI_BSP_LCD> <zhuli> <2022-07-18> add for sub lcd test end

    //Antaiui <AI_BSP_LCD> <zhuli> <2022-07-08> cancel sub lcd time display in hardware test begin
    if(g_gc9106_data.is_suspend != 1)
        cancel_delayed_work(&(g_gc9106_data.dwork));
    //Antaiui <AI_BSP_LCD> <zhuli> <2022-07-08> cancel sub lcd time display in hardware test end

    //Antaiui <AI_BSP_LCD> <zhuli> <2022-06-16> add for sub lcd mmi display begin
    if(g_gc9106_data.is_init != 1) {
        ai_gc9106_init_lcd();
    }
    //Antaiui <AI_BSP_LCD> <zhuli> <2022-06-16> add for sub lcd mmi display end
    //Antaiui <AI_BSP_LCD> <zhuli> <2022-06-28> add for sub lcd display after mmi test begin
    if(g_gc9106_data.is_suspend == 1) {
        ai_gc9106_exit_sleep();
    }

    //Antaiui <AI_BSP_LCD> <zhuli> <2022-06-28> add for sub lcd display after mmi test end
    lcm_spi_interface(sbuf, DISPLAY_BUFSIZE, 1);
}

//Antaiui <AI_BSP_LCD> <zhuli> <2022-06-28> add for sub lcd sleep interface begin
void ai_sublcd_gc9106_sleep_interface(bool sleep)
{
    pr_info("sublcd %s, sleep = %d, testing = %d \n", __func__, sleep, g_gc9106_data.is_testing);

    //Antaiui <AI_BSP_LCD> <zhuli> <2022-07-18> add for sub lcd test begin
    if(g_gc9106_data.is_testing)
    {
        return;
    }
    //Antaiui <AI_BSP_LCD> <zhuli> <2022-07-18> add for sub lcd test end
    //Antaiui <AI_BSP_LCD> <zhuli> <2022-07-08> cancel sub lcd time display in hardware test begin
    if(g_gc9106_data.is_suspend != 1)
        cancel_delayed_work(&(g_gc9106_data.dwork));
    //Antaiui <AI_BSP_LCD> <zhuli> <2022-07-08> cancel sub lcd time display in hardware test end

    if(!sleep) {
        if(g_gc9106_data.is_init == 0) {
            ai_gc9106_init_lcd();
        }
        if(g_gc9106_data.is_suspend == 1) {
            ai_gc9106_exit_sleep();
        }
    }else {
        if(g_gc9106_data.is_init == 1) {
            ai_gc9106_enter_sleep();
        }
    }
}

EXPORT_SYMBOL(ai_sublcd_gc9106_sleep_interface);
//Antaiui <AI_BSP_LCD> <zhuli> <2022-06-28> add for sub lcd sleep interface end
EXPORT_SYMBOL(ai_sublcd_gc9106_frame_interface);


static int __init ai_sublcd_gc9106_init(void)
{
	int ret;

	pr_info("%s start.\n", __func__);

	ret = platform_driver_register(&ai_gc9106_platform_driver);
	if (ret) {
		pr_info("Failed to register platform driver\n");
		return ret;
	}
    ai_sublcd_display_init();

	pr_info("Init done.\n");

	return 0;
}

static void __exit ai_sublcd_gc9106_exit(void)
{
	pr_info("Exit start.\n");

	platform_driver_unregister(&ai_gc9106_platform_driver);
    ai_sublcd_display_exit();

	pr_info("Exit done.\n");
}

module_init(ai_sublcd_gc9106_init);
module_exit(ai_sublcd_gc9106_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Yuechun Yao<yaoyc@ant-ai.cn>");
MODULE_DESCRIPTION("Ant gc9106 Driver");
