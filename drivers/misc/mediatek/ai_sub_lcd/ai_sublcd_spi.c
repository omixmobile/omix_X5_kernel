#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/ioport.h>
#include <linux/errno.h>
#include <linux/spi/spi.h>
#include <linux/workqueue.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/spi/spidev.h>
#include <linux/semaphore.h>
#include <linux/poll.h>
#include <linux/fcntl.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/signal.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/gpio.h>
//#include <DpDataType.h>

//#include <lcm_spi.h>
#include "ai_sublcd_spi.h"

//#include "ddp_info.h"

//#include "mtkfb_info.h"


//by rober
static struct of_device_id silead_of_match[] = {
	{ .compatible = "mediatek,lcm-spi-bus", },
	{}
};

static int lcm_suspend(struct device *dev, pm_message_t state);
static int lcm_resume(struct device *dev);
static int lcm_probe(struct spi_device *spi);


//static SIMPLE_DEV_PM_OPS(pm_ops, lcm_suspend, lcm_resume);
static struct spi_driver lcm_driver = {
	.driver = {
		.name	= LCM_SPI_NAME,
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	#ifdef CONFIG_PM
		.suspend = lcm_suspend,
		.resume  = lcm_resume,
	#endif
		.of_match_table = silead_of_match,
	},
	.probe	 = lcm_probe,
};

typedef enum DTS_STATE {
    STATE_CS_CLR1,
    STATE_CS_CLR0,
    STATE_CS_SET,
    STATE_MI_CLR1,
    STATE_MI_CLR0,
    STATE_MI_SET,
    STATE_MO_CLR0,
    STATE_MO_CLR1,
    STATE_MO_SET,
    STATE_CLK_CLR0,
    STATE_CLK_CLR1,
    STATE_CLK_SET,
    STATE_RST_CLR0,
    STATE_RST_CLR1,
    STATE_A0_CLR0,
    STATE_A0_CLR1,
    STATE_MAX,  /* array size */
} dts_status_t;
static struct pinctrl* this_pctrl; /* static pinctrl instance */
/* DTS state mapping name */
static const char* dts_state_name[STATE_MAX] = {
	"spi_cs_clr1",
	"spi_cs_clr0",
	"spi_cs_set",
	"spi_mi_clr1",
	"spi_mi_clr0",
	"spi_mi_set",
	"spi_mo_clr0",
	"spi_mo_clr1",
	"spi_mo_set",
	"spi_mclk_clr0",
	"spi_mclk_clr1",
	"spi_mclk_set",
	"lcm_rst_clr0",
	"lcm_rst_clr1",
	"lcm_a0_clr0",
	"lcm_a0_clr1"
};


struct pinctrl_state* pinctrl_state_name[STATE_MAX];


/* pinctrl implementation */
inline static long dts_set_state(dts_status_t s)
{
    long ret = 0;
    BUG_ON(!this_pctrl);

    ret = pinctrl_select_state(this_pctrl, pinctrl_state_name[s]);

    if ( ret < 0) {
        printk("lcm_spi -------- pinctrl_select_state %s failed\n", dts_state_name[s]);
    }
	printk("lcm_spi -------- pinctrl_select_state %s success\n", dts_state_name[s]);
    return ret;
}

inline static long dts_select_state(dts_status_t s)
{
	FUNC_ENTRY();
    BUG_ON(!((unsigned int)(s) < (unsigned int)(STATE_MAX)));
    return dts_set_state(s);
}

inline static long dts_pin_out_state(int pin_num, int level)
{
	FUNC_ENTRY();
	
	BUG_ON(!(pin_num < (unsigned int)(STATE_MAX)));
	switch (pin_num)
	{
		case LCM_SPI_SCK_PIN: // GPIO_SPI_SCK_PIN
			if (level == GPIO_OUT_ONE)
				dts_select_state(STATE_CLK_CLR1);
			else
				dts_select_state(STATE_CLK_CLR0);
			break;		
		case LCM_SPI_CS_PIN: // GPIO_SPI_CS_PIN
			if (level == GPIO_OUT_ONE)
				dts_select_state(STATE_CS_CLR1);
			else
				dts_select_state(STATE_CS_CLR0);
			break;
		case LCM_SPI_MOSI_PIN:// GPIO_SPI_MOSI_PIN
			if (level == GPIO_OUT_ONE)
				dts_select_state(STATE_MO_CLR1);
			else
				dts_select_state(STATE_MO_CLR0);
			break;
		case GPIO_SPI_MISO_PIN:// GPIO_SPI_RS_PIN
			if (level == GPIO_OUT_ONE)
				dts_select_state(STATE_MI_CLR1);
			else
				dts_select_state(STATE_MI_CLR0);
			break;
		case GPIO_SPI_RST_PIN:// GPIO_SPI_RS_PIN
			if (level == GPIO_OUT_ONE)
				dts_select_state(STATE_RST_CLR1);
			else
				dts_select_state(STATE_RST_CLR0);
			break;

		default:
			return -1;
	}
	return 0;
}

// Configure IO to make it work in SPI mode	
static void set_pin_mode(int enable)
{
   //FUNC_ENTRY();
   if(enable == SPI_PIN_MODE)
	{

		dts_select_state(STATE_CS_SET);
		dts_select_state(STATE_CLK_SET);
		dts_select_state(STATE_MO_SET);
		//dts_select_state(STATE_MI_SET);
	}
	else{

		dts_select_state(STATE_CS_CLR1);
		dts_select_state(STATE_CLK_CLR0);
		dts_select_state(STATE_MO_CLR0);
		dts_select_state(STATE_MI_CLR0);
	}
	
  //FUNC_EXIT();
}

// SPI GPIO mode
static void lcm_gpio_send(u8 data)
{
    unsigned int i;
	
	for (i = 0; i <8; i++)
	{
		if (data & (0x80)) {
			SET_GPIO_OUT(LCM_SPI_MOSI_PIN,GPIO_OUT_ONE);
		} else {
			SET_GPIO_OUT(LCM_SPI_MOSI_PIN,GPIO_OUT_ZERO);
		}
		SET_GPIO_OUT(LCM_SPI_SCK_PIN,GPIO_OUT_ZERO);
		SET_GPIO_OUT(LCM_SPI_SCK_PIN,GPIO_OUT_ONE);
		data <<= 1;
	}
	
	return ;
}

static void lcm_gpio_recive(u8 *sbuf, size_t spilen)
{	
	return;
//    // read functin is not implemented
//    unsigned int i,j;
//	unsigned char cmd;
//	
//	cmd=sbuf[0];
//	sbuf[0]=0;
//	
//    udelay(1);
//
//    for (i = 0; i < 8; ++ i)
//    {   
//        if (cmd & 0x80) {
//            SET_GPIO_OUT(LCM_SPI_MOSI_PIN,GPIO_OUT_ONE);
//        } else {
//            SET_GPIO_OUT(LCM_SPI_MOSI_PIN,GPIO_OUT_ZERO);
//        }
//		SET_GPIO_OUT(GPIO_SPI_SCK_PIN,GPIO_OUT_ZERO);
//        SET_GPIO_OUT(GPIO_SPI_SCK_PIN,GPIO_OUT_ONE);
//        cmd <<= 1;
//    }
//
//    SET_GPIO_DIR(LCM_SPI_MOSI_PIN,GPIO_DIR_IN);
//   
//	for(i=0;i<spilen;i++)
//	{
//		for(j=0;j<8;j++) // 8 Data
//		{		
//		  SET_GPIO_OUT(GPIO_SPI_SCK_PIN,GPIO_OUT_ZERO);
//		  udelay(10);
//		  SET_GPIO_OUT(GPIO_SPI_SCK_PIN,GPIO_OUT_ONE);
//		  sbuf[i]<<= 1;
//		  sbuf[i] |= GET_GPIO_VALUE(LCM_SPI_MOSI_PIN);	 
//		  udelay(10); 
//		 }
//	}
//    
//	SET_GPIO_DIR(LCM_SPI_MOSI_PIN,GPIO_DIR_OUT);
//	
//	return;

}


// SPI DMA AND FIFO mode
static int lcm_spi_send(u8 *tx,size_t spilen)
{
    int ret=0;
	struct spi_message m;
	
	struct spi_transfer t;


//    struct spi_transfer t = {
//		.cs_change = 0,
//		.delay_usecs = 0,
//		.speed_hz = LCM_SPI_CLOCK_SPEED,
//		.tx_buf = tx,
//		//.rx_buf = g_get_data,
//		.len = spilen,
//		.tx_dma = 1,
//		.rx_dma = 1,
//		.bits_per_word = 0,
//	};


	FUNC_ENTRY();

    spi_message_init(&m);
	
	memset(&t, 0, sizeof(struct spi_transfer));
	t.tx_buf = tx;
	t.len = spilen;
	t.speed_hz = LCM_SPI_CLOCK_SPEED;
	
    spi_message_add_tail(&t, &m);
    ret= spi_sync(g_lcm->spi,&m);
	
	SPI_DBG("<SPI>status= %d,len=%u\n",ret,spilen);
	
    FUNC_EXIT();
	
    return ret;   
}


void lcm_spi_interface(unsigned char *sbuf, size_t spilen, unsigned char mode)
{
	int i;
	size_t times = 0;
	size_t left = 0;

	FUNC_ENTRY();
	switch(mode)
	{
		case WRITE_CMD:
			if(g_lcm->mode_flag!=GPIO_PIN_MODE)
			{
				set_pin_mode(GPIO_PIN_MODE);
				g_lcm->mode_flag=GPIO_PIN_MODE;
			}
			SET_GPIO_OUT(GPIO_SPI_CS_PIN,GPIO_OUT_ZERO);
			for(i=0;i<spilen;i++)
			{
				SET_SEND_TYPE(GPIO_OUT_ZERO);
				lcm_gpio_send(sbuf[i]);
				//printk("#caojian WRITE_CMD %x\n", sbuf[i]);
			}
			SET_GPIO_OUT(GPIO_SPI_CS_PIN,GPIO_OUT_ONE);
			break;
		case WRITE_DATA:
//			if(spilen<32)
//			{
//				if(g_lcm->mode_flag!=GPIO_PIN_MODE)
//				{
//					set_pin_mode(GPIO_PIN_MODE);
//					g_lcm->mode_flag=GPIO_PIN_MODE;
//				}
//				SET_GPIO_OUT(GPIO_SPI_CS_PIN,GPIO_OUT_ZERO);
//				for(i=0;i<spilen;i++)
//				{
//					SET_SEND_TYPE(GPIO_OUT_ONE);
//					lcm_gpio_send(sbuf[i]);
//					//printk("#caojian WRITE_DATA %x\n", (int)sbuf[i]);
//				}
//				SET_GPIO_OUT(GPIO_SPI_CS_PIN,GPIO_OUT_ONE);
//			}
//			else 
//			{


				SET_SEND_TYPE(GPIO_OUT_ONE);
			
				if(g_lcm->mode_flag!=SPI_PIN_MODE)
				{
					set_pin_mode(SPI_PIN_MODE);
					g_lcm->mode_flag=SPI_PIN_MODE;
				}
				
//				if(spilen<1024)
//				{
//					set_spi_mode(SPI_FIFO_MODE);
//				}
//				else
//				{
//					set_spi_mode(SPI_DMA_MODE);
//				}	
				 				

				
				//SET_SEND_TYPE(GPIO_OUT_ONE);
				packet_size = SPI_DMA_BATYE_PER_TIMES; //
				
				printk("lcm_spi_interface WRITE_DATA============packet_size=%lu\n",packet_size); 	
				times=spilen/packet_size;
				printk("lcm_spi_interface WRITE_DATA============times00=%lu\n",times); 	
				for(i=0; i<times; i++)
				{
				    printk("lcm_spi_interface WRITE_DATA============times11=%lu\n",times); 
					lcm_spi_send(&sbuf[i*packet_size], packet_size);
				}

				if(spilen % packet_size != 0)
				{
					left = spilen - times*packet_size;
				    printk("lcm_spi_interface WRITE_DATA============left==%lu\\n",left); 
					lcm_spi_send(&sbuf[times*packet_size], left);
				} 
//			}
			break;
		case READ_DATA:
			SET_GPIO_OUT(GPIO_SPI_CS_PIN,GPIO_OUT_ZERO);
			SET_SEND_TYPE(GPIO_OUT_ZERO);	
			lcm_gpio_recive(sbuf,spilen);
			SET_GPIO_OUT(GPIO_SPI_CS_PIN,GPIO_OUT_ONE);
			break;
		default:
			SPI_DBG("No such mode!\n"); 
			break;	
	}
	//FUNC_EXIT();
}

void print_framebuf(size_t len, unsigned char *src_addr)
{
    size_t i;
    unsigned char *data;
	size_t data_len;

	data=src_addr;
	data_len=len;
	
	printk("================== dump framebuffer ==================\n");
    for(i = 0; i < len; i++)
    {                
       printk("0x%02x,",data[i]); 	
 		if(i%100==0)
       		printk("\n");
    }
	return;
} 

static int lcm_get_frambuffer(void);

static int lcm_get_frambuffer(void)
{
	int i, j;
	//unsigned int ret=0;
	
	unsigned char *p = (unsigned char*)For_Capture_p;
	unsigned char *pOut = (unsigned char*)For_Dispaly_p;

	FUNC_ENTRY();

	//ret = primary_display_capture_framebuffer_ovl((unsigned long)For_Capture_p,UFMT_RGB565);

	
	/* switch endian, RGB565 */
	for(i=0;i<LCM_HEIGHT;i++)
	{
		for(j=0;j<LCM_WIDTH;j++)
		{
			*pOut = *(p+1);
			pOut++;
			*pOut = *p;
			pOut++;
			p += 2;
		}
	}
	/* set window para */
	for(i=0;i<ARRAY_SIZE(spi_set_window);i++)
		lcm_spi_interface(spi_set_window[i].data,spi_set_window[i].num,spi_set_window[i].type);
	
	/* display with SPI */	
	lcm_spi_interface((unsigned char *)For_Dispaly_p,ONE_FB_SIZE_DISPLAY,WRITE_DATA);

	memset(For_Capture_p,0x00,ONE_FB_SIZE_OVL);
	memset(For_Dispaly_p,0x00,ONE_FB_SIZE_OVL);
	
	return 0;
}

// Interrupt handler function
void lcm_spiCap_trigger(int wakeup)
{
	FUNC_ENTRY();

	if (wakeup)
	{   	
		atomic_set(&trigger_spiCap_thread_flag,1);
		wake_up_interruptible(&trigger_spiCap_thread_wq); 		
	}
	else
	{
		atomic_set(&trigger_spiCap_thread_flag,0);
	}
	FUNC_EXIT();
}


//Thread processing function
static int Lcm_trigger_spiCap_thread(void *unused)
{

	//struct sched_param param = { .sched_priority = 4 }; //RTPM_PRIO_SCRN_UPDATE };//设置线程的优先级
	//sched_setscheduler(current, SCHED_FIFO, &param);    //设置当前线程的优先级
	
	FUNC_ENTRY();
	
	do
	{

		SPI_DBG("\n  trigger_spiCap_thread() before wake up\n");
		//down_interruptible(&g_lcm->lcm_sem);
		wait_event_interruptible(trigger_spiCap_thread_wq, atomic_read(&trigger_spiCap_thread_flag));
		atomic_set(&trigger_spiCap_thread_flag,0);
		//atomic_set(&trigger_spiCap_enable_flag,0);
		//if (!primary_display_is_sleepd())
		{
			SPI_DBG("\n trigger_spiCap_thread() after wake up\n");
		    lcm_get_frambuffer(); 
		}
		//atomic_set(&trigger_spiCap_enable_flag,1);
		//up(&g_lcm->lcm_sem);
	}while(!kthread_should_stop());
	
	FUNC_EXIT();
	return 0;
}

static int lcm_suspend(struct device *dev, pm_message_t state)
{

	FUNC_ENTRY();
	set_pin_mode(GPIO_PIN_MODE);
	SET_GPIO_OUT(GPIO_SPI_CS_PIN,GPIO_OUT_ONE);	
	FUNC_EXIT();
	return 0;
}


static int lcm_resume(struct device *dev)
{
	FUNC_ENTRY();
	set_pin_mode(GPIO_PIN_MODE);
	SET_GPIO_OUT(GPIO_SPI_CS_PIN,GPIO_OUT_ZERO);	
	FUNC_EXIT();
	return 0;

}


static int  lcm_dts_gpio_init(struct spi_device**   spidev)
{
    struct spi_device*   spi = *spidev;
	int i, ret = 0;

#ifdef CONFIG_OF
    //spi->dev.of_node = of_find_compatible_node(NULL, NULL, "mediatek,ai_gc9106");
    this_pctrl = devm_pinctrl_get(&spi->dev);


	
    if (IS_ERR(this_pctrl)) {
        dev_err(&spi->dev, "Cannot find lcm_spi pctrl!\n");
        return -ENODEV;
    }

	for (i = 0; i < STATE_MAX; i++) {
		pinctrl_state_name[i] = pinctrl_lookup_state(this_pctrl, dts_state_name[i]);

		if (IS_ERR(pinctrl_state_name[i])) {
			pr_err("lcm_spi ---- pinctrl_lookup_state '%s' failed\n", dts_state_name[i]);
			ret = PTR_ERR(pinctrl_state_name[i]);
			return ret;
		}
	}
#endif
    return 0;
}


// test

void spi_updata_screen(int x, int y, unsigned char *screen_data)
{
    unsigned long int len   = x*y*2;
	unsigned char col_cmd   = 0x2A, 
				  row_cmd   = 0x2B, 
				  ramwr_cmd = 0x2C,
				  XY_satrt  = 0x00,
				  X_end_8bit_front = ((x-1)/0xFF),
				  X_end_8bit_rear  = ((x-1)%0xFF),
				  Y_end_8bit_front = ((y-1)/0xFF),
				  Y_end_8bit_rear  = ((y-1)%0xFF);	
	
	/* set x and y */
	lcm_spi_interface(&col_cmd, 1, WRITE_CMD);
	lcm_spi_interface(&XY_satrt, 1, WRITE_DATA);
	lcm_spi_interface(&XY_satrt, 1, WRITE_DATA);
	lcm_spi_interface(&X_end_8bit_front, 1, WRITE_DATA);
	lcm_spi_interface(&X_end_8bit_rear, 1, WRITE_DATA);
	lcm_spi_interface(&row_cmd, 1, WRITE_CMD);
	lcm_spi_interface(&XY_satrt, 1, WRITE_DATA);
	lcm_spi_interface(&XY_satrt, 1, WRITE_DATA);
	lcm_spi_interface(&Y_end_8bit_front, 1, WRITE_DATA);
	lcm_spi_interface(&Y_end_8bit_rear, 1, WRITE_DATA);
	lcm_spi_interface(&ramwr_cmd, 1, WRITE_CMD);
	
	/* write color data */	
	lcm_spi_interface(screen_data,len,WRITE_DATA);

}

static int lcm_probe(struct spi_device *spi)
{
	struct spi_data *spi_data = NULL;	
	int status = -ENODEV;
	
	FUNC_ENTRY();

	spi_data = kzalloc(sizeof(struct spi_data),GFP_KERNEL);
	if (!spi_data)
	{
		return -ENOMEM;
	}
	
	For_Capture_p = (unsigned int*)kmalloc((ONE_FB_SIZE_OVL+64), GFP_KERNEL | GFP_DMA);
	if (!For_Capture_p)
	{
		return -ENOMEM;
	}
	For_Dispaly_p = (unsigned int*)kmalloc((ONE_FB_SIZE_OVL+64), GFP_KERNEL | GFP_DMA);
	if (!For_Dispaly_p)
	{
		return -ENOMEM;
	}
	For_Capture_p = (unsigned int*)ALIGN_TO((unsigned long)For_Capture_p,64); //地址64 align//MTK_FB_ALIGNMENT
	For_Dispaly_p = (unsigned int*)ALIGN_TO((unsigned long)For_Dispaly_p,64); //地址64 align//MTK_FB_ALIGNMENT

	//init head
	INIT_LIST_HEAD(&spi_data->device_entry);
	spin_lock_init(&spi_data->spi_lock);

	//init lock
	mutex_init(&spi_data->buf_lock);
	
	sema_init(&spi_data->lcm_sem, 1);
	
	mutex_lock(&device_list_lock);

	spi_data->mode_flag = PIN_MODE_NULL;

	status = IS_ERR(spi_data->device)? PTR_ERR(spi_data->device):0;
	if (status !=0)
	{
		return status;
	}
	spi_set_drvdata(spi,spi_data); 
	
	spi_data->spi = spi;
	spi_data->spi->bits_per_word = 8;
	spi_data->spi->mode = SPI_MODE_0;
	spi_data->spi->max_speed_hz = LCM_SPI_CLOCK_SPEED; // spi speed

//	spi_data->spi->controller_data = (void*)&spi_conf;
//	spi_setup(spi_data->spi); 


	
	lcm_device = spi_data->device;
	
	//Thread operation
	lcm_thread = kthread_run(Lcm_trigger_spiCap_thread,"Spi lcm thread","spithread");
	
	g_lcm = spi_data;

	lcm_dts_gpio_init(&spi_data->spi);  // gpio init
	
	//spi init
	//set_pin_mode(SPI_PIN_MODE);
	
	mutex_unlock(&device_list_lock);
	FUNC_EXIT();
	return status;
}


static int lcm_spi_init(void)
{
	int ret=0;
	
	FUNC_ENTRY();
	
	//ret=spi_register_board_info(spi_board_info,ARRAY_SIZE(spi_board_info));
	ret=spi_register_driver(&lcm_driver);

	FUNC_EXIT();
	return ret; 
}


static void lcm_spi_exit(void)
{
	FUNC_ENTRY();
	
	spi_unregister_driver(&lcm_driver);
	device_destroy(g_lcm->class,g_lcm->devno);
	if (g_lcm->class)
	{
		class_destroy(g_lcm->class);
	}
	unregister_chrdev_region (g_lcm->devno,1);
	list_del(&g_lcm->device_entry);
	
	FUNC_EXIT();
}

module_init(lcm_spi_init);
module_exit(lcm_spi_exit);

MODULE_DESCRIPTION("MT6739 SPI Controller Driver");
MODULE_AUTHOR("SPI");
MODULE_LICENSE("GPL");
MODULE_ALIAS("lcm_spi");
