#ifndef __LCM_SPI_H__
#define __LCM_SPI_H__

#include <linux/delay.h>

#define LCM_SPI_NAME		"lcm-spi-bus"

/*
#define SPI_MODE_MASK		(SPI_CPHA | SPI_CPOL | SPI_CS_HIGH \
							| SPI_LSB_FIRST | SPI_3WIRE | SPI_LOOP \
							| SPI_NO_CS | SPI_READY)
*/
#define ALIGN_TO(x, n)  	(((x) + ((n) - 1)) & ~((n) - 1))
// #define ENDIAN32bit_SWITCH(x)    (((((x) >> 16) & 0xFFFF) <<  0) |((((x) >>  0) & 0xFFFF) << 16))

#define LCM_WIDTH					240
#define LCM_HEIGHT				320 
#define LCM_BPP						16
#define LCM_SPI_CLOCK_SPEED 		27000000                             // 16*1000*1000
#define ONE_FB_SIZE_OVL 			(LCM_WIDTH*LCM_HEIGHT*(sizeof(int)))
#define ONE_FB_SIZE_DISPLAY  		(LCM_WIDTH*LCM_HEIGHT*(LCM_BPP/8))
#define SPI_DMA_BATYE_PER_TIMES  	(1024*112)
#define SPI_FIFO_BATYE_PER_TIMES  	32
#define SPI_GPIO_BATYE_PER_TIMES  	1
#define FIFO_TRANSFER_MAX_LEN		1024

#define SPI_BUS_NUM                 0
#define SPI_CHIP_SELECT             0

#define GPIO_OUT_ZERO     0
#define GPIO_OUT_ONE      1

//spi gpio
#ifndef CONFIG_OF
#define GPIO_SPI_CS_PIN        		(16 | 0x80000000)
#define GPIO_SPI_SCK_PIN       		(17 | 0x80000000)
#define GPIO_SPI_MISO_PIN     		(19 | 0x80000000)
#define GPIO_SPI_MOSI_PIN    		(18 | 0x80000000)
#else
#define GPIO_SPI_CS_PIN             (1)
#define GPIO_SPI_SCK_PIN            (2)
#define GPIO_SPI_MISO_PIN           (3)
#define GPIO_SPI_MOSI_PIN           (4)
#define GPIO_SPI_RST_PIN            (5)
#endif

#define	LCM_SPI_SCK_PIN	        	GPIO_SPI_SCK_PIN            
#define	LCM_SPI_SCK_PIN_M_GPIO		GPIO_MODE_00
#define	LCM_SPI_SCK_PIN_M_SPI_CK    GPIO_MODE_01

#define	LCM_SPI_CS_PIN				GPIO_SPI_CS_PIN
#define	LCM_SPI_CS_PIN_M_GPIO		GPIO_MODE_00
#define	LCM_SPI_CS_PIN_M_SPI_CS		GPIO_MODE_01
	
#define	LCM_SPI_MOSI_PIN			GPIO_SPI_MOSI_PIN
#define	LCM_SPI_MOSI_PIN_M_GPIO		GPIO_MODE_00
#define	LCM_SPI_MOSI_PIN_M_SPI_MOSI	GPIO_MODE_01

#define	LCM_SPI_MISO_PIN			GPIO_SPI_MISO_PIN
#define	LCM_SPI_MISO_PIN_M_GPIO		GPIO_MODE_00
#define	LCM_SPI_MISO_PIN_M_SPI_MISO	GPIO_MODE_01


#ifndef CONFIG_OF
#define SET_GPIO_MODE(num,mode)							(mt_set_gpio_mode((num),(mode)))
#define SET_GPIO_DIR(num,dir)							(mt_set_gpio_dir((num),(dir)))
#define SET_GPIO_OUT(num,level)							(mt_set_gpio_out((num), (level)))
#define SET_GPIO_PULL_ENABLE(num,enable)				(mt_set_gpio_pull_enable((num),(enable)))
#define GET_GPIO_VALUE(num)								(mt_get_gpio_in(num))
#else
#define SET_GPIO_MODE(num,mode)							//
#define SET_GPIO_DIR(num,dir)							//
#define SET_GPIO_OUT(num,level)							(dts_pin_out_state(num,level))	
#define SET_GPIO_PULL_ENABLE(num,enable)				//
#define GET_GPIO_VALUE(num)								//
#endif

#if 0 // 9bit(without RS pin) or 8bit(with RS pin)
#define SET_SEND_TYPE(level) 				{SET_GPIO_OUT(GPIO_SPI_MOSI_PIN,(level));SET_GPIO_OUT(GPIO_SPI_SCK_PIN,GPIO_OUT_ZERO);SET_GPIO_OUT(GPIO_SPI_SCK_PIN,GPIO_OUT_ONE);}
#else
#define SET_SEND_TYPE(level)				{SET_GPIO_OUT(GPIO_SPI_MISO_PIN,(level));}
#endif	

//#define SET_RESET_PIN(v)   {SET_GPIO_OUT(GPIO_SPI_RST_PIN,(level));}

struct spi_data {
	struct spi_device *spi;
	struct class *class;
	struct device *device;
	struct cdev wg_cdev;
	struct mutex buf_lock;
	struct list_head device_entry;
	unsigned users;
	struct semaphore lcm_sem;
	dev_t devno;
	spinlock_t spi_lock;
	int mode_flag;
};

struct send_data {
	unsigned char type;
	size_t num;
	unsigned char data[4];
};


typedef enum
{
	SPI_DMA_MODE=0,
	SPI_FIFO_MODE=1
}SPI_MODE;

typedef enum
{
	GPIO_PIN_MODE=0,
	SPI_PIN_MODE=1,
	PIN_MODE_NULL=2
}PIN_SET;


typedef enum
{
	WRITE_CMD=0,
	WRITE_DATA=1,
	READ_DATA=2
}SPI_OP_MODE;


#if 0
//high_time and low_time Decision SCK frequency , lcm_ Using 10M
static struct mt_chip_conf spi_conf=
{
 	.setuptime = 7,
	.holdtime = 7,
	.high_time = 50,  //此处决定slk的频率10--6m   15--4m   20--3m  30--2m  [ 60--1m 120--0.5m  300--0.2m]
	.low_time =  50,  
	.cs_idletime = 2,

	.cpol = 0,
	.cpha = 0,

	.rx_mlsb = 1,   //先传高位 
	.tx_mlsb = 1,

	.tx_endian = 1, //tx_endian 表示大端模式
	.rx_endian = 1,

	.com_mod = DMA_TRANSFER,
	.pause = 1,
	.finish_intr = 1,
	.deassert = 0,
	.ulthigh = 0,
	.tckdly = 0,
};
#endif

static struct send_data spi_set_window[] = {
	{0,1,{0x2A}},
	{1,4,{0x00,0x00,((LCM_WIDTH-1)/0xFF),((LCM_WIDTH-1)%0xFF)}},   // Xstart -- Xend
	{0,1,{0x2B}},
	{1,4,{0x00,0x00,((LCM_HEIGHT-1)/0xFF),((LCM_HEIGHT-1)%0xFF)}}, // Ystart -- Yend
	{0,1,{0x2c}},
};

//Declare and initialize the queue
static struct spi_data *g_lcm= NULL;
static struct device *lcm_device = NULL;
static struct task_struct *lcm_thread = NULL;
static DEFINE_MUTEX(device_list_lock);
static LIST_HEAD(device_list);
atomic_t trigger_spiCap_thread_flag = ATOMIC_INIT(0);
atomic_t trigger_spiCap_enable_flag = ATOMIC_INIT(1);
DECLARE_WAIT_QUEUE_HEAD(trigger_spiCap_thread_wq);

static size_t packet_size=0;
unsigned int *For_Capture_p = NULL;
unsigned int *For_Dispaly_p = NULL;

//static int lcm_spi_init(void);
//static void lcm_spi_exit(void);
//static int lcm_suspend(struct device *dev, pm_message_t state);
//static int lcm_resume(struct device *dev);
//static int lcm_probe(struct spi_device *spi);
extern int primary_display_capture_framebuffer_ovl(unsigned long pbuf,unsigned int format);
extern int primary_display_is_sleepd(void);



//#define LCM_SPI_DEBUG  // debug on/off, default off.
#ifdef LCM_SPI_DEBUG
#define SPI_DBG(fmt, args...) do{ \
					printk("lcm_spi:" fmt, ##args);\
				}while(0)
#define FUNC_ENTRY()  printk("lcm_spi:%s, entry\n", __func__)
#define FUNC_EXIT()   printk("lcm_spi:%s, exit\n", __func__)
#else
#define SPI_DBG(fmt, args...)
#define FUNC_ENTRY()
#define FUNC_EXIT()
#endif
#endif