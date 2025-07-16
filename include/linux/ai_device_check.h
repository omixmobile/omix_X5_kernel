//antaiui <GN_BSP_DEVICECHECK> <lihl> <20171117> add for 49184 start

#ifndef __AI_DEVICE_CHECK_H__
#define __AI_DEVICE_CHECK_H__

//static int ai_device_debug_enable = 0;
#define AI_DEVICE_DEBUG(format, args...) do{ \
	if(0) \
	{\
		printk(KERN_EMERG format,##args);\
	}\
}while(0)

#define AI_DEVICE_NAME_LEN 50
#define AI_DEVICE_VENDOR_LEN 20
#define AI_DEVICE_VERSION_LEN 20

enum ai_device_type
{
	AI_DEVICE_TYPE_LCD = 0,
	AI_DEVICE_TYPE_ACCELEROMETER, 
	AI_DEVICE_TYPE_TP, 
	//AI_DEVICE_TYPE_TP_IC,
	//AI_DEVICE_TYPE_TP_LCM,
	AI_DEVICE_TYPE_MAGNETIC_FIELD,
	AI_DEVICE_TYPE_GYROSCOPE,
	AI_DEVICE_TYPE_LIGHT ,
	AI_DEVICE_TYPE_PRESSURE,
	AI_DEVICE_TYPE_TEMPERATURE,
	AI_DEVICE_TYPE_PROXIMITY,
//Antaiui <AI_BSP_SENSOR> <hehl> <2020-07-08> add for 1907 begin		
	AI_DEVICE_TYPE_ALSPS,
//Antaiui <AI_BSP_SENSOR> <hehl> <2020-07-08> add for 1907 end		
	AI_DEVICE_TYPE_CAP_KEY,
	AI_DEVICE_TYPE_MAIN_CAM,
	AI_DEVICE_TYPE_SUB_CAM,
//Antaiui camera xionggh modify for ASW1102A-669 begin
	AI_DEVICE_TYPE_MAIN2_CAM,
	AI_DEVICE_TYPE_SUB2_CAM,
//Antaiui camera xionggh modify for ASW1102A-669 end
/* <ANTITEK> <ANT_BSP_CAM> <xuel> <2019-05-17> added begin */
	AI_DEVICE_TYPE_MAIN3_CAM,
/* <ANTITEK> <ANT_BSP_CAM> <xuel> <2019-05-17> added end */
/* Antaiui <ANT_BSP_CAM> <xieht> <2020-07-24> add sub3 begin */
	AI_DEVICE_TYPE_SUB3_CAM,
/* Antaiui <ANT_BSP_CAM> <xieht> <2020-07-24> add sub3  end  */
	AI_DEVICE_TYPE_FINGER,
	AI_DEVICE_TYPE_FM,
	AI_DEVICE_TYPE_CPU,
	AI_DEVICE_TYPE_BATTERY,
	AI_DEVICE_TYPE_MEMORY,
//Antaiui <AI_BSP_CHG> <zhouli> <20180321> add for ASW1102A-813 begin
	AI_DEVICE_TYPE_CHARGER,
//Antaiui <AI_BSP_CHG> <zhouli> <20180321> add for ASW1102A-813 end
	AI_DEVICE_TYPE_NFC,
//Antai <AI_BSP_MEMORY> <luobowen> <2021-06-28> add for ddr start
	AI_DEVICE_TYPE_DDR,
//Antai <AI_BSP_MEMORY> <luobowen> <2021-06-28> add for ddr end
	AI_DEVICE_TYPE_TOTAL, 
};

struct ai_device_info
{
	char     name[AI_DEVICE_NAME_LEN];
	char     vendor[AI_DEVICE_VENDOR_LEN];
	char     version[AI_DEVICE_VERSION_LEN];
	int      ai_dev_type;
};

extern int ai_set_device_info(struct ai_device_info ai_dev_info);

//#ifdef CUSTOM_KERNEL_LCM
#define AI_DEVICE_LCD
//#endif

//#ifdef CUSTOM_KERNEL_ACCELEROMETER
#define AI_DEVICE_ACCELEROMETER
//#endif

//#ifdef CUSTOM_KERNEL_TOUCHPANEL
#define AI_DEVICE_TOUCHPANEL
//#endif

//Antaiui <AI_BSP_SENS> <hehl> <2022-02-23> modify begin
#ifdef CONFIG_CUSTOM_KERNEL_MAGNETOMETER
#define AI_DEVICE_MAGNETIC_FIELD
#endif
//Antaiui <AI_BSP_SENS> <hehl> <2022-02-23> modify end

#ifdef CUSTOM_KERNEL_GYROSCOPE
//#define AI_DEVICE_GYROSCOPE
#endif

//Antaiui <AI_BSP_SENSOR> <hehl> <2020-07-08> add for 1907 begin	
//#ifdef CUSTOM_KERNEL_ALSPS
#define AI_DEVICE_LIGHT
//#endif
//Antaiui <AI_BSP_SENSOR> <hehl> <2020-07-08> add for 1907 end	

//#ifdef CUSTOM_KERNEL_PRESSURE
//#define AI_DEVICE_PRESSURE
//#endif

//#ifdef CUSTOM_KERNEL_THERMOMETER
//#define AI_DEVICE_TEMPERATURE
//#endif

//#ifdef CUSTOM_KERNEL_PROXIMITY
//Antaiui <AI_BSP_SENSOR> <hehl> <2020-07-07> add for 1907 begin
#define AI_DEVICE_PROXIMITY
//Antaiui <AI_BSP_SENSOR> <hehl> <2020-07-07> add for 1907 begin
//#endif

//Antaiui <AI_BSP_SENSOR> <hehl> <2020-07-08> add for 1907 begin	
//#define AI_DEVICE_ALSPS
//Antaiui <AI_BSP_SENSOR> <hehl> <2020-07-08> add for 1907 end	
//#ifdef CUSTOM_KERNEL_CAPKEY
//#define AI_DEVICE_CAP_KEY
//#endif

//#ifdef CUSTOM_KERNEL_FM
//#define AI_DEVICE_FM
//#endif

//#define AI_DEVICE_CPU //chiva
#ifdef CONFIG_MTK_FINGERPRINT_SUPPORT
#define AI_DEVICE_FINGER 
#endif
//antaiui <GN_BSP_DEVICECHECK> <lihl> <20171117> add for 49184 start
#define AI_DEVICE_MEMORY
//antaiui <GN_BSP_DEVICECHECK> <lihl> <20171117> add for 49184 end


#define AI_DEVICE_BATTERY 

//Antaiui <AI_BSP_CHG> <zhouli> <20180321> add for ASW1102A-813 begin
#define AI_DEVICE_CHARGER
//Antaiui <AI_BSP_CHG> <zhouli> <20180321> add for ASW1102A-813 end
#ifdef CONFIG_NFC
#define AI_DEVICE_NFC
#endif

//Antai <AI_BSP_MEMORY> <luobowen> <2021-06-28> add for ddr start
//#define AI_DEVICE_DDR
//Antai <AI_BSP_MEMORY> <luobowen> <2021-06-28> add for ddr end


#endif // __AI_DEVICE_CHECK_H__

//antaiui <GN_BSP_DEVICECHECK> <lihl> <20171117> add for 49184 end

