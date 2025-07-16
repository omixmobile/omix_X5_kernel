# Makefile for the focaltech touchscreen drivers.

ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/$(CONFIG_TOUCHSCREEN_MTK_FTS_DIRECTORY)
ccflags-y += -I$(srctree)/drivers/spi/mediatek/$(MTK_PLATFORM)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/
ccflags-y += -I$(srctree)/drivers/base
ccflags-y += -I$(srctree)/drivers/misc/mediatek/hwmon/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/alsps/inc/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/hwmon/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/alsps/inc/

obj-$(CONFIG_AI_BSP_TOUCHSCREEN_MTK_FOCALTECH_SPI_TOUCH)	+=  focaltech_core.o
obj-$(CONFIG_AI_BSP_TOUCHSCREEN_MTK_FOCALTECH_SPI_TOUCH)	+=  focaltech_ex_fun.o
obj-$(CONFIG_AI_BSP_TOUCHSCREEN_MTK_FOCALTECH_SPI_TOUCH)	+=  focaltech_ex_mode.o
obj-$(CONFIG_AI_BSP_TOUCHSCREEN_MTK_FOCALTECH_SPI_TOUCH)	+=  focaltech_gesture.o
obj-$(CONFIG_AI_BSP_TOUCHSCREEN_MTK_FOCALTECH_SPI_TOUCH)	+=  focaltech_esdcheck.o
obj-$(CONFIG_AI_BSP_TOUCHSCREEN_MTK_FOCALTECH_SPI_TOUCH)   +=  focaltech_point_report_check.o
obj-$(CONFIG_AI_BSP_TOUCHSCREEN_MTK_FOCALTECH_SPI_TOUCH)   +=  focaltech_proximity.o
obj-$(CONFIG_AI_BSP_TOUCHSCREEN_MTK_FOCALTECH_SPI_TOUCH)   +=  focaltech_test/

#Antaiui <AI_BSP_CTP> <chenht> <2022-11-30> add for gesture mode begin
obj-$(CONFIG_AI_BSP_TOUCHSCREEN_MTK_FOCALTECH_SPI_TOUCH)   +=  ai_tpd_feature.o
#Antaiui <AI_BSP_CTP> <chenht> <2022-11-30> add for gesture mode end


obj-$(CONFIG_AI_BSP_TOUCHSCREEN_MTK_FOCALTECH_SPI_TOUCH)	+=  focaltech_flash.o



obj-$(CONFIG_AI_BSP_TOUCHSCREEN_MTK_FOCALTECH_SPI_TOUCH)   +=  focaltech_spi.o
