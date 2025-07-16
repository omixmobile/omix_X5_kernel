subdir-ccflags-y += -Werror
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/ai_tp
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
# In case the platform does NOT support this type of sensors
obj-y   +=  mtk_tpd.o
obj-y   +=  tpd_button.o
obj-y   +=  tpd_calibrate.o
obj-y   +=  tpd_debug.o
obj-y   +=  tpd_default.o
obj-y   +=  tpd_misc.o
obj-y   +=  tpd_setting.o
#Antai <AI_BSP_LCD> <hehl> <2020-12-30> modify for 1919 start
obj-$(CONFIG_AI_BSP_TOUCHSCREEN_MTK_ILITEK_SPI)	+=  ilitek_spi_touch_driver/
#Antai <AI_BSP_LCD> <hehl> <2020-12-30> modify for 1919 end
#Antaiui <AI_BSP_TP> <hehl> <2021-08-03> modify for 2003 begin
obj-$(CONFIG_AI_BSP_TOUCHSCREEN_MTK_ILI9883A_SPI)	+=  ai_ili9883a/
#Antaiui <AI_BSP_TP> <hehl> <2021-08-03> modify for 2003 end
#Antaiui <AI_BSP_TP> <hehl> <2022-01-20> modify for 2200 begin
obj-$(CONFIG_AI_BSP_TOUCHSCREEN_MTK_NT36672_SPI)	+=  ai_spi_nt36672/
#Antaiui <AI_BSP_TP> <hehl> <2022-01-20> modify for 2200 end
#Antaiui <AI_BSP_TP> <hehl> <2022-03-24> add tp for 2002 begin
obj-$(CONFIG_TOUCHSCREEN_MTK_NT36xxx)	+=  NT36xxx/
#Antaiui <AI_BSP_TP> <hehl> <2022-03-24> add tp for 2002 end
#Antaiui <AI_BSP_TP> <wutj> <2022-05-27> add tp for 2106 begin
obj-$(CONFIG_TOUCHSCREEN_FTS) += focaltech_touch/
#Antaiui <AI_BSP_TP> <wutj> <2022-05-27> add tp for 2106 end
#Antaiui <AI_BSP_TP> <chenht> <2022-08-03> add for 2201 begin
obj-$(CONFIG_AI_BSP_TOUCHSCREEN_MTK_OMNIVISION)	+=  ai_omnivision_tcm/
#Antaiui <AI_BSP_TP> <chenht> <2022-08-03> add for 2201 end
#Antaiui <AI_BSP_TP> <chenht> <2022-09-23> add for 2206 begin
obj-$(CONFIG_AI_BSP_TOUCHSCREEN_MTK_FOCALTECH_SPI_TOUCH)	+=  focaltech_touch_spi/
#Antaiui <AI_BSP_TP> <chenht> <2022-09-23> add for 2206 end
#Antaiui <AI_BSP_TP> <chenht> <2023-01-30> add TP for 2206 begin
obj-$(CONFIG_AI_BSP_TOUCHSCREEN_MTK_CHIPONE_SPI_TOUCH)	+=  ai_chipone_icnl9916_spi/
#Antaiui <AI_BSP_TP> <chenht> <2023-01-30> add TP for 2206 end