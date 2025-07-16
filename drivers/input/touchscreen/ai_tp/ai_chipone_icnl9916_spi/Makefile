ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/

obj-$(CONFIG_TOUCHSCREEN_CHIPONE) += chipone-tddi.o

obj-$(CONFIG_AI_BSP_TOUCHSCREEN_MTK_CHIPONE_SPI_TOUCH) := cts_tpd_driver.o cts_i2c_driver.o
obj-$(CONFIG_AI_BSP_TOUCHSCREEN_MTK_CHIPONE_SPI_TOUCH) += cts_core.o
obj-$(CONFIG_AI_BSP_TOUCHSCREEN_MTK_CHIPONE_SPI_TOUCH) += cts_sfctrlv2.o
obj-$(CONFIG_AI_BSP_TOUCHSCREEN_MTK_CHIPONE_SPI_TOUCH) += cts_spi_flash.o
obj-$(CONFIG_AI_BSP_TOUCHSCREEN_MTK_CHIPONE_SPI_TOUCH) += cts_firmware.o
obj-$(CONFIG_AI_BSP_TOUCHSCREEN_MTK_CHIPONE_SPI_TOUCH) += cts_test.o
obj-$(CONFIG_AI_BSP_TOUCHSCREEN_MTK_CHIPONE_SPI_TOUCH) += cts_charger_detect.o
obj-$(CONFIG_AI_BSP_TOUCHSCREEN_MTK_CHIPONE_SPI_TOUCH) += cts_earjack_detect.o
obj-$(CONFIG_AI_BSP_TOUCHSCREEN_MTK_CHIPONE_SPI_TOUCH) += cts_tcs.o
obj-$(CONFIG_AI_BSP_TOUCHSCREEN_MTK_CHIPONE_SPI_TOUCH) += cts_platform.o
obj-$(CONFIG_AI_BSP_TOUCHSCREEN_MTK_CHIPONE_SPI_TOUCH) += cts_tool.o
obj-$(CONFIG_AI_BSP_TOUCHSCREEN_MTK_CHIPONE_SPI_TOUCH) += cts_sysfs.o
obj-$(CONFIG_AI_BSP_TOUCHSCREEN_MTK_CHIPONE_SPI_TOUCH) += cts_strerror.o
obj-$(CONFIG_AI_BSP_TOUCHSCREEN_MTK_CHIPONE_SPI_TOUCH) += cts_oem.o

#Antaiui <ANT_AI_TP> <chenht> <2023-02-16> add mmi ctp test begin
obj-$(CONFIG_AI_BSP_TOUCHSCREEN_MTK_CHIPONE_SPI_TOUCH) += ai_tpd_feature.o
#Antaiui <ANT_AI_TP> <chenht> <2023-02-16> add mmi ctp test end