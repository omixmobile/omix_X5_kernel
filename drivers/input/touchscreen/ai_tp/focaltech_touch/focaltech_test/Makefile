#drivers/input/touchscreen/mediatek/focaltech_touch/focaltech_test
#
#Makefile for test module
#
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/$(CONFIG_TOUCHSCREEN_MTK_FTS_DIRECTORY)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/
ccflags-y += -I$(srctree)/drivers/base
	
obj-y += focaltech_test.o
obj-y += focaltech_test_ini.o
obj-y += focaltech_test_detail_threshold.o
obj-y += supported_ic/

