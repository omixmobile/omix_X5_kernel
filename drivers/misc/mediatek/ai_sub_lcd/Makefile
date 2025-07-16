#
# Makefile for the kernel side torch device drivers.
#
#include $(srctree)/drivers/misc/mediatek/Makefile.custom
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat
ccflags-y += -I$(srctree)/drivers/misc/mediatek

obj-$(CONFIG_AI_BSP_SUBLCD_GC9106) +=  ai_sublcd_gc9106.o 
obj-$(CONFIG_AI_BSP_SUBLCD_GC9106) +=  ai_sublcd_spi.o
obj-$(CONFIG_AI_BSP_SUBLCD_GC9106) +=  ai_sublcd_dis.o
#ccflags-y	:= -DDEBUG
