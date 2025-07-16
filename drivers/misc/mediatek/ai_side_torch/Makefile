#
# Makefile for the kernel side torch device drivers.
#
#include $(srctree)/drivers/misc/mediatek/Makefile.custom
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat
ccflags-y += -I$(srctree)/drivers/misc/mediatek

# obj-$(CONFIG_AI_BSP_SIDE_TORCH_SUPPORT)  :=  ai_side_torch.o
obj-y       +=  torchs-aw36515.o
obj-y       +=  ai_side_torch.o

#ccflags-y	:= -DDEBUG
