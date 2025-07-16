# include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Linux driver folder
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/
#Antai <AI_BSP_TP> <hehl> <20210220> add for TP factory test start
obj-$(CONFIG_TOUCHSCREEN_MTK_NT36xxx) += nt36xxx.o nt36xxx_fw_update.o nt36xxx_ext_proc.o nt36xxx_mp_ctrlram.o ai_tpd_feature.o
#Antai <AI_BSP_TP> <hehl> <20210220> add for TP factory test end