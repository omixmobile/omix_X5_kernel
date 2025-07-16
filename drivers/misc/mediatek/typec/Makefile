# SPDX-License-Identifier: GPL-2.0

subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/typec/inc

obj-$(CONFIG_TCPC_CLASS) += tcpc/
obj-$(CONFIG_MTK_USB_TYPEC_U3_MUX) += switch/
obj-$(CONFIG_OCP96011_I2C) += switch/
obj-$(CONFIG_MTK_USB_TYPEC_DP_MUX) += dp/
obj-$(CONFIG_MTK_USB_TYPEC_DP_MUX_V2) += dp/

#Antai <AI_BSP_Audio> <yaoyc> <2022-02-16> add for ASW2200 HL5280 driver begin
obj-$(CONFIG_AI_BSP_USB_TYPEC_HL5280) += hl5280/
#Antai <AI_BSP_Audio> <yaoyc> <2022-02-16> add for ASW2200 HL5280 driver end
#Antai <AI_BSP_Audio> <hehl> <2022-06-13> coompatiable ic YHM1003 driver begin 
obj-$(CONFIG_AI_BSP_USB_TYPEC_YHM1003) += yhm1003/
#Antai <AI_BSP_Audio> <hehl> <2022-06-13> coompatiable ic YHM1003 driver end