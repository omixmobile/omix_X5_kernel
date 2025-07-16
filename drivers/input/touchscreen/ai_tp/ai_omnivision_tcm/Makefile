#
# Makefile for the Omnivision TCM touchscreen driver.
#

# Each configuration option enables a list of files.

obj-y += omnivision_tcm_spi.o
#obj-$(CONFIG_TOUCHSCREEN_OMNIVISION_TCM_I2C) += omnivision_tcm_i2c.o
obj-y += omnivision_tcm_core_module.o
omnivision_tcm_core_module-objs := omnivision_tcm_core.o omnivision_tcm_touch.o
obj-y += omnivision_tcm_zeroflash.o
obj-y += omnivision_tcm_reflash.o
obj-y += omnivision_tcm_device.o
obj-y += omnivision_tcm_testing.o
obj-y += omnivision_tcm_csv_parser.o
obj-y += omnivision_tcm_recovery.o
obj-y += omnivision_tcm_diagnostics.o
#Antaiui <AI_BSP_CTP> <chenht> <2022-12-22> add for gesture mode begin
obj-y +=  ai_tpd_feature.o
#Antaiui <AI_BSP_CTP> <chenht> <2022-12-22> add for gesture mode end