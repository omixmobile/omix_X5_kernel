/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include "eeprom_i2c_dev.h"

// Antaiui <AI_BSP_CAM> <xieht> <2021-01-29> 2002 camera otp porting begin
static enum EEPROM_I2C_DEV_IDX gi2c_dev_sel[IMGSENSOR_SENSOR_IDX_MAX_NUM] = {
	I2C_DEV_IDX_1, /* main */
	I2C_DEV_IDX_2, /* sub */
	I2C_DEV_IDX_2, /* main2 */
	I2C_DEV_IDX_2, /* sub2 */
	I2C_DEV_IDX_2, /* main3 */
};
// Antaiui <AI_BSP_CAM> <xieht> <2021-01-29> 2002 camera otp porting end

enum EEPROM_I2C_DEV_IDX get_i2c_dev_sel(enum IMGSENSOR_SENSOR_IDX idx)
{
	if (idx < IMGSENSOR_SENSOR_IDX_MAX_NUM)
		return gi2c_dev_sel[(unsigned int)idx];
	return I2C_DEV_IDX_1;
}

int gi2c_dev_timing[I2C_DEV_IDX_MAX] = {
	100, /* dev1, 100k */
	100, /* dev2, 100k */
	100, /* dev3, 100k */
};

