/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/kernel.h>
#include "cam_cal_list.h"
#include "eeprom_i2c_common_driver.h"
#include "eeprom_i2c_custom_driver.h"
#include "kd_imgsensor.h"

struct stCAM_CAL_LIST_STRUCT g_camCalList[] = {
	/*Below is commom sensor */
	{IMX519_SENSOR_ID, 0xA0, Common_read_region},
	{S5K2T7SP_SENSOR_ID, 0xA4, Common_read_region},
	{IMX338_SENSOR_ID, 0xA0, Common_read_region},
	{S5K4E6_SENSOR_ID, 0xA8, Common_read_region},
	{IMX386_SENSOR_ID, 0xA0, Common_read_region},
	{S5K3M3_SENSOR_ID, 0xA0, Common_read_region},
	{S5K2L7_SENSOR_ID, 0xA0, Common_read_region},
	{IMX398_SENSOR_ID, 0xA0, Common_read_region},
	{IMX350_SENSOR_ID, 0xA0, Common_read_region},
	{IMX318_SENSOR_ID, 0xA0, Common_read_region},
	{IMX386_MONO_SENSOR_ID, 0xA0, Common_read_region},
	/*B+B. No Cal data for main2 OV8856*/
	{S5K2P7_SENSOR_ID, 0xA0, Common_read_region},
#ifdef SUPPORT_S5K4H7
	{S5K4H7_SENSOR_ID, 0xA0, zte_s5k4h7_read_region},
	{S5K4H7SUB_SENSOR_ID, 0xA0, zte_s5k4h7_sub_read_region},
#endif
    // Antaiui <AI_BSP_CAM> <xieht> <2021-01-29> 2002 camera otp porting begin
	{S5KJN1SQ03_SENSOR_ID, 0xA0, Common_read_region},
	{S5K4H8_SENSOR_ID, 0xA0, Common_read_region},
	{GC8034_SENSOR_ID, 0xA2, Common_read_region},
	{GC02M1MICRO_SENSOR_ID, 0xA4, Common_read_region},
	// Antaiui <AI_BSP_CAM> <xieht> <2021-01-29> 2002 camera otp porting end
	/*  ADD before this line */
	{0, 0, 0}       /*end of list */
};

unsigned int cam_cal_get_sensor_list(
	struct stCAM_CAL_LIST_STRUCT **ppCamcalList)
{
	if (ppCamcalList == NULL)
		return 1;

	*ppCamcalList = &g_camCalList[0];
	return 0;
}


