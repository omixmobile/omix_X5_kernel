#ifndef _AI_TPD_FEATURE_H_
#define _AI_TPD_FEATURE_H_
//Antai <AI_BSP_TP> <hehl> <2021-02-20> add for TP mmi start
#ifdef CONFIG_AI_BSP_MTK_DEVICE_CHECK
#include <linux/ai_device_check.h>
#endif
//Antai <AI_BSP_TP> <hehl> <2021-02-20> add for TP mmi end
int ai_nvt_tpd_feature_init_data(struct i2c_client *client);
int ai_nvt_tpd_feature_reinit(void);
#endif
