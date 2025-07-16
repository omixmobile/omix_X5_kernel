#include "cts_config.h"


//extern int ant_cts_test_mode_init(struct device *dev);
//extern int ant_ctp_test_mode_exit(struct device *dev);

#ifndef _GN_TPD_FEATURE_H_
#define _GN_TPD_FEATURE_H_
int ai_cts_tpd_feature_init_data(struct device *dev);
int ai_cts_tpd_feature_exit(struct device *dev);
#endif