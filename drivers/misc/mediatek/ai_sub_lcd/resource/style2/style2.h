#include "style2_time0.h"
#include "style2_time1.h"
#include "style2_time2.h"
#include "style2_time3.h"
#include "style2_time4.h"
#include "style2_time5.h"
#include "style2_time6.h"
#include "style2_time7.h"
#include "style2_time8.h"
#include "style2_time9.h"

#include "style2_day0.h"
#include "style2_day1.h"
#include "style2_day2.h"
#include "style2_day3.h"
#include "style2_day4.h"
#include "style2_day5.h"
#include "style2_day6.h"
#include "style2_day7.h"
#include "style2_day8.h"
#include "style2_day9.h"

#include "style2_week1.h"
#include "style2_week2.h"
#include "style2_week3.h"
#include "style2_week4.h"
#include "style2_week5.h"
#include "style2_week6.h"
#include "style2_week7.h"

#include "style2_batt25.h"
#include "style2_batt50.h"
#include "style2_batt75.h"
#include "style2_batt100.h"

#include "style2_bar.h"


#define HOUR_STYLE2_WIDTH 	        (18)
#define HOUR_STYLE2_HEIGHT 	        (52)
#define HOUR_STYLE2_L_X_COOR 	    (41)
#define HOUR_STYLE2_L_Y_COOR 	    (6)//5
#define HOUR_STYLE2_H_X_COOR 	    (17)
#define HOUR_STYLE2_H_Y_COOR 	    (6)//5
#define MINUTE_STYLE2_WIDTH 	    (18)
#define MINUTE_STYLE2_HEIGHT 	    (52)
#define MINUTE_STYLE2_L_X_COOR 	    (41)
#define MINUTE_STYLE2_L_Y_COOR 	    (65)
#define MINUTE_STYLE2_H_X_COOR 	    (17)
#define MINUTE_STYLE2_H_Y_COOR 	    (65)

#define BATTERY_STYLE2_WIDTH 	    (14)
#define BATTERY_STYLE2_HEIGHT 	    (30)
#define BATTERY_STYLE2_X_COOR 	    (54)//54
#define BATTERY_STYLE2_Y_COOR 	    (123)

#define WEEK_STYLE2_WIDTH 	        (31)
#define WEEK_STYLE2_HEIGHT 	        (10)
#define WEEK_STYLE2_X_COOR 	        (9)
#define WEEK_STYLE2_Y_COOR 	        (139)//128

#define DAY_STYLE2_WIDTH 	        (9)
#define DAY_STYLE2_HEIGHT 	        (11)
#define MONTH_STYLE2_L_X_COOR 	    (19)//14
#define MONTH_STYLE2_L_Y_COOR 	    (125)
#define MONTH_STYLE2_H_X_COOR 	    (9)//6
#define MONTH_STYLE2_H_Y_COOR 	    (125)
#define BAR_STYLE2_WIDTH 	        (5)
#define BAR_STYLE2_HEIGHT 	        (11)
#define BAR_STYLE2_X_COOR 	        (29)//22
#define BAR_STYLE2_Y_COOR 	        (125)//145
#define DAY_STYLE2_L_X_COOR 	    (45)//36
#define DAY_STYLE2_L_Y_COOR 	    (125)//145
#define DAY_STYLE2_H_X_COOR 	    (35)//28
#define DAY_STYLE2_H_Y_COOR 	    (125)//145


const unsigned char *style2_time_pic[10] = {gImage_style2_time0, gImage_style2_time1, gImage_style2_time2, gImage_style2_time3, gImage_style2_time4,
    gImage_style2_time5, gImage_style2_time6, gImage_style2_time7, gImage_style2_time8, gImage_style2_time9};

const unsigned char *style2_date_pic[10] = {gImage_style2_day0, gImage_style2_day1, gImage_style2_day2, gImage_style2_day3, gImage_style2_day4,
    gImage_style2_day5, gImage_style2_day6, gImage_style2_day7, gImage_style2_day8, gImage_style2_day9};

const unsigned char *style2_week_pic[7] = {gImage_style2_week7, gImage_style2_week1, gImage_style2_week2, gImage_style2_week3, gImage_style2_week4,
    gImage_style2_week5, gImage_style2_week6};

const unsigned char *style2_battery_pic[4] = {gImage_style2_batt25, gImage_style2_batt50, gImage_style2_batt75, gImage_style2_batt100};