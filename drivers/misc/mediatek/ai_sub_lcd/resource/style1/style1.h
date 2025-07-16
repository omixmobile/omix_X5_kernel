#include "style1_time0.h"
#include "style1_time1.h"
#include "style1_time2.h"
#include "style1_time3.h"
#include "style1_time4.h"
#include "style1_time5.h"
#include "style1_time6.h"
#include "style1_time7.h"
#include "style1_time8.h"
#include "style1_time9.h"

#include "style1_day0.h"
#include "style1_day1.h"
#include "style1_day2.h"
#include "style1_day3.h"
#include "style1_day4.h"
#include "style1_day5.h"
#include "style1_day6.h"
#include "style1_day7.h"
#include "style1_day8.h"
#include "style1_day9.h"

#include "style1_week1.h"
#include "style1_week2.h"
#include "style1_week3.h"
#include "style1_week4.h"
#include "style1_week5.h"
#include "style1_week6.h"
#include "style1_week7.h"

#include "style1_batt25.h"
#include "style1_batt50.h"
#include "style1_batt75.h"
#include "style1_batt100.h"

#include "style1_bar.h"


#define HOUR_STYLE1_WIDTH 	        (18)
#define HOUR_STYLE1_HEIGHT 	        (52)
#define HOUR_STYLE1_L_X_COOR 	    (32)
#define HOUR_STYLE1_L_Y_COOR 	    (16)//14
#define HOUR_STYLE1_H_X_COOR 	    (8)
#define HOUR_STYLE1_H_Y_COOR 	    (16)//14
#define MINUTE_STYLE1_WIDTH 	    (18)
#define MINUTE_STYLE1_HEIGHT 	    (52)
#define MINUTE_STYLE1_L_X_COOR 	    (32)
#define MINUTE_STYLE1_L_Y_COOR 	    (90)
#define MINUTE_STYLE1_H_X_COOR 	    (8)
#define MINUTE_STYLE1_H_Y_COOR 	    (90)

#define BATTERY_STYLE1_WIDTH 	    (14)
#define BATTERY_STYLE1_HEIGHT 	    (30)
#define BATTERY_STYLE1_X_COOR 	    (56)
#define BATTERY_STYLE1_Y_COOR 	    (112)

#define WEEK_STYLE1_WIDTH 	        (10)
#define WEEK_STYLE1_HEIGHT 	        (31)
#define WEEK_STYLE1_X_COOR 	        (57)//22
#define WEEK_STYLE1_Y_COOR 	        (69)//102

#define DAY_STYLE1_WIDTH 	        (11)
#define DAY_STYLE1_HEIGHT 	        (9)
#define MONTH_STYLE1_L_X_COOR 	    (57)
#define MONTH_STYLE1_L_Y_COOR 	    (42)
#define MONTH_STYLE1_H_X_COOR 	    (57)
#define MONTH_STYLE1_H_Y_COOR 	    (51)
#define BAR_STYLE1_WIDTH 	        (11)
#define BAR_STYLE1_HEIGHT 	        (5)//12
#define BAR_STYLE1_X_COOR 	        (57)
#define BAR_STYLE1_Y_COOR 	        (36)
#define DAY_STYLE1_L_X_COOR 	    (57)
#define DAY_STYLE1_L_Y_COOR 	    (17)
#define DAY_STYLE1_H_X_COOR 	    (57)
#define DAY_STYLE1_H_Y_COOR 	    (26)



const unsigned char *style1_time_pic[10] = {gImage_style1_time0, gImage_style1_time1, gImage_style1_time2, gImage_style1_time3, gImage_style1_time4,
    gImage_style1_time5, gImage_style1_time6, gImage_style1_time7, gImage_style1_time8, gImage_style1_time9};

const unsigned char *style1_date_pic[10] = {gImage_style1_day0, gImage_style1_day1, gImage_style1_day2, gImage_style1_day3, gImage_style1_day4,
    gImage_style1_day5, gImage_style1_day6, gImage_style1_day7, gImage_style1_day8, gImage_style1_day9};

const unsigned char *style1_week_pic[7] = {gImage_style1_week7, gImage_style1_week1, gImage_style1_week2, gImage_style1_week3, gImage_style1_week4,
    gImage_style1_week5, gImage_style1_week6};

const unsigned char *style1_battery_pic[4] = {gImage_style1_batt25, gImage_style1_batt50, gImage_style1_batt75, gImage_style1_batt100};