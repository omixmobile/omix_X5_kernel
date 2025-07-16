#include "style4_time0.h"
#include "style4_time1.h"
#include "style4_time2.h"
#include "style4_time3.h"
#include "style4_time4.h"
#include "style4_time5.h"
#include "style4_time6.h"
#include "style4_time7.h"
#include "style4_time8.h"
#include "style4_time9.h"

#include "style4_day0.h"
#include "style4_day1.h"
#include "style4_day2.h"
#include "style4_day3.h"
#include "style4_day4.h"
#include "style4_day5.h"
#include "style4_day6.h"
#include "style4_day7.h"
#include "style4_day8.h"
#include "style4_day9.h"

#include "style4_week1.h"
#include "style4_week2.h"
#include "style4_week3.h"
#include "style4_week4.h"
#include "style4_week5.h"
#include "style4_week6.h"
#include "style4_week7.h"

#include "style4_batt25.h"
#include "style4_batt50.h"
#include "style4_batt75.h"
#include "style4_batt100.h"

#include "style4_bar.h"


#define HOUR_STYLE4_WIDTH 	        (18)
#define HOUR_STYLE4_HEIGHT 	        (52)
#define HOUR_STYLE4_L_X_COOR 	    (40)
#define HOUR_STYLE4_L_Y_COOR 	    (6)//5 low bit
#define HOUR_STYLE4_H_X_COOR 	    (17)
#define HOUR_STYLE4_H_Y_COOR 	    (6)//5 high bit
#define MINUTE_STYLE4_WIDTH 	    (18)
#define MINUTE_STYLE4_HEIGHT 	    (52)
#define MINUTE_STYLE4_L_X_COOR 	    (40)
#define MINUTE_STYLE4_L_Y_COOR 	    (80)
#define MINUTE_STYLE4_H_X_COOR 	    (17)
#define MINUTE_STYLE4_H_Y_COOR 	    (80)

#define BATTERY_STYLE4_WIDTH 	    (30)
#define BATTERY_STYLE4_HEIGHT 	    (14)
#define BATTERY_STYLE4_X_COOR 	    (41)//18
#define BATTERY_STYLE4_Y_COOR 	    (139)//138

#define WEEK_STYLE4_WIDTH 	        (31)
#define WEEK_STYLE4_HEIGHT 	        (10)
#define WEEK_STYLE4_X_COOR 	        (7)//63
#define WEEK_STYLE4_Y_COOR 	        (141)//139

#define DAY_STYLE4_WIDTH 	        (9)
#define DAY_STYLE4_HEIGHT 	        (11)
#define MONTH_STYLE4_L_X_COOR 	    (26)//2
#define MONTH_STYLE4_L_Y_COOR 	    (64)
#define MONTH_STYLE4_H_X_COOR 	    (17)//2
#define MONTH_STYLE4_H_Y_COOR 	    (64)
#define BAR_STYLE4_WIDTH 	        (5)
#define BAR_STYLE4_HEIGHT 	        (11)
#define BAR_STYLE4_X_COOR 	        (36)//2
#define BAR_STYLE4_Y_COOR 	        (64)
#define DAY_STYLE4_L_X_COOR 	    (51)//2
#define DAY_STYLE4_L_Y_COOR 	    (64)
#define DAY_STYLE4_H_X_COOR 	    (42)//2
#define DAY_STYLE4_H_Y_COOR 	    (64)


const unsigned char *style4_time_pic[10] = {gImage_style4_time0, gImage_style4_time1, gImage_style4_time2, gImage_style4_time3, gImage_style4_time4,
    gImage_style4_time5, gImage_style4_time6, gImage_style4_time7, gImage_style4_time8, gImage_style4_time9};

const unsigned char *style4_date_pic[10] = {gImage_style4_day0, gImage_style4_day1, gImage_style4_day2, gImage_style4_day3, gImage_style4_day4,
    gImage_style4_day5, gImage_style4_day6, gImage_style4_day7, gImage_style4_day8, gImage_style4_day9};

const unsigned char *style4_week_pic[7] = {gImage_style4_week7, gImage_style4_week1, gImage_style4_week2, gImage_style4_week3, gImage_style4_week4,
    gImage_style4_week5, gImage_style4_week6};

const unsigned char *style4_battery_pic[4] = {gImage_style4_batt25, gImage_style4_batt50, gImage_style4_batt75, gImage_style4_batt100};