#include "style3_up0.h"
#include "style3_up1.h"
#include "style3_up2.h"
#include "style3_up3.h"
#include "style3_up4.h"
#include "style3_up5.h"
#include "style3_up6.h"
#include "style3_up7.h"
#include "style3_up8.h"
#include "style3_up9.h"
#include "style3_down0.h"
#include "style3_down1.h"
#include "style3_down2.h"
#include "style3_down3.h"
#include "style3_down4.h"
#include "style3_down5.h"
#include "style3_down6.h"
#include "style3_down7.h"
#include "style3_down8.h"
#include "style3_down9.h"

#include "style3_day0.h"
#include "style3_day1.h"
#include "style3_day2.h"
#include "style3_day3.h"
#include "style3_day4.h"
#include "style3_day5.h"
#include "style3_day6.h"
#include "style3_day7.h"
#include "style3_day8.h"
#include "style3_day9.h"

#include "style3_week1.h"
#include "style3_week2.h"
#include "style3_week3.h"
#include "style3_week4.h"
#include "style3_week5.h"
#include "style3_week6.h"
#include "style3_week7.h"

#include "style3_batt25.h"
#include "style3_batt50.h"
#include "style3_batt75.h"
#include "style3_batt100.h"

#include "style3_bar.h"


#define HOUR_STYLE3_WIDTH 	        (16)
#define HOUR_STYLE3_HEIGHT 	        (48)
#define HOUR_STYLE3_L_X_COOR 	    (51)
#define HOUR_STYLE3_L_Y_COOR 	    (28)
#define HOUR_STYLE3_H_X_COOR 	    (29)
#define HOUR_STYLE3_H_Y_COOR 	    (28)
#define MINUTE_STYLE3_WIDTH 	    (24)
#define MINUTE_STYLE3_HEIGHT 	    (69)
#define MINUTE_STYLE3_L_X_COOR 	    (43)
#define MINUTE_STYLE3_L_Y_COOR 	    (81)
#define MINUTE_STYLE3_H_X_COOR 	    (11)
#define MINUTE_STYLE3_H_Y_COOR 	    (81)

#define BATTERY_STYLE3_WIDTH 	    (30)
#define BATTERY_STYLE3_HEIGHT 	    (14)
#define BATTERY_STYLE3_X_COOR 	    (41)//12
#define BATTERY_STYLE3_Y_COOR 	    (9)//22

#define WEEK_STYLE3_WIDTH 	        (31)
#define WEEK_STYLE3_HEIGHT 	        (10)
#define WEEK_STYLE3_X_COOR 	        (8)//42
#define WEEK_STYLE3_Y_COOR 	        (11)//9

#define DAY_STYLE3_WIDTH 	        (11)
#define DAY_STYLE3_HEIGHT 	        (9)
#define MONTH_STYLE3_L_X_COOR 	    (12)//11
#define MONTH_STYLE3_L_Y_COOR 	    (54)
#define MONTH_STYLE3_H_X_COOR 	    (12)//3
#define MONTH_STYLE3_H_Y_COOR 	    (63)
#define BAR_STYLE3_WIDTH 	        (11)
#define BAR_STYLE3_HEIGHT 	        (5)
#define BAR_STYLE3_X_COOR 	        (12)//20
#define BAR_STYLE3_Y_COOR 	        (48)//60
#define DAY_STYLE3_L_X_COOR 	    (12)//31
#define DAY_STYLE3_L_Y_COOR 	    (29)//60
#define DAY_STYLE3_H_X_COOR 	    (12)//23
#define DAY_STYLE3_H_Y_COOR 	    (38)//60


const unsigned char *style3_minute_pic[10] = {gImage_style3_down0, gImage_style3_down1, gImage_style3_down2, gImage_style3_down3, gImage_style3_down4,
    gImage_style3_down5, gImage_style3_down6, gImage_style3_down7, gImage_style3_down8, gImage_style3_down9};
const unsigned char *style3_hour_pic[10] = {gImage_style3_up0, gImage_style3_up1, gImage_style3_up2, gImage_style3_up3, gImage_style3_up4,
    gImage_style3_up5, gImage_style3_up6, gImage_style3_up7, gImage_style3_up8, gImage_style3_up9};

const unsigned char *style3_date_pic[10] = {gImage_style3_day0, gImage_style3_day1, gImage_style3_day2, gImage_style3_day3, gImage_style3_day4,
    gImage_style3_day5, gImage_style3_day6, gImage_style3_day7, gImage_style3_day8, gImage_style3_day9};

const unsigned char *style3_week_pic[7] = {gImage_style3_week7, gImage_style3_week1, gImage_style3_week2, gImage_style3_week3, gImage_style3_week4,
    gImage_style3_week5, gImage_style3_week6};

const unsigned char *style3_battery_pic[4] = {gImage_style3_batt25, gImage_style3_batt50, gImage_style3_batt75, gImage_style3_batt100};