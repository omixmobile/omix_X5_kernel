//Antaiui <AI_BSP_LCD> <zhuli> <2022-06-28> add for sub lcd display style begin
#include "resource/style1/style1.h"
#include "resource/style2/style2.h"
#include "resource/style3/style3.h"
#include "resource/style4/style4.h"

#include "resource/bg1.h"
#include "resource/bg2.h"
#include "resource/bg3.h"
#include "resource/bg4.h"



#define MEMORY_DEFAULT_SIZE 	    (80*160*2)
#define MEMORY_WIDTH 	            (80)





struct common_region
{
    char 	x_coordinate;
	char	y_coordinate;
	char 	width;
	char	height;
};

struct time_region
{
    unsigned char 	hour_h_x_coordinate;
	unsigned char	hour_h_y_coordinate;
    unsigned char 	hour_l_x_coordinate;
	unsigned char	hour_l_y_coordinate;
	unsigned char 	hour_width;
	unsigned char	hour_height;
    unsigned char 	minute_h_x_coordinate;
	unsigned char	minute_h_y_coordinate;
    unsigned char 	minute_l_x_coordinate;
	unsigned char	minute_l_y_coordinate;
	unsigned char 	minute_width;
	unsigned char	minute_height;
};

struct date_region
{
	unsigned char 	month_width;
	unsigned char	month_height;
    unsigned char 	month_h_x_coordinate;
	unsigned char	month_h_y_coordinate;
    unsigned char 	month_l_x_coordinate;
	unsigned char	month_l_y_coordinate;
	unsigned char 	bar_width;
	unsigned char	bar_height;
    unsigned char 	bar_x_coordinate;
	unsigned char	bar_y_coordinate;
    unsigned char 	day_h_x_coordinate;
	unsigned char	day_h_y_coordinate;
    unsigned char 	day_l_x_coordinate;
	unsigned char	day_l_y_coordinate;
};

struct display_element
{
	unsigned char 	value;
	bool is_update;
};

struct display_content
{
	//unsigned char 	minute;
	struct display_element month;
	struct display_element day;
	struct display_element hour;
    struct display_element minute;
	struct display_element week;
	struct display_element battery_level;
	struct mutex buf_lock;
};


struct display_ctrl
{
    bool is_sleep;
    bool is_draw_refresh;
    bool is_pattern_refresh;
    unsigned char pattern_sele;
};


static ssize_t get_display_pattern(struct device* cd,struct device_attribute *attr, char* buf);
static ssize_t set_display_pattern(struct device* cd, struct device_attribute *attr, const char* buf, size_t len);
static ssize_t get_display_battery(struct device* cd,struct device_attribute *attr, char* buf);
static ssize_t set_display_battery(struct device* cd, struct device_attribute *attr, const char* buf, size_t len);
static ssize_t get_display_doze(struct device* cd,struct device_attribute *attr, char* buf);
static ssize_t set_display_doze(struct device* cd, struct device_attribute *attr, const char* buf, size_t len);



extern void ai_sublcd_gc9106_frame_interface(unsigned char *sbuf);
extern void ai_sublcd_gc9106_sleep_interface(bool sleep);
//Antaiui <AI_BSP_LCD> <zhuli> <2022-06-28> add for sub lcd display style end