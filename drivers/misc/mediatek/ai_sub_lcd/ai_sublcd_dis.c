//Antaiui <AI_BSP_LCD> <zhuli> <2022-06-28> add for sub lcd display style begin
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <asm/io.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/timex.h>
#include <linux/rtc.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>


#include <ai_sublcd_dis.h>


#define DRAWREGION(X, Y, W, H) \
    for(i=0; i< H; i++) \
        memcpy(sub_fb + (X+(Y+i)*MEMORY_WIDTH)*2, pic + (W*2*i), W*2);



unsigned char sub_fb[MEMORY_DEFAULT_SIZE];
unsigned char battery_level = 0;

const unsigned char *p_pic_bg[4] = {gImage_bg1, gImage_bg2, gImage_bg3, gImage_bg4};
const unsigned char *p_pic_bar[4] = {gImage_style1_bar, gImage_style2_bar, gImage_style3_bar, gImage_style4_bar};
const unsigned char **p_pic_hour = style4_time_pic;
const unsigned char **p_pic_minute = style4_time_pic;
const unsigned char **p_pic_date = style4_date_pic;
const unsigned char **p_pic_week = style4_week_pic;
const unsigned char **p_pic_battery = style4_battery_pic;


static struct time_region st_time;
static struct date_region  st_date;
static struct common_region st_battery;
static struct common_region st_week;
static struct display_content st_display;
//Antaiui <AI_BSP_LCD> <zhuli> <2022-07-20> refine sub lcd display begin
static struct display_ctrl st_ctrl;
//Antaiui <AI_BSP_LCD> <zhuli> <2022-07-20> refine sub lcd display end

static struct rtc_time real_tm;

static struct workqueue_struct *queue = NULL;
static struct delayed_work   d_work;

char week[7][10] = { "Sunday","Monday","Tuesday","Wednesday","Thursday","Friday","Saturday" };
//Antaiui <AI_BSP_LCD> <zhuli> <2022-06-28> add for sub lcd display style end




static DEVICE_ATTR(disp_patt, 0660, get_display_pattern,  set_display_pattern);
static DEVICE_ATTR(disp_batt, 0660, get_display_battery,  set_display_battery);
static DEVICE_ATTR(disp_doze, 0660, get_display_doze,  set_display_doze);

//Antaiui <AI_BSP_LCD> <zhuli> <2022-06-28> add for sub lcd display pattern begin
static ssize_t get_display_pattern(struct device* cd,struct device_attribute *attr, char* buf)
{
    pr_info("sublcd  %s , pattern:%d  \n",__func__, st_ctrl.pattern_sele);

    return sprintf(buf, " pattern:%d \n", st_ctrl.pattern_sele);
}

static ssize_t set_display_pattern(struct device* cd, struct device_attribute *attr, const char* buf, size_t len)
{
	int cmd = 0;

    kstrtos32(buf, 10, &cmd);
    pr_info("sublcd  %s , buf: %s, cmd:%d   \n",__func__, buf, cmd);
    if((cmd < 1) || (cmd > 4))
    {
        printk("sublcd cmd is invalid! \n");
        cmd = 1;
    }

    cancel_delayed_work(&d_work);
    st_ctrl.pattern_sele = cmd;
    memset(sub_fb, 0x00, MEMORY_DEFAULT_SIZE);
    st_ctrl.is_pattern_refresh = true;
    st_ctrl.is_draw_refresh = true;
    queue_delayed_work(queue, &d_work, 0);//HZ/5

    return len;
}
//Antaiui <AI_BSP_LCD> <zhuli> <2022-06-28> add for sub lcd display pattern end

//Antaiui <AI_BSP_LCD> <zhuli> <2022-06-28> add for sub lcd display battery begin
static ssize_t get_display_battery(struct device* cd,struct device_attribute *attr, char* buf)
{
	pr_info("sublcd  %s , battery:%d  \n",__func__, battery_level);

    return sprintf(buf, " battery:%d \n", battery_level);
}

static ssize_t set_display_battery(struct device* cd, struct device_attribute *attr, const char* buf, size_t len)
{
	int cmd = 0;
    kstrtos32(buf, 10, &cmd);
    pr_info("sublcd  %s , buf: %s, cmd:%d   \n",__func__, buf, cmd);
    if((cmd < 1) || (cmd > 10))
        printk("sublcd cmd is invalid! \n");
    battery_level = cmd;

    return len;
}
//Antaiui <AI_BSP_LCD> <zhuli> <2022-06-28> add for sub lcd display battery end

//Antaiui <AI_BSP_LCD> <zhuli> <2022-06-28> add for sub lcd display doze begin
static ssize_t get_display_doze(struct device* cd,struct device_attribute *attr, char* buf)
{
	pr_info("sublcd  %s , doze:%d  \n",__func__, st_ctrl.is_sleep);

    return sprintf(buf, " doze:%d \n", st_ctrl.is_sleep);
}

static ssize_t set_display_doze(struct device* cd, struct device_attribute *attr, const char* buf, size_t len)
{
	int cmd = 0;
    kstrtos32(buf, 10, &cmd);
    pr_info("sublcd  %s , buf: %s, cmd:%d   \n",__func__, buf, cmd);
    if(cmd == 1) {
        //Antaiui <AI_BSP_LCD> <zhuli> <2022-07-13> sub lcd wake up when sleep in hardware test begin
        if(!st_ctrl.is_sleep) {
            ai_sublcd_gc9106_sleep_interface(true);
            st_ctrl.is_sleep = true;
            cancel_delayed_work(&d_work);
        }
        //Antaiui <AI_BSP_LCD> <zhuli> <2022-07-13> sub lcd wake up when sleep in hardware test end
    }else if(cmd == 0) {
        cancel_delayed_work(&d_work);
        ai_sublcd_gc9106_sleep_interface(false);
        memset(sub_fb, 0x00, MEMORY_DEFAULT_SIZE);
        st_ctrl.is_draw_refresh = true;
        queue_delayed_work(queue, &d_work, 0);
    }

    return len;
}
//Antaiui <AI_BSP_LCD> <zhuli> <2022-06-28> add for sub lcd display doze end


void gc9106_display_create_sysfs(struct platform_device *dev)
{
	device_create_file(&(dev->dev), &dev_attr_disp_patt);
	device_create_file(&(dev->dev), &dev_attr_disp_batt);
	device_create_file(&(dev->dev), &dev_attr_disp_doze);
}



/*
蔡勒(Zeller)公式：是一个计算星期的公式
w=(y+[y/4]+[c/4]-2c+[26(m+1)/10]+d-1)mod 7

公式细节
w：星期； w对7取模得：0-星期日，1-星期一，2-星期二，3-星期三，4-星期四，5-星期五，6-星期六
c：世纪（注：一般情况下，在公式中取值为已经过的世纪数，也就是年份除以一百的结果，而非正在进行的世纪，也就是现在常用的年份除以一百加一；不过如果年份是公元前的年份且非整百数的话，c应该等于所在世纪的编号，如公元前253年，是公元前3世纪，c就等于-3）
y：年（一般情况下是后两位数，如果是公元前的年份且非整百数，y应该等于cMOD100+100）
m：月（m大于等于3，小于等于14，即在蔡勒公式中，某年的1、2月要看作上一年的13、14月来计算，比如2003年1月1日要看作2002年的13月1日来计算）
d：日
*/

static int return_week( unsigned int year, unsigned int month, unsigned int day )
{
    int week = 0;
    unsigned int y = 0, c = 0, m = 0, d = 0;

    if ( month == 1 || month == 2 )
    {
        c = ( year - 1 ) / 100;
        y = ( year - 1 ) % 100;
        m = month + 12;
        d = day;
    }
    else
    {
        c = year / 100;
        y = year % 100;
        m = month;
        d = day;
    }

    week = y + y / 4 + c / 4 - 2 * c + 26 * ( m + 1 ) / 10 + d - 1; //蔡勒公式
    week = week >= 0 ? ( week % 7 ) : ( week % 7 + 7 ); //week为负时取模

    return week;
}

//Antaiui <AI_BSP_LCD> <zhuli> <2022-06-28> add for sub lcd display region begin
static void draw_region_time(void)
{
    unsigned char i;
    const unsigned char *pic = NULL;

    //printk("TTT real time :  %d:%d h-%d:%d m-%d:%d \n", real_tm.tm_hour, real_tm.tm_min, real_tm.tm_hour/10, real_tm.tm_hour%10, real_tm.tm_min/10, real_tm.tm_min%10);

    if (st_display.hour.is_update)
    {
        pic = p_pic_hour[st_display.hour.value/10];
        DRAWREGION(st_time.hour_h_x_coordinate, st_time.hour_h_y_coordinate, st_time.hour_width, st_time.hour_height)
        pic = p_pic_hour[st_display.hour.value%10];
        DRAWREGION(st_time.hour_l_x_coordinate, st_time.hour_l_y_coordinate, st_time.hour_width, st_time.hour_height)
        st_display.hour.is_update = false;
    }

    if (st_display.minute.is_update)
    {
    //分钟高位
    pic = p_pic_minute[real_tm.tm_min/10];
    DRAWREGION(st_time.minute_h_x_coordinate, st_time.minute_h_y_coordinate, st_time.minute_width, st_time.minute_height)

    //分钟低位
    pic = p_pic_minute[real_tm.tm_min%10];
    DRAWREGION(st_time.minute_l_x_coordinate, st_time.minute_l_y_coordinate, st_time.minute_width, st_time.minute_height)
        st_display.minute.is_update = false;
    }
}

static void draw_region_week(void)
{
    unsigned char i;
    const unsigned char *pic = p_pic_week[st_display.week.value];

    if (st_display.week.is_update)
    {
        DRAWREGION(st_week.x_coordinate, st_week.y_coordinate, st_week.width, st_week.height)
        st_display.week.is_update = false;
    }
}

static void draw_region_date(void)
{
    unsigned char i;
    const unsigned char *pic = p_pic_date[st_display.month.value/10];

    //month
    if (st_display.month.is_update)
    {
        DRAWREGION(st_date.month_h_x_coordinate, st_date.month_h_y_coordinate, st_date.month_width, st_date.month_height);
        pic = p_pic_date[st_display.month.value%10];
        DRAWREGION(st_date.month_l_x_coordinate, st_date.month_l_y_coordinate, st_date.month_width, st_date.month_height);
        st_display.month.is_update = false;
    }

    //bar
    if (st_ctrl.is_draw_refresh)
    {
        pic = p_pic_bar[st_ctrl.pattern_sele-1];
        DRAWREGION(st_date.bar_x_coordinate, st_date.bar_y_coordinate, st_date.bar_width, st_date.bar_height);
        //st_ctrl.is_draw_refresh = false;
    }

    //day
    if (st_display.day.is_update)
    {
        pic = p_pic_date[st_display.day.value/10];
        DRAWREGION(st_date.day_h_x_coordinate, st_date.day_h_y_coordinate, st_date.month_width, st_date.month_height);
        pic = p_pic_date[st_display.day.value%10];
        DRAWREGION(st_date.day_l_x_coordinate, st_date.day_l_y_coordinate, st_date.month_width, st_date.month_height);
        st_display.day.is_update = false;
    }
}

static void draw_region_battery(void)
{
    unsigned char i;
    const unsigned char *pic = p_pic_battery[st_display.battery_level.value];

    if (st_display.battery_level.is_update)
    {
        DRAWREGION(st_battery.x_coordinate, st_battery.y_coordinate, st_battery.width, st_battery.height)
        st_display.battery_level.is_update = false;
    }

}

static void draw_style_select(void)
{
    if (!st_ctrl.is_pattern_refresh)
        return;

    if(4 < st_ctrl.pattern_sele)
    {
        printk("sublcd pattern is not support!! :%d \n", st_ctrl.pattern_sele);
        st_ctrl.pattern_sele = 1;
    }
    switch(st_ctrl.pattern_sele)
    {
        case 4:
            p_pic_minute = style4_time_pic;
            p_pic_hour = style4_time_pic;
            p_pic_date = style4_date_pic;
            p_pic_week = style4_week_pic;
            p_pic_battery = style4_battery_pic;

            st_time.hour_width = HOUR_STYLE4_WIDTH;
            st_time.hour_height = HOUR_STYLE4_HEIGHT;
            st_time.hour_h_x_coordinate = HOUR_STYLE4_H_X_COOR;
            st_time.hour_h_y_coordinate = HOUR_STYLE4_H_Y_COOR;
            st_time.hour_l_x_coordinate = HOUR_STYLE4_L_X_COOR;
            st_time.hour_l_y_coordinate = HOUR_STYLE4_L_Y_COOR;
            st_time.minute_width = MINUTE_STYLE4_WIDTH;
            st_time.minute_height = MINUTE_STYLE4_HEIGHT;
            st_time.minute_h_x_coordinate = MINUTE_STYLE4_H_X_COOR;
            st_time.minute_h_y_coordinate = MINUTE_STYLE4_H_Y_COOR;
            st_time.minute_l_x_coordinate = MINUTE_STYLE4_L_X_COOR;
            st_time.minute_l_y_coordinate = MINUTE_STYLE4_L_Y_COOR;

            st_date.month_width = DAY_STYLE4_WIDTH;
            st_date.month_height = DAY_STYLE4_HEIGHT;
            st_date.month_h_x_coordinate = MONTH_STYLE4_H_X_COOR;
            st_date.month_h_y_coordinate = MONTH_STYLE4_H_Y_COOR;
            st_date.month_l_x_coordinate = MONTH_STYLE4_L_X_COOR;
            st_date.month_l_y_coordinate = MONTH_STYLE4_L_Y_COOR;
            st_date.bar_width = BAR_STYLE4_WIDTH;
            st_date.bar_height = BAR_STYLE4_HEIGHT;
            st_date.bar_x_coordinate = BAR_STYLE4_X_COOR;
            st_date.bar_y_coordinate = BAR_STYLE4_Y_COOR;
            st_date.day_h_x_coordinate = DAY_STYLE4_H_X_COOR;
            st_date.day_h_y_coordinate = DAY_STYLE4_H_Y_COOR;
            st_date.day_l_x_coordinate = DAY_STYLE4_L_X_COOR;
            st_date.day_l_y_coordinate = DAY_STYLE4_L_Y_COOR;

            st_battery.width = BATTERY_STYLE4_WIDTH;
            st_battery.height = BATTERY_STYLE4_HEIGHT;
            st_battery.x_coordinate = BATTERY_STYLE4_X_COOR;
            st_battery.y_coordinate = BATTERY_STYLE4_Y_COOR;

            st_week.width = WEEK_STYLE4_WIDTH;
            st_week.height = WEEK_STYLE4_HEIGHT;
            st_week.x_coordinate = WEEK_STYLE4_X_COOR;
            st_week.y_coordinate = WEEK_STYLE4_Y_COOR;
            break;

        case 3:
            p_pic_minute = style3_minute_pic;
            p_pic_hour = style3_hour_pic;
            p_pic_date = style3_date_pic;
            p_pic_week = style3_week_pic;
            p_pic_battery = style3_battery_pic;

            st_time.hour_width = HOUR_STYLE3_WIDTH;
            st_time.hour_height = HOUR_STYLE3_HEIGHT;
            st_time.hour_h_x_coordinate = HOUR_STYLE3_H_X_COOR;
            st_time.hour_h_y_coordinate = HOUR_STYLE3_H_Y_COOR;
            st_time.hour_l_x_coordinate = HOUR_STYLE3_L_X_COOR;
            st_time.hour_l_y_coordinate = HOUR_STYLE3_L_Y_COOR;
            st_time.minute_width = MINUTE_STYLE3_WIDTH;
            st_time.minute_height = MINUTE_STYLE3_HEIGHT;
            st_time.minute_h_x_coordinate = MINUTE_STYLE3_H_X_COOR;
            st_time.minute_h_y_coordinate = MINUTE_STYLE3_H_Y_COOR;
            st_time.minute_l_x_coordinate = MINUTE_STYLE3_L_X_COOR;
            st_time.minute_l_y_coordinate = MINUTE_STYLE3_L_Y_COOR;

            st_date.month_width = DAY_STYLE3_WIDTH;
            st_date.month_height = DAY_STYLE3_HEIGHT;
            st_date.month_h_x_coordinate = MONTH_STYLE3_H_X_COOR;
            st_date.month_h_y_coordinate = MONTH_STYLE3_H_Y_COOR;
            st_date.month_l_x_coordinate = MONTH_STYLE3_L_X_COOR;
            st_date.month_l_y_coordinate = MONTH_STYLE3_L_Y_COOR;
            st_date.bar_width = BAR_STYLE3_WIDTH;
            st_date.bar_height = BAR_STYLE3_HEIGHT;
            st_date.bar_x_coordinate = BAR_STYLE3_X_COOR;
            st_date.bar_y_coordinate = BAR_STYLE3_Y_COOR;
            st_date.day_h_x_coordinate = DAY_STYLE3_H_X_COOR;
            st_date.day_h_y_coordinate = DAY_STYLE3_H_Y_COOR;
            st_date.day_l_x_coordinate = DAY_STYLE3_L_X_COOR;
            st_date.day_l_y_coordinate = DAY_STYLE3_L_Y_COOR;

            st_battery.width = BATTERY_STYLE3_WIDTH;
            st_battery.height = BATTERY_STYLE3_HEIGHT;
            st_battery.x_coordinate = BATTERY_STYLE3_X_COOR;
            st_battery.y_coordinate = BATTERY_STYLE3_Y_COOR;

            st_week.width = WEEK_STYLE3_WIDTH;
            st_week.height = WEEK_STYLE3_HEIGHT;
            st_week.x_coordinate = WEEK_STYLE3_X_COOR;
            st_week.y_coordinate = WEEK_STYLE3_Y_COOR;
            break;

        case 2:
            p_pic_minute = style2_time_pic;
            p_pic_hour = style2_time_pic;
            p_pic_date = style2_date_pic;
            p_pic_week = style2_week_pic;
            p_pic_battery = style2_battery_pic;

            st_time.hour_width = HOUR_STYLE2_WIDTH;
            st_time.hour_height = HOUR_STYLE2_HEIGHT;
            st_time.hour_h_x_coordinate = HOUR_STYLE2_H_X_COOR;
            st_time.hour_h_y_coordinate = HOUR_STYLE2_H_Y_COOR;
            st_time.hour_l_x_coordinate = HOUR_STYLE2_L_X_COOR;
            st_time.hour_l_y_coordinate = HOUR_STYLE2_L_Y_COOR;
            st_time.minute_width = MINUTE_STYLE2_WIDTH;
            st_time.minute_height = MINUTE_STYLE2_HEIGHT;
            st_time.minute_h_x_coordinate = MINUTE_STYLE2_H_X_COOR;
            st_time.minute_h_y_coordinate = MINUTE_STYLE2_H_Y_COOR;
            st_time.minute_l_x_coordinate = MINUTE_STYLE2_L_X_COOR;
            st_time.minute_l_y_coordinate = MINUTE_STYLE2_L_Y_COOR;

            st_date.month_width = DAY_STYLE2_WIDTH;
            st_date.month_height = DAY_STYLE2_HEIGHT;
            st_date.month_h_x_coordinate = MONTH_STYLE2_H_X_COOR;
            st_date.month_h_y_coordinate = MONTH_STYLE2_H_Y_COOR;
            st_date.month_l_x_coordinate = MONTH_STYLE2_L_X_COOR;
            st_date.month_l_y_coordinate = MONTH_STYLE2_L_Y_COOR;
            st_date.bar_width = BAR_STYLE2_WIDTH;
            st_date.bar_height = BAR_STYLE2_HEIGHT;
            st_date.bar_x_coordinate = BAR_STYLE2_X_COOR;
            st_date.bar_y_coordinate = BAR_STYLE2_Y_COOR;
            st_date.day_h_x_coordinate = DAY_STYLE2_H_X_COOR;
            st_date.day_h_y_coordinate = DAY_STYLE2_H_Y_COOR;
            st_date.day_l_x_coordinate = DAY_STYLE2_L_X_COOR;
            st_date.day_l_y_coordinate = DAY_STYLE2_L_Y_COOR;

            st_battery.width = BATTERY_STYLE2_WIDTH;
            st_battery.height = BATTERY_STYLE2_HEIGHT;
            st_battery.x_coordinate = BATTERY_STYLE2_X_COOR;
            st_battery.y_coordinate = BATTERY_STYLE2_Y_COOR;

            st_week.width = WEEK_STYLE2_WIDTH;
            st_week.height = WEEK_STYLE2_HEIGHT;
            st_week.x_coordinate = WEEK_STYLE2_X_COOR;
            st_week.y_coordinate = WEEK_STYLE2_Y_COOR;
            break;

        case 1:
        default:
            p_pic_minute = style1_time_pic;
            p_pic_hour = style1_time_pic;
            p_pic_date = style1_date_pic;
            p_pic_week = style1_week_pic;
            p_pic_battery = style1_battery_pic;

            st_time.hour_width = HOUR_STYLE1_WIDTH;
            st_time.hour_height = HOUR_STYLE1_HEIGHT;
            st_time.hour_h_x_coordinate = HOUR_STYLE1_H_X_COOR;
            st_time.hour_h_y_coordinate = HOUR_STYLE1_H_Y_COOR;
            st_time.hour_l_x_coordinate = HOUR_STYLE1_L_X_COOR;
            st_time.hour_l_y_coordinate = HOUR_STYLE1_L_Y_COOR;
            st_time.minute_width = MINUTE_STYLE1_WIDTH;
            st_time.minute_height = MINUTE_STYLE1_HEIGHT;
            st_time.minute_h_x_coordinate = MINUTE_STYLE1_H_X_COOR;
            st_time.minute_h_y_coordinate = MINUTE_STYLE1_H_Y_COOR;
            st_time.minute_l_x_coordinate = MINUTE_STYLE1_L_X_COOR;
            st_time.minute_l_y_coordinate = MINUTE_STYLE1_L_Y_COOR;

            st_date.month_width = DAY_STYLE1_WIDTH;
            st_date.month_height = DAY_STYLE1_HEIGHT;
            st_date.month_h_x_coordinate = MONTH_STYLE1_H_X_COOR;
            st_date.month_h_y_coordinate = MONTH_STYLE1_H_Y_COOR;
            st_date.month_l_x_coordinate = MONTH_STYLE1_L_X_COOR;
            st_date.month_l_y_coordinate = MONTH_STYLE1_L_Y_COOR;
            st_date.bar_width = BAR_STYLE1_WIDTH;
            st_date.bar_height = BAR_STYLE1_HEIGHT;
            st_date.bar_x_coordinate = BAR_STYLE1_X_COOR;
            st_date.bar_y_coordinate = BAR_STYLE1_Y_COOR;
            st_date.day_h_x_coordinate = DAY_STYLE1_H_X_COOR;
            st_date.day_h_y_coordinate = DAY_STYLE1_H_Y_COOR;
            st_date.day_l_x_coordinate = DAY_STYLE1_L_X_COOR;
            st_date.day_l_y_coordinate = DAY_STYLE1_L_Y_COOR;

            st_battery.width = BATTERY_STYLE1_WIDTH;
            st_battery.height = BATTERY_STYLE1_HEIGHT;
            st_battery.x_coordinate = BATTERY_STYLE1_X_COOR;
            st_battery.y_coordinate = BATTERY_STYLE1_Y_COOR;

            st_week.width = WEEK_STYLE1_WIDTH;
            st_week.height = WEEK_STYLE1_HEIGHT;
            st_week.x_coordinate = WEEK_STYLE1_X_COOR;
            st_week.y_coordinate = WEEK_STYLE1_Y_COOR;
            break;
    }

    st_ctrl.is_pattern_refresh = false;
}


static void draw_regions(void)
{
    draw_style_select();

    draw_region_time();

    draw_region_week();

    draw_region_date();

    draw_region_battery();
}

static void get_display_content(void)
{
    struct timex  txc;
    int index;
    union power_supply_propval value;

    // get battery soc from external "battery" power supply if support
    struct power_supply *ba_psy = power_supply_get_by_name("battery");

    //pr_info("sublcd  %s , enter \n",__func__);

    if (ba_psy) {
        power_supply_get_property(ba_psy, POWER_SUPPLY_PROP_CAPACITY, &value);
        pr_info("s:get ba_psy success, soc(%d)\n",__func__, value.intval);
        if (75 < value.intval)
        {   if((st_display.battery_level.value != 3) || (st_ctrl.is_draw_refresh))
            {
                st_display.battery_level.value = 3;
                st_display.battery_level.is_update = true;
            }
        }else if(50 < value.intval){
            if((st_display.battery_level.value != 2) || (st_ctrl.is_draw_refresh))
            {
                st_display.battery_level.value = 2;
                st_display.battery_level.is_update = true;
            }
        }else if(25 < value.intval){
           if((st_display.battery_level.value != 1) || (st_ctrl.is_draw_refresh))
            {
                st_display.battery_level.value = 1;
                st_display.battery_level.is_update = true;
            }
        }else{
            if((st_display.battery_level.value != 0) || (st_ctrl.is_draw_refresh))
            {
                st_display.battery_level.value = 0;
                st_display.battery_level.is_update = true;
            }
        }
    }

    do_gettimeofday(&(txc.time));
    txc.time.tv_sec -= sys_tz.tz_minuteswest * 60;
    rtc_time_to_tm(txc.time.tv_sec,&real_tm);
    real_tm.tm_year += 1900;
    real_tm.tm_mon += 1;

    //st_display.minute = real_tm.tm_min;
    if ((st_display.month.value != real_tm.tm_mon) || (st_ctrl.is_draw_refresh))
    {
        st_display.month.value = real_tm.tm_mon;
        st_display.month.is_update = true;
    }
    if ((st_display.day.value != real_tm.tm_mday) || (st_ctrl.is_draw_refresh))
    {
        st_display.day.value = real_tm.tm_mday;
        st_display.day.is_update = true;
    }
    if ((st_display.hour.value != real_tm.tm_hour) || (st_ctrl.is_draw_refresh))
    {
        st_display.hour.value = real_tm.tm_hour;
        st_display.hour.is_update = true;
    }
    if ((st_display.minute.value != real_tm.tm_min) || (st_ctrl.is_draw_refresh))
    {
        st_display.minute.value = real_tm.tm_min;
        st_display.minute.is_update = true;
    }

    index = return_week(real_tm.tm_year, real_tm.tm_mon, real_tm.tm_mday);

    if ((st_display.week.value != index) || (st_ctrl.is_draw_refresh))
    {
        st_display.week.value = index;
        st_display.week.is_update = true;
    }
    printk("sublcd get_display_content UTC time :%d-%d-%d %d:%d:%d week:%s battery percentage:%d \n",real_tm.tm_year, real_tm.tm_mon, real_tm.tm_mday,
        real_tm.tm_hour,real_tm.tm_min,real_tm.tm_sec, week[index], value.intval);

    //pr_info("sublcd  %s , exit \n",__func__);
}
//Antaiui <AI_BSP_LCD> <zhuli> <2022-06-28> add for sub lcd display region end

//Antaiui <AI_BSP_LCD> <zhuli> <2022-06-28> add for sub lcd display workqueue begin
static void work_handler(struct work_struct *data)
{
    //printk("sublcd work handler function enter.\n");
    if(st_ctrl.is_sleep)
    {
        ai_sublcd_gc9106_sleep_interface(false);
    }

    get_display_content();

    if((st_display.month.is_update) || (st_display.day.is_update) || (st_display.hour.is_update)
             || (st_display.minute.is_update) || (st_display.week.is_update)  || (st_display.battery_level.is_update))
    {
        st_ctrl.is_draw_refresh = true;
    }
    printk("sublcd work handler function draw_regions. month:%d, day:%d, hour:%d, minute:%d, week:%d, battery_level:%d, \n",
        st_display.month.is_update, st_display.day.is_update,st_display.hour.is_update,
        st_display.minute.is_update, st_display.week.is_update, st_display.battery_level.is_update);

    draw_regions();

    if(st_ctrl.is_draw_refresh)
    {
        //Antaiui <AI_BSP_LCD> <zhuli> <2022-07-14> spi_sync may sleep and syscall write can enter then, set draw_refresh flag begin
        st_ctrl.is_draw_refresh = false;
        //Antaiui <AI_BSP_LCD> <zhuli> <2022-07-14> spi_sync may sleep and syscall write can enter then, set draw_refresh flag end
        ai_sublcd_gc9106_frame_interface(sub_fb);
        st_ctrl.is_sleep = false;
        printk("sublcd work handler function display sublcd.  \n");
    }
}

//Antaiui <AI_BSP_LCD> <zhuli> <2022-07-04> add for sub lcd test in factory test begin
void cancel_display_work(void)
{
    if(!st_ctrl.is_sleep) {
        cancel_delayed_work(&d_work);
        //st_ctrl.is_sleep = true;
    }
}
//Antaiui <AI_BSP_LCD> <zhuli> <2022-07-04> add for sub lcd test in factory test end

static int __init queue_init(void)
{
    queue = create_singlethread_workqueue("hello");/*创建一个单线程的工作队列*/
    if (!queue)
        goto err;

    INIT_DELAYED_WORK(&d_work,work_handler);
    //Antaiui <AI_BSP_LCD> <zhuli> <2022-07-05> add for sub lcd test in factory test begin
    //queue_delayed_work(queue, &d_work, 30*HZ);
    cancel_delayed_work(&d_work);
    //Antaiui <AI_BSP_LCD> <zhuli> <2022-07-05> add for sub lcd test in factory test end

    return 0;
    err:
    return -1;
}

static void __exit queue_exit(void)
{
    destroy_workqueue(queue);
}


int __init ai_sublcd_display_init(void)
{
    queue_init();
    return 0;
}

void __exit ai_sublcd_display_exit(void)
{
    queue_exit();
}
//Antaiui <AI_BSP_LCD> <zhuli> <2022-06-28> add for sub lcd display workqueue end
