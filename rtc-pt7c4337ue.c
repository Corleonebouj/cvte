/*************************************************************************
	> File Name: rtc-pt7c4337ue.c
	> Author:Zhuzhaoyang 
	> Mail:18909251097@163.com 
	> Created Time: Mon 17 Jul 2023 09:26:04 AM CST
 ************************************************************************/
#include <linux/rtc.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/mutex.h>
#include <linux/bcd.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>  
#include "../cvte/include/cvte_board_config.h"

#undef TAG
#define TAG "[pt7c4337ue]"
#define LOG_ENABLE 1

#if LOG_ENABLE
#define log_crit(args...)         printk(KERN_CRIT TAG "[c]" args)
#define log_err(args...)          printk(KERN_ERR  TAG "[e]" args)
#define log_warn(args...)         printk(KERN_WARNING TAG "[w]" args)
#define log_info(args...)         printk(KERN_CRIT args)
#define log_debug(args...)        printk(KERN_DEBUG TAG "[d]" args)

#else
#define log_crit(args...)
#define log_err(args...)
#define log_warn(args...)
#define log_info(args...)
#define log_debug(args...)
#endif

struct pt7c4337ue_DEV {
        struct mutex pt7c4337ue_i2c_mutex;
        struct i2c_client *i2c_client;
        struct rtc_device *pt7c4337ue_device;
        struct work_struct intr_b_wq;
};

struct bcd_time
{
        unsigned char sec;
        unsigned char min;
        unsigned char hour;
        unsigned char mday;
        unsigned char wday;
        unsigned char mon;
        unsigned char year;

};

#define SET_BIT(x,a) ((x) |= (a))
#define CLEAR_BIT(x,a) ((x) &= (~a)) 

//Clock REG 
#define SECONDE_REG     (0x00)
#define MIN_REG         (0x01)
#define HOUR_REG        (0x02)
#define WEEK_REG        (0x03)
#define DAY_REG         (0x04)
#define MON_REG         (0x05)
#define YEAR_REG        (0x06)
#define CONTROL1_REG    (0x0E)
#define STATUS_REG      (0x0F)
//Alarm REG
#define ALARM_1_SEC_REG (0X07)
#define ALARM_1_MIN_REG (0X08)
#define ALARM_1_HOU_REG (0X09)
#define ALARM_1_DAY_REG (0X0A)

#define ALARM_2_MIN_REG (0X0B)
#define ALARM_2_HOU_REG (0X0C)
#define ALARM_2_DAY_REG (0X0D)
//HOUR_REG
#define TIME_MODE_BIT    0b01000000
//STATUS_REG
#define A2FG_BIT         0b00000010
#define A1FG_BIT         0b00000001
//CONTROL1_REG
#define A2IE_BIT         0b00000010
#define A1IE_BIT         0b00000001
#define INTCN_BIT        0b00000100
//DAY OR DATE
#define DOD_BIT          0b01000000

#define RTC_32BIT_UNIX_YEAR_MAX         35 /* year 2035 */
#define RTC_YEAR_DEFAULT                70 /* year 1970 */

static void cover_bcd_time_to_rtc_tm(const struct bcd_time *bcdTime, struct rtc_time *tm)
{
        int year = 0;

        tm->tm_sec  = bcd2bin(bcdTime->sec);
        tm->tm_min  = bcd2bin(bcdTime->min);
        tm->tm_hour = bcd2bin(bcdTime->hour);
        tm->tm_mday = bcd2bin(bcdTime->mday);
        tm->tm_wday = bcd2bin(bcdTime->wday);

        tm->tm_mon = bcd2bin(bcdTime->mon) - 1;
        
        year = bcd2bin(bcdTime->year);

        if (year <= RTC_32BIT_UNIX_YEAR_MAX)
        {
            year += 100;    
        }
        else if ((year >= RTC_YEAR_DEFAULT) && (year < 100))
        {

        }
        else
        {
            log_info("pt7c4337ue:%s year=%d is unvalid, force to DEFAULT=%d!\n", __func__, year, RTC_YEAR_DEFAULT);
            year = RTC_YEAR_DEFAULT;    
        }
        
        tm->tm_year = year;

}

static void cover_rtc_tm_to_bcd_time(const struct rtc_time *tm, struct bcd_time *bcdTime)
{
        int year = 0;

        bcdTime->sec    = bin2bcd(tm->tm_sec);
        bcdTime->min    = bin2bcd(tm->tm_min);
        bcdTime->hour   = bin2bcd(tm->tm_hour);
        bcdTime->mday   = bin2bcd(tm->tm_mday);
        bcdTime->wday   = bin2bcd(tm->tm_wday);

        bcdTime->mon    = bin2bcd(tm->tm_mon + 1) ;

        year = tm->tm_year;
        if ((year >= 100) && (year <= (100+RTC_32BIT_UNIX_YEAR_MAX)))
        {
                year -= 100; // Only need to save LSB to RTC.    
        }
        else if ((year >= RTC_YEAR_DEFAULT) && (year < 100))
        {

        }
        else
        {
                log_info("pt7c4337ue:%s year=%d is unvalid, force to DEFAULT=%d!\n",
                                        __func__, year, RTC_YEAR_DEFAULT);
                year = RTC_YEAR_DEFAULT;    
        }
        bcdTime->year = bin2bcd(year);

}

static long long get_time_stamp(const struct rtc_time *tm)
{
        return mktime64(tm->tm_year+1900, tm->tm_mon+1, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec);

}

int pt7c4337ue_i2c_Write_Byte(struct i2c_client *client, unsigned char reg, unsigned char value)
{
        unsigned char buffer[2] = { reg, value  };
        struct i2c_msg msg;
        int err;

        msg.addr = client->addr;
        msg.flags = 0;
        msg.len = sizeof(buffer);
        msg.buf = buffer;

        err = i2c_transfer(client->adapter, &msg, 1);
        if (err < 0)
            return err;

        return 0;

}

int pt7c4337ue_i2c_Read_Byte(struct i2c_client *client, unsigned char reg, unsigned char *value)
{
        struct i2c_msg msgs[]={
            {
                        .addr = client->addr,
                        .flags = 0,
                        .len = 1,
                        .buf = &reg
                    
            },
            {
                        .addr = client->addr,
                        .flags = I2C_M_RD,
                        .len = sizeof(*value),
                        .buf = value
                    
            }
            
        };

        int ret = 0;
        ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
        if (ret < 0)
            return ret;

        return 0;

}


static int read_time(struct i2c_client *client, struct bcd_time *bcdTime)
{
        int ret = 0;
        
        ret = pt7c4337ue_i2c_Read_Byte(client, SECONDE_REG, &bcdTime->sec);
        if(ret < 0)
        {
                ret = -1;
                goto err;
            
        }

        ret = pt7c4337ue_i2c_Read_Byte(client, MIN_REG, &bcdTime->min);
        if(ret < 0)
        {
                ret = -1;
                goto err;
            
        }

        ret = pt7c4337ue_i2c_Read_Byte(client, HOUR_REG, &bcdTime->hour);
        if(ret < 0)
        {
                ret = -1;
                goto err;
            
        }

        
        ret = pt7c4337ue_i2c_Read_Byte(client, DAY_REG, &bcdTime->mday);
        if(ret < 0)
        {
                ret = -1;
                goto err;
            
        }

        ret = pt7c4337ue_i2c_Read_Byte(client, WEEK_REG, &bcdTime->wday);
        if(ret < 0)
        {
                ret = -1;
                goto err;
            
        }

        ret = pt7c4337ue_i2c_Read_Byte(client, MON_REG, &bcdTime->mon);
        if(ret < 0)
        {
                ret = -1;
                goto err;
            
        }

        ret = pt7c4337ue_i2c_Read_Byte(client, YEAR_REG, &bcdTime->year);
        if(ret < 0)
        {
                    ret = -1;
                    goto err;
                
        }
            
err:
        return ret;

} 


static int write_time(struct i2c_client *client, struct bcd_time bcdTime)
{   
    int ret;

    ret=pt7c4337ue_i2c_Write_Byte(client, SECONDE_REG,bcdTime.sec);
    if (ret < 0) {
                goto err;
            
    }
    ret=pt7c4337ue_i2c_Write_Byte(client, MIN_REG,bcdTime.min);
    if (ret < 0) {
                goto err;
            
    }
    ret=pt7c4337ue_i2c_Write_Byte(client, HOUR_REG,bcdTime.hour);
    if (ret < 0) {
                goto err;
            
    }
    ret=pt7c4337ue_i2c_Write_Byte(client, DAY_REG,bcdTime.mday);
    if (ret < 0) {
                goto err;
            
    }
    ret=pt7c4337ue_i2c_Write_Byte(client, WEEK_REG,bcdTime.wday);
    if (ret < 0) {
                goto err;
            
    }
    ret=pt7c4337ue_i2c_Write_Byte(client, MON_REG,bcdTime.mon);
    if (ret < 0) {
                goto err;
            
    }
    ret=pt7c4337ue_i2c_Write_Byte(client, YEAR_REG,bcdTime.year);
    if (ret < 0) {
                goto err;
            
    }
    return 0;
err:
    return -1;
}

static int get_correct_wday(const struct rtc_time *rtc_tm)
{
        struct rtc_time my_tm;
        long long seconds;

        seconds = get_time_stamp(rtc_tm);

        rtc_time64_to_tm(seconds, &my_tm);

        return my_tm.tm_wday;

}

int pt7c4337ue_rtc_check_bcdTime(struct pt7c4337ue_DEV *rtc_dev)
{
        int ret;
        int save_hardware_flag = 0;
        struct bcd_time bcdTime;
        struct rtc_time tm;
        int tm_wday = 0;
        log_info("pt7c4337ue:read rtc clock  !\n");

        ret = read_time(rtc_dev->i2c_client, &bcdTime);
        if (ret < 0) {
            goto err;    
        }

        cover_bcd_time_to_rtc_tm(&bcdTime, &tm);

        log_info("pt7c4337ue_rtc_check_bcdTime: tm_year=%d, tm_mon=%d, tm_wday=%d,tm_mday=%d,tm_hour=%d,tm_min=%d,tm_sec=%d\n", tm.tm_year, tm.tm_mon, tm.tm_wday, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
        
        if ((tm.tm_year == RTC_YEAR_DEFAULT) || (rtc_valid_tm(&tm) < 0))
        {
                /* reset to default */
                log_info("pt7c4337ue reset to default\n");
                save_hardware_flag = 1;
                tm.tm_year  = 123;
                tm.tm_mon   = 7;
                tm.tm_mday  = 19;
                tm.tm_wday  = 3;
                tm.tm_hour  = 14;
                tm.tm_min   = 0;
                tm.tm_sec   = 0;
            
        }

        tm_wday = get_correct_wday(&tm);
        if(tm_wday != 3) {
            log_info("pt7c4337ue get_time_stamp:%d\n", tm_wday);
        }
        if (tm_wday != tm.tm_wday)
        {
                save_hardware_flag = 1;
                tm.tm_wday = tm_wday;
            
        }

        if (save_hardware_flag)
        {
                cover_rtc_tm_to_bcd_time(&tm, &bcdTime);
                ret = write_time(rtc_dev->i2c_client, bcdTime);
                log_info("pt7c4337ue_rtc_check_bcdTime: save!!tm_year=%d, tm_mon=%d, tm_wday=%d,tm_mday=%d,tm_hour=%d,tm_min=%d,tm_sec=%d\n", tm.tm_year, tm.tm_mon, tm.tm_wday, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
                if (ret < 0)
                {
                        goto err;
                    
                }
            
        }

        return ret;
err:
        log_info("pt7c4337ue:pt7c4337ue_rtc_check_bcdTime  error  !\n");
        return ret;
}

// interrupt B workqueue
static void intr_b_wq_func(struct work_struct *data)
{
        unsigned char  value;
        struct pt7c4337ue_DEV * rtc_dev = container_of(data, struct pt7c4337ue_DEV, intr_b_wq);
        log_info("pt7c4337ue:interrput B happend ! \n");
        pt7c4337ue_i2c_Read_Byte(rtc_dev->i2c_client, STATUS_REG,&value);
        CLEAR_BIT(value,A1FG_BIT);
        CLEAR_BIT(value,A2FG_BIT);
        pt7c4337ue_i2c_Write_Byte(rtc_dev->i2c_client, STATUS_REG,value);
}

//interrupt B handler
static irqreturn_t intr_b_handler(int gpio_irq, void *data)
{
        struct pt7c4337ue_DEV *rtv_dev = (struct pt7c4337ue_DEV *)data;
        schedule_work(&rtv_dev->intr_b_wq);
        log_info("pt7c4337ue gpio interrupt success !\n");
        return IRQ_HANDLED;
}

int pt7c4337ue_rtc_init(struct pt7c4337ue_DEV *rtc_dev)
{
        int irq = 0;
        volatile unsigned char value = 0;
        int ret = 0;
        mutex_lock(&rtc_dev->pt7c4337ue_i2c_mutex);
        
        ret=pt7c4337ue_i2c_Read_Byte(rtc_dev->i2c_client, CONTROL1_REG, (unsigned char *)&value);
        if (ret < 0) {
                goto err;    
        }
        log_info("pt7c4337ue:init CONFIG REG 1  origin value is %x  !\n",value);
        
        CLEAR_BIT(value, A2FG_BIT);
        CLEAR_BIT(value, A1FG_BIT);
        SET_BIT(value, INTCN_BIT); 
        ret=pt7c4337ue_i2c_Write_Byte(rtc_dev->i2c_client, CONTROL1_REG,value);
        value = 0;
        ret=pt7c4337ue_i2c_Read_Byte(rtc_dev->i2c_client, CONTROL1_REG, (unsigned char *)&value);
        if(ret < 0) {
            goto err;
        }
        
        log_info("pt7c4337ue:init CONFIG REG 1  final value is %x  !\n",value);
        if (ret < 0) {
            goto err;
        }
        
        pt7c4337ue_rtc_check_bcdTime(rtc_dev);
        mutex_unlock(&rtc_dev->pt7c4337ue_i2c_mutex);
        
        INIT_WORK(&rtc_dev->intr_b_wq,intr_b_wq_func);
        irq=gpio_to_irq(BOARD_RTC_INT_GPIONUM);
        ret = gpio_request(BOARD_RTC_INT_GPIONUM,"pt7c4337ue");
        if (ret < 0) {
                log_info("gpio request failed !\n ");
                goto err;
            
        }
        ret=request_irq(irq, intr_b_handler, IRQF_TRIGGER_FALLING, "pt7c4337ue", (void *)rtc_dev);
        if (ret < 0) {
                log_info("irq request failed !\n ");
                goto err;
            
        }
        enable_irq(irq);
        log_info("init over!\n");
        return 0;
err:
        log_info("pt7c4337ue rtc init error !\n");
        return -1;
}

static int pt7c4337ue_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
        int ret = -1;
        volatile unsigned char value;
        struct bcd_time bcdTime;

        struct i2c_client *client = NULL;
        struct pt7c4337ue_DEV * rtc_dev = NULL;
        client = to_i2c_client(dev);
        if(NULL == client)
        {
                goto err1;
            
        }

        rtc_dev = i2c_get_clientdata(client);
        if(NULL == rtc_dev)
        {
                goto err1;
            
        }

        mutex_lock(&rtc_dev->pt7c4337ue_i2c_mutex);

        value=0;
        ret = read_time(rtc_dev->i2c_client, &bcdTime);
        if (ret < 0) {
                goto err;
            
        }

        cover_bcd_time_to_rtc_tm(&bcdTime, tm);

        mutex_unlock(&rtc_dev->pt7c4337ue_i2c_mutex);
        return ret;
err:
        log_info("pt7c4337ue:read time error  !\n");
        mutex_unlock(&rtc_dev->pt7c4337ue_i2c_mutex);
err1:
        return ret;

}

static int pt7c4337ue_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
        int ret = -1;
        volatile unsigned char value;
        struct bcd_time bcdTime;

        struct i2c_client *client = NULL;
        struct pt7c4337ue_DEV * rtc_dev = NULL;
        client = to_i2c_client(dev);
        if(NULL == client)
        {
                goto err1;
            
        }

        rtc_dev = i2c_get_clientdata(client);
        if(NULL == rtc_dev)
        {
                goto err1;
            
        }
        
        mutex_lock(&rtc_dev->pt7c4337ue_i2c_mutex);

        ret= pt7c4337ue_i2c_Read_Byte(rtc_dev->i2c_client, CONTROL1_REG, (unsigned char *)&value);
        if (ret < 0) {
                goto err;
            
        }
        //12/24 hour
        value &= (~TIME_MODE_BIT);
        ret=pt7c4337ue_i2c_Write_Byte(rtc_dev->i2c_client, CONTROL1_REG, value);
        if (ret < 0) {
                goto err;
                    
        }
            

        cover_rtc_tm_to_bcd_time(tm, &bcdTime);

        log_info("pt7c4337ue_rtc_set_time: tm_year=%d, tm_mon=%d, tm_wday=%d,tm_mday=%d,tm_hour=%d,tm_min=%d,tm_sec=%d\n", tm->tm_year, tm->tm_mon, tm->tm_wday, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec);

        ret = write_time(rtc_dev->i2c_client, bcdTime);
        if (ret < 0) {
                goto err;
            
        }

        mutex_unlock(&rtc_dev->pt7c4337ue_i2c_mutex);
        return ret;
err:
        log_info("pt7c4337ue:read time error  !\n");
        mutex_unlock(&rtc_dev->pt7c4337ue_i2c_mutex);
err1:
        return ret;
}

static int is_alarm_enabled(struct i2c_client *client)
{
        int ret;
        volatile unsigned char ctrl_reg = 0;
        volatile unsigned char alarm_week_reg = 0;

        ret = pt7c4337ue_i2c_Read_Byte(client, CONTROL1_REG, (unsigned char *)&ctrl_reg);
        if (ret < 0)
        {
                return 0;
            
        }

        ret=pt7c4337ue_i2c_Read_Byte(client, ALARM_1_DAY_REG, (unsigned char *)&alarm_week_reg);
        if (ret < 0)
        {
                return 0;
            
        }

        if ((ctrl_reg & A1IE_BIT) && (alarm_week_reg != 0))
        {
                log_info("pt7c4337ue is_alarm_enabled!\n");
                return 1;
            
        }
        else
        {
                log_info("pt7c4337ue is_alarm_disabled!\n");
                return 0;
            
        }

}


static int read_alarm(struct i2c_client *client, struct bcd_time *bcdTime)
{
        int ret;
        unsigned char value;
        log_info("pt7c4337ue read alarm\n");
        ret=pt7c4337ue_i2c_Read_Byte(client, ALARM_1_SEC_REG, &bcdTime->sec);
        if (ret < 0) {
                goto err;         
        }


        ret=pt7c4337ue_i2c_Read_Byte(client, ALARM_1_MIN_REG, &bcdTime->min);
        if (ret < 0) {
                goto err;
            
        }
        ret=pt7c4337ue_i2c_Read_Byte(client, ALARM_1_HOU_REG, &bcdTime->hour);
        if (ret < 0) {
                goto err;
            
        }

        ret=pt7c4337ue_i2c_Read_Byte(client, ALARM_1_DAY_REG, (unsigned char *)&value);
        if (ret < 0) {
                goto err;
            
        }

        if(value & DOD_BIT) 
        {
            ret=pt7c4337ue_i2c_Read_Byte(client, ALARM_1_DAY_REG, &bcdTime->wday);
            if (ret < 0) {
                goto err;
   
            }
            log_info("pt7c4337ue read_time wday!\n");
            bcdTime->mday = bin2bcd(1);
        } 
        else 
        {
            ret=pt7c4337ue_i2c_Read_Byte(client, ALARM_1_DAY_REG, &bcdTime->mday);
            if (ret < 0) {
                goto err;
  
            }
            log_info("pt7c4337ue read_time mday!\n");
            bcdTime->wday = bin2bcd(1);
        }

        bcdTime->year   = bin2bcd(RTC_YEAR_DEFAULT);
        bcdTime->mon    = bin2bcd(1);

        return 0;
err:
        return -1;      
 }

static int pt7c4337ue_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
        int ret = -1;
        struct bcd_time bcdTime;
        struct i2c_client *client = NULL;
        struct pt7c4337ue_DEV *rtc_dev = NULL;
        client = to_i2c_client(dev);
        log_info("pt7c4337ue_rtc_read_alarm\n");
        if(NULL == client)
        {
                goto err1;
            
        }

        rtc_dev = i2c_get_clientdata(client);
        if(NULL == rtc_dev)
        {
                goto err1;
            
        }

        mutex_lock(&rtc_dev->pt7c4337ue_i2c_mutex);

        ret = read_alarm(rtc_dev->i2c_client, &bcdTime);
        log_info("pt7c4337ue_rtc_read_alarm: BCD code mday=%x, wday=%x, hour=%x, min=%x, sec=%x\n",bcdTime.mday, bcdTime.wday,bcdTime.hour, bcdTime.min, bcdTime.sec);
        if (ret < 0) {
                goto err;
            
        }

        cover_bcd_time_to_rtc_tm(&bcdTime, &(alrm->time));

        if (is_alarm_enabled(rtc_dev->i2c_client))
        {
                alrm->enabled = 1;
            
        }
        else
        {
                alrm->enabled = 0;
            
        }

        log_info("pt7c4337ue_rtc_read_alarm: enabled=%d,tm_year=%d, tm_mon=%d, tm_wday=%d,tm_mday=%d,tm_hour=%d,tm_min=%d,tm_sec=%d\n", alrm->enabled, alrm->time.tm_year, alrm->time.tm_mon, alrm->time.tm_wday, alrm->time.tm_mday, alrm->time.tm_hour, alrm->time.tm_min, alrm->time.tm_sec);


        mutex_unlock(&rtc_dev->pt7c4337ue_i2c_mutex);
        return ret;
err:
        log_info("pt7c4337ue:read time error  !\n");
        mutex_unlock(&rtc_dev->pt7c4337ue_i2c_mutex);
err1:
        return ret;
}

static int write_alarm(struct i2c_client *client, struct bcd_time bcdTime, int enabled)
{
        int ret;
        volatile unsigned char value;
        value=0;
        ret=pt7c4337ue_i2c_Read_Byte(client, CONTROL1_REG, (unsigned char *)&value);
        if (ret < 0) {
                goto err;
            
        }
        CLEAR_BIT(value,A1IE_BIT);
        ret=pt7c4337ue_i2c_Write_Byte(client, CONTROL1_REG, value);
        if (ret < 0) {
                goto err;
            
        }
        
        ret=pt7c4337ue_i2c_Write_Byte(client, ALARM_1_SEC_REG, bcdTime.sec);
        if (ret < 0) {
                goto err;

        }

        ret=pt7c4337ue_i2c_Write_Byte(client, ALARM_1_MIN_REG, bcdTime.min);
        if (ret < 0) {
                goto err;
            
        }
        ret=pt7c4337ue_i2c_Write_Byte(client, ALARM_1_HOU_REG, bcdTime.hour);
        if (ret < 0) {
                goto err;
            
        }

        value = 0;
        if (enabled)
        {
                value = 1 <<  bcd2bin(bcdTime.wday);
            
        }

        ret=pt7c4337ue_i2c_Write_Byte(client, ALARM_1_DAY_REG, value);
        if (ret < 0) {
                goto err;
            
        }

        value =0;
        ret=pt7c4337ue_i2c_Read_Byte(client, CONTROL1_REG, (unsigned char *)&value);
        if (ret < 0) {
                goto err;
            
        }

        if (enabled)
        {
                SET_BIT(value, A1IE_BIT);
                log_info("enabled A1IE_BIT = %d\n", value);
            
        }
        else
        {
                CLEAR_BIT(value, A1IE_BIT);
                log_info("cleaned A1IE_BIT = %d\n", value);
        }

        ret=pt7c4337ue_i2c_Write_Byte(client, CONTROL1_REG, value);
        if (ret < 0) {
                    goto err;
                
        }
        return 0;
err:
        return -1;
}


static int pt7c4337ue_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm) 
{
        int ret = -1;
        struct bcd_time bcdTime;
        struct rtc_wkalrm tmp;
        struct rtc_time tm;
        long long alarm_sec = 0;
        long long nowtime_sec = 0;
        struct i2c_client *client = NULL;
        struct pt7c4337ue_DEV *rtc_dev = NULL;

        client = to_i2c_client(dev);
        if(NULL == client)
        {
                goto err1;
            
        }

        rtc_dev = i2c_get_clientdata(client);
        if(NULL == rtc_dev)
        {
                goto err1;
            
        }

        mutex_lock(&rtc_dev->pt7c4337ue_i2c_mutex);

        alarm_sec = get_time_stamp(&(alrm->time));

        ret = read_time(rtc_dev->i2c_client, &bcdTime);
        if (ret < 0) {
                goto err;
            
        }
        cover_bcd_time_to_rtc_tm(&bcdTime, &tm);
        nowtime_sec = get_time_stamp(&tm);

        if (alarm_sec  > nowtime_sec)
        {
                alrm->enabled = 1;
            
        }
        else
        {
                alrm->enabled = 0;
            
        }

        cover_rtc_tm_to_bcd_time(&(alrm->time), &bcdTime);

        log_info("pt7c4337ue_rtc_set_alarm: enabled=%d, tm_year=%d, tm_mon=%d, tm_wday=%d,tm_mday=%d,tm_hour=%d,tm_min=%d,tm_sec=%d\n", alrm->enabled,alrm->time.tm_year, alrm->time.tm_mon, alrm->time.tm_wday, alrm->time.tm_mday, alrm->time.tm_hour, alrm->time.tm_min, alrm->time.tm_sec);

        if (alrm->enabled)
        {
                ret = write_alarm(rtc_dev->i2c_client, bcdTime, 1);
                    
        }
        else
        {
                ret = write_alarm(rtc_dev->i2c_client, bcdTime, 0);
                        
        }
        if (ret < 0) {
                goto err;
                        
        }
        memset(&bcdTime, 0, sizeof(struct bcd_time));
        ret = read_alarm(rtc_dev->i2c_client, &bcdTime);

        cover_bcd_time_to_rtc_tm(&bcdTime, &(tmp.time));

        log_info("pt7c4337ue_rtc_set_alarm: final result is wday : %d ,hour:  %d ,min: %d", tmp.time.tm_wday, tmp.time.tm_hour, tmp.time.tm_min);

        mutex_unlock(&rtc_dev->pt7c4337ue_i2c_mutex);
        return ret;
err:
        log_info("bl5372:write time error  !\n");
        mutex_unlock(&rtc_dev->pt7c4337ue_i2c_mutex);
err1:
        return ret;
}

static int pt7c4337ue_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
        unsigned char value = -1;
        int ret = 0;
        struct i2c_client *client = NULL;
        struct pt7c4337ue_DEV *rtc_dev = NULL;
        client = to_i2c_client(dev);
        enabled = 1;
        log_info("%s:set alrm->enabled=%d\n", __func__, enabled);
 
        if(NULL == client)
        {
                goto err1;
            
        }

        rtc_dev = i2c_get_clientdata(client);
        if(NULL == rtc_dev)
        {
                goto err1;
            
        }
        mutex_lock(&rtc_dev->pt7c4337ue_i2c_mutex);
        value =0;
        ret=pt7c4337ue_i2c_Read_Byte(client, CONTROL1_REG, (unsigned char *)&value);
        if (ret < 0) {
                goto err;
            
        }

        if (enabled)
        {
                SET_BIT(value, A1IE_BIT);
                log_info("alarm enabled A1IE_BIT = %d\n", value);
        }
        else
        {
                CLEAR_BIT(value, A1IE_BIT);
                log_info("alarm disabled A1IE_BIT = %d\n", value);
        }

        ret=pt7c4337ue_i2c_Write_Byte(client, CONTROL1_REG, value);
        if (ret < 0) {
                goto err;
            
        }
        
        mutex_unlock(&rtc_dev->pt7c4337ue_i2c_mutex);
        return 0;
err:
        mutex_unlock(&rtc_dev->pt7c4337ue_i2c_mutex);
err1:
        return ret;
    
        return 0;
}

static const struct rtc_class_ops pt7c4337ue_rtc_ops = {
        .read_time  = pt7c4337ue_rtc_read_time,
        .set_time   = pt7c4337ue_rtc_set_time,
        .set_alarm  = pt7c4337ue_rtc_set_alarm,
        .read_alarm = pt7c4337ue_rtc_read_alarm,
        .alarm_irq_enable= pt7c4337ue_alarm_irq_enable

};

static int  rtc_pt7c4337ue_drv_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
        int ret = -EPERM;
        const char *dev_name;
        struct pt7c4337ue_DEV * rtc_dev= kzalloc(sizeof(struct pt7c4337ue_DEV), GFP_KERNEL);
        if (!rtc_dev) {
            ret = -ENOMEM;
            goto err; 
        }

        log_info("rtc pt7c4337ue enter probe!\n");
        mutex_init(&rtc_dev->pt7c4337ue_i2c_mutex);
        rtc_dev->i2c_client = client;
        i2c_set_clientdata(client, rtc_dev);
        
        ret= pt7c4337ue_rtc_init(rtc_dev);
        if( ret <0 ){
            goto err1;     
        }

        ret = of_property_read_string(client->dev.of_node, "dev-name", &dev_name);
        if (ret || dev_name) {
            log_err("%s of_property_read_string fail ret=%d\n", __func__, ret);
            dev_name = "rtc_pt7c4337ue";
        }
        device_set_wakeup_capable(&client->dev, 1);
        rtc_dev->pt7c4337ue_device = devm_rtc_device_register(&client->dev, dev_name, &pt7c4337ue_rtc_ops, THIS_MODULE);
        if(IS_ERR(rtc_dev->pt7c4337ue_device))
        {
            log_info("Unable to create pt7c4337ue_device\n");
            goto err1;
                        
        }
        return 0;
                        
err1:
        log_err("pt7c4337ue err\n");
        mutex_destroy(&rtc_dev->pt7c4337ue_i2c_mutex);
        kfree(rtc_dev);
err:
        return ret;    
}

static int  rtc_pt7c4337ue_drv_remove(struct i2c_client *client)
{
        struct pt7c4337ue_DEV * rtc_dev = i2c_get_clientdata(client);
        mutex_destroy(&rtc_dev->pt7c4337ue_i2c_mutex);
        kfree(rtc_dev); 
        log_info("rtc_pt7c4337ue remove\n");
        return 0;
}

static struct of_device_id pt7c4337ue_of_match[]={
        {.compatible="cvte, rtc-pt7c4337ue",},
        {},

};
MODULE_DEVICE_TABLE(of, pt7c4337ue_of_match);

static struct i2c_driver rtc_pt7c4337ue_drv = {
        .probe = rtc_pt7c4337ue_drv_probe,
        .remove = rtc_pt7c4337ue_drv_remove,
        .driver = {
                .name = "rtc_pt7c4337ue",
                .of_match_table = of_match_ptr(pt7c4337ue_of_match),
        },
};

module_i2c_driver(rtc_pt7c4337ue_drv);

MODULE_AUTHOR("zzy");
MODULE_DESCRIPTION("Device_create Driver for date");
MODULE_LICENSE("GPL");

