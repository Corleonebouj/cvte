#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>	                                                  
#include <linux/rtc.h>
#include <error.h>
#include <sys/ioctl.h>
#include <fcntl.h>

int main(void)
{
    int ret, fd, data;
    struct rtc_time my_rtc;
    struct rtc_wkalrm my_alarm;
    fd = open("/dev/rtc0", O_RDWR);
    if (fd < 0) {
        perror("open device rtc error");
        exit(1);
    }
    
    //Enter any time
    printf("please enter any time!\n");
    scanf("%d %d %d %d %d %d %d", &my_rtc.tm_year, &my_rtc.tm_mon, &my_rtc.tm_mday, &my_rtc.tm_wday, &my_rtc.tm_hour, &my_rtc.tm_min, &my_rtc.tm_sec);
    printf("Your enter date/time is %d-%d-%d-%d,%02d:%02d:%02d.\n",my_rtc.tm_year, my_rtc.tm_mon, my_rtc.tm_mday, my_rtc.tm_wday, my_rtc.tm_hour, my_rtc.tm_min, my_rtc.tm_sec);


    my_rtc.tm_year -= 1900;
    my_rtc.tm_mon -= 1;

    //Rtc set and read time
    ret = ioctl(fd, RTC_SET_TIME, &my_rtc);
    printf("Set rtc time %s\n",ret ? "error":"ok");
    ret = ioctl(fd, RTC_RD_TIME, &my_rtc);
    printf("Read rtc time %s\n",ret ? "error":"ok");
    printf("Current rtc date/time is %d-%d-%d-%d,%02d:%02d:%02d.\n",my_rtc.tm_year + 1900, my_rtc.tm_mon + 1, my_rtc.tm_mday, my_rtc.tm_wday, my_rtc.tm_hour, my_rtc.tm_min, my_rtc.tm_sec);
    
    my_alarm.time.tm_hour = 13;
    my_alarm.time.tm_min = 50;
    my_alarm.time.tm_sec = 20;
    
    //Rtc set and read alarm
    ret = ioctl(fd, RTC_ALM_SET, &my_alarm.time);
    printf("Set alarm time %s\n",ret ? "error":"ok");
    ret = ioctl(fd, RTC_AIE_ON,0);
    printf("Set rtc int %s\n",ret ? "error":"ok");
    ret = ioctl(fd, RTC_ALM_READ, &my_alarm.time);
    printf("Alarm time now set to %d-%d-%d-%d %02d:%02d:%02d.\n", my_rtc.tm_year + 1900, my_rtc.tm_mon + 1, my_rtc.tm_mday, my_rtc.tm_wday, my_alarm.time.tm_hour, my_alarm.time.tm_min, my_alarm.time.tm_sec);
    
    //Waiting for interrupt
    read(fd, &data, sizeof(unsigned long));
    printf("Oh, interrupt come....\n");
    return 0;
}
