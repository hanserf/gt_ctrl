#include "serfFun.h"
#define LED_RB 2;
#define LED_COB 4;
#define LED_RB_PWM 3;
#define LED_COB_PWM 5;

//Objects
extern OneWire oneWire(0);
extern DallasTemperature sensors(&oneWire);
extern int numberOfDevices;
extern DeviceAddress tempDeviceAddress;
extern RTC_PCF8523 rtc;
extern char daysOfTheWeek[7][12];

gt_environment::gt_environment(void)
{
    int rc;
    rc = setThisTemp(0, 0.0);
    rc = setThisTemp(1, 0.0);
    rc = setPrevTemp(0, 0.0);
    rc = setPrevTemp(1, 0.0);
    this->led_cob = (int)LED_COB;
    this->led_string = (int)LED_RB;
    this->led_cob_pwm = (int)LED_COB_PWM;
    this->led_string_pwm = (int)LED_RB_PWM;
    currentTime = rtc.now();
    prevTime = currentTime;

}
float gt_environment::getPreviousTemp(int sensor)
{
    switch (sensor)
    {
    case 0:
        return this->indoor_temp_q;
    case 1:
        return this->outdoor_temp_q;
    default:
        return float(0.0);
    }
}
float gt_environment::getThisTemp(int sensor)
{
    switch (sensor)
    {
    case 0:
        return this->indoor_temp_d;
    case 1:
        return this->outdoor_temp_d;
    default:
        return float(0.0);
    }
}
int gt_environment::setThisTemp(int sensor, float aTemp)
{
    switch (sensor)
    {
    case 0:
        this->indoor_temp_d = aTemp;
    case 1:
        this->outdoor_temp_d = aTemp;
    default:
        this->indoor_temp_d = aTemp;
        this->outdoor_temp_d = aTemp;
    }
}
int gt_environment::setPrevTemp(int sensor, float aTemp)
{
    switch (sensor)
    {
    case 0:
        this->indoor_temp_q = aTemp;
    case 1:
        this->outdoor_temp_q = aTemp;
    default:
        this->indoor_temp_q = aTemp;
        this->outdoor_temp_q = aTemp;
    }
}