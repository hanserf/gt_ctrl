
#ifndef _SERFFUN_H_
#define _SERFFUN_H_
#include "Arduino.h"

#include "pins_arduino.h" // for digitalPinToBitMask, etc
#include "RTClib.h"
#include <OneWire.h>
#include <DallasTemperature.h>

//Global vars

class gt_environment
{
private:
    //RTC objects
    DateTime currentTime;
    DateTime prevTime;
    DateTime lights_off;
    DateTime lights_on;
    float indoor_temp_d;
    float indoor_temp_q;
    float outdoor_temp_d;
    float outdoor_temp_q;
    int led_string;     // 2;
    int led_string_pwm; // 3;
    int led_cob;        // 4;
    int led_cob_pwm;    // 5;

public:
    gt_environment();
    void setSignals(const int a, const int b, const int c, const int d);
    float getPreviousTemp(int sensor);
    float getThisTemp(int sensor);
    int setThisTemp(int sensor, float aTemp);
    int setPrevTemp(int sensor, float aTemp);

    String getFullTimeString(DateTime aTime);
    String getClockString(DateTime aTime);
    void setFutureEvent(DateTime aTime);
    DateTime getFutureEvent();
    int isFutureevent(DateTime now);
    void printTemperatures();
    void printDallasDevices();
    void printAddress(DeviceAddress deviceAddress);
    void pwm_set(const int myPin, int value);
} gt_environment_t
#endif // _SERFFUN_H_
