#include "serfFun.h"

void setSignals(signals_t bus, const int a, const int b, const int c, const int d)
{
    bus.led_string = a;
    bus.led_string_pwm = b;
    bus.led_cob = c;
    bus.led_cob_pwm = d;
}
