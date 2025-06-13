#include "delay.h"
#include "peripheral/coretimer/plib_coretimer.h"


void delay(uint32_t delay_ms)
{
    CORETIMER_DelayMs(delay_ms);
}
