#include "led_duty.h"

int led_duty_for(int mode, int tick) {
    switch (mode) {
    case LED_MODE_OFF:
        return LED_DUTY_DIME;
    case LED_MODE_ALWAYS_ON:
        return LED_DUTY_ON;
    case LED_MODE_FAST_BLINK:
        return ((tick / 2) % 2 == 0) ? LED_DUTY_ON : LED_DUTY_DIME;
    case LED_MODE_SLOW_BLINK:
        return ((tick / 20) % 2 == 0) ? LED_DUTY_ON : LED_DUTY_DIME;
    default:
        return LED_DUTY_DIME;
    }
}
