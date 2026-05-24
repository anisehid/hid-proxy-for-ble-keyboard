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
    case LED_MODE_ADMIN: {
        int phase = tick % 40;
        // ON at 0..1 and 4..5, DIME everywhere else
        if ((phase >= 0 && phase < 2) || (phase >= 4 && phase < 6)) {
            return LED_DUTY_ON;
        }
        return LED_DUTY_DIME;
    }
    default:
        return LED_DUTY_DIME;
    }
}
