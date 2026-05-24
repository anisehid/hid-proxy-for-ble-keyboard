#ifndef LED_DUTY_H_
#define LED_DUTY_H_

#include <stdint.h>

// Duty values mirror led.c's existing LEDC_DUTY_ON / LEDC_DUTY_DIME.
#define LED_DUTY_ON   0
#define LED_DUTY_DIME 8000

// Mode constants — kept in sync with led.h LED_MODE_* values.
#define LED_MODE_OFF        0
#define LED_MODE_FAST_BLINK 1
#define LED_MODE_ALWAYS_ON  2
#define LED_MODE_SLOW_BLINK 3

// Returns the LEDC duty value for the given mode at the given 50ms tick.
int led_duty_for(int mode, int tick);

#endif
