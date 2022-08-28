#ifndef LED_H_
#define LED_H_

#include "driver/ledc.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <stdio.h>

/* try to connect to any bluetooth */
#define LED_MODE_FAST_BLINK 1
/* already connected */
#define LED_MODE_ALWAYS_ON 2
/* try to connect to a recorded (previously connected) bluetooth */
#define LED_MODE_SLOW_BLINK 3
/* light off */
#define LED_MODE_OFF 0

typedef struct _LEDMessage {
  int32_t mode;
} LEDMessage;

void ledc_init(void);

// change led if update_led == true
// after setting, set update_led to false
void change_led(void *);

// set led mode to m and set update_led to true
bool set_led_mode(int m);

// set update_led to x
bool set_update_led(bool x);

// return current led mode and update_led
bool get_led_mode(int *mode, bool *update_led);

void print_led_mode(int mode);

#endif // LED_H_
