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
/* web admin AP is up (double-pulse every 2 s) */
#define LED_MODE_ADMIN 4

typedef struct _LEDMessage {
  int32_t mode;
} LEDMessage;

void ledc_init(void);

// FreeRTOS task: drive the LED according to the current mode.
void change_led(void *);

// Set the LED mode (thread-safe).
bool set_led_mode(int m);

// Read the current LED mode (thread-safe).
bool get_led_mode(int *mode);

void print_led_mode(int mode);

#endif // LED_H_
