#include "led.h"
#include "led_duty.h"
#include "driver/ledc.h"
#include "freertos/portmacro.h"
#include "freertos/projdefs.h"
#include "hal/ledc_types.h"
#include <stdio.h>

#define LEDC_TIMER     LEDC_TIMER_0
#define LEDC_MODE      LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO (3)
#define LEDC_CHANNEL   LEDC_CHANNEL_0
#define LEDC_DUTY_RES  LEDC_TIMER_13_BIT
#define LEDC_FREQUENCY (5000)
#define LED_TICK_MS    50

static LEDMessage led_message;
static SemaphoreHandle_t led_semaphor = NULL;

void change_led(void *_) {
  int last_mode = -1;
  int phase = 0;
  while (true) {
    int mode = LED_MODE_OFF;
    if (!get_led_mode(&mode)) {
      vTaskDelay(LED_TICK_MS / portTICK_PERIOD_MS);
      continue;
    }
    if (mode != last_mode) {
      phase = 0;
      last_mode = mode;
    }
    int duty = led_duty_for(mode, phase);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
    phase++;
    vTaskDelay(LED_TICK_MS / portTICK_PERIOD_MS);
  }
}

bool get_led_mode(int *mode) {
  if (xSemaphoreTake(led_semaphor, (TickType_t)10) == pdTRUE) {
    if (mode != NULL) {
      *mode = led_message.mode;
    }
    xSemaphoreGive(led_semaphor);
    return true;
  }
  return false;
}

bool set_led_mode(int m) {
  if (xSemaphoreTake(led_semaphor, (TickType_t)10) == pdTRUE) {
    led_message.mode = m;
    xSemaphoreGive(led_semaphor);
    return true;
  }
  return false;
}

void ledc_init(void) {
  ledc_timer_config_t ledc_timer = {
      .speed_mode = LEDC_MODE,
      .timer_num = LEDC_TIMER,
      .duty_resolution = LEDC_DUTY_RES,
      .freq_hz = LEDC_FREQUENCY,
      .clk_cfg = LEDC_AUTO_CLK};
  ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

  ledc_channel_config_t ledc_channel = {.speed_mode = LEDC_MODE,
                                        .channel = LEDC_CHANNEL,
                                        .timer_sel = LEDC_TIMER,
                                        .intr_type = LEDC_INTR_DISABLE,
                                        .gpio_num = LEDC_OUTPUT_IO,
                                        .duty = LED_DUTY_DIME,
                                        .hpoint = 0};
  ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
  vSemaphoreCreateBinary(led_semaphor);
}

void print_led_mode(int mode) {
  switch (mode) {
  case LED_MODE_ALWAYS_ON:  printf("LED_MODE_ALWAYS_ON");  break;
  case LED_MODE_FAST_BLINK: printf("LED_MODE_FAST_BLINK"); break;
  case LED_MODE_SLOW_BLINK: printf("LED_MODE_SLOW_BLINK"); break;
  case LED_MODE_OFF:        printf("LED_MODE_OFF");        break;
  default:                  printf("Unknown LED_MODE");    break;
  }
}
