#include "led.h"
#include "driver/ledc.h"
#include "freertos/portmacro.h"
#include "freertos/projdefs.h"
#include "hal/ledc_types.h"
#include <stdio.h>

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO (3) // Define the output GPIO
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY_ON (0)                // set duty to max
#define LEDC_DUTY_OFF (8191)            // set duty to min
#define LEDC_DUTY_DIME (8000) // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
#define LEDC_FREQUENCY (5000) // Frequency in Hertz. Set frequency at 5 kHz
#define LEDC_FAST_BLINK_INTERVAL (100)
#define LEDC_SLOW_BLINK_INTERVAL (1000)

static bool update_led = false;
static LEDMessage led_message;
static SemaphoreHandle_t led_semaphor = NULL;

static void ledc_blink(int interval) {
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_ON));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
  vTaskDelay(interval / portTICK_PERIOD_MS);
  // go dark
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_DIME));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
  vTaskDelay(interval / portTICK_PERIOD_MS);
}

static void led_fast_blink() {
  while (true) {
    int mode = LED_MODE_FAST_BLINK;
    get_led_mode(&mode, NULL);
    if (mode != LED_MODE_FAST_BLINK) {
      break;
    }
    ledc_blink(LEDC_FAST_BLINK_INTERVAL);
  }
}

static void led_slow_blink() {
  while (true) {
    int mode = LED_MODE_SLOW_BLINK;
    get_led_mode(&mode, NULL);
    if (mode != LED_MODE_SLOW_BLINK) {
      break;
    }
    ledc_blink(LEDC_SLOW_BLINK_INTERVAL);
  }
}

static void ledc_always_on() {
  while (true) {
    int mode = LED_MODE_ALWAYS_ON;
    get_led_mode(&mode, NULL);
    if (mode != LED_MODE_ALWAYS_ON) {
      break;
    }
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_ON));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
    vTaskDelay(LEDC_SLOW_BLINK_INTERVAL / portTICK_PERIOD_MS);
  }
}

static void ledc_off() {
  while (true) {
    int mode = LED_MODE_OFF;
    get_led_mode(&mode, NULL);
    if (mode != LED_MODE_OFF) {
      break;
    }
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_DIME));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
    vTaskDelay(LEDC_SLOW_BLINK_INTERVAL / portTICK_PERIOD_MS);
  }
}

void change_led(void *) {
  while (true) {
    /* printf("Enter Read ledc msg and change led\n"); */
    // read data to msg
    int mode;
    bool need_update;
    if (!get_led_mode(&mode, &need_update)) {
      printf("ERROR: Get LED mode and need_update Failed\n");
      /* return; */
      continue;
    }
    // set update_led to false, keep trying until success
    if (need_update) {
      set_update_led(false);
      /* printf("%d, CHANGE LED MODE TO ", (int)update_led); */
      /* print_led_mode(mode); */
      /* printf("\n"); */
      switch (mode) {
      case LED_MODE_ALWAYS_ON:
        ledc_always_on();
        break;
      case LED_MODE_FAST_BLINK:
        led_fast_blink();
        break;
      case LED_MODE_SLOW_BLINK:
        led_slow_blink();
        break;
      case LED_MODE_OFF:
        ledc_off();
        break;
      }
    }
  }
}

bool set_update_led(bool x) {
  if (xSemaphoreTake(led_semaphor, (TickType_t)10) == pdTRUE) {
    update_led = x;
    /* printf("Set Update led to %d\n", (int)update_led); */
    xSemaphoreGive(led_semaphor);
    return true;
  }
  return false;
}

bool get_led_mode(int *mode, bool *update_led) {
  if (xSemaphoreTake(led_semaphor, (TickType_t)10) == pdTRUE) {
    if (mode != NULL) {
      *mode = led_message.mode;
    }

    if (update_led != NULL) {
      *update_led = update_led;
    }
    xSemaphoreGive(led_semaphor);
    return true;
  }
  return false;
}

bool set_led_mode(int m) {
  if (xSemaphoreTake(led_semaphor, (TickType_t)10) == pdTRUE) {
    led_message.mode = m;
    printf("Set update led to true\n");
    update_led = true;
    xSemaphoreGive(led_semaphor);
    return true;
  }
  return false;
}

void ledc_init(void) {
  // Prepare and then apply the LEDC PWM timer configuration
  ledc_timer_config_t ledc_timer = {
      .speed_mode = LEDC_MODE,
      .timer_num = LEDC_TIMER,
      .duty_resolution = LEDC_DUTY_RES,
      .freq_hz = LEDC_FREQUENCY, // Set output frequency at 5 kHz
      .clk_cfg = LEDC_AUTO_CLK};
  ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

  // Prepare and then apply the LEDC PWM channel configuration
  ledc_channel_config_t ledc_channel = {.speed_mode = LEDC_MODE,
                                        .channel = LEDC_CHANNEL,
                                        .timer_sel = LEDC_TIMER,
                                        .intr_type = LEDC_INTR_DISABLE,
                                        .gpio_num = LEDC_OUTPUT_IO,
                                        .duty =
                                            LEDC_DUTY_DIME, // Set duty to 0%
                                        .hpoint = 0};
  ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
  vSemaphoreCreateBinary(led_semaphor);
}

void print_led_mode(int mode) {
  switch (mode) {
  case LED_MODE_ALWAYS_ON:
    printf("LED_MODE_ALWAYS_ON");
    break;
  case LED_MODE_FAST_BLINK:
    printf("LED_MODE_FAST_BLINK");
    break;
  case LED_MODE_SLOW_BLINK:
    printf("LED_MODE_SLOW_BLINK");
    break;
  case LED_MODE_OFF:
    printf("LED_MODE_OFF");
    break;
  default:
    printf("Unknon LED_MODE");
    break;
  }
}
