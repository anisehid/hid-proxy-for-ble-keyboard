#ifndef CH932X_H_
#define CH932X_H_

/*
 *  Send data to CH9328/CH9329
 */

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "string.h"
#include <stdint.h>

static const int TX_BUF_SIZE = 1024;
static const int uart_num = UART_NUM_1;

#define TXD_PIN (GPIO_NUM_7)
#define RXD_PIN (GPIO_NUM_8)

/*Init uart*/
void init_uart();

/*send data to port*/
int send_data_to_uart(void *data, int len);

#ifdef USE_CH9329
#include "ch9329_pack.h"

// Shared packed-frame buffer owned by ch932x.c; caller fills via pack_ch9329_data.
extern uint8_t s_packed_buf[CH9329_HEADER_LEN + CH9329_MAX_PAYLOAD];
#endif

#endif // CH932X_H_
