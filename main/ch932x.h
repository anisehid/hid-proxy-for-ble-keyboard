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
#define FIXED_CH9329_DATA_LEN 6

/* ch9329 command, currently only is used*/
typedef enum enum_CH9329CMD {
  CMD_GET_INFO = 0x01,
  CMD_SEND_KB_GENERAL_DATA = 0x02,
  CMD_SEND_KB_MEDIA_DATA = 0x03,
  CMD_MAX
} CH9329CMD;

extern uint8_t *g_packed_data;
extern int g_packed_data_len;

/*
 *  Pack the original keyboard
 *  Format:
 *  HEAD 2B
 *  ADDR 1B
 *  CMD  1B
 *    CMD_SEND_KB_GENERAL_DATA 0x02
 *  LEN  1B
 *  DATA NB(N=0-64)
 *  SUM  1B(SUM=HEAD+ADDR+CMD+LEN+DATA)
 *  */
void pack_ch9329_data(CH9329CMD cmd, uint8_t *data, int length);
#endif // USE_CH9329

#endif // CH932X_H_
