#include "ch932x.h"
#include <esp_log.h>
#include <stdio.h>
#include <stdlib.h>

#ifdef USE_CH9329
static const char *CH932X_TAG = "CH9329";
#else
static const char *CH932X_TAG = "CH9328";
#endif

// initialize uart port
void init_uart() {
  uart_config_t uart_config = {
      .baud_rate = 9600,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
  };

  ESP_ERROR_CHECK(
      uart_driver_install(uart_num, TX_BUF_SIZE * 2, 0, 0, NULL, 0));
  // Configure UART parameters
  ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
  ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE,
                               UART_PIN_NO_CHANGE));
  ESP_LOGI("UART", "SET PIN: TX = %d, RX = %d\n", TXD_PIN, RXD_PIN);
}

int send_data_to_uart(void *data, int length) {
  const int tx_bytes = uart_write_bytes(uart_num, data, length);
  ESP_LOG_BUFFER_HEX(CH932X_TAG, data, length);
  return tx_bytes;
}

#ifdef USE_CH9329
/// Packed data for ch9329
/// Head(2) + addr(1) + cmd(1) + len(1) + sum(1) = 6byte
/* static uint8_t *g_packed_data = NULL; */
/* static int g_packed_data_len = 0; */
uint8_t *g_packed_data = NULL;
int g_packed_data_len = 0;

// complete packed_data
// set data and length to packed_data, and compute sum
void pack_ch9329_data(CH9329CMD cmd, uint8_t *data, int length) {
  // realloc data if the data lenght is not match
  if (g_packed_data != NULL && g_packed_data_len != length) {
    free(g_packed_data);
    g_packed_data = NULL;
  }
  if (g_packed_data == NULL) {
    g_packed_data = (uint8_t *)malloc(length + FIXED_CH9329_DATA_LEN);
  }
  // set length to global len
  g_packed_data_len = length;

  // head
  g_packed_data[0] = 0x57;
  g_packed_data[1] = 0xAB;
  // addr
  g_packed_data[2] = 0x0;
  // cmd
  g_packed_data[3] = (uint8_t)cmd;
  // len
  g_packed_data[4] = (uint8_t)length;
  // data
  for (int i = 5, j = 0; i < length + 5; ++i, ++j) {
    g_packed_data[i] = data[j];
  }
  // sum
  int sum_pos = length + FIXED_CH9329_DATA_LEN - 1;
  g_packed_data[sum_pos] = 0;
  for (int i = 0; i < sum_pos; ++i)
    g_packed_data[sum_pos] += g_packed_data[i];
}

#endif
