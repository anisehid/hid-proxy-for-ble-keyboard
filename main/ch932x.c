#include "ch932x.h"
#include <esp_log.h>
#include <stdio.h>

#ifdef USE_CH9329
static const char *CH932X_TAG = "CH9329";
uint8_t s_packed_buf[CH9329_HEADER_LEN + CH9329_MAX_PAYLOAD];
#else
static const char *CH932X_TAG = "CH9328";
#endif

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
