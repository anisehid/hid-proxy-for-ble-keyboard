#include "runtime_mode.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "RUNTIME_MODE";
static runtime_mode_t   g_mode = RUNTIME_MODE_RELAY;
static SemaphoreHandle_t g_sem = NULL;

void runtime_mode_init(void) {
    if (g_sem == NULL) {
        g_sem = xSemaphoreCreateMutex();
        if (g_sem == NULL) ESP_LOGE(TAG, "xSemaphoreCreateMutex failed");
    }
}

runtime_mode_t runtime_mode_get(void) {
    runtime_mode_t m = RUNTIME_MODE_RELAY;
    if (!g_sem) return m;
    if (xSemaphoreTake(g_sem, pdMS_TO_TICKS(50)) == pdTRUE) {
        m = g_mode;
        xSemaphoreGive(g_sem);
    } else {
        ESP_LOGE(TAG, "runtime_mode_get: mutex timeout");
    }
    return m;
}

void runtime_mode_set(runtime_mode_t m) {
    if (!g_sem) {
        ESP_LOGE(TAG, "runtime_mode_set called before init");
        return;
    }
    if (xSemaphoreTake(g_sem, pdMS_TO_TICKS(50)) == pdTRUE) {
        g_mode = m;
        xSemaphoreGive(g_sem);
    } else {
        ESP_LOGE(TAG, "runtime_mode_set: mutex timeout - mode change DROPPED");
    }
}
