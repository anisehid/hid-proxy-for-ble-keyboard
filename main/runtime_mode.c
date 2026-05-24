#include "runtime_mode.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static runtime_mode_t   g_mode = RUNTIME_MODE_RELAY;
static SemaphoreHandle_t g_sem = NULL;

void runtime_mode_init(void) {
    if (g_sem == NULL) g_sem = xSemaphoreCreateMutex();
}

runtime_mode_t runtime_mode_get(void) {
    runtime_mode_t m = RUNTIME_MODE_RELAY;
    if (g_sem && xSemaphoreTake(g_sem, pdMS_TO_TICKS(50)) == pdTRUE) {
        m = g_mode;
        xSemaphoreGive(g_sem);
    }
    return m;
}

void runtime_mode_set(runtime_mode_t m) {
    if (g_sem && xSemaphoreTake(g_sem, pdMS_TO_TICKS(50)) == pdTRUE) {
        g_mode = m;
        xSemaphoreGive(g_sem);
    }
}
