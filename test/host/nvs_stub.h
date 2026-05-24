#ifndef NVS_STUB_H_
#define NVS_STUB_H_

#include "device_store.h"

// Install the in-memory stub as the device_store's NVS ops.
void nvs_stub_install(void);

// Reset the in-memory store between tests.
void nvs_stub_reset(void);

// Helpers to pre-seed v2 state for migration tests.
void nvs_stub_seed_v2_device(const uint8_t bda[6]);

#endif
