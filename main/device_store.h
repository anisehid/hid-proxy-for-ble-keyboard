#ifndef DEVICE_STORE_H_
#define DEVICE_STORE_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define DEVICE_STORE_MAX 5
#define DEVICE_NAME_LEN  8   // 7 chars + NUL

typedef struct {
    uint8_t bda[6];
    uint8_t addr_type;
    uint8_t _reserved;
    char    name[DEVICE_NAME_LEN];
} device_entry_t;

// Opaque NVS facade — production wires this to nvs_flash, tests wire to a stub.
typedef struct nvs_ops {
    int  (*open)(const char *ns, void **out_handle);
    int  (*get_blob)(void *handle, const char *key, void *buf, size_t *size);
    int  (*set_blob)(void *handle, const char *key, const void *buf, size_t size);
    int  (*get_u8)(void *handle, const char *key, uint8_t *out);
    int  (*set_u8)(void *handle, const char *key, uint8_t in);
    int  (*get_i32)(void *handle, const char *key, int32_t *out);
    int  (*set_i32)(void *handle, const char *key, int32_t in);
    int  (*erase_all)(void *handle);
    int  (*commit)(void *handle);
    void (*close)(void *handle);
} nvs_ops_t;

// Install ops (call once at boot). Idempotent.
void device_store_set_ops(const nvs_ops_t *ops);

// Read full list from NVS. Returns count (0..DEVICE_STORE_MAX).
// out[] must hold DEVICE_STORE_MAX entries.
int device_store_list(device_entry_t *out);

// Upsert. If MAC already present, updates name in place.
// If full, shifts slot 0 out and appends at slot 4. Returns true on success.
bool device_store_upsert(const device_entry_t *entry);

// Remove by MAC. Returns true if removed, false if not found.
bool device_store_remove(const uint8_t bda[6]);

// True if this MAC is in the saved list.
bool device_store_contains(const uint8_t bda[6]);

// Erase everything (called from storage_factory_reset).
void device_store_clear(void);

// One-shot migration from v2 single-MAC storage.
// Reads v2 keys "ble_status" and "ble_results"; if v2 has a device,
// installs it as devices[0]. Idempotent (no-op if v2 absent).
void device_store_migrate_from_v2(void);

#endif
