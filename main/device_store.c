#include "device_store.h"
#include <string.h>

#define NS_V3 "hidproxy_v3"
#define NS_V2 "hidproxy_v2"
#define K_DEVICES "devices"
#define K_COUNT   "device_count"

static const nvs_ops_t *g_ops = NULL;

void device_store_set_ops(const nvs_ops_t *ops) { g_ops = ops; }

static int macs_equal(const uint8_t a[6], const uint8_t b[6]) {
    return memcmp(a, b, 6) == 0;
}

int device_store_list(device_entry_t *out) {
    if (!g_ops || !out) return 0;
    void *h = NULL;
    if (g_ops->open(NS_V3, &h) != 0) return 0;
    uint8_t count = 0;
    g_ops->get_u8(h, K_COUNT, &count);
    if (count > DEVICE_STORE_MAX) count = DEVICE_STORE_MAX;
    size_t size = sizeof(device_entry_t) * DEVICE_STORE_MAX;
    int rc = g_ops->get_blob(h, K_DEVICES, out, &size);
    g_ops->close(h);
    if (rc != 0) {
        memset(out, 0, sizeof(device_entry_t) * DEVICE_STORE_MAX);
        return 0;
    }
    return count;
}

static int load(device_entry_t *buf, uint8_t *out_count) {
    void *h = NULL;
    if (g_ops->open(NS_V3, &h) != 0) return -1;
    uint8_t c = 0;
    g_ops->get_u8(h, K_COUNT, &c);
    if (c > DEVICE_STORE_MAX) c = DEVICE_STORE_MAX;
    size_t size = sizeof(device_entry_t) * DEVICE_STORE_MAX;
    if (g_ops->get_blob(h, K_DEVICES, buf, &size) != 0) {
        memset(buf, 0, sizeof(device_entry_t) * DEVICE_STORE_MAX);
    }
    *out_count = c;
    g_ops->close(h);
    return 0;
}

static int save(const device_entry_t *buf, uint8_t count) {
    void *h = NULL;
    if (g_ops->open(NS_V3, &h) != 0) return -1;
    g_ops->set_blob(h, K_DEVICES, buf, sizeof(device_entry_t) * DEVICE_STORE_MAX);
    g_ops->set_u8(h, K_COUNT, count);
    g_ops->commit(h);
    g_ops->close(h);
    return 0;
}

bool device_store_upsert(const device_entry_t *entry) {
    if (!g_ops || !entry) return false;
    device_entry_t buf[DEVICE_STORE_MAX];
    uint8_t count = 0;
    if (load(buf, &count) != 0) return false;

    for (uint8_t i = 0; i < count; ++i) {
        if (macs_equal(buf[i].bda, entry->bda)) {
            buf[i] = *entry;
            return save(buf, count) == 0;
        }
    }
    if (count < DEVICE_STORE_MAX) {
        buf[count++] = *entry;
        return save(buf, count) == 0;
    }
    // Full — shift oldest out.
    memmove(&buf[0], &buf[1], sizeof(device_entry_t) * (DEVICE_STORE_MAX - 1));
    buf[DEVICE_STORE_MAX - 1] = *entry;
    return save(buf, DEVICE_STORE_MAX) == 0;
}

bool device_store_remove(const uint8_t bda[6]) {
    if (!g_ops || !bda) return false;
    device_entry_t buf[DEVICE_STORE_MAX];
    uint8_t count = 0;
    if (load(buf, &count) != 0) return false;
    for (uint8_t i = 0; i < count; ++i) {
        if (macs_equal(buf[i].bda, bda)) {
            for (uint8_t j = i; j + 1 < count; ++j) buf[j] = buf[j + 1];
            count--;
            memset(&buf[count], 0, sizeof buf[0]);
            return save(buf, count) == 0;
        }
    }
    return false;
}

bool device_store_contains(const uint8_t bda[6]) {
    if (!g_ops || !bda) return false;
    device_entry_t buf[DEVICE_STORE_MAX];
    uint8_t count = 0;
    if (load(buf, &count) != 0) return false;
    for (uint8_t i = 0; i < count; ++i) {
        if (macs_equal(buf[i].bda, bda)) return true;
    }
    return false;
}

void device_store_clear(void) {
    if (!g_ops) return;
    void *h = NULL;
    if (g_ops->open(NS_V3, &h) != 0) return;
    g_ops->erase_all(h);
    g_ops->commit(h);
    g_ops->close(h);
}

void device_store_migrate_from_v2(void) {
    if (!g_ops) return;
    // If v3 already has entries, skip.
    device_entry_t existing[DEVICE_STORE_MAX];
    uint8_t existing_count = 0;
    if (load(existing, &existing_count) == 0 && existing_count > 0) return;

    void *h = NULL;
    if (g_ops->open(NS_V2, &h) != 0) return;
    int32_t status = 0;
    if (g_ops->get_i32(h, "ble_status", &status) != 0 || status != 1) {
        g_ops->close(h);
        return;
    }
    uint8_t bda[6] = {0};
    size_t  sz = sizeof bda;
    if (g_ops->get_blob(h, "ble_results", bda, &sz) != 0 || sz != 6) {
        g_ops->close(h);
        return;
    }
    g_ops->close(h);

    device_entry_t e = {0};
    memcpy(e.bda, bda, 6);
    e.addr_type = 0;
    device_store_upsert(&e);
}
