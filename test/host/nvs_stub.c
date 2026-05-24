#include "nvs_stub.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_ENTRIES 32
#define MAX_NS      4
#define MAX_KEY     32

typedef struct {
    char    ns[MAX_KEY];
    char    key[MAX_KEY];
    uint8_t bytes[256];
    size_t  size;
    int     kind; // 0=blob, 1=u8, 2=i32
} entry_t;

static entry_t g_entries[MAX_ENTRIES];
static int     g_count = 0;

static entry_t *find(const char *ns, const char *key) {
    for (int i = 0; i < g_count; ++i) {
        if (strcmp(g_entries[i].ns, ns) == 0 &&
            strcmp(g_entries[i].key, key) == 0) {
            return &g_entries[i];
        }
    }
    return NULL;
}

static entry_t *put(const char *ns, const char *key) {
    entry_t *e = find(ns, key);
    if (e) return e;
    if (g_count >= MAX_ENTRIES) return NULL;
    e = &g_entries[g_count++];
    snprintf(e->ns,  sizeof e->ns,  "%s", ns);
    snprintf(e->key, sizeof e->key, "%s", key);
    e->size = 0;
    return e;
}

static const char *current_ns = NULL;

static int stub_open(const char *ns, void **out_handle) {
    current_ns  = ns;
    *out_handle = (void *)ns;
    return 0;
}
static int stub_get_blob(void *h, const char *key, void *buf, size_t *size) {
    entry_t *e = find((const char *)h, key);
    if (!e || e->kind != 0) return -1;
    if (buf == NULL) { *size = e->size; return 0; }
    if (*size < e->size) return -1;
    memcpy(buf, e->bytes, e->size);
    *size = e->size;
    return 0;
}
static int stub_set_blob(void *h, const char *key, const void *buf, size_t size) {
    entry_t *e = put((const char *)h, key);
    if (!e) return -1;
    if (size > sizeof e->bytes) return -1;
    memcpy(e->bytes, buf, size);
    e->size = size;
    e->kind = 0;
    return 0;
}
static int stub_get_u8(void *h, const char *key, uint8_t *out) {
    entry_t *e = find((const char *)h, key);
    if (!e || e->kind != 1) return -1;
    *out = e->bytes[0];
    return 0;
}
static int stub_set_u8(void *h, const char *key, uint8_t in) {
    entry_t *e = put((const char *)h, key);
    if (!e) return -1;
    e->bytes[0] = in;
    e->size     = 1;
    e->kind     = 1;
    return 0;
}
static int stub_get_i32(void *h, const char *key, int32_t *out) {
    entry_t *e = find((const char *)h, key);
    if (!e || e->kind != 2) return -1;
    memcpy(out, e->bytes, sizeof *out);
    return 0;
}
static int stub_set_i32(void *h, const char *key, int32_t in) {
    entry_t *e = put((const char *)h, key);
    if (!e) return -1;
    memcpy(e->bytes, &in, sizeof in);
    e->size = sizeof in;
    e->kind = 2;
    return 0;
}
static int stub_erase_all(void *h) {
    const char *ns = (const char *)h;
    int w = 0;
    for (int i = 0; i < g_count; ++i) {
        if (strcmp(g_entries[i].ns, ns) != 0) g_entries[w++] = g_entries[i];
    }
    g_count = w;
    return 0;
}
static int  stub_commit(void *h) { (void)h; return 0; }
static void stub_close (void *h) { (void)h; }

static const nvs_ops_t g_stub_ops = {
    .open = stub_open, .get_blob = stub_get_blob, .set_blob = stub_set_blob,
    .get_u8 = stub_get_u8, .set_u8 = stub_set_u8,
    .get_i32 = stub_get_i32, .set_i32 = stub_set_i32,
    .erase_all = stub_erase_all, .commit = stub_commit, .close = stub_close,
};

void nvs_stub_install(void) { device_store_set_ops(&g_stub_ops); }

void nvs_stub_reset(void) { g_count = 0; }

void nvs_stub_seed_v2_device(const uint8_t bda[6]) {
    void *h = NULL;
    stub_open("hidproxy_v2", &h);
    int32_t one = 1;
    stub_set_i32(h, "ble_status", one);
    stub_set_blob(h, "ble_results", bda, 6);
}
