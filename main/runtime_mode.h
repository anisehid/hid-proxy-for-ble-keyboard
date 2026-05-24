#ifndef RUNTIME_MODE_H_
#define RUNTIME_MODE_H_

typedef enum {
    RUNTIME_MODE_RELAY = 0,
    RUNTIME_MODE_ADMIN = 1,
} runtime_mode_t;

// Initialize internal semaphore. Call once before any other function.
void runtime_mode_init(void);

runtime_mode_t runtime_mode_get(void);
void           runtime_mode_set(runtime_mode_t m);

#endif
