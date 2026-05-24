#ifndef WEB_SERVER_H_
#define WEB_SERVER_H_

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

struct httpd_req;

esp_err_t web_server_start(void);
esp_err_t web_server_stop(void);

// Touched by every authenticated request; consumed by the idle-shutdown task (Task 14).
int64_t web_server_last_activity_us(void);

// Returns true if the request carries a valid (non-expired, well-signed) session cookie.
bool web_request_authenticated(struct httpd_req *req);

#endif
