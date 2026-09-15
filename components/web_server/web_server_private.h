#ifndef WEB_SERVER_PRIVATE_H
#define WEB_SERVER_PRIVATE_H

#include <stdbool.h>

#include "esp_err.h"
#include "esp_http_server.h"

#define WEB_SERVER_MAX_BODY_LEN 1024

/** True when the request arrived on the SoftAP interface rather than the LAN. */
bool web_is_request_from_ap(httpd_req_t *req);

/**
 * @brief Enforce HTTP Basic Auth, except on the provisioning access point.
 *
 * @return ESP_OK to continue. On any other value the response has already been
 *         sent and the handler must return ESP_OK immediately.
 */
esp_err_t web_require_auth(httpd_req_t *req);

/** Read a JSON request body into a heap buffer the caller must free. */
esp_err_t web_read_body(httpd_req_t *req, char **out, size_t *out_len);

esp_err_t web_send_json(httpd_req_t *req, const char *json);
esp_err_t web_send_error(httpd_req_t *req, httpd_err_code_t code, const char *message);

/** Reboot after the given delay, so the response reaches the browser first. */
void web_schedule_restart(uint32_t delay_ms);

esp_err_t web_register_api_handlers(httpd_handle_t server);

#endif
