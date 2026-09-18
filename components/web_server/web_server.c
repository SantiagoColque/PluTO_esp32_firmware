#include "web_server.h"

#include <stdlib.h>
#include <string.h>

#include "config_store.h"
#include "dns_hijack.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "lwip/sockets.h"
#include "mbedtls/base64.h"
#include "sdkconfig.h"
#include "web_server_private.h"

#define WEB_AP_ADDRESS "192.168.4.1"
#define WEB_PORTAL_URL "http://192.168.4.1/"
#define WEB_HTTPD_STACK 8192
#define WEB_HTTPD_MAX_SOCKETS 7
/* "Basic " plus the base64 of "user:password". Base64 expands by 4/3, so the
 * encoded form of a WEB_CREDENTIALS_LEN buffer needs ceil(96/3)*4 + 1 bytes. */
#define WEB_CREDENTIALS_LEN 96
#define WEB_AUTH_ENCODED_LEN 132
#define WEB_AUTH_HEADER_LEN 160

extern const uint8_t index_html_gz_start[] asm("_binary_index_html_gz_start");
extern const uint8_t index_html_gz_end[] asm("_binary_index_html_gz_end");

static const char *TAG = "web_server";

static httpd_handle_t s_server;
static dns_hijack_handle_t s_dns;
static esp_timer_handle_t s_restart_timer;
static char s_expected_auth[WEB_AUTH_HEADER_LEN];

/* ------------------------------------------------------------- utilities -- */

bool web_is_request_from_ap(httpd_req_t *req)
{
    struct sockaddr_in6 local;
    socklen_t len = sizeof(local);
    int fd = httpd_req_to_sockfd(req);
    char addr[INET_ADDRSTRLEN];

    if (fd < 0 || getsockname(fd, (struct sockaddr *)&local, &len) != 0) {
        return false;
    }

    if (local.sin6_family == PF_INET) {
        inet_ntoa_r(((struct sockaddr_in *)&local)->sin_addr, addr, sizeof(addr));
    } else {
        /* lwIP hands back IPv4-mapped addresses on a dual-stack socket. */
        inet_ntoa_r(*(struct in_addr *)&local.sin6_addr.un.u32_addr[3], addr, sizeof(addr));
    }

    return strcmp(addr, WEB_AP_ADDRESS) == 0;
}

static void rebuild_expected_auth(void)
{
    const pluto_config_t *cfg = config_store_peek();
    char credentials[WEB_CREDENTIALS_LEN];
    unsigned char encoded[WEB_AUTH_ENCODED_LEN];
    size_t encoded_len = 0;
    int len;

    len = snprintf(credentials, sizeof(credentials), "%s:%s", CONFIG_PLUTO_ADMIN_USER, cfg->admin_password);
    if (len < 0 || len >= (int)sizeof(credentials)) {
        s_expected_auth[0] = '\0';
        return;
    }

    /* Encoding the expected value is cheaper than decoding what the browser
     * sent, and avoids handling malformed base64 altogether. */
    if (mbedtls_base64_encode(encoded, sizeof(encoded), &encoded_len, (const unsigned char *)credentials, len) != 0) {
        s_expected_auth[0] = '\0';
        return;
    }

    encoded[encoded_len] = '\0';
    snprintf(s_expected_auth, sizeof(s_expected_auth), "Basic %s", encoded);
}

esp_err_t web_require_auth(httpd_req_t *req)
{
    char header[WEB_AUTH_HEADER_LEN];

    /* The provisioning access point is open on purpose: there is no password
     * the user could know yet, and locking the portal would make a device that
     * lost its network unrecoverable. */
    if (web_is_request_from_ap(req)) {
        return ESP_OK;
    }

    rebuild_expected_auth();
    if (s_expected_auth[0] == '\0') {
        return ESP_OK;
    }

    if (httpd_req_get_hdr_value_str(req, "Authorization", header, sizeof(header)) == ESP_OK &&
        strcmp(header, s_expected_auth) == 0) {
        return ESP_OK;
    }

    httpd_resp_set_status(req, "401 Unauthorized");
    httpd_resp_set_hdr(req, "WWW-Authenticate", "Basic realm=\"PluTO\"");
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_sendstr(req, "Authentication required");

    return ESP_FAIL;
}

esp_err_t web_send_json(httpd_req_t *req, const char *json)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");

    return httpd_resp_sendstr(req, json);
}

esp_err_t web_send_error(httpd_req_t *req, httpd_err_code_t code, const char *message)
{
    return httpd_resp_send_err(req, code, message);
}

esp_err_t web_read_body(httpd_req_t *req, char **out, size_t *out_len)
{
    char *buf;
    size_t total = 0;

    *out = NULL;
    *out_len = 0;

    if (req->content_len == 0 || req->content_len > WEB_SERVER_MAX_BODY_LEN) {
        return ESP_ERR_INVALID_SIZE;
    }

    buf = malloc(req->content_len + 1);
    if (buf == NULL) {
        return ESP_ERR_NO_MEM;
    }

    while (total < req->content_len) {
        int received = httpd_req_recv(req, buf + total, req->content_len - total);

        if (received == HTTPD_SOCK_ERR_TIMEOUT) {
            /* A short read is normal; only a real error aborts. */
            continue;
        }

        if (received <= 0) {
            free(buf);
            return ESP_FAIL;
        }

        total += received;
    }

    buf[total] = '\0';
    *out = buf;
    *out_len = total;

    return ESP_OK;
}

static void restart_timer_cb(void *arg)
{
    ESP_LOGW(TAG, "Restarting to apply the new configuration");
    esp_restart();
}

void web_schedule_restart(uint32_t delay_ms)
{
    if (s_restart_timer == NULL) {
        const esp_timer_create_args_t args = {
            .callback = restart_timer_cb,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "web_restart",
        };

        if (esp_timer_create(&args, &s_restart_timer) != ESP_OK) {
            return;
        }
    }

    esp_timer_stop(s_restart_timer);
    esp_timer_start_once(s_restart_timer, (uint64_t)delay_ms * 1000ULL);
}

/* -------------------------------------------------------------- handlers -- */

static esp_err_t root_get_handler(httpd_req_t *req)
{
    if (web_require_auth(req) != ESP_OK) {
        return ESP_OK;
    }

    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");

    return httpd_resp_send(req,
                           (const char *)index_html_gz_start,
                           index_html_gz_end - index_html_gz_start);
}

static esp_err_t captive_404_handler(httpd_req_t *req, httpd_err_code_t err)
{
    /* Only hijack unknown paths on the portal side. Doing this on the LAN would
     * mask genuine 404s and swallow the dashboard's own failed fetches. */
    if (!web_is_request_from_ap(req)) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Not found");
        return ESP_FAIL;
    }

    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", WEB_PORTAL_URL);
    httpd_resp_set_type(req, "text/html");

    /* iOS does not treat a bodyless redirect as a captive portal, so the
     * redirect carries a page of its own. */
    httpd_resp_sendstr(req,
                       "<!doctype html><meta charset=\"utf-8\">"
                       "<title>PluTO</title>"
                       "<a href=\"" WEB_PORTAL_URL "\">Configurar PluTO</a>");

    return ESP_OK;
}

/* ------------------------------------------------------------- lifecycle -- */

void web_server_set_captive(bool enabled)
{
    if (enabled && s_dns == NULL) {
        s_dns = dns_hijack_start("WIFI_AP_DEF");
    } else if (!enabled && s_dns != NULL) {
        dns_hijack_stop(s_dns);
        s_dns = NULL;
    }
}

esp_err_t web_server_start(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    const httpd_uri_t root = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_get_handler,
    };

    if (s_server != NULL) {
        return ESP_OK;
    }

    config.stack_size = WEB_HTTPD_STACK;
    config.max_open_sockets = WEB_HTTPD_MAX_SOCKETS;
    config.max_uri_handlers = 10;
    /* Browsers open several parallel connections and keep them alive; without
     * this the server refuses new ones and the page half loads. */
    config.lru_purge_enable = true;
    config.recv_wait_timeout = 10;
    config.send_wait_timeout = 10;

    ESP_RETURN_ON_ERROR(httpd_start(&s_server, &config), TAG, "Unable to start the HTTP server");

    ESP_ERROR_CHECK_WITHOUT_ABORT(httpd_register_uri_handler(s_server, &root));
    ESP_ERROR_CHECK_WITHOUT_ABORT(web_register_api_handlers(s_server));
    ESP_ERROR_CHECK_WITHOUT_ABORT(httpd_register_err_handler(s_server, HTTPD_404_NOT_FOUND, captive_404_handler));

    /* Captive portal probing produces a flood of these otherwise. */
    esp_log_level_set("httpd_uri", ESP_LOG_ERROR);
    esp_log_level_set("httpd_txrx", ESP_LOG_ERROR);
    esp_log_level_set("httpd_parse", ESP_LOG_ERROR);

    ESP_LOGI(TAG, "HTTP server listening on port %d", config.server_port);

    return ESP_OK;
}

esp_err_t web_server_stop(void)
{
    web_server_set_captive(false);

    if (s_server == NULL) {
        return ESP_OK;
    }

    httpd_stop(s_server);
    s_server = NULL;

    return ESP_OK;
}

bool web_server_is_running(void)
{
    return s_server != NULL;
}
