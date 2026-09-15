#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "cJSON.h"
#include "config_store.h"
#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_check.h"
#include "mqtt_manager.h"
#include "sdkconfig.h"
#include "sntp_manager.h"
#include "web_server_private.h"
#include "wifi_manager.h"

#define WEB_RESTART_DELAY_MS 1500

static const char *TAG = "web_api";

static const char *state_name(wifi_mgr_state_t state)
{
    switch (state) {
    case WIFI_MGR_STATE_CONNECTING:
        return "connecting";
    case WIFI_MGR_STATE_CONNECTED:
        return "connected";
    case WIFI_MGR_STATE_PROVISIONING:
        return "provisioning";
    case WIFI_MGR_STATE_AP_FALLBACK:
        return "ap_fallback";
    default:
        return "idle";
    }
}

/**
 * @brief Copy a string field out of a JSON object, leaving dest untouched when
 *        the field is absent or empty.
 *
 * An empty field means "keep what is stored", which lets the dashboard round
 * trip the form without ever holding the current password.
 */
static void json_copy_string(const cJSON *root, const char *key, char *dest, size_t dest_len)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(root, key);

    if (cJSON_IsString(item) && item->valuestring != NULL && item->valuestring[0] != '\0') {
        strlcpy(dest, item->valuestring, dest_len);
    }
}

/* ---------------------------------------------------------------- status -- */

static esp_err_t status_get_handler(httpd_req_t *req)
{
    const pluto_config_t *cfg = config_store_peek();
    wifi_mgr_status_t wifi;
    cJSON *root;
    cJSON *node;
    char device_id[WIFI_DEVICE_ID_LEN];
    char *body;

    if (web_require_auth(req) != ESP_OK) {
        return ESP_OK;
    }

    wifi_manager_get_status(&wifi);
    wifi_manager_get_device_id(device_id, sizeof(device_id));

    root = cJSON_CreateObject();
    if (root == NULL) {
        return web_send_error(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory");
    }

    cJSON_AddStringToObject(root, "device_id", device_id);
    cJSON_AddBoolToObject(root, "provisioned", config_store_is_provisioned());
    cJSON_AddNumberToObject(root, "uptime_sec", esp_timer_get_time() / 1000000);
    cJSON_AddNumberToObject(root, "heap_free", esp_get_free_heap_size());
    cJSON_AddNumberToObject(root, "heap_min_free", esp_get_minimum_free_heap_size());
    cJSON_AddStringToObject(root, "firmware", esp_app_get_description()->version);
    cJSON_AddStringToObject(root, "admin_user", CONFIG_PLUTO_ADMIN_USER);

    node = cJSON_AddObjectToObject(root, "wifi");
    cJSON_AddStringToObject(node, "state", state_name(wifi.state));
    cJSON_AddStringToObject(node, "ssid", wifi.ssid);
    cJSON_AddStringToObject(node, "ip", wifi.ip);
    cJSON_AddNumberToObject(node, "rssi", wifi.rssi);
    cJSON_AddNumberToObject(node, "channel", wifi.channel);
    cJSON_AddBoolToObject(node, "connected", wifi.sta_connected);
    cJSON_AddNumberToObject(node, "retries", wifi.retry_count);
    cJSON_AddNumberToObject(node, "next_retry_sec", wifi.next_retry_sec);
    cJSON_AddNumberToObject(node, "last_reason", wifi.last_disconnect_reason);
    cJSON_AddBoolToObject(node, "auth_suspected_bad", wifi.auth_suspected_bad);
    /* The stored passphrase is never sent back; the dashboard only needs to
     * know whether one exists so it can show the field as already set. */
    cJSON_AddBoolToObject(node, "password_set", cfg->wifi_password[0] != '\0');

    node = cJSON_AddObjectToObject(root, "ap");
    cJSON_AddBoolToObject(node, "active", wifi.ap_active);
    cJSON_AddStringToObject(node, "ssid", wifi.ap_ssid);
    cJSON_AddStringToObject(node, "ip", wifi.ap_ip);
    cJSON_AddNumberToObject(node, "clients", wifi.ap_client_count);

    node = cJSON_AddObjectToObject(root, "mqtt");
    cJSON_AddBoolToObject(node, "connected", mqtt_manager_is_connected());
    cJSON_AddStringToObject(node, "uri", cfg->mqtt_uri);
    cJSON_AddStringToObject(node, "username", cfg->mqtt_username);
    cJSON_AddBoolToObject(node, "password_set", cfg->mqtt_password[0] != '\0');

    node = cJSON_AddObjectToObject(root, "ntp");
    cJSON_AddBoolToObject(node, "synced", sntp_manager_is_synced());
    cJSON_AddStringToObject(node, "server", cfg->ntp_server);
    cJSON_AddStringToObject(node, "timezone", cfg->ntp_timezone);

    if (sntp_manager_is_synced()) {
        time_t now = time(NULL);
        struct tm timeinfo;
        char stamp[32];

        localtime_r(&now, &timeinfo);
        strftime(stamp, sizeof(stamp), "%Y-%m-%d %H:%M:%S", &timeinfo);
        cJSON_AddStringToObject(node, "time", stamp);
    }

    body = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    if (body == NULL) {
        return web_send_error(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory");
    }

    web_send_json(req, body);
    free(body);

    return ESP_OK;
}

/* ------------------------------------------------------------------ scan -- */

static esp_err_t scan_get_handler(httpd_req_t *req)
{
    wifi_mgr_ap_t networks[WIFI_SCAN_MAX_RESULTS];
    size_t count = 0;
    int64_t age_ms = -1;
    cJSON *root;
    cJSON *array;
    char *body;

    if (web_require_auth(req) != ESP_OK) {
        return ESP_OK;
    }

    wifi_manager_scan_get_results(networks, WIFI_SCAN_MAX_RESULTS, &count, &age_ms);

    /* A scan takes the shared radio off channel for a second or two, which
     * stalls anyone attached to the portal, so results are reused briefly
     * instead of rescanning on every refresh. */
    if (!wifi_manager_scan_in_progress() && (age_ms < 0 || age_ms > 10000)) {
        wifi_manager_scan_start();
    }

    root = cJSON_CreateObject();
    if (root == NULL) {
        return web_send_error(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory");
    }

    cJSON_AddBoolToObject(root, "scanning", wifi_manager_scan_in_progress());
    cJSON_AddNumberToObject(root, "age_ms", age_ms);

    array = cJSON_AddArrayToObject(root, "networks");
    for (size_t i = 0; i < count; i++) {
        cJSON *entry = cJSON_CreateObject();

        cJSON_AddStringToObject(entry, "ssid", networks[i].ssid);
        cJSON_AddNumberToObject(entry, "rssi", networks[i].rssi);
        cJSON_AddNumberToObject(entry, "channel", networks[i].channel);
        cJSON_AddBoolToObject(entry, "secure", networks[i].secure);
        cJSON_AddItemToArray(array, entry);
    }

    body = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    if (body == NULL) {
        return web_send_error(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory");
    }

    web_send_json(req, body);
    free(body);

    return ESP_OK;
}

/* ---------------------------------------------------------------- config -- */

static esp_err_t config_post_handler(httpd_req_t *req)
{
    pluto_config_t cfg;
    cJSON *root;
    char *body = NULL;
    size_t body_len = 0;
    char error[96] = {0};
    esp_err_t err;

    if (web_require_auth(req) != ESP_OK) {
        return ESP_OK;
    }

    err = web_read_body(req, &body, &body_len);
    if (err == ESP_ERR_INVALID_SIZE) {
        return web_send_error(req, HTTPD_400_BAD_REQUEST, "Body missing or too large");
    }
    if (err != ESP_OK) {
        return web_send_error(req, HTTPD_400_BAD_REQUEST, "Unable to read the request body");
    }

    root = cJSON_ParseWithLength(body, body_len);
    free(body);

    if (root == NULL) {
        return web_send_error(req, HTTPD_400_BAD_REQUEST, "Malformed JSON");
    }

    config_store_get(&cfg);

    json_copy_string(root, "wifi_ssid", cfg.wifi_ssid, sizeof(cfg.wifi_ssid));
    json_copy_string(root, "wifi_password", cfg.wifi_password, sizeof(cfg.wifi_password));
    json_copy_string(root, "mqtt_uri", cfg.mqtt_uri, sizeof(cfg.mqtt_uri));
    json_copy_string(root, "mqtt_username", cfg.mqtt_username, sizeof(cfg.mqtt_username));
    json_copy_string(root, "mqtt_password", cfg.mqtt_password, sizeof(cfg.mqtt_password));
    json_copy_string(root, "ntp_server", cfg.ntp_server, sizeof(cfg.ntp_server));
    json_copy_string(root, "ntp_timezone", cfg.ntp_timezone, sizeof(cfg.ntp_timezone));

    /* Changing the SSID invalidates the cached channel of the old network. */
    if (strcmp(cfg.wifi_ssid, config_store_peek()->wifi_ssid) != 0) {
        cfg.home_channel = 0;
    }

    cJSON_Delete(root);

    if (config_store_validate(&cfg, error, sizeof(error)) != ESP_OK) {
        return web_send_error(req, HTTPD_400_BAD_REQUEST, error);
    }

    cfg.provisioned = true;

    if (config_store_set(&cfg) != ESP_OK) {
        return web_send_error(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Unable to save the configuration");
    }

    web_send_json(req, "{\"ok\":true,\"restart_ms\":" "1500" "}");

    /* Restarting is what applies the change: it reconfigures the station, the
     * MQTT client and the NTP client in one go, through the same code path the
     * device uses on every boot. */
    ESP_LOGI(TAG, "Configuration accepted; restarting in %d ms", WEB_RESTART_DELAY_MS);
    web_schedule_restart(WEB_RESTART_DELAY_MS);

    return ESP_OK;
}

/* ----------------------------------------------------------- admin password */

static esp_err_t admin_pass_post_handler(httpd_req_t *req)
{
    cJSON *root;
    const cJSON *item;
    char *body = NULL;
    size_t body_len = 0;
    esp_err_t err;

    if (web_require_auth(req) != ESP_OK) {
        return ESP_OK;
    }

    err = web_read_body(req, &body, &body_len);
    if (err != ESP_OK) {
        return web_send_error(req, HTTPD_400_BAD_REQUEST, "Unable to read the request body");
    }

    root = cJSON_ParseWithLength(body, body_len);
    free(body);

    if (root == NULL) {
        return web_send_error(req, HTTPD_400_BAD_REQUEST, "Malformed JSON");
    }

    item = cJSON_GetObjectItemCaseSensitive(root, "password");

    if (!cJSON_IsString(item) || item->valuestring == NULL || strlen(item->valuestring) < 4) {
        cJSON_Delete(root);
        return web_send_error(req, HTTPD_400_BAD_REQUEST, "Password must be at least 4 characters");
    }

    err = config_store_set_admin_password(item->valuestring);
    cJSON_Delete(root);

    if (err != ESP_OK) {
        return web_send_error(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Unable to save the password");
    }

    return web_send_json(req, "{\"ok\":true}");
}

/* ------------------------------------------------------- reboot and reset -- */

static esp_err_t reboot_post_handler(httpd_req_t *req)
{
    if (web_require_auth(req) != ESP_OK) {
        return ESP_OK;
    }

    web_send_json(req, "{\"ok\":true,\"restart_ms\":1500}");
    web_schedule_restart(WEB_RESTART_DELAY_MS);

    return ESP_OK;
}

static esp_err_t factory_reset_post_handler(httpd_req_t *req)
{
    if (web_require_auth(req) != ESP_OK) {
        return ESP_OK;
    }

    if (config_store_factory_reset() != ESP_OK) {
        return web_send_error(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Unable to erase the configuration");
    }

    web_send_json(req, "{\"ok\":true,\"restart_ms\":1500}");
    web_schedule_restart(WEB_RESTART_DELAY_MS);

    return ESP_OK;
}

/* -------------------------------------------------------------- registry -- */

esp_err_t web_register_api_handlers(httpd_handle_t server)
{
    static const httpd_uri_t handlers[] = {
        {.uri = "/api/status", .method = HTTP_GET, .handler = status_get_handler},
        {.uri = "/api/scan", .method = HTTP_GET, .handler = scan_get_handler},
        {.uri = "/api/config", .method = HTTP_POST, .handler = config_post_handler},
        {.uri = "/api/admin-pass", .method = HTTP_POST, .handler = admin_pass_post_handler},
        {.uri = "/api/reboot", .method = HTTP_POST, .handler = reboot_post_handler},
        {.uri = "/api/factory-reset", .method = HTTP_POST, .handler = factory_reset_post_handler},
    };

    for (size_t i = 0; i < sizeof(handlers) / sizeof(handlers[0]); i++) {
        ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &handlers[i]), TAG, "%s", handlers[i].uri);
    }

    return ESP_OK;
}
