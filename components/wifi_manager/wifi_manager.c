#include "wifi_manager.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/param.h>

#include "config_store.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#define WIFI_CONNECTED_BIT BIT0

#define WIFI_BACKOFF_BASE_SEC 1
#define WIFI_BACKOFF_MAX_SEC CONFIG_WIFI_RECONNECT_MAX_BACKOFF_SEC
#define WIFI_AP_FALLBACK_SEC CONFIG_WIFI_AP_FALLBACK_SEC
#define WIFI_AP_TEARDOWN_GRACE_SEC 60
#define WIFI_AUTH_FAIL_STREAK 3
#define WIFI_AP_MAX_CLIENTS 4
#define WIFI_AP_IP "192.168.4.1"
#define WIFI_PORTAL_URI "http://192.168.4.1"
#define WIFI_SCAN_CACHE_MS 10000

#define WIFI_TASK_STACK 4096
#define WIFI_TASK_PRIORITY 5
#define WIFI_QUEUE_LEN 12

static const char *TAG = "wifi_manager";

typedef enum {
    CMD_EVT_STA_START,
    CMD_EVT_DISCONNECTED,
    CMD_EVT_GOT_IP,
    CMD_EVT_AP_CLIENTS_CHANGED,
    CMD_RECONNECT_TIMER,
    CMD_AP_FALLBACK_TIMER,
    CMD_AP_TEARDOWN_TIMER,
    CMD_START_STA,
    CMD_START_AP,
    CMD_STOP_AP,
    CMD_SCAN,
} wifi_cmd_id_t;

typedef struct {
    wifi_cmd_id_t id;
    uint8_t reason;
} wifi_cmd_t;

static EventGroupHandle_t s_event_group;
static QueueHandle_t s_queue;
static TaskHandle_t s_task;

static esp_netif_t *s_sta_netif;
static esp_netif_t *s_ap_netif;

static esp_timer_handle_t s_reconnect_timer;
static esp_timer_handle_t s_ap_fallback_timer;
static esp_timer_handle_t s_ap_teardown_timer;

static wifi_mgr_state_t s_state = WIFI_MGR_STATE_IDLE;
static bool s_sta_connected;
static bool s_ap_active;
static bool s_self_disconnect;
static bool s_scan_pending;
static bool s_scan_in_progress;
static bool s_auth_suspected_bad;

static uint32_t s_backoff_sec;
static uint32_t s_retry_count;
static uint32_t s_auth_fail_streak;
static uint8_t s_ap_client_count;
static uint8_t s_last_reason;

static char s_sta_ip[16];
static char s_ap_ssid[WIFI_AP_SSID_LEN];
static int8_t s_rssi;
static uint8_t s_channel;

static wifi_mgr_ap_t s_scan_results[WIFI_SCAN_MAX_RESULTS];
static size_t s_scan_count;
static int64_t s_scan_timestamp_us;

/* esp_netif_dhcps_option() stores this pointer rather than copying the string,
 * so it has to outlive the call. */
static const char s_portal_uri[] = WIFI_PORTAL_URI;

static void post_cmd(wifi_cmd_id_t id, uint8_t reason)
{
    wifi_cmd_t cmd = {.id = id, .reason = reason};

    if (s_queue != NULL) {
        xQueueSend(s_queue, &cmd, 0);
    }
}

static void post_cmd_from_isr_safe(wifi_cmd_id_t id)
{
    post_cmd(id, 0);
}

/* ---------------------------------------------------------------- timers -- */

static void reconnect_timer_cb(void *arg)
{
    post_cmd_from_isr_safe(CMD_RECONNECT_TIMER);
}

static void ap_fallback_timer_cb(void *arg)
{
    post_cmd_from_isr_safe(CMD_AP_FALLBACK_TIMER);
}

static void ap_teardown_timer_cb(void *arg)
{
    post_cmd_from_isr_safe(CMD_AP_TEARDOWN_TIMER);
}

static void timer_arm_once(esp_timer_handle_t timer, uint64_t delay_us)
{
    /* esp_timer_start_once() fails with ESP_ERR_INVALID_STATE on an already
     * armed timer, so always disarm first. */
    esp_timer_stop(timer);
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_timer_start_once(timer, delay_us));
}

static uint32_t next_backoff_sec(void)
{
    if (s_backoff_sec == 0) {
        s_backoff_sec = WIFI_BACKOFF_BASE_SEC;
    } else {
        s_backoff_sec = MIN(s_backoff_sec * 2, (uint32_t)WIFI_BACKOFF_MAX_SEC);
    }

    /* While somebody is on the portal, stop running short retries underneath
     * them: every attempt drags the shared radio off the AP's channel. */
    if (s_ap_client_count > 0) {
        s_backoff_sec = WIFI_BACKOFF_MAX_SEC;
    }

    return s_backoff_sec;
}

static void schedule_reconnect(void)
{
    uint32_t delay_sec = next_backoff_sec();
    uint64_t delay_us = (uint64_t)delay_sec * 1000000ULL;

    /* Downward-only jitter so a room full of devices does not stampede a router
     * the moment it finishes rebooting, while the worst case stays exactly at
     * the ceiling the RNF-06.1 budget is computed from. */
    delay_us -= (uint64_t)(esp_random() % (delay_sec * 200000ULL));

    s_retry_count++;
    ESP_LOGI(TAG, "Reconnecting in %.1f s (attempt %lu)", delay_us / 1000000.0, (unsigned long)s_retry_count);

    timer_arm_once(s_reconnect_timer, delay_us);
}

/* ------------------------------------------------------------- utilities -- */

static bool reason_is_auth_failure(uint8_t reason)
{
    /* Deliberately excludes the handshake timeouts: the ESP32 driver reports
     * those for weak signal and for a busy access point at least as often as
     * for a wrong passphrase, and treating them as bad credentials raises the
     * portal on a perfectly good network at the edge of range. */
    switch (reason) {
    case WIFI_REASON_AUTH_FAIL:
    case WIFI_REASON_802_1X_AUTH_FAILED:
    case WIFI_REASON_MIC_FAILURE:
        return true;
    default:
        return false;
    }
}

static void refresh_link_info(void)
{
    wifi_ap_record_t ap_info;

    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
        s_rssi = ap_info.rssi;
        s_channel = ap_info.primary;
        config_store_set_home_channel(ap_info.primary);
    }
}

static esp_err_t apply_sta_config(void)
{
    const pluto_config_t *cfg = config_store_peek();
    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
        },
    };

    if (cfg->wifi_ssid[0] == '\0') {
        return ESP_ERR_INVALID_STATE;
    }

    /* An open network cannot clear a WPA2 threshold. */
    if (cfg->wifi_password[0] == '\0') {
        wifi_config.sta.threshold.authmode = WIFI_AUTH_OPEN;
    }

    strlcpy((char *)wifi_config.sta.ssid, cfg->wifi_ssid, sizeof(wifi_config.sta.ssid));
    strlcpy((char *)wifi_config.sta.password, cfg->wifi_password, sizeof(wifi_config.sta.password));

    return esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
}

static esp_err_t apply_ap_config(void)
{
    const pluto_config_t *cfg = config_store_peek();
    wifi_config_t ap_config = {
        .ap = {
            .ssid_len = 0,
            .max_connection = WIFI_AP_MAX_CLIENTS,
            .authmode = WIFI_AUTH_OPEN,
        },
    };

    strlcpy((char *)ap_config.ap.ssid, s_ap_ssid, sizeof(ap_config.ap.ssid));

    /* Single radio: in AP+STA the access point is dragged onto whatever channel
     * the station lands on. Starting it on the last known channel of the home
     * network means a phone on the portal does not lose the AP the instant the
     * station associates. */
    ap_config.ap.channel = (cfg->home_channel >= 1 && cfg->home_channel <= 13) ? cfg->home_channel : 1;

    return esp_wifi_set_config(WIFI_IF_AP, &ap_config);
}

static void advertise_captive_portal(void)
{
    if (s_ap_netif == NULL) {
        return;
    }

    /* RFC 8910. Recent Android and iOS open the portal from this alone, with no
     * DNS interception at all; the DNS hijack stays as the fallback for older
     * clients. */
    esp_netif_dhcps_stop(s_ap_netif);
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_netif_dhcps_option(s_ap_netif,
                                                         ESP_NETIF_OP_SET,
                                                         ESP_NETIF_CAPTIVEPORTAL_URI,
                                                         (void *)s_portal_uri,
                                                         strlen(s_portal_uri)));
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_netif_dhcps_start(s_ap_netif));
}

/* ------------------------------------------------------ command handlers -- */

static void handle_start_sta(void)
{
    /* Order matters: esp_wifi_set_config() rejects an interface that the
     * current mode has not enabled, and the driver would otherwise start with
     * whatever credentials it cached in nvs.net80211 on a previous run. */
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_set_mode(s_ap_active ? WIFI_MODE_APSTA : WIFI_MODE_STA));

    if (apply_sta_config() != ESP_OK) {
        ESP_LOGE(TAG, "No stored SSID; cannot start the station");
        return;
    }

    s_state = WIFI_MGR_STATE_CONNECTING;
    s_backoff_sec = 0;
    s_retry_count = 0;

    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_start());

    /* A provisioned device that cannot reach its network must still be
     * reconfigurable, so raise the portal after a while and keep retrying
     * underneath it. */
    timer_arm_once(s_ap_fallback_timer, (uint64_t)WIFI_AP_FALLBACK_SEC * 1000000ULL);
}

static void handle_start_ap(bool provisioning_only)
{
    if (s_ap_active) {
        return;
    }

    /* AP+STA even while provisioning, although nothing is going to be joined:
     * scanning needs the station interface to exist, and the portal's network
     * picker is useless without it. The station is left idle rather than
     * connecting, so it never drags the access point onto another channel. */
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_set_mode(WIFI_MODE_APSTA));

    if (apply_ap_config() != ESP_OK) {
        ESP_LOGE(TAG, "Unable to configure the access point");
        return;
    }

    /* Set before starting: esp_wifi_start() raises WIFI_EVENT_STA_START, and
     * the handler for it consults the state to decide whether to connect. */
    s_state = provisioning_only ? WIFI_MGR_STATE_PROVISIONING : WIFI_MGR_STATE_AP_FALLBACK;

    if (provisioning_only) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_start());
    }

    s_ap_active = true;
    ESP_LOGW(TAG, "Provisioning access point \"%s\" is up at %s", s_ap_ssid, WIFI_AP_IP);
}

static void handle_stop_ap(void)
{
    if (!s_ap_active) {
        return;
    }

    esp_timer_stop(s_ap_teardown_timer);

    if (s_ap_client_count > 0) {
        ESP_LOGI(TAG, "Keeping the access point up: %d client(s) still attached", s_ap_client_count);
        timer_arm_once(s_ap_teardown_timer, (uint64_t)WIFI_AP_TEARDOWN_GRACE_SEC * 1000000ULL);
        return;
    }

    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_set_mode(WIFI_MODE_STA));
    s_ap_active = false;
    s_state = s_sta_connected ? WIFI_MGR_STATE_CONNECTED : WIFI_MGR_STATE_CONNECTING;

    ESP_LOGI(TAG, "Access point down; the dashboard stays reachable at %s", s_sta_ip);
}

static void handle_disconnected(uint8_t reason)
{
    s_sta_connected = false;
    s_last_reason = reason;
    s_sta_ip[0] = '\0';
    xEventGroupClearBits(s_event_group, WIFI_CONNECTED_BIT);

    if (s_self_disconnect) {
        /* Our own esp_wifi_disconnect(), not a network problem: do not let the
         * backoff race whatever asked for the disconnect. */
        s_self_disconnect = false;

        if (s_scan_pending) {
            post_cmd(CMD_SCAN, 0);
        }
        return;
    }

    if (s_state == WIFI_MGR_STATE_PROVISIONING) {
        return;
    }

    if (reason_is_auth_failure(reason)) {
        s_auth_fail_streak++;
        ESP_LOGW(TAG,
                 "WiFi authentication rejected (reason %d, %lu in a row)",
                 reason,
                 (unsigned long)s_auth_fail_streak);

        if (s_auth_fail_streak >= WIFI_AUTH_FAIL_STREAK && !s_ap_active) {
            s_auth_suspected_bad = true;
            ESP_LOGE(TAG, "Credentials look wrong; raising the portal early");
            handle_start_ap(false);
        }
    } else {
        ESP_LOGW(TAG, "WiFi disconnected (reason %d)", reason);
    }

    if (s_state != WIFI_MGR_STATE_AP_FALLBACK) {
        s_state = WIFI_MGR_STATE_CONNECTING;
    }

    schedule_reconnect();
}

static void handle_got_ip(void)
{
    esp_netif_ip_info_t ip_info;

    s_sta_connected = true;
    s_auth_fail_streak = 0;
    s_auth_suspected_bad = false;
    s_backoff_sec = 0;
    s_retry_count = 0;

    esp_timer_stop(s_reconnect_timer);
    esp_timer_stop(s_ap_fallback_timer);

    if (esp_netif_get_ip_info(s_sta_netif, &ip_info) == ESP_OK) {
        snprintf(s_sta_ip, sizeof(s_sta_ip), IPSTR, IP2STR(&ip_info.ip));
    }

    refresh_link_info();

    s_state = s_ap_active ? WIFI_MGR_STATE_AP_FALLBACK : WIFI_MGR_STATE_CONNECTED;
    xEventGroupSetBits(s_event_group, WIFI_CONNECTED_BIT);

    ESP_LOGI(TAG, "IP assigned: %s (channel %d, RSSI %d)", s_sta_ip, s_channel, s_rssi);

    if (s_ap_active) {
        /* Give anyone mid-submit on the portal a moment before pulling it. */
        timer_arm_once(s_ap_teardown_timer, (uint64_t)WIFI_AP_TEARDOWN_GRACE_SEC * 1000000ULL);
    }
}

static void handle_scan(void)
{
    wifi_scan_config_t scan_config = {
        .show_hidden = false,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time.active = {.min = 60, .max = 120},
    };
    wifi_ap_record_t *records;
    uint16_t number = WIFI_SCAN_MAX_RESULTS;
    esp_err_t err;

    /* A scan is rejected outright while the station is mid-association, which
     * is the normal state in AP fallback. Step out of the way first and pick
     * the scan back up from the disconnect handler. */
    if (s_state == WIFI_MGR_STATE_CONNECTING && !s_scan_pending) {
        s_scan_pending = true;
        s_self_disconnect = true;
        esp_timer_stop(s_reconnect_timer);
        esp_wifi_disconnect();
        return;
    }

    s_scan_pending = false;
    s_scan_in_progress = true;

    records = calloc(WIFI_SCAN_MAX_RESULTS, sizeof(wifi_ap_record_t));
    if (records == NULL) {
        s_scan_in_progress = false;
        return;
    }

    err = esp_wifi_scan_start(&scan_config, true);
    if (err == ESP_OK) {
        err = esp_wifi_scan_get_ap_records(&number, records);
    }

    if (err == ESP_OK) {
        s_scan_count = 0;
        for (uint16_t i = 0; i < number && s_scan_count < WIFI_SCAN_MAX_RESULTS; i++) {
            if (records[i].ssid[0] == '\0') {
                continue;
            }
            strlcpy(s_scan_results[s_scan_count].ssid,
                    (const char *)records[i].ssid,
                    sizeof(s_scan_results[s_scan_count].ssid));
            s_scan_results[s_scan_count].rssi = records[i].rssi;
            s_scan_results[s_scan_count].channel = records[i].primary;
            s_scan_results[s_scan_count].secure = records[i].authmode != WIFI_AUTH_OPEN;
            s_scan_count++;
        }
        s_scan_timestamp_us = esp_timer_get_time();
        ESP_LOGI(TAG, "Scan finished: %d network(s)", (int)s_scan_count);
    } else {
        ESP_LOGW(TAG, "Scan failed: %s", esp_err_to_name(err));
    }

    free(records);
    s_scan_in_progress = false;

    /* Scanning left the station idle; resume the normal retry cycle at once
     * instead of waiting out a backoff that no longer applies. */
    if (!s_sta_connected && s_state != WIFI_MGR_STATE_PROVISIONING) {
        s_backoff_sec = 0;
        esp_wifi_connect();
    }
}

static void wifi_manager_task(void *arg)
{
    wifi_cmd_t cmd;

    while (xQueueReceive(s_queue, &cmd, portMAX_DELAY) == pdTRUE) {
        switch (cmd.id) {
        case CMD_EVT_STA_START:
            /* While provisioning the station is deliberately idle: it exists so
             * the portal can scan, and there are no credentials to try yet. */
            if (s_state != WIFI_MGR_STATE_PROVISIONING) {
                esp_wifi_connect();
            }
            break;

        case CMD_EVT_DISCONNECTED:
            handle_disconnected(cmd.reason);
            break;

        case CMD_EVT_GOT_IP:
            handle_got_ip();
            break;

        case CMD_EVT_AP_CLIENTS_CHANGED:
            ESP_LOGI(TAG, "Access point clients: %d", s_ap_client_count);
            break;

        case CMD_RECONNECT_TIMER:
            if (!s_sta_connected) {
                esp_wifi_connect();
            }
            break;

        case CMD_AP_FALLBACK_TIMER:
            if (!s_sta_connected && !s_ap_active) {
                ESP_LOGW(TAG,
                         "No address after %d s; raising the portal while the station keeps retrying",
                         WIFI_AP_FALLBACK_SEC);
                handle_start_ap(false);
            }
            break;

        case CMD_AP_TEARDOWN_TIMER:
        case CMD_STOP_AP:
            handle_stop_ap();
            break;

        case CMD_START_STA:
            handle_start_sta();
            break;

        case CMD_START_AP:
            handle_start_ap(true);
            break;

        case CMD_SCAN:
            handle_scan();
            break;
        }
    }

    vTaskDelete(NULL);
}

/* --------------------------------------------------------- event handler -- */

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    /* The system event task has a small stack and posting into the WiFi driver
     * from here can deadlock against it, so this only ever enqueues work. */
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
        case WIFI_EVENT_STA_START:
            post_cmd(CMD_EVT_STA_START, 0);
            break;

        case WIFI_EVENT_STA_DISCONNECTED: {
            wifi_event_sta_disconnected_t *event = event_data;
            post_cmd(CMD_EVT_DISCONNECTED, event->reason);
            break;
        }

        case WIFI_EVENT_AP_STACONNECTED:
            s_ap_client_count++;
            post_cmd(CMD_EVT_AP_CLIENTS_CHANGED, 0);
            break;

        case WIFI_EVENT_AP_STADISCONNECTED:
            if (s_ap_client_count > 0) {
                s_ap_client_count--;
            }
            post_cmd(CMD_EVT_AP_CLIENTS_CHANGED, 0);
            break;

        default:
            break;
        }
        return;
    }

    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        post_cmd(CMD_EVT_GOT_IP, 0);
    }
}

/* --------------------------------------------------------------- public -- */

esp_err_t wifi_manager_get_device_id(char *buf, size_t buf_len)
{
    uint8_t mac[6];

    if (buf == NULL || buf_len < WIFI_DEVICE_ID_LEN) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Read from eFuse rather than from the driver: the provisioning SSID needs
     * the device id before the WiFi driver has been started. */
    ESP_RETURN_ON_ERROR(esp_read_mac(mac, ESP_MAC_WIFI_STA), TAG, "Unable to read the station MAC address");

    snprintf(buf, buf_len, "%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    return ESP_OK;
}

esp_err_t wifi_manager_get_ap_ssid(char *buf, size_t buf_len)
{
    if (buf == NULL || buf_len < sizeof(s_ap_ssid)) {
        return ESP_ERR_INVALID_ARG;
    }

    strlcpy(buf, s_ap_ssid, buf_len);

    return ESP_OK;
}

esp_err_t wifi_manager_init(void)
{
    uint8_t mac[6];
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    if (s_event_group != NULL) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(esp_read_mac(mac, ESP_MAC_WIFI_STA), TAG, "Unable to read the station MAC address");
    snprintf(s_ap_ssid, sizeof(s_ap_ssid), CONFIG_PLUTO_AP_SSID_PREFIX "%02x%02x%02x", mac[3], mac[4], mac[5]);

    s_event_group = xEventGroupCreate();
    if (s_event_group == NULL) {
        return ESP_ERR_NO_MEM;
    }

    s_queue = xQueueCreate(WIFI_QUEUE_LEN, sizeof(wifi_cmd_t));
    if (s_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* Both interfaces exist from the start. An access point netif that is not
     * in the current mode costs nothing and avoids creating and destroying it
     * every time the portal is raised. */
    s_sta_netif = esp_netif_create_default_wifi_sta();
    s_ap_netif = esp_netif_create_default_wifi_ap();

    /* The DHCP server has not started yet at this point, which is exactly when
     * its options can be set. Doing it here also keeps these esp_netif calls
     * off the system event task. */
    advertise_captive_portal();

    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    const esp_timer_create_args_t reconnect_args = {
        .callback = reconnect_timer_cb,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "wifi_reconnect",
    };
    const esp_timer_create_args_t fallback_args = {
        .callback = ap_fallback_timer_cb,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "wifi_ap_fallback",
    };
    const esp_timer_create_args_t teardown_args = {
        .callback = ap_teardown_timer_cb,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "wifi_ap_teardown",
    };

    ESP_ERROR_CHECK(esp_timer_create(&reconnect_args, &s_reconnect_timer));
    ESP_ERROR_CHECK(esp_timer_create(&fallback_args, &s_ap_fallback_timer));
    ESP_ERROR_CHECK(esp_timer_create(&teardown_args, &s_ap_teardown_timer));

    if (xTaskCreate(wifi_manager_task, "wifi_mgr", WIFI_TASK_STACK, NULL, WIFI_TASK_PRIORITY, &s_task) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "WiFi driver ready; provisioning SSID would be \"%s\"", s_ap_ssid);

    return ESP_OK;
}

esp_err_t wifi_manager_start_sta(void)
{
    if (s_queue == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    post_cmd(CMD_START_STA, 0);

    return ESP_OK;
}

esp_err_t wifi_manager_start_provisioning(void)
{
    if (s_queue == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    post_cmd(CMD_START_AP, 0);

    return ESP_OK;
}

esp_err_t wifi_manager_stop_ap(void)
{
    if (s_queue == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    post_cmd(CMD_STOP_AP, 0);

    return ESP_OK;
}

bool wifi_manager_is_connected(void)
{
    return s_sta_connected;
}

esp_err_t wifi_manager_wait_connected(uint32_t timeout_ms)
{
    EventBits_t bits;

    if (s_event_group == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    bits = xEventGroupWaitBits(s_event_group,
                               WIFI_CONNECTED_BIT,
                               pdFALSE,
                               pdFALSE,
                               timeout_ms == portMAX_DELAY ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms));

    return (bits & WIFI_CONNECTED_BIT) ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t wifi_manager_get_status(wifi_mgr_status_t *out)
{
    const pluto_config_t *cfg = config_store_peek();

    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(out, 0, sizeof(*out));

    if (s_sta_connected) {
        refresh_link_info();
    }

    out->state = s_state;
    out->sta_connected = s_sta_connected;
    out->ap_active = s_ap_active;
    out->rssi = s_sta_connected ? s_rssi : 0;
    out->channel = s_channel;
    out->retry_count = s_retry_count;
    out->next_retry_sec = s_sta_connected ? 0 : s_backoff_sec;
    out->ap_client_count = s_ap_client_count;
    out->last_disconnect_reason = s_last_reason;
    out->auth_suspected_bad = s_auth_suspected_bad;

    strlcpy(out->ssid, cfg->wifi_ssid, sizeof(out->ssid));
    strlcpy(out->ip, s_sta_ip, sizeof(out->ip));
    strlcpy(out->ap_ssid, s_ap_ssid, sizeof(out->ap_ssid));
    strlcpy(out->ap_ip, s_ap_active ? WIFI_AP_IP : "", sizeof(out->ap_ip));

    return ESP_OK;
}

esp_err_t wifi_manager_scan_start(void)
{
    if (s_queue == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_scan_in_progress || s_scan_pending) {
        return ESP_ERR_INVALID_STATE;
    }

    post_cmd(CMD_SCAN, 0);

    return ESP_OK;
}

bool wifi_manager_scan_in_progress(void)
{
    return s_scan_in_progress || s_scan_pending;
}

esp_err_t wifi_manager_scan_get_results(wifi_mgr_ap_t *out, size_t max, size_t *count, int64_t *age_ms)
{
    size_t n;

    if (out == NULL || count == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    n = MIN(max, s_scan_count);
    memcpy(out, s_scan_results, n * sizeof(wifi_mgr_ap_t));
    *count = n;

    if (age_ms != NULL) {
        *age_ms = s_scan_timestamp_us == 0 ? -1 : (esp_timer_get_time() - s_scan_timestamp_us) / 1000;
    }

    return ESP_OK;
}
