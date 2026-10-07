#include <stdio.h>
#include <sys/time.h>

#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "config_store.h"
#include "log_ring.h"
#include "mqtt_manager.h"
#include "rotor.h"
#include "sntp_manager.h"
#include "web_server.h"
#include "wifi_manager.h"

#define ROTOR_TICK_MS 50
#define STATE_PERIOD_TRACKING_MS 1000
#define STATE_PERIOD_IDLE_MS 10000

static const char *TAG = "pluto_main";

/* Shared between the MQTT task, which hands in payloads, and the rotor task,
 * which advances the trajectory. */
static rotor_t s_rotor;
static SemaphoreHandle_t s_rotor_lock;

/* The mode in the last state published, so a change of mode (the end of a
 * pass, for one) is reported right away and only once. */
static rotor_mode_t s_reported_mode = ROTOR_MODE_IDLE;

static int64_t now_ms(void)
{
    struct timeval tv;

    gettimeofday(&tv, NULL);
    return (int64_t)tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

static void publish_state(void)
{
    char json[ROTOR_STATE_JSON_MAX];
    int len;

    xSemaphoreTake(s_rotor_lock, portMAX_DELAY);
    len = rotor_state_json(&s_rotor, now_ms(), sntp_manager_is_synced(), json, sizeof(json));
    s_reported_mode = s_rotor.mode;
    xSemaphoreGive(s_rotor_lock);

    if (len < 0) {
        ESP_LOGE(TAG, "State does not fit in %d bytes", (int)sizeof(json));
        return;
    }
    /* Nothing to do while disconnected: the next state after reconnecting is current anyway. */
    mqtt_manager_publish_state(json, len);
}

/*
 * LOCAL DEBUG AID - not meant to ship.
 *
 * Centidegrees as signed degrees with two decimals. Written by hand rather than
 * with %f because splitting the value arithmetically loses the sign for
 * anything between -1 and 0 degrees, and because CONFIG_LOG_MAXIMUM_LEVEL only
 * reaches INFO, so this has to survive on integer formatting.
 */
static const char *cdeg_str(int32_t cdeg, char *buf, size_t len)
{
    /* Widened so negating INT32_MIN cannot overflow, even though the DTO bounds
     * the real values to +-35999. */
    int64_t magnitude = cdeg < 0 ? -(int64_t)cdeg : cdeg;

    snprintf(buf, len, "%s%lld.%02lld",
             cdeg < 0 ? "-" : "",
             (long long)(magnitude / 100),
             (long long)(magnitude % 100));

    return buf;
}

/*
 * LOCAL DEBUG AID - not meant to ship.
 *
 * The accept line says nothing about where the antenna was told to go, which is
 * the whole point of looking at the monitor. The points printed are the ones
 * kept after the expiry rule, so a count below the payload's own is the
 * tolerance at work rather than a decode problem.
 */
static void log_batch_points(int payload_len,
                             const rotor_batch_report_t *report,
                             const trajectory_t *loaded,
                             int64_t received_ms)
{
    int on_wire = (payload_len - PLUTO_DTO_HEADER_LEN) / PLUTO_DTO_POINT_LEN;
    /* Sized for the full int32 range, not just the DTO's, so the formatting
     * above cannot be truncated. */
    char az[16];
    char el[16];

    if (report->latency_valid) {
        ESP_LOGI(TAG, "  %d points on the wire, %u kept, latency %lld ms",
                 on_wire, (unsigned)loaded->count, (long long)report->latency_ms);
    } else {
        ESP_LOGI(TAG, "  %d points on the wire, %u kept, latency unknown (no synced clock)",
                 on_wire, (unsigned)loaded->count);
    }

    for (size_t i = 0; i < loaded->count; i++) {
        const trajectory_point_t *point = &loaded->points[i];

        ESP_LOGI(TAG, "  [%u] t%+lld ms  az=%s el=%s",
                 (unsigned)i,
                 (long long)(point->t_ms - received_ms),
                 cdeg_str(point->az_cdeg, az, sizeof(az)),
                 cdeg_str(point->el_cdeg, el, sizeof(el)));
    }
}

static void coordinates_message_handler(const char *topic,
                                        int topic_len,
                                        const char *payload,
                                        int payload_len)
{
    bool accepted;
    const char *error;
    rotor_mode_t mode;
    rotor_batch_report_t report;
    trajectory_t loaded;
    int64_t received_ms;

    (void)topic;
    (void)topic_len;

    /* One reading of the clock for both the rotor and the debug dump, so the
     * offsets printed are relative to the instant the batch was actually
     * handled. */
    received_ms = now_ms();

    xSemaphoreTake(s_rotor_lock, portMAX_DELAY);
    accepted = rotor_handle_payload(&s_rotor,
                                    (const uint8_t *)payload,
                                    (size_t)payload_len,
                                    received_ms,
                                    sntp_manager_is_synced());
    error = s_rotor.last_batch.error;
    mode = s_rotor.mode;
    /* Copied out so the dump below prints without holding the rotor task off. */
    report = s_rotor.last_batch;
    loaded = s_rotor.trajectory;
    xSemaphoreGive(s_rotor_lock);

    if (accepted) {
        ESP_LOGI(TAG, "Batch accepted (%d bytes), mode %s", payload_len, rotor_mode_name(mode));
        log_batch_points(payload_len, &report, &loaded, received_ms);
    } else {
        ESP_LOGW(TAG, "Batch rejected: %s", error);
    }

    /* The server hears about every batch right away instead of on the next
     * period (device-state.md §3). */
    publish_state();
}

static void rotor_task(void *arg)
{
    TickType_t last_state = xTaskGetTickCount();
    rotor_mode_t last_mode = ROTOR_MODE_IDLE;

    (void)arg;

    for (;;) {
        pan_tilt_pose_t pose;
        rotor_mode_t mode;
        bool moved;
        bool mode_unreported;
        TickType_t period;

        xSemaphoreTake(s_rotor_lock, portMAX_DELAY);
        moved = rotor_tick(&s_rotor, now_ms());
        pose = s_rotor.pose;
        mode = s_rotor.mode;
        mode_unreported = mode != s_reported_mode;
        xSemaphoreGive(s_rotor_lock);

        if (moved) {
            /* The servo driver (issue #4) takes the angles from here. */
            ESP_LOGD(TAG, "pan=%ld tilt=%ld (%s)", (long)pose.pan_cdeg, (long)pose.tilt_cdeg,
                     pose.mode == PAN_TILT_FLIPPED ? "flipped" : "normal");
        }
        if (mode != last_mode) {
            ESP_LOGI(TAG, "Rotor mode: %s", rotor_mode_name(mode));
            last_mode = mode;
        }

        period = pdMS_TO_TICKS(mode == ROTOR_MODE_TRACKING ? STATE_PERIOD_TRACKING_MS : STATE_PERIOD_IDLE_MS);
        if (mode_unreported || xTaskGetTickCount() - last_state >= period) {
            publish_state();
            last_state = xTaskGetTickCount();
        }

        vTaskDelay(pdMS_TO_TICKS(ROTOR_TICK_MS));
    }
}

static void start_rotor(void)
{
    /* Mount facing north with both servos on their full range until the
     * calibration (az_ref and limits) is stored in config_store. */
    const pan_tilt_config_t mount = PAN_TILT_CONFIG_DEFAULT;

    rotor_init(&s_rotor, &mount, CONFIG_PLUTO_EXPIRED_POINT_TOLERANCE_MS);
    s_rotor_lock = xSemaphoreCreateMutex();
    configASSERT(s_rotor_lock != NULL);
    xTaskCreate(rotor_task, "rotor", 4096, NULL, 5, NULL);
}

/*
 * The captive DNS responder can only bind once the access point interface has
 * an address, so it follows the AP's own lifecycle rather than being started
 * alongside it. Living here keeps wifi_manager free of any dependency on
 * web_server, which would otherwise be a circular one.
 */
static void ap_lifecycle_handler(void *arg,
                                 esp_event_base_t event_base,
                                 int32_t event_id,
                                 void *event_data)
{
    if (event_id == WIFI_EVENT_AP_START) {
        web_server_set_captive(true);
    } else if (event_id == WIFI_EVENT_AP_STOP) {
        web_server_set_captive(false);
    }
}

static esp_err_t init_nvs(void)
{
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    return ret;
}

void app_main(void)
{
    char device_id[WIFI_DEVICE_ID_LEN];

    /* First, so the /logs page sees the boot from configuration onwards. */
    log_ring_install();

    ESP_ERROR_CHECK(init_nvs());
    ESP_ERROR_CHECK(config_store_init());
    ESP_ERROR_CHECK(wifi_manager_init());
    ESP_ERROR_CHECK(wifi_manager_get_device_id(device_id, sizeof(device_id)));

    ESP_LOGI(TAG, "Using device_id: %s", device_id);

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &ap_lifecycle_handler,
                                                        NULL,
                                                        NULL));

    /* One server for both roles: the captive portal while the access point is
     * up, the dashboard on the LAN address once the station is connected. */
    ESP_ERROR_CHECK(web_server_start());

    if (!config_store_is_provisioned()) {
        ESP_LOGW(TAG, "Not provisioned; starting the setup portal");
        ESP_ERROR_CHECK(wifi_manager_start_provisioning());
        /* Nothing to wait for: configuration arrives over HTTP and the device
         * restarts into the station path. */
        return;
    }

    ESP_ERROR_CHECK(wifi_manager_start_sta());

    /* Only this task blocks. The WiFi manager, the HTTP server and the event
     * loop all run on their own tasks, so the dashboard stays reachable and
     * reconnection keeps going for as long as this takes. */
    ESP_ERROR_CHECK(wifi_manager_wait_connected(portMAX_DELAY));

    /* A missing clock is not worth a reboot loop over: the poller keeps trying
     * in the background. */
    if (sntp_manager_init() != ESP_OK) {
        ESP_LOGW(TAG, "Continuing without a synchronized clock");
    }

    start_rotor();
    ESP_ERROR_CHECK(mqtt_manager_init(coordinates_message_handler, device_id));
}
