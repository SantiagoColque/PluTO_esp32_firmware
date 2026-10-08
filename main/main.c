#include <stdio.h>
#include <sys/time.h>

#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "as5600.h"
#include "config_store.h"
#include "encoder_angle.h"
#include "log_ring.h"
#include "mqtt_manager.h"
#include "rotor.h"
#include "servo_driver.h"
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

#if CONFIG_PLUTO_ENCODERS
/* The AS5600's address is fixed, so each encoder gets a bus of its own, on
 * pins clear of the servos and of the strapping and flash pins. */
#define ENCODER_PAN_SDA_GPIO 21
#define ENCODER_PAN_SCL_GPIO 22
#define ENCODER_TILT_SDA_GPIO 32
#define ENCODER_TILT_SCL_GPIO 33

/* Used only by the rotor task, after start_rotor() sets them up. */
static as5600_t s_pan_encoder;
static as5600_t s_tilt_encoder;
static encoder_calibration_t s_pan_encoder_cal;
static encoder_calibration_t s_tilt_encoder_cal;
#endif

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

/* The servo angles the encoders read. False without encoders, or when either
 * one has no reading: the state then reports none. */
static bool read_encoders(int32_t *pan_cdeg, int32_t *tilt_cdeg)
{
#if CONFIG_PLUTO_ENCODERS
    uint16_t pan_raw;
    uint16_t tilt_raw;
    /* Both every tick, so the log follows each one's problems. */
    bool pan_ok = as5600_read(&s_pan_encoder, &pan_raw) == ESP_OK;
    bool tilt_ok = as5600_read(&s_tilt_encoder, &tilt_raw) == ESP_OK;

    if (!pan_ok || !tilt_ok) {
        return false;
    }
    *pan_cdeg = encoder_angle_cdeg(&s_pan_encoder_cal, pan_raw);
    *tilt_cdeg = encoder_angle_cdeg(&s_tilt_encoder_cal, tilt_raw);
    return true;
#else
    (void)pan_cdeg;
    (void)tilt_cdeg;
    return false;
#endif
}

static void rotor_task(void *arg)
{
    TickType_t last_state = xTaskGetTickCount();
    rotor_mode_t last_mode = ROTOR_MODE_IDLE;
    bool last_resting = true;
    /* The servos move in monotonic time: SNTP steps the wall clock, and a
     * jump there must not become a jump of the antenna. */
    int64_t last_tick_us = esp_timer_get_time();

    (void)arg;

    for (;;) {
        pan_tilt_pose_t pose;
        bool has_pose;
        rotor_mode_t mode;
        bool resting;
        bool mode_unreported;
        TickType_t period;
        int32_t measured_pan = 0;
        int32_t measured_tilt = 0;
        bool measured;
        int32_t dt_ms = (int32_t)((esp_timer_get_time() - last_tick_us) / 1000);

        /* Advanced by whole milliseconds so the remainder carries over to the
         * next tick instead of making the profile run slow. */
        last_tick_us += (int64_t)dt_ms * 1000;

        /* Before taking the lock: the I2C reads would hold off the MQTT handler. */
        measured = read_encoders(&measured_pan, &measured_tilt);

        xSemaphoreTake(s_rotor_lock, portMAX_DELAY);
        rotor_set_measured(&s_rotor, measured, measured_pan, measured_tilt);
        rotor_tick(&s_rotor, now_ms(), dt_ms);
        has_pose = s_rotor.has_pose;
        pose = s_rotor.pose;
        mode = s_rotor.mode;
        resting = rotor_outputs_resting(&s_rotor);
        mode_unreported = mode != s_reported_mode;
        xSemaphoreGive(s_rotor_lock);

        /* Outside the lock, every tick: the driver skips a duty it already has
         * and resumes released outputs on the next set. Its only error is "not
         * started", which start_rotor() has already logged. */
        if (resting) {
            servo_driver_release();
        } else if (has_pose) {
            servo_driver_set(pose.pan_cdeg, pose.tilt_cdeg);
        }

        if (mode != last_mode) {
            ESP_LOGI(TAG, "Rotor mode: %s", rotor_mode_name(mode));
            last_mode = mode;
        }
        if (resting != last_resting) {
            /* Logged even with the driver down, to follow HU-35 without servos. */
            ESP_LOGI(TAG, "Servo outputs %s", resting ? "at rest" : "active");
            last_resting = resting;
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
    pluto_calibration_t cal;
    pan_tilt_config_t mount;
    servo_driver_config_t servos;
    /* Kconfig is in degrees for whoever tunes it, the profile in centidegrees. */
    const motion_profile_config_t motion = {
        .v_max_cdeg_s = CONFIG_PLUTO_MOTION_MAX_SPEED_DEG_S * 100,
        .a_max_cdeg_s2 = CONFIG_PLUTO_MOTION_MAX_ACCEL_DEG_S2 * 100,
        .rest_after_ms = CONFIG_PLUTO_MOTION_REST_AFTER_MS,
    };

    /* Cannot fail once config_store_init() has succeeded. */
    ESP_ERROR_CHECK(config_store_get_calibration(&cal));

    mount = (pan_tilt_config_t){
        .az_ref_cdeg = cal.az_ref_cdeg,
        .pan_min_cdeg = cal.pan_min_cdeg,
        .pan_max_cdeg = cal.pan_max_cdeg,
        .tilt_min_cdeg = cal.tilt_min_cdeg,
        .tilt_max_cdeg = cal.tilt_max_cdeg,
    };
    /* The pins follow the wiring, so they stay in Kconfig; the pulses are part
     * of the stored calibration. */
    servos = (servo_driver_config_t){
        .pan = {.gpio_num = CONFIG_PLUTO_SERVO_PAN_GPIO, .calibration = {cal.pan_us_at_0, cal.pan_us_at_180}},
        .tilt = {.gpio_num = CONFIG_PLUTO_SERVO_TILT_GPIO, .calibration = {cal.tilt_us_at_0, cal.tilt_us_at_180}},
    };

    ESP_LOGI(TAG, "Mount: az_ref=%ld pan=%ld..%ld tilt=%ld..%ld cdeg",
             (long)mount.az_ref_cdeg,
             (long)mount.pan_min_cdeg,
             (long)mount.pan_max_cdeg,
             (long)mount.tilt_min_cdeg,
             (long)mount.tilt_max_cdeg);

    /* Any fallback for a bad stored calibration is a guess that could drive
     * the mount past limits someone set, so the servos stay still instead.
     * Not worth a reboot loop either: the board keeps reporting, with the
     * dashboard and /logs reachable to say why. */
    if (!pan_tilt_config_valid(&mount)) {
        ESP_LOGE(TAG, "Stored mount calibration is invalid; servos not started");
        mount = PAN_TILT_CONFIG_DEFAULT; /* only so the rotor keeps reporting */
    } else if (servo_driver_init(&servos) != ESP_OK) { /* refuses bad pulses or pins */
        ESP_LOGE(TAG, "Servos not started; tracking without moving the mount");
    }

#if CONFIG_PLUTO_ENCODERS
    s_pan_encoder_cal = (encoder_calibration_t){cal.pan_enc_zero, cal.pan_enc_dir};
    s_tilt_encoder_cal = (encoder_calibration_t){cal.tilt_enc_zero, cal.tilt_enc_dir};
    /* A failed init is logged and leaves the state without a measurement; the
     * servos do not depend on the encoders. */
    as5600_init(&s_pan_encoder, "pan", I2C_NUM_0,
                ENCODER_PAN_SDA_GPIO, ENCODER_PAN_SCL_GPIO);
    as5600_init(&s_tilt_encoder, "tilt", I2C_NUM_1,
                ENCODER_TILT_SDA_GPIO, ENCODER_TILT_SCL_GPIO);
#endif

    rotor_init(&s_rotor, &mount, CONFIG_PLUTO_EXPIRED_POINT_TOLERANCE_MS, &motion);
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
