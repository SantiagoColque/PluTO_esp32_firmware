#include "servo_driver.h"

#include <stdbool.h>

#include "driver/ledc.h"
#include "esp_check.h"
#include "esp_log.h"

/* The ESP32's high-speed group: its channels latch a new duty without the
 * extra update step the low-speed ones need, and the low-speed group stays
 * free for anything else. It only exists on the plain ESP32, the target. */
#define SERVO_SPEED_MODE LEDC_HIGH_SPEED_MODE
#define SERVO_TIMER LEDC_TIMER_0

_Static_assert(LEDC_TIMER_16_BIT == SERVO_DUTY_RESOLUTION_BITS,
               "servo_pulse_duty() computes the duty for the resolution the timer is set to");

static const char *TAG = "servo";

typedef struct {
    ledc_channel_t channel;
    servo_calibration_t calibration;
    uint32_t duty;  /* last duty written, meaningful while active */
    bool active;    /* pulses going out; false from init and after a release */
} servo_axis_t;

static servo_axis_t s_pan = {.channel = LEDC_CHANNEL_0};
static servo_axis_t s_tilt = {.channel = LEDC_CHANNEL_1};
static bool s_started;

/*
 * Pins of the ESP32 a servo signal must not use. 6-11 run the SPI flash, and
 * driving one crashes the chip. 0, 2, 5, 12 and 15 are strapping pins sampled
 * at reset, where whatever the servo input does to the line can change the
 * boot mode, or with 12 the flash voltage. 1 and 3 are UART0, the console. 14
 * puts out a PWM of its own during boot, which would jerk the servo on every
 * reset. 34-39 are inputs only.
 */
static bool pin_is_safe(int gpio)
{
    if (gpio < 0 || gpio > 33) {
        return false;
    }
    if (gpio >= 6 && gpio <= 11) {
        return false;
    }
    switch (gpio) {
    case 0:
    case 1:
    case 2:
    case 3:
    case 5:
    case 12:
    case 14:
    case 15:
        return false;
    default:
        return true;
    }
}

static esp_err_t check_axis(const char *name, const servo_axis_config_t *axis)
{
    ESP_RETURN_ON_FALSE(pin_is_safe(axis->gpio_num), ESP_ERR_INVALID_ARG, TAG,
                        "%s servo: GPIO%d is not safe for a servo signal", name, axis->gpio_num);
    ESP_RETURN_ON_FALSE(servo_calibration_valid(&axis->calibration), ESP_ERR_INVALID_ARG, TAG,
                        "%s servo: calibration %ld..%ld us is outside %d..%d us or flat",
                        name, (long)axis->calibration.us_at_0, (long)axis->calibration.us_at_180,
                        SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US);
    return ESP_OK;
}

static esp_err_t start_channel(servo_axis_t *axis, const servo_axis_config_t *config)
{
    /* Duty 0 keeps the line low: no pulse at all until the first pose. */
    const ledc_channel_config_t channel = {
        .gpio_num = config->gpio_num,
        .speed_mode = SERVO_SPEED_MODE,
        .channel = axis->channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = SERVO_TIMER,
        .duty = 0,
        .hpoint = 0,
    };

    axis->calibration = config->calibration;
    axis->duty = 0;
    axis->active = false;
    return ledc_channel_config(&channel);
}

static esp_err_t stop_axis(servo_axis_t *axis)
{
    esp_err_t err = ledc_stop(SERVO_SPEED_MODE, axis->channel, 0);

    if (err == ESP_OK) {
        axis->active = false;
    }
    return err;
}

static esp_err_t drive_axis(servo_axis_t *axis, int32_t cdeg)
{
    uint32_t duty = servo_pulse_duty(servo_pulse_us(&axis->calibration, cdeg));
    esp_err_t err;

    if (axis->active && duty == axis->duty) {
        return ESP_OK;
    }

    err = ledc_set_duty(SERVO_SPEED_MODE, axis->channel, duty);
    if (err == ESP_OK) {
        /* Also what turns the output back on after ledc_stop(). */
        err = ledc_update_duty(SERVO_SPEED_MODE, axis->channel);
    }
    if (err == ESP_OK) {
        axis->duty = duty;
        axis->active = true;
    }
    return err;
}

esp_err_t servo_driver_init(const servo_driver_config_t *config)
{
    const ledc_timer_config_t timer = {
        .speed_mode = SERVO_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_16_BIT,
        .timer_num = SERVO_TIMER,
        .freq_hz = SERVO_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t err;

    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG, "no configuration");
    ESP_RETURN_ON_FALSE(!s_started, ESP_ERR_INVALID_STATE, TAG, "already started");
    /* Everything is checked before LEDC is touched, so a bad configuration
     * never puts out a single pulse. */
    err = check_axis("pan", &config->pan);
    if (err == ESP_OK) {
        err = check_axis("tilt", &config->tilt);
    }
    if (err != ESP_OK) {
        return err;
    }
    ESP_RETURN_ON_FALSE(config->pan.gpio_num != config->tilt.gpio_num, ESP_ERR_INVALID_ARG, TAG,
                        "pan and tilt share GPIO%d", config->pan.gpio_num);

    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer), TAG, "50 Hz timer");
    ESP_RETURN_ON_ERROR(start_channel(&s_pan, &config->pan), TAG, "pan channel");
    ESP_RETURN_ON_ERROR(start_channel(&s_tilt, &config->tilt), TAG, "tilt channel");

    /* The same state a release leaves, so "never driven" and "released"
     * are one case for servo_driver_set(). */
    ESP_RETURN_ON_ERROR(stop_axis(&s_pan), TAG, "pan channel");
    ESP_RETURN_ON_ERROR(stop_axis(&s_tilt), TAG, "tilt channel");

    s_started = true;
    ESP_LOGI(TAG, "Pan on GPIO%d (%ld..%ld us), tilt on GPIO%d (%ld..%ld us); idle until the first pose",
             config->pan.gpio_num, (long)config->pan.calibration.us_at_0,
             (long)config->pan.calibration.us_at_180, config->tilt.gpio_num,
             (long)config->tilt.calibration.us_at_0, (long)config->tilt.calibration.us_at_180);
    return ESP_OK;
}

esp_err_t servo_driver_set(int32_t pan_cdeg, int32_t tilt_cdeg)
{
    esp_err_t err;

    if (!s_started) {
        return ESP_ERR_INVALID_STATE;
    }

    /* No logging here: this runs on every pose change, and the caller
     * decides how loudly to report a failure. */
    err = drive_axis(&s_pan, pan_cdeg);
    if (err == ESP_OK) {
        err = drive_axis(&s_tilt, tilt_cdeg);
    }
    return err;
}

esp_err_t servo_driver_release(void)
{
    esp_err_t pan_err;
    esp_err_t tilt_err;

    if (!s_started) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Both, even if the first fails: a servo left pulsing is the worse outcome. */
    pan_err = stop_axis(&s_pan);
    tilt_err = stop_axis(&s_tilt);
    return pan_err != ESP_OK ? pan_err : tilt_err;
}
