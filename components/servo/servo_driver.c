#include "servo_driver.h"

#include <stdbool.h>

#include "driver/ledc.h"
#include "esp_check.h"
#include "esp_log.h"

/* High-speed group: latches a new duty without the low-speed update step.
 * Only the plain ESP32 has it. */
#define SERVO_SPEED_MODE LEDC_HIGH_SPEED_MODE
#define SERVO_TIMER LEDC_TIMER_0

_Static_assert(LEDC_TIMER_16_BIT == SERVO_DUTY_RESOLUTION_BITS,
               "servo_pulse_duty() computes the duty for the resolution the timer is set to");

static const char *TAG = "servo";

typedef struct {
    ledc_channel_t channel;
    servo_calibration_t calibration;
    uint32_t duty; /* last written; 0 means no pulses */
} servo_axis_t;

static servo_axis_t s_pan = {.channel = LEDC_CHANNEL_0};
static servo_axis_t s_tilt = {.channel = LEDC_CHANNEL_1};
static bool s_started;

/* 6-11 run the SPI flash. 0, 2, 5, 12 and 15 are strapping pins the servo line
 * could flip at reset (12 sets the flash voltage). 1 and 3 are the console, 14
 * puts out PWM during boot and would jerk the servo, 34-39 are input only. */
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
    return ledc_channel_config(&channel);
}

/* Duty latches at the end of the frame, so release writes 0 instead of calling
 * ledc_stop(), which can cut a pulse short. Errors only come from bad args. */
static void write_duty(servo_axis_t *axis, uint32_t duty)
{
    if (duty != axis->duty) {
        ledc_set_duty(SERVO_SPEED_MODE, axis->channel, duty);
        ledc_update_duty(SERVO_SPEED_MODE, axis->channel);
        axis->duty = duty;
    }
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

    /* Check everything before touching LEDC, so a bad config never emits a pulse. */
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

    s_started = true;
    ESP_LOGI(TAG, "Pan on GPIO%d (%ld..%ld us), tilt on GPIO%d (%ld..%ld us); idle until the first pose",
             config->pan.gpio_num, (long)config->pan.calibration.us_at_0,
             (long)config->pan.calibration.us_at_180, config->tilt.gpio_num,
             (long)config->tilt.calibration.us_at_0, (long)config->tilt.calibration.us_at_180);
    return ESP_OK;
}

esp_err_t servo_driver_set(int32_t pan_cdeg, int32_t tilt_cdeg)
{
    if (!s_started) {
        return ESP_ERR_INVALID_STATE;
    }
    write_duty(&s_pan, servo_pulse_duty(servo_pulse_us(&s_pan.calibration, pan_cdeg)));
    write_duty(&s_tilt, servo_pulse_duty(servo_pulse_us(&s_tilt.calibration, tilt_cdeg)));
    return ESP_OK;
}

esp_err_t servo_driver_release(void)
{
    if (!s_started) {
        return ESP_ERR_INVALID_STATE;
    }
    write_duty(&s_pan, 0);
    write_duty(&s_tilt, 0);
    return ESP_OK;
}
