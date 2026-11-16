#include "as5600.h"

#include "esp_check.h"
#include "esp_log.h"

#define AS5600_ADDR 0x36
#define REG_STATUS 0x0B
#define REG_RAW_ANGLE 0x0C /* high byte; the low one follows at 0x0D */

#define STATUS_MH (1 << 3)
#define STATUS_ML (1 << 4)
#define STATUS_MD (1 << 5)

/* Slow on purpose: the wires run through the joints, next to the servos. */
#define BUS_SPEED_HZ 100000
/* Counted in 10 ms FreeRTOS ticks, where a single tick could expire at once. */
#define XFER_TIMEOUT_MS 20

static const char *TAG = "encoder";

esp_err_t as5600_init(as5600_t *encoder, const char *name, int port, int sda_gpio, int scl_gpio)
{
    const i2c_master_bus_config_t bus_config = {
        .i2c_port = port,
        .sda_io_num = sda_gpio,
        .scl_io_num = scl_gpio,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        /* Too weak for long wires, but they hold an empty bus high, so a missing
         * encoder NACKs fast instead of timing out. */
        .flags.enable_internal_pullup = true,
    };
    const i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AS5600_ADDR,
        .scl_speed_hz = BUS_SPEED_HZ,
    };
    i2c_master_bus_handle_t bus;

    encoder->name = name;
    encoder->dev = NULL;
    encoder->issue = "not read yet"; /* so the first good read gets logged */

    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_config, &bus), TAG, "%s encoder: I2C%d", name, port);
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus, &dev_config, &encoder->dev), TAG, "%s encoder", name);
    return ESP_OK;
}

static esp_err_t read_register(const as5600_t *encoder, uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(encoder->dev, &reg, 1, data, len, XFER_TIMEOUT_MS);
}

esp_err_t as5600_read(as5600_t *encoder, uint16_t *raw)
{
    uint8_t status = 0;
    uint8_t angle[2] = {0};
    const char *issue = NULL;
    esp_err_t err;

    if (encoder->dev == NULL) {
        return ESP_ERR_INVALID_STATE; /* as5600_init() logged why */
    }

    /* Two reads, not a burst: the datasheet treats the pointer specially at RAW ANGLE. */
    err = read_register(encoder, REG_STATUS, &status, 1);
    if (err == ESP_OK) {
        err = read_register(encoder, REG_RAW_ANGLE, angle, sizeof(angle));
    }

    if (err != ESP_OK) {
        /* i2c_master reports a NACK, a missing chip, as ESP_ERR_INVALID_STATE. */
        issue = "no answer on I2C";
    } else if (!(status & STATUS_MD)) {
        issue = "no magnet";
        err = ESP_ERR_NOT_FOUND;
    } else if (status & STATUS_ML) {
        issue = "magnet too weak";
    } else if (status & STATUS_MH) {
        issue = "magnet too strong";
    }
    *raw = (uint16_t)(((angle[0] & 0x0F) << 8) | angle[1]);

    if (issue != encoder->issue) {
        if (issue != NULL) {
            ESP_LOGW(TAG, "%s encoder: %s", encoder->name, issue);
        } else {
            ESP_LOGI(TAG, "%s encoder reading, raw %u", encoder->name, (unsigned)*raw);
        }
        encoder->issue = issue;
    }
    return err;
}
