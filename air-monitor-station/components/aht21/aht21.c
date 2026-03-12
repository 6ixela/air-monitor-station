#include <stdio.h>
#include "aht21.h"
#include "driver/i2c.h"
#include "esp_log.h"

static const char *TAG = "AHT21";

void i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));
}

aht21_stat_t aht21_calibrate(void)
{
    esp_err_t err = aht21_write_data(AHT21_CAL, (uint8_t *)AHT_CALIBBRATION_CMD, sizeof(AHT_CALIBBRATION_CMD));
    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "Sensor calibrated");
        return AHT21_OK;
    }
    else
    {
        ESP_LOGE(TAG, "Failed to calibrate sensor: %s", esp_err_to_name(err));
        return AHT21_CAL_ERR;
    }
}

aht21_stat_t aht_start_meas(void)
{
    esp_err_t err = aht21_write_data(AHT_START_MES, (uint8_t *)AHT_MEASUREMENT_CMD, sizeof(AHT_MEASUREMENT_CMD));
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start sensor measurement: %s", esp_err_to_name(err));
        return AHT21_READ_ERR;
    }
    return AHT21_OK;
}

aht21_data_t aht21_get_temp_hum(void)
{
    aht21_data_t result = {.temperature = 0, .humidity = 0, .status = AHT21_READ_ERR};
    uint8_t data[7];

    // Step 1: Read status to check calibration
    if (aht21_read_data(AHT_READ, data, sizeof(data)) != ESP_OK)
    {
        ESP_LOGE(TAG, "Sensor not responding. Check connection.");
        return result;
    }

    // Step 2: Check and perform calibration if needed
    if (!(data[0] & (1 << 3)))
    {
        ESP_LOGI(TAG, "Sensor not calibrated. Calibrating...");
        if (aht21_calibrate() != AHT21_OK)
        {
            ESP_LOGE(TAG, "Calibration failed");
            result.status = AHT21_CAL_ERR;
            return result;
        }
        vTaskDelay(100 / portTICK_PERIOD_MS); // Wait after calibration
    }

    vTaskDelay(10 / portTICK_PERIOD_MS); // Wait before starting measurement


    // Step 3: Start measurement
    if (aht_start_meas() != AHT21_OK)
    {
        ESP_LOGE(TAG, "Failed to start measurement");
        result.status = AHT21_READ_ERR;
        return result;
    }

    vTaskDelay(80 / portTICK_PERIOD_MS); // Wait for measurement to complete

    // Step 4: Read data after measurement
    if (aht21_read_data(AHT_READ, data, sizeof(data)) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read sensor data");
        result.status = AHT21_READ_ERR;
        return result;
    }

    if (data[0] & (1 << 7))
    {
        ESP_LOGE(TAG, "Sensor busy, measurement not ready");
        result.status = AHT21_READ_ERR;
        return result;
    }

    // Step 5: Parse the sensor data
    uint32_t raw_humidity = ((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) | (data[3] >> 4);
    uint32_t raw_temperature = ((uint32_t)(data[3] & 0x0F) << 16) | ((uint32_t)data[4] << 8) | data[5];
    result.temperature = (float)raw_temperature * AHT21_TEMP_SCALE / AHT21_RESOLUTION - AHT21_TEMP_OFFSET;
    result.humidity = (float)raw_humidity * AHT21_HUM_SCALE / AHT21_RESOLUTION;
    result.status = AHT21_OK;
    ESP_LOGI(TAG, "Temperature: %f C, Humidity: %lu %%", result.temperature, result.humidity);

    return result;
}

esp_err_t aht21_write_data(uint8_t reg_addr, uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ATH_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true); // Write register address
    i2c_master_write(cmd, data, len, true);     // Write data buffer
    i2c_master_stop(cmd);
    esp_err_t result = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return result;
}

esp_err_t aht21_send_cmd(uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ATH_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, data, len, true);
    i2c_master_stop(cmd);
    esp_err_t result = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return result;
}

esp_err_t aht21_read_data(uint8_t reg_addr, uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ATH_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    // i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ATH_I2C_ADDRESS << 1) | I2C_MASTER_READ, true);
    if (len > 1)
    {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t result = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return result;
}
