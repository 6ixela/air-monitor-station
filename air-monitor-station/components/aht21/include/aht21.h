#ifndef AHT21_H
#define AHT21_H

#include "driver/i2c_master.h"
#include "esp_log.h"

#define ATH_I2C_ADDRESS 0x38
#define AHT21_CAL 0xE1
#define AHT_READ 0x71
#define AHT_START_MES 0xAC

#define SDA_IO GPIO_NUM_8
#define SCL_IO GPIO_NUM_9
#define I2C_MASTER_NUM I2C_NUM_0   // I2C port number
#define I2C_MASTER_FREQ_HZ 100000  // I2C master clock frequency
#define I2C_MASTER_TIMEOUT_MS 1000 // Timeout for I2C operations
#define AHT21_TEMP_SCALE 200.0f
#define AHT21_HUM_SCALE 100.0f
#define AHT21_TEMP_OFFSET 50.0f
#define AHT21_RESOLUTION 1048576.0f

static const uint8_t AHT_MEASUREMENT_CMD[] = {0x33, 0x00};
static const uint8_t AHT_CALIBBRATION_CMD[] = {0x08, 0x00};

typedef enum aht21_stat_t
{
    AHT21_OK,
    AHT21_CAL_ERR,
    AHT21_READ_ERR,
} aht21_stat_t;

typedef struct aht21_data_t
{
    float temperature;
    uint32_t humidity;
    aht21_stat_t status;
} aht21_data_t;

void i2c_master_init(void);
void aht21_init(void);
esp_err_t aht21_write_data(uint8_t reg_addr, uint8_t *data, size_t len);
esp_err_t aht21_read_data(uint8_t reg_addr, uint8_t *data, size_t len);
esp_err_t aht21_send_cmd(uint8_t *data, size_t len);
aht21_stat_t aht_calibrate(void);
aht21_data_t aht21_get_temp_hum(void);

#endif
