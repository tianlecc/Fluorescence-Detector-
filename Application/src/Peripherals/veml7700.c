/**************************************************************************/ /**
 * @file      veml7700.c
 * @brief     VEML7700 Ambient Light Sensor Driver
 *
 * @details   This file implements functions to initialize the VEML7700 sensor,
 *            configure gain and integration time, read ambient light levels,
 *            and manage sensor operation with FreeRTOS.
 ******************************************************************************/

/******************************************************************************
 * Includes
 ******************************************************************************/
#include "veml7700.h"
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "WifiHandlerThread/WifiHandler.h"
#include "esp_i2c.h"

/******************************************************************************
 * Internal Variables
 ******************************************************************************/
static uint16_t g_veml7700_config = 0;          ///< Current VEML7700 configuration register value
static uint8_t  g_current_gain = VEML7700_GAIN_1; ///< Current sensor gain
static uint8_t  g_current_it   = VEML7700_IT_100MS; ///< Current integration time setting

/******************************************************************************
 * Internal Function Prototypes
 ******************************************************************************/
static void veml7700_readWait(void);
static int _veml7700_write_config_reg(uint16_t config_value);

/******************************************************************************
 * Internal Functions
 ******************************************************************************/
/**
 * @brief Waits for sensor conversion to complete.
 *
 * This function delays for at least 2× the integration time to ensure
 * the VEML7700 has completed a measurement.
 */
static void veml7700_readWait(void)
{
    uint32_t integration_time_ms;
    switch (g_current_it) {
        case VEML7700_IT_25MS:  integration_time_ms = 25;  break;
        case VEML7700_IT_50MS:  integration_time_ms = 50;  break;
        case VEML7700_IT_100MS: integration_time_ms = 100; break;
        case VEML7700_IT_200MS: integration_time_ms = 200; break;
        case VEML7700_IT_400MS: integration_time_ms = 400; break;
        case VEML7700_IT_800MS: integration_time_ms = 800; break;
        default: integration_time_ms = 100; break;
    }

    vTaskDelay(pdMS_TO_TICKS(2 * integration_time_ms));
}

/**
 * @brief Writes a 16-bit configuration value to the VEML7700 configuration register.
 *
 * @param[in] config_value Configuration register value to write.
 * @return VEML7700_OK on success, VEML7700_ERROR on failure.
 */
static int _veml7700_write_config_reg(uint16_t config_value)
{
    uint8_t buf[3];
    buf[0] = VEML7700_REG_ALS_CONFIG;
    buf[1] = config_value & 0xFF;          ///< Low byte
    buf[2] = (config_value >> 8) & 0xFF;   ///< High byte

    I2C_Data data;
    data.address = VEML7700_I2C_ADDR;
    data.msgOut = buf;
    data.lenOut = 3;
    data.msgIn = NULL;
    data.lenIn = 0;

    if (I2cWriteDataWait(&data, WAIT_I2C_LINE_MS) != 0) {
        return VEML7700_ERROR;
    }
    return VEML7700_OK;
}

/******************************************************************************
 * Public API Functions
 ******************************************************************************/

/**
 * @brief Initializes the VEML7700 sensor with specified gain and integration time.
 *
 * @param[in] als_gain Gain setting (e.g., VEML7700_GAIN_1).
 * @param[in] als_integration_time Integration time setting (e.g., VEML7700_IT_100MS).
 * @return VEML7700_OK on success, VEML7700_ERROR on failure.
 */
int veml7700_init(uint8_t als_gain, uint8_t als_integration_time)
{
    uint16_t config = 0x0000;
    config |= ((als_gain & 0x03) << 11);
    config |= ((als_integration_time & 0x0F) << 6);
    config |= (VEML7700_PERS_1 << 4);
    config |= VEML7700_INT_DISABLE;
    config |= VEML7700_POWER_ON;

    g_veml7700_config = config;
    g_current_gain = als_gain & 0x03;
    g_current_it   = als_integration_time & 0x0F;

    if (_veml7700_write_config_reg(g_veml7700_config) != VEML7700_OK) {
        return VEML7700_ERROR;
    }

    // Disable power saving
    uint8_t psm_buf[3] = { VEML7700_REG_POWER_SAVE, 0x00, 0x00 };
    I2C_Data psm_data;
    psm_data.address = VEML7700_I2C_ADDR;
    psm_data.msgOut = psm_buf;
    psm_data.lenOut = 3;
    psm_data.msgIn = NULL;
    psm_data.lenIn = 0;
    I2cWriteDataWait(&psm_data, WAIT_I2C_LINE_MS);

    return VEML7700_OK;
}

/**
 * @brief Reads the ambient light level in lux.
 *
 * @return Light intensity in lux, or -1.0f on error.
 */
float veml7700_read_lux(void)
{
    int32_t err;
    uint8_t cmd = VEML7700_REG_ALS_DATA;
    uint8_t read_buf[2] = {0};
    uint16_t als_raw;

    err = I2cWriteReadNoStopBlocking_VEML(
            VEML7700_I2C_ADDR,
            &cmd,
            1,
            read_buf,
            2
          );
    if (err != ERROR_NONE) {
        return -1.0f;
    }

    als_raw = (uint16_t)(read_buf[0] | (read_buf[1] << 8));

    float integration_ms;
    switch (g_current_it) {
        case VEML7700_IT_25MS:  integration_ms = 25.0f;  break;
        case VEML7700_IT_50MS:  integration_ms = 50.0f;  break;
        case VEML7700_IT_100MS: integration_ms = 100.0f; break;
        case VEML7700_IT_200MS: integration_ms = 200.0f; break;
        case VEML7700_IT_400MS: integration_ms = 400.0f; break;
        case VEML7700_IT_800MS: integration_ms = 800.0f; break;
        default: integration_ms = 100.0f; break;
    }

    float gain_factor;
    switch (g_current_gain) {
        case VEML7700_GAIN_1:   gain_factor = 1.0f;   break;
        case VEML7700_GAIN_2:   gain_factor = 2.0f;   break;
        case VEML7700_GAIN_1_4: gain_factor = 0.25f;  break;
        case VEML7700_GAIN_1_8: gain_factor = 0.125f; break;
        default: gain_factor = 1.0f; break;
    }

    float resolution = 0.0576f * (100.0f / integration_ms) / gain_factor;
    float lux = als_raw * resolution;
    return lux;
}

/**
 * @brief Sets the sensor gain.
 *
 * @param[in] gain_value Gain value (e.g., VEML7700_GAIN_2).
 * @return VEML7700_OK on success, VEML7700_ERROR on failure.
 */
int veml7700_set_gain(uint8_t gain_value)
{
    g_veml7700_config &= ~((uint16_t)0x3 << 11);
    g_veml7700_config |= ((gain_value & 0x03) << 11);
    g_current_gain = gain_value & 0x03;
    return _veml7700_write_config_reg(g_veml7700_config);
}

/**
 * @brief Sets the sensor integration time.
 *
 * @param[in] it_value Integration time setting (e.g., VEML7700_IT_200MS).
 * @return VEML7700_OK on success, VEML7700_ERROR on failure.
 */
int veml7700_set_integration_time(uint8_t it_value)
{
    g_veml7700_config &= ~((uint16_t)0xF << 6);
    g_veml7700_config |= ((it_value & 0x0F) << 6);
    g_current_it = it_value & 0x0F;
    return _veml7700_write_config_reg(g_veml7700_config);
}

/**
 * @brief Directly sets the full configuration register.
 *
 * @param[in] config_value 16-bit configuration value.
 * @return VEML7700_OK on success, VEML7700_ERROR on failure.
 */
int veml7700_set_config(uint16_t config_value)
{
    config_value &= ~((uint16_t)0xE000);
    config_value &= ~((uint16_t)0x0400);
    config_value &= ~((uint16_t)0x000C);

    g_veml7700_config = config_value;
    g_current_gain = (config_value >> 11) & 0x03;
    g_current_it   = (config_value >> 6) & 0x0F;
    return _veml7700_write_config_reg(g_veml7700_config);
}

/**
 * @brief Reads the raw ALS value from the VEML7700 sensor.
 *
 * @param[out] als_raw Pointer to store the raw ALS reading.
 * @return VEML7700_OK on success, VEML7700_ERROR on failure.
 */
int veml7700_read_raw(uint16_t *als_raw)
{
    if (als_raw == NULL) {
        return VEML7700_ERROR;
    }

    uint8_t cmd = VEML7700_REG_ALS_DATA;
    uint8_t data_buf[2] = {0};

    int32_t err = I2cWriteReadNoStopBlocking_VEML(
        VEML7700_I2C_ADDR,
        &cmd,
        1,
        data_buf,
        2
    );
    if (err != ERROR_NONE) {
        return VEML7700_ERROR;
    }

    *als_raw = (uint16_t)(data_buf[0] | (data_buf[1] << 8));
    return VEML7700_OK;
}

/***************************************************************************//**
 * @brief  FreeRTOS task – read VEML7700 *on request* and enqueue the result.
 *
 * The task sleeps until @ref g_requestIntensity is set by the “detect” MQTT
 * message.  It then waits ~1 s, reads the sensor, scales the result to
 * **lux × 100**, sends it to @ref WifiAddIntensityToQueue() and clears the
 * request flag.
 ******************************************************************************/
void vVemlTask(void *pvParameters)
{
    (void)pvParameters;

    if (veml7700_init(VEML7700_GAIN_1, VEML7700_IT_100MS) != VEML7700_OK)
        vTaskSuspend(NULL);               /* fatal – stop task */

    while (1)
    {
        if (g_requestIntensity)
        {
			esp_i2c_set_color_flag(ESP_COLOR_BLUE);
			
            vTaskDelay(pdMS_TO_TICKS(1000));          /* settling time */

            float lux = veml7700_read_lux();          /* read sensor   */
            if (lux >= 0.0f)
            {
				uint16_t lux_int = (uint16_t)(lux + 0.5f);
                WifiAddIntensityToQueue(lux_int);
            }
			
			esp_i2c_set_color_flag(ESP_COLOR_OFF);
            g_requestIntensity = false;               /* reset flag    */
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

