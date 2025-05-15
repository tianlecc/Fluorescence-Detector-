/**************************************************************************/ /**
 * @file      veml7700.h
 * @brief     VEML7700 Ambient Light Sensor Driver Interface
 *
 * @details
 * This file defines constants, macros, and function prototypes for operating
 * the VEML7700 ambient light sensor using I2C communication. It includes
 * initialization, configuration, light level reading, and task support for
 * FreeRTOS environments.
 ******************************************************************************/

#ifndef VEML7700_H
#define VEML7700_H

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "I2cDriver.h"  ///< Modify this include path according to your project structure

/******************************************************************************
 * Macro Definitions
 ******************************************************************************/
/**
 * @defgroup VEML7700_Definitions VEML7700 Constants and Register Definitions
 * @{
 */

/** @brief FreeRTOS task configuration */
#define VEML_TASK_STACK_SIZE (256)                  ///< Stack size for VEML7700 task
#define VEML_TASK_PRIORITY   (configMAX_PRIORITIES - 2) ///< Priority for VEML7700 task

/** @brief VEML7700 I2C slave address (7-bit) */
#define VEML7700_I2C_ADDR       0x10  ///< Some I2C drivers shift address internally

/** @brief VEML7700 register addresses */
#define VEML7700_REG_ALS_CONFIG     0x00  ///< ALS configuration register
#define VEML7700_REG_ALS_THRES_HIGH 0x01  ///< ALS high threshold register
#define VEML7700_REG_ALS_THRES_LOW  0x02  ///< ALS low threshold register
#define VEML7700_REG_POWER_SAVE     0x03  ///< Power saving mode register
#define VEML7700_REG_ALS_DATA       0x04  ///< ALS data output register
#define VEML7700_REG_WHITE_DATA     0x05  ///< WHITE channel data output register
#define VEML7700_REG_INTERRUPT      0x06  ///< Interrupt status register
#define VEML7700_REG_ID             0x07  ///< Device ID register

/** @brief Gain settings (ALS_GAIN bits [12:11]) */
#define VEML7700_GAIN_1        0x00  ///< Gain x1
#define VEML7700_GAIN_2        0x01  ///< Gain x2
#define VEML7700_GAIN_1_8      0x02  ///< Gain x1/8
#define VEML7700_GAIN_1_4      0x03  ///< Gain x1/4

/** @brief Integration time settings (ALS_IT bits [9:6]) */
#define VEML7700_IT_25MS       0x0C  ///< Integration time 25 ms
#define VEML7700_IT_50MS       0x08  ///< Integration time 50 ms
#define VEML7700_IT_100MS      0x00  ///< Integration time 100 ms
#define VEML7700_IT_200MS      0x01  ///< Integration time 200 ms
#define VEML7700_IT_400MS      0x02  ///< Integration time 400 ms
#define VEML7700_IT_800MS      0x03  ///< Integration time 800 ms

/** @brief Persistence protect number settings (ALS_PERS bits [5:4]) */
#define VEML7700_PERS_1        0x00  ///< 1 trigger event
#define VEML7700_PERS_2        0x01  ///< 2 trigger events
#define VEML7700_PERS_4        0x02  ///< 4 trigger events
#define VEML7700_PERS_8        0x03  ///< 8 trigger events

/** @brief Interrupt control (ALS_INT_EN bit [1]) */
#define VEML7700_INT_DISABLE   0x00  ///< Interrupt disabled
#define VEML7700_INT_ENABLE    0x02  ///< Interrupt enabled

/** @brief Power control (ALS_SD bit [0]) */
#define VEML7700_POWER_ON      0x00  ///< Sensor enabled (normal operation)
#define VEML7700_SHUTDOWN      0x01  ///< Sensor shutdown (low power)

/** @brief Return codes */
#define VEML7700_OK            0     ///< Operation successful
#define VEML7700_ERROR         -1    ///< Operation failed or error

/** @} */ // End of VEML7700_Definitions

/******************************************************************************
 * Function Prototypes
 ******************************************************************************/
/**
 * @brief Initializes the VEML7700 ambient light sensor.
 *
 * @param[in] als_gain Gain setting (e.g., VEML7700_GAIN_1, VEML7700_GAIN_2).
 * @param[in] als_integration_time Integration time setting (e.g., VEML7700_IT_100MS).
 * @return VEML7700_OK if successful, otherwise VEML7700_ERROR.
 */
int veml7700_init(uint8_t als_gain, uint8_t als_integration_time);

/**
 * @brief Reads ambient light level in lux from the VEML7700 sensor.
 *
 * @return Measured light intensity in lux, or a negative value on failure.
 */
float veml7700_read_lux(void);

/**
 * @brief Sets the ALS gain.
 *
 * @param[in] gain_value Gain setting value (see VEML7700_GAIN_*).
 * @return VEML7700_OK if successful, otherwise VEML7700_ERROR.
 */
int veml7700_set_gain(uint8_t gain_value);

/**
 * @brief Sets the ALS integration time.
 *
 * @param[in] it_value Integration time setting (see VEML7700_IT_*).
 * @return VEML7700_OK if successful, otherwise VEML7700_ERROR.
 */
int veml7700_set_integration_time(uint8_t it_value);

/**
 * @brief Sets the full 16-bit configuration register value.
 *
 * @param[in] config_value 16-bit configuration register value (reserved bits cleared).
 * @return VEML7700_OK if successful, otherwise VEML7700_ERROR.
 */
int veml7700_set_config(uint16_t config_value);

/**
 * @brief Reads raw ALS 16-bit data output from the sensor.
 *
 * @param[out] als_raw Pointer to store the raw ALS value.
 * @return VEML7700_OK if successful, otherwise VEML7700_ERROR.
 */
int veml7700_read_raw(uint16_t *als_raw);

/**
 * @brief FreeRTOS task function for handling VEML7700 ambient light measurements.
 *
 * @param[in] pvParameters Task input parameters (unused).
 */
void vVemlTask(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif /* VEML7700_H */
