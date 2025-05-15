/**************************************************************************/ /**
 * @file      max31855.h
 * @brief     MAX31855 Thermocouple-to-Digital Converter Driver Interface
 *
 * @details
 * This file provides function declarations, macros, and global variables
 * for interacting with the MAX31855 device over SPI. It supports initialization,
 * temperature reading, and includes a FreeRTOS task for periodic updates.
 ******************************************************************************/

#ifndef MAX31855_H_
#define MAX31855_H_

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * Includes
 ******************************************************************************/
#include <asf.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "spi.h"

/******************************************************************************
 * Macro Definitions
 ******************************************************************************/
/**
 * @brief Pin definitions for MAX31855 connections.
 *
 * Adjust these macros based on your specific hardware layout.
 */
#define MAX31855_SCK_PIN     PIN_PA17   /**< SPI Clock (SCK) pin */
#define MAX31855_CS_PIN      PIN_PA20   /**< Chip Select (CS) pin */
#define MAX31855_SO_PIN      PIN_PA21   /**< SPI MISO (SO) pin */
#define HEATING_PAD_PIN      PIN_PB03   /**< Heating pad control pin */

/**
 * @brief FreeRTOS task configuration for the MAX31855 monitoring task.
 */
#define MAX31855_TASK_STACK_SIZE   (256U)    /**< Stack size for MAX31855 task */
#define MAX31855_TASK_PRIORITY     (configMAX_PRIORITIES - 2) /**< Priority level for MAX31855 task */

/******************************************************************************
 * Public Function Prototypes
 ******************************************************************************/

/**
 * @brief Initializes the SPI interface and required GPIOs for MAX31855 communication.
 *
 * This function configures the SERCOM peripheral in master mode, sets up pinmuxing,
 * and creates a FreeRTOS mutex to manage concurrent access.
 */
void max31855_init(void);

/**
 * @brief Reads the current temperature from the MAX31855 sensor (blocking).
 *
 * @note This function uses a mutex to ensure safe concurrent access.
 *
 * @return Current temperature in Celsius (float). Returns -1000.0f on error.
 */
float max31855_read_temp(void);

void max31855_spi_disable_and_reset(void);


void max31855_spi_reinit(void);

/**
 * @brief FreeRTOS task for periodically reading the MAX31855 temperature and controlling heating.
 *
 * Task behavior:
 * - Continuously reads temperature.
 * - Updates `g_max31855_latest_temp`.
 * - Controls the heating pad using simple hysteresis logic.
 *
 * @param[in] pvParameters Unused parameter.
 */
void vMax31855Task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif /* MAX31855_H_ */
