/**************************************************************************/ /**
 * @file      esp_i2c.h
 * @brief     ESP32 I2C Driver interface for controlling NeoPixel via I2C commands.
 *
 * @details   Provides an API and FreeRTOS task for managing communication with
 *            an ESP32 device controlling NeoPixels over I2C.
 *
 * @author    Tianle Chen
 * @date      2025-04
 ******************************************************************************/

#ifndef ESP_I2C_H
#define ESP_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/**************************************************************************/ /**
 * @brief Task configuration constants
 ******************************************************************************/
#define ESP_I2C_TASK_STACK_SIZE   (128)                   ///< Stack size for ESP32 I2C FreeRTOS task
#define ESP_I2C_TASK_PRIORITY     (configMAX_PRIORITIES - 2) ///< Priority for ESP32 I2C task

/**************************************************************************/ /**
 * @brief Enumeration of supported NeoPixel colors.
 ******************************************************************************/
typedef enum {
    ESP_COLOR_OFF = 0,
    ESP_COLOR_RED,
    ESP_COLOR_GREEN,
    ESP_COLOR_BLUE,
    ESP_COLOR_YELLOW,
    ESP_COLOR_CYAN,
    ESP_COLOR_MAGENTA
    // Extend with additional colors if needed
} esp_color_t;

/**************************************************************************/ /**
 * @brief Initializes ESP32 I2C driver data structures.
 *
 * @param[in] i2c_addr  7-bit I2C address for ESP32 (e.g., 0x28).
 * @return 0 on success, non-zero otherwise.
 ******************************************************************************/
int esp_i2c_init(uint8_t i2c_addr);

/**************************************************************************/ /**
 * @brief ESP32 I2C FreeRTOS task to control NeoPixels via I2C.
 *
 * @param[in] pvParameters  Unused FreeRTOS task parameters.
 ******************************************************************************/
void vEspI2cTask(void *pvParameters);

/**************************************************************************/ /**
 * @brief Sets the color flag to trigger the ESP32 I2C task to update NeoPixel color.
 *
 * @param[in] color  Desired NeoPixel color from esp_color_t enum.
 ******************************************************************************/
void esp_i2c_set_color_flag(esp_color_t color);

#ifdef __cplusplus
}
#endif

#endif // ESP_I2C_H
