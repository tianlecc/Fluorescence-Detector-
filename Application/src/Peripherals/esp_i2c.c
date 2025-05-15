/**************************************************************************/ /**
 * @file      esp_i2c.c
 * @brief     FreeRTOS-compatible ESP32 I2C driver and task implementation
 *
 * @details   Provides functionality to set LED color on an ESP32 device using
 *            an I2C communication interface. Utilizes FreeRTOS for task handling
 *            and mutex protection to ensure safe I2C transactions.
 *
 ******************************************************************************/

/******************************************************************************
 * Includes
 ******************************************************************************/
#include "esp_i2c.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "I2cDriver.h"
#include "SerialConsole.h"
#include <stdio.h>

/******************************************************************************
 * Internal Variables
 ******************************************************************************/
static uint8_t g_espI2cAddress = 0x28;             ///< Default ESP32 I2C 7-bit address
static esp_color_t g_colorFlag = ESP_COLOR_OFF;    ///< Desired color to set
static esp_color_t g_lastColor = ESP_COLOR_OFF;    ///< Last color sent via I2C

/******************************************************************************
 * Internal Function Prototypes
 ******************************************************************************/
static void _mapColorToRGB(esp_color_t color, uint8_t *r, uint8_t *g, uint8_t *b);

/******************************************************************************
 * Internal Functions
 ******************************************************************************/
/**
 * @brief Maps esp_color_t enum to RGB values.
 *
 * @param[in]  color  Color enum value.
 * @param[out] r      Red intensity (0-255).
 * @param[out] g      Green intensity (0-255).
 * @param[out] b      Blue intensity (0-255).
 */
static void _mapColorToRGB(esp_color_t color, uint8_t *r, uint8_t *g, uint8_t *b)
{
    switch (color) {
        case ESP_COLOR_OFF:     *r = 0;   *g = 0;   *b = 0;   break;
        case ESP_COLOR_RED:     *r = 255; *g = 0;   *b = 0;   break;
        case ESP_COLOR_GREEN:   *r = 0;   *g = 255; *b = 0;   break;
        case ESP_COLOR_BLUE:    *r = 0;   *g = 0;   *b = 255; break;
        case ESP_COLOR_YELLOW:  *r = 255; *g = 255; *b = 0;   break;
        case ESP_COLOR_CYAN:    *r = 0;   *g = 255; *b = 255; break;
        case ESP_COLOR_MAGENTA: *r = 255; *g = 0;   *b = 255; break;
        default:                *r = 0;   *g = 0;   *b = 0;   break;
    }
}

/******************************************************************************
 * Public API Functions
 ******************************************************************************/
/**
 * @brief Initializes the ESP32 I2C configuration.
 *
 * @param[in] i2c_addr  7-bit I2C device address.
 * @return 0 if success, non-zero otherwise.
 */
int esp_i2c_init(uint8_t i2c_addr)
{
    g_espI2cAddress = i2c_addr;
    g_colorFlag = ESP_COLOR_OFF;
    g_lastColor = ESP_COLOR_OFF;

    // Additional initialization logic can be added here.

    return 0;
}

/**
 * @brief Sets the color flag which is processed by the ESP32 I2C task.
 *
 * @param[in] color Desired color to set.
 */
void esp_i2c_set_color_flag(esp_color_t color)
{
    g_colorFlag = color;
}

/**
 * @brief FreeRTOS task that periodically checks for color change requests
 *        and communicates with ESP32 device over I2C.
 *
 * @param[in] pvParameters Task parameters (unused).
 */
void vEspI2cTask(void *pvParameters)
{
    (void)pvParameters;

    esp_i2c_init(g_espI2cAddress);

    static uint8_t txBuff[4]; // Buffer format: [cmd, R, G, B]
    I2C_Data i2cData;
    i2cData.address = g_espI2cAddress;
    i2cData.msgOut  = txBuff;
    i2cData.lenOut  = sizeof(txBuff);
    i2cData.msgIn   = NULL;
    i2cData.lenIn   = 0;
	
	//esp_i2c_set_color_flag(ESP_COLOR_BLUE);

    while (1)
    {
        esp_color_t currentColor = g_colorFlag;
        if (currentColor != g_lastColor)
        {
            g_lastColor = currentColor;

            uint8_t r, g, b;
            _mapColorToRGB(currentColor, &r, &g, &b);
            txBuff[0] = 1; // SET_COLOR command
            txBuff[1] = r;
            txBuff[2] = g;
            txBuff[3] = b;

            if (I2cGetMutex(pdMS_TO_TICKS(50)) == ERROR_NONE) {
                if (I2cWriteData(&i2cData) == 0) {
                    SerialConsoleWriteString("esp_i2c: Color change command sent.\r\n");
                } else {
                    SerialConsoleWriteString("esp_i2c: Failed to send I2C data.\r\n");
                }
                I2cFreeMutex();
            } else {
                SerialConsoleWriteString("esp_i2c: Could not obtain I2C mutex.\r\n");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
