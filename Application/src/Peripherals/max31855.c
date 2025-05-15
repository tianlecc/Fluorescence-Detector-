/**************************************************************************/ /**
 * @file      max31855.c
 * @brief     MAX31855 Thermocouple-to-Digital Converter Driver
 *
 * @details
 * This file implements initialization, reading, and control logic for the
 * MAX31855 thermocouple-to-digital converter over SPI using FreeRTOS-based APIs.
 * It includes mutex protection to ensure thread-safe SPI communication.
 ******************************************************************************/

/******************************************************************************
 * Includes
 ******************************************************************************/
#include "max31855.h"
#include "SerialConsole.h" /**< Optional: For debug prints via SerialConsoleWriteString */
#include "WifiHandlerThread/WifiHandler.h"


/******************************************************************************
 * Internal Variables
 ******************************************************************************/
/**
 * @brief SPI module instance used exclusively for MAX31855 communication.
 */
static struct spi_module max31855_spi_module;

/**
 * @brief FreeRTOS mutex handle to guard SPI access for MAX31855.
 */
static SemaphoreHandle_t xMax31855Mutex = NULL;

/******************************************************************************
 * Public Variables
 ******************************************************************************/
/**
 * @brief Latest temperature reading from the MAX31855 sensor.
 */
float g_max31855_latest_temp = 0.0f;

/******************************************************************************
 * Internal Function Prototypes
 ******************************************************************************/
static BaseType_t max31855_acquire_mutex(void);
static void max31855_release_mutex(void);

/******************************************************************************
 * Internal Functions
 ******************************************************************************/
/**
 * @brief Acquires the mutex protecting the MAX31855 SPI bus.
 *
 * @return pdTRUE if successful, pdFALSE otherwise.
 */
static BaseType_t max31855_acquire_mutex(void)
{
    const TickType_t xMaxWait = pdMS_TO_TICKS(100);
    return xSemaphoreTake(xMax31855Mutex, xMaxWait);
}

/**
 * @brief Releases the mutex after SPI transaction.
 */
static void max31855_release_mutex(void)
{
    xSemaphoreGive(xMax31855Mutex);
}

/******************************************************************************
 * Public Functions
 ******************************************************************************/

/**
 * @brief Disables and resets the SPI module used by the MAX31855.
 *
 * Stops ongoing SPI transactions and resets SERCOM hardware block.
 */
void max31855_spi_disable_and_reset(void)
{
    spi_disable(&max31855_spi_module);
    spi_reset(&max31855_spi_module);

    /* Note: If spi_reset() is unavailable, use system_periph_reset(SERCOM3) or equivalent. */
}

/**
 * @brief Reinitializes the SPI configuration for the MAX31855 after reset.
 *
 * Call after max31855_spi_disable_and_reset() to restore SPI operation.
 */
void max31855_spi_reinit(void)
{
    struct spi_config config_spi;
    spi_get_config_defaults(&config_spi);

    config_spi.mode                   = SPI_MODE_MASTER;
    config_spi.data_order             = SPI_DATA_ORDER_MSB;
    config_spi.transfer_mode          = SPI_TRANSFER_MODE_0;
    config_spi.mux_setting            = SPI_SIGNAL_MUX_SETTING_D;
    config_spi.pinmux_pad0            = PINMUX_UNUSED;
    config_spi.pinmux_pad1            = PINMUX_PA17D_SERCOM3_PAD1;
    config_spi.pinmux_pad2            = PINMUX_UNUSED;
    config_spi.pinmux_pad3            = PINMUX_PA21D_SERCOM3_PAD3;
    config_spi.run_in_standby         = true;
    config_spi.mode_specific.master.baudrate = 2000000;

    spi_init(&max31855_spi_module, SERCOM3, &config_spi);
    spi_enable(&max31855_spi_module);

    /* Optional: Reconfigure CS pin here if needed */
}

/**
 * @brief Initializes the MAX31855 SPI interface and associated GPIO.
 *
 * Configures SERCOM as SPI master and sets up a mutex for safe access.
 */
void max31855_init(void)
{
    struct spi_config config_spi;
    spi_get_config_defaults(&config_spi);

    config_spi.mode = SPI_MODE_MASTER;
    config_spi.data_order = SPI_DATA_ORDER_MSB;
    config_spi.transfer_mode = SPI_TRANSFER_MODE_0;
    config_spi.mux_setting = SPI_SIGNAL_MUX_SETTING_D;
    config_spi.pinmux_pad0 = PINMUX_UNUSED;
    config_spi.pinmux_pad1 = PINMUX_PA17D_SERCOM3_PAD1;
    config_spi.pinmux_pad2 = PINMUX_UNUSED;
    config_spi.pinmux_pad3 = PINMUX_PA21D_SERCOM3_PAD3;
    config_spi.run_in_standby = true;
    config_spi.mode_specific.master.baudrate = 2000000;

    spi_init(&max31855_spi_module, SERCOM3, &config_spi);
    spi_enable(&max31855_spi_module);

    struct port_config pin_conf;
    port_get_config_defaults(&pin_conf);
    pin_conf.direction = PORT_PIN_DIR_OUTPUT;
    port_pin_set_config(MAX31855_CS_PIN, &pin_conf);
    port_pin_set_output_level(MAX31855_CS_PIN, true);

    xMax31855Mutex = xSemaphoreCreateMutex();
    if (xMax31855Mutex == NULL)
    {
        SerialConsoleWriteString("ERROR: Could not create MAX31855 SPI mutex!\r\n");
    }
}

/**
 * @brief Reads the temperature value from the MAX31855 sensor.
 *
 * @return Temperature in degrees Celsius. Returns -1000.0f on failure.
 */
float max31855_read_temp(void)
{
    enum status_code spi_status;
    float temperatureC = -1000.0f;

    if (max31855_acquire_mutex() == pdTRUE)
    {
        uint8_t rx_buffer[4] = {0};
        uint8_t tx_buffer[4] = {0xFF, 0xFF, 0xFF, 0xFF};

        port_pin_set_output_level(MAX31855_CS_PIN, false);
        spi_status = spi_transceive_buffer_wait(
            &max31855_spi_module, tx_buffer, rx_buffer, 4);
        port_pin_set_output_level(MAX31855_CS_PIN, true);

        if (spi_status == STATUS_OK)
        {
            uint32_t rawData = ((uint32_t)rx_buffer[0] << 24) |
                               ((uint32_t)rx_buffer[1] << 16) |
                               ((uint32_t)rx_buffer[2] << 8)  |
                               ((uint32_t)rx_buffer[3]);

            int16_t tempData = (rawData >> 18) & 0x3FFF;
            if (tempData & 0x2000)
            {
                tempData |= 0xC000;
            }
            temperatureC = (float)tempData * 0.25f;
        }

        max31855_release_mutex();
    }
    else
    {
        SerialConsoleWriteString("WARNING: Could not acquire MAX31855 SPI mutex.\r\n");
    }

    return temperatureC;
}

/**
 * @brief FreeRTOS task to periodically read temperature and control a heating pad.
 *
 * @param[in] pvParameters Unused task parameter.
 *
 * Task behavior:
 * - Reads temperature every second
 * - Updates global variable g_max31855_latest_temp
 * - Controls heating pad using basic hysteresis control.
 */
void vMax31855Task(void *pvParameters)
{
    (void)pvParameters;

    max31855_init();
	char payload[16]; // For Node-Red UI

    struct port_config pin_conf;
    port_get_config_defaults(&pin_conf);
    pin_conf.direction = PORT_PIN_DIR_OUTPUT;
    port_pin_set_config(HEATING_PAD_PIN, &pin_conf);
    port_pin_set_output_level(HEATING_PAD_PIN, false);

    float setpoint = 40.0f;

    for (;;)
    {
		if (g_heating_enable){
			float current = max31855_read_temp();
			WifiAddTemperatureToQueue(current);

			float error = setpoint - current;
			if (error > 0.5f)
			{
				port_pin_set_output_level(HEATING_PAD_PIN, true);
			}
			else if (error < -0.25f)
			{
				port_pin_set_output_level(HEATING_PAD_PIN, false);
			}
		}else{
			port_pin_set_output_level(HEATING_PAD_PIN, false);
		}
        max31855_spi_disable_and_reset();
        vTaskDelay(pdMS_TO_TICKS(1000));
        max31855_spi_reinit();
    }
}
