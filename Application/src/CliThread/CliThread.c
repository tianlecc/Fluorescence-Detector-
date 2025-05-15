/**
 * @file      CliThread.c
 * @brief     File for the CLI Thread handler. Uses FREERTOS + CLI
 * @author    Eduardo Garcia
 * @date      2020-02-15
 
 ******************************************************************************/

/******************************************************************************
 * Includes
 ******************************************************************************/
#include "CliThread.h"

#include "I2cDriver/I2cDriver.h"
#include "WifiHandlerThread/WifiHandler.h"
#include "task.h"  // For xTaskGetTickCount
#include "max31855.h"  
#include "veml7700.h"
#include "esp_i2c.h"

/******************************************************************************
 * Defines
 ******************************************************************************/

/******************************************************************************
 * Variables
 ******************************************************************************/
static const char pcWelcomeMessage[] = "FreeRTOS CLI.\r\nType Help to view a list of registered commands.\r\n";

static const CLI_Command_Definition_t xOTAUCommand = {"fw", "fw: Download a file and perform an FW update\r\n", (const pdCOMMAND_LINE_CALLBACK)CLI_OTAU, 0};
static const CLI_Command_Definition_t xResetCommand = {"reset", "reset: Resets the device\r\n", (const pdCOMMAND_LINE_CALLBACK)CLI_ResetDevice, 0};
static const CLI_Command_Definition_t xI2cScan = {"i2c", "i2c: Scans I2C bus\r\n", (const pdCOMMAND_LINE_CALLBACK)CLI_i2cScan, 0};
const CLI_Command_Definition_t xClearScreen = {CLI_COMMAND_CLEAR_SCREEN, CLI_HELP_CLEAR_SCREEN, CLI_CALLBACK_CLEAR_SCREEN, CLI_PARAMS_CLEAR_SCREEN};

// New definitions
static const CLI_Command_Definition_t xVersionCommand = {"version","version: Prints firmware version.\r\n",CLI_VersionCommand,0};
static const CLI_Command_Definition_t xTicksCommand = {"ticks","ticks: Prints the number of RTOS ticks since startup.\r\n",CLI_TicksCommand,0 };
static const CLI_Command_Definition_t xGoldCommand ={"gold","gold: Copies application.bin to g_application.bin\r\n",CLI_GoldCommand,0};
static const CLI_Command_Definition_t xTempCommand = {"temp","temp: Prints the current temperature (in Celsius) from the MAX31855.\r\n",CLI_TempCommand,0};
static const CLI_Command_Definition_t xLightCommand ={ "light", "light: Prints the last light-intensity reading (lux).\r\n",CLI_LightCommand, 0 };
static const CLI_Command_Definition_t xLedOffCommand ={ "led_off",  "led_off : Turn RGB LED off.\r\n",CLI_LedOffCommand, 0 };
static const CLI_Command_Definition_t xLedBlueCommand ={ "led_blue", "led_blue: Set LED blue.\r\n",CLI_LedBlueCommand, 0 };
static const CLI_Command_Definition_t xLedRedCommand  ={ "led_red",  "led_red : Set LED red.\r\n", CLI_LedRedCommand, 0 };
static const CLI_Command_Definition_t xLedGreenCommand ={ "led_green","led_green: Set LED green.\r\n",CLI_LedGreenCommand, 0 };


SemaphoreHandle_t cliCharReadySemaphore;  ///< Semaphore to indicate that a character has been received
SemaphoreHandle_t xRxSemaphore;

static uint8_t s_downloadBuffer[400]; //Move it outside the function as a static global variable.

/******************************************************************************
 * Forward Declarations
 ******************************************************************************/
static void FreeRTOS_read(char *character);

/******************************************************************************
 * Callback Functions
 ******************************************************************************/

/******************************************************************************
 * CLI Thread
 ******************************************************************************/

void vCommandConsoleTask(void *pvParameters)
{
    // REGISTER COMMANDS HERE
    FreeRTOS_CLIRegisterCommand(&xOTAUCommand);
    FreeRTOS_CLIRegisterCommand(&xClearScreen);
    FreeRTOS_CLIRegisterCommand(&xResetCommand);
    FreeRTOS_CLIRegisterCommand(&xI2cScan);

    FreeRTOS_CLIRegisterCommand(&xVersionCommand);
    FreeRTOS_CLIRegisterCommand(&xTicksCommand);
    FreeRTOS_CLIRegisterCommand(&xGoldCommand);
    FreeRTOS_CLIRegisterCommand(&xTempCommand);  
	FreeRTOS_CLIRegisterCommand( &xLightCommand   );
    FreeRTOS_CLIRegisterCommand( &xLedOffCommand  );
    FreeRTOS_CLIRegisterCommand( &xLedBlueCommand );
    FreeRTOS_CLIRegisterCommand( &xLedRedCommand  );
    FreeRTOS_CLIRegisterCommand( &xLedGreenCommand);

    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    char cRxedChar[2];
    unsigned char cInputIndex = 0;
    BaseType_t xMoreDataToFollow;
    /* The input and output buffers are declared static to keep them off the stack. */
    static char pcOutputString[MAX_OUTPUT_LENGTH_CLI], pcInputString[MAX_INPUT_LENGTH_CLI];
    static char pcLastCommand[MAX_INPUT_LENGTH_CLI];
    static bool isEscapeCode = false;
    static char pcEscapeCodes[4];
    static uint8_t pcEscapeCodePos = 0;

    /* This code assumes the peripheral being used as the console has already
    been opened and configured, and is passed into the task as the task
    parameter.  Cast the task parameter to the correct type. */

    /* Send a welcome message to the user knows they are connected. */
    SerialConsoleWriteString((char *)pcWelcomeMessage);

    // Any semaphores/mutexes/etc you needed to be initialized, you can do them here
    cliCharReadySemaphore = xSemaphoreCreateBinary();
    if (cliCharReadySemaphore == NULL) {
        LogMessage(LOG_ERROR_LVL, "Could not allocate semaphore\r\n");
        vTaskSuspend(NULL);
    }

    for (;;) {
        FreeRTOS_read(&cRxedChar[0]);

        if (cRxedChar[0] == '\n' || cRxedChar[0] == '\r') {
            /* A newline character was received, so the input command string is
            complete and can be processed.  Transmit a line separator, just to
            make the output easier to read. */
            SerialConsoleWriteString((char *)"\r\n");
            // Copy for last command
            isEscapeCode = false;
            pcEscapeCodePos = 0;
            strncpy(pcLastCommand, pcInputString, MAX_INPUT_LENGTH_CLI - 1);
            pcLastCommand[MAX_INPUT_LENGTH_CLI - 1] = 0;  // Ensure null termination

            /* The command interpreter is called repeatedly until it returns
            pdFALSE.  See the "Implementing a command" documentation for an
            explanation of why this is. */
            do {
                /* Send the command string to the command interpreter.  Any
                output generated by the command interpreter will be placed in the
                pcOutputString buffer. */
                xMoreDataToFollow = FreeRTOS_CLIProcessCommand(pcInputString,        /* The command string.*/
                                                               pcOutputString,       /* The output buffer. */
                                                               MAX_OUTPUT_LENGTH_CLI /* The size of the output buffer. */
                );

                /* Write the output generated by the command interpreter to the
                console. */
                // Ensure it is null terminated
                pcOutputString[MAX_OUTPUT_LENGTH_CLI - 1] = 0;
                SerialConsoleWriteString(pcOutputString);

            } while (xMoreDataToFollow != pdFALSE);

            /* All the strings generated by the input command have been sent.
            Processing of the command is complete.  Clear the input string ready
            to receive the next command. */
            cInputIndex = 0;
            memset(pcInputString, 0x00, MAX_INPUT_LENGTH_CLI);
            memset(pcOutputString, 0, MAX_OUTPUT_LENGTH_CLI);
        } else {

            if (true == isEscapeCode) {
                if (pcEscapeCodePos < CLI_PC_ESCAPE_CODE_SIZE) {
                    pcEscapeCodes[pcEscapeCodePos++] = cRxedChar[0];
                } else {
                    isEscapeCode = false;
                    pcEscapeCodePos = 0;
                }

                if (pcEscapeCodePos >= CLI_PC_MIN_ESCAPE_CODE_SIZE) {
                    // UP ARROW SHOW LAST COMMAND
                    if (strcasecmp(pcEscapeCodes, "oa")) {
                        /// Delete current line and add prompt (">")
                        sprintf(pcInputString, "%c[2K\r>", 27);
                        SerialConsoleWriteString((char *)pcInputString);
                        /// Clear input buffer
                        cInputIndex = 0;
                        memset(pcInputString, 0x00, MAX_INPUT_LENGTH_CLI);
                        /// Send last command
                        strncpy(pcInputString, pcLastCommand, MAX_INPUT_LENGTH_CLI - 1);
                        cInputIndex = (strlen(pcInputString) < MAX_INPUT_LENGTH_CLI - 1) ? strlen(pcLastCommand) : MAX_INPUT_LENGTH_CLI - 1;
                        SerialConsoleWriteString(pcInputString);
                    }

                    isEscapeCode = false;
                    pcEscapeCodePos = 0;
                }
            }
            else if (cRxedChar[0] == '\r') {
                /* Ignore carriage returns. */
            } else if (cRxedChar[0] == ASCII_BACKSPACE || cRxedChar[0] == ASCII_DELETE) {
                char erase[4] = {0x08, 0x20, 0x08, 0x00};
                SerialConsoleWriteString(erase);
                /* Backspace was pressed.  Erase the last character in the input
                buffer - if there are any. */
                if (cInputIndex > 0) {
                    cInputIndex--;
                    pcInputString[cInputIndex] = 0;
                }
            }
            // ESC
            else if (cRxedChar[0] == ASCII_ESC) {
                isEscapeCode = true;  // Next characters will be code arguments
                pcEscapeCodePos = 0;
            } else {
                /* A character was entered.  It was not a new line, backspace
                or carriage return, so it is accepted as part of the input and
                placed into the input buffer.  When a n is entered the complete
                string will be passed to the command interpreter. */
                if (cInputIndex < MAX_INPUT_LENGTH_CLI) {
                    pcInputString[cInputIndex] = cRxedChar[0];
                    cInputIndex++;
                }

                // Order Echo
                cRxedChar[1] = 0;
                SerialConsoleWriteString(&cRxedChar[0]);
            }
        }
    }
}

/**
 * @fn			void FreeRTOS_read(char* character)
 * @brief		STUDENTS TO COMPLETE. This function block the thread unless we received a character. How can we do this?
                                 There are multiple solutions! Check all the inter-thread communications available! See https://www.freertos.org/a00113.html
 * @details		STUDENTS TO COMPLETE.
 * @note
 *****************************************************************************/
static void FreeRTOS_read(char *character)
{
    if (xSemaphoreTake(xRxSemaphore, portMAX_DELAY) == pdTRUE)
    {
        uint8_t tempChar = 0;
        if (SerialConsoleReadCharacter(&tempChar) == 0)
        {
            *character = (char)tempChar;
        }
        else
        {
            *character = '\0';
        }
    }
    else
    {
        *character = '\0';
    }
}

/**
  * @fn			void CliCharReadySemaphoreGiveFromISR(void)
  * @brief		Give cliCharReadySemaphore binary semaphore from an ISR
  * @details
  * @note
  */
void CliCharReadySemaphoreGiveFromISR(void)
{
    static BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(cliCharReadySemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


/******************************************************************************
 * CLI Functions - Define here
 ******************************************************************************/

// THIS COMMAND USES vt100 TERMINAL COMMANDS TO CLEAR THE SCREEN ON A TERMINAL PROGRAM LIKE TERA TERM
// SEE http://www.csie.ntu.edu.tw/~r92094/c++/VT100.html for more info
// CLI SPECIFIC COMMANDS
static char bufCli[CLI_MSG_LEN];
BaseType_t xCliClearTerminalScreen(char *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
    char clearScreen = ASCII_ESC;
    snprintf(bufCli, CLI_MSG_LEN - 1, "%c[2J", clearScreen);
    snprintf(pcWriteBuffer, xWriteBufferLen, bufCli);
    return pdFALSE;
}

// Example CLI Command. Reads from the IMU and returns data.


/**************************************************************
 * 2) The CLI_OTAU Command
 *    Call this via "fw" in your serial console
 **************************************************************/
BaseType_t CLI_OTAU(int8_t *pcWriteBuffer,
                    size_t xWriteBufferLen,
                    const int8_t *pcCommandString)
{
    FRESULT fr;
    // Delete previous firmware files: application.bin and g_application.bin
    fr = f_unlink("0:/application.bin");
    if (fr != FR_OK && fr != FR_NO_FILE) {
        SerialConsoleWriteString("Warning: Failed to delete previous application.bin\r\n");
    }

    SerialConsoleWriteString("Starting firmware download via WifiHandler...\r\n");
    WifiHandlerSetState(WIFI_DOWNLOAD_INIT);

    snprintf((char*)pcWriteBuffer, xWriteBufferLen,
             "Firmware download started. Once complete, device will reset.\r\n");
    WifiHandlerSetState(WIFI_DOWNLOAD_HANDLE);

    return pdFALSE; 
}

/**
 * @brief CLI command "gold": Copies the current 0:/application.bin to 0:/g_application.bin
 *        This command helps create a "Golden Image" backup on the SD card.
 */
BaseType_t CLI_GoldCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
    FIL srcFile, dstFile;
    FRESULT fr;
    UINT br, bw;
    char copyBuffer[256];

    // Attempt to open "0:/application.bin" in read mode.
    fr = f_open(&srcFile, "0:/application.bin", FA_READ);
    if (fr != FR_OK) {
        snprintf((char*)pcWriteBuffer, xWriteBufferLen,
                 "Error: Could not open application.bin\r\n");
        return pdFALSE;
    }

    // Delete existing "0:/g_application.bin" if it exists.
    fr = f_unlink("0:/g_application.bin");
    if (fr != FR_OK && fr != FR_NO_FILE) {
         SerialConsoleWriteString("Warning: Could not delete previous g_application.bin\r\n");
    }

    // Create or overwrite "0:/g_application.bin".
    fr = f_open(&dstFile, "0:/g_application.bin", FA_CREATE_ALWAYS | FA_WRITE);
    if (fr != FR_OK) {
        f_close(&srcFile);
        snprintf((char*)pcWriteBuffer, xWriteBufferLen,
                 "Error: Could not create g_application.bin\r\n");
        return pdFALSE;
    }

    // Copy in chunks of 256 bytes.
    for (;;) {
        fr = f_read(&srcFile, copyBuffer, sizeof(copyBuffer), &br);
        if (fr != FR_OK || br == 0) {
            // Either an error or EOF reached
            break;
        }
        fr = f_write(&dstFile, copyBuffer, br, &bw);
        if (fr != FR_OK || bw < br) {
            // Error writing to file
            break;
        }
    }
    f_close(&srcFile);
    f_close(&dstFile);

    if (fr == FR_OK) {
        snprintf((char*)pcWriteBuffer, xWriteBufferLen,
                 "Golden Image Created: g_application.bin\r\n");
    } else {
        snprintf((char*)pcWriteBuffer, xWriteBufferLen,
                 "Error copying to g_application.bin\r\n");
    }

    return pdFALSE;
}


// Example CLI Command. Resets system.
BaseType_t CLI_ResetDevice(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
    WifiPublishResetNotification();
    system_reset();
    return pdFALSE;
}

/**
 * @brief    Scans fot connected i2c devices
 * @param    p_cli
 * @param    argc
 * @param    argv
 ******************************************************************************/
BaseType_t CLI_i2cScan(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
    I2C_Data i2cDevice;
    uint8_t address;
    // Send 0 command byte
    uint8_t dataOut[2] = {0, 0};
    uint8_t dataIn[2];
    dataOut[0] = 0;
    dataOut[1] = 0;
    i2cDevice.address = 0;
    i2cDevice.msgIn = (uint8_t *)&dataIn[0];
    i2cDevice.lenOut = 1;
    i2cDevice.msgOut = (const uint8_t *)&dataOut[0];
    i2cDevice.lenIn = 1;

    SerialConsoleWriteString("0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
    for (int i = 0; i < 128; i += 16) {
        snprintf(bufCli, CLI_MSG_LEN - 1, "%02x: ", i);
        SerialConsoleWriteString(bufCli);

        for (int j = 0; j < 16; j++) {
            i2cDevice.address = (i + j) << 1;

            int32_t ret = I2cWriteDataWait(&i2cDevice, 100);
            if (ret == 0) {
                snprintf(bufCli, CLI_MSG_LEN - 1, "%02x: ", i2cDevice.address);
                SerialConsoleWriteString(bufCli);
            } else {
                snprintf(bufCli, CLI_MSG_LEN - 1, "X ");
                SerialConsoleWriteString(bufCli);
            }
        }
        SerialConsoleWriteString("\r\n");
    }
    SerialConsoleWriteString("\r\n");
    return pdFALSE;
}

BaseType_t CLI_VersionCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
    snprintf((char *)pcWriteBuffer, xWriteBufferLen, "Firmware version: %s\r\n", FIRMWARE_VERSION);
    return pdFALSE;
}

BaseType_t CLI_TicksCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
    TickType_t ticks = xTaskGetTickCount();
    snprintf((char *)pcWriteBuffer, xWriteBufferLen, "%lu\r\n", (unsigned long)ticks);
    return pdFALSE;
}

/**
 * @brief CLI command function for "temp".
 *
 * When the user types "temp", it will print the last-read temperature
 * from the MAX31855 sensor. The reading is updated in @ref vMax31855Task().
 *
 * @param[in,out] pcWriteBuffer   Buffer for output text.
 * @param[in]     xWriteBufferLen Length of the buffer.
 * @param[in]     pcCommandString User's full command string (unused here).
 * @return pdFALSE, indicating the command does not return more data.
 */
BaseType_t CLI_TempCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
    (void)pcCommandString; // Unused parameter
	
    max31855_spi_disable_and_reset();
    max31855_spi_reinit();
	float temp_cli = max31855_read_temp();
	uint16_t temp_int_cli = (uint16_t)(temp_cli + 0.5f);

    /* Print the most recent temperature reading */
    snprintf((char *)pcWriteBuffer, xWriteBufferLen, "Current Temperature: %d C\r\n", temp_int_cli);
    return pdFALSE; // No more strings to return
}

/***************************************************************************//**
 * @brief CLI command: Print latest VEML7700 light intensity (lux).
 *
 * Reads ambient light intensity from the VEML7700 sensor and prints
 * the value (rounded to the nearest integer).
 *
 * @param[in,out] pcWriteBuffer   Buffer to write the output string.
 * @param[in]     xWriteBufferLen Size of output buffer.
 * @param[in]     pcCommandString Full input command string (unused).
 *
 * @return pdFALSE (no additional output expected).
 ******************************************************************************/
BaseType_t CLI_LightCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
    (void)pcCommandString;

    float lux = veml7700_read_lux();
    uint16_t lux_int = (uint16_t)(lux + 0.5f);

    snprintf((char *)pcWriteBuffer, xWriteBufferLen,
             "Light Intensity: %d Lux\r\n", lux_int);
    return pdFALSE;
}

/***************************************************************************//**
 * @brief CLI command: Turn RGB LED OFF.
 *
 * Sets the ESP32-controlled RGB LED color to OFF (0,0,0).
 *
 * @param[in,out] pcWriteBuffer   Buffer to write the output string.
 * @param[in]     xWriteBufferLen Size of output buffer.
 * @param[in]     pcCommandString Full input command string (unused).
 *
 * @return pdFALSE (no additional output expected).
 ******************************************************************************/
BaseType_t CLI_LedOffCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
    (void)pcCommandString;

    esp_i2c_set_color_flag(ESP_COLOR_OFF);

    snprintf((char *)pcWriteBuffer, xWriteBufferLen,
             "LED set to OFF\r\n");
    return pdFALSE;
}

/***************************************************************************//**
 * @brief CLI command: Set RGB LED to BLUE.
 *
 * Sets the ESP32-controlled RGB LED to solid blue color.
 *
 * @param[in,out] pcWriteBuffer   Buffer to write the output string.
 * @param[in]     xWriteBufferLen Size of output buffer.
 * @param[in]     pcCommandString Full input command string (unused).
 *
 * @return pdFALSE (no additional output expected).
 ******************************************************************************/
BaseType_t CLI_LedBlueCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
    (void)pcCommandString;

    esp_i2c_set_color_flag(ESP_COLOR_BLUE);

    snprintf((char *)pcWriteBuffer, xWriteBufferLen,
             "LED set to BLUE\r\n");
    return pdFALSE;
}

/***************************************************************************//**
 * @brief CLI command: Set RGB LED to RED.
 *
 * Sets the ESP32-controlled RGB LED to solid red color.
 *
 * @param[in,out] pcWriteBuffer   Buffer to write the output string.
 * @param[in]     xWriteBufferLen Size of output buffer.
 * @param[in]     pcCommandString Full input command string (unused).
 *
 * @return pdFALSE (no additional output expected).
 ******************************************************************************/
BaseType_t CLI_LedRedCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
    (void)pcCommandString;

    esp_i2c_set_color_flag(ESP_COLOR_RED);

    snprintf((char *)pcWriteBuffer, xWriteBufferLen,
             "LED set to RED\r\n");
    return pdFALSE;
}

/***************************************************************************//**
 * @brief CLI command: Set RGB LED to GREEN.
 *
 * Sets the ESP32-controlled RGB LED to solid green color.
 *
 * @param[in,out] pcWriteBuffer   Buffer to write the output string.
 * @param[in]     xWriteBufferLen Size of output buffer.
 * @param[in]     pcCommandString Full input command string (unused).
 *
 * @return pdFALSE (no additional output expected).
 ******************************************************************************/
BaseType_t CLI_LedGreenCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
    (void)pcCommandString;

    esp_i2c_set_color_flag(ESP_COLOR_GREEN);

    snprintf((char *)pcWriteBuffer, xWriteBufferLen,
             "LED set to GREEN\r\n");
    return pdFALSE;
}

