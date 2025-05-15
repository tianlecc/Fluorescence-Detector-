/**************************************************************************/
/**
 * @file        SerialConsole.c
 * @ingroup     Serial Console
 * @brief       This file contains the code required to run the CLI and Serial Debugger.
 *              It initializes a UART channel and uses it to receive commands
 *              from the user and to print debug information.
 *
 * @details     This file will:
 *              --Initialize a SERCOM port as a UART channel running at 115200 baud, 8N1
 *              --Register callbacks for reading/writing characters asynchronously
 *              --Initialize the CLI and Debug Logger data structures
 *
 * Usage:
 *      - Call InitializeSerialConsole() at startup.
 *      - Use SerialConsoleWriteString() to print text.
 *      - Use LogMessage() to print messages that respect log levels.
 *
 * @copyright   
 * @author      
 * @date        Jan 26, 2019
 * @version     0.1
 ****************************************************************************/

/******************************************************************************
 * Includes
 ******************************************************************************/
#include "SerialConsole.h" // Includes circular buffer, FreeRTOS, etc.
// (Removed #include "CliThread.h" to avoid environment conflicts or missing headers)

/******************************************************************************
 * Defines
 ******************************************************************************/
#define RX_BUFFER_SIZE 512 ///< Character buffer size for RX, in bytes
#define TX_BUFFER_SIZE 512 ///< Character buffer size for TX, in bytes

/******************************************************************************
 * Global Variables
 ******************************************************************************/
struct usart_module usart_instance;

char rxCharacterBuffer[RX_BUFFER_SIZE]; ///< Buffer to store received characters
char txCharacterBuffer[TX_BUFFER_SIZE]; ///< Buffer to store characters to be transmitted

cbuf_handle_t cbufRx; ///< Circular buffer handle for receiving characters
cbuf_handle_t cbufTx; ///< Circular buffer handle for transmitting characters

char latestRx; ///< Holds the most recently received character
char latestTx; ///< Holds the most recently transmitted character

enum eDebugLogLevels currentDebugLevel = LOG_INFO_LVL; ///< Default debug log level

/** 
 * If your project requires the CLI thread to wait for incoming data,
 * define a semaphore here and signal it in usart_read_callback() using xSemaphoreGiveFromISR().
 */
SemaphoreHandle_t xRxSemaphore = NULL;

/******************************************************************************
 * Local Function Declarations
 ******************************************************************************/
static void configure_usart(void);
static void configure_usart_callbacks(void);

/******************************************************************************
 * Callback Declarations
 ******************************************************************************/
void usart_write_callback(struct usart_module *const usart_module);
void usart_read_callback(struct usart_module *const usart_module);

/******************************************************************************
 * Global Functions
 ******************************************************************************/
void InitializeSerialConsole(void)
{
    // Initialize circular buffers for RX and TX
    cbufRx = circular_buf_init((uint8_t *)rxCharacterBuffer, RX_BUFFER_SIZE);
    cbufTx = circular_buf_init((uint8_t *)txCharacterBuffer, TX_BUFFER_SIZE);

    // If the CLI thread is expected to block until data is received, create a semaphore here
    // and use xSemaphoreGiveFromISR() inside the RX callback to notify it
    xRxSemaphore = xSemaphoreCreateBinary();

    // Configure USART and register callbacks
    configure_usart();
    configure_usart_callbacks();

    // Start asynchronous read (1 byte at a time); usart_read_callback() will be triggered on completion
    usart_read_buffer_job(&usart_instance, (uint8_t *)&latestRx, 1);
}

void DeinitializeSerialConsole(void)
{
    usart_disable(&usart_instance);
}

void SerialConsoleWriteString(char *string)
{
    if (string == NULL)
    {
        return;
    }

    // Push all characters into the TX circular buffer
    for (size_t iter = 0; iter < strlen(string); iter++)
    {
        circular_buf_put(cbufTx, (uint8_t)string[iter]);
    }

    // If USART is not busy, trigger the write immediately
    if (usart_get_job_status(&usart_instance, USART_TRANSCEIVER_TX) == STATUS_OK)
    {
        // Fetch one byte from the buffer and start transmission
        if (circular_buf_get(cbufTx, (uint8_t *)&latestTx) == 0)
        {
            usart_write_buffer_job(&usart_instance, (uint8_t *)&latestTx, 1);
        }
    }
}

int SerialConsoleReadCharacter(uint8_t *rxChar)
{
    // If there is thread contention, consider using critical sections or locks
    vTaskSuspendAll();
    int ret = circular_buf_get(cbufRx, (uint8_t *)rxChar);
    xTaskResumeAll();
    return ret;
}

enum eDebugLogLevels getLogLevel(void)
{
    return currentDebugLevel;
}

void setLogLevel(enum eDebugLogLevels debugLevel)
{
    currentDebugLevel = debugLevel;
}

void LogMessage(enum eDebugLogLevels level, const char *format, ...)
{
    if (level < currentDebugLevel || currentDebugLevel == LOG_OFF_LVL)
    {
        return;
    }

    char buffer[256];
    memset(buffer, 0, sizeof(buffer));

    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    SerialConsoleWriteString(buffer);
}

/******************************************************************************
 * Local Functions
 ******************************************************************************/
static void configure_usart(void)
{
    struct usart_config config_usart;
    usart_get_config_defaults(&config_usart);

    config_usart.baudrate    = 115200;
    config_usart.mux_setting = EDBG_CDC_SERCOM_MUX_SETTING;
    config_usart.pinmux_pad0 = EDBG_CDC_SERCOM_PINMUX_PAD0;
    config_usart.pinmux_pad1 = EDBG_CDC_SERCOM_PINMUX_PAD1;
    config_usart.pinmux_pad2 = EDBG_CDC_SERCOM_PINMUX_PAD2;
    config_usart.pinmux_pad3 = EDBG_CDC_SERCOM_PINMUX_PAD3;

    while (usart_init(&usart_instance, EDBG_CDC_MODULE, &config_usart) != STATUS_OK)
    {
        // Keep trying until successful
    }

    usart_enable(&usart_instance);
}

static void configure_usart_callbacks(void)
{
    usart_register_callback(&usart_instance,
                            usart_write_callback,
                            USART_CALLBACK_BUFFER_TRANSMITTED);
    usart_register_callback(&usart_instance,
                            usart_read_callback,
                            USART_CALLBACK_BUFFER_RECEIVED);

    usart_enable_callback(&usart_instance, USART_CALLBACK_BUFFER_TRANSMITTED);
    usart_enable_callback(&usart_instance, USART_CALLBACK_BUFFER_RECEIVED);
}

/******************************************************************************
 * Callback Functions
 ******************************************************************************/
void usart_read_callback(struct usart_module *const usart_module)
{
    // Place the latest received character into the RX circular buffer
    circular_buf_put(cbufRx, (uint8_t)latestRx);

    // If the CLI thread waits on a semaphore for input, notify it here
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(xRxSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

    // Start another read job to keep receiving data
    usart_read_buffer_job(&usart_instance, (uint8_t *)&latestRx, 1);
}

void usart_write_callback(struct usart_module *const usart_module)
{
    // Continue transmission if there are still bytes in the TX buffer
    if (circular_buf_get(cbufTx, (uint8_t *)&latestTx) == 0)
    {
        usart_write_buffer_job(&usart_instance, (uint8_t *)&latestTx, 1);
    }
}
