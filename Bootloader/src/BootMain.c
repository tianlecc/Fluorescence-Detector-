/**
 * @file      BootMain.c
 * @brief     Main file for the ESE5160 bootloader. Handles updating the main application
 * @details   Main file for the ESE5160 bootloader. Handles updating the main application
 *            This version verifies the last 4 bytes of firmware files as a CRC32,
 *            then only burns (fileSize - 4) to Flash.
 * @author    
 * @date      
 * @version   
 */

/**************************************************************************
 * Includes
 **************************************************************************/
#include "conf_example.h"
#include "sd_mmc_spi.h"
#include <asf.h>
#include <string.h>

#include "ASF/sam0/drivers/dsu/crc32/crc32.h"  // For DSU-based CRC on FLASH
#include "ASF/sam0/drivers/nvm/nvm.h"
#include "SD Card/SdCard.h"
#include "SerialConsole/SerialConsole.h"
#include "Systick/Systick.h"

/**************************************************************************
 * Defines
 **************************************************************************/
#define APP_START_ADDRESS           ((uint32_t) 0xF000) ///< Start of main application. Must be address of start of main application
#define APP_START_RESET_VEC_ADDRESS (APP_START_ADDRESS + 0x04)

#define FLASH_PAGE_SIZE             (64U)
#define FLASH_ROW_SIZE              (256U)

/**************************************************************************
 * Structures and Enumerations
 **************************************************************************/
struct usart_module cdc_uart_module;   ///< Structure for UART module connected to EDBG (used for unit test output)

/**************************************************************************
 * Local Function Declaration
 **************************************************************************/
static void jumpToApplication(void);
static bool StartFilesystemAndTest(void);
static void configure_nvm(void);


/**
 * @brief Reads the specified .bin file from the SD card and writes it
 *        into the application region of Flash starting at APP_START_ADDRESS,
 *        writing the entire file from start to finish (no truncation).
 * @note  This is your older function, left intact for reference or other usage.
 */
static bool LoadFirmwareFromSDtoFlash(const char *binFilename);

/**************************************************************************
 * "Check CRC at the end" approach
 **************************************************************************/
/**
 * @brief   Calculate CRC32 for the first (fileSize - 4) bytes of the file and compare with the last 4 bytes.
 *          If they match, burn only the first (fileSize - 4) bytes into Flash.
 */
static bool CheckAndBurnCrcFirmware(const char *binFilename);

/**
 * @brief   Read the specified length (validSize) of data from the SD card and write to Flash starting at APP_START_ADDRESS.
 * @note    Write in rows of 256 bytes and pages of 64 bytes.
 */
static bool LoadFirmwareFromSDtoFlashTruncate(const char *binFilename, uint32_t validSize);

/**
 * @brief   Calculate the CRC32 (IEEE 802.3 Polynomial) for the first 'length' bytes of the file.
 * @note    DSU is not used here, because DSU is for computing the CRC of the MCU's Flash.
 *          Since we need to compute the CRC of an SD file, software or another method is used.
 */
static bool compute_file_crc32(const char *filename, uint32_t length, uint32_t *outCrc);

/**************************************************************************
 * (Optional) DSU-based function to verify firmware in Flash
 **************************************************************************/
/**
 * @brief   Verifies the CRC32 of the firmware stored in Flash (already burned).
 * @details Example usage if you want to confirm the final result in Flash.
 *          Currently not used in the main flow, but you can call after burn.
 */
static bool verify_firmware_crc(uint32_t expected_crc, uint32_t firmware_length)
{
    uint32_t computed_crc = 0xFFFFFFFFU;  // Initial value
    char buffer[64];

    // Calculate CRC32 over the application region in Flash using DSU
    if (dsu_crc32_cal(APP_START_ADDRESS, firmware_length, &computed_crc) != STATUS_OK) {
        SerialConsoleWriteString("CRC calculation failed!\r\n");
        return false;
    }
    computed_crc = ~computed_crc;

    sprintf(buffer, "Computed CRC: 0x%08X\r\n", computed_crc);
    SerialConsoleWriteString(buffer);

    // Compare computed CRC with expected value
    if (computed_crc != expected_crc) {
        sprintf(buffer, "CRC mismatch! Expected: 0x%08X, Got: 0x%08X\r\n",
                expected_crc, computed_crc);
        SerialConsoleWriteString(buffer);
        return false;
    }

    SerialConsoleWriteString("CRC match! Firmware integrity verified.\r\n");
    return true;
}

/**************************************************************************
 * Global Variables
 **************************************************************************/
char test_file_name[] = "0:sd_mmc_test.txt"; ///< Test TEXT File name
char test_bin_file[]  = "0:sd_binary.bin";   ///< Test BINARY File name
Ctrl_status status;
FRESULT res;
FATFS fs;
FIL file_object;

/**************************************************************************
 * Global Functions
 **************************************************************************/

/**
* @fn      int main(void)
* @brief   Main function for ESE5160 Bootloader Application
*
* @return  Unused (ANSI-C compatibility).
* @note    Bootloader code initiates here.
*****************************************************************************/
int main(void)
{
    /*1.) INIT SYSTEM PERIPHERALS INITIALIZATION*/
    system_init();
    delay_init();
    InitializeSerialConsole();
    system_interrupt_enable_global();

    /* Initialize SD MMC stack */
    sd_mmc_init();

    // Initialize the NVM driver
    configure_nvm();

    irq_initialize_vectors();
    cpu_irq_enable();

    // Configure DSU CRC
    dsu_crc32_init();

    SerialConsoleWriteString("ESE5160 - ENTER BOOTLOADER\r\n");

    /* END SYSTEM PERIPHERALS INITIALIZATION*/

    /*2.) STARTS SIMPLE SD CARD MOUNTING AND TEST!*/
    SerialConsoleWriteString("\x0C\n\r-- SD/MMC Card Example on FatFs --\n\r");
    if (!StartFilesystemAndTest()) {
        SerialConsoleWriteString("SD CARD failed! Check your connections. System will restart in 5 seconds...\r\n");
        delay_cycles_ms(5000);
        system_reset();
    } else {
        SerialConsoleWriteString("SD CARD mount success! Filesystem also mounted.\r\n");
    }
    /* END SIMPLE SD CARD MOUNTING AND TEST!*/


    /*3.) STARTS BOOTLOADER HERE!*/
    {
        FRESULT flagRes;
        FIL flagFile;
        bool flagUpdate = false;

        // Check if "FlagA.txt" exists
        flagRes = f_open(&flagFile, "0:FlagA.txt", FA_READ);
        if (flagRes == FR_OK) {
            flagUpdate = true;
            f_close(&flagFile);
        }

        if (flagUpdate) {
            SerialConsoleWriteString("FlagA.txt detected - updating firmware from application.bin...\r\n");

            // First, attempt to verify and burn "application.bin" (last 4 bytes contain the CRC)
            if (CheckAndBurnCrcFirmware("0:application.bin")) {
                // After a successful update, delete FlagUpdate.txt
                f_unlink("0:FlagA.txt");
            } else {
                // If that fails, fall back to using "g_application.bin"
                SerialConsoleWriteString("Error updating firmware from application.bin.\r\n");
                SerialConsoleWriteString("Falling back to g_application.bin...\r\n");

                if (LoadFirmwareFromSDtoFlash("0:g_application.bin")) {
                    f_unlink("0:FlagA.txt");
                } else {
                    SerialConsoleWriteString("Error updating firmware from g_application.bin.\r\n");
                }
            }
        } else {
            SerialConsoleWriteString("No update flag found. Go to the firmware in the flash.\r\n");
        }
    }
    /* END BOOTLOADER HERE!*/


    // 4.) DEINITIALIZE HW AND JUMP TO MAIN APPLICATION!
    SerialConsoleWriteString("ESE5160 - EXIT BOOTLOADER\r\n");
    delay_cycles_ms(100);

    DeinitializeSerialConsole();
    sd_mmc_deinit();

    // Jump to application
    jumpToApplication();

    // Should not reach here! The device should have jumped to the main firmware.
    while(1);
}

/**************************************************************************
 * Static Functions
 **************************************************************************/

/**
 * @brief       Starts the filesystem and tests it. Sets the filesystem to the global variable fs.
 * @details     This function performs a test by creating a text file, writing to it, and then
 *              creating a binary file and writing to it. It is used to verify that the SD card
 *              and FATFS are functioning before jumping to the main application.
 * @return      Returns true if the SD card and filesystem test passed, false otherwise.
 ***************************************************************************/
static bool StartFilesystemAndTest(void)
{
    bool sdCardPass = true;
    uint8_t binbuff[256];

    // Before we begin - fill the buffer for the binary write test with values 0x00 to 0xFF
    for (int i = 0; i < 256; i++) {
        binbuff[i] = (uint8_t)i;
    }

    // MOUNT SD CARD
    Ctrl_status sdStatus = SdCard_Initiate();
    if (sdStatus == CTRL_GOOD) {
        SerialConsoleWriteString("SD Card initiated correctly!\r\n");

        // Attempt to mount a FAT file system on the SD card using FATFS
        SerialConsoleWriteString("Mount disk (f_mount)...\r\n");
        memset(&fs, 0, sizeof(FATFS));
        res = f_mount(LUN_ID_SD_MMC_0_MEM, &fs);
        if (FR_INVALID_DRIVE == res) {
            LogMessage(LOG_INFO_LVL, "[FAIL] res %d\r\n", res);
            return false;
        }
        SerialConsoleWriteString("[OK]\r\n");

        // Create and open a file
        SerialConsoleWriteString("Create a file (f_open)...\r\n");
        test_file_name[0] = LUN_ID_SD_MMC_0_MEM + '0';
        res = f_open(&file_object, (char const *) test_file_name, FA_CREATE_ALWAYS | FA_WRITE);
        if (res != FR_OK) {
            LogMessage(LOG_INFO_LVL, "[FAIL] res %d\r\n", res);
            sdCardPass = false;
            goto test_end;
        }
        SerialConsoleWriteString("[OK]\r\n");

        // Write to a file
        SerialConsoleWriteString("Write to test file (f_puts)...\r\n");
        if (0 == f_puts("Test SD/MMC stack\n", &file_object)) {
            f_close(&file_object);
            LogMessage(LOG_INFO_LVL, "[FAIL]\r\n");
            sdCardPass = false;
            goto test_end;
        }
        SerialConsoleWriteString("[OK]\r\n");
        f_close(&file_object); // Close file

        SerialConsoleWriteString("Test is successful.\r\n");

        // Write binary file
        test_bin_file[0] = LUN_ID_SD_MMC_0_MEM + '0';
        res = f_open(&file_object, (char const *) test_bin_file, FA_WRITE | FA_CREATE_ALWAYS);
        if (res != FR_OK) {
            SerialConsoleWriteString("Could not open binary file!\r\n");
            LogMessage(LOG_INFO_LVL, "[FAIL] res %d\r\n", res);
            sdCardPass = false;
            goto test_end;
        }

        // Write to a binary file
        SerialConsoleWriteString("Write to test file (f_write)...\r\n");
        uint32_t varWrite = 0;
        if (0 != f_write(&file_object, binbuff, 256, &varWrite)) {
            f_close(&file_object);
            LogMessage(LOG_INFO_LVL, "[FAIL]\r\n");
            sdCardPass = false;
            goto test_end;
        }
        SerialConsoleWriteString("[OK]\r\n");
        f_close(&file_object);

        SerialConsoleWriteString("Test is successful.\r\n");

    test_end:
        SerialConsoleWriteString("End of Test.\r\n");
    } else {
        SerialConsoleWriteString("SD Card failed initiation! Check connections!\r\n");
        sdCardPass = false;
    }

    return sdCardPass;
}

/**
 * @brief       Jumps to the main application.
 * @details     Before jumping, please turn off ALL peripherals that were turned on by the bootloader.
 * @return      None.
 ***************************************************************************/
static void jumpToApplication(void)
{
    // Function pointer to the application section
    void (*applicationCodeEntry)(void);

    // Rebase the stack pointer
    __set_MSP(*(uint32_t *) APP_START_ADDRESS);

    // Rebase the vector table
    SCB->VTOR = ((uint32_t) APP_START_ADDRESS & SCB_VTOR_TBLOFF_Msk);

    // Set pointer to the application section (Reset Handler)
    applicationCodeEntry = (void (*)(void))(*(unsigned *) (APP_START_RESET_VEC_ADDRESS));

    // Jump to application
    applicationCodeEntry();
}

/**
 * @brief       Configures the NVM driver.
 * @details     Retrieves default configuration, modifies settings, and sets the configuration.
 * @return      None.
 ***************************************************************************/
static void configure_nvm(void)
{
    struct nvm_config config_nvm;
    nvm_get_config_defaults(&config_nvm);
    config_nvm.manual_page_write = false;
    nvm_set_config(&config_nvm);
}

/**************************************************************************
 *  1) Calculate CRC32 for the first (fileSize - 4) bytes and compare with the last 4 bytes.
 *  2) If they match, burn (fileSize - 4) bytes to Flash.
 **************************************************************************/
static bool CheckAndBurnCrcFirmware(const char *binFilename)
{
    // Open file to get file size
    FIL fp;
    FRESULT fr = f_open(&fp, binFilename, FA_READ);
    if (fr != FR_OK) {
        SerialConsoleWriteString("Cannot open file for CRC check.\r\n");
        return false;
    }
    uint32_t fileSize = fp.fsize;
    f_close(&fp);

    if (fileSize < 4) {
        SerialConsoleWriteString("File too small for appended CRC.\r\n");
        return false;
    }

    // Read the last 4 bytes
    uint8_t last4[4];
    fr = f_open(&fp, binFilename, FA_READ);
    if (fr == FR_OK) {
        f_lseek(&fp, fileSize - 4);
        UINT readLen;
        fr = f_read(&fp, last4, 4, &readLen);
        f_close(&fp);
    }

    if (fr != FR_OK) {
        SerialConsoleWriteString("Failed to read last 4 bytes of file.\r\n");
        return false;
    }

    // Little-endian parsing
    uint32_t appendedCrc = last4[0]
                         | (last4[1] << 8)
                         | (last4[2] << 16)
                         | (last4[3] << 24);

    // Compute CRC32 for the first (fileSize - 4) bytes
    uint32_t computedCrc = 0;
    if (!compute_file_crc32(binFilename, fileSize - 4, &computedCrc)) {
        SerialConsoleWriteString("Error computing file CRC.\r\n");
        return false;
    }

    char msg[128];
    sprintf(msg, "Appended CRC=0x%08lX, Computed CRC=0x%08lX\r\n", (unsigned long)appendedCrc, (unsigned long)computedCrc);
    SerialConsoleWriteString(msg);

    if (appendedCrc == computedCrc) {
        // If CRC valid, burn (fileSize - 4) bytes to Flash
        sprintf(msg, "CRC valid. Burning %lu bytes.\r\n", (unsigned long)(fileSize - 4));
        SerialConsoleWriteString(msg);
        if (LoadFirmwareFromSDtoFlashTruncate(binFilename, fileSize - 4)) {
            SerialConsoleWriteString("Firmware updated via CRC method.\r\n");
            return true;
        } else {
            SerialConsoleWriteString("CRC method burn fail!\r\n");
            return false;
        }
    } else {
        SerialConsoleWriteString("CRC mismatch!\r\n");
        return false;
    }
}

/**************************************************************************
 *  Read from SD Card and burn only the first validSize bytes to FLASH
 **************************************************************************/
static bool LoadFirmwareFromSDtoFlashTruncate(const char *binFilename, uint32_t validSize)
{
    FRESULT res;
    FIL binFile;
    UINT bytesRead = 0;
    bool updateSuccess = true;

    uint8_t buffer[FLASH_ROW_SIZE];
    uint32_t currentAddr = APP_START_ADDRESS;
    uint32_t bytesRemaining = validSize;

    // Open file
    res = f_open(&binFile, binFilename, FA_READ);
    if (res != FR_OK) {
        SerialConsoleWriteString("Truncate: open file fail.\r\n");
        return false;
    }

    SerialConsoleWriteString("Starting truncated firmware update...\r\n");

    while (bytesRemaining > 0) {
        uint32_t chunkSize = (bytesRemaining >= FLASH_ROW_SIZE) ? FLASH_ROW_SIZE : bytesRemaining;
        memset(buffer, 0xFF, FLASH_ROW_SIZE);

        res = f_read(&binFile, buffer, chunkSize, &bytesRead);
        if (res != FR_OK || bytesRead == 0) {
            updateSuccess = false;
            break;
        }

        // Erase 256 bytes row
        if (nvm_erase_row(currentAddr) != STATUS_OK) {
            SerialConsoleWriteString("Error erasing row!\r\n");
            updateSuccess = false;
            break;
        }

        // Write 4 pages (64B each)
        for (uint32_t i = 0; i < (FLASH_ROW_SIZE / FLASH_PAGE_SIZE); i++) {
            enum status_code writeRet = nvm_write_buffer(currentAddr + (i * FLASH_PAGE_SIZE),
                                                         &buffer[i * FLASH_PAGE_SIZE],
                                                         FLASH_PAGE_SIZE);
            if (writeRet != STATUS_OK) {
                SerialConsoleWriteString("Error writing page!\r\n");
                updateSuccess = false;
                break;
            }
        }
        if (!updateSuccess) {
            break;
        }

        currentAddr    += bytesRead;
        bytesRemaining -= bytesRead;
    }

    f_close(&binFile);

    if (updateSuccess) {
        SerialConsoleWriteString("Truncated firmware update done!\r\n");
    } else {
        SerialConsoleWriteString("Truncated firmware update failed!\r\n");
    }

    return updateSuccess;
}

/**************************************************************************
 *  Compute the CRC32 for the first 'length' bytes of the file on the SD card
 *  using a method outside of ASF/SDK (such as a software loop or other method).
 *  Here, an incremental approach is used.
 **************************************************************************/
static bool compute_file_crc32(const char *filename, uint32_t length, uint32_t *outCrc)
{
    // Microchip's "crc32_cal()" is for calculating a buffer in memory or
    // to recalc partial. But we want to read from file. So let's do it manually.

    FIL fp;
    FRESULT fr = f_open(&fp, filename, FA_READ);
    if (fr != FR_OK) {
        return false;
    }

    uint8_t buffer[256];
    UINT bytesRead;
    uint32_t remaining = length;
    // We'll implement a straightforward polynomial approach or partial approach using "crc32_cal" in a loop.
    // But let's do a simpler approach with the ASF's software: We'll feed data in chunks.

    crc32_t crcVal = 0; // let "crcVal=0" => then first call is "crc32_cal(...)"

    // We will read file in small pieces
    while (remaining > 0) {
        uint32_t chunk = (remaining > sizeof(buffer)) ? sizeof(buffer) : remaining;
        fr = f_read(&fp, buffer, chunk, &bytesRead);
        if (fr != FR_OK || bytesRead == 0) {
            f_close(&fp);
            return false;
        }
        remaining -= bytesRead;

        // If first call, use crc32_cal. If subsequent calls, use crc32_recalculate
        // But simpler is to do:
        if (crcVal == 0) {
            // first block
            if (crc32_calculate(buffer, bytesRead, &crcVal) != STATUS_OK) {
                f_close(&fp);
                return false;
            }
        } else {
            if (crc32_recalculate(buffer, bytesRead, &crcVal) != STATUS_OK) {
                f_close(&fp);
                return false;
            }
        }
    }

    f_close(&fp);
    *outCrc = crcVal;
    return true;
}

/**
 * @brief   Reads the specified .bin file from the SD card and writes it into the
 *          application region of Flash starting at APP_START_ADDRESS.
 * @param   binFilename The path to the .bin file on SD (e.g. "0:TestA.bin")
 * @return  true if the write is successful, false otherwise
 */
static bool LoadFirmwareFromSDtoFlash(const char *binFilename)
{
    FRESULT res;
    FIL binFile;
    uint32_t currentAddr = APP_START_ADDRESS;
    UINT bytesRead = 0;
    uint8_t buffer[FLASH_ROW_SIZE];  // 256-byte buffer for row write
    bool updateSuccess = true;

    // Open the .bin file
    res = f_open(&binFile, binFilename, FA_READ);
    if (res != FR_OK) {
        SerialConsoleWriteString("Error opening bin file!\r\n");
        return false;
    }

    SerialConsoleWriteString("Starting firmware update...\r\n");

    // Optionally, you could erase the entire application area here if needed.
    // Simplified: erase each row before writing it.
    while (1) {
        // Read 256 bytes
        memset(buffer, 0xFF, FLASH_ROW_SIZE);
        res = f_read(&binFile, buffer, FLASH_ROW_SIZE, &bytesRead);
        if ((res != FR_OK) || (bytesRead == 0)) {
            // Error occurred or reached the end of the file
            break;
        }

        // Erase the corresponding flash row (one row = 256 bytes)
        enum status_code eraseRet = nvm_erase_row(currentAddr);
        if (eraseRet != STATUS_OK) {
            SerialConsoleWriteString("Error erasing row!\r\n");
            updateSuccess = false;
            break;
        }

        // On SAMD21, write operations are typically performed in pages (64 bytes each)
        // Divide the row into 4 pages and write each page
        for (uint32_t i = 0; i < (FLASH_ROW_SIZE / FLASH_PAGE_SIZE); i++) {
            uint32_t offset = i * FLASH_PAGE_SIZE;
            enum status_code writeRet = nvm_write_buffer(currentAddr + offset, &buffer[offset], FLASH_PAGE_SIZE);
            if (writeRet != STATUS_OK) {
                SerialConsoleWriteString("Error writing page!\r\n");
                updateSuccess = false;
                break;
            }
        }

        if (!updateSuccess) {
            break;
        }

        currentAddr += bytesRead;

        // If the number of bytes read is less than 256, then the end of the file has been reached.
        if (bytesRead < FLASH_ROW_SIZE) {
            break;
        }
    }

    f_close(&binFile);

    if (updateSuccess) {
        SerialConsoleWriteString("Firmware update done!\r\n");
    } else {
        SerialConsoleWriteString("Firmware update failed!\r\n");
    }

    return updateSuccess;
}
