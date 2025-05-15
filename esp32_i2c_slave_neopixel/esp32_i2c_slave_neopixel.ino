/**
 * @file    esp32_i2c_slave_neopixel.ino
 * @brief   ESP32 acts as I2C slave at address 0x28,
 *          controlling a WS2812B (NeoPixel) LED via Adafruit_NeoPixel library.
 *
 * @details
 *   - SDA = GPIO 21, SCL = GPIO 22 (default for many ESP32 boards),
 *     check your board’s pinout or usage. 
 *   - I2C address = 0x28 (7-bit).
 *   - Data command format from master: [cmd, R, G, B]
 *       cmd=1 => set color
 *
 *   Make sure to connect:
 *     - SDA -> master SDA,
 *     - SCL -> master SCL,
 *     - GND -> common GND,
 *     - appropriate pull-ups on SDA/SCL (if not already on board).
 *
 *   The master side (e.g. SAMD21) can send something like:
 *       I2C_Data i2cData = {0x28, [1, 255, 0, 0], 4, NULL, 0};
 *       I2cWriteData(&i2cData);
 *     This will turn the LED red.
 *
 *   Tested with Arduino IDE style environment for ESP32.
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>

// =============== User configuration ===============
#define I2C_SLAVE_ADDR   0x28   // 7-bit I2C address
#define LED_PIN          23     // WS2812B data pin
#define NUM_LEDS         1      // Number of NeoPixel LEDs

// =============== Global objects ===============
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// =============== I2C callback ===============
void onI2CReceive(int numBytes) {
  // We expect at least 4 bytes => [cmd, R, G, B]
  if (Wire.available() >= 4) {
    uint8_t cmd = Wire.read();
    uint8_t r   = Wire.read();
    uint8_t g   = Wire.read();
    uint8_t b   = Wire.read();

    if (cmd == 1) {
      // Set color
      strip.setPixelColor(0, strip.Color(r, g, b));
      strip.show();
      Serial.printf("[I2C] Received cmd=1 => R=%d, G=%d, B=%d, LED updated.\n", r, g, b);
    } else {
      // Unrecognized cmd
      Serial.printf("[I2C] Received unknown cmd=%u\n", cmd);
    }
  } else {
    Serial.printf("[I2C] Not enough data, got %d bytes\n", numBytes);
    while(Wire.available()) {
      Wire.read(); // flush
    }
  }
}

// =============== Setup ===============
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== ESP32 I2C Slave + NeoPixel Demo ===");

  // Initialize NeoPixel
  strip.begin();
  strip.show(); // Turn LED off initially

  // Initialize I2C as slave
  Wire.begin(I2C_SLAVE_ADDR);      // 7-bit address
  Wire.onReceive(onI2CReceive);    // register callback

  Serial.printf("I2C slave started at address 0x%02X\n", I2C_SLAVE_ADDR);
}

// =============== Loop ===============
void loop() {
  // No button logic here, just idle loop
  delay(10);
}
