#include "servo.h"

// I2C 扫描函数
void scanI2C() {
    Serial.println("\nScanning I2C bus...");
    for (uint8_t address = 1; address < 127; address++) {
      Wire1.beginTransmission(address);
      if (Wire1.endTransmission() == 0) {
        Serial.print("I2C device found at address 0x");
        Serial.println(address, HEX);
      }
    }
    Serial.println("Scan complete.");
  }