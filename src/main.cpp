
 
#include <Arduino.h>
#include <TFT_eSPI.h> // Graphics and font library for ST7735 driver chip
#include <SPI.h>
#include <Servo.h>

TFT_eSPI tft = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h

#define TFT_GREY 0xBDF7

void setup(void) {
      // 初始化串口用于调试
      Serial.begin(115200);
      Serial.println("Initializing...");
}

void loop() {

    delay(1000);
    static bool initialized = false;
    if (!initialized) {
      // 初始化 TFT 屏幕
      tft.init();
      tft.setRotation(0);
      tft.fillScreen(TFT_GREY);
      tft.setTextColor(TFT_GREEN, TFT_GREY);  // 设置文本颜色和背景颜色
    
      // 在屏幕中央显示文本
      //tft.drawCentreString("Hello, World!", 64, 64, 4);
      tft.println("Hello, Worldddd!\n");
      
      initialized = true;
      Serial.println("Initialization done.");
    }
  

  // Nothing to do here
}