
 
#include <Arduino.h>
#include <TFT_eSPI.h> // Graphics and font library for ST7735 driver chip
#include <SPI.h>
#include <DHT.h>
//#include <DHT_U.h>
#include <Servo.h>

 TFT_eSPI tft = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h
 //Servo myservo;  // 创建舵机对象

 #define TFT_GREY 0xBDF7
 #define DHTPIN 4     // Digital pin connected to the DHT sensor
 #define DHTTYPE DHT11   // DHT 11

 DHT dht(DHTPIN, DHTTYPE);

void setup(void) {
      // 初始化串口用于调试
      Serial.begin(115200);
      dht.begin();
      Serial.println("Initializing...");
}

void loop() {
  static bool initialized = false;
  if (!initialized) {
    // 初始化 TFT 屏幕
    tft.init();
    tft.setRotation(0);
    tft.fillScreen(TFT_GREY);
    tft.setTextColor(TFT_GREEN, TFT_GREY);  // 设置文本颜色和背景颜色
    tft.setTextSize(2);
    initialized = true;
  }

  delay(2000);

  float h = dht.readHumidity();
  float t = dht.readTemperature();
  float f = dht.readTemperature(true);

  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  float hif = dht.computeHeatIndex(f, h);
  float hic = dht.computeHeatIndex(t, h, false);

  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.print(F("%  Temperature: "));
  Serial.print(t);
  Serial.print(F("°C "));
  Serial.print(f);
  Serial.print(F("°F  Heat index: "));
  Serial.print(hic);
  Serial.print(F("°C "));
  Serial.print(hif);
  Serial.println(F("°F"));

  tft.fillScreen(TFT_GREY);
  tft.setCursor(0, 0);
  tft.printf("Humidity: %.2f%%\n", h);
  tft.printf("Temp: %.2f°C\n", t);
  tft.printf("Temp: %.2f°F\n", f);
  tft.printf("Heat index: %.2f°C\n", hic);
  tft.printf("Heat index: %.2f°F\n", hif);
}




