#include <Arduino.h>
#include "freertos/FreeRTOS.h"    
#include "freertos/task.h"

#include <TFT_eSPI.h> // Graphics and font library for ST7735 driver chip
#include <SPI.h>
#include <simsum20.h>
#include <DHT.h>

#include <usart.h>

 #include <Adafruit_PWMServoDriver.h>
 #include "Wire.h"
#include "servo.h"
#include <FastLED.h>

#include <WiFi.h>
#include <time.h>
#include <esp_sleep.h>
#include "esp_system.h"

#include <PubSubClient.h>
#include <Ticker.h>
#include "AS608.h"
#include "ADC.h"
#include "SIM800A.h"
// ////Wi-Fi 配置
	// const char* ssid = "机器人学院A工作室_5G";
	// const char* password = "JQRXY2022";
	// const char* ssid = "Xiaomi13";
	// const char* password = "07080910";
	const char* ssid = "JGPRO";
	const char* password = "07080910";
	const char* mqttServer = "8.138.46.109";
	// NTP 服务器配置
	const char* ntpServer = "pool.ntp.org";
	const long gmtOffset_sec = 8 * 3600; // 北京时间 GMT+8
	const int daylightOffset_sec = 0;   // 无夏令时
////tft屏幕引脚定义
	TFT_eSPI tft = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h
	#define TFT_GREY 0xBDF7
	#define TFT_BLACK 0x0000
////PCA9685引脚定义
	Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);  // 默认地址为 0x40
	#define SDA_PIN 21  // ESP32 的默认 SDA 引脚
	#define SCL_PIN 20  // ESP32 的默认 SCL 引脚
////温湿度传感器引脚定义
	#define DHTPIN 8     // Digital pin connected to the DHT sensor
	#define DHTTYPE DHT11   // DHT 11
	DHT dht(DHTPIN, DHTTYPE);
//// 定义 RGB 灯的引脚和参数
	#define LED_PIN     48      // 板载 RGB 灯的 GPIO 引脚
	#define NUM_LEDS    1       // 板载 RGB 灯的数量
	#define LED_TYPE    WS2812  // 灯的类型（通常为 WS2812 或 NeoPixel）
	#define COLOR_ORDER GRB     // RGB 灯的颜色顺序
	CRGB leds[NUM_LEDS];        // 定义 LED 数组
////指纹模块引脚定义
	SysPara AS608Para;//指纹模块AS608参数
	// uint16_t ValidN;//模块内有效模板个数
	extern uint8_t ID;//指纹ID
	extern SearchResult seach;
	extern uint8_t OK_; // 录入成功标志位

	//extern uint16_t user_id ; // 用户ID
	//extern uint16_t fingerOK;
////压力传感器引脚定义
	#define SENSOR1_PIN 2  // 出药处压力传感器引脚
	#define SENSOR2_PIN 1  // 药仓处压力传感器引脚	
////服务器
	WiFiClient wifiClient;
	PubSubClient mqttClient(wifiClient);
	Ticker ticker;
	int count;  // Ticker计数用变量
// 定义引脚宏
#define HEATER_PIN GPIO_NUM_41  // 加热片引脚
#define WATER_PIN GPIO_NUM_40    // 水泵引脚
#define COOLER_PIN GPIO_NUM_42  // 制冷引脚
#define SOS_PIN GPIO_NUM_38   // 紧急按键引脚

// ****************************************************
// 注意！以下需要用户根据然也物联平台信息进行修改！否则无法工作!
// ****************************************************
const char* mqttUserName = "123";                                            // 服务端连接用户名(需要修改)
const char* mqttPassword = "8888";                   						 // 服务端连接密码(需要修改)
const char* clientId = "esp32";  											 // 客户端id (需要修改)

const char* subTopic = "medicine";                                           // 订阅主题(需要修改)
const char* subTopic1 = "user";                                         // 订阅主题(需要修改)
const char* subTopic2 = "dose";                                         // 订阅主题(需要修改)
const char* subTopic3 = "time";                                         // 订阅主题(需要修改)

const char* pubTopic = "4G"  ;                                       // 发布主题(需要修改)
const char* pubTopic1 = "humi";                                         // 发布主题(需要修改)
const char* pubTopic2 = "temp";                                         // 发布主题(需要修改)
const char* pubTopic3 = "finger";                                         // 发布主题(需要修改) 1和2
const char* pubTopic4 = "medicine";                                         // 发布主题(需要修改)


const char* willTopic = "die";                                               // 发布主题(需要修改)
// ****************************************************
 
//遗嘱相关信息
const char* willMsg = "esp8266 offline";  // 遗嘱主题信息
const int willQos = 0;                    // 遗嘱QoS
const int willRetain = false;             // 遗嘱保留
 
const int subQoS = 0;             // 客户端订阅主题时使用的QoS级别（截止2020-10-07，仅支持QoS = 1，不支持QoS = 2）
const bool cleanSession = true;  // 清除会话（如QoS>0必须要设为false）
 
bool ledStatus = HIGH;

// extern void connectWifi();
extern void tickerCount();
extern void connectMQTTserver();
extern void receiveCallback(char* topic, byte* payload, unsigned int length);
extern void subscribeTopic();
extern void subscribeTopic_(const char* topic);
extern void pubMQTTmsg();
extern void publishMQTTMessage_(const char* topic, const char* message);
extern void handleUserTopic(byte* payload, unsigned int length) ;
extern void handleTimeTopic(byte* payload, unsigned int length);
extern void handleDoseTopic(byte* payload, unsigned int length);
extern void creatTask();

uint32_t targetHour = 0; // 目标小时
uint32_t targetMinute = 0; // 目标分钟
uint32_t targetSecond = 0; // 目标秒数
#define MIN_WEIGHT_THRESHOLD 12 // 最小重量阈值（克）
#define MAX_TEMP 35.0 // 最大温度阈值（摄氏度）
#define MAX_HUMIDITY 75.0 // 最大湿度阈值（百分比）

uint8_t user1box1Dose,user1box2Dose,user1box3Dose,user1box4Dose = 0; // 用户1药盒的剂量
uint8_t user2box1Dose,user2box2Dose,user2box3Dose,user2box4Dose = 0; // 用户2的剂量
uint8_t user1box5Dose,user2box5Dose =0;	
uint8_t res = 1;
uint8_t JR = 0; // 1表示加热器工作，0表示停止工作
uint8_t ZL = 1; // 1表示制冷器工作，0表示停止工作
uint8_t hum_;
uint8_t temp_;
//匹配成功标志位
// 传感器数据结构体
typedef struct {
    float temperature;
    float humidity;
    uint32_t pressure[4];
} SensorData;	
// 全局同步对象
// EventGroupHandle_t xEventGroup = xEventGroupCreate();

SemaphoreHandle_t xSerialMutex = xSemaphoreCreateMutex();
SemaphoreHandle_t xTftMutex;

// // 定义串口中断回调函数，用于指纹模块的串口接收
// void IRAM_ATTR onSerial1Data() {
// 	//Serial.println("串口中断接收数据");
//     static uint8_t tempBuffer[RXBUFFERSIZE]; // 临时缓冲区
//     static uint16_t tempLen = 0;            // 临时缓冲区长度

//     while (Serial1.available()) {
//         uint8_t data = Serial1.read();

//         // 将数据存入临时缓冲区
//         if (tempLen < RXBUFFERSIZE) {
//             tempBuffer[tempLen++] = data;
//         }

//         // 检查是否接收到完整的数据包（以 0xEF 0x01 开头）
//         if (tempLen >= 9 && tempBuffer[0] == 0xEF && tempBuffer[1] == 0x01) {
//             uint16_t packetLength = (tempBuffer[7] << 8) | tempBuffer[8]; // 包长度字段
//             if (tempLen >= (9 + packetLength)) { // 检查是否接收到完整包
//                 // 将临时缓冲区的数据复制到全局缓冲区
//                 memcpy(aRxBuffer, tempBuffer, 9 + packetLength);
//                 RX_len = 9 + packetLength;
//                 // 清空临时缓冲区
//                 tempLen = 0;
//                 // // 打印接收到的数据（调试用）
//                  Serial.print("串口串口串口的中断Received data: ");
//                  for (int i = 0; i < RX_len; i++) {
//                      Serial.printf("0x%02X ", aRxBuffer[i]);
//                  }
//                  Serial.println();
				
//             }
//         }
//     }
// }



// extern QueueHandle_t xQueue;

// // 定义串口中断回调函数，用于指纹模块的串口接收
// void IRAM_ATTR onSerial1Data() {
//     static uint8_t tempBuffer[RXBUFFERSIZE];
//     static uint16_t tempLen = 0;
//     BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//     while (Serial1.available()) {
//         uint8_t data = Serial1.read();
//         if (tempLen < RXBUFFERSIZE) {
//             tempBuffer[tempLen++] = data;
//         }
// 		// 包头同步：如果不是0xEF 0x01开头，丢弃前面数据
//         while (tempLen >= 2 && !(tempBuffer[0] == 0xEF && tempBuffer[1] == 0x01)) {
//             // 左移一位
//             memmove(tempBuffer, tempBuffer + 1, --tempLen);
//         }
//         // 检查是否接收到完整的数据包（以 0xEF 0x01 开头）
//         if (tempLen >= 9 && tempBuffer[0] == 0xEF && tempBuffer[1] == 0x01) {
//             uint16_t packetLength = (tempBuffer[7] << 8) | tempBuffer[8];
//             if (tempLen >= (9 + packetLength)) {
//                 // 将完整包通过队列通知任务处理（这里只传递长度，数据用全局缓冲区）
//                 memcpy(aRxBuffer, tempBuffer, 9 + packetLength);
//                 RX_len = 9 + packetLength;
//                 tempLen = 0;
// 				                if (tempLen > 0) {
//                     memmove(tempBuffer, tempBuffer + RX_len, tempLen);
//                 }
//                 // 通知任务有新包到达
//                 xQueueSendFromISR(xQueue, &RX_len, &xHigherPriorityTaskWoken);
//             }
//         }
//     }
//     portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
// }
// // 任务中处理完整包
// void processSerialDataTask(void *pvParameters) {
//     uint8_t packetLen;
//     while (1) {
//         if (xQueueReceive(xQueue, &packetLen, portMAX_DELAY)) {
//             Serial.print("处理接收到的完整包: ");
//             for (int i = 0; i < packetLen; i++) {
//                 Serial.printf("0x%02X ", aRxBuffer[i]);
//             }
//             Serial.println();
//             // 这里可以调用你的指纹包解析函数
//         }
//     }
// }



//tft屏幕打印函数
void tft_printf(int line, int column, const char* format, ...) {
	tft.fillScreen(TFT_BLACK); // 填充背景色
	tft.setCursor(column, line); // 设置光标位置
	//va_list args;
	//va_start(args, format);
	tft.printf(format); // 使用 vprintf 处理可变参数
	//va_end(args);
}
// //tft屏幕打印函数，，横屏
// void tft_printf(int line, int column, const char* format, ...) {
// 	tft.fillScreen(TFT_BLACK); // 填充背景色
// 	tft.setRotation(1); // 设置屏幕为横屏模式
// 	tft.setCursor(column, line); // 设置光标位置
// 	tft.printf(format); // 使用 printf 处理可变参数
// }
void displayOnTFT(const char* content, int x, int y, int width, int height) {
    if (xSemaphoreTake(xTftMutex, portMAX_DELAY)) {
        tft.fillRect(x, y, width, height, TFT_BLACK); // 填充背景色
        tft.setCursor(x, y); // 设置光标位置
        tft.printf(content); // 显示内容
        xSemaphoreGive(xTftMutex);
    }
}

void setGPIOHigh(int pin) {
	pinMode(pin, OUTPUT); // 设置引脚为输出模式
	digitalWrite(pin, HIGH); // 设置引脚为高电平
}
void setGPIOLow(int pin) {
	pinMode(pin, OUTPUT); // 设置引脚为输出模式
	digitalWrite(pin, LOW); // 设置引脚为低电平
}

TaskHandle_t Message_TaskHandle;
TaskHandle_t connect_taskHandle;
TaskHandle_t DisplayTime_TaskHandle;

TaskHandle_t Medicine_TaskHandle;
TaskHandle_t Environment_TaskHandle;
TaskHandle_t Emergency_TaskHandle;

void connect_task(void *pvParam) {
	while (1) {
		if (!mqttClient.connected()) {
			Serial.println("MQTT连接断开，尝试重新连接...");
			connectMQTTserver();
		  }
		  else
		  	Serial.println("MQTT正常连接");
		  if (count == 3) {
			//pubMQTTmsg();  // 每隔3秒钟发布一次信息
			count = 0;
		  }
		  // 处理信息以及心跳
		  mqttClient.loop();
		vTaskDelay(pdMS_TO_TICKS(1000)); // 每秒检查一次连接状态
	}
}

void DisplayTime_Task(void *pvParam) {
	while (1) {
		// 获取当前时间
		struct tm timeinfo;
		if (!getLocalTime(&timeinfo)) {
			Serial.println("无法获取时间");
			WiFi.disconnect();
			WiFi.begin(ssid, password);
			configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
			vTaskDelay(pdMS_TO_TICKS(1000));
			continue;
		}
		if(xSemaphoreTake(xTftMutex, portMAX_DELAY)) {
			tft.fillRect(0, 1, tft.width(), 20, TFT_BLACK); // 清除顶部区域
			tft.setCursor(0, 1); // 设置光标到屏幕顶部
			tft.printf("      %02d:%02d:%02d\n",
				timeinfo.tm_hour,
				timeinfo.tm_min,
				timeinfo.tm_sec);
			xSemaphoreGive(xTftMutex);
		}
		vTaskDelay(pdMS_TO_TICKS(1000)); // 每秒更新一次
		if (JR == 1)
		{
			displayOnTFT("热水:ON", 0, 96, tft.width(), 16); // 只清除并填充第三行
		}
		else
		{
			displayOnTFT("热水:OFF", 0, 96, tft.width(), 16); // 只清除并填充第二行
		}
		if(ZL == 1)
		{
			displayOnTFT("制冷:ON", 0, 112, tft.width(), 16); // 只清除并填充第三行
		}
		else
		{
			displayOnTFT("制冷:OFF", 0, 112, tft.width(), 16); // 只清除并填充第三行
		}
		//displayOnTFT("下次吃药时间", 0, 112, tft.width(), 16); // 只清除并填充第四行
		if (xSemaphoreTake(xTftMutex, portMAX_DELAY)) {
			tft.fillRect(0, 20, tft.width(), 16, TFT_BLACK); // 清除第二行
			tft.setCursor(0,20 ); // 设置光标到第二行
			tft.printf("吃药时间%02d:%02d:%02d", targetHour, targetMinute, targetSecond); // 显示下次吃药时间
			xSemaphoreGive(xTftMutex);
		}
	}
}

void message_task(void *pvParam) {
		//displayOnTFT("进行信息录入", 0, 16, tft.width(), 16); // 只清除并填充第二行
		
		Serial.println("进行信息录入");
		vTaskSuspend(connect_taskHandle); // 暂停连接任务
		vTaskSuspend(DisplayTime_TaskHandle); // 暂停时间显示任务
		vTaskSuspend(Medicine_TaskHandle); // 暂停吃药任务
		vTaskSuspend(Environment_TaskHandle); // 暂停环境监测任务
		vTaskSuspend(Emergency_TaskHandle); // 暂停紧急任务

		if(xSemaphoreTake(xTftMutex, portMAX_DELAY)) {
			tft.fillRect(0, 0, tft.width(), 48, TFT_BLACK); // 清除第一、二、三行区域
			tft.setCursor(0, 1); // 设置光标到第一行
			tft.printf("   进行指纹录入");
			xSemaphoreGive(xTftMutex);
		}
		while (1)
		{
			Add_FR(); // 录入指纹
			if (OK_ == 1)
			{
				// if(xSemaphoreTake(xTftMutex, portMAX_DELAY)) {
				// 	tft.fillRect(0, 1, tft.width(), 20, TFT_BLACK); // 清除顶部区域
				// 	tft.setCursor(0, 1); // 设置光标到屏幕顶部
				// 	tft.printf(" 指纹录入成功\n");
				// 	xSemaphoreGive(xTftMutex);
				// }
				break; // 录入成功，跳出循环
			}
			
		}
		
		vTaskResume(connect_taskHandle); // 恢复连接任务
		vTaskResume(DisplayTime_TaskHandle); // 恢复时间显示任务
		vTaskResume(Medicine_TaskHandle); // 恢复吃药任务
		vTaskResume(Environment_TaskHandle); // 恢复环境监测任务
		vTaskResume(Emergency_TaskHandle); // 恢复紧急任务

		vTaskDelay(pdMS_TO_TICKS(1000)); 
		JR = 0; // 重置加热器状态
		vTaskDelete(NULL);
	//press_FR(); // 刷指纹
}

uint16_t peopleNum = 0; // 初始化人数为0
uint16_t finished = 0;
uint8_t CY_1 = 0; 
void Medicine_task(void *pvParam) {/// 到达吃药时间，语音提醒任务
	static int lastExecutedSecond = -1; // 记录上一次执行的秒数
	while (1) {
		// 获取当前时间
		struct tm timeinfo;
		if (!getLocalTime(&timeinfo)) {
			Serial.println("无法获取时间");
			vTaskDelay(pdMS_TO_TICKS(1000));
			continue;
		}

		uint16_t time_current = timeinfo.tm_hour * 3600 + timeinfo.tm_min * 60 + timeinfo.tm_sec; // 当前时间的秒数
		uint16_t time_target = targetHour * 3600 + targetMinute * 60 + targetSecond; // 目标时间的秒数

		if (time_current == time_target - 20) {// 检查是否到达服药时间前20秒
			Serial.println("到达吃药时间前20秒,触发加热");
			setGPIOHigh(HEATER_PIN); // 设置加热引脚为高电平
			JR = 1;
		}
		
		if (time_current == time_target - 10) {// 检查是否到达服药时间前10秒
			Serial.println("到达吃药时间前10秒,停止加热，触发抽水");
			setGPIOLow(HEATER_PIN); // 设置加热引脚为低电平
			setGPIOHigh(WATER_PIN); // 设置制冷引脚为高电平
			vTaskDelay(pdMS_TO_TICKS(9000));
			setGPIOLow(WATER_PIN); // 设置制冷引脚为低电平
		}

		if (timeinfo.tm_hour == targetHour &&// 检查是否到达服药时间
			timeinfo.tm_min == targetMinute &&
			timeinfo.tm_sec == targetSecond ){
			//&&
			// lastExecutedSecond != timeinfo.tm_sec) {
			// lastExecutedSecond = timeinfo.tm_sec; // 更新上一次执行的秒数
			Serial.println("到达吃药时间");

			setGPIOLow(GPIO_NUM_4);
			finished = 0; // 重置人数
			if (xSemaphoreTake(xTftMutex, portMAX_DELAY)) {
				tft.fillRect(0, 0, tft.width(), tft.height() - 60, TFT_BLACK); // 填充背景色，保留底部三行
				tft.setCursor(0, 0); // 设置光标位置
				tft.printf("到达吃药时间 "); // 显示提示信息
				xSemaphoreGive(xTftMutex);
			}
			vTaskDelay(pdMS_TO_TICKS(300)); // 延时

			vTaskSuspend(connect_taskHandle); // 暂停连接任务
			vTaskSuspend(DisplayTime_TaskHandle); // 暂停时间显示任务
			vTaskSuspend(Environment_TaskHandle); // 暂停环境监测任务
			vTaskSuspend(Emergency_TaskHandle); // 暂停紧急任务

			bool user1Done = false, user2Done = false;
			    		if (xSemaphoreTake(xTftMutex, portMAX_DELAY)) {
							tft.fillRect(0, 20, tft.width(), 16, TFT_BLACK); // 清除第二行
							tft.setCursor(0,20 ); // 设置光标到第二行
							tft.printf("请刷指纹"); // 显示下次吃药时间
							xSemaphoreGive(xTftMutex);
							}
				Serial.println("请刷指纹");
			while (finished < peopleNum) {
				Serial.printf("finished = %d\n", finished);

				uint8_t a = press_FR(); // 刷指纹

				if ((a == 1) && (seach.pageID+1 == 1) && !user1Done) {
					Serial.println("用户1匹配成功,投药动作");
					int maxDose = max(user1box1Dose, max(user1box2Dose, user1box5Dose));
					for (int i = 0; i < maxDose; i++) {
						if (i < user1box1Dose) {
							pwm.setPWM(2, 0, 200);
							Serial.println("药盒1转动");
							vTaskDelay(pdMS_TO_TICKS(640));
						} else {
							pwm.setPWM(2, 0, 280);
						}
						if (i < user1box2Dose) {					
							pwm.setPWM(3, 0, 200);
							Serial.println("药盒2转动");
							vTaskDelay(pdMS_TO_TICKS(640));
						} else {
							pwm.setPWM(3, 0, 280);
						}
						if (i < user1box5Dose) {
							if (CY_1 == 1)
							{
								vTaskDelay(pdMS_TO_TICKS(400));
							}
							pwm.setPWM(0, 0, 155);
							pwm.setPWM(1, 0, 430);
							Serial.println("药仓转动");
							vTaskDelay(pdMS_TO_TICKS(640));
							CY_1 = 1;
							pwm.setPWM(0, 0, 280);
							pwm.setPWM(1, 0, 280);
						} else {
							pwm.setPWM(0, 0, 280);
							pwm.setPWM(1, 0, 280);
						}
					}
					pwm.setPWM(2, 0, 280);
					pwm.setPWM(3, 0, 280);
					pwm.setPWM(0, 0, 280);
					pwm.setPWM(1, 0, 280);
					user1Done = true;
					finished+=1;
					Serial.println("用户1投药完成");
					vTaskDelay(pdMS_TO_TICKS(500));
					continue;
				} 
				else if ((a == 1) && (seach.pageID+1 == 2) && !user2Done) {
					Serial.println("用户2匹配成功,投药动作");
					int maxDose = max(user2box1Dose, max(user2box2Dose, user2box5Dose));
					for (int i = 0; i < maxDose; i++) {
						if (i < user2box1Dose) {
							pwm.setPWM(2, 0, 200);
							Serial.println("药盒1转动");
							vTaskDelay(pdMS_TO_TICKS(980));
						} else {
							pwm.setPWM(2, 0, 280);
						}
						if (i < user2box2Dose) {
							pwm.setPWM(3, 0, 200);
							vTaskDelay(pdMS_TO_TICKS(980));
						} else {
							pwm.setPWM(3, 0, 280);
						}
						if (i < user2box5Dose) {
							pwm.setPWM(0, 0, 170);
							pwm.setPWM(1, 0, 410);
							Serial.println("药仓转动");
							vTaskDelay(pdMS_TO_TICKS(640));
						} else {
							pwm.setPWM(0, 0, 280);
							pwm.setPWM(1, 0, 280);
						}
					}
					pwm.setPWM(2, 0, 280);
					pwm.setPWM(3, 0, 280);
					pwm.setPWM(0, 0, 280);
					pwm.setPWM(1, 0, 280);
					user2Done = true;
					finished  +=1;
					Serial.println("用户2投药完成");
					vTaskDelay(pdMS_TO_TICKS(500));
					continue;
				} else {
					Serial.println("请重新刷指纹");
					vTaskDelay(pdMS_TO_TICKS(1000));
				}
			}
			setGPIOHigh(GPIO_NUM_4); // 设置引脚为低电平

			vTaskResume(connect_taskHandle); // 恢复连接任务
			vTaskResume(DisplayTime_TaskHandle); // 恢复时间显示任务
			vTaskResume(Environment_TaskHandle); // 恢复环境监测任务
			vTaskResume(Emergency_TaskHandle); // 恢复紧急任务

			vTaskDelay(pdMS_TO_TICKS(1000)); // 延时
		}
	}
}

//检测药物余量任务
void Environment_Task(void *pvParam) {
    while (1) {

        float temp = dht.readTemperature();// 采集温湿度数据
        float hum = dht.readHumidity(); 
		// if(hum > 60)
		// {
		// 	setGPIOHigh(COOLER_PIN); // 设置制冷引脚为高电平
		// }
		// else
		// {
		// 	setGPIOLow(COOLER_PIN); // 设置制冷引脚为低电平
		// }        
        long pressure1 = getPressValue(SENSOR1_PIN);// 检查药物存储仓的压力传感器值
        long pressure2 = getPressValue(SENSOR2_PIN);
		if (xSemaphoreTake(xTftMutex, portMAX_DELAY)) {
			tft.fillRect(0, tft.height() - 80, tft.width(), 40, TFT_BLACK); // 填充底部倒数第三行背景色
			tft.setCursor(0, tft.height() - 80); // 设置光标到倒数第三行
			if (temp_ =1)
			{
				tft.printf("温度: %.2f°C", temp); // 显示温度数据
				tft.setCursor(0, tft.height() - 60); // 设置光标到倒数第二行

			}
			else
			{
				tft.printf("温度: %.2f°C  不宜", temp); // 显示温度数据
				tft.setCursor(0, tft.height() - 60); // 设置光标到倒数第二行
			}
			if(hum_ =1)
			{
				tft.printf("湿度: %.2f%%", hum); // 显示湿度数据
			}
			else
			{
				tft.printf("湿度: %.2f%%  不宜", hum); // 显示湿度数据
			}
			
			xSemaphoreGive(xTftMutex);
		}
		///Serial.printf("温度: %.2f°C 湿度: %.2f%%\n", temp, hum); // 打印到串口
		//  Serial.printf("出药处压力传感器: %ld g\n", pressure1); // 打印到串口
		// Serial.printf("药包处压力传感器: %ld g\n", pressure2); // 打印到串口

		char messageBuffer[64];
		snprintf(messageBuffer, sizeof(messageBuffer), "%.2f", hum);// 发布湿度数据
		publishMQTTMessage_(pubTopic1, messageBuffer);

		char messageBuffer2[64];
		snprintf(messageBuffer2, sizeof(messageBuffer2), "%.2f", temp);		// 发布温度数据
		publishMQTTMessage_(pubTopic2, messageBuffer2);
		
		char messageBuffer3[64];
        // 判断药物余量是否不足
        if ( pressure2 < MIN_WEIGHT_THRESHOLD) {
            //Serial.println("药物余量不足，请及时补充");
			snprintf(messageBuffer3, sizeof(messageBuffer3), "0"); // 发布药物余量不足消息
			publishMQTTMessage_(pubTopic4, messageBuffer3);
        }
		else
		{
			snprintf(messageBuffer3, sizeof(messageBuffer3), "1"); // 发布药物余量充足消息
			publishMQTTMessage_(pubTopic4, messageBuffer3);
		}

        // 提示用户摆放药盒
        if ( hum > MAX_HUMIDITY) {
			hum_ = 0; // 重置湿度值
            Serial.println("请将药盒放置在适宜环境中");
			if (xSemaphoreTake(xTftMutex, portMAX_DELAY)) {
				// tft.fillRect(0, tft.height() - 60, tft.width(), 40, TFT_BLACK); // 填充底部倒数第二和倒数第三行背景色
				// tft.setCursor(0, tft.height() - 60); // 设置光标到倒数第三行
				// tft.printf("温度: %.2f°C", temp); // 显示温度数据
				// tft.setCursor(0, tft.height() - 40); // 设置光标到倒数第二行
				// tft.printf("湿度: %.2f%%", hum); // 显示湿度数据
				// xSemaphoreGive(xTftMutex);
				
				tft.fillRect(0, tft.height() - 80, tft.width(), 40, TFT_BLACK); // 填充底部倒数第三行背景色
				tft.setCursor(0, tft.height() - 80); // 设置光标到倒数第三行
				tft.printf("温度: %.2f°C", temp); // 显示温度数据
				tft.setCursor(0, tft.height() - 60); // 设置光标到倒数第二行
				tft.printf("湿度: %.2f%%", hum); // 显示湿度数据
				xSemaphoreGive(xTftMutex);
				
			}
			snprintf(messageBuffer, sizeof(messageBuffer), "0"); // 发布药物余量不足消息
			publishMQTTMessage_(pubTopic1, messageBuffer);
		}
		if (temp > MAX_TEMP) {
			temp_ = 0; // 重置温度值
			Serial.println("温度过高，请将药盒放置在适宜环境中");
			//snprintf(messageBuffer3, sizeof(messageBuffer3), "{\"0\"}"); // 发布温度过高消息
			snprintf(messageBuffer2, sizeof(messageBuffer2), "0"); // 发布温度过高消息
			publishMQTTMessage_(pubTopic2, messageBuffer2);
		}

        vTaskDelay(pdMS_TO_TICKS(5000)); // 每5秒更新一次
    }
}

//紧急任务
void Emergency_Task(void *pvParam) {
	while (1) {
		if (digitalRead(SOS_PIN) == HIGH) { // 检查按键是否被按下
			Serial.println("紧急按键被按下，拨打电话！");
			while (1) {
				res = SIM_MAKE_CALL("18125942327");
				
				if (res == 0) {
					Serial.println("拨打电话成功！");
					break; // 成功拨打电话，退出循环
				} else {
					Serial.println("拨打电话失败，正在重试...");
				}
			}
		}
		vTaskDelay(pdMS_TO_TICKS(100)); // 延时以避免频繁检测
	}
}

	QueueHandle_t xQueue;
void setup(void) {


	xQueue = xQueueCreate(10, sizeof(uint8_t));
    if (xQueue == NULL) {
        Serial.println("Queue creation failed!");
        while (1);
    }
	pinMode(SOS_PIN, INPUT_PULLUP); // 设置引脚42为输入模式

	setGPIOLow(HEATER_PIN); // 设置加热器引脚为低电平	
	setGPIOLow(WATER_PIN); // 设置水泵引脚为低电平
	setGPIOHigh(COOLER_PIN); // 设置制冷引脚为低电平
	setGPIOHigh(GPIO_NUM_4); 
////////初始化TFT屏幕
	tft.init();
	tft.setRotation(0); // 设置屏幕方向为短边
	tft.loadFont(simsum20); //指定tft屏幕对象载入font_12字库
	//tft.unloadFont(); //释放字库文件,节省资源
	tft.fillScreen(TFT_BLACK); // 填充背景色
	tft.setRotation(1); // 设置屏幕为横屏模式
	tft.setTextColor(TFT_WHITE, TFT_BLACK); // 设置文字颜色和背景色
	// tft.setTextSize(2); // 设置文字大小为2
	// tft.setCursor(0, 0); // 设置光标位置
	// tft.printf("初始化中 ...\n");
	
//////// 初始化串口用于调试
    Serial.begin(115200);
    Serial.println("Initializing Serial ...");
///////初始化互斥锁
	xTftMutex = xSemaphoreCreateMutex();
	if (xTftMutex == NULL) {
		Serial.println("TFT互斥锁创建失败!");
		while (1); // 停止程序
	}
//初始化DHT传感器
	dht.begin();
/////// 指纹模块的串口初始化。Serial1
    Serial1.begin(57600, SERIAL_8N1, 18, 17);  //RX=GPIO18，TX=GPIO17
	// Serial1.onReceive(onSerial1Data); // 注册中断回调函数
	Serial.println("指纹模块串口初始化成功.");
	GZ_Empty(); // 清空指纹模块的缓存
	//tft_printf(0, 0, "指纹模块初始化成功.");
/////// gsm模块的串口初始化，Serial2 
	Serial2.begin(115200, SERIAL_8N1, 13, 14);	//RX=GPIO13，TX=GPIO14 
	// uint8_t res = 1;
	res = GSM_Dect();
 
//////压力传感器引脚初始化
	analogSetPinAttenuation(SENSOR1_PIN, ADC_11db);
	analogSetPinAttenuation(SENSOR2_PIN, ADC_11db);
/////// 初始化PCA9685的 I2C 总线
	if (!Wire1.begin(SDA_PIN, SCL_PIN)) {
		Serial.println("初始化I2C失败,请检查 SDA 和 SCL 连接。");
		while (1);  // 停止程序
	}
	Serial.println("I2C初始化成功.");
	// tft.setCursor(0, 32); // 设置光标位置
	// tft.printf(" I2C初始化成功.");
	// 扫描 I2C 总线
	/////////////////////////////////////////////////////////////////scanI2C();
	// 初始化 PCA9685
	pwm.begin();
	pwm.write8(0x00, 0x01);  // 清除 SLEEP 位，启用 PWM 输出
	pwm.setPWMFreq(50);  // 设置 PWM 频率为 50Hz
	Serial.println("PCA9685已设置PWM频率为50HZ .");
	Serial.println("PCA9685初始化成功.");
	// tft.fillScreen(TFT_BLACK); // 填充背景色
	//  tft.setCursor(0, 48); // 设置光标位置
	//  tft.printf(" PCA9685初始化成功.");
	//tft.printf(2, 0, "PCA9685初始化成功.");
		// // 检查寄存器
		// uint8_t mode1 = pwm.read8(0x00);  // MODE1 寄存器
		// uint8_t prescale = pwm.read8(0xFE);  // PRESCALE 寄存器
		// Serial.print("MODE1 register: 0x");
		// Serial.println(mode1, HEX);
		// Serial.print("PRESCALE register: 0x");
		// Serial.println(prescale, HEX);
////////// 连接 Wi-Fi，设置时间
    WiFi.begin(ssid, password);
    Serial.print("正在连接 Wi-Fi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWi-Fi 已连接");
	// 配置 NTP
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
	// 配置时间
	struct tm timeinfo;
	///////如果需要收到设置时间
	// timeinfo.tm_year = 2025 - 1900 ; // 年份从 1900 开始
	// timeinfo.tm_mon = 4 - 1 ;            // 4 月（从 0 开始计数）
	// timeinfo.tm_mday = 22;          // 日期
	// timeinfo.tm_hour = 12;          // 小时
	// timeinfo.tm_min = 0;           // 分钟
	// timeinfo.tm_sec = 0;            // 秒
	// time_t t = mktime(&timeinfo);
	// struct timeval now = { .tv_sec = t };
	// 	Serial.printf("设置的时间戳: %ld\n", t);
	//settimeofday(&now, NULL); // 设置系统时间
	Serial.println("RTC 已初始化！");


	ticker.attach(1, tickerCount);  // Ticker定时对象
	//设置ESP8266工作模式为无线终端模式
	WiFi.mode(WIFI_STA);
	// 设置MQTT服务器和端口号
	mqttClient.setServer(mqttServer, 1883);
	mqttClient.setCallback(receiveCallback);
	// 连接MQTT服务器
	connectMQTTserver();
	// char* pubMessage;
	// pubMessage = "{\"heart\":\"1\"}";

	// // 实现ESP8266向主题发布信息
	// if (mqttClient.publish(pubTopic, pubMessage)) {
	//   Serial.println("Publish Topic:");
	//   Serial.println(pubTopic);
	//   Serial.println(pubMessage);
	// } else {
	//   Serial.println("Message Publish Failed.");connect_task
	// }

	  xTaskCreate(connect_task, "TickerTask", 4096, NULL, 2, &connect_taskHandle); // 创建 Ticker 任务
	xTaskCreate(DisplayTime_Task, "DisplayTimeTask", 2048, NULL, 1, &DisplayTime_TaskHandle);
	  xTaskCreate(Environment_Task, "EnvironmentTask", 4096, NULL, 2, &Environment_TaskHandle);
      xTaskCreate(Medicine_task, "ReminderTask", 4096, NULL, 3, &Medicine_TaskHandle);
	xTaskCreate(Emergency_Task, "EmergencyTask", 4096, NULL, 2, &Emergency_TaskHandle); 

}


void loop() {

}


// 计时器
void tickerCount() {
	count++;
  }
 
void connectMQTTserver() { // 连接MQTT服务器并订阅信息
	// 根据ESP8266的MAC地址生成客户端ID（避免与其它ESP8266的客户端ID重名）
   
	/* 连接MQTT服务器
	boolean connect(const char* id, const char* user, 
					const char* pass, const char* willTopic, 
					uint8_t willQos, boolean willRetain, 
					const char* willMessage, boolean cleanSession); 
	若让设备在离线时仍然能够让qos1工作，则connect时的cleanSession需要设置为false                
					*/
	if (mqttClient.connect(clientId, mqttUserName,
						   mqttPassword, willTopic,
						   willQos, willRetain, willMsg, cleanSession)) {
	//   Serial.print("MQTT Server Connected. ClientId: ");
	//   Serial.println(clientId);
	//   Serial.print("MQTT Server: ");
	//   Serial.println(mqttServer);
	//   Serial.print("SubTopic: ");                                                                                                                                                                 
	  //subscribeTopic();  // 订阅指定主题
	  subscribeTopic_(subTopic1); // 订阅用户信息话题
	  subscribeTopic_(subTopic2); // 订阅用户信息话题
	  subscribeTopic_(subTopic3); // 订阅用户信息话题
	} else {
	//   Serial.print("MQTT Server Connect Failed. Client State:");
	//   Serial.println(mqttClient.state());
	  //delay(5000);
	  vTaskDelay(pdMS_TO_TICKS(5000)); // 延时5秒后重试连接
	}
  }
   
  // 收到信息后的回调函数
void receiveCallback(char* topic, byte* payload, unsigned int length) {
	// Serial.print("Message Received [");
	// Serial.print(topic);
	// Serial.print("] ");
	// for (int i = 0; i < length; i++) {
	//   Serial.print((char)payload[i]);
	// }
	// Serial.println("");
	// Serial.print("Message Length(Bytes) ");
	// Serial.println(length);
	// if ((char)payload[0] == '1') {  // 如果收到的信息以“1”为开始
	//   ledStatus = LOW;
	// } else {
	//   ledStatus = HIGH;
	// }
	// pubMQTTmsg();

	// Serial.printf("Message Received [%s]: ", topic);
    // for (unsigned int i = 0; i < length; i++) {
    //     Serial.print((char)payload[i]);
    // }
    // Serial.println();
	if (strcmp(topic, "user") == 0) {
        // 处理 "user" 主题的消息
        handleUserTopic(payload, length);
    } else if (strcmp(topic, "dose") == 0) {
        // 处理 "dose" 主题的消息
        handleDoseTopic(payload, length);
    } else if (strcmp(topic, "time") == 0) {
        // 处理 "time" 主题的消息
        handleTimeTopic(payload, length);
    } else {
		// Serial.printf("Topic: %s, Message: ", topic);
		// for (unsigned int i = 0; i < length; i++) {
		// 	Serial.print((char)payload[i]);
		// }
		// Serial.println();
        Serial.println("Unknown topic received.");
    }

  }

// 处理 "user" 主题的消息
void handleUserTopic(byte* payload, unsigned int length) {
	static bool user1Handled = false; // 标志位，确保用户1的操作只执行一次
	static bool user2Handled = false; // 标志位，确保用户2的操作只执行一次
	static unsigned long startMillis = 0;
	static bool startTimeSet = false;

	Serial.println("处理 'user' 话题...");
	// 将 payload 转换为字符串
	char message[length + 1];
	memcpy(message, payload, length);
	message[length] = '\0';

	// 记录上电时间
	if (!startTimeSet) {
		startMillis = millis();
		startTimeSet = true;
	}

	// 上电30秒内不处理user话题
	if (millis() - startMillis < 5000) {
		Serial.println("millss: ");
		Serial.println(millis());
		Serial.println("startMillis: ");
		Serial.println(startMillis);
		Serial.println("上电后10秒内,忽略'user'话题消息。");
		return;
	}

	if (strcmp(message, "1") == 0) {
		if (!user1Handled) { // 检查是否已经处理过用户1
			Serial.println("接收到'1': 用户1...");
			user1Handled = true; // 设置标志位，防止重复执行
			currentUser = 1; // 设置当前用户为1
			creatTask(); // 创建任务
			peopleNum +=1; // 增加人数
		} 
	} 
	else if (strcmp(message, "2") == 0) {
		if (!user2Handled) { // 检查是否已经处理过用户2
			Serial.println("接收到'2': 用户2...");
			user2Handled = true; // 设置标志位，防止重复执行
			currentUser = 2; // 设置当前用户为2
			creatTask(); // 创建任务
			peopleNum +=1; // 增加人数
		} 
	} 
	else {
		Serial.printf("Unknown command for 'user' topic: %s\n", message);
	}
}
// 处理 "dose" 主题的消息
void handleDoseTopic(byte* payload, unsigned int length) {
	 Serial.println("处理 'dose' 话题...");
	// Serial.print("Received 'dose' topic, message: ");
	// for (unsigned int i = 0; i < length; i++) {
	// 	Serial.print((char)payload[i]);
	// }
	// Serial.println();
	char message[length + 1]; // 将 payload 转换为字符串
	memcpy(message, payload, length);
	message[length] = '\0';

	int userNumber = 0, boxNumber = 0, doseValue = 0; // 解析数据格式 "用户:药盒:药量" 或 "用户:药仓:药量"
	if (sscanf(message, "%d:%d:%d", &userNumber, &boxNumber, &doseValue) == 3) {
		//Serial.printf("Received user: %d, box number: %d, dose value: %d\n", userNumber, boxNumber, doseValue);

		if (userNumber == 1) { // 用户1
			if (boxNumber >= 1 && boxNumber <= 4) { // 药盒
				Serial.printf("用户1的药盒 %d 的药量为 %d.\n", boxNumber, doseValue);
				if (boxNumber == 1) user1box1Dose = doseValue;
				else if (boxNumber == 2) user1box2Dose = doseValue;
				else if (boxNumber == 3) user1box3Dose = doseValue;
				else if (boxNumber == 4) user1box4Dose = doseValue;
			} else if (boxNumber == 5) { // 药仓
				Serial.printf("用户1的药仓用量为 %d.\n", doseValue);
				Serial.println();
				user1box5Dose = doseValue;
			} else {
				Serial.println("Invalid box number received for user 1.");
			}
		} else if (userNumber == 2) { // 用户2
			if (boxNumber >= 1 && boxNumber <= 4) { // 药盒
				Serial.printf("用户2的药盒 %d 的药量为 %d.\n", boxNumber, doseValue);
				Serial.println();
				if (boxNumber == 1) user2box1Dose = doseValue;
				else if (boxNumber == 2) user2box2Dose = doseValue;
				else if (boxNumber == 3) user2box3Dose = doseValue;
				else if (boxNumber == 4) user2box4Dose = doseValue;
			} else if (boxNumber == 5) { // 药仓
				Serial.printf("用户2的药仓用量为 %d.\n", doseValue);
				user2box5Dose = doseValue;
			} else {
				Serial.println("Invalid box number received for user 2.");
			}
		} else {
			Serial.println("Invalid user number received.");
		}
	} else {
		Serial.println("Invalid dose format received. Expected format: 用户:药盒:药量 或 用户:药仓:药量");
	}
}

// 处理 "time" 主题的消息
void handleTimeTopic(byte* payload, unsigned int length) {
    Serial.println("处理 'time' 话题...");
	Serial.print("Received 'time' topic, message: ");
	for (unsigned int i = 0; i < length; i++) {
		Serial.print((char)payload[i]);
	}
    // 将 payload 转换为字符串
    char message[length + 1];
    memcpy(message, payload, length);
    message[length] = '\0';

    // 示例：解析时间并设置目标时间
    int hour, minute, second;
    if (sscanf(message, "%d:%d:%d", &hour, &minute, &second) == 3) {
        targetHour = hour;
        targetMinute = minute;
        targetSecond = second;
        Serial.printf(" 吃药时间设置为 %02d:%02d:%02d\n", targetHour, targetMinute, targetSecond);
    } else {
        Serial.println(" Invalid time format. Expected format: HH:MM:SS");
    }
}

  // 订阅指定主题
void subscribeTopic() {
   
	// 通过串口监视器输出是否成功订阅主题以及订阅的主题名称
	// 请注意subscribe函数第二个参数数字为QoS级别。这里为QoS = 1
	if (mqttClient.subscribe(subTopic, subQoS)) {
	  Serial.print("Subscribed Topic: ");
	  Serial.println(subTopic);
	} else {
	  Serial.print("Subscribe Fail...");
	}
   
  }
   

// 订阅指定主题
void subscribeTopic_(const char* topic) {
	// 通过串口监视器输出是否成功订阅主题以及订阅的主题名称
	// 请注意subscribe函数第二个参数数字为QoS级别。这里为QoS = 1
	if (mqttClient.subscribe(topic, subQoS)) {
		Serial.print("Subscribed Topic: ");
		Serial.println(topic);
	} else {
		Serial.print("Subscribe Fail for Topic: ");
		Serial.println(topic);
	}
}
  
// 发布信息
void pubMQTTmsg() {
	const char* pubMessage;
   
	pubMessage = "{\"heart\":\"1\"}";
	// char messageBuffer[128];
	// snprintf(messageBuffer, sizeof(messageBuffer), 
	// 		 "{\"temperature\":%.2f,\"humidity\":%.2f,\"pressure1\":%ld,\"pressure2\":%ld}", 
	// 		 dht.readTemperature(), dht.readHumidity(), 
	// 		 getPressValue(SENSOR1_PIN), getPressValue(SENSOR2_PIN));
	// pubMessage = messageBuffer;
   
	// 实现ESP8266向主题发布信息
	if (mqttClient.publish(pubTopic, pubMessage)) {
	//   Serial.println("Publish Topic:");
	//   Serial.println(pubTopic);
	//   Serial.println(pubMessage);
	} else {
	  //Serial.println("Message Publish Failed.");
	}
  }

void publishMQTTMessage_(const char* topic, const char* message) {
    if (mqttClient.publish(topic, message)) {
		//Serial.printf("Publish Topic: %s  ;  Message: %s\n", topic, message);
    } else {
        //Serial.println("Message Publish Failed.");
    }
}


void creatTask(){
	xTaskCreate(message_task, "MessageTask", 6144, NULL, 3, &Message_TaskHandle);
}
//   // ESP8266连接wifi
//   void connectWifi() {
// 	WiFi.begin(ssid, password);
// 	//等待WiFi连接,成功连接后输出成功信息
// 	while (WiFi.status() != WL_CONNECTED) {
// 	  delay(1000);
// 	  Serial.print(".");
// 	}
// 	Serial.println("");
// 	Serial.println("WiFi Connected!");
// 	Serial.println("");
//   }