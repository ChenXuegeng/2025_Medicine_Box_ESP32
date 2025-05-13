#include "ADC.h"

#define PRESS_MIN 0   // 最小压力值
#define PRESS_MAX 200 // 最大压力值
#define VOLTAGE_MIN 0  // 最小电压值 (mV)
#define VOLTAGE_MAX 3000 // 最大电压值 (mV)



int getFilteredADCValue(int pin) {
    const int sampleCount = 10; // 采样次数
    long total = 0;
    for (int i = 0; i < sampleCount; i++) {
        total += analogRead(pin);
        delay(5); // 每次采样间隔
    }
    return total / sampleCount;
}

long getPressValue(int pin) {
    long PRESS_AO = 0;
    int VOLTAGE_AO = 0;

    // 获取滤波后的 ADC 值
    int value = getFilteredADCValue(pin);

    // 打印原始 ADC 值
    // Serial.print("Raw ADC Value (Pin ");
    // Serial.print(pin);
    // Serial.print("): ");
    // Serial.println(value);

    // 将 ADC 值映射为电压值 (mV)
    VOLTAGE_AO = map(value, 0, 4095, VOLTAGE_MIN, VOLTAGE_MAX);

    // 打印映射后的电压值
    // Serial.print("Voltage (mV) (Pin ");
    // Serial.print(pin);
    // Serial.print("): ");
    // Serial.println(VOLTAGE_AO);

    // 根据电压值计算压力值
    if (VOLTAGE_AO < VOLTAGE_MIN) {
        PRESS_AO = PRESS_MIN;
    } else if (VOLTAGE_AO > VOLTAGE_MAX) {
        PRESS_AO = PRESS_MAX;
    } else {
        PRESS_AO = map(VOLTAGE_AO, VOLTAGE_MIN, VOLTAGE_MAX, PRESS_MIN, PRESS_MAX);
    }

    // 打印映射后的压力值
    // Serial.print("Pressure (g) (Pin ");
    // Serial.print(pin);
    // Serial.print("): ");
    // Serial.println(PRESS_AO);

    return PRESS_AO;
}
// long getPressValue(int pin) {
//     long PRESS_AO = 0;
//     int VOLTAGE_AO = 0;
//     int value = analogRead(pin);  // 读取模拟值
//     // int reversedValue = 4095 - value;  // ESP32 的 ADC 分辨率为 12 位 (0-4095)

// 	// Serial.print("Raw ADC Value (Pin ");
//     // Serial.print(pin);
//     // Serial.print("): ");
//     // Serial.println(value);
//     // 将模拟值映射为电压值
//     VOLTAGE_AO = map(value, 0, 4095, 0, 3000);

//     // 根据电压值计算压力值
//     if (VOLTAGE_AO < VOLTAGE_MIN) {
//         PRESS_AO = 0;
//     } else if (VOLTAGE_AO > VOLTAGE_MAX) {
//         PRESS_AO = PRESS_MAX;
//     } else {
//         PRESS_AO = map(VOLTAGE_AO, VOLTAGE_MIN, VOLTAGE_MAX, PRESS_MIN, PRESS_MAX);
//     }

// 	// 打印映射后的压力值
//     // Serial.print("Pressure (g) (Pin ");
//     // Serial.print(pin);
//     // Serial.print("): ");
//     // Serial.println(PRESS_AO);


//     return PRESS_AO;
// }