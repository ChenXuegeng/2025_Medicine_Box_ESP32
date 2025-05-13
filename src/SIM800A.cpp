#include <Arduino.h>
#include <string.h>
#include "SIM800A.h"

// 定义缓冲区和标志变量
uint8_t SIM900_CSQ[3];
uint8_t dtbuf[50]; // 打印缓冲区
uint8_t Flag_Rec_Call = 0;

// 定义串口
#define SIM900_SERIAL Serial2 // 使用 Serial2 作为 SIM900A 的串口

// 接收缓冲区
#define RX_BUFFER_SIZE 256
char RX_BUFFER[RX_BUFFER_SIZE];
volatile uint16_t RX_INDEX = 0;

String convertToUCS2(const char *content) {
    String ucs2 = "";
    while (*content) {
      uint16_t unicode;
      if ((uint8_t)*content < 0x80) {
        // 单字节字符 (ASCII)
        unicode = *content++;
      } else if ((uint8_t)*content < 0xE0) {
        // 双字节字符 (常见于非中文字符)
        unicode = ((*content & 0x1F) << 6) | (*(content + 1) & 0x3F);
        content += 2;
      } else if ((uint8_t)*content < 0xF0) {
        // 三字节字符 (中文字符)
        unicode = ((*content & 0x0F) << 12) | ((*(content + 1) & 0x3F) << 6) | (*(content + 2) & 0x3F);
        content += 3;
      } else {
        // 不支持的字符，跳过
        content++;
        continue;
      }
  
      // 转换为 UCS2 格式的十六进制字符串
      char buf[5];
      sprintf(buf, "%04X", unicode);
      ucs2 += buf;
    }
    return ucs2;
  }

// 串口接收中断模拟（ESP32 不支持直接中断，需要轮询或回调）
void serialEvent2() {
  while (SIM900_SERIAL.available()) {
    char c = SIM900_SERIAL.read();
    if (RX_INDEX < RX_BUFFER_SIZE - 1) {
      RX_BUFFER[RX_INDEX++] = c;
    }
    RX_BUFFER[RX_INDEX] = '\0'; // 添加结束符
  }
}

// 检查接收到的 AT 指令应答
uint8_t *sim900a_check_cmd(const char *str) {
  return (uint8_t *)strstr((const char *)RX_BUFFER, str);
}



// uint8_t sim900a_send_cmd(const char *cmd, const char *ack, uint16_t waittime) {
//   RX_INDEX = 0; // 清空接收缓冲区
//   memset(RX_BUFFER, 0, RX_BUFFER_SIZE);
//   // 发送命令
//   SIM900_SERIAL.println(cmd);
//   Serial.print("发送命令: ");
//   Serial.println(cmd);
//   // 等待应答
//   uint32_t start = millis();
//   while (millis() - start < waittime * 10) {
//     while (SIM900_SERIAL.available()) {
//       char c = SIM900_SERIAL.read();
//       if (RX_INDEX < RX_BUFFER_SIZE - 1) {
//         RX_BUFFER[RX_INDEX++] = c;
//       }
//       RX_BUFFER[RX_INDEX] = '\0'; // 添加结束符
//     }
//     // 仅在接收缓冲区有内容时打印
//     if (RX_INDEX > 0) {
//       Serial.print("接收到的内容: ");
//       Serial.println(RX_BUFFER);
//     }
//     if (sim900a_check_cmd(ack)) {
//       Serial.println("接收到期望的应答");
//       return 0; // 成功
//     }
//     delay(10);
//   }
//   Serial.println("等待应答超时！");
//   return 1; // 超时失败
// }

void logReceivedContent(const char *content) {
  static String lastContent = ""; // 保存上一次的内容
  static int repeatCount = 0;     // 重复计数

  String currentContent = String(content);
  if (currentContent == lastContent) {
    repeatCount++;
    if (repeatCount <= 5) { // 显示前 5 次重复内容
      Serial.print("接收到的内容: ");
      Serial.println(content);
    } else if (repeatCount == 6) { // 第 6 次提示不再显示
      Serial.println("接收到的内容重复 5 次了，不再显示...");
    }
  } else {
    repeatCount = 0; // 重置重复计数
    lastContent = currentContent;
    Serial.print("接收到的内容: ");
    Serial.println(content);
  }
}



uint8_t sim900a_send_cmd(const char *cmd, const char *ack, uint16_t waittime) {
  RX_INDEX = 0; // 清空接收缓冲区
  memset(RX_BUFFER, 0, RX_BUFFER_SIZE);

  // 发送命令
  SIM900_SERIAL.println(cmd);
  Serial.print("发送命令: ");
  Serial.println(cmd);

  // 等待应答
  uint32_t start = millis();
  while (millis() - start < waittime * 10) {
    while (SIM900_SERIAL.available()) {
      char c = SIM900_SERIAL.read();
      if (RX_INDEX < RX_BUFFER_SIZE - 1) {
        RX_BUFFER[RX_INDEX++] = c;
      }
      RX_BUFFER[RX_INDEX] = '\0'; // 添加结束符
    }

    // 打印接收到的内容
    if (RX_INDEX > 0) {
      logReceivedContent(RX_BUFFER);
    }

    if (sim900a_check_cmd(ack)) {
      Serial.println("接收到期望的应答");
      return 0; // 成功
    }
    vTaskDelay(pdMS_TO_TICKS(10)); // 延时 10ms，避免过快的循环
    //delay(10);
  }

  Serial.println("等待应答超时！");
  return 1; // 超时失败
}

uint8_t sim900a_work_test() {
  if (sim900a_send_cmd("AT", "OK", 100)) 
  {
    vTaskDelay(pdMS_TO_TICKS(100)); // 等待 100ms
    //delay(100); // 等待 100ms
    if (sim900a_send_cmd("AT", "OK", 100)) 
    {
        return 1; // 通信失败
    }
  }
  if (sim900a_send_cmd("AT+CPIN?", "READY", 400)) 
  {
    return 2; // 没有检测到 SIM 卡
  }
  if (sim900a_send_cmd("AT+CREG?", "0,1", 400)) 
  {
    if (!sim900a_check_cmd("0,5")) 
    {
      if (!sim900a_send_cmd("AT+CSQ", "OK", 200)) 
      {
        memcpy(SIM900_CSQ, RX_BUFFER + 15, 2);
      }
      return 3; // 网络注册失败
    }
  }
  return 0; // 成功
}
// uint8_t GSM_Dect() {
//   Serial.println("开始通信测试...");
//   if (sim900a_send_cmd("AT", "OK", 100)) {
//     Serial.println("通信测试失败，重试...");
//     if (sim900a_send_cmd("AT", "OK", 100)) {
//       Serial.println("通信失败！");
//       return 1; // 通信失败
//     }
//   }
//   Serial.println("通信测试成功");
//   Serial.println("检查 SIM 卡状态...");
//   if (sim900a_send_cmd("AT+CPIN?", "READY", 400)) {
//     Serial.println("未检测到 SIM 卡！");
//     return 2; // 没有检测到 SIM 卡
//   }
//   Serial.println("SIM 卡状态正常");
//   Serial.println("检查信号强度...");
//   if (!sim900a_send_cmd("AT+CSQ", "OK", 200)) {
//     memcpy(SIM900_CSQ, RX_BUFFER + 6, 2); // 提取信号强度
//     Serial.print("信号强度: ");
//     Serial.println((char *)SIM900_CSQ);
//     if (strcmp((char *)SIM900_CSQ, "0") == 0) {
//       Serial.println("信号强度为 0,无法注册到网络！");
//       return 3; // 信号强度不足
//     }
//   }
//   Serial.println("检查网络注册状态...");
//   if (sim900a_send_cmd("AT+CREG?", "OK", 400)) {
//     if (sim900a_check_cmd("0,1") || sim900a_check_cmd("0,5")) {
//       Serial.println("网络注册成功");
//       return 0; // 成功
//     } else {
//       Serial.println("未注册到网络，尝试手动注册...");
//       if (sim900a_send_cmd("AT+COPS=0", "OK", 1000)) {
//         Serial.println("手动注册失败！");
//         return 4; // 网络注册失败
//       }
//       Serial.println("手动注册成功！");
//     }
//   }
//   // 再次检查网络注册状态
//   Serial.println("再次检查网络注册状态...");
//   if (sim900a_send_cmd("AT+CREG?", "OK", 400)) {
//     if (sim900a_check_cmd("0,1") || sim900a_check_cmd("0,5")) {
//       Serial.println("网络注册成功");
//       return 0; // 成功
//     }
//   }
//   Serial.println("手动注册后仍未成功注册到网络！");
//   return 4; // 网络注册失败
// }
// GSM 模块自检
uint8_t GSM_Dect() {
  uint8_t res = sim900a_work_test();
  switch (res) {
    case 0:
      Serial.println("GSM 模块自检成功");
      break;
    case 1:
      Serial.println("与 GSM 模块通讯未成功，请等待");
      break;
    case 2:
      Serial.println("未检测到 SIM 卡");
      break;
    case 3:
      Serial.print("注册网络中... 当前信号值：");
      Serial.println((char *)SIM900_CSQ);
      break;
    default:
      break;
  }
  return res;
}

// 发送短信
uint8_t sim900a_send_chmessage_zc(const char *number, const char *content) {
  char cmd[100];
  sprintf(cmd, "AT+CMGS=\"%s\"", number);

  if (sim900a_send_cmd("AT+CMGF=1", "OK", 100)) return 1; // 设置文本模式失败
  if (sim900a_send_cmd("AT+CSCS=\"GSM\"", "OK", 100)) return 2; // 设置字符集失败
  if (sim900a_send_cmd("AT+CSCA?", "OK", 100)) return 3; // 获取短信中心失败
  if (sim900a_send_cmd("AT+CSMP=17,167,0,241", "OK", 100)) return 4; // 设置短信参数失败
  if (sim900a_send_cmd(cmd, ">", 100)) return 5; // 发送号码失败

    // 转换短信内容为 UCS2 编码
    // String ucs2Content = convertToUCS2(content);
    // SIM900_SERIAL.print(ucs2Content); // 发送短信内容
    // SIM900_SERIAL.write(0x1A);        // 发送 Ctrl+Z 表示结束
  
  SIM900_SERIAL.print(content); // 发送短信内容
  SIM900_SERIAL.write(0x1A);    // 发送 Ctrl+Z 表示结束

  if (sim900a_send_cmd("", "+CMGS:", 1500)) return 6; // 发送失败
  return 0; // 成功
}
//发送中文短信
uint8_t sim900a_send_chmessage_zc_01(const char *number, const char *content) {
    char cmd[100];
  
    // 转换电话号码为 UCS2 编码
    String ucs2Number = convertToUCS2(number);
    sprintf(cmd, "AT+CMGS=\"%s\"", ucs2Number.c_str());
  
    Serial.println(ucs2Number);
    if (sim900a_send_cmd("AT+CMGF=1", "OK", 100)) return 1; // 设置文本模式失败
    if (sim900a_send_cmd("AT+CSCS=\"UCS2\"", "OK", 100)) return 2; // 设置字符集失败
    if (sim900a_send_cmd("AT+CSCA?", "OK", 100)) return 3; // 获取短信中心失败
    if (sim900a_send_cmd("AT+CSMP=17,167,0,25", "OK", 100)) return 4; // 设置短信参数失败
    if (sim900a_send_cmd(cmd, ">", 100)) return 5; // 发送号码失败
  
    // 转换短信内容为 UCS2 编码
    String ucs2Content = convertToUCS2(content);
     SIM900_SERIAL.print(ucs2Content); // 发送短信内容
     SIM900_SERIAL.write(0x1A);        // 发送 Ctrl+Z 表示结束
  
    if (sim900a_send_cmd("", "+CMGS:", 1500)) return 6; // 发送失败
    return 0; // 成功
  }

  //打电话
  uint8_t SIM_MAKE_CALL(const char *number) {
    char cmd[50]; // 定义命令缓冲区
    sprintf(cmd, "ATD%s;", number); // 格式化拨号命令

    // 发送拨号命令并检查应答
    if (sim900a_send_cmd(cmd, "OK", 200)) {
        Serial.println("拨号失败");
        return 1; // 返回错误码
    }

    Serial.println("拨号成功");
    return 0; // 返回成功码
}
