#include  <stdint.h>
#include  <string.h>  
#include  <Arduino.h>
#include  "usart.h"
#include "HardwareSerial.h"
void uart_send(uint8_t data)
{
  Serial1.write(data); // 发送单个字节
}