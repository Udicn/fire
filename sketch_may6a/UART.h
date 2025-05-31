#ifndef UART_H
#define UART_H

#include <Arduino.h>
#define SERIAL_BUFFER_SIZE 200 // 定义串口内部缓冲区大小
const int RX2_PIN = 10; // IO10 作为 Serial2 的 RX
const int TX2_PIN = 47; // IO47 作为 Serial2 的 TX
// 初始化串口通信，指定波特率、RX引脚和TX引脚
void setupSerial2(long baudRate, int rxPin, int txPin);

// 处理串口2的接收和发送
bool getCompleteSerial2Message(char* providedBuffer, int providedBufferSize);

#endif
