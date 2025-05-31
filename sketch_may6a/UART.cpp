#include "UART.h"

#include <string.h> // 需要包含 string.h 以使用 strncmp
String lat="30.444615000000045";
String lng="114.42752399999995";
// 注意：确保你的目标板支持将Serial2映射到指定的rxPin和txPin
// 例如，在ESP32上这是常见的做法。

void setupSerial2(long baudRate, int rxPin, int txPin) {
  // 使用 Serial2，并指定 RX 和 TX 引脚
  // SERIAL_8N1 是一个常见的配置：8个数据位，无奇偶校验，1个停止位
  Serial2.begin(baudRate, SERIAL_8N1, rxPin, txPin);
}

// 将NMEA格式的经纬度转换为十进制度
float nmeaToDecimal(float nmeaValue) {
  int degrees = int(nmeaValue / 100);
  float minutes = nmeaValue - (degrees * 100);
  return degrees + (minutes / 60.0);
}

bool getCompleteSerial2Message(char* providedBuffer, int providedBufferSize) {
  static char internalBuffer[SERIAL_BUFFER_SIZE];
  static int bufferIndex = 0;
  bool messageCopiedToBuffer = false;
  const char* gpggaPrefix = "$GPGGA";
  int prefixLen = 6;

  // Helper function to copy string to providedBuffer and set flag
  auto copyToProvidedBuffer = [&](const String& str) {
    if (str.length() < providedBufferSize) {
      strncpy(providedBuffer, str.c_str(), providedBufferSize - 1);
      providedBuffer[providedBufferSize - 1] = '\0';
      messageCopiedToBuffer = true;
    } else {
      Serial.println("错误: 输出字符串对于 providedBuffer 过长 (截断)");
      strncpy(providedBuffer, str.c_str(), providedBufferSize - 1);
      providedBuffer[providedBufferSize - 1] = '\0';
      messageCopiedToBuffer = true; // Still true, but data is truncated
    }
  };

  if (Serial2.available() > 0) {
    // 添加超时检测
    unsigned long startTime = millis();
    while(Serial2.available() && (millis() - startTime) < 100) { // 100ms超时
      char incomingByte = Serial2.read();

      if (incomingByte == '\n' || incomingByte == '\r') { 
        if (bufferIndex > 0) { 
          internalBuffer[bufferIndex] = '\0'; 
          String debugMessage = ""; // 用于存储错误信息

          if (bufferIndex >= prefixLen && strncmp(internalBuffer, gpggaPrefix, prefixLen) == 0) {
            char *p = internalBuffer;
            char *field[15]; 
            int fieldIndex = 0;
            field[fieldIndex++] = p; 

            while ((p = strchr(p, ',')) && fieldIndex < 15) {
              *p = '\0'; 
              field[fieldIndex++] = ++p;
            }

            if (fieldIndex >= 10 && field[6] && field[6][0] != '0') { 
              float nmeaLatitude = atof(field[2]);
              float nmeaLongitude = atof(field[4]);
              float altitude = atof(field[9]);

              float decimalLatitude_float = nmeaToDecimal(nmeaLatitude);
              float decimalLongitude_float = nmeaToDecimal(nmeaLongitude);

              String decimalLatitude_str = String(decimalLatitude_float, 10);
              lat=decimalLatitude_str;
              String decimalLongitude_str = String(decimalLongitude_float, 10);
              lng=decimalLongitude_str;
              String altitude_str = String(altitude, 1);

              String outputString = "纬度: " + decimalLatitude_str +
                                  ", 经度: " + decimalLongitude_str +
                                  ", 高度: " + altitude_str + " 米";
                                  
              Serial.println(outputString);
              copyToProvidedBuffer(outputString);

            } else if (fieldIndex >= 7 && field[6] && field[6][0] == '0') {
              debugMessage = "GPS数据无效：定位质量为0,使用默认经纬度";
              Serial.println(debugMessage);
              copyToProvidedBuffer(debugMessage);
            } else {
              debugMessage = "GPS数据错误：格式不完整或字段缺失,使用默认经纬度";
              Serial.println(debugMessage);
              copyToProvidedBuffer(debugMessage);
            }
          } else {
            // 如果不是GPGGA数据，也可以选择将原始的 internalBuffer 内容放入 providedBuffer
            // 或者特定的错误信息
            // 例如: debugMessage = "非GPGGA数据: " + String(internalBuffer);
            // Serial.println(debugMessage);
            // copyToProvidedBuffer(debugMessage);
          }
          
          bufferIndex = 0; 
        }
      } else if (bufferIndex < SERIAL_BUFFER_SIZE - 1) { 
        internalBuffer[bufferIndex++] = incomingByte; 
      } else {
        String errorMessage = "GPS数据错误：数据长度超出缓冲区";
        Serial.println(errorMessage);
        copyToProvidedBuffer(errorMessage); // 将缓冲区溢出错误也放入 providedBuffer
        bufferIndex = 0; 
      }
    }
    // 如果没有收到有效数据，使用上一次的有效数据
    if (!messageCopiedToBuffer) {
      String outputString = "纬度: " + lat + ", 经度: " + lng;
      copyToProvidedBuffer(outputString);
    }
    return messageCopiedToBuffer;
  }
  return messageCopiedToBuffer;
}
