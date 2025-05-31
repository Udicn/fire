#ifndef API_H
#define API_H

#include <Arduino.h>
extern String lat;
extern String lng;

// 函数声明
// void Get_Tx_API(); // 旧的声明
// 新的声明：通过 String 数组引用传出数据，返回 bool 表示成功或失败
bool Get_Tx_API(String weather_data_out[], int data_out_size);

// 修改函数声明：通过 String 数组引用传出数据，返回 bool 表示成功或失败
// 数组大小假定为3，分别存储 天气, 风向, 风力
bool send_get_request(String url, String weather_data[], int data_size);

#endif