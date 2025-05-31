#include "API.h"
#include <ArduinoJson.h>
#include <HTTPClient.h>

const char* key1 = "RLOBZ-MN4LH-CG7DA-WMPXK-23TW7-WPFA2";


const size_t capacity = 512;

// 返回 bool 表示是否成功，天气数据通过 weather_data 数组传出
bool send_get_request(String url, String weather_data[], int data_size) {
    if (data_size < 3) { // 确保数组大小至少为3，以存储天气、风向、风力
        Serial.println(F("Error: weather_data array size is insufficient."));
        return false;
    }

    HTTPClient http;
    bool success = false; // 操作成功标志

    Serial.print("[HTTP] begin...\n");
    Serial.println("Requesting URL: " + url);
    http.begin(url);

    Serial.print("[HTTP] GET...\n");
    int httpResponseCode = http.GET();

    if (httpResponseCode > 0) {
        Serial.printf("[HTTP] GET... code: %d\n", httpResponseCode);
        String payload = http.getString();
        Serial.println("Raw payload: " + payload);

        DynamicJsonDocument doc(capacity);
        DeserializationError error = deserializeJson(doc, payload);

        if (error) {
            Serial.print(F("deserializeJson() failed: "));
            Serial.println(error.f_str());
            // 可以选择在数组中填充错误信息，或者依赖返回的 false
            weather_data[0] = "Error_JSON_Parse";
        } else {
            int status_code = doc["status"];
            Serial.printf("JSON status: %d\n", status_code);

            if (status_code == 0) {
                JsonObject infos = doc["result"]["realtime"][0]["infos"];
                const char* weather_c = infos["weather"];
                const char* wind_direction_c = infos["wind_direction"];
                const char* wind_power_c = infos["wind_power"];
                
                weather_data[0] = weather_c ? String(weather_c) : "N/A";
                weather_data[1] = wind_direction_c ? String(wind_direction_c) : "N/A";
                weather_data[2] = wind_power_c ? String(wind_power_c) : "N/A";
                success = true; // 数据成功解析并填充

                Serial.println(F("--- Parsed Weather Info (to array) ---"));
                Serial.print(F("天气: ")); Serial.println(weather_data[0]);
                Serial.print(F("风向: ")); Serial.println(weather_data[1]);
                Serial.print(F("风力: ")); Serial.println(weather_data[2]);
            } else {
                const char* message_c = doc["message"];
                weather_data[0] = "Error_API_Status";
                if (message_c) weather_data[1] = String(message_c); // 可选：将API错误消息放入数组的某个位置
                Serial.print(F("API Error Message: "));
                Serial.println(message_c ? message_c : "Unknown API error");
            }
        }
    } else {
        String error_msg = http.errorToString(httpResponseCode).c_str();
        weather_data[0] = "Error_HTTP_GET";
        if (!error_msg.isEmpty()) weather_data[1] = error_msg; // 可选：将HTTP错误消息放入数组
        Serial.printf("[HTTP] GET... failed, error: %s\n", error_msg.c_str());
    }
    http.end();
    return success; // 返回操作是否成功
}

// 修改后的 Get_Tx_API 函数
// 它接收一个外部的 String 数组来填充数据，并返回操作是否成功
bool Get_Tx_API(String weather_data_out[], int data_out_size) {
    if (data_out_size < 3) { // 确保外部传入的数组大小至少为3
        Serial.println(F("Error: weather_data_out array size is insufficient in Get_Tx_API."));
        if (data_out_size > 0) weather_data_out[0] = "Error_Array_Size";
        return false;
    }

    String url_weather = "https://apis.map.qq.com/ws/weather/v1/?location=" + lat + "," + lng + "&key=" + String(key1) + "&type=now";
    Serial.println("Weather URL: " + url_weather);
    bool success = send_get_request(url_weather, weather_data_out, data_out_size); // 确保传递正确的数组和大小

    if (success) {
        Serial.print("  天气: "); Serial.println(weather_data_out[0]);
        Serial.print("  风向: "); Serial.println(weather_data_out[1]);
        Serial.print("  风力: "); Serial.println(weather_data_out[2]);
    } else {
     //   Serial.println("Get_Tx_API: 获取天气数据失败。检查输出数组中的错误信息 (如果填充了):");
        if (data_out_size > 0) { // 检查是否有空间存储错误信息
            Serial.print("  weather_data_out[0]: "); Serial.println(weather_data_out[0]);
            if (data_out_size > 1 && !weather_data_out[1].isEmpty()) { // 如果有填充额外错误信息
              Serial.print("  weather_data_out[1] (details): "); Serial.println(weather_data_out[1]);
            }
        }
    }
    return success; // 返回操作的成功状态
}
