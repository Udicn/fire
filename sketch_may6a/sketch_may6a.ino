// 引入 DHT 库
#include "DHT.h"
// 引入 WiFi 和 HTTPClient 库
#include <WiFi.h>
#include <HTTPClient.h>
#include "LD3320.h"
#include "Arduino.h"
// 引入 MQTT 客户端库
#include <PubSubClient.h>
#include "ICM45686.h"
#include <HardwareSerial.h>
const char* ssid = "whvcse";         // 您的WiFi名称
const char* password = ""; // WiFi密码为空 (开放网络)
// --- WiFi 配置结束 ---

// --- 校园网认证服务器配置 ---
const char* campusAuthUrl = "http://10.80.1.122/eportal/InterFace.do?method=login";
const char* campusAuthPostData = "userId=23429501151389&password=123456&service=after-auth&queryString=wlanuserip%3D10.36.204.166%26wlanacname%3D%26nasip%3D172.10.1.1%26wlanparameter%3Df4-c8-8a-3f-80-f9%26url%3Dhttp%3A%2F%2Fwww.msftconnecttest.com%2Fredirect%26userlocation%3Dethtrunk%2F200%3A1816.0&operatorPwd=&operatorUserId=&validcode=&passwordEncrypt=false";
// --- 校园网认证服务器配置结束 ---

// --- MQTT代理配置 ---
const char* mqttServer = "broker.emqx.io"; // 公共MQTT代理，或替换为您的代理地址
const int mqttPort = 1883;                     // MQTT默认端口 (非加密)
const char* mqttcommendTopic = "device/a6cb2bc6443ed9be/send_commend";
const char* mqttTemperatureTopic = "device/a6cb2bc6443ed9be/temperature";
const char* mqttspeakTopic = "device/a6cb2bc6443ed9be/speak";
const char* mqttICM45686Topic = "device/a6cb2bc6443ed9be/ICM45686";
const char* mqtthongwaiTopic="device/a6cb2bc6443ed9be/temperature_hongwai";
const char* mqttsmogTopic="device/a6cb2bc6443ed9be/smog";
// const char* mqttUser = "your_mqtt_user";    // 如果您的MQTT代理需要认证
// const char* mqttPassword = "your_mqtt_password"; // 如果您的MQTT代理需要认证
TaskHandle_t dhtTaskHandle = NULL;
TaskHandle_t get_commendHandle =NULL;
TaskHandle_t ICM45686Handle=NULL;
TaskHandle_t LD3320Handle=NULL;
TaskHandle_t SmogHanle=NULL;
char flag_temperature=0;
#define DHTPIN 0    // 例如，将 DHT11 的数据引脚连接到 ESP32-S3 的 GPIO0
                    // 注意: GPIO0 有时用于启动模式，建议使用其他引脚如 GPIO2, GPIO4 等，如果遇到问题。

// 定义 DHT 传感器类型
#define DHTTYPE DHT11   // 我们使用的是 DHT11

// 初始化 DHT 对象
DHT dht(DHTPIN, DHTTYPE);

// 初始化 WiFi 和 MQTT 客户端
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// 任务句柄

bool campusAuthSuccessful = false; // 标志校园网认证是否成功
/*---------------ICM45686---------------------------*/
const int ESP32_S3_SPI_SCK_PIN = 5;   // GPIO5 用于 SCK (串行时钟)
const int ESP32_S3_SPI_MOSI_PIN = 7;  // GPIO7 用于 MOSI (主出从入)
const int ESP32_S3_SPI_MISO_PIN = 16; // GPIO16 用于 MISO (主入从出)
const int ESP32_S3_SPI_CS_PIN = 18;    // GPIO8 用于 CS (片选) - 与原始代码一致
ICM456xx IMU(SPI, ESP32_S3_SPI_CS_PIN);

// 姿态计算相关变量
float roll = 0.0, pitch = 0.0, yaw = 0.0;
float gyro_bias[3] = {0.0, 0.0, 0.0};
bool isCalibrated = false;
float last_time = 0.0;
const float alpha = 0.98; // 互补滤波系数

//smog的AD引脚
const int MQ_SENSOR_PIN = 14; // 
float readStabilizedADC(int pin) {
  float total = 0;
  for (int i = 0; i < 10; i++) {
    total += analogRead(pin);
    vTaskDelay(50); // 每次采样之间短暂延迟，可以根据需要调整
  }
  return total / 10;
}
void smog_get(void *pvParameters){
  while(1)
  {
    float adcValue = readStabilizedADC(MQ_SENSOR_PIN);

    Serial.print("Stabilized ADC Value: ");
    float a=adcValue/4095.0;
    a*=100;
    char msgBuffer[30];
    sprintf(msgBuffer,"%.1f%%",a);
    mqttClient.publish(mqttsmogTopic, msgBuffer);

    vTaskDelay(1000); // 每秒读取一次
  }

}


// ICM45686初始化函数
void ICM45686_init() {
  int ret;
  // 使用自定义引脚初始化 SPI 总线
  // SPI.begin(sck, miso, mosi, ss);
  // 此处 ss (片选) 参数可以省略或设为-1，因为 ICM456xx 库会通过构造函数中的 CS 引脚参数来手动控制片选
  SPI.begin(ESP32_S3_SPI_SCK_PIN, ESP32_S3_SPI_MISO_PIN, ESP32_S3_SPI_MOSI_PIN);
  Serial.println("SPI bus configured with custom pins:");
  Serial.print("SCK: GPIO"); Serial.println(ESP32_S3_SPI_SCK_PIN);
  Serial.print("MOSI: GPIO"); Serial.println(ESP32_S3_SPI_MOSI_PIN);
  Serial.print("MISO: GPIO"); Serial.println(ESP32_S3_SPI_MISO_PIN);
  Serial.print("CS: GPIO"); Serial.println(ESP32_S3_SPI_CS_PIN);

  // 初始化 ICM456XX
  ret = IMU.begin();
  if (ret != 0) {
    Serial.println("ICM45686初始化失败");
    while(1);
  }
  // 设置加速度计：ODR = 20 Hz, 量程 = 16G
  IMU.startAccel(20,16);
  // 设置陀螺仪：ODR = 20 Hz, 量程 = 2000 dps
  IMU.startGyro(20,2000);
  // 等待 IMU 启动和稳定
  vTaskDelay(100);
  Serial.println("ICM456xx初始化成功");
}

// 校准陀螺仪零偏
void calibrateGyro() {
  Serial.println("开始校准陀螺仪...");
  float gx_sum = 0, gy_sum = 0, gz_sum = 0;
  const int samples = 100;
  
  for (int i = 0; i < samples; i++) {
    inv_imu_sensor_data_t imu_data;
    IMU.getDataFromRegisters(imu_data);
    
    // 陀螺仪数据转换
    float gx = imu_data.gyro_data[0] / 16.4f; // 2000dps量程下，1dps = 16.4 LSB
    float gy = imu_data.gyro_data[1] / 16.4f;
    float gz = imu_data.gyro_data[2] / 16.4f;
    
    gx_sum += gx;
    gy_sum += gy;
    gz_sum += gz;
    vTaskDelay(10);
  }
  
  // 计算平均值作为零偏
  gyro_bias[0] = gx_sum / samples;
  gyro_bias[1] = gy_sum / samples;
  gyro_bias[2] = gz_sum / samples;
  
  Serial.printf("陀螺仪零偏校准完成: X=%.2f, Y=%.2f, Z=%.2f\n", gyro_bias[0], gyro_bias[1], gyro_bias[2]);
  
  // 使用加速度计设置初始roll和pitch
  inv_imu_sensor_data_t imu_data;
  IMU.getDataFromRegisters(imu_data);
  
  // 加速度计数据转换（±16g量程下，1g = 2048 LSB）
  float ax = imu_data.accel_data[0] / 2048.0f;
  float ay = imu_data.accel_data[1] / 2048.0f;
  float az = imu_data.accel_data[2] / 2048.0f;
  
  // 计算初始Roll和Pitch
  roll = atan2(ay, sqrt(ax*ax + az*az));
  pitch = atan2(-ax, sqrt(ay*ay + az*az));
  yaw = 0; // 初始化yaw为0
  
  isCalibrated = true;
  last_time = millis() / 1000.0f;
}

// 更新姿态计算
void updateAttitude() {
  inv_imu_sensor_data_t imu_data;
  IMU.getDataFromRegisters(imu_data);
  
  // 当前时间
  float now = millis() / 1000.0f;
  float dt = now - last_time;
  if (dt > 0.5 || dt <= 0) dt = 0.01; // 处理异常时间间隔
  last_time = now;
  
  // 加速度计数据转换（±16g量程下，1g = 2048 LSB）
  float ax = imu_data.accel_data[0] / 2048.0f;
  float ay = imu_data.accel_data[1] / 2048.0f;
  float az = imu_data.accel_data[2] / 2048.0f;
  
  // 陀螺仪数据转换（±2000dps量程下，1dps = 16.4 LSB）
  float gx = imu_data.gyro_data[0] / 16.4f - gyro_bias[0];
  float gy = imu_data.gyro_data[1] / 16.4f - gyro_bias[1];
  float gz = imu_data.gyro_data[2] / 16.4f - gyro_bias[2];
  
  // 检测是否静止
  float accel_mag = sqrt(ax*ax + ay*ay + az*az);
  bool is_stationary = (0.9 < accel_mag && accel_mag < 1.1);
  
  // 将角速度转换为弧度/秒
  gx = gx * (3.14159265358979323846f / 180.0f);
  gy = gy * (3.14159265358979323846f / 180.0f);
  gz = gz * (3.14159265358979323846f / 180.0f);
  
  // 陀螺仪积分计算角度变化
  roll += gx * dt;
  pitch += gy * dt;
  yaw += gz * dt;
  
  // 如果静止，使用加速度计微调Roll和Pitch
  if (is_stationary) {
    float acc_roll = atan2(ay, sqrt(ax*ax + az*az));
    float acc_pitch = atan2(-ax, sqrt(ay*ay + az*az));
    
    // 互补滤波
    float weight = 0.01; // 低权重，主要依赖陀螺仪
    roll = roll * (1 - weight) + acc_roll * weight;
    pitch = pitch * (1 - weight) + acc_pitch * weight;
  }
}
/*--------------------------------------------------------------------------hongwai-------------------------------------------------------------------------------------*/

TaskHandle_t hongwaiHandle=NULL;
HardwareSerial SerialPort(0);
void hongwai_init()
{
  SerialPort.begin(38400);
  vTaskDelay(1000);
  byte configCmd[] = {0xAA, 0xA5, 0x04, 0x05, 0x01, 0x0A, 0x55};
  SerialPort.write(configCmd, sizeof(configCmd));
  Serial.println("已向红外测温发送配置命令");
  vTaskDelay(1000);
}
void hongwai_get(void *pvParameters)
{
  hongwai_init();
  static float targetTemperature;
  while(1)
  {
    if(flag_temperature==1)
    {
      flag_temperature=0;
      byte tempCmd[] = {0xAA, 0xA5, 0x03, 0x01, 0x04, 0x55};
      SerialPort.write(tempCmd, sizeof(tempCmd));
      vTaskDelay(100);
      if (SerialPort.available()) {
        int availableBytes = SerialPort.available();
        byte data[256];
        int bytesRead = SerialPort.readBytes(data, availableBytes);
        bool foundStart = false;
        int startIndex = -1;
        for (int i = 0; i < bytesRead - 1; i++) {
          if (data[i] == 0xAA && data[i + 1] == 0xA5) {
            foundStart = true;
            startIndex = i;
            break;
          }
        }
        if (foundStart && startIndex + 9 < bytesRead) {
          word targetTempHex = (data[startIndex + 5] << 8) | data[startIndex + 6];
         targetTemperature = targetTempHex / 10.0;
          word ambientTempHex = (data[startIndex + 7] << 8) | data[startIndex + 8];
          float ambientTemperature = ambientTempHex / 10.0;
          char msgBuffer[30];
          sprintf(msgBuffer,"目标温度:%f°C,环境温度:%f",targetTemperature,ambientTemperature);
          mqttClient.publish(mqtthongwaiTopic, msgBuffer);

        }
      }
      else {
        Serial.println("未收到数据");
      }
    }
    if(targetTemperature>=27)
    {
      digitalWrite(36,LOW);
      vTaskDelay(500);
      digitalWrite(36,HIGH);
      vTaskDelay(500);
    }
    else {
      digitalWrite(36,HIGH);
    }
  vTaskDelay(1000);
  }
  
}

/* ------------------------------------------------------------------------------------------------------------LD3320---------------------------------------------------------------------------------------------------------------------*/


void IRAM_ATTR ProcessInt(void);
LD3320 WE;
u8 nAsrStatus=0;
u8 nAsrRes=0;
extern u8  ucRegVal;
u8 flag=0;
void LD3320_init()
{
  WE.LD3320_IO_Init();
  WE.LD_Reset();
  // attachInterrupt(1, ProcessInt, FALLING); // 原来的代码
  attachInterrupt(digitalPinToInterrupt(9), ProcessInt, FALLING); // 修改后的代码，假设 IRQ 连接到 GPIO 9
  nAsrStatus = LD_ASR_NONE;    //初始状态：没有在作ASR
  SCS_0;
  Serial.println("Start\r\n"); 
}
void LD3320_Start(void *pvParameters)
{
  LD3320_init();
  while(1)
  {
  switch(nAsrStatus)
    {
      case LD_ASR_RUNING:
      case LD_ASR_ERROR:  
           break;
      case LD_ASR_NONE:
      {
        nAsrStatus=LD_ASR_RUNING;
        if (WE.RunASR()==0)  /*  启动一次ASR识别流程：ASR初始化，ASR添加关键词语，启动ASR运算*/
        {
          nAsrStatus = LD_ASR_ERROR;
        }
        break;
      }

      case LD_ASR_FOUNDOK: /* 一次ASR识别流程结束，去取ASR识别结果*/
      {
        nAsrRes = WE.LD_GetResult();   /*获取结果*/                        
        User_Modification(nAsrRes);
        nAsrStatus = LD_ASR_NONE;
        break;
      }
      case LD_ASR_FOUNDZERO:
      default:
      {
        nAsrStatus = LD_ASR_NONE;
        break;
      }
    } 
    vTaskDelay(500);
  }


}
void User_Modification(u8 dat)
{
  char msgBuffer[10];
  if(dat ==0)
  {
    flag=1;
    Serial.println("Received\r\n");
    digitalWrite(36,LOW);
    vTaskDelay(500);
    digitalWrite(36,HIGH);
    
    sprintf(msgBuffer, "%s", "speak");
    mqttClient.publish(mqttcommendTopic, msgBuffer);
  }
  else if(flag)
  {
    flag=0;
    switch(nAsrRes)      /*对结果执行相关操作,客户修改*/
    {
      case CODE_DMCS:     /*命令"代码测试"*/
          Serial.println("dai ma ce shi\r\n"); /*text.....*/
                        break;
      case CODE_CSWB:     /*命令"测试完毕"*/
          Serial.println("ce shi wan bi\r\n"); /*text.....*/
                        break;
      
      case CODE_1KL1:  /*命令"北京"*/
          Serial.println("bei jing\r\n"); /*text.....*/
          sprintf(msgBuffer, "%s", "bei jing");
          mqttClient.publish(mqttspeakTopic, msgBuffer);
                        break;
      case CODE_1KL2:   /*命令"上海"*/
    
          Serial.println("shang hai\r\n"); /*text.....*/
                        break;
      case CODE_1KL3:  /*命令"开灯"*/
          Serial.println("kai deng\r\n"); /*text.....*/
                        break;
      case CODE_1KL4:   /*命令"关灯"*/        
          Serial.println("guan deng\r\n"); /*text.....*/
                        break;
      
      case CODE_2KL1:  /*命令"...."*/
          Serial.println("guang zhou\r\n"); /*text.....*/
                        break;
      case CODE_2KL2:  /*命令"...."*/
          Serial.println("shen zhen\r\n"); /*text.....*/
                        break;
      case CODE_2KL3:  /*命令"...."*/
          Serial.println("xiang zuo zhuan\r\n"); /*text.....*/
                        break;
      case CODE_2KL4:  /*命令"...."*/
          Serial.println("xiang you zhuan\r\n"); /*text.....*/
                              break;
            
      case CODE_3KL1:  /*命令"...."*/
          Serial.println("da kai kong tiao\r\n"); /*text.....*/
                        break;
      case CODE_3KL2:  /*命令"...."*/
          Serial.println("guan bi kong tiao\r\n"); /*text.....*/
                        break;
      case CODE_5KL1:  /*命令"...."*/
          Serial.println("hou tui"); /*text.....*/
                        break;
      default:break;
    }
  }
  else  
  {
    Serial.println("唤醒失败\r\n"); /*text.....*/  
  }
  
}
void IRAM_ATTR ProcessInt(void)
{
  u8 nAsrResCount=0;
  ucRegVal = WE.LD_ReadReg(0x2B);
  WE.LD_WriteReg(0x29,0) ;
  WE.LD_WriteReg(0x02,0) ;
  if((ucRegVal & 0x10)&&WE.LD_ReadReg(0xb2)==0x21&&WE.LD_ReadReg(0xbf)==0x35)     /*识别成功*/
  { 
    nAsrResCount = WE.LD_ReadReg(0xba);
    if(nAsrResCount>0 && nAsrResCount<=4) 
    {
      nAsrStatus=LD_ASR_FOUNDOK;
    }
    else
    {
      nAsrStatus=LD_ASR_FOUNDZERO;
    } 
  }                              /*没有识别结果*/
  else
  {  
    nAsrStatus=LD_ASR_FOUNDZERO;
  }
    
  WE.LD_WriteReg(0x2b, 0);
  WE.LD_WriteReg(0x1C,0);/*写0:ADC不可用*/
  WE.LD_WriteReg(0x29,0);
  WE.LD_WriteReg(0x02,0);
  WE.LD_WriteReg(0x2B,0);
  WE.LD_WriteReg(0xBA,0);  
  WE.LD_WriteReg(0xBC,0);  
  WE.LD_WriteReg(0x08,1);   /*清除FIFO_DATA*/
  WE.LD_WriteReg(0x08,0);  /*清除FIFO_DATA后 再次写0*/
}


/* ----------------------------------------------------------------------------------------------------------LD3320-----------------------------------------------------------------------------------------------------------------------------------*/


// MQTT消息回调函数 
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  String messageTemp;
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    messageTemp += (char)payload[i];
  }
  Serial.println();

  // 检查是否是命令主题，并且消息内容是 "temperature"
  if (String(topic) == mqttcommendTopic && messageTemp == "temperature") {
    flag_temperature=1;
    Serial.println("收到temperature的温度请求");
    
  }
}

// WiFi 连接函数
void setupWifi() {
  vTaskDelay(10);
  Serial.println();
  Serial.print("Connecting to WiFi network: ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.print("Attempting to connect");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    vTaskDelay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected successfully!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nFailed to connect to WiFi.");
  }
}

// 执行校园网认证POST请求
bool performCampusAuthentication() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Campus Auth: WiFi not connected. Skipping authentication.");
    return false;
  }

  HTTPClient http;
  Serial.println("Performing campus network authentication...");
  Serial.print("Auth URL: ");
  Serial.println(campusAuthUrl);

  http.begin(campusAuthUrl);
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");

  Serial.println("Sending authentication POST request...");

  int httpResponseCode = http.POST(campusAuthPostData);

  if (httpResponseCode > 0) {
    Serial.print("Campus Auth HTTP Response code: ");
    Serial.println(httpResponseCode);
    String responsePayload = http.getString();
    Serial.print("Campus Auth Server response: ");
    Serial.println(responsePayload);
    if (httpResponseCode == 200) {
      Serial.println("Campus authentication successful!");
      http.end();
      return true;
    } else {
      Serial.println("Campus authentication failed (HTTP Error or non-200 response).");
    }
  } else {
    Serial.print("Error on sending campus auth POST: ");
    Serial.println(httpResponseCode);
    Serial.printf("Campus Auth HTTP POST failed, error: %s\n", http.errorToString(httpResponseCode).c_str());
  }
  http.end();
  return false;
}

// MQTT重新连接函数
void reconnectMQTT() {
  // 循环直到重新连接
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // 创建一个随机的客户端ID
    String clientId = "ESP32Client-"; // 使用一个独特的前缀
    clientId += String(random(0xffff), HEX);  // 加上一个随机的十六进制数，确保唯一性
    // 尝试连接
    // 如果您的MQTT代理需要用户名和密码，请使用:
    // if (mqttClient.connect(clientId.c_str(), mqttUser, mqttPassword)) {
    if (mqttClient.connect(clientId.c_str())) {
      Serial.println("connected");
      // 连接成功后，订阅温度请求主题
      mqttClient.subscribe(mqttcommendTopic);
      Serial.print("Subscribed to: ");
      Serial.println(mqttcommendTopic);
      // mqttClient.publish("esp32/status", "Connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      // 等待5秒后重试
      vTaskDelay(5000); // 在重连循环中使用常规delay是可以接受的
    }
  }
}
//读取MQTT命令
void get_commend(void *pvParameters) {
  while(1)
  {
    if (!mqttClient.connected() && campusAuthSuccessful && WiFi.status() == WL_CONNECTED) {
      reconnectMQTT();
  }
  if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi disconnected. Skipping DHT read and MQTT publish.");
        vTaskDelay(pdMS_TO_TICKS(10000)); // 等待一段时间再检查WiFi
        setupWifi(); // 尝试重新连接WiFi
        if(WiFi.status() == WL_CONNECTED && !campusAuthSuccessful) {
            campusAuthSuccessful = performCampusAuthentication(); // 如果WiFi重连成功但未认证，则重新认证
        }
        continue;
    }

    if (!campusAuthSuccessful) {
        Serial.println("Campus authentication not successful. Attempting re-authentication.");
        campusAuthSuccessful = performCampusAuthentication();
        if (!campusAuthSuccessful) {
            Serial.println("Re-authentication failed. Skipping MQTT operations.");
            vTaskDelay(pdMS_TO_TICKS(10000)); // 等待一段时间再尝试
            continue;
        }
    }
    mqttClient.loop(); // 保持MQTT连接并处理消息
    vTaskDelay(pdMS_TO_TICKS(500));
  }

}
/*-----------------------------------------------ICM45686------------------------------------------------*/
void ICM45686_getdata(void *pvParameters) {
  // 首先执行校准
  ICM45686_init();
  calibrateGyro();
  while(1) {
    // 更新姿态
    if (isCalibrated) {
      updateAttitude();
    }
    
    // 读取传感器数据
    inv_imu_sensor_data_t imu_data;
    IMU.getDataFromRegisters(imu_data);
    
    char msgBuffer[150];  // 增加缓冲区大小以容纳姿态数据
    
    // 格式化数据，包含原始传感器数据和计算的姿态
    sprintf(msgBuffer, "AccelX:%d，AccelY:%d，AccelZ:%d，\r\n GyroX:%d，GyroY:%d，GyroZ:%d，\r\nTemperature:%d，\r\nRoll:%.2f，Pitch:%.2f，Yaw:%.2f",
            imu_data.accel_data[0], imu_data.accel_data[1], imu_data.accel_data[2],
            imu_data.gyro_data[0], imu_data.gyro_data[1], imu_data.gyro_data[2],
            imu_data.temp_data, 
            roll * (180.0f / 3.14159265358979323846f),  // 转换为角度
            pitch * (180.0f / 3.14159265358979323846f), 
            yaw * (180.0f / 3.14159265358979323846f));
    
    // 发布到MQTT
    if (mqttClient.connected()) {
      mqttClient.publish(mqttICM45686Topic, msgBuffer);
    }
    
    vTaskDelay(pdMS_TO_TICKS(500));  // 降低延迟，提高更新率
  }
} 


void dhtTask(void *pvParameters) {
  
  dht.begin(); // 初始化   DHT 传感器
  while(1){
      float t = dht.readTemperature(); // 读取当前温度
      if (!isnan(t)) {
      char msgBuffer[10];
      dtostrf(t, 5, 2, msgBuffer); // 转换温度为字符串，保留两位小数 (例如 "25.50")

      mqttClient.publish(mqttTemperatureTopic, msgBuffer);
    }
    vTaskDelay(pdMS_TO_TICKS(2000)); // 每10秒读取一次并发布
  }
}

void setup() {
  pinMode(36,OUTPUT);
  digitalWrite(36,HIGH);
  Serial.begin(115200);
  randomSeed(micros()); // 用于生成随机MQTT客户端ID
  
  Serial.println(F("CampusNet Auth, DHT Sensor & MQTT Test"));

  setupWifi(); // 1. 连接到WiFi

  if (WiFi.status() == WL_CONNECTED) {
    campusAuthSuccessful = performCampusAuthentication(); // 2. 执行校园网认证
    if (!campusAuthSuccessful) {
        Serial.println("Authentication failed. MQTT might not work if internet access is restricted.");
    } else {
        Serial.println("Campus authentication successful. Testing internet connectivity...");
        HTTPClient httpTest;
        Serial.print("Attempting to connect to www.baidu.com...");
        httpTest.begin("http://www.baidu.com"); // 尝试访问百度
        int httpTestResponseCode = httpTest.GET();

        if (httpTestResponseCode > 0) {
            Serial.print(" HTTP Response code: ");
            Serial.println(httpTestResponseCode);
            if (httpTestResponseCode == HTTP_CODE_OK || httpTestResponseCode == HTTP_CODE_MOVED_PERMANENTLY || httpTestResponseCode == 302) { // 200 OK or 301/302 Redirect usually means internet is accessible
                Serial.println("Successfully connected to Baidu. Internet access confirmed.");
            } else {
                Serial.println("Connected to Baidu, but got an unexpected HTTP status code.");
            }
        } else {
            Serial.print("Failed to connect to Baidu, error: ");
            Serial.println(httpTest.errorToString(httpTestResponseCode).c_str());
        }
        httpTest.end();

        // 只有在校园网认证成功后才设置MQTT服务器
        mqttClient.setServer(mqttServer, mqttPort);
        // 设置MQTT回调函数
        mqttClient.setCallback(mqttCallback); 
        Serial.println("MQTT server configured and callback set.");
    }
  } else {
    Serial.println("WiFi connection failed. Cannot proceed.");
    return;
  }

  
  // // 确保初始化ICM45686传感器
  
  xTaskCreatePinnedToCore(
    LD3320_Start,
    "LD3320_Start", // 任务名更新
    4096,            // 栈大小，MQTT和WiFi可能需要更多
    NULL,
    1,
    &LD3320Handle,
    0
  );

  xTaskCreatePinnedToCore(
    get_commend,
    "get_commend", // 任务名更新
    4096,            // 栈大小，MQTT和WiFi可能需要更多
    NULL,
    1,
    &get_commendHandle,
    0
  );
  xTaskCreatePinnedToCore(
    dhtTask,
    "DHT_MQTT_Task", // 任务名更新
    4096,            // 栈大小，MQTT和WiFi可能需要更多
    NULL,
    1,
    &dhtTaskHandle,
    0
  );
  xTaskCreatePinnedToCore(
    ICM45686_getdata,
    "ICM45686_getdata", // 任务名更新
    4096,            // 栈大小，MQTT和WiFi可能需要更多
    NULL,
    1,
    &ICM45686Handle,
    0
  );
  xTaskCreatePinnedToCore(
    hongwai_get,
    "hongwai_get",
    4096,
    NULL,
    1,
    &hongwaiHandle,
    0
  );
  xTaskCreatePinnedToCore(
    smog_get,
    "smog_get",
    4096,
    NULL,
    1,
    &SmogHanle,
    0
  );
  if(SmogHanle!=NULL)
  {
    Serial.println("smog_get任务创建成功");
  }
  else
  { 
    Serial.println("smog_get任务创建失败");    
  }
  if(hongwaiHandle!=NULL)
  {
    Serial.println("hongwai_get任务创建成功");
  }
  else
  {
    Serial.println("hongwai_get任务创建失败");
  }
  if(get_commendHandle!=NULL)
  {
    Serial.println("get_commend任务创建成功");
  }
  else
  {
    Serial.println("get_commend任务创建失败");
  }
  if(dhtTaskHandle!=NULL)
  {
    Serial.println("dhtTask任务创建成功");
  }
  else
  {
    Serial.println("dhtTask任务创建失败");
  }
  if(LD3320Handle!=NULL)
  {
    Serial.println("LD3320任务创建成功");
  }
  else
  {
    Serial.println("LD3320任务创建失败");
  }
  if(ICM45686Handle!=NULL)
  {
    Serial.println("ICM45686任务创建成功");
  }
  else
  {
    Serial.println("ICM45686任务创建失败");
  }
}

void loop() {
  // 主循环可以保持空，因为主要逻辑在FreeRTOS任务中

 
}
