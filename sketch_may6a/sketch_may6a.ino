// 引入 DHT 库
#include "DHT.h"
// 引入 WiFi 和 HTTPClient 库
#include <WiFi.h>
#include <HTTPClient.h>
#include "LD3320.h"
#include "Arduino.h"
// 引入 MQTT 客户端库
#include <PubSubClient.h>

// --- WiFi 配置 ---
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
// const char* mqttUser = "your_mqtt_user";    // 如果您的MQTT代理需要认证
// const char* mqttPassword = "your_mqtt_password"; // 如果您的MQTT代理需要认证
TaskHandle_t dhtTaskHandle = NULL;
TaskHandle_t get_commendHandle =NULL;
char flag_temperature=0;


// --- MQTT代理配置结束 ---

// 定义 DHT 传感器连接的引脚
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

/* ------------------------------------------------------------------------------------------------------------LD3320---------------------------------------------------------------------------------------------------------------------*/
TaskHandle_t LD3320Handle=NULL;
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
      case CODE_DMCS:     /*命令“代码测试”*/
          Serial.println("dai ma ce shi\r\n"); /*text.....*/
                        break;
      case CODE_CSWB:     /*命令“测试完毕”*/
          Serial.println("ce shi wan bi\r\n"); /*text.....*/
                        break;
      
      case CODE_1KL1:  /*命令“北京”*/
          Serial.println("bei jing\r\n"); /*text.....*/
          sprintf(msgBuffer, "%s", "bei jing");
          mqttClient.publish(mqttspeakTopic, msgBuffer);
                        break;
      case CODE_1KL2:   /*命令“上海”*/
    
          Serial.println("shang hai\r\n"); /*text.....*/
                        break;
      case CODE_1KL3:  /*命令“开灯”*/
          Serial.println("kai deng\r\n"); /*text.....*/
                        break;
      case CODE_1KL4:   /*命令“关灯”*/        
          Serial.println("guan deng\r\n"); /*text.....*/
                        break;
      
      case CODE_2KL1:  /*命令“....”*/
          Serial.println("guang zhou\r\n"); /*text.....*/
                        break;
      case CODE_2KL2:  /*命令“....”*/
          Serial.println("shen zhen\r\n"); /*text.....*/
                        break;
      case CODE_2KL3:  /*命令“....”*/
          Serial.println("xiang zuo zhuan\r\n"); /*text.....*/
                        break;
      case CODE_2KL4:  /*命令“....”*/
          Serial.println("xiang you zhuan\r\n"); /*text.....*/
                              break;
            
      case CODE_3KL1:  /*命令“....”*/
          Serial.println("da kai kong tiao\r\n"); /*text.....*/
                        break;
      case CODE_3KL2:  /*命令“....”*/
          Serial.println("guan bi kong tiao\r\n"); /*text.....*/
                        break;
      case CODE_5KL1:  /*命令“....”*/
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
  delay(10);
  Serial.println();
  Serial.print("Connecting to WiFi network: ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.print("Attempting to connect");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
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
      delay(5000); // 在重连循环中使用常规delay是可以接受的
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


void dhtTask(void *pvParameters) {
  
  while(1){
   
    if(flag_temperature==1)
    {
      flag_temperature=0;
      float t = dht.readTemperature(); // 读取当前温度
      if (!isnan(t)) {
      char msgBuffer[10];
      dtostrf(t, 5, 2, msgBuffer); // 转换温度为字符串，保留两位小数 (例如 "25.50")
      mqttClient.publish(mqttTemperatureTopic, msgBuffer);
      Serial.print("当前温度是: ");
      Serial.println(msgBuffer);
    }
    if(t>=23)
    {
      digitalWrite(36,LOW);
    }
    else {
      digitalWrite(36,HIGH);
    }
    }
    vTaskDelay(pdMS_TO_TICKS(1000)); // 每10秒读取一次并发布
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

  dht.begin(); // 初始化   DHT 传感器
  LD3320_init();
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
}

void loop() {
  // 主循环可以保持空，因为主要逻辑在FreeRTOS任务中

 
}
