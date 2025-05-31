#include "esp_camera.h"
#include <WiFi.h>
#include <HTTPClient.h> // 添加HTTP客户端库

//
// WARNING!!! PSRAM IC required for UXGA resolution and high JPEG quality
//            Ensure ESP32 Wrover Module or other board with PSRAM is selected
//            Partial images will be transmitted if image exceeds buffer size
//
//            You must select partition scheme from the board menu that has at least 3MB APP space.
//            Face Recognition is DISABLED for ESP32 and ESP32-S2, because it takes up from 15
//            seconds to process single frame. Face Detection is ENABLED if PSRAM is enabled as well

// ===================
// Select camera model
// ===================
//#define CAMERA_MODEL_WROVER_KIT // Has PSRAM
//#define CAMERA_MODEL_ESP_EYE  // Has PSRAM
//#define CAMERA_MODEL_ESP32S3_EYE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_PSRAM // Has PSRAM
//#define CAMERA_MODEL_M5STACK_V2_PSRAM // M5Camera version B Has PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_ESP32CAM // No PSRAM
//#define CAMERA_MODEL_M5STACK_UNITCAM // No PSRAM
//#define CAMERA_MODEL_M5STACK_CAMS3_UNIT  // Has PSRAM
#define CAMERA_MODEL_AI_THINKER // Has PSRAM
//#define CAMERA_MODEL_TTGO_T_JOURNAL // No PSRAM
//#define CAMERA_MODEL_XIAO_ESP32S3 // Has PSRAM
// ** Espressif Internal Boards **
//#define CAMERA_MODEL_ESP32_CAM_BOARD
//#define CAMERA_MODEL_ESP32S2_CAM_BOARD
//#define CAMERA_MODEL_ESP32S3_CAM_LCD
//#define CAMERA_MODEL_DFRobot_FireBeetle2_ESP32S3 // Has PSRAM
//#define CAMERA_MODEL_DFRobot_Romeo_ESP32S3 // Has PSRAM
#include "camera_pins.h"

// ===========================
// 校园网WiFi认证配置
// ===========================
const char* ssid = "whvcse";         // 校园网WiFi名称
const char* password = ""; // WiFi密码为空 (开放网络)

// 校园网认证配置
const char* campusAuthUrl = "http://10.80.1.122/eportal/InterFace.do?method=login";
const char* campusAuthPostData = "userId=23429501151389&password=123456&service=after-auth&queryString=wlanuserip%3D10.36.204.166%26wlanacname%3D%26nasip%3D172.10.1.1%26wlanparameter%3Df4-c8-8a-3f-80-f9%26url%3Dhttp%3A%2F%2Fwww.msftconnecttest.com%2Fredirect%26userlocation%3Dethtrunk%2F200%3A1816.0&operatorPwd=&operatorUserId=&validcode=&passwordEncrypt=false";

// 认证状态标志
bool campusAuthSuccessful = false;

void startCameraServer();
void setupLedFlash(int pin);

// 校园网认证函数
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

// 测试互联网连接
bool testInternetConnection() {
  HTTPClient httpTest;
  Serial.print("Attempting to connect to www.baidu.com...");
  httpTest.begin("http://www.baidu.com");
  int httpTestResponseCode = httpTest.GET();

  if (httpTestResponseCode > 0) {
    Serial.print(" HTTP Response code: ");
    Serial.println(httpTestResponseCode);
    if (httpTestResponseCode == HTTP_CODE_OK || httpTestResponseCode == HTTP_CODE_MOVED_PERMANENTLY || httpTestResponseCode == 302) {
      Serial.println("Successfully connected to Baidu. Internet access confirmed.");
      httpTest.end();
      return true;
    } else {
      Serial.println("Connected to Baidu, but got an unexpected HTTP status code.");
    }
  } else {
    Serial.print("Failed to connect to Baidu, error: ");
    Serial.println(httpTest.errorToString(httpTestResponseCode).c_str());
  }
  httpTest.end();
  return false;
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_VGA;
  config.pixel_format = PIXFORMAT_JPEG;  // for streaming
  //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.jpeg_quality = 12;
      config.fb_count = 1;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      // Limit the frame size when PSRAM is not available
      config.frame_size = FRAMESIZE_VGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    // Best option for face detection/recognition
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);        // flip it back
    s->set_brightness(s, 1);   // up the brightness just a bit
    s->set_saturation(s, -2);  // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  if (config.pixel_format == PIXFORMAT_JPEG) {
    s->set_framesize(s, FRAMESIZE_QVGA);
  }

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif

// Setup LED FLash if LED pin is defined in camera_pins.h
#if defined(LED_GPIO_NUM)
  setupLedFlash(LED_GPIO_NUM);
#endif

  // 连接WiFi
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  Serial.print("WiFi connecting");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  Serial.println("");
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    
    // 执行校园网认证
    campusAuthSuccessful = performCampusAuthentication();
    
    if (!campusAuthSuccessful) {
      Serial.println("Authentication failed. MQTT might not work if internet access is restricted.");
    } else {
      Serial.println("Campus authentication successful. Testing internet connectivity...");
      testInternetConnection();
    }
  } else {
    Serial.println("Failed to connect to WiFi.");
  }

  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
}

void loop() {
  // Do nothing. Everything is done in another task by the web server
  delay(10000);
}
