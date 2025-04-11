#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>

const char* ssid = "doyun";
const char* password = "11111111";

const char* websocket_server = "172.20.10.2";
const uint16_t websocket_port = 12345;

const int RED_PIN = 33; 
const int GREEN_PIN = 26;
const int BLUE_PIN = 27;

const int PWM_CHANNEL_R = 0;
const int PWM_CHANNEL_G = 1;
const int PWM_CHANNEL_B = 2;
const int PWM_RESOLUTION = 8;
const int PWM_FREQUENCY = 5000;

WebSocketsClient webSocket;

void setupRGBLed() {
  ledcSetup(PWM_CHANNEL_R, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_G, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_B, PWM_FREQUENCY, PWM_RESOLUTION);
  
  ledcAttachPin(RED_PIN, PWM_CHANNEL_R);
  ledcAttachPin(GREEN_PIN, PWM_CHANNEL_G);
  ledcAttachPin(BLUE_PIN, PWM_CHANNEL_B);
  
  ledcWrite(PWM_CHANNEL_R, 0);
  ledcWrite(PWM_CHANNEL_G, 0);
  ledcWrite(PWM_CHANNEL_B, 0);
}

int mapBrightness(int brightness) {
  return map(brightness, 0, 100, 0, 255);
}

void setColorByName(String color, int brightness) {
  int brightnessValue = mapBrightness(brightness);
  
  Serial.print("색상 설정: ");
  Serial.print(color);
  Serial.print(", 밝기: ");
  Serial.println(brightness);
  
  if (color == "Red") {
    ledcWrite(PWM_CHANNEL_R, brightnessValue);
    ledcWrite(PWM_CHANNEL_G, 0);
    ledcWrite(PWM_CHANNEL_B, 0);
    Serial.println("RGB LED: 빨간색");
  } 
  else if (color == "Green") {
    ledcWrite(PWM_CHANNEL_R, 0);
    ledcWrite(PWM_CHANNEL_G, brightnessValue);
    ledcWrite(PWM_CHANNEL_B, 0);
    Serial.println("RGB LED: 녹색");
  } 
  else if (color == "Blue") {
    ledcWrite(PWM_CHANNEL_R, 0);
    ledcWrite(PWM_CHANNEL_G, 0);
    ledcWrite(PWM_CHANNEL_B, brightnessValue);
    Serial.println("RGB LED: 파란색");
  } 
  else if (color == "Yellow") {
    ledcWrite(PWM_CHANNEL_R, brightnessValue);
    ledcWrite(PWM_CHANNEL_G, brightnessValue);
    ledcWrite(PWM_CHANNEL_B, 0);
    Serial.println("RGB LED: 노란색");
  } 
  else if (color == "Purple") {
    ledcWrite(PWM_CHANNEL_R, brightnessValue);
    ledcWrite(PWM_CHANNEL_G, 0);
    ledcWrite(PWM_CHANNEL_B, brightnessValue);
    Serial.println("RGB LED: 보라색");
  } 
  else if (color == "White") {
    ledcWrite(PWM_CHANNEL_R, brightnessValue);
    ledcWrite(PWM_CHANNEL_G, brightnessValue);
    ledcWrite(PWM_CHANNEL_B, brightnessValue);
    Serial.println("RGB LED: 흰색");
  } 
  else {
    ledcWrite(PWM_CHANNEL_R, brightnessValue);
    ledcWrite(PWM_CHANNEL_G, brightnessValue);
    ledcWrite(PWM_CHANNEL_B, brightnessValue);
    Serial.println("RGB LED: 알 수 없는 색상, 흰색으로 설정");
  }
}

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.println("웹소켓 연결 해제!");
      delay(1000);
      webSocket.begin(websocket_server, websocket_port, "/");
      break;
      
    case WStype_CONNECTED:
      {
        Serial.println("웹소켓 서버에 연결됨");
        
        DynamicJsonDocument doc(1024);
        doc["type"] = "register";
        doc["client_type"] = "lamp";
        
        String registerMessage;
        serializeJson(doc, registerMessage);
        webSocket.sendTXT(registerMessage);
        Serial.println("등록 메시지 전송: " + registerMessage);
      }
      break;
      
    case WStype_TEXT:
      {
        Serial.println("수신된 메시지: " + String((char*)payload));
        
        DynamicJsonDocument doc(1024);
        DeserializationError error = deserializeJson(doc, payload);
        
        if (error) {
          Serial.print("deserializeJson() 실패: ");
          Serial.println(error.c_str());
          return;
        }
        
        String messageType = doc["type"];
        
        if (messageType == "lamp") {
          String status = doc["status"];
          int brightness = doc["brightness"];
          String color = doc["color"];
          
          Serial.print("램프 상태: ");
          Serial.println(status);
          Serial.print("밝기: ");
          Serial.println(brightness);
          Serial.print("색상: ");
          Serial.println(color);
          
          if (status == "ON") {
            setColorByName(color, brightness);
          } else {
            ledcWrite(PWM_CHANNEL_R, 0);
            ledcWrite(PWM_CHANNEL_G, 0);
            ledcWrite(PWM_CHANNEL_B, 0);
            Serial.println("RGB LED 꺼짐");
          }
        }
        else if (messageType == "control") {
          String device = doc["device"];
          bool turned = doc["data"]["turned"];
          int value = doc["data"]["value"];
          
          Serial.print("디바이스: ");
          Serial.println(device);
          Serial.print("상태: ");
          Serial.println(turned ? "켜짐" : "꺼짐");
          Serial.print("값: ");
          Serial.println(value);
          
          if (device == "rgbLED") {
            if (!turned) {
              ledcWrite(PWM_CHANNEL_R, 0);
              ledcWrite(PWM_CHANNEL_G, 0);
              ledcWrite(PWM_CHANNEL_B, 0);
              Serial.println("RGB LED 꺼짐");
            } else {
              if (value <= 30) {
                ledcWrite(PWM_CHANNEL_R, 0);
                ledcWrite(PWM_CHANNEL_G, 0);
                ledcWrite(PWM_CHANNEL_B, 255);
                Serial.println("RGB LED: 파란색");
              } else if (value <= 60) {
                ledcWrite(PWM_CHANNEL_R, 0);
                ledcWrite(PWM_CHANNEL_G, 128);
                ledcWrite(PWM_CHANNEL_B, 255);
                Serial.println("RGB LED: 청록색");
              } else if (value <= 90) {
                ledcWrite(PWM_CHANNEL_R, 0);
                ledcWrite(PWM_CHANNEL_G, 255);
                ledcWrite(PWM_CHANNEL_B, 0);
                Serial.println("RGB LED: 녹색");
              } else if (value <= 120) {
                ledcWrite(PWM_CHANNEL_R, 255);
                ledcWrite(PWM_CHANNEL_G, 255);
                ledcWrite(PWM_CHANNEL_B, 0);
                Serial.println("RGB LED: 노란색");
              } else {
                ledcWrite(PWM_CHANNEL_R, 255);
                ledcWrite(PWM_CHANNEL_G, 0);
                ledcWrite(PWM_CHANNEL_B, 0);
                Serial.println("RGB LED: 빨간색");
              }
            }
          }
        }
        else if (messageType == "register_response") {
          String status = doc["status"];
          Serial.print("등록 응답: ");
          Serial.println(status);
        }
      }
      break;
      
    case WStype_ERROR:
      Serial.println("웹소켓 오류 발생!");
      break;
      
    default:
      break;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("ESP32 IoT 클라이언트 시작");
  
  setupRGBLed();
  
  WiFi.begin(ssid, password);
  Serial.print("WiFi에 연결 중");
  
  int retry_count = 0;
  int max_retries = 20;
  
  while (WiFi.status() != WL_CONNECTED && retry_count < max_retries) {
    delay(500);
    Serial.print(".");
    retry_count++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("WiFi 연결됨, IP 주소: ");
    Serial.println(WiFi.localIP());
    
    webSocket.begin(websocket_server, websocket_port, "/");
    webSocket.onEvent(webSocketEvent);
    webSocket.setReconnectInterval(5000);  // 5초마다 재연결 시도
  } else {
    Serial.println();
    Serial.println("WiFi 연결 실패!");
    Serial.print("WiFi 상태 코드: ");
    Serial.println(WiFi.status());
  }
}

void loop() {
  webSocket.loop();
  
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi 연결이 끊어졌습니다. 재연결 시도 중...");
    WiFi.begin(ssid, password);
    
    int retry_count = 0;
    while (WiFi.status() != WL_CONNECTED && retry_count < 10) {
      delay(500);
      Serial.print(".");
      retry_count++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println();
      Serial.print("WiFi 재연결 성공, IP 주소: ");
      Serial.println(WiFi.localIP());
      
      webSocket.begin(websocket_server, websocket_port, "/");
    } else {
      Serial.println();
      Serial.println("WiFi 재연결 실패!");
    }
  }
}