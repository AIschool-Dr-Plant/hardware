#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <SPI.h>
#include <LoRa.h>
#include "CRC16.h"

// WiFi 설정
const char* ssid = "aischool class03(2.4Ghz)";
const char* password = "gjaischool";
const char* api_server = "http://123.100.174.98:8155/api/test";

// LoRa 핀 설정 (Wemos D1)
#define SS D8
#define RST D0
#define DIO0 D1

// 객체 초기화
CRC16 crc;
WiFiClient wifiClient;
HTTPClient http;

//// 디바이스 ID (수신기)
// const String device_id = "A2410001";
// 디바이스 ID (송신기) - 서버에 전송해야 하는 장치정보는 송신기.
const String device_id = "B2410001";

void setup() {
   Serial.begin(115200);
   while(!Serial); // 시리얼 포트가 준비될 때까지 대기
   
   // WiFi 연결
   Serial.print("Connecting to WiFi");
   WiFi.mode(WIFI_STA);
   WiFi.begin(ssid, password);
   while (WiFi.status() != WL_CONNECTED) {
       delay(500);
       Serial.print(".");
   }
   Serial.println("\nWiFi connected");
   Serial.println("IP address: " + WiFi.localIP().toString());
   
   // LoRa 초기화
   Serial.println("Initializing LoRa...");
   LoRa.setPins(SS, RST, DIO0);
   if (!LoRa.begin(920.9E6)) {
       Serial.println("LoRa initialization failed!");
       while (1);
   }
   
   // LoRa 설정
   LoRa.setSpreadingFactor(7);
   LoRa.setSignalBandwidth(125E3);
   LoRa.setCodingRate4(5);
   
   Serial.println("Weather Station (Receiver) Initialized");
}

void loop() {
   int packetSize = LoRa.parsePacket();
   if (packetSize) {
       Serial.println("\nReceived packet:");
       processLoRaPacket(packetSize);
   }
}

void processLoRaPacket(int packetSize) {
   byte packet[50];
   int idx = 0;
   
   while (LoRa.available()) {
       packet[idx++] = LoRa.read();
   }
   
   // CRC 검증
   uint16_t receivedCRC = (packet[idx-2] << 8) | packet[idx-1];
   crc.reset();
   crc.setPolynome(0x1021);
   for (int i = 0; i < idx-2; i++) {
       crc.add(packet[i]);
   }
   
   if (receivedCRC != crc.getCRC()) {
       Serial.println("CRC Error!");
       return;
   }
   
   // 데이터 파싱
   int dataIdx = 8;  // ID 건너뛰기
   
   // 송신부에서 보낸 값 그대로 받기
   int16_t pressureInt = (packet[dataIdx] << 8) | packet[dataIdx + 1];
   float pressure = pressureInt / 100.0;
   dataIdx += 2;
   
   int16_t humidityInt = (packet[dataIdx] << 8) | packet[dataIdx + 1];
   float humidity = humidityInt / 100.0;
   dataIdx += 2;
   
   int16_t temperatureInt = (packet[dataIdx] << 8) | packet[dataIdx + 1];
   float temperature = temperatureInt / 100.0;
   dataIdx += 2;
   
   int16_t windSpeedInt = (packet[dataIdx] << 8) | packet[dataIdx + 1];
   float windSpeed = windSpeedInt / 100.0;
   dataIdx += 2;
   
   int16_t windDirectionInt = (packet[dataIdx] << 8) | packet[dataIdx + 1];
   float windDirection = windDirectionInt / 100.0;
   dataIdx += 2;
   
   int16_t rainfallInt = (packet[dataIdx] << 8) | packet[dataIdx + 1];
   float rainfall = rainfallInt / 100.0;
   dataIdx += 2;
   
   // 값 검증
   if (pressure < 800 || pressure > 1100) {
       Serial.println("Invalid pressure value received. Using standard pressure.");
       pressure = 1013.25; // 비정상적인 값이면 표준 기압 사용
   }
   
   // 타임스탬프 추출
   char timestamp[20];
   memcpy(timestamp, packet + dataIdx, 19);
   timestamp[19] = '\0';
   
   // 디버그 출력
   Serial.println("\nProcessed Sensor Data:");
   Serial.println("Raw Pressure Int: " + String(pressureInt));
   Serial.println("Pressure: " + String(pressure, 2) + " hPa");
   Serial.println("Humidity: " + String(humidity, 2) + " %");
   Serial.println("Temperature: " + String(temperature, 2) + " °C");
   Serial.println("Wind Speed: " + String(windSpeed, 2) + " m/s");
   Serial.println("Wind Direction: " + String(windDirection, 2) + "°");
   Serial.println("Rainfall: " + String(rainfall, 2) + " mm");
   Serial.println("Timestamp: " + String(timestamp));
   
   // API 서버로 전송
   sendToServer(pressure, humidity, temperature, 
               windSpeed, windDirection, rainfall, 
               String(timestamp));
}

void sendToServer(float pressure, float humidity, float temperature,
                float windSpeed, float windDirection, float rainfall,
                String measureDate) {
   if (WiFi.status() != WL_CONNECTED) {
       Serial.println("WiFi connection lost!");
       return;
   }
   
   String url = String(api_server);
   url += "?A=" + String(pressure, 2);
   url += "&B=" + String(humidity, 2);
   url += "&C=" + String(temperature, 2);
   url += "&D=" + String(windSpeed, 2);
   url += "&E=" + String(windDirection, 2);
   url += "&F=" + String(rainfall, 2);
   url += "&G=" + measureDate;
   url += "&H=" + device_id;
   
   Serial.println("Sending to server: " + url);
   
   http.begin(wifiClient, url);
   int httpCode = http.GET();
   
   if (httpCode > 0) {
       String response = http.getString();
       Serial.println("HTTP Response code: " + String(httpCode));
       Serial.println("Response: " + response);
   } else {
       Serial.println("HTTP Request failed");
   }
   
   http.end();
}
