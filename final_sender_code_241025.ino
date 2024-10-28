#include <SPI.h>
#include <LoRa.h>
#include "CRC16.h"
#include "Adafruit_BMP085.h"
#include "DHT.h"
#include <TimeLib.h>

// 센서 핀 정의
#define DHTPIN 4        // DHT11 센서의 데이터 핀
#define DHTTYPE DHT11   // DHT 센서 타입 정의
#define LORA_SS 10      // LoRa SS 핀
#define LORA_RST 9      // LoRa RESET 핀
#define LORA_DIO0 2     // LoRa DIO0 핀
#define WIND_SPEED_PIN 3 // 풍속계 인터럽트 핀
#define RAIN_GAUGE_PIN 5 // 우량계 인터럽트 핀
#define WIND_DIR_PIN A0  // 풍향계 아날로그 핀

// 풍향 전압 테이블 (16방위)
const float windDirTable[] = {
   3.84, 1.98, 2.25, 0.41, 0.45, 0.32, 0.90, 0.62,
   1.40, 1.19, 3.08, 2.93, 4.62, 4.04, 4.33, 3.43
};

const float windDirDegrees[] = {
   0, 22.5, 45, 67.5, 90, 112.5, 135, 157.5,
   180, 202.5, 225, 247.5, 270, 292.5, 315, 337.5
};

// 디바이스 ID 설정
byte sender_id[] = {0xB2, 0x41, 0x00, 0x01}; // B2410001
byte receive_id[] = {0xA2, 0x41, 0x00, 0x01}; // A2410001

// 전역 변수
volatile unsigned long windRotations = 0;
volatile unsigned long rainCount = 0;
volatile unsigned long lastWindTime = 0;
volatile unsigned long lastRainTime = 0;

// 객체 초기화
CRC16 crc;
Adafruit_BMP085 bmp;
DHT dht(DHTPIN, DHTTYPE);

void setup() {
   Serial.begin(9600);
   while(!Serial); // 시리얼 포트가 준비될 때까지 대기
   
   // 시간 설정 (2024년 10월 25일 9시 20분으로 설정)
   setTime(9, 20, 0, 25, 10, 2024);
   
   Serial.println("Initializing BMP180 sensor...");
   if (!bmp.begin()) {
       Serial.println("BMP180 initialization failed! Check wiring:");
       Serial.println("- SDA to A4");
       Serial.println("- SCL to A5");
       Serial.println("- VCC to 3.3V or 5V");
       Serial.println("- GND to GND");
       while (1);
   }
   Serial.println("BMP180 initialization successful!");

   // 초기 압력값 테스트
   long testPressure = bmp.readPressure();
   Serial.println("Initial pressure reading (Pa): " + String(testPressure));
   Serial.println("Initial pressure reading (hPa): " + String(testPressure/100.0));
   
   // DHT11 초기화
   dht.begin();
   Serial.println("DHT11 initialization complete");
   
   // 인터럽트 핀 설정
   pinMode(WIND_SPEED_PIN, INPUT_PULLUP);
   pinMode(RAIN_GAUGE_PIN, INPUT_PULLUP);
   attachInterrupt(digitalPinToInterrupt(WIND_SPEED_PIN), windSpeedIsr, FALLING);
   attachInterrupt(digitalPinToInterrupt(RAIN_GAUGE_PIN), rainGaugeIsr, FALLING);
   
   // LoRa 초기화
   LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
   if (!LoRa.begin(920.9E6)) {
       Serial.println("LoRa initialization failed!");
       while (1);
   }
   
   // LoRa 설정
   LoRa.setTxPower(20);
   LoRa.setSpreadingFactor(7);
   LoRa.setSignalBandwidth(125E3);
   LoRa.setCodingRate4(5);
   
   Serial.println("Weather Station (Transmitter) Initialized");
}

void loop() {
   static unsigned long lastSendTime = 0;
   if (millis() - lastSendTime >= 60000) { // 1분마다 전송
       sendWeatherData();
       lastSendTime = millis();
   }
}

void sendWeatherData() {
   // DHT11 안정화를 위한 지연
   delay(2000);
   
   // 기압 센서 읽기 시도
   long rawPressure = bmp.readPressure();  // Pa 단위로 읽기
   float pressure;
   
   Serial.println("\nPressure Sensor Debug:");
   Serial.println("Raw Pressure (Pa): " + String(rawPressure));
   
   if (rawPressure < 0 || rawPressure > 110000) {  // Pa 단위 유효 범위 체크
       Serial.println("Invalid pressure reading! Using default value.");
       pressure = 1013.25; // 기본값 (hPa)
   } else {
       pressure = rawPressure / 100.0;  // Pa를 hPa로 변환
   }

   // 나머지 센서 데이터 읽기
   float humidity = dht.readHumidity();
   float temperature = dht.readTemperature();
   float windSpeed = calculateWindSpeed();
   float windDirection = calculateWindDirection();
   float rainfall = calculateRainfall();
   
   // 센서값 유효성 검사 및 범위 체크
   if (isnan(humidity) || humidity < 0 || humidity > 100) {
       Serial.println("Invalid humidity reading!");
       humidity = 0;
   }
   if (isnan(temperature) || temperature < -40 || temperature > 80) {
       Serial.println("Invalid temperature reading!");
       temperature = 0;
   }
   
   // 디버그 출력
   Serial.println("\nProcessed Readings:");
   Serial.println("Pressure (hPa): " + String(pressure, 2));
   Serial.println("Temperature (°C): " + String(temperature, 2));
   Serial.println("Humidity (%): " + String(humidity, 2));
   Serial.println("Wind Speed (m/s): " + String(windSpeed, 2));
   Serial.println("Wind Direction (°): " + String(windDirection, 2));
   Serial.println("Rainfall (mm): " + String(rainfall, 2));

   // 패킷 생성 전에 최종 값 확인
   if (pressure < 800 || pressure > 1100) {
       Serial.println("Final pressure check failed! Using standard pressure.");
       pressure = 1013.25;
   }
   
   // 패킷 생성
   byte packet[40];
   int idx = 0;
   
   // 헤더 추가
   memcpy(packet + idx, sender_id, 4); idx += 4;
   memcpy(packet + idx, receive_id, 4); idx += 4;
   
   // 센서 데이터 추가
   addFloatToPacket(packet, idx, pressure);
   addFloatToPacket(packet, idx, humidity);
   addFloatToPacket(packet, idx, temperature);
   addFloatToPacket(packet, idx, windSpeed);
   addFloatToPacket(packet, idx, windDirection);
   addFloatToPacket(packet, idx, rainfall);
   
   // 타임스탬프 추가
   char timestamp[20];
   sprintf(timestamp, "%04d-%02d-%02d %02d:%02d:%02d",
           year(), month(), day(), hour(), minute(), second());
   memcpy(packet + idx, timestamp, strlen(timestamp));
   idx += strlen(timestamp);
   
   // CRC 추가
   crc.reset();
   crc.setPolynome(0x1021);
   for (int i = 0; i < idx; i++) {
       crc.add(packet[i]);
   }
   uint16_t crcValue = crc.getCRC();
   packet[idx++] = crcValue >> 8;
   packet[idx++] = crcValue & 0xFF;
   
   // 전송
   LoRa.beginPacket();
   LoRa.write(packet, idx);
   LoRa.endPacket(true);
   
   // 카운터 초기화
   windRotations = 0;
   rainCount = 0;
}

void addFloatToPacket(byte* packet, int& index, float value) {
   int16_t intValue = (int16_t)(value * 100);
   packet[index++] = intValue >> 8;
   packet[index++] = intValue & 0xFF;
}

float calculateWindSpeed() {
   float rotationsPerSecond = windRotations / 60.0;
   return rotationsPerSecond * 2.4;
}

float calculateWindDirection() {
   float voltage = analogRead(WIND_DIR_PIN) * 5.0 / 1023.0;
   float minDiff = 5.0;
   int direction = 0;
   
   // 디버그 출력
   Serial.println("\nWind Direction Calculation:");
   Serial.println("ADC Value: " + String(analogRead(WIND_DIR_PIN)));
   Serial.println("Voltage: " + String(voltage));
   
   // 가장 가까운 방향 찾기
   for (int i = 0; i < 16; i++) {
       float diff = abs(voltage - windDirTable[i]);
       Serial.println("Direction " + String(i) + ": diff = " + String(diff));
       if (diff < minDiff) {
           minDiff = diff;
           direction = i;
       }
   }
   
   // 선택된 방향 출력
   Serial.println("Selected Direction Index: " + String(direction));
   Serial.println("MinDiff: " + String(minDiff));
   
   // 너무 큰 차이가 있으면 0 반환
   if (minDiff > 4.0) {
       Serial.println("Invalid wind direction reading!");
       return 0;
   }
   
   return direction * 22.5;
}

float calculateRainfall() {
   return rainCount * 0.2794;
}

void windSpeedIsr() {
   if (millis() - lastWindTime > 10) {
       windRotations++;
       lastWindTime = millis();
   }
}

void rainGaugeIsr() {
   if (millis() - lastRainTime > 10) {
       rainCount++;
       lastRainTime = millis();
   }
}