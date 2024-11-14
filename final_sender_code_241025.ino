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
#define LORA_DIO0 8     // LoRa DIO0 핀
#define WIND_SPEED_PIN 2 // 풍속계 인터럽트 핀
#define RAIN_GAUGE_PIN 3 // 우량계 인터럽트 핀
#define WIND_DIR_PIN A0  // 풍향계 아날로그 핀

#define DEBOUNCE_TIME 10  // 디바운스 시간 (밀리초)

// 풍향 전압 테이블 (16방위)
const float windDirTable[] = {
    3.84, 1.98, 2.25, 0.41, 0.45, 0.32, 0.90, 0.62,
    1.40, 1.19, 3.08, 2.93, 4.62, 4.04, 4.33, 3.43
};

const float windDirDegrees[] = {
    0, 22.5, 45, 67.5, 90, 112.5, 135, 157.5,
    180, 202.5, 225, 247.5, 270, 292.5, 315, 337.5
};

// 송신기 ID를 문자열로 정의
const char sender_id[] = "B2411001";

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
    while(!Serial);
    
    // 시간 설정
    setTime(10, 43, 00, 14, 11, 2024);
    
    // BMP180 초기화
    if (!bmp.begin()) {
        Serial.println("BMP180 initialization failed!");
        while (1);
    }
    
    // DHT11 초기화
    dht.begin();
    
    // 인터럽트 핀 설정
    pinMode(WIND_SPEED_PIN, INPUT_PULLUP);
    pinMode(RAIN_GAUGE_PIN, INPUT_PULLUP);
    delay(100);
    
    attachInterrupt(digitalPinToInterrupt(WIND_SPEED_PIN), windSpeedIsr, FALLING);
    attachInterrupt(digitalPinToInterrupt(RAIN_GAUGE_PIN), rainGaugeIsr, FALLING);
    
    // LoRa 초기화
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
    if (millis() - lastSendTime >= 600000) {
        sendWeatherData();
        lastSendTime = millis();
    }
}

void sendWeatherData() {
    delay(2000);
    
    // 센서 데이터 읽기
    long rawPressure = bmp.readPressure();
    float pressure = (rawPressure < 0 || rawPressure > 110000) ? 1013.25 : (rawPressure / 100.0);
    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature();
    float windSpeed = calculateWindSpeed();
    float windDirection = calculateWindDirection();
    float rainfall = calculateRainfall();
    
    // 센서값 검증
    humidity = (isnan(humidity) || humidity < 0 || humidity > 100) ? 0 : humidity;
    temperature = (isnan(temperature) || temperature < -40 || temperature > 80) ? 0 : temperature;
    pressure = (pressure < 800 || pressure > 1100) ? 1013.25 : pressure;
    
    // 패킷 생성 (전체 크기를 50바이트로 고정)
    byte packet[50] = {0}; // 전체 패킷 크기를 40바이트로 설정
    int idx = 0;
    
    // 송신기 ID 추가 (8바이트 고정 길이로 추가)
    memcpy(packet + idx, sender_id, 8); // ID는 8바이트로 고정
    idx += 8;
    
    // 센서 데이터 추가 - 수정된 방식으로 추가
    addFloatToPacket(packet, idx, pressure, true);  // pressure는 특별 처리
    addFloatToPacket(packet, idx, humidity, false);
    addFloatToPacket(packet, idx, temperature, false);
    addFloatToPacket(packet, idx, windSpeed, false);
    addFloatToPacket(packet, idx, windDirection, false);
    addFloatToPacket(packet, idx, rainfall, false);
    
    // 타임스탬프 추가 (19바이트로 고정하여 복사)
    char timestamp[20] = {0}; // 배열을 0으로 초기화하여 문자열 끝에 NULL 포함
    snprintf(timestamp, sizeof(timestamp), "%04d-%02d-%02d %02d:%02d:%02d",
             year(), month(), day(), hour(), minute(), second());
    memcpy(packet + idx, timestamp, 19); // 19바이트 고정 길이 복사
    idx += 19;
    
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
    
    // 디버그 출력
    Serial.println("Data sent:");
    Serial.print("Sender ID: ");
    Serial.println(sender_id);
    Serial.print("Pressure: ");
    Serial.println(pressure, 2);
    Serial.print("Temperature: ");
    Serial.println(temperature, 2);
    Serial.print("Humidity: ");
    Serial.println(humidity, 2);
    Serial.print("Wind Speed: ");
    Serial.println(windSpeed, 2);
    Serial.print("Wind Direction: ");
    Serial.println(windDirection, 2);
    Serial.print("Rainfall: ");
    Serial.println(rainfall, 2);
    Serial.print("Timestamp: ");
    Serial.println(timestamp);
}

void addFloatToPacket(byte* packet, int& index, float value, bool isPressure) {
    // 대기압일 경우 10을 곱하고, 다른 값들은 100을 곱함
    int16_t intValue = (int16_t)(value * (isPressure ? 10 : 100));
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
    
    for (int i = 0; i < 16; i++) {
        float diff = abs(voltage - windDirTable[i]);
        if (diff < minDiff) {
            minDiff = diff;
            direction = i;
        }
    }
    
    return minDiff > 4.0 ? 0 : direction * 22.5;
}

float calculateRainfall() {
    return rainCount * 0.2794;
}

void windSpeedIsr() {
    unsigned long currentTime = millis();
    if (currentTime - lastWindTime > DEBOUNCE_TIME) {
        windRotations++;
        lastWindTime = currentTime;
    }
}

void rainGaugeIsr() {
    unsigned long currentTime = millis();
    if (currentTime - lastRainTime > DEBOUNCE_TIME) {
        rainCount++;
        lastRainTime = currentTime;
    }
}
