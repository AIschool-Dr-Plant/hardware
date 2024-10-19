#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_BT");

  // Bluetooth 연결 여부 확인
  if (SerialBT.hasClient()) {
    Serial.println("Bluetooth 연결됨");
  } else {
    Serial.println("Bluetooth 연결되지 않음");
  }
}

void loop() {
  // 여기에 추가적인 코드를 넣을 수 있습니다.
}

