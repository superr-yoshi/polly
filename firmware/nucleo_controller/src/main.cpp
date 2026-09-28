#include <Arduino.h>
#include <Adafruit_BNO08x.h>

Adafruit_BNO08x bno08x(-1);

void setup() {
  Serial.begin(115200);

  if (!bno08x.begin_I2C()) {
    Serial.println("BNO085 NOT FOUND");
  } else {
    Serial.println("BNO085 FOUND");
  }
}

void loop() {
  Serial.println("POLI NUCLEO READY");
  delay(1000);
}