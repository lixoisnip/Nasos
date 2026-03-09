#include <Wire.h>
#include "../i2c_link_config.h"

volatile uint8_t counter = 0;

void onRequest() {
  Wire.write(counter++);
}

void onReceive(int len) {
  while (Wire.available()) (void)Wire.read();
  Serial.print("[DIAG] RX bytes=");
  Serial.println(len);
}

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("\n[DIAG] Nano minimal I2C slave start");
  Serial.print("[DIAG] Address: 0x");
  Serial.println(i2cLinkCfg::NANO_SLAVE_ADDRESS, HEX);
  Wire.begin(i2cLinkCfg::NANO_SLAVE_ADDRESS);
  Wire.onRequest(onRequest);
  Wire.onReceive(onReceive);
  Serial.println("[DIAG] Wire.begin(slaveAddress) done");
}

void loop() {
  static unsigned long last = 0;
  if (millis() - last > 2000) {
    last = millis();
    Serial.println("[DIAG] Nano minimal slave alive");
  }
}
