#include <Wire.h>
#include "../i2c_link_config.h"

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("\n[DIAG] ESP32 I2C probe/scanner start");
  Serial.println(String("[DIAG] SDA=") + i2cLinkCfg::ESP32_SDA_PIN + ", SCL=" + i2cLinkCfg::ESP32_SCL_PIN);
  Serial.println(String("[DIAG] Expected Nano address: 0x") + String(i2cLinkCfg::NANO_SLAVE_ADDRESS, HEX));

  Wire.begin(i2cLinkCfg::ESP32_SDA_PIN, i2cLinkCfg::ESP32_SCL_PIN, i2cLinkCfg::BUS_FREQUENCY_HZ);

  for (uint8_t i = 1; i < 127; i++) {
    Wire.beginTransmission(i);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      Serial.println(String("[DIAG] ACK at 0x") + String(i, HEX));
    }
  }

  Wire.beginTransmission(i2cLinkCfg::NANO_SLAVE_ADDRESS);
  uint8_t err = Wire.endTransmission();
  Serial.println(String("[DIAG] Nano probe code=") + err + (err == 0 ? " (ACK)" : " (NO ACK)"));
}

void loop() {
  delay(1000);
}
