#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "M5EPD.h"
#include <Wire.h>
#include <Adafruit_GPS.h>
#include <Adafruit_MLX90614.h>
#include <GSMSimSMS.h>
#include "fonction.h"

#define AXP192_ADDR 0x34

// ================= AXP192 =================
void writeAXP(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(AXP192_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void enable5VPortA() {
  writeAXP(0x12, 0xFF);
  delay(100);
  Serial.println("5V Port A activé");
}

