//============================ Alexandros Chrissantakis =============================
//=================================== 6TQA Inraci ===================================
//============================= MPU6050 PA1010D MLX90614 ============================
//=== I2Cdev MPU6050_6Axis_MotionApps20 M5EPD WIRE Adafruit_GPS Adafruit_MLX90614 ===
//===================== Code_SIPS_5_Alexandros_Chrissantakis.ino ====================
//==================================== 06-02-2026 ===================================

// ================= LIBRAIRIES =================
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "M5EPD.h"
#include <Wire.h>
#include <Adafruit_GPS.h>
#include <Adafruit_MLX90614.h>
#include <GSMSimSMS.h>
#include "fonction.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

// ================= DEFINES =================
#define RX 18
#define TX 19
#define SDA 32
#define SCL 25
#define AXP192_ADDR 0x34
#define INTERRUPT_PIN 2

// ================= INIT =================
static volatile int num = 0;
HardwareSerial SerialGSM(1);
GSMSim gsm(SerialGSM);
GSMSimSMS sms(SerialGSM);
Adafruit_GPS GPS(&Wire);
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
M5EPD_Canvas canvas(&M5.EPD);
uint32_t timer = 0;
MPU6050 mpu;  

// Variables MPU6050
bool DMPReady = false;
uint8_t devStatus;
uint16_t packetSize;
uint8_t FIFOBuffer[64];

// Variables pour orientation et mouvement
Quaternion q;                
VectorInt16 aa, aaReal, gy;  
VectorFloat gravity;         
float ypr[3];               

const byte RATE_SIZE = 4; 
byte rates[RATE_SIZE];    
byte rateSpot = 0;
long lastBeat = 0; 

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;

  Wire.begin(SDA, SCL);

  SerialGSM.begin(115200, SERIAL_8N1, RX, TX);

  M5.begin();
  enable5VPortA();
  M5.EPD.SetRotation(1);
  M5.EPD.Clear(true);

  canvas.createCanvas(540, 960); 
  canvas.setTextSize(2);
  canvas.setTextColor(15);

  canvas.drawString("Code Test", 10, 10);
  canvas.pushCanvas(0, 0, UPDATE_MODE_GC16);
  delay(1000);

  // ================= GPS =================
  if (!GPS.begin(0x10)) {
    Serial.println("PA1010D non connecté");
    canvas.drawString("PA1010D PAS OK", 10, 60);
    canvas.pushCanvas(0, 0, UPDATE_MODE_GC16);
  }
  Serial.println("PA1010D connecté");
  canvas.drawString("step 1", 10, 60);
  canvas.drawString("PA1010D OK", 10, 85);
  canvas.pushCanvas(0, 0, UPDATE_MODE_GC16);

  // ================= Configuration GPS =================
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  canvas.drawString("step 2", 10, 110);
  canvas.pushCanvas(0, 0, UPDATE_MODE_GC16);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  canvas.drawString("step 3", 10, 135);
  canvas.pushCanvas(0, 0, UPDATE_MODE_GC16);
  GPS.sendCommand(PGCMD_ANTENNA);
  canvas.drawString("step 4", 10, 160);
  canvas.pushCanvas(0, 0, UPDATE_MODE_GC16);
  canvas.clear();
  canvas.pushCanvas(0, 0, UPDATE_MODE_GC16);

  // ================= MLX90614 =================
  if (!mlx.begin()) {
    Serial.println("MLX90614 non connecté");
    canvas.drawString("MLX90614 PAS OK", 10, 85);
    canvas.pushCanvas(0, 0, UPDATE_MODE_GC16);
  }

  Serial.println("MLX90614 connecté");
  canvas.drawString("step 1", 10, 85);
  canvas.pushCanvas(0, 0, UPDATE_MODE_GC16);
  canvas.drawString("MLX90614 OK", 10, 110);
  canvas.pushCanvas(0, 0, UPDATE_MODE_GC16);
  canvas.clear();
  canvas.pushCanvas(0, 0, UPDATE_MODE_GC16);

  // ================= MPU6050 =================
  canvas.drawString("step 1", 10, 160);
  canvas.pushCanvas(0, 0, UPDATE_MODE_GC16);
  Serial.println("Initialisation MPU6050...");
  canvas.drawString("step 2", 10, 185);
  canvas.pushCanvas(0, 0, UPDATE_MODE_GC16);
  mpu.initialize();
  canvas.drawString("step 3", 10, 210);
  canvas.pushCanvas(0, 0, UPDATE_MODE_GC16);

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 non connecté");
    canvas.drawString("MPU6050 PAS OK", 10, 110);
    canvas.pushCanvas(0, 0, UPDATE_MODE_GC16);
  }

  Serial.println("MPU6050 connecté");

  canvas.clear();
  canvas.pushCanvas(0, 0, UPDATE_MODE_GC16);

  canvas.drawString("MPU6050 OK", 10, 110);
  canvas.drawString("Initialisation DMP...", 10, 135);
  canvas.pushCanvas(0, 0, UPDATE_MODE_GC16);

  // ================= DMP =================

  Serial.println("Initialisation DMP...");
  devStatus = mpu.dmpInitialize();
  canvas.drawString("Step 1", 10, 160);
  canvas.pushCanvas(0, 0, UPDATE_MODE_GC16);

  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    canvas.drawString("Step 2", 10, 185);
    canvas.pushCanvas(0, 0, UPDATE_MODE_GC16);
    mpu.CalibrateGyro(6);
    canvas.drawString("Step 3", 10, 210);
    canvas.pushCanvas(0, 0, UPDATE_MODE_GC16);
    mpu.setDMPEnabled(true);
    canvas.drawString("Step 4", 10, 235);
    canvas.pushCanvas(0, 0, UPDATE_MODE_GC16);
    DMPReady = true;
    canvas.drawString("Step 5", 10, 260);
    canvas.pushCanvas(0, 0, UPDATE_MODE_GC16);
    packetSize = mpu.dmpGetFIFOPacketSize();
    canvas.drawString("Step 6", 10, 285);
    canvas.pushCanvas(0, 0, UPDATE_MODE_GC16);
    Serial.println("DMP prêt");
    canvas.drawString("DMP OK", 10, 310);
    canvas.pushCanvas(0, 0, UPDATE_MODE_GC16);
  } else {
    Serial.print("Erreur DMP : ");
    canvas.drawString("DMP PAS OK", 10, 135);
    canvas.pushCanvas(0, 0, UPDATE_MODE_GC16);
    Serial.println(devStatus);
  }

  delay(500);
  canvas.clear();

  // ================= GSM =================

  gsm.init();  // init module

  String rep = gsm.sendATCommand("AT");
  Serial.println(rep);
  rep.trim();
  delay(100);

  if (rep != "OK") {
    Serial.println("SIM800C non connecté");
    canvas.drawString("SIM800C PAS OK", 10, 45);
    canvas.pushCanvas(0, 0, UPDATE_MODE_GC16);
  } else {
    Serial.println("SIM800C connecté");
    canvas.drawString("SIM800C OK", 10, 45);
    canvas.pushCanvas(0, 0, UPDATE_MODE_GC16);
  }

  Serial.print("is Module Registered to Network?... ");
  Serial.println(sms.isRegistered());
  delay(100);
  canvas.drawString("is Module Registered to Network?... ", 10, 60);
  canvas.drawString(String(sms.isRegistered()), 10, 85);
  canvas.pushCanvas(0, 0, UPDATE_MODE_GC16);

  Serial.print("Signal Quality... ");
  Serial.println(sms.signalQuality());
  delay(100);
  canvas.drawString("Signal Quality... ", 10, 110);
  canvas.drawString(String(sms.signalQuality()), 10, 135);
  canvas.pushCanvas(0, 0, UPDATE_MODE_GC16);

  Serial.print("Operator Name... ");
  Serial.println(sms.operatorNameFromSim());
  delay(100);
  canvas.drawString("Operator Name... ", 10, 160);
  canvas.drawString(String(sms.operatorNameFromSim()), 10, 185);
  canvas.pushCanvas(0, 0, UPDATE_MODE_GC16);
}

// ================= LOOP =================
void loop() {

  // ===== BAT =====
  int voltage = M5.getBatteryVoltage();
  int percent = ((voltage - 3000) * 100 / 1450);

  if (percent > 100) percent = 100;
  if (percent < 0) percent = 0;

  // ===== SCN =====
  canvas.clear();
  canvas.drawString("Temp: " + String(mlx.readObjectTempC()) + " C", 10, 50);
  canvas.drawString("YAW: " + String(ypr[0] * 180 / M_PI) + " Dg", 10, 75);
  canvas.drawString("Pitch: " + String(ypr[1] * 180 / M_PI) + " Dg", 10, 100);
  canvas.drawString("Roll: " + String(ypr[2] * 180 / M_PI) + " Dg", 10, 125);
  canvas.drawString("Vlt: " + String(voltage) + " mV", 10, 150);
  canvas.drawString("Prc: " + String(percent) + " %", 10, 175);
  canvas.drawString("Sat: " + String(GPS.satellites), 10, 200);
  canvas.drawString("Lat: " + String(GPS.latitudeDegrees), 10, 225);
  canvas.drawString("Lon: " + String(GPS.longitudeDegrees), 10, 250);
  canvas.drawString("            HELP ", 10, 455);
  canvas.drawRect(100, 400, 325, 140, 15);
  canvas.drawRect(101, 401, 326, 141, 15);
  canvas.drawRect(102, 402, 327, 142, 15);
  canvas.pushCanvas(0, 0, UPDATE_MODE_GC16);
  canvas.pushCanvas(0, 0, UPDATE_MODE_GC16);

  // ===== GPS =====
  GPS.read();
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) return;
  }

  if (millis() - timer > 1000) {
    timer = millis();


    Serial.println("---------------");
    Serial.println("----- GPS -----");
    Serial.print("Satellites : ");
    Serial.println(GPS.satellites);

    if (GPS.fix) {  // Si fix GPS
      Serial.print("Latitude  : ");
      Serial.print(GPS.latitudeDegrees, 6);
      Serial.println("°");

      Serial.print("Longitude : ");
      Serial.print(GPS.longitudeDegrees, 6);
      Serial.println("°");
    } else {
      Serial.println("Pas de fix GPS");
    }

    if (Serial1.available()) {
      String buffer = "";
      buffer = Serial1.readString();
      num = num + 1;
      Serial.print(num);
      Serial.print(". ");
      //Serial.println(buffer);

      // ===== GSM =====
      if (buffer.indexOf("+CMTI:") != -1) {

        Serial.print("SMS Index No... ");
        int indexno = sms.indexFromSerial(buffer);
        Serial.println(indexno);

        Serial.print("Who send the message?...");
        Serial.println(sms.getSenderNo(indexno));

        Serial.print("Read the message... ");
        Serial.println(sms.readFromSerial(buffer));

        canvas.drawString("SMS Index No... ", 10, 150);
        canvas.drawString(String(indexno), 10, 175);
        canvas.drawString("Who send the message?...", 10, 200);
        canvas.drawString(String(sms.getSenderNo(indexno)), 10, 225);
        canvas.drawString("Read the message... ", 10, 250);
        canvas.drawString(String(sms.readFromSerial(buffer)), 10, 275);
        canvas.pushCanvas(0, 0, UPDATE_MODE_GC16);

      } else {
        Serial.println(buffer);
      }
    }
  }

  // ===== MPU =====
  if (!DMPReady) return;  // Sort si DMP non prêt

  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {  // Lecture paquet FIFO
    // Calcul orientation
    mpu.dmpGetQuaternion(&q, FIFOBuffer);       // Quaternion
    mpu.dmpGetGravity(&gravity, &q);            // Gravité
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);  // Yaw, Pitch, Roll

    // Accélérations et gyro
    mpu.dmpGetAccel(&aa, FIFOBuffer);               // Accélération brute
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);  // Accélération linéaire
    mpu.dmpGetGyro(&gy, FIFOBuffer);                // Vitesse angulaire

    // Affichage orientation
    Serial.println("----- MPU6050 -----");
    Serial.print("Yaw: ");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print(" | Pitch: ");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print(" | Roll: ");
    Serial.println(ypr[2] * 180 / M_PI);
  }
}
