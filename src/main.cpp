#include <Arduino.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <Wire.h>

#include <limits.h>
#include <math.h>
#include <stdlib.h>

// ======================================================================
// โครงการ: ชุดการเรียนรู้ PID ทรงตัว 1 แกน
// บอร์ด: ESP8266 NodeMCU v2
// เซนเซอร์: MPU6050 อ่านค่า accelerometer และ gyroscope โดยตรงผ่าน Wire.h
// มอเตอร์ไดรเวอร์: MX1508 ควบคุมมอเตอร์ DC 3.3V แบบสองทิศทาง
// ======================================================================

// -------------------------- ขาเชื่อมต่อฮาร์ดแวร์ --------------------------
// MPU6050:
//   SDA = D2
//   SCL = D1
// MX1508:
//   IN1 = D5
//   IN2 = D6
//
// ต้องต่อ GND ของ ESP8266, MPU6050, MX1508 และแหล่งจ่ายมอเตอร์ร่วมกันทั้งหมด
const uint8_t PIN_MPU_SDA = D2;
const uint8_t PIN_MPU_SCL = D1;
const uint8_t PIN_MOTOR_IN1 = D5;
const uint8_t PIN_MOTOR_IN2 = D6;

// -------------------------- ค่า MPU6050 --------------------------
const uint8_t MPU6050_ADDRESS = 0x68;
const uint8_t MPU_REG_PWR_MGMT_1 = 0x6B;
const uint8_t MPU_REG_SMPLRT_DIV = 0x19;
const uint8_t MPU_REG_CONFIG = 0x1A;
const uint8_t MPU_REG_GYRO_CONFIG = 0x1B;
const uint8_t MPU_REG_ACCEL_CONFIG = 0x1C;
const uint8_t MPU_REG_ACCEL_XOUT_H = 0x3B;
const uint8_t MPU_REG_WHO_AM_I = 0x75;

// เลือกช่วงวัด +/-2g และ +/-250 deg/s จึงใช้ scale factor ตาม datasheet
const float ACCEL_LSB_PER_G = 16384.0f;
const float GYRO_LSB_PER_DPS = 131.0f;
const float RAD_TO_DEGREE = 57.2957795131f;

// -------------------------- Sign convention --------------------------
// ในโค้ดนี้กำหนดว่า:
//   filteredAngle > 0  คือแท่งเอียงไปด้านหน้า
//   filteredAngle < 0  คือแท่งเอียงไปด้านหลัง
//   targetAngle = 0    คืออยากให้แท่งตั้งตรง
//
// ตัวอย่างนี้ใช้การเอียงรอบแกน X ของ MPU6050:
//   accelAngle = atan2(accelY, accelZ)
//   gyroRate   = gyroX
//
// ถ้าติดตั้ง MPU6050 แล้วมุมอ่านกลับด้าน ให้เปลี่ยน ANGLE_SIGN จาก 1.0 เป็น -1.0
// ถ้ามอเตอร์ดึงกลับผิดทิศ ให้เปลี่ยน MOTOR_DIRECTION_SIGN จาก 1 เป็น -1
// หรือสลับสายมอเตอร์ที่ออกจาก MX1508 ได้เช่นกัน
const float ANGLE_SIGN = 1.0f;
const int MOTOR_DIRECTION_SIGN = 1;

// -------------------------- ค่าเวลาควบคุมและความปลอดภัย --------------------------
const int PWM_MAX = 1023;
const unsigned long CONTROL_INTERVAL_US = 5000UL;  // 200 Hz โดยประมาณ
const float MIN_VALID_DT = 0.001f;                 // dt ต่ำผิดปกติให้ข้าม
const float MAX_VALID_DT = 0.050f;                 // dt สูงผิดปกติให้หยุดรอบนั้น
const float COMPLEMENTARY_ALPHA = 0.98f;           // gyro 98%, accel 2%
const float SAFE_ANGLE_LIMIT = 45.0f;              // เอียงเกินนี้ให้หยุดมอเตอร์ทันที
const float MAX_REASONABLE_TARGET_ANGLE = 15.0f;   // จำกัด setpoint offset จากหน้าเว็บ
const float I_TERM_LIMIT = 350.0f;                 // จำกัดผลของ I term เพื่อกัน integral windup

// -------------------------- ตั้งค่า Wi-Fi --------------------------
// แก้สองบรรทัดนี้ให้ตรงกับ Wi-Fi ที่ต้องการใช้งาน
// ถ้าเชื่อมต่อไม่ได้ ระบบจะเปิด SoftAP สำรองชื่อ PID-ESP8266
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

const char* FALLBACK_AP_SSID = "PID-ESP8266";
const char* FALLBACK_AP_PASSWORD = "123456789";

ESP8266WebServer server(80);

// -------------------------- ค่าที่ปรับได้จากหน้าเว็บ --------------------------
// ค่าเริ่มต้นตั้งแบบค่อนข้างนุ่มเพื่อใช้เริ่มทดลองจริง
// มอเตอร์และโครงสร้างแต่ละชุดต่างกันมาก ต้องจูนใหม่บนฮาร์ดแวร์จริงเสมอ
float Kp = 28.0f;
float Ki = 0.0f;
float Kd = 0.85f;
int deadband = 130;
float targetAngle = 0.0f;

// -------------------------- ตัวแปรค่า raw จาก MPU6050 --------------------------
int16_t rawAx = 0;
int16_t rawAy = 0;
int16_t rawAz = 0;
int16_t rawTemp = 0;
int16_t rawGx = 0;
int16_t rawGy = 0;
int16_t rawGz = 0;

// -------------------------- ตัวแปรที่แปลงหน่วยแล้ว --------------------------
float accelX = 0.0f;
float accelY = 0.0f;
float accelZ = 0.0f;
float gyroX = 0.0f;
float gyroY = 0.0f;
float gyroZ = 0.0f;

// -------------------------- ค่า offset ของ gyro หลัง calibrate --------------------------
float gyroXOffsetRaw = 0.0f;
float gyroYOffsetRaw = 0.0f;
float gyroZOffsetRaw = 0.0f;

// -------------------------- ตัวแปรมุมและ PID --------------------------
float accelAngle = 0.0f;
float filteredAngle = 0.0f;
float errorAngle = 0.0f;
float integral = 0.0f;
float derivative = 0.0f;
float pidOutput = 0.0f;
float previousError = 0.0f;

bool angleInitialized = false;
bool previousErrorValid = false;
bool safetyStop = false;

// appliedPwm เป็นค่าแบบมีเครื่องหมายเพื่อบอกทิศทาง
// appliedPwmMagnitude เป็นค่า PWM จริงแบบ 0-1023 ที่ส่งให้ MX1508
int appliedPwm = 0;
int appliedPwmMagnitude = 0;
const char* motorDirectionText = "STOP";

// -------------------------- ตัวแปรเวลา --------------------------
unsigned long lastControlMicros = 0;
unsigned long lastSerialMillis = 0;
unsigned long lastMPUErrorMillis = 0;

// -------------------------- ประกาศฟังก์ชัน --------------------------
bool initMPU();
bool readMPU();
void calibrateGyro();
void computeAngle(float dt);
void computePID(float dt);
void driveMotor(float output);
void stopMotor();
void setupWebServer();
void connectWiFi();
void resetPIDState();
void printTelemetry();

bool writeMPURegister(uint8_t reg, uint8_t value);
bool readMPUBytes(uint8_t startRegister, uint8_t* buffer, size_t length);

String buildMainPage();
String buildStatusJson();
String getPreferredIPText();

void handleRoot();
void handleSet();
void handleStatus();
void handleNotFound();

bool parseFloatValue(const String& text, float& value);
bool parseIntValue(const String& text, int& value);

// ======================================================================
// ฟังก์ชันอ่าน/เขียน MPU6050 ผ่าน I2C
// ======================================================================

bool writeMPURegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readMPUBytes(uint8_t startRegister, uint8_t* buffer, size_t length) {
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(startRegister);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  const uint8_t requestedLength = static_cast<uint8_t>(length);
  const uint8_t bytesRead = Wire.requestFrom(MPU6050_ADDRESS, requestedLength, static_cast<uint8_t>(true));

  if (bytesRead != requestedLength) {
    return false;
  }

  for (size_t i = 0; i < length; ++i) {
    buffer[i] = Wire.read();
  }

  return true;
}

bool initMPU() {
  // ปลุก MPU6050 และใช้ clock จาก gyro แกน X เพื่อให้เสถียรกว่า internal oscillator
  if (!writeMPURegister(MPU_REG_PWR_MGMT_1, 0x01)) {
    return false;
  }
  delay(100);

  // Sample rate = gyro output rate / (1 + divider)
  // เมื่อเปิด DLPF แล้ว gyro output rate ประมาณ 1 kHz, divider 4 จึงได้ประมาณ 200 Hz
  if (!writeMPURegister(MPU_REG_SMPLRT_DIV, 0x04)) {
    return false;
  }

  // DLPF = 3 ช่วยลด noise เหลือ bandwidth ประมาณ 44 Hz เหมาะกับชุดทดลองบนโต๊ะ
  if (!writeMPURegister(MPU_REG_CONFIG, 0x03)) {
    return false;
  }

  // Gyro full scale = +/-250 deg/s
  if (!writeMPURegister(MPU_REG_GYRO_CONFIG, 0x00)) {
    return false;
  }

  // Accelerometer full scale = +/-2g
  if (!writeMPURegister(MPU_REG_ACCEL_CONFIG, 0x00)) {
    return false;
  }

  uint8_t whoAmI = 0;
  if (!readMPUBytes(MPU_REG_WHO_AM_I, &whoAmI, 1)) {
    return false;
  }

  return whoAmI == 0x68;
}

bool readMPU() {
  // อ่าน register 0x3B ถึง 0x48 รวม 14 byte:
  // accel X/Y/Z, temperature, gyro X/Y/Z
  uint8_t buffer[14];
  if (!readMPUBytes(MPU_REG_ACCEL_XOUT_H, buffer, sizeof(buffer))) {
    return false;
  }

  rawAx = static_cast<int16_t>((buffer[0] << 8) | buffer[1]);
  rawAy = static_cast<int16_t>((buffer[2] << 8) | buffer[3]);
  rawAz = static_cast<int16_t>((buffer[4] << 8) | buffer[5]);
  rawTemp = static_cast<int16_t>((buffer[6] << 8) | buffer[7]);
  rawGx = static_cast<int16_t>((buffer[8] << 8) | buffer[9]);
  rawGy = static_cast<int16_t>((buffer[10] << 8) | buffer[11]);
  rawGz = static_cast<int16_t>((buffer[12] << 8) | buffer[13]);

  // แปลง accelerometer เป็นหน่วย g
  accelX = static_cast<float>(rawAx) / ACCEL_LSB_PER_G;
  accelY = static_cast<float>(rawAy) / ACCEL_LSB_PER_G;
  accelZ = static_cast<float>(rawAz) / ACCEL_LSB_PER_G;

  // ลบ offset ของ gyro แล้วแปลงเป็น deg/s
  gyroX = (static_cast<float>(rawGx) - gyroXOffsetRaw) / GYRO_LSB_PER_DPS;
  gyroY = (static_cast<float>(rawGy) - gyroYOffsetRaw) / GYRO_LSB_PER_DPS;
  gyroZ = (static_cast<float>(rawGz) - gyroZOffsetRaw) / GYRO_LSB_PER_DPS;

  return true;
}

void calibrateGyro() {
  // ต้องวางชุดทดลองให้นิ่งที่สุด เพื่อให้ offset ของ gyro ถูกต้อง
  const int sampleCount = 1500;
  long sumGx = 0;
  long sumGy = 0;
  long sumGz = 0;
  int validSamples = 0;

  Serial.println();
  Serial.println(F("เริ่ม calibrate gyroscope"));
  Serial.println(F("กรุณาวางอุปกรณ์ให้นิ่งบนโต๊ะประมาณ 3-5 วินาที"));

  while (validSamples < sampleCount) {
    if (readMPU()) {
      sumGx += rawGx;
      sumGy += rawGy;
      sumGz += rawGz;
      ++validSamples;

      if (validSamples % 250 == 0) {
        Serial.print(F("calibrating... "));
        Serial.print(validSamples);
        Serial.print(F("/"));
        Serial.println(sampleCount);
      }
    }

    delay(2);
    yield();
  }

  gyroXOffsetRaw = static_cast<float>(sumGx) / static_cast<float>(sampleCount);
  gyroYOffsetRaw = static_cast<float>(sumGy) / static_cast<float>(sampleCount);
  gyroZOffsetRaw = static_cast<float>(sumGz) / static_cast<float>(sampleCount);

  Serial.println(F("calibration เสร็จแล้ว"));
  Serial.print(F("gyroX offset raw = "));
  Serial.println(gyroXOffsetRaw, 3);
  Serial.print(F("gyroY offset raw = "));
  Serial.println(gyroYOffsetRaw, 3);
  Serial.print(F("gyroZ offset raw = "));
  Serial.println(gyroZOffsetRaw, 3);
}

// ======================================================================
// ฟังก์ชันคำนวณมุมและ PID
// ======================================================================

void computeAngle(float dt) {
  // accelerometer ให้มุมระยะยาวที่ไม่ drift แต่ไวต่อแรงสั่น
  // gyro ให้ความเร็วเชิงมุมที่ตอบสนองเร็ว แต่ drift ได้เมื่อเวลาผ่านไป
  // complementary filter รวมข้อดีของทั้งสองส่วนเข้าด้วยกัน
  const float accelAngleLocal = ANGLE_SIGN * atan2f(accelY, accelZ) * RAD_TO_DEGREE;
  const float gyroRate = ANGLE_SIGN * gyroX;

  accelAngle = accelAngleLocal;

  if (!angleInitialized) {
    filteredAngle = accelAngle;
    angleInitialized = true;
    return;
  }

  filteredAngle = (COMPLEMENTARY_ALPHA * (filteredAngle + gyroRate * dt)) +
                  ((1.0f - COMPLEMENTARY_ALPHA) * accelAngle);
}

void computePID(float dt) {
  // นิยาม error = targetAngle - filteredAngle
  // ถ้าแท่งเอียงหน้าและ filteredAngle เป็นบวก error จะติดลบ
  // จากนั้น output ที่ติดลบจะถูกส่งไปยัง driveMotor() เพื่อหมุนกลับอีกทิศ
  errorAngle = targetAngle - filteredAngle;

  // ถ้ามุมเอียงมากเกินไปให้หยุดทันที เพื่อไม่ให้มอเตอร์เร่งตอนชุดทดลองล้ม
  if (fabsf(filteredAngle) >= SAFE_ANGLE_LIMIT) {
    safetyStop = true;
    pidOutput = 0.0f;
    resetPIDState();
    return;
  }

  safetyStop = false;

  // Integral สะสม error ตามเวลา และถูกจำกัดด้วย anti-windup
  integral += errorAngle * dt;
  if (Ki > 0.0f) {
    const float integralLimit = I_TERM_LIMIT / Ki;
    integral = constrain(integral, -integralLimit, integralLimit);
  } else {
    integral = 0.0f;
  }

  // Derivative ช่วยลดการสั่นและชะลอการเคลื่อนที่เร็ว
  if (previousErrorValid) {
    derivative = (errorAngle - previousError) / dt;
  } else {
    derivative = 0.0f;
    previousErrorValid = true;
  }

  const float pTerm = Kp * errorAngle;
  const float iTerm = Ki * integral;
  const float dTerm = Kd * derivative;

  pidOutput = pTerm + iTerm + dTerm;
  pidOutput = constrain(pidOutput, -static_cast<float>(PWM_MAX), static_cast<float>(PWM_MAX));

  previousError = errorAngle;
}

void resetPIDState() {
  integral = 0.0f;
  derivative = 0.0f;
  previousError = 0.0f;
  previousErrorValid = false;
}

// ======================================================================
// ฟังก์ชันควบคุมมอเตอร์ MX1508
// ======================================================================

void stopMotor() {
  analogWrite(PIN_MOTOR_IN1, 0);
  analogWrite(PIN_MOTOR_IN2, 0);
  appliedPwm = 0;
  appliedPwmMagnitude = 0;
  motorDirectionText = "STOP";
}

void driveMotor(float output) {
  // output จาก PID เป็นค่ามีเครื่องหมาย:
  //   output > 0 หมุนทิศทางหนึ่ง
  //   output < 0 หมุนอีกทิศทางหนึ่ง
  // ถ้าทิศชดเชยผิด ให้เปลี่ยน MOTOR_DIRECTION_SIGN หรือสลับสายมอเตอร์
  if (safetyStop) {
    stopMotor();
    return;
  }

  int command = static_cast<int>(lroundf(output));
  command = constrain(command, -PWM_MAX, PWM_MAX);

  // ถ้าคำสั่ง PWM ต่ำกว่า deadband ให้หยุด เพราะมอเตอร์จริงมักยังไม่หมุน
  // การหยุดตรงนี้ช่วยลดเสียงฮัมและความร้อนจากการจ่าย PWM ต่ำเกินไป
  if (abs(command) < deadband) {
    stopMotor();
    return;
  }

  command *= MOTOR_DIRECTION_SIGN;
  const int pwm = constrain(abs(command), 0, PWM_MAX);

  if (command > 0) {
    analogWrite(PIN_MOTOR_IN1, pwm);
    analogWrite(PIN_MOTOR_IN2, 0);
    motorDirectionText = "DIR_POS";
  } else {
    analogWrite(PIN_MOTOR_IN1, 0);
    analogWrite(PIN_MOTOR_IN2, pwm);
    motorDirectionText = "DIR_NEG";
  }

  appliedPwm = command;
  appliedPwmMagnitude = pwm;
}

// ======================================================================
// ฟังก์ชัน Wi-Fi และ Web Server
// ======================================================================

void connectWiFi() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print(F("กำลังเชื่อม Wi-Fi"));
  const unsigned long startAttempt = millis();

  while (WiFi.status() != WL_CONNECTED && (millis() - startAttempt) < 15000UL) {
    delay(500);
    Serial.print(F("."));
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(F("เชื่อม Wi-Fi สำเร็จ"));
    Serial.print(F("SSID: "));
    Serial.println(WIFI_SSID);
    Serial.print(F("IP Address: "));
    Serial.println(WiFi.localIP());
    return;
  }

  // ถ้า Wi-Fi หลักไม่สำเร็จ เปิดโหมด AP สำรองเพื่อให้ยังเข้าหน้าเว็บได้
  Serial.println(F("เชื่อม Wi-Fi หลักไม่สำเร็จ จะเปิด SoftAP สำรอง"));
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(FALLBACK_AP_SSID, FALLBACK_AP_PASSWORD);

  Serial.print(F("SoftAP SSID: "));
  Serial.println(FALLBACK_AP_SSID);
  Serial.print(F("SoftAP Password: "));
  Serial.println(FALLBACK_AP_PASSWORD);
  Serial.print(F("SoftAP IP Address: "));
  Serial.println(WiFi.softAPIP());
}

String getPreferredIPText() {
  if (WiFi.status() == WL_CONNECTED) {
    return WiFi.localIP().toString();
  }

  return WiFi.softAPIP().toString();
}

String buildStatusJson() {
  String json;
  json.reserve(640);

  json += F("{\"kp\":");
  json += String(Kp, 3);
  json += F(",\"ki\":");
  json += String(Ki, 3);
  json += F(",\"kd\":");
  json += String(Kd, 3);
  json += F(",\"deadband\":");
  json += String(deadband);
  json += F(",\"targetAngle\":");
  json += String(targetAngle, 2);
  json += F(",\"accelAngle\":");
  json += String(accelAngle, 2);
  json += F(",\"filteredAngle\":");
  json += String(filteredAngle, 2);
  json += F(",\"error\":");
  json += String(errorAngle, 2);
  json += F(",\"integral\":");
  json += String(integral, 4);
  json += F(",\"derivative\":");
  json += String(derivative, 2);
  json += F(",\"pidOutput\":");
  json += String(pidOutput, 2);
  json += F(",\"pwm\":");
  json += String(appliedPwm);
  json += F(",\"pwmAbs\":");
  json += String(appliedPwmMagnitude);
  json += F(",\"motorDirection\":\"");
  json += motorDirectionText;
  json += F("\",\"safetyStop\":");
  json += (safetyStop ? F("true") : F("false"));
  json += F(",\"ip\":\"");
  json += getPreferredIPText();
  json += F("\"}");

  return json;
}

String buildMainPage() {
  String html;
  html.reserve(7600);

  html += F(
      "<!DOCTYPE html><html lang='th'><head><meta charset='utf-8'>"
      "<meta name='viewport' content='width=device-width,initial-scale=1'>"
      "<title>ESP8266 PID Balance</title>"
      "<style>"
      ":root{--bg:#f5f7fa;--card:#fff;--line:#d7dee8;--text:#192330;--muted:#5f6d7a;--accent:#08746f;--danger:#b42318}"
      "*{box-sizing:border-box}body{margin:0;font-family:Tahoma,'Noto Sans Thai',Arial,sans-serif;background:var(--bg);color:var(--text)}"
      ".wrap{max-width:880px;margin:0 auto;padding:14px}.card{background:var(--card);border:1px solid var(--line);border-radius:8px;padding:14px;margin-bottom:12px}"
      "h1{margin:0 0 8px;font-size:1.25rem;line-height:1.3}p{margin:6px 0;color:var(--muted);line-height:1.5}"
      ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(190px,1fr));gap:10px}.field{display:flex;flex-direction:column;gap:5px}"
      "label{font-weight:700;font-size:.95rem}input{width:100%;padding:11px;border-radius:8px;border:1px solid var(--line);font-size:1rem;background:#fff}"
      "button{width:100%;padding:12px;border:0;border-radius:8px;background:var(--accent);color:#fff;font-size:1rem;font-weight:700}"
      ".status{display:grid;grid-template-columns:repeat(auto-fit,minmax(140px,1fr));gap:8px;margin-top:10px}"
      ".pill{padding:10px;border-radius:8px;background:#f9fbfd;border:1px solid var(--line)}.pill b{display:block;font-size:.78rem;color:var(--muted);margin-bottom:4px}"
      ".mono{font-family:Consolas,Monaco,monospace}.warn{color:var(--danger);font-weight:700}.endpoint{display:inline-block;padding:3px 7px;border-radius:8px;background:#eef7f6;border:1px solid #cfe9e6;margin:3px 4px 0 0}"
      "</style></head><body><div class='wrap'>");

  html += F("<div class='card'><h1>ชุดการเรียนรู้ PID ทรงตัว 1 แกน</h1>");
  html += F("<p>ปรับค่า Kp, Ki, Kd, deadband และ targetAngle ได้ทันทีโดยไม่ต้องอัปโหลดโค้ดใหม่</p>");
  html += F("<p>IP ปัจจุบัน: <span class='mono'>http://");
  html += getPreferredIPText();
  html += F("</span></p>");
  html += F("<p>endpoint: <span class='endpoint'>GET /</span><span class='endpoint'>GET /set</span><span class='endpoint'>GET /status</span></p></div>");

  html += F("<div class='card'><h1>PID Settings</h1><form action='/set' method='get'><div class='grid'>");

  html += F("<div class='field'><label for='kp'>Kp</label><input id='kp' name='kp' type='number' min='0' step='0.01' value='");
  html += String(Kp, 3);
  html += F("'></div>");

  html += F("<div class='field'><label for='ki'>Ki</label><input id='ki' name='ki' type='number' min='0' step='0.01' value='");
  html += String(Ki, 3);
  html += F("'></div>");

  html += F("<div class='field'><label for='kd'>Kd</label><input id='kd' name='kd' type='number' min='0' step='0.01' value='");
  html += String(Kd, 3);
  html += F("'></div>");

  html += F("<div class='field'><label for='deadband'>deadband (0-1023)</label><input id='deadband' name='deadband' type='number' min='0' max='1023' step='1' value='");
  html += String(deadband);
  html += F("'></div>");

  html += F("<div class='field'><label for='targetAngle'>targetAngle / setpoint offset</label><input id='targetAngle' name='targetAngle' type='number' min='-15' max='15' step='0.1' value='");
  html += String(targetAngle, 2);
  html += F("'></div>");

  html += F("</div><div style='margin-top:12px'><button type='submit'>Update / Save</button></div></form></div>");

  html += F(
      "<div class='card'><h1>สถานะปัจจุบัน</h1>"
      "<div class='status'>"
      "<div class='pill'><b>Filtered Angle</b><span id='filteredAngle'>-</span></div>"
      "<div class='pill'><b>Accel Angle</b><span id='accelAngle'>-</span></div>"
      "<div class='pill'><b>Error</b><span id='error'>-</span></div>"
      "<div class='pill'><b>Integral</b><span id='integral'>-</span></div>"
      "<div class='pill'><b>Derivative</b><span id='derivative'>-</span></div>"
      "<div class='pill'><b>PID Output</b><span id='pidOutput'>-</span></div>"
      "<div class='pill'><b>PWM</b><span id='pwm'>-</span></div>"
      "<div class='pill'><b>Motor Direction</b><span id='motorDirection'>-</span></div>"
      "<div class='pill'><b>Safety Stop</b><span id='safetyStop'>-</span></div>"
      "</div>"
      "<p>ข้อมูลสถานะดึงจาก <span class='mono'>/status</span> ทุก 1 วินาที</p>"
      "</div>");

  html += F(
      "<script>"
      "async function loadStatus(){"
      "try{"
      "const r=await fetch('/status',{cache:'no-store'});"
      "const s=await r.json();"
      "document.getElementById('filteredAngle').textContent=s.filteredAngle+' deg';"
      "document.getElementById('accelAngle').textContent=s.accelAngle+' deg';"
      "document.getElementById('error').textContent=s.error+' deg';"
      "document.getElementById('integral').textContent=s.integral;"
      "document.getElementById('derivative').textContent=s.derivative;"
      "document.getElementById('pidOutput').textContent=s.pidOutput;"
      "document.getElementById('pwm').textContent=s.pwm;"
      "document.getElementById('motorDirection').textContent=s.motorDirection;"
      "document.getElementById('safetyStop').textContent=s.safetyStop?'ACTIVE':'NORMAL';"
      "}catch(e){console.log(e)}"
      "}"
      "loadStatus();setInterval(loadStatus,1000);"
      "</script></div></body></html>");

  return html;
}

bool parseFloatValue(const String& text, float& value) {
  char* endPtr = nullptr;
  value = strtof(text.c_str(), &endPtr);
  return endPtr != text.c_str() && endPtr != nullptr && *endPtr == '\0' && isfinite(value);
}

bool parseIntValue(const String& text, int& value) {
  char* endPtr = nullptr;
  const long parsed = strtol(text.c_str(), &endPtr, 10);

  if (endPtr == text.c_str() || endPtr == nullptr || *endPtr != '\0') {
    return false;
  }

  if (parsed < INT_MIN || parsed > INT_MAX) {
    return false;
  }

  value = static_cast<int>(parsed);
  return true;
}

void handleRoot() {
  server.send(200, "text/html; charset=utf-8", buildMainPage());
}

void handleStatus() {
  server.send(200, "application/json; charset=utf-8", buildStatusJson());
}

void handleSet() {
  float newKp = Kp;
  float newKi = Ki;
  float newKd = Kd;
  float newTargetAngle = targetAngle;
  int newDeadband = deadband;

  String errorText;

  if (server.hasArg("kp")) {
    float value = 0.0f;
    if (!parseFloatValue(server.arg("kp"), value) || value < 0.0f) {
      errorText += F("Kp ต้องเป็นตัวเลขและต้องไม่ติดลบ\n");
    } else {
      newKp = value;
    }
  }

  if (server.hasArg("ki")) {
    float value = 0.0f;
    if (!parseFloatValue(server.arg("ki"), value) || value < 0.0f) {
      errorText += F("Ki ต้องเป็นตัวเลขและต้องไม่ติดลบ\n");
    } else {
      newKi = value;
    }
  }

  if (server.hasArg("kd")) {
    float value = 0.0f;
    if (!parseFloatValue(server.arg("kd"), value) || value < 0.0f) {
      errorText += F("Kd ต้องเป็นตัวเลขและต้องไม่ติดลบ\n");
    } else {
      newKd = value;
    }
  }

  if (server.hasArg("deadband")) {
    int value = 0;
    if (!parseIntValue(server.arg("deadband"), value) || value < 0 || value > PWM_MAX) {
      errorText += F("deadband ต้องอยู่ในช่วง 0 ถึง 1023\n");
    } else {
      newDeadband = value;
    }
  }

  if (server.hasArg("targetAngle")) {
    float value = 0.0f;
    if (!parseFloatValue(server.arg("targetAngle"), value) ||
        value < -MAX_REASONABLE_TARGET_ANGLE ||
        value > MAX_REASONABLE_TARGET_ANGLE) {
      errorText += F("targetAngle ต้องอยู่ในช่วง -15 ถึง 15 องศา\n");
    } else {
      newTargetAngle = value;
    }
  }

  if (errorText.length() > 0) {
    server.send(400, "text/plain; charset=utf-8", errorText);
    return;
  }

  Kp = newKp;
  Ki = newKi;
  Kd = newKd;
  deadband = newDeadband;
  targetAngle = newTargetAngle;

  // ล้าง integral และ derivative เมื่อเปลี่ยนค่า เพื่อกันการกระชากจากสถานะ PID เก่า
  resetPIDState();

  Serial.print(F("Update PID: Kp="));
  Serial.print(Kp, 3);
  Serial.print(F(" Ki="));
  Serial.print(Ki, 3);
  Serial.print(F(" Kd="));
  Serial.print(Kd, 3);
  Serial.print(F(" deadband="));
  Serial.print(deadband);
  Serial.print(F(" targetAngle="));
  Serial.println(targetAngle, 2);

  server.sendHeader("Location", "/", true);
  server.send(303, "text/plain; charset=utf-8", "Updated");
}

void handleNotFound() {
  String message;
  message.reserve(220);
  message += F("ไม่พบ endpoint นี้\n\n");
  message += F("ใช้งานได้ที่:\n");
  message += F("/\n");
  message += F("/set?kp=28&ki=0&kd=0.85&deadband=130&targetAngle=0\n");
  message += F("/status\n");
  server.send(404, "text/plain; charset=utf-8", message);
}

void setupWebServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/set", HTTP_GET, handleSet);
  server.on("/status", HTTP_GET, handleStatus);
  server.onNotFound(handleNotFound);
  server.begin();

  Serial.println(F("Web server started"));
  Serial.print(F("เปิดเว็บที่: http://"));
  Serial.println(getPreferredIPText());
}

// ======================================================================
// Serial Monitor / Serial Plotter
// ======================================================================

void printTelemetry() {
  // รูปแบบ label:value คั่นด้วยช่องว่าง ใช้ดูได้ทั้ง Serial Monitor และ Serial Plotter
  Serial.print(F("accel_angle:"));
  Serial.print(accelAngle, 2);
  Serial.print(F(" filtered_angle:"));
  Serial.print(filteredAngle, 2);
  Serial.print(F(" error:"));
  Serial.print(errorAngle, 2);
  Serial.print(F(" integral:"));
  Serial.print(integral, 4);
  Serial.print(F(" derivative:"));
  Serial.print(derivative, 2);
  Serial.print(F(" pid_output:"));
  Serial.print(pidOutput, 2);
  Serial.print(F(" pwm:"));
  Serial.println(appliedPwm);
}

// ======================================================================
// setup() และ loop()
// ======================================================================

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println(F("ESP8266 1-Axis PID Balance Demo Starting..."));

  pinMode(PIN_MOTOR_IN1, OUTPUT);
  pinMode(PIN_MOTOR_IN2, OUTPUT);
  analogWriteRange(PWM_MAX);
  analogWriteFreq(1000);
  stopMotor();

  Wire.begin(PIN_MPU_SDA, PIN_MPU_SCL);
  Wire.setClock(400000);

  if (!initMPU()) {
    Serial.println(F("ไม่สามารถเริ่มต้น MPU6050 ได้"));
    Serial.println(F("ตรวจสอบ SDA=D2, SCL=D1, VCC, GND และ address 0x68"));
    while (true) {
      stopMotor();
      delay(1000);
      yield();
    }
  }

  calibrateGyro();

  // อ่านครั้งแรกเพื่อกำหนดมุมเริ่มต้นของ complementary filter
  if (readMPU()) {
    computeAngle(0.01f);
  }

  connectWiFi();
  setupWebServer();

  Serial.println(F("เปิด Serial Monitor ที่ 115200 เพื่อดู IP Address"));
  Serial.println(F("จากนั้นเปิดเว็บเบราว์เซอร์ไปที่ IP ของ ESP8266"));
  Serial.println(F("เริ่มระบบควบคุมแล้ว"));
}

void loop() {
  server.handleClient();

  const unsigned long nowMicros = micros();

  if (lastControlMicros == 0) {
    lastControlMicros = nowMicros;
    return;
  }

  const unsigned long elapsedMicros = nowMicros - lastControlMicros;

  // คุมรอบการอ่านและคำนวณ ไม่ให้เร็วเกินไปจน dt เล็กและ noisy
  if (elapsedMicros < CONTROL_INTERVAL_US) {
    yield();
    return;
  }

  const float dt = static_cast<float>(elapsedMicros) / 1000000.0f;
  lastControlMicros = nowMicros;

  // ป้องกัน dt ผิดปกติ เช่น loop ถูกบล็อกนานจาก Wi-Fi หรือ Serial
  if (dt < MIN_VALID_DT || dt > MAX_VALID_DT) {
    stopMotor();
    resetPIDState();
    return;
  }

  if (!readMPU()) {
    stopMotor();
    resetPIDState();

    if (millis() - lastMPUErrorMillis > 1000UL) {
      Serial.println(F("อ่านค่า MPU6050 ไม่สำเร็จ โปรดตรวจสอบการเชื่อมต่อ"));
      lastMPUErrorMillis = millis();
    }
    return;
  }

  computeAngle(dt);
  computePID(dt);

  if (safetyStop) {
    stopMotor();
  } else {
    driveMotor(pidOutput);
  }

  if (millis() - lastSerialMillis >= 100UL) {
    printTelemetry();
    lastSerialMillis = millis();
  }

  yield();
}
