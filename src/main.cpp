#include <Arduino.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <Wire.h>

#include <limits.h>
#include <math.h>
#include <stdlib.h>

// -------------------------- การตั้งค่าฮาร์ดแวร์ --------------------------
// หมายเหตุเรื่อง sign convention:
// - ในตัวอย่างนี้ "มุมบวก" หมายถึงแท่งเอียงไปด้านหน้า
// - "มุมลบ" หมายถึงแท่งเอียงไปด้านหลัง
// - ถ้าทิศทางมุมที่อ่านได้กลับด้าน ให้เปลี่ยน ANGLE_SIGN จาก 1.0f เป็น -1.0f
// - ถ้าทิศมอเตอร์ชดเชยผิด ให้เปลี่ยน MOTOR_DIRECTION_SIGN จาก 1 เป็น -1
//   หรือสลับสายมอเตอร์ที่ออกจาก MX1508 แทนก็ได้

constexpr uint8_t PIN_MPU_SDA = D2;
constexpr uint8_t PIN_MPU_SCL = D1;
constexpr uint8_t PIN_MOTOR_IN1 = D5;
constexpr uint8_t PIN_MOTOR_IN2 = D6;

constexpr uint8_t MPU6050_ADDRESS = 0x68;

constexpr float ACCEL_LSB_PER_G = 16384.0f;   // ตั้งค่าไว้สำหรับช่วง +/-2g
constexpr float GYRO_LSB_PER_DPS = 131.0f;    // ตั้งค่าไว้สำหรับช่วง +/-250 deg/s
constexpr float DEG_PER_RAD = 57.2957795131f;

constexpr int PWM_MAX = 1023;
constexpr float COMPLEMENTARY_ALPHA = 0.98f;
constexpr float SAFE_ANGLE_LIMIT = 45.0f;
constexpr float MAX_REASONABLE_TARGET_ANGLE = 15.0f;
constexpr float MIN_VALID_DT = 0.001f;        // 1 ms
constexpr float MAX_VALID_DT = 0.050f;        // 50 ms
constexpr float I_TERM_LIMIT = 400.0f;        // จำกัดผลรวมของ I term เพื่อกัน windup

constexpr float ANGLE_SIGN = 1.0f;
constexpr int MOTOR_DIRECTION_SIGN = 1;

// -------------------------- การตั้งค่า Wi-Fi --------------------------
// แก้ไข SSID และ Password ให้ตรงกับเครือข่ายที่ต้องการใช้งานจริง
const char* WIFI_SSID = "AP-PTICEC";
const char* WIFI_PASSWORD = "";

// ถ้าเชื่อม Wi-Fi หลักไม่ได้ จะเปิด SoftAP สำรองเพื่อให้ยังเข้าเว็บได้
const char* FALLBACK_AP_SSID = "PID-ESP8266";
const char* FALLBACK_AP_PASSWORD = "123456789";

ESP8266WebServer server(80);

// -------------------------- พารามิเตอร์ที่ปรับได้จากหน้าเว็บ --------------------------
float Kp = 35.0f;
float Ki = 0.80f;
float Kd = 1.20f;
int deadband = 120;
float targetAngle = 0.0f;

// -------------------------- ตัวแปรดิบจากเซนเซอร์ --------------------------
int16_t rawAx = 0;
int16_t rawAy = 0;
int16_t rawAz = 0;
int16_t rawTemp = 0;
int16_t rawGx = 0;
int16_t rawGy = 0;
int16_t rawGz = 0;

float accelX = 0.0f;
float accelY = 0.0f;
float accelZ = 0.0f;
float gyroX = 0.0f;
float gyroY = 0.0f;
float gyroZ = 0.0f;

float gyroXOffsetRaw = 0.0f;
float gyroYOffsetRaw = 0.0f;
float gyroZOffsetRaw = 0.0f;

// -------------------------- ตัวแปรคำนวณมุมและ PID --------------------------
float accelAngle = 0.0f;
float filteredAngle = 0.0f;
float errorAngle = 0.0f;
float integral = 0.0f;
float derivative = 0.0f;
float pidOutput = 0.0f;
float previousError = 0.0f;

bool angleInitialized = false;
bool safetyStop = false;

int appliedPwm = 0;           // ค่า PWM แบบมีเครื่องหมาย เพื่อดูทิศทาง
int appliedPwmMagnitude = 0;  // ค่า PWM จริงที่ส่งออก
const char* motorDirectionText = "STOP";

// -------------------------- ตัวแปรเวลา --------------------------
unsigned long lastControlMicros = 0;
unsigned long lastSerialMillis = 0;
unsigned long lastMPUErrorMillis = 0;

// -------------------------- ประกาศฟังก์ชันล่วงหน้า --------------------------
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
String buildMainPage();
String buildStatusJson();
String getPreferredIPText();
void printTelemetry();
void handleRoot();
void handleSet();
void handleStatus();
void handleNotFound();
bool writeMPURegister(uint8_t reg, uint8_t value);
bool readMPUBytes(uint8_t startRegister, uint8_t* buffer, size_t length);
bool parseFloatValue(const String& text, float& value);
bool parseIntValue(const String& text, int& value);

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

  const size_t bytesRead = Wire.requestFrom(MPU6050_ADDRESS, static_cast<uint8_t>(length), static_cast<uint8_t>(true));
  if (bytesRead != length) {
    return false;
  }

  for (size_t i = 0; i < length; ++i) {
    buffer[i] = Wire.read();
  }
  return true;
}

bool initMPU() {
  // ปลุก MPU6050 จาก sleep mode
  if (!writeMPURegister(0x6B, 0x00)) {
    return false;
  }
  delay(100);

  // ตั้ง sample rate, low-pass filter, ช่วงวัด accelerometer และ gyro
  if (!writeMPURegister(0x19, 0x04)) {  // Sample Rate Divider
    return false;
  }
  if (!writeMPURegister(0x1A, 0x03)) {  // DLPF ~44Hz
    return false;
  }
  if (!writeMPURegister(0x1B, 0x00)) {  // Gyro +/-250 deg/s
    return false;
  }
  if (!writeMPURegister(0x1C, 0x00)) {  // Accel +/-2g
    return false;
  }

  uint8_t whoAmI = 0;
  if (!readMPUBytes(0x75, &whoAmI, 1)) {
    return false;
  }

  return whoAmI == 0x68;
}

bool readMPU() {
  uint8_t buffer[14];
  if (!readMPUBytes(0x3B, buffer, sizeof(buffer))) {
    return false;
  }

  rawAx = static_cast<int16_t>((buffer[0] << 8) | buffer[1]);
  rawAy = static_cast<int16_t>((buffer[2] << 8) | buffer[3]);
  rawAz = static_cast<int16_t>((buffer[4] << 8) | buffer[5]);
  rawTemp = static_cast<int16_t>((buffer[6] << 8) | buffer[7]);
  rawGx = static_cast<int16_t>((buffer[8] << 8) | buffer[9]);
  rawGy = static_cast<int16_t>((buffer[10] << 8) | buffer[11]);
  rawGz = static_cast<int16_t>((buffer[12] << 8) | buffer[13]);

  accelX = static_cast<float>(rawAx) / ACCEL_LSB_PER_G;
  accelY = static_cast<float>(rawAy) / ACCEL_LSB_PER_G;
  accelZ = static_cast<float>(rawAz) / ACCEL_LSB_PER_G;

  gyroX = (static_cast<float>(rawGx) - gyroXOffsetRaw) / GYRO_LSB_PER_DPS;
  gyroY = (static_cast<float>(rawGy) - gyroYOffsetRaw) / GYRO_LSB_PER_DPS;
  gyroZ = (static_cast<float>(rawGz) - gyroZOffsetRaw) / GYRO_LSB_PER_DPS;

  return true;
}

void calibrateGyro() {
  // ผู้ใช้ต้องวางชุดทดลองให้นิ่งที่สุดก่อนเริ่ม calibration
  constexpr int sampleCount = 1500;
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
    }

    if (validSamples % 250 == 0) {
      Serial.print(F("calibrating... "));
      Serial.print(validSamples);
      Serial.print(F("/"));
      Serial.println(sampleCount);
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

void computeAngle(float dt) {
  // ตัวอย่างนี้สมมติให้แกนทรงตัวอยู่ที่การหมุนรอบแกน X
  // ใช้ accel Y และ Z หาองศาเอียง และใช้ gyro X ช่วยเติมข้อมูลตอนเคลื่อนไหวเร็ว
  const float accelAngleLocal = ANGLE_SIGN * atan2f(accelY, accelZ) * DEG_PER_RAD;
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
  // นิยาม error = target - current
  // ถ้ามุมจริงเป็นบวกมากเกินไป ค่า error จะติดลบ เพื่อสั่งมอเตอร์หมุนดึงกลับ
  errorAngle = targetAngle - filteredAngle;

  // ถ้ามุมเอียงเกินช่วงปลอดภัย ให้หยุดทันทีและล้าง integral
  if (fabsf(filteredAngle) >= SAFE_ANGLE_LIMIT) {
    safetyStop = true;
    resetPIDState();
    pidOutput = 0.0f;
    return;
  }

  safetyStop = false;

  integral += errorAngle * dt;

  // anti-windup: จำกัด integral ตามขนาด I term สูงสุดที่ยอมให้มีผลต่อเอาต์พุต
  if (Ki > 0.0f) {
    const float integralLimit = I_TERM_LIMIT / Ki;
    integral = constrain(integral, -integralLimit, integralLimit);
  } else {
    integral = 0.0f;
  }

  derivative = (errorAngle - previousError) / dt;

  const float pTerm = Kp * errorAngle;
  const float iTerm = Ki * integral;
  const float dTerm = Kd * derivative;

  pidOutput = pTerm + iTerm + dTerm;
  pidOutput = constrain(pidOutput, -static_cast<float>(PWM_MAX), static_cast<float>(PWM_MAX));

  previousError = errorAngle;
}

void stopMotor() {
  analogWrite(PIN_MOTOR_IN1, 0);
  analogWrite(PIN_MOTOR_IN2, 0);
  appliedPwm = 0;
  appliedPwmMagnitude = 0;
  motorDirectionText = "STOP";
}

void driveMotor(float output) {
  // ถ้ากำลังอยู่ใน safety stop ให้หยุดมอเตอร์โดยไม่สนใจ output
  if (safetyStop) {
    stopMotor();
    return;
  }

  int command = static_cast<int>(lroundf(output));
  command = constrain(command, -PWM_MAX, PWM_MAX);

  // deadband ใช้กับค่า PWM ที่จะส่งจริง ถ้าต่ำเกินไปให้หยุด เพื่อกันมอเตอร์ฮัมแต่ไม่หมุน
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

void resetPIDState() {
  integral = 0.0f;
  derivative = 0.0f;
  previousError = 0.0f;
}

String getPreferredIPText() {
  if (WiFi.status() == WL_CONNECTED) {
    return WiFi.localIP().toString();
  }
  return WiFi.softAPIP().toString();
}

String buildStatusJson() {
  String json;
  json.reserve(512);

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
  html.reserve(6000);

  html += F(
      "<!DOCTYPE html><html lang='th'><head><meta charset='utf-8'>"
      "<meta name='viewport' content='width=device-width, initial-scale=1'>"
      "<title>ESP8266 PID Balance</title>"
      "<style>"
      ":root{--bg:#f4f7fb;--card:#ffffff;--line:#d8e1ec;--text:#1d2733;--muted:#637182;--accent:#0b7a75;--danger:#b42318;}"
      "*{box-sizing:border-box}body{margin:0;font-family:Tahoma,'Noto Sans Thai',sans-serif;background:linear-gradient(180deg,#eef5fb 0%,#f7fafc 100%);color:var(--text)}"
      ".wrap{max-width:860px;margin:0 auto;padding:16px}.card{background:var(--card);border:1px solid var(--line);border-radius:18px;padding:16px;box-shadow:0 10px 30px rgba(25,39,52,.08);margin-bottom:16px}"
      "h1{margin:0 0 10px;font-size:1.35rem}p{margin:6px 0;color:var(--muted);line-height:1.5}"
      ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:12px}.field{display:flex;flex-direction:column;gap:6px}"
      "label{font-weight:700}input{width:100%;padding:12px;border-radius:12px;border:1px solid var(--line);font-size:1rem}"
      "button{width:100%;padding:13px;border:none;border-radius:12px;background:var(--accent);color:#fff;font-size:1rem;font-weight:700}"
      ".status{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:10px;margin-top:12px}"
      ".pill{padding:12px;border-radius:14px;background:#f7fafc;border:1px solid var(--line)}"
      ".pill b{display:block;font-size:.82rem;color:var(--muted);margin-bottom:4px}"
      ".warn{color:var(--danger);font-weight:700}.mono{font-family:Consolas,Monaco,monospace}"
      ".foot{font-size:.92rem}.endpoint{display:inline-block;padding:4px 8px;border-radius:999px;background:#edf7f5;border:1px solid #cde9e6;margin:3px 6px 0 0}"
      "</style></head><body><div class='wrap'>");

  html += F("<div class='card'><h1>ชุดการเรียนรู้ PID ทรงตัว 1 แกน</h1>");
  html += F("<p>หน้าเว็บนี้ใช้ปรับค่า PID, deadband และ setpoint offset ได้ทันทีโดยไม่ต้องอัปโหลดโค้ดใหม่</p>");
  html += F("<p>IP ปัจจุบันของ ESP8266: <span class='mono'>");
  html += getPreferredIPText();
  html += F("</span></p>");
  html += F("<p>ถ้าเปลี่ยนค่าแล้วระบบตอบสนองผิดทิศ ให้กลับทิศที่สายมอเตอร์หรือเปลี่ยนค่าคงที่ <span class='mono'>MOTOR_DIRECTION_SIGN</span> ในโค้ด</p></div>");

  html += F("<div class='card'><form action='/set' method='get'><div class='grid'>");

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

  html += F("<div class='field'><label for='targetAngle'>targetAngle</label><input id='targetAngle' name='targetAngle' type='number' min='-15' max='15' step='0.1' value='");
  html += String(targetAngle, 2);
  html += F("'></div>");

  html += F("</div><div style='margin-top:14px'><button type='submit'>Update / Save</button></div></form></div>");

  html += F("<div class='card'><h1>สถานะปัจจุบัน</h1><div class='status'>"
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
            "<p class='foot'>ระบบจะดึงข้อมูลใหม่จาก <span class='mono'>/status</span> ทุก 1 วินาที</p>"
            "<p class='foot'><span class='endpoint'>GET /</span><span class='endpoint'>GET /set</span><span class='endpoint'>GET /status</span></p>"
            "</div>");

  html += F(
      "<script>"
      "async function loadStatus(){"
      "try{"
      "const res=await fetch('/status',{cache:'no-store'});"
      "const s=await res.json();"
      "document.getElementById('filteredAngle').textContent=s.filteredAngle+' deg';"
      "document.getElementById('accelAngle').textContent=s.accelAngle+' deg';"
      "document.getElementById('error').textContent=s.error+' deg';"
      "document.getElementById('integral').textContent=s.integral;"
      "document.getElementById('derivative').textContent=s.derivative;"
      "document.getElementById('pidOutput').textContent=s.pidOutput;"
      "document.getElementById('pwm').textContent=s.pwm;"
      "document.getElementById('motorDirection').textContent=s.motorDirection;"
      "document.getElementById('safetyStop').textContent=s.safetyStop ? 'ACTIVE' : 'NORMAL';"
      "}catch(e){console.log(e);}"
      "}"
      "loadStatus();"
      "setInterval(loadStatus,1000);"
      "</script></div></body></html>");

  return html;
}

bool parseFloatValue(const String& text, float& value) {
  char* endPtr = nullptr;
  value = strtof(text.c_str(), &endPtr);
  return endPtr != text.c_str() && endPtr != nullptr && *endPtr == '\0';
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

  // เคลียร์สถานะ PID บางส่วนเพื่อให้ค่าที่อัปเดตเริ่มทำงานแบบไม่กระชากเกินไป
  resetPIDState();

  server.sendHeader("Location", "/", true);
  server.send(303, "text/plain; charset=utf-8", "Updated");
}

void handleNotFound() {
  String message;
  message.reserve(160);
  message += F("ไม่พบ endpoint นี้\n\nใช้งานได้ที่:\n");
  message += F("/\n/set?kp=35&ki=0.8&kd=1.2&deadband=120&targetAngle=0\n/status\n");
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

void connectWiFi() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print(F("กำลังเชื่อม Wi-Fi"));
  const unsigned long startAttempt = millis();

  while (WiFi.status() != WL_CONNECTED && (millis() - startAttempt) < 20000UL) {
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

void printTelemetry() {
  // รูปแบบนี้อ่านผ่าน Serial Monitor ได้ง่าย และเอาไปเปิด Serial Plotter ได้ด้วย
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

void setup() {
  Serial.begin(115200);
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
    Serial.println(F("ไม่สามารถเริ่มต้น MPU6050 ได้ ตรวจสอบสาย SDA/SCL/VCC/GND"));
    while (true) {
      stopMotor();
      delay(1000);
    }
  }

  calibrateGyro();

  // อ่านค่าเซนเซอร์หนึ่งครั้งเพื่อใช้ตั้งต้น filtered angle ให้ใกล้ของจริง
  if (readMPU()) {
    computeAngle(0.01f);
  }

  connectWiFi();
  setupWebServer();

  Serial.println(F("เปิด Serial Monitor เพื่อดู IP แล้วเข้าผ่านเว็บเบราว์เซอร์"));
  Serial.println(F("เริ่มระบบควบคุมแล้ว"));
}

void loop() {
  server.handleClient();

  const unsigned long nowMicros = micros();
  if (lastControlMicros == 0) {
    lastControlMicros = nowMicros;
    return;
  }

  const float dt = static_cast<float>(nowMicros - lastControlMicros) / 1000000.0f;
  lastControlMicros = nowMicros;

  // ป้องกันกรณี dt ผิดปกติ เช่น รอบแรกหรือมีการหยุดนานผิดปกติ
  if (dt < MIN_VALID_DT || dt > MAX_VALID_DT) {
    return;
  }

  if (!readMPU()) {
    stopMotor();

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
