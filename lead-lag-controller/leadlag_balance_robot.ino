#include <Wire.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>

// ─── Data Logging Requirements ───
#define BUFFER_SIZE 2000
float angleBuffer[BUFFER_SIZE];
int bufferIndex = 0;
bool recording = false;
bool recordComplete = false;

// ─── Existing Setup ──────────────────────────────────
const char* ssid = "ESP_burners";
const char* password = "password123";
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

const int MPU_addr = 0x68;
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
float accAngle = 0, gyroRate = 0, currentAngle = 0, gyroBiasY = 0;
unsigned long lastTime = 0, lastTele = 0, lastSerialPrint = 0;
float dt = 0;

// ─── Reference Tuning Parameters ──────────────────────
float baseSetpoint = 0.90; // Calibrated physical balance angle
float setpoint = 0.90; // Handled dynamically by position loop

// Outer Loop (Position) Control
float Kp_pos = 0.008;
float robotPosition = 0;
float filteredPID = 0;

float safetyLimit = 25.0;
float PID_value = 0;

const int PWMA = 25, AIN1 = 26, AIN2 = 27, PWMB = 32, BIN1 = 33, BIN2 = 14;
int deadbandLeft = 35, deadbandRight = 35;
const float MOTOR_DIR_SIGN = 1.0;

// ─── LEAD-LAG CONTROLLER STORAGE & COEFFICIENTS (200Hz Tustin) ───
// Error historic states: e[k], e[k-1], e[k-2]
float e0 = 0.0, e1 = 0.0, e2 = 0.0;
// Control historic states: u[k], u[k-1], u[k-2]
float u0 = 0.0, u1 = 0.0, u2 = 0.0;

// Normalized discrete transfer function parameters from MATLAB script
const float b0 = 77.88543924;
const float b1 = -152.22080282;
const float b2 = 74.36797451;
const float a1 = -1.72548152;
const float a2 = 0.72591912;

void readMPU();
void moveMotors(float speed);
void stopMotors();

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); Wire.write(0); Wire.endTransmission(true);

  WiFi.mode(WIFI_AP); WiFi.softAP(ssid, password);

  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  analogWriteFrequency(PWMA, 20000);
  analogWriteFrequency(PWMB, 20000);

  // Gyro Calibration (Hold the robot perfectly still during boot!)
  long sum = 0;
  for (int i = 0; i < 500; i++) { readMPU(); sum += GyX; delay(2); }
  gyroBiasY = sum / 500.0;
  lastTime = micros();
  setpoint = baseSetpoint;
  Serial.print("Your base stable setpoint is: ");
  Serial.println(baseSetpoint);
}

void loop() {
  unsigned long currentTime = micros();
  dt = (currentTime - lastTime) / 1000000.0;
  if (dt < 0.005) return; // Fixes loop rate precisely at 200Hz (Matches Ts=0.005s)
  lastTime = currentTime;

  readMPU();

  // Calculate Accelerometer Angle
  accAngle = atan2(AcY, -AcZ) * 180.0 / PI;

  // 🛡️ Safety Wrap: Shuts off motors and flushes historical states if robot falls over
  if (abs(accAngle) > 40.0 || abs(accAngle - baseSetpoint) > 40.0) {
    stopMotors();
    robotPosition = 0;
    filteredPID = 0;

    // CRITICAL: Flush Lead-Lag historic buffer to prevent windup upon recovery
    e0 = e1 = e2 = 0.0;
    u0 = u1 = u2 = 0.0;
    return;
  }

  // Convert raw gyro to degrees/sec and apply calibration bias
  gyroRate = -((GyX - gyroBiasY) / 131.0);

  // Complementary Filter
  currentAngle = 0.95 * (currentAngle + gyroRate * dt) + 0.05 * accAngle;

  // ─── Outer Loop: Smoothed Position Tracking ───
  filteredPID = (0.99 * filteredPID) + (0.01 * PID_value);
  //robotPosition += filteredPID * dt;
  // robotPosition = constrain(robotPosition, -40.0, 40.0);
  robotPosition = 0;

  float setpointOffset = robotPosition * Kp_pos;
  setpointOffset = constrain(setpointOffset, -3.0, 3.0); // Maximum 3 degrees tilt correction
  setpoint = baseSetpoint;

  // ─── Inner Loop: Model-Based Lead-Lag Compensator ───
  // Shift error history
  e2 = e1;
  e1 = e0;
  e0 = currentAngle - setpoint; // Current Error e[k]

  // Shift control history
  u2 = u1;
  u1 = u0;

  // Calculate Direct Difference Equation:
  // u[k] = b0*e[k] + b1*e[k-1] + b2*e[k-2] - a1*u[k-1] - a2*u[k-2]
  u0 = (b0 * e0) + (b1 * e1) + (b2 * e2) - (a1 * u1) - (a2 * u2);

  // Apply scaling / assignment to plant output mapping
  PID_value = u0 * 2.5 * MOTOR_DIR_SIGN;

  // Serial Debugging output
  if (millis() - lastSerialPrint > 100) {
    Serial.print("Angle: "); Serial.print(currentAngle);
    Serial.print(" | Target Setpoint: "); Serial.print(setpoint);
    Serial.print(" | Smooth Pos: "); Serial.print(robotPosition);
    Serial.print(" | Lead-Lag Output: "); Serial.println(PID_value);
    lastSerialPrint = millis();
  }

  moveMotors(PID_value);
}

void readMPU() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B); Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read(); // Skip temperature
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();
}

void moveMotors(float speed) {
  speed = constrain(speed, -155.0, 155.0);
  int motorSpeed = (int)abs(speed);

  // Apply deadband correction so motors break static friction instantly
  int s = (motorSpeed > 0) ? (motorSpeed + deadbandLeft) : 0;
  s = constrain(s, 0, 255);

  if (speed > 0) {
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
  } else {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
  }
  analogWrite(PWMA, s); analogWrite(PWMB, s);
}

void stopMotors() {
  analogWrite(PWMA, 0); analogWrite(PWMB, 0);
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
}
