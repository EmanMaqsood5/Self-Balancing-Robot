#include <Wire.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>

// =====================================================================
// Lab 13 — Self-Balancing Robot + Light Following (MIMO Control)
// OPTIMIZED FOR: Aggressive Light Following / High Steering Priority
// =====================================================================

// ─── Data Logging Requirements ───
#define BUFFER_SIZE 2000
float angleBuffer[BUFFER_SIZE];
int bufferIndex = 0;
bool recording = false;
bool recordComplete = false;

// ─── Wi-Fi Setup ──────────────────────────────────
const char* ssid = "ESP_burners";
const char* password = "password123";
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ─── MPU-6050 Setup ───
const int MPU_addr = 0x68;
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
float accAngle = 0, gyroRate = 0, currentAngle = 0, gyroBiasY = 0;
unsigned long lastTime = 0, lastTele = 0, lastSerialPrint = 0;
float dt = 0;

// ─── INNER LOOP: Balance PID Tuning Parameters ───
float baseSetpoint = 0.90; // Your calibrated physical balance angle
float setpoint = 0.90; // Handled dynamically by position loop
float Kp = 50.0;
float Ki = 0.8;
float Kd = 3.0;

// ─── MIDDLE LOOP: Smoothed Position Cascade Control ───
float Kp_pos = 0.008;
float robotPosition = 0;
float filteredPID = 0;
float u_bal = 0;

float safetyLimit = 25.0;
float error = 0, I = 0, PID_value = 0;

// ─── PIN CONFIGURATION & CONTROLS (YOUR HARDWARE) ───
const int PWMA = 25, AIN1 = 26, AIN2 = 27, PWMB = 32, BIN1 = 33, BIN2 = 14;
int deadbandLeft = 35, deadbandRight = 35;
const float MOTOR_DIR_SIGN = 1.0;

// ─── OUTER LOOP: Light-Following Parameters (GPIO 34 & 35) ───
const int LDR_LEFT_PIN = 34;
const int LDR_RIGHT_PIN = 35;

// Hardware balance calibration offset
const int LDR_RIGHT_OFFSET = 140;

int ldrLeft = 0;
int ldrRight = 0;
float e_light = 0;
float e_light_prev = 0;
float I_light = 0;
unsigned long lastLightLoop = 0;

// AGGRESSIVE STEERING GAINS (Increased authority for rapid tracking)
float Kp_L = 0.25; // Increased from 0.08 for sharp, instant turn response
float Ki_L = 0.008;
float Kd_L = 0.08; // Increased from 0.02 to whip the motors during fast transitions
float u_dir = 0;

// ─── Forward Declarations ───
void readMPU();
void moveMotors(float bal, float dir);
void stopMotors();

// =====================================================================
// MODERN CONTROL PANEL DASHBOARD
// =====================================================================
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<title>NeoBot Control Panel</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
*{ margin:0; padding:0; box-sizing:border-box; }
body{ background:#0f172a; font-family:'Trebuchet MS',sans-serif; color:white; min-height:100vh; display:flex; justify-content:center; align-items:center; padding:20px; }
.panel{ width:100%; max-width:520px; background:#111827; border:2px solid #1e293b; border-radius:24px; overflow:hidden; box-shadow:0 0 35px rgba(0,0,0,0.5); }
.topbar{ background:linear-gradient(135deg,#06b6d4,#3b82f6); padding:22px; text-align:center; }
.topbar h1{ font-size:2rem; letter-spacing:1px; }
.topbar p{ font-size:0.9rem; opacity:0.9; margin-top:5px; }
.content{ padding:20px; }
.angleBox{ background:#1e293b; border-radius:20px; padding:22px; text-align:center; margin-bottom:18px; border:1px solid #334155; }
.angleBox span{ display:block; font-size:0.9rem; color:#94a3b8; margin-bottom:10px; }
#angleDisp{ font-size:4rem; font-weight:bold; color:#22d3ee; font-family:monospace; }
.sensorGrid{ display:grid; grid-template-columns:repeat(3,1fr); gap:12px; margin-bottom:20px; }
.sensor{ background:#1e293b; padding:14px; border-radius:16px; text-align:center; border:1px solid #334155; }
.sensor h4{ font-size:0.7rem; color:#94a3b8; margin-bottom:8px; }
.sensor div{ font-size:1.4rem; font-weight:bold; color:#f8fafc; }
.heading{ margin:18px 0 10px; font-size:0.85rem; color:#38bdf8; text-transform:uppercase; letter-spacing:2px; }
.card{ background:#1e293b; border-radius:16px; padding:14px; margin-bottom:12px; display:flex; justify-content:space-between; align-items:center; border:1px solid #334155; }
.card label{ font-weight:bold; font-size:0.95rem; }
.card input{ width:110px; padding:10px; border:none; border-radius:10px; background:#0f172a; color:#22d3ee; text-align:center; font-size:0.95rem; outline:none; }
.stop{ width:100%; margin-top:18px; padding:16px; border:none; border-radius:18px; background:#ef4444; color:white; font-size:1rem; font-weight:bold; cursor:pointer; transition:0.2s; }
.stop:hover{ background:#dc2626; transform:translateY(-2px); }
.bottom{ text-align:center; margin-top:18px; color:#64748b; font-size:0.8rem; }
@media(max-width:500px){ .sensorGrid{ grid-template-columns:1fr; } #angleDisp{ font-size:3rem; } }
</style>
</head>
<body>
<div class="panel">
  <div class="topbar">
    <h1>NeoBot Dashboard</h1>
    <p>Self Balancing + Light Tracking System</p>
  </div>
  <div class="content">
    <div class="angleBox">
      <span>Current Tilt Angle</span>
      <div id="angleDisp">0.0°</div>
    </div>
    <div class="sensorGrid">
      <div class="sensor"><h4>LEFT SENSOR</h4><div id="ldrL">0</div></div>
      <div class="sensor"><h4>RIGHT SENSOR</h4><div id="ldrR">0</div></div>
      <div class="sensor"><h4>LIGHT ERROR</h4><div id="ldrE">0</div></div>
    </div>
    <div class="heading">Balance Controller</div>
    <div class="card"><label>Kp</label><input type="number" step="0.5" value="50" onchange="u('P',this.value)"></div>
    <div class="card"><label>Ki</label><input type="number" step="0.1" value="0.8" onchange="u('I',this.value)"></div>
    <div class="card"><label>Kd</label><input type="number" step="0.1" value="3" onchange="u('D',this.value)"></div>
    <div class="card"><label>Setpoint</label><input type="number" step="0.1" value="0.9" onchange="u('S',this.value)"></div>
    <div class="card"><label>Safety Limit</label><input type="number" step="1" value="25" onchange="u('L',this.value)"></div>
    <div class="heading">Light Tracking Controller</div>
    <div class="card"><label>Kp_L</label><input type="number" step="0.01" value="0.25" onchange="u('LP',this.value)"></div>
    <div class="card"><label>Ki_L</label><input type="number" step="0.001" value="0.008" onchange="u('LI',this.value)"></div>
    <div class="card"><label>Kd_L</label><input type="number" step="0.01" value="0.08" onchange="u('LD',this.value)"></div>
    <button class="stop" onclick="u('STOP',0)">EMERGENCY STOP</button>
    <div class="bottom">WiFi PID Monitoring Interface</div>
  </div>
</div>
<script>
var w = new WebSocket(`ws://${location.hostname}/ws`);
w.onclose = () => setTimeout(() => location.reload(), 2000);
w.onmessage = (e) => {
  var parts = e.data.split(',');
  if(parts.length === 4){
    document.getElementById('angleDisp').innerHTML = parts[0] + '°';
    document.getElementById('ldrL').innerHTML = parts[1];
    document.getElementById('ldrR').innerHTML = parts[2];
    document.getElementById('ldrE').innerHTML = parts[3];
  }
};
function u(c,v){ if(w.readyState===1 && v!=="") w.send(c + '=' + v); }
</script>
</body>
</html>
)rawliteral";

// ─── WebSocket Event Handler ───
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
             AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_DATA) {
    data[len] = 0;
    String msg = (char*)data;
    char p = msg.charAt(0);
    float v = msg.substring(2).toFloat();
    if (p == 'P') Kp = v;
    else if (p == 'I') Ki = v;
    else if (p == 'D') Kd = v;
    else if (p == 'S') { setpoint = v; baseSetpoint = v; }
    else if (p == 'L') safetyLimit = v;
    else if (msg.startsWith("LP")) Kp_L = v;
    else if (msg.startsWith("LI")) Ki_L = v;
    else if (msg.startsWith("LD")) Kd_L = v;
    else if (msg.startsWith("STOP")) { stopMotors(); I = 0; I_light = 0; u_dir = 0; }
  }
}

// =====================================================================
// SETUP
// =====================================================================
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); Wire.write(0);
  Wire.endTransmission(true);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  ws.onEvent(onEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });
  server.begin();

  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  stopMotors();
  analogWriteFrequency(PWMA, 20000);
  analogWriteFrequency(PWMB, 20000);

  // Gyro Calibration
  long sum = 0;
  for (int i = 0; i < 500; i++) { readMPU(); sum += GyX; delay(2); }
  gyroBiasY = sum / 500.0;
  lastTime = micros();
  setpoint = baseSetpoint;
}

// =====================================================================
// MAIN LOOP
// =====================================================================
void loop() {
  ws.cleanupClients();

  unsigned long currentTime = micros();
  dt = (currentTime - lastTime) / 1000000.0;
  if (dt < 0.005) return;
  lastTime = currentTime;

  readMPU();

  accAngle = atan2(AcY, -AcZ) * 180.0 / PI;

  // Safety Trip-switch
  if (abs(accAngle) > safetyLimit || abs(accAngle - baseSetpoint) > safetyLimit) {
    stopMotors();
    I = 0;
    I_light = 0;
    robotPosition = 0;
    filteredPID = 0;
    u_dir = 0;
    return;
  }

  gyroRate = -((GyX - gyroBiasY) / 131.0);
  currentAngle = 0.95 * (currentAngle + gyroRate * dt) + 0.05 * accAngle;

  // ────────────────────────────────────────────────────────────────
  // OUTER LOOP — Aggressive Light Following @ 10 Hz
  // ────────────────────────────────────────────────────────────────
  if (millis() - lastLightLoop >= 100) {
    lastLightLoop = millis();

    ldrLeft = analogRead(LDR_LEFT_PIN);
    ldrRight = analogRead(LDR_RIGHT_PIN);

    int calibratedRight = ldrRight - LDR_RIGHT_OFFSET;
    if (calibratedRight < 0) calibratedRight = 0;

    e_light = (float)(ldrLeft - calibratedRight);

    // ULTRA-LOW SENSITIVITY DEADBAND: Lowered to 20 for near-instant tracking triggers
    int threshold = 20;
    if (abs(e_light) < threshold) {
      e_light = 0;
    }

    // Lightweight filtering to prevent structural loop locking
    static float e_buf[2] = {0, 0};
    e_buf[1] = e_buf[0]; e_buf[0] = e_light;
    e_light = (e_buf[0] + e_buf[1]) / 2.0;

    // Steering PID Logic
    float dt_light = 0.1;
    float P_L = Kp_L * e_light;
    I_light += Ki_L * e_light * dt_light;
    I_light = constrain(I_light, -30, 30);
    float D_L = Kd_L * (e_light - e_light_prev) / dt_light;
    e_light_prev = e_light;

    u_dir = P_L + I_light + D_L;

    // MAXIMUM AUTHORITY BOOST: Allowed steering up to 110 for high-torque fast snaps
    u_dir = constrain(u_dir, -110, 110);
  }

  // ─── Middle Loop: Position Cascade ───
  filteredPID = (0.99 * filteredPID) + (0.01 * u_bal);
  robotPosition += filteredPID * dt;
  robotPosition = constrain(robotPosition, -40.0, 40.0);

  float setpointOffset = robotPosition * Kp_pos;
  setpointOffset = constrain(setpointOffset, -3.0, 3.0);
  setpoint = baseSetpoint + setpointOffset;

  // ─── Inner Loop: Angle PID ───
  error = currentAngle - setpoint;
  if (abs(error) < 6.0) {
    I += Ki * error * dt;
    I = constrain(I, -40.0, 40.0);
  } else {
    I *= 0.2;
  }

  float D = Kd * gyroRate;
  PID_value = ((Kp * error) + I + D) * MOTOR_DIR_SIGN;
  u_bal = PID_value;

  // ─── Telemetry Outputs ───
  if (millis() - lastSerialPrint > 100) {
    Serial.print(currentAngle, 3); Serial.print(",");
    Serial.print(ldrLeft); Serial.print(",");
    Serial.print(ldrRight); Serial.print(",");
    Serial.print(u_dir, 3); Serial.print(",");
    Serial.println(u_bal, 3);
    lastSerialPrint = millis();
  }

  if (millis() - lastTele > 100) {
    String payload = String(currentAngle, 1) + "," +
                     String(ldrLeft) + "," +
                     String(ldrRight) + "," +
                     String((int)e_light);
    ws.textAll(payload);
    lastTele = millis();
  }

  moveMotors(u_bal, u_dir);
}

// =====================================================================
// MPU-6050 READ UTILITY
// =====================================================================
void readMPU() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B); Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();
}

// =====================================================================
// MOTOR DRIVER CONTROL MATRIX (MIMO Mixing Logic)
// =====================================================================
void moveMotors(float bal, float dir) {
  float cmdLeft = bal + dir;
  float cmdRight = bal - dir;

  static float filtL = 0, filtR = 0;
  filtL = 0.4 * filtL + 0.6 * cmdLeft;
  filtR = 0.4 * filtR + 0.6 * cmdRight;

  filtL = constrain(filtL, -255, 255);
  filtR = constrain(filtR, -255, 255);

  if (filtL >= 0) {
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
  } else {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  }
  if (filtR >= 0) {
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
  } else {
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
  }

  int s_left = (abs(filtL) > 0) ? ((int)abs(filtL) + deadbandLeft) : 0;
  int s_right = (abs(filtR) > 0) ? ((int)abs(filtR) + deadbandRight) : 0;

  analogWrite(PWMA, constrain(s_left, 0, 255));
  analogWrite(PWMB, constrain(s_right, 0, 255));
}

// =====================================================================
// EMERGENCY MOTOR TERMINATION
// =====================================================================
void stopMotors() {
  analogWrite(PWMA, 0); analogWrite(PWMB, 0);
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
}
