#include <Wire.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>

// ─── Data Logging Requirements ──
#define BUFFER_SIZE 2000
float angleBuffer[BUFFER_SIZE];
int bufferIndex = 0;
bool recording = false;
bool recordComplete = false;

// ─── Existing Setup ──────────────────────────────────
const char* ssid     = "ESP_burners";
const char* password = "password123";
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

const int MPU_addr = 0x68;
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
float accAngle = 0, gyroRate = 0, currentAngle = 0, gyroBiasY = 0;
unsigned long lastTime = 0, lastTele = 0, lastSerialPrint = 0;
float dt = 0;

// ─── PID Tuning Parameters (START WITH STEP 1 BELOW) ───
float baseSetpoint = 0.90; // Your calibrated physical balance angle
float setpoint = 0.90;     // Handled dynamically by position loop

// Inner Loop (Angle) PID
float Kp = 50.0;   // Lowered slightly to start tuning safely
float Ki = 0.8;    // Keep 0 until Step 2
float Kd = 3;     // Gyro-based D requires lower values (Start around 0.2 - 0.8)

// Outer Loop (Position) Control
float Kp_pos = 0.008; // CRITICAL: Keep 0.0 until Step 3!
float robotPosition = 0;
float filteredPID = 0;

float safetyLimit = 25.0;
float error = 0, I = 0, PID_value = 0;

const int PWMA = 25, AIN1 = 26, AIN2 = 27, PWMB = 32, BIN1 = 33, BIN2 = 14;
int deadbandLeft = 35, deadbandRight = 35; // Adjust if one motor starts late
const float MOTOR_DIR_SIGN = 1.0;

void readMPU();

// [NOT VISIBLE IN PDF: lines 46-47 between "void readMPU();" and the HTML below.
//  The HTML string is referenced as index_html (see send_P in setup()), and the
//  block ends with )rawliteral"; so it most likely opens with something like:
//  const char index_html[] PROGMEM = R"rawliteral(  ]

<!DOCTYPE html>
<html>
<head>
<title>ESP Controller</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
*{ margin:0; padding:0; box-sizing:border-box; }
body{ background:#09090b; font-family:Verdana,sans-serif; color:white; min-height:100vh; display:flex; justify-content
.container{ width:100%; max-width:420px; background:#18181b; border-radius:28px; padding:24px; box-shadow: 0 0 30px rgb
.header{ text-align:center; margin-bottom:22px; }
.header h2{ font-size:2rem; color:#c084fc; letter-spacing:1px; }
.liveCard{ background:linear-gradient(145deg,#27272a,#18181b); border-radius:22px; padding:22px; text-align:center; mar
.liveText{ color:#a1a1aa; font-size:0.9rem; margin-bottom:10px; }
#angleDisp{ font-size:3.8rem; font-weight:bold; color:#f472b6; font-family:monospace; text-shadow:0 0 15px rgba(244,114
.grid{ display:flex; flex-direction:column; gap:14px; }
.control{ background:#232326; border-radius:16px; padding:14px 16px; display:flex; justify-content:space-between; alig
.control label{ font-size:1rem; color:#e4e4e7; font-weight:bold; }
.control input{ width:110px; padding:10px; border:none; outline:none; border-radius:10px; background:#09090b; color:#f4
.stopBtn{ width:100%; margin-top:22px; padding:16px; border:none; border-radius:18px; background:linear-gradient(90deg,
.stopBtn:hover{ transform:scale(1.03); opacity:0.95; }
.footer{ text-align:center; margin-top:16px; color:#71717a; font-size:0.8rem; }
</style>
</head>
<body>
<div class="container">
  <div class="header"><h2>ESP BURNERS</h2></div>
  <div class="liveCard">
    <div class="liveText">CURRENT ANGLE</div>
    <div id="angleDisp">0.0°</div>
  </div>
  <div class="grid">
    <div class="control">
      <label>Kp</label>
      <input type="number" step="0.5" value="28.0" onchange="u('P',this.value)">
    </div>
    <div class="control">
      <label>Ki</label>
      <input type="number" step="0.05" value="4" onchange="u('I',this.value)">
    </div>
    <div class="control">
      <label>Kd</label>
      <input type="number" step="0.05" value="4.0" onchange="u('D',this.value)">
    </div>
    <div class="control">
      <label>Setpoint</label>
      <input type="number" step="0.1" value="-2.6" onchange="u('S',this.value)">
    </div>
    <div class="control">
      <label>Safety</label>
      <input type="number" step="1" value="25" onchange="u('L',this.value)">
    </div>
  </div>
  <button class="stopBtn" onclick="u('STOP',0)">EMERGENCY STOP</button>
  <div class="footer">Wireless PID Dashboard</div>
</div>
<script>
var w = new WebSocket(`ws://${location.hostname}/ws`);
w.onclose = () => setTimeout(() => location.reload(),2000);
w.onmessage = (e) => { document.getElementById('angleDisp').innerHTML = e.data + '°'; };
function u(c,v){ if(w.readyState===1 && v!==""){ w.send(c + '=' + v); } }
</script>
</body>
</html>
)rawliteral";

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t le   /* line truncated in PDF: remainder of the parameter list not visible */
    if (type == WS_EVT_DATA) {
        data[len] = 0;
        String msg = (char*)data;
        char p = msg.charAt(0);
        float v = msg.substring(2).toFloat();
        if      (p == 'P') Kp = v;
        else if (p == 'I') Ki = v;
        else if (p == 'D') Kd = v;
        else if (p == 'S') setpoint = v;
        else if (p == 'L') safetyLimit = v;
        else if (msg.startsWith("STOP")) isEnabled = false;
    }
}

void setup() {
    Serial.begin(115200);

    // ✅ UPDATED I2C pins for NodeMCU ESP-32S
    Wire.begin(21, 22);   // SDA = GPIO 21, SCL = GPIO 22

    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password);
    ws.onEvent(onEvent);
    server.addHandler(&ws);
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send_P(200, "text/html", index_html);
    });
    server.begin();

    Wire.beginTransmission(MPU_addr);
    Wire.write(0x6B); Wire.write(0);
    Wire.endTransmission(true);

    pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
    pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
    stopMotors();

    long sum = 0;
    for (int i = 0; i < 500; i++) { readMPU(); sum += GyY; delay(2); }
    gyroBiasY = sum / 500.0;
    lastTime = micros();
}

void loop() {
    ws.cleanupClients();

    unsigned long currentTime = micros();
    dt = (currentTime - lastTime) / 1000000.0;
    if (dt < 0.005) return;
    // [NOT VISIBLE IN PDF: lines 163-164, partly hidden by the image seam
    //  between pages 24's two screenshots - likely "lastTime = currentTime;"]

    readMPU();
    accAngle     = atan2(-AcY, -AcZ) * 180.0 / PI;
    gyroRate     = (GyY - gyroBiasY) / 131.0;
    currentAngle = 0.98 * (currentAngle + gyroRate * dt) + 0.02 * accAngle;

    unsigned long currentMICROS = micros();
    if (currentMICROS - lastSerialPrint >= 500) {
        Serial.println(currentAngle);
        lastSerialPrint = currentMICROS;
    }

    if (millis() - lastTele > 100) {
        ws.textAll(String(currentAngle, 1));
        lastTele = millis();
    }

    if (abs(currentAngle) > safetyLimit) isEnabled = false;
    if (!isEnabled && abs(currentAngle - setpoint) < 3) { isEnabled = true; I = 0; }
    if (!isEnabled) { stopMotors(); return; }

    error = currentAngle - setpoint;
    P = Kp * error;
    if (abs(error) < 10) { I += Ki * error * dt; I = constrain(I, -40, 40); } else { I *= 0.9; }
    D = Kd * (error - previousError) / dt;
    PID_value = P + I + D;
    previousError = error;

    if (abs(PID_value) < 10) PID_value = 0;
    moveMotors(PID_value);
}

void readMPU() {
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 14, true);
    AcX = Wire.read() << 8 | Wire.read();
    AcY = Wire.read() << 8 | Wire.read();
    AcZ = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read();
    GyX = Wire.read() << 8 | Wire.read();
    GyY = Wire.read() << 8 | Wire.read();
    GyZ = Wire.read() << 8 | Wire.read();
}

void moveMotors(float speed) {
    static float filteredSpeed = 0;
    filteredSpeed = 0.5 * filteredSpeed + 0.5 * speed;
    filteredSpeed = constrain(filteredSpeed, -255, 255);
    int motorSpeed = abs(filteredSpeed);
    int speedL = (motorSpeed > 0) ? (motorSpeed + deadbandLeft)  : 0;
    int speedR = (motorSpeed > 0) ? (motorSpeed + deadbandRight) : 0;

    if (filteredSpeed > 0) {
        digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH);
        digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
    } else {
        digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
        digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH);
    }
    analogWrite(PWMA, constrain(speedL, 0, 255));
    analogWrite(PWMB, constrain(speedR, 0, 255));
}

void stopMotors() {
    analogWrite(PWMA, 0); analogWrite(PWMB, 0);
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
}
