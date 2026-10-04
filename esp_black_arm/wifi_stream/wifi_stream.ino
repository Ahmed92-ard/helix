/*
 * BLACK ARM CONTROLLER — ESP32 + PCA9685 (Dual-Arm Edition)
 * Hardware: ESP32 + PCA9685 (I2C: SDA=GPIO21, SCL=GPIO22)
 *
 * Role in Dual-Arm system:
 *   • Receives commands from Python Black GUI on port 8888
 *   • Executes A→C motion: picks from A, places at Press (C)
 *   • On PRESS_SYNC: connects to Press ESP2 (port 9999),
 *     sends HELLO → READY_PRESS, waits for PRESS_DONE
 *   • Press ESP2 then notifies White Arm independently
 *
 * E-STOP BUTTON: GPIO18 → GND (momentary push button, INPUT_PULLUP)
 *   Press  → arm stops immediately, notifies Python
 *   Resume → GUI sends RESUME_BTN → arm resumes
 *
 * Commands (after HELLO handshake):
 *   HELLO                          → READY
 *   MOVE:a0,a1,a2,a3,a4,a5,speed  → smooth move all 6 servos → MOVE_DONE
 *   SERVO:idx,angle,speed          → smooth move single servo  → SERVO_DONE:idx
 *   HOME                           → move to home angles       → HOME_DONE
 *   STOP                           → stop current move         → STOPPED
 *   SPEED:val                      → set step delay
 *   RESUME_BTN                     → clear estop flag          → BTN_RESUMED
 *   PRESS_SYNC                     → trigger ESP2 press cycle  → PRESS_DONE / PRESS_ERROR:*
 *   SET_ESP2_IP:x.x.x.x            → update ESP2 target IP
 *   PRESS_CHECK                    → verify ESP2 reachable
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFi.h>
#include "esp_wifi_config.h"

#define TCP_PORT  ESP1_TCP_PORT

// Forward declarations
void handleCommand(const char* line, int source);
void processChar(char c, char* buf, int& idx, int source);
void drainTcpInPauseLoop();
void smoothMove(int targets[6], int stepDelay, bool freezeGripper);
void smoothMoveSingle(int idx, int target, int stepDelay);
bool triggerPressMachine();
void sendStopToEsp2();

// ── PCA9685 ──────────────────────────────────────────────
#define PCA9685_ADDR     0x40
#define PCA9685_OSC_FREQ 27000000
#define PWM_FREQ_HZ      50

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);

// ── Servo config ─────────────────────────────────────────
#define NUM_SERVOS 6
const int SERVO_MIN_US[NUM_SERVOS] = { 500, 500, 500, 500, 500, 500 };
const int SERVO_MAX_US[NUM_SERVOS] = { 2400, 2400, 2400, 2400, 2400, 2400 };
const int SERVO_CH[NUM_SERVOS]     = { 0, 1, 2, 3, 4, 5 };

#define GRIPPER_IDX   5
#define GRIPPER_OPEN  40
#define GRIPPER_CLOSE 15

const int HOME_ANGLES[NUM_SERVOS] = { 90, 90, 143, 90, 90, GRIPPER_OPEN };

#define DEFAULT_STEP_DELAY 10
#define HOME_STEP_DELAY    20
int g_stepDelay = DEFAULT_STEP_DELAY;

// ── E-STOP button ─────────────────────────────────────────
#define ESTOP_BTN_PIN 18

volatile bool estopFlag    = false;
bool estopReported         = false;

volatile unsigned long lastBtnMs = 0;

void IRAM_ATTR onEstopPressed() {
  unsigned long now = millis();
  if (now - lastBtnMs < 200) return;
  lastBtnMs = now;
  estopFlag = true;
}

// ── IR stop flag (from ESP2 IR sensors) ──────────────────
volatile bool irStopFlag = false;

// ── Software stop flag ────────────────────────────────────
bool softStopFlag = false;

// ── Servo state ───────────────────────────────────────────
int  currentAngle[NUM_SERVOS];
bool servosEnabled = false;

// ── Serial / WiFi buffers ─────────────────────────────────
#define BUF_SIZE 256
char serialBuf[BUF_SIZE];  int serialBufIdx = 0;  bool serialHandshake = false;
char wifiBuf[BUF_SIZE];    int wifiBufIdx   = 0;  bool wifiHandshake   = false;

// ── WiFi TCP server ───────────────────────────────────────
WiFiServer tcpServer(TCP_PORT);
WiFiClient tcpClient;

// ── Press ESP2 target ─────────────────────────────────────
IPAddress esp2Host(ESP2_HOST_IP);
String serialMonitorLine;

// ═════════════════════════════════════════════════════════
//  OUTPUT helpers
// ═════════════════════════════════════════════════════════
void replyAll(const char* msg) {
  Serial.println(msg);
  if (tcpClient && tcpClient.connected()) {
    tcpClient.println(msg);
  }
}

void replyAllInt(const char* prefix, int val) {
  char buf[64];
  snprintf(buf, sizeof(buf), "%s%d", prefix, val);
  replyAll(buf);
}

// ── Read a line from a WiFiClient with timeout ────────────
bool readLineWithTimeout(WiFiClient& client, char* buf, size_t bufSize, unsigned long timeoutMs) {
  size_t idx = 0;
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    while (client.available()) {
      char c = (char)client.read();
      if (c == '\r') continue;
      if (c == '\n') { buf[idx] = '\0'; return true; }
      if (idx < bufSize - 1) buf[idx++] = c;
    }
    delay(1);
  }
  buf[idx] = '\0';
  return false;
}

// ═════════════════════════════════════════════════════════
//  Send STOP to ESP2 (press) over WiFi after e-stop
// ═════════════════════════════════════════════════════════
void sendStopToEsp2() {
  WiFiClient c;
  Serial.println("[ESTOP] Sending STOP to ESP2 via WiFi...");
  if (c.connect(esp2Host, ESP2_TCP_PORT, 800)) {
    c.setNoDelay(true);
    c.print("STOP\n");
    c.flush();
    c.stop();
    Serial.println("[ESTOP] STOP sent to ESP2");
  } else {
    Serial.println("[ESTOP] Could not reach ESP2");
  }
}

// ═════════════════════════════════════════════════════════
//  Trigger ESP2 press cycle (blocking — called from PRESS_SYNC)
//  Black Arm calls this; after PRESS_DONE Press ESP2 separately
//  notifies White Arm via PICK_READY.
// ═════════════════════════════════════════════════════════
bool triggerPressMachine() {
  char rxBuf[64];
  const int MAX_RETRIES = 3;
  const int RETRY_DELAY_MS = 1000;

  for (int attempt = 1; attempt <= MAX_RETRIES; attempt++) {
    if (estopFlag) {
      Serial.println("[PRESS_SYNC] Aborted — e-stop active");
      replyAll("STOPPED");
      return false;
    }

    WiFiClient pressClient;
    Serial.printf("[PRESS_SYNC] Connecting to Press ESP2 (attempt %d)\n", attempt);

    if (!pressClient.connect(esp2Host, ESP2_TCP_PORT, ESP2_CONNECT_TIMEOUT_MS)) {
      Serial.printf("[PRESS_SYNC] Connect failed (attempt %d)\n", attempt);
      pressClient.stop();
      if (attempt < MAX_RETRIES) { delay(RETRY_DELAY_MS); continue; }
      replyAll("PRESS_ERROR:CONNECT");
      return false;
    }

    pressClient.setNoDelay(true);
    pressClient.println("HELLO");
    pressClient.flush();

    if (!readLineWithTimeout(pressClient, rxBuf, sizeof(rxBuf), 2000) || strstr(rxBuf, "READY") == NULL) {
      Serial.printf("[PRESS_SYNC] Bad HELLO reply: '%s'\n", rxBuf);
      pressClient.stop();
      if (attempt < MAX_RETRIES) { delay(RETRY_DELAY_MS); continue; }
      replyAll("PRESS_ERROR:HELLO");
      return false;
    }

    pressClient.println("READY_PRESS");
    pressClient.flush();

    if (!readLineWithTimeout(pressClient, rxBuf, sizeof(rxBuf), ESP2_CONNECT_TIMEOUT_MS)) {
      Serial.println("[PRESS_SYNC] No PRESS_ACK");
      replyAll("PRESS_ERROR:ACK_TIMEOUT");
      pressClient.stop();
      return false;
    }
    if (strstr(rxBuf, "PRESS_ACK") == NULL) {
      Serial.printf("[PRESS_SYNC] Unexpected ACK: %s\n", rxBuf);
      replyAll("PRESS_ERROR:ACK");
      pressClient.stop();
      return false;
    }

    unsigned long deadline = millis() + ESP2_RESPONSE_TIMEOUT_MS;
    size_t rxIdx = 0;
    unsigned long lastKeepalive = millis();

    while ((long)(millis() - deadline) < 0) {
      if (estopFlag) {
        Serial.println("[PRESS_SYNC] Aborted by e-stop while waiting for DONE");
        replyAll("STOPPED");
        pressClient.stop();
        return false;
      }
      if (millis() - lastKeepalive >= 3000) {
        lastKeepalive = millis();
        replyAll("PRESS_WAIT");
      }
      if (!pressClient.connected() && !pressClient.available()) {
        replyAll("PRESS_ERROR:DONE_TIMEOUT");
        pressClient.stop();
        return false;
      }
      while (pressClient.available()) {
        char c = (char)pressClient.read();
        if (c == '\r') continue;
        if (c == '\n') {
          rxBuf[rxIdx] = '\0';
          if (strstr(rxBuf, "PRESS_DONE") != NULL) {
            Serial.println("[PRESS_SYNC] Press completed — White Arm will be notified by ESP2");
            replyAll("PRESS_DONE");
            pressClient.stop();
            return true;
          }
          Serial.printf("[PRESS_SYNC] Press ended early: '%s'\n", rxBuf);
          replyAll(rxBuf);
          pressClient.stop();
          return false;
        }
        if (rxIdx < sizeof(rxBuf) - 1) rxBuf[rxIdx++] = c;
      }
      delay(5);
    }

    Serial.println("[PRESS_SYNC] Timeout waiting for PRESS_DONE");
    replyAll("PRESS_ERROR:DONE_TIMEOUT");
    pressClient.stop();
    return false;
  }

  replyAll("PRESS_ERROR:CONNECT");
  return false;
}

// ═════════════════════════════════════════════════════════
//  PCA9685 helpers
// ═════════════════════════════════════════════════════════
uint16_t usToPca(int us) {
  float period_us = 1000000.0f / PWM_FREQ_HZ;
  return (uint16_t)((float)us / period_us * 4096.0f);
}

uint16_t angleToPca(int idx, int angle) {
  angle = constrain(angle, 0, 180);
  int us = map(angle, 0, 180, SERVO_MIN_US[idx], SERVO_MAX_US[idx]);
  return usToPca(us);
}

void writeServo(int idx, int angle) {
  pwm.setPWM(SERVO_CH[idx], 0, angleToPca(idx, angle));
}

// ═════════════════════════════════════════════════════════
//  SETUP
// ═════════════════════════════════════════════════════════
void setup() {
  Serial.begin(9600);

  pinMode(ESTOP_BTN_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ESTOP_BTN_PIN), onEstopPressed, FALLING);
  Serial.println("[SETUP] E-stop button on GPIO18");

  Wire.begin(21, 22);
  Wire.setClock(400000);
  pwm.begin();
  pwm.setOscillatorFrequency(PCA9685_OSC_FREQ);
  pwm.setPWMFreq(PWM_FREQ_HZ);
  delay(10);
  Serial.println("[SETUP] PCA9685 ready");

  for (int i = 0; i < NUM_SERVOS; i++) currentAngle[i] = HOME_ANGLES[i];

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("[WIFI] Connecting to %s\n", WIFI_SSID);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500);
    Serial.printf("[WIFI] Status: %d | Attempt: %d\n", WiFi.status(), attempts);
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    WiFi.setSleep(false); // Disable WiFi sleep to prevent connection drops/timeouts
    delay(1000);
    Serial.println("\n--- BLACK ARM WIFI CONNECTED ---");
    Serial.print("IP:"); Serial.println(WiFi.localIP());
    Serial.printf("TCP Port: %d (Black Arm GUI)\n", TCP_PORT);
    Serial.printf("Press ESP2: %s:%d\n", esp2Host.toString().c_str(), ESP2_TCP_PORT);
    Serial.println("--------------------------------");
    tcpServer.begin();
    tcpServer.setNoDelay(true);
  } else {
    Serial.println("[WIFI] FAILED — USB only");
  }
}

// ═════════════════════════════════════════════════════════
//  Lazy servo enable
// ═════════════════════════════════════════════════════════
void ensureServosEnabled() {
  if (servosEnabled) return;
  servosEnabled = true;
  Serial.println("[INIT] Ramping servos to home...");

  int maxSteps = 0;
  for (int i = 0; i < NUM_SERVOS; i++) {
    int diff = abs(HOME_ANGLES[i] - currentAngle[i]);
    if (diff > maxSteps) maxSteps = diff;
  }
  if (maxSteps > 0) {
    for (int step = 1; step <= maxSteps; step++) {
      for (int i = 0; i < NUM_SERVOS; i++) {
        int pos = currentAngle[i] + (int)((long)(HOME_ANGLES[i] - currentAngle[i]) * step / maxSteps);
        writeServo(i, pos);
      }
      delay(HOME_STEP_DELAY);
    }
  } else {
    for (int i = 0; i < NUM_SERVOS; i++) writeServo(i, currentAngle[i]);
  }
  for (int i = 0; i < NUM_SERVOS; i++) currentAngle[i] = HOME_ANGLES[i];
  Serial.println("[INIT] Servos at home");
  replyAll("SERVOS_ATTACHED");
}

// ═════════════════════════════════════════════════════════
//  SMOOTH MOVE
// ═════════════════════════════════════════════════════════
void drainTcpInPauseLoop() {
  while (tcpClient && tcpClient.connected() && tcpClient.available()) {
    char c = (char)tcpClient.read();
    processChar(c, wifiBuf, wifiBufIdx, 1);
  }
}

void smoothMove(int targets[NUM_SERVOS], int stepDelay, bool freezeGripper) {
  ensureServosEnabled();
  if (stepDelay < 1) stepDelay = 1;
  softStopFlag = false;

  int eff[NUM_SERVOS];
  for (int i = 0; i < NUM_SERVOS; i++) eff[i] = targets[i];
  if (freezeGripper) eff[GRIPPER_IDX] = currentAngle[GRIPPER_IDX];

  int maxSteps = 0;
  for (int i = 0; i < NUM_SERVOS; i++) {
    int diff = abs(eff[i] - currentAngle[i]);
    if (diff > maxSteps) maxSteps = diff;
  }
  if (maxSteps == 0) return;

  for (int step = 1; step <= maxSteps; step++) {
    if (estopFlag) {
      if (!estopReported) {
        replyAll("BTN_PAUSED");
        estopReported = true;
      }
      while (estopFlag) {
        drainTcpInPauseLoop();
        if (softStopFlag) { softStopFlag = false; return; }
        delay(20);
      }
      step--;
      continue;
    }

    for (int i = 0; i < NUM_SERVOS; i++) {
      int pos = currentAngle[i] + (int)((long)(eff[i] - currentAngle[i]) * step / maxSteps);
      writeServo(i, pos);
    }
    delay(stepDelay);

    drainTcpInPauseLoop();
    if (softStopFlag) { softStopFlag = false; return; }
  }

  for (int i = 0; i < NUM_SERVOS; i++) currentAngle[i] = eff[i];
}

void smoothMoveSingle(int idx, int target, int stepDelay) {
  if (idx < 0 || idx >= NUM_SERVOS) return;
  ensureServosEnabled();
  if (stepDelay < 1) stepDelay = 1;
  softStopFlag = false;
  int start = currentAngle[idx];
  int steps = abs(target - start);
  if (steps == 0) return;
  int dir = (target > start) ? 1 : -1;
  for (int s = 1; s <= steps; s++) {
    writeServo(idx, start + dir * s);
    delay(stepDelay);
  }
  currentAngle[idx] = target;
}

// ═════════════════════════════════════════════════════════
//  PARSE HELPERS
// ═════════════════════════════════════════════════════════
int parseCSV(const char* str, int* out, int maxCount) {
  int count = 0;
  const char* p = str;
  while (*p && count < maxCount) {
    out[count++] = atoi(p);
    while (*p && *p != ',') p++;
    if (*p == ',') p++;
  }
  return count;
}

// ═════════════════════════════════════════════════════════
//  COMMAND HANDLER
// ═════════════════════════════════════════════════════════
void handleCommand(const char* line, int source) {
  Serial.printf("[CMD] %s\n", line);
  bool& handshakeDone = (source == 0) ? serialHandshake : wifiHandshake;

  if (strcmp(line, "HELLO") == 0) {
    handshakeDone = true;
    replyAll("READY");
    return;
  }
  if (!handshakeDone) return;

  if (strcmp(line, "HOME") == 0) {
    replyAll("MOVING_HOME");
    int targets[NUM_SERVOS];
    for (int i = 0; i < NUM_SERVOS; i++) targets[i] = HOME_ANGLES[i];
    smoothMove(targets, HOME_STEP_DELAY, false);
    replyAll("HOME_DONE");
    return;
  }

  if (strcmp(line, "STOP") == 0) {
    softStopFlag = true;
    replyAll("STOPPED");
    return;
  }

  if (strcmp(line, "RESUME_BTN") == 0) {
    estopFlag     = false;
    estopReported = false;
    softStopFlag  = false;
    Serial.println("[ESTOP] RESUME_BTN received — arm unpaused");
    replyAll("BTN_RESUMED");
    return;
  }

  if (strcmp(line, "IR_STOP") == 0) {
    irStopFlag = true;
    replyAll("IR_STOP_ACK");
    return;
  }
  if (strcmp(line, "IR_RESUME") == 0) {
    irStopFlag = false;
    replyAll("IR_RESUME_ACK");
    return;
  }

  if (strncmp(line, "SET_ESP2_IP:", 12) == 0) {
    esp2Host.fromString(line + 12);
    replyAll("ESP2_IP_SET");
    return;
  }

  if (strcmp(line, "PRESS_CHECK") == 0) {
    WiFiClient c;
    if (c.connect(esp2Host, ESP2_TCP_PORT, ESP2_CONNECT_TIMEOUT_MS)) {
      c.setNoDelay(true);
      c.println("HELLO");
      c.flush();
      char rxBuf[64];
      bool ok = readLineWithTimeout(c, rxBuf, sizeof(rxBuf), 2000);
      c.stop();
      replyAll((ok && strstr(rxBuf, "READY")) ? "PRESS_CHECK_OK" : "PRESS_CHECK_FAIL");
    } else {
      replyAll("PRESS_CHECK_FAIL");
    }
    return;
  }

  if (strcmp(line, "PRESS_SYNC") == 0) {
    if (estopFlag) {
      Serial.println("[PRESS_SYNC] Blocked — e-stop active");
      replyAll("STOPPED");
      return;
    }
    triggerPressMachine();
    return;
  }

  if (strncmp(line, "SPEED:", 6) == 0) {
    g_stepDelay = max(1, atoi(line + 6));
    replyAllInt("SPEED_SET:", g_stepDelay);
    return;
  }

  if (strncmp(line, "SERVO:", 6) == 0) {
    int vals[3];
    int n = parseCSV(line + 6, vals, 3);
    if (n >= 2) {
      int idx   = vals[0];
      int angle = constrain(vals[1], 0, 180);
      int spd   = (n >= 3) ? max(1, vals[2]) : g_stepDelay;
      smoothMoveSingle(idx, angle, spd);
      replyAllInt("SERVO_DONE:", idx);
    }
    return;
  }

  if (strncmp(line, "MOVE:", 5) == 0) {
    int vals[NUM_SERVOS + 1];
    int n = parseCSV(line + 5, vals, NUM_SERVOS + 1);
    if (n >= NUM_SERVOS) {
      int targets[NUM_SERVOS];
      for (int i = 0; i < NUM_SERVOS; i++) targets[i] = constrain(vals[i], 0, 180);
      int spd = (n >= NUM_SERVOS + 1) ? max(1, vals[NUM_SERVOS]) : g_stepDelay;
      smoothMove(targets, spd, false);
      if (!estopFlag && !softStopFlag) {
        replyAll("MOVE_DONE");
      }
    }
    return;
  }

  char errBuf[BUF_SIZE + 16];
  snprintf(errBuf, sizeof(errBuf), "ERR:UNKNOWN:%s", line);
  replyAll(errBuf);
}

// ═════════════════════════════════════════════════════════
//  Byte-by-byte line accumulator
// ═════════════════════════════════════════════════════════
void processChar(char c, char* buf, int& idx, int source) {
  if (c == '\n' || c == '\r') {
    if (idx > 0) {
      buf[idx] = '\0';
      handleCommand(buf, source);
      idx = 0;
    }
  } else {
    if (idx < BUF_SIZE - 1) buf[idx++] = c;
  }
}

// ═════════════════════════════════════════════════════════
//  MAIN LOOP
// ═════════════════════════════════════════════════════════
void loop() {
  if (estopFlag && !estopReported) {
    replyAll("BTN_PAUSED");
    estopReported = true;
  }

  while (Serial.available()) {
    char c = Serial.read();
    processChar(c, serialBuf, serialBufIdx, 0);
    if (c == '\r') continue;
    if (c == '\n') {
      serialMonitorLine.trim();
      if (serialMonitorLine == "TEST_ESP2") triggerPressMachine();
      else if (serialMonitorLine == "STATUS") {
        Serial.printf("[STATUS] IP:%s Port:%d ESP2:%s:%d\n",
          WiFi.localIP().toString().c_str(), TCP_PORT,
          esp2Host.toString().c_str(), ESP2_TCP_PORT);
      }
      serialMonitorLine = "";
    } else {
      serialMonitorLine += c;
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    WiFiClient newClient = tcpServer.accept();
    if (newClient) {
      if (tcpClient && tcpClient.connected()) {
        Serial.println("[WIFI] New client — closing stale connection");
        tcpClient.stop();
      }
      tcpClient     = newClient;
      wifiHandshake = false;
      wifiBufIdx    = 0;
      tcpClient.setNoDelay(true);
      Serial.print("[WIFI] Black GUI client: ");
      Serial.println(tcpClient.remoteIP());
    }

    if (tcpClient && tcpClient.connected()) {
      while (tcpClient.available()) {
        char c = (char)tcpClient.read();
        processChar(c, wifiBuf, wifiBufIdx, 1);
      }
    }
  }
}
