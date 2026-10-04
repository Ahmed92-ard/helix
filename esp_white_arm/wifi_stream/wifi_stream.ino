/*
 * WHITE ARM CONTROLLER — ESP32 + PCA9685 (Dual-Arm Edition)
 * Hardware: ESP32 + PCA9685 (I2C: SDA=GPIO21, SCL=GPIO22)
 *
 * Role in Dual-Arm system:
 *   • Receives commands from Python White GUI on port 8889
 *   • Receives PICK_READY from Press ESP2 (port 9999) on port 8889
 *     after the press cycle completes — signals White Arm to pick from C
 *   • Executes C→B motion: picks from Press (C), places at B
 *
 * KEY DIFFERENCE from Black Arm:
 *   • TCP port is 8889 (not 8888)
 *   • Handles PICK_READY command from Press ESP2 (not from Python GUI)
 *     When PICK_READY arrives, it is stored as a pending flag.
 *     When Python sends WAIT_PICK_READY, ESP1-W responds with PICK_READY_FROM_PRESS
 *     as soon as the flag is set (or immediately if already set).
 *
 * E-STOP BUTTON: GPIO18 → GND (momentary push button, INPUT_PULLUP)
 *
 * Commands (after HELLO handshake):
 *   HELLO                          → READY
 *   MOVE:a0,a1,a2,a3,a4,a5,speed  → smooth move all 6 servos → MOVE_DONE
 *   SERVO:idx,angle,speed          → smooth move single servo  → SERVO_DONE:idx
 *   HOME                           → move to home angles       → HOME_DONE
 *   STOP                           → stop current move         → STOPPED
 *   SPEED:val                      → set step delay
 *   RESUME_BTN                     → clear estop flag          → BTN_RESUMED
 *   WAIT_PICK_READY                → wait until PICK_READY received from press
 *                                   → PICK_READY_FROM_PRESS (when press signals)
 *
 * From Press ESP2 (direct connection, no prior HELLO):
 *   PICK_READY                     → stored as pending flag; no reply needed
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFi.h>
#include "esp_wifi_config.h"

#define TCP_PORT  ESP1_TCP_PORT   // 8889 for White Arm

// Forward declarations
void handleCommand(const char* line, int source);
void processChar(char c, char* buf, int& idx, int source);
void drainTcpInPauseLoop();
void smoothMove(int targets[6], int stepDelay, bool freezeGripper);
void smoothMoveSingle(int idx, int target, int stepDelay);

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
#define GRIPPER_OPEN  60
#define GRIPPER_CLOSE 10

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

// ── IR stop flag ──────────────────────────────────────────
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
// Listens on port 8889 — both White GUI and Press ESP2 connect here
WiFiServer tcpServer(TCP_PORT);
WiFiClient tcpClient;        // connected Python White GUI client

// ── PICK_READY state ──────────────────────────────────────
// Set to true when Press ESP2 sends PICK_READY after press cycle.
// Cleared to false after WAIT_PICK_READY is satisfied.
volatile bool pickReadyFlag   = false;
bool waitingForPickReady      = false;   // true while Python is blocked on WAIT_PICK_READY

IPAddress esp2Host(ESP2_HOST_IP);
String serialMonitorLine;

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
#if defined(ESP1_WHITE_STATIC_IP)
  IPAddress local_IP(ESP1_WHITE_STATIC_IP);
  IPAddress gateway(ESP1_WHITE_GATEWAY);
  IPAddress subnet(ESP1_WHITE_SUBNET);
  if (!WiFi.config(local_IP, gateway, subnet)) {
    Serial.println("[WIFI] Static IP configuration failed!");
  }
#endif
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
    Serial.println("\n--- WHITE ARM WIFI CONNECTED ---");
    Serial.print("IP:"); Serial.println(WiFi.localIP());
    Serial.printf("TCP Port: %d (White GUI + Press PICK_READY)\n", TCP_PORT);
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
//
//  source=0 → Serial, source=1 → WiFi (could be GUI or Press ESP2)
//
//  Special: PICK_READY arrives from Press ESP2 on same port 8889.
//  Press ESP2 does NOT send HELLO first — it just connects and sends
//  PICK_READY then disconnects. We handle it before the handshake check.
// ═════════════════════════════════════════════════════════
void handleCommand(const char* line, int source) {
  Serial.printf("[CMD] %s\n", line);

  // ── PICK_READY — from Press ESP2 (no handshake needed) ─
  // This arrives on source=1 (WiFi) but from Press ESP2, not the GUI.
  // The GUI TCP client may be replaced temporarily but we restore it.
  if (strcmp(line, "PICK_READY") == 0) {
    pickReadyFlag = true;
    Serial.println("[PICK_READY] Received from Press ESP2 — White Arm may now pick from C");
    // No reply needed — Press ESP2 disconnects after sending
    return;
  }

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

  // ── WAIT_PICK_READY — Python White GUI waits for press signal ─
  // Python sends this when the motion loop reaches the wait_for_press phase.
  // We reply PICK_READY_FROM_PRESS immediately if already received,
  // or block (polling) until pickReadyFlag is set.
  //
  // IMPORTANT: this handler blocks inside handleCommand(), so the main loop()
  // never runs and tcpServer.accept() is never called.  We therefore call
  // accept() ourselves inside the wait loop so ESP2 can deliver PICK_READY
  // even while we are blocked here.
  if (strcmp(line, "WAIT_PICK_READY") == 0) {
    Serial.println("[WAIT] Python waiting for PICK_READY from Press ESP2...");
    waitingForPickReady = true;

    unsigned long startWait = millis();
    const unsigned long WAIT_TIMEOUT_MS = 60000UL;  // 60 second max wait

    while (!pickReadyFlag) {
      // ── Abort conditions ──────────────────────────────
      if (softStopFlag || estopFlag) {
        waitingForPickReady = false;
        if (estopFlag && !estopReported) {
          replyAll("BTN_PAUSED");
          estopReported = true;
        }
        replyAll("STOPPED");
        return;
      }

      // ── Accept new TCP connections while blocked ──────
      // This is the critical addition: without it, ESP2's PICK_READY
      // connection sits in the TCP backlog and is never serviced.
      if (WiFi.status() == WL_CONNECTED) {
        WiFiClient newClient = tcpServer.accept();
        if (newClient) {
          newClient.setNoDelay(true);
          if (!tcpClient || !tcpClient.connected()) {
            // No active GUI client — treat this as the primary connection
            tcpClient     = newClient;
            wifiHandshake = false;
            wifiBufIdx    = 0;
            Serial.print("[WAIT] New primary client: ");
            Serial.println(tcpClient.remoteIP());
          } else {
            // GUI already connected — read one message inline (ESP2 PICK_READY)
            Serial.print("[WAIT] Secondary client (Press ESP2?): ");
            Serial.println(newClient.remoteIP());
            char inlineBuf[BUF_SIZE]; int inlineIdx = 0;
            unsigned long t0 = millis();
            while (millis() - t0 < 1000 && newClient.connected()) {
              while (newClient.available()) {
                char c = (char)newClient.read();
                if (c == '\r') continue;
                if (c == '\n') {
                  inlineBuf[inlineIdx] = '\0';
                  if (inlineIdx > 0) handleCommand(inlineBuf, 1);
                  inlineIdx = 0;
                } else if (inlineIdx < BUF_SIZE - 1) {
                  inlineBuf[inlineIdx++] = c;
                }
              }
              delay(5);
            }
            newClient.stop();
          }
        }
      }

      // ── Drain existing client bytes (STOP, RESUME_BTN) ─
      drainTcpInPauseLoop();

      if (millis() - startWait > WAIT_TIMEOUT_MS) {
        waitingForPickReady = false;
        replyAll("PRESS_ERROR:PICK_TIMEOUT");
        Serial.println("[WAIT] Timed out waiting for PICK_READY");
        return;
      }
      delay(20);
    }

    // Got the signal — clear flag and notify Python
    pickReadyFlag = false;
    waitingForPickReady = false;
    Serial.println("[WAIT] PICK_READY received — replying PICK_READY_FROM_PRESS");
    replyAll("PICK_READY_FROM_PRESS");
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
//
//  The TCP server on port 8889 accepts two kinds of clients:
//    1. Python White GUI — persistent connection, full handshake
//    2. Press ESP2       — short connection, sends PICK_READY only
//
//  We detect Press ESP2 by the fact that it does NOT send HELLO.
//  handleCommand() handles PICK_READY before the handshake check,
//  so it works correctly regardless of which client sent it.
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
      if (serialMonitorLine == "STATUS") {
        Serial.printf("[STATUS] IP:%s Port:%d PickReady:%d\n",
          WiFi.localIP().toString().c_str(), TCP_PORT, (int)pickReadyFlag);
      }
      serialMonitorLine = "";
    } else {
      serialMonitorLine += c;
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    WiFiClient newClient = tcpServer.accept();
    if (newClient) {
      // Check: is this a persistent GUI client or a quick Press ESP2 connection?
      // We accept all clients. If the current tcpClient (GUI) is active, we
      // only replace it if the new client sends HELLO. For PICK_READY from Press
      // ESP2, we handle it via processChar then it disconnects on its own.
      //
      // Strategy: if no GUI client is connected, store it as tcpClient.
      // If a GUI client IS connected, handle the new client inline for one message.
      if (!tcpClient || !tcpClient.connected()) {
        // No GUI — accept as primary client (could be GUI or Press)
        tcpClient     = newClient;
        wifiHandshake = false;
        wifiBufIdx    = 0;
        tcpClient.setNoDelay(true);
        Serial.print("[WIFI] Client connected: ");
        Serial.println(tcpClient.remoteIP());
      } else {
        // GUI already connected — this is likely Press ESP2 sending PICK_READY
        // Read one message from it inline, then let it go
        Serial.print("[WIFI] Secondary client (likely Press ESP2): ");
        Serial.println(newClient.remoteIP());
        newClient.setNoDelay(true);
        unsigned long t0 = millis();
        char inlineBuf[BUF_SIZE]; int inlineIdx = 0;
        while (millis() - t0 < 1000 && newClient.connected()) {
          while (newClient.available()) {
            char c = (char)newClient.read();
            if (c == '\r') continue;
            if (c == '\n') {
              inlineBuf[inlineIdx] = '\0';
              if (inlineIdx > 0) handleCommand(inlineBuf, 1);
              inlineIdx = 0;
            } else if (inlineIdx < BUF_SIZE - 1) {
              inlineBuf[inlineIdx++] = c;
            }
          }
          delay(5);
        }
        newClient.stop();
      }
    }

    if (tcpClient && tcpClient.connected()) {
      while (tcpClient.available()) {
        char c = (char)tcpClient.read();
        processChar(c, wifiBuf, wifiBufIdx, 1);
      }
    }
  }
}
