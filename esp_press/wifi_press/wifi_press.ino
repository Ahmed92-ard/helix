/*
 * PRESS CONTROLLER — ESP32 Press Machine Controller (Dual-Arm Edition)
 * Hardware: ESP32 + Servo on GPIO13
 *
 * Role in Dual-Arm system:
 *   • Receives HELLO → READY_PRESS from Black Arm (ESP1-B) on port 9999
 *   • Executes press cycle
 *   • Sends PRESS_DONE back to Black Arm
 *   • After PRESS_DONE: connects to White Arm (ESP1-W) on port 8889
 *     and sends PICK_READY so White Arm knows it can pick from C
 *
 * E-STOP: handled by receiving "STOP" over WiFi from ESP1-B.
 *         No hardware relay pin. No D25 input.
 *
 * Protocol (ESP1-B → ESP2):
 *   HELLO        → READY
 *   READY_PRESS  → PRESS_ACK … PRESS_DONE / PRESS_ERROR:STOPPED
 *   STOP         → stops press immediately
 *   TEST_PRESS   → run one press cycle (serial debug)
 */

#include <WiFi.h>
#include <ESP32Servo.h>
#include "esp_wifi_config.h"

#define TCP_PORT  ESP2_TCP_PORT

// Forward declarations
void send_message(const char* msg);
bool isIrBlocked();
void sendIrStopToEsp1();
void sendIrResumeToEsp1();
void poll_runtime_commands();
bool move_servo_smooth(int from_angle, int to_angle, int step_deg);
bool execute_press();
void handle_command(String cmd);
void notifyWhiteArm();

// ── Press servo ───────────────────────────────────────────
#define PRESS_SERVO_PIN  13
#define SERVO_MIN_US     500
#define SERVO_MAX_US     2400
#define SERVO_FREQ_HZ    50
#define PRESS_SERVO_MIN  60    // raised (home)
#define PRESS_SERVO_MAX  170   // lowered (full press)
#define PRESS_SERVO_HOME 60

Servo pressServo;
int current_press_angle = PRESS_SERVO_HOME;

// ── Timing ────────────────────────────────────────────────
const int PRESS_STEP_DEG = 3;
const int PRESS_DWELL_MS = 225;
const int STEP_DELAY_MS  = 3;

// ── IR Safety Sensors ─────────────────────────────────────
#define IR_SENSOR_PIN_1   32
#define IR_SENSOR_PIN_2   35
#define IR_DETECTED_STATE HIGH  // beam-break: blocked = HIGH = stop
bool ir_pause_active      = false;
bool ir_stop_sent_to_esp1 = false;

// ── State ─────────────────────────────────────────────────
bool emergency_stop_requested = false;
bool press_running            = false;
String serial_line;

// ── WiFi server ───────────────────────────────────────────
WiFiServer server(TCP_PORT);
WiFiClient client;
bool client_connected = false;

// ── Caller IP (Black Arm ESP1 — learned from incoming connection) ─
IPAddress esp1Host(192, 168, 0, 0);
bool esp1_ip_known = false;

// ── White Arm ESP1 target ─────────────────────────────────
IPAddress esp1WhiteHost(ESP1_WHITE_HOST_IP);

// ── WiFi watchdog ─────────────────────────────────────────
unsigned long lastWifiCheck = 0;
bool serverStarted          = false;
#define WIFI_CHECK_INTERVAL_MS 10000

// ═════════════════════════════════════════════════════════
//  Helpers
// ═════════════════════════════════════════════════════════
void send_message(const char* msg) {
    if (client_connected && client.connected()) {
        client.print(msg);
        client.print("\n");
        client.flush();
        Serial.printf("[TX] %s\n", msg);
    }
}

bool isIrBlocked() {
    bool ir1 = digitalRead(IR_SENSOR_PIN_1) == IR_DETECTED_STATE;
    bool ir2 = digitalRead(IR_SENSOR_PIN_2) == IR_DETECTED_STATE;
    if (ir1 || ir2) {
        delay(2);
        ir1 = digitalRead(IR_SENSOR_PIN_1) == IR_DETECTED_STATE;
        ir2 = digitalRead(IR_SENSOR_PIN_2) == IR_DETECTED_STATE;
        if (ir1 || ir2) {
            Serial.printf("[IR] Blocked S1:%d S2:%d\n", ir1, ir2);
            return true;
        }
    }
    return false;
}

void sendIrStopToEsp1() {
    if (!esp1_ip_known) return;
    WiFiClient c;
    if (c.connect(esp1Host, ESP1_BLACK_TCP_PORT, 600)) {
        c.setNoDelay(true);
        c.print("IR_STOP\n");
        c.flush();
        c.stop();
        Serial.println("[IR] IR_STOP sent to Black Arm ESP1");
    }
}

void sendIrResumeToEsp1() {
    if (!esp1_ip_known) return;
    WiFiClient c;
    if (c.connect(esp1Host, ESP1_BLACK_TCP_PORT, 600)) {
        c.setNoDelay(true);
        c.print("IR_RESUME\n");
        c.flush();
        c.stop();
        Serial.println("[IR] IR_RESUME sent to Black Arm ESP1");
    }
}

// ─────────────────────────────────────────────────────────
//  After a successful press: notify White Arm ESP1 (port 8889)
//  Fire-and-forget — short timeout so we don't block long.
// ─────────────────────────────────────────────────────────
void notifyWhiteArm() {
    Serial.printf("[PICK_READY] Notifying White Arm at %s:%d\n",
        esp1WhiteHost.toString().c_str(), ESP1_WHITE_TCP_PORT);
    WiFiClient c;
    if (c.connect(esp1WhiteHost, ESP1_WHITE_TCP_PORT, ESP1W_CONNECT_TIMEOUT_MS)) {
        c.setNoDelay(true);
        c.print("PICK_READY\n");
        c.flush();
        delay(50);  // give White Arm a moment to receive
        c.stop();
        Serial.println("[PICK_READY] Sent to White Arm ESP1");
    } else {
        Serial.println("[PICK_READY] Could not reach White Arm ESP1 — skipping");
    }
}

// Poll for STOP command while press is running
void poll_runtime_commands() {
    if (client_connected && client.connected() && client.available()) {
        String line = client.readStringUntil('\n');
        line.trim();
        if (line == "STOP") {
            Serial.println("[SAFETY] STOP received during press");
            emergency_stop_requested = true;
            press_running = false;
        }
    }
}

// ═════════════════════════════════════════════════════════
//  Smooth servo move with IR pause and STOP check
// ═════════════════════════════════════════════════════════
bool move_servo_smooth(int from_angle, int to_angle, int step_deg) {
    int dir     = (to_angle > from_angle) ? 1 : -1;
    int current = from_angle;

    while (current != to_angle) {
        poll_runtime_commands();

        // IR pause
        if (isIrBlocked()) {
            if (!ir_pause_active) {
                Serial.printf("[IR] Blocked at %d — pausing press\n", current);
                ir_pause_active = true;
                if (!ir_stop_sent_to_esp1) { sendIrStopToEsp1(); ir_stop_sent_to_esp1 = true; }
            }
            current_press_angle = current;
            pressServo.write(current);
            delay(5);
            continue;
        }
        if (ir_pause_active) {
            Serial.println("[IR] Clear — resuming press");
            ir_pause_active = false;
            if (ir_stop_sent_to_esp1) { sendIrResumeToEsp1(); ir_stop_sent_to_esp1 = false; }
        }

        if (emergency_stop_requested) {
            current_press_angle = current;
            return false;
        }

        current += dir * step_deg;
        if (dir > 0 && current > to_angle) current = to_angle;
        if (dir < 0 && current < to_angle) current = to_angle;
        pressServo.write(current);
        delay(STEP_DELAY_MS);
    }

    current_press_angle = to_angle;
    ir_pause_active = false;
    return true;
}

// ═════════════════════════════════════════════════════════
//  Press cycle: down → dwell → up
// ═════════════════════════════════════════════════════════
bool execute_press() {
    Serial.println("[PRESS] Starting press cycle");

    if (!move_servo_smooth(current_press_angle, PRESS_SERVO_MAX, PRESS_STEP_DEG)) {
        Serial.println("[PRESS] Stopped going down");
        return false;
    }

    unsigned long dwellUntil = millis() + PRESS_DWELL_MS;
    while ((long)(millis() - dwellUntil) < 0) {
        poll_runtime_commands();
        if (emergency_stop_requested) { Serial.println("[PRESS] Stopped during dwell"); return false; }
        delay(10);
    }

    if (!move_servo_smooth(current_press_angle, PRESS_SERVO_HOME, PRESS_STEP_DEG)) {
        Serial.println("[PRESS] Stopped going up");
        return false;
    }

    Serial.println("[PRESS] Cycle complete");
    return true;
}

// ═════════════════════════════════════════════════════════
//  Command handler
// ═════════════════════════════════════════════════════════
void handle_command(String cmd) {
    cmd.trim();
    Serial.printf("[RX] %s\n", cmd.c_str());

    if (cmd == "HELLO") {
        send_message("READY");
        // Read the follow-up READY_PRESS inline — avoids loop() cleanup
        // running between HELLO and READY_PRESS and dropping the command.
        client.setTimeout(5000);
        String next = client.readStringUntil('\n');
        next.trim();
        if (next.length() > 0) {
            Serial.printf("[RX-INLINE] %s\n", next.c_str());
            handle_command(next);
        } else {
            Serial.println("[PRESS] Timeout waiting for READY_PRESS after HELLO");
        }
        return;
    }

    if (cmd == "READY_PRESS") {
        if (press_running) { client.print("PRESS_BUSY\n"); client.flush(); return; }
        emergency_stop_requested = false;
        client.print("PRESS_ACK\n"); client.flush();
        Serial.println("[TX] PRESS_ACK");
        bool ok = execute_press();
        client.print(ok ? "PRESS_DONE\n" : "PRESS_ERROR:STOPPED\n");
        client.flush();
        Serial.println(ok ? "[TX] PRESS_DONE" : "[TX] PRESS_ERROR:STOPPED");

        // ── NEW: Notify White Arm so it can pick from C ───────
        if (ok) {
            notifyWhiteArm();
        }
        return;
    }

    if (cmd == "STOP") {
        emergency_stop_requested = true;
        press_running = false;
        Serial.println("[SAFETY] STOP — press halted");
        return;
    }

    if (cmd == "TEST_PRESS") {
        emergency_stop_requested = false;
        bool ok = execute_press();
        send_message(ok ? "PRESS_DONE" : "PRESS_ERROR:STOPPED");
        if (ok) notifyWhiteArm();
        return;
    }

    if (cmd == "STATUS") {
        Serial.printf("[STATUS] IP:%s Port:%d Servo:%d WhiteArm:%s:%d\n",
            WiFi.localIP().toString().c_str(), TCP_PORT, current_press_angle,
            esp1WhiteHost.toString().c_str(), ESP1_WHITE_TCP_PORT);
        return;
    }
}

// ═════════════════════════════════════════════════════════
//  SETUP
// ═════════════════════════════════════════════════════════
void setup() {
    Serial.begin(9600);
    delay(500);
    Serial.println("[SETUP] PRESS CONTROLLER (Dual-Arm) starting");

    // Servo
    pressServo.setPeriodHertz(SERVO_FREQ_HZ);
    pressServo.attach(PRESS_SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
    delay(200);
    pressServo.write(PRESS_SERVO_HOME);
    current_press_angle = PRESS_SERVO_HOME;
    Serial.printf("[SETUP] Press servo on GPIO%d, homed to %d\n", PRESS_SERVO_PIN, PRESS_SERVO_HOME);

    // IR sensors (GPIO35 has no internal pull — use plain INPUT)
    pinMode(IR_SENSOR_PIN_1, INPUT);
    pinMode(IR_SENSOR_PIN_2, INPUT);
    Serial.println("[SETUP] IR sensors on GPIO32, GPIO35");

    // WiFi
    WiFi.mode(WIFI_STA);
    IPAddress local_IP(ESP2_STATIC_IP);
    IPAddress gateway(ESP2_GATEWAY);
    IPAddress subnet(ESP2_SUBNET);
    if (!WiFi.config(local_IP, gateway, subnet)) {
        Serial.println("[WIFI] Static IP configuration failed!");
    }
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.printf("[WIFI] Connecting to %s\n", WIFI_SSID);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500); Serial.print("."); attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\n[WIFI] Connected! IP: %s  Port: %d\n",
            WiFi.localIP().toString().c_str(), TCP_PORT);
        Serial.printf("[WIFI] White Arm target: %s:%d\n",
            esp1WhiteHost.toString().c_str(), ESP1_WHITE_TCP_PORT);
    } else {
        Serial.println("\n[WIFI] Connection failed");
    }

    server.begin();
    serverStarted = true;
    Serial.println("[SETUP] TCP server started");
}

// ═════════════════════════════════════════════════════════
//  LOOP
// ═════════════════════════════════════════════════════════
void loop() {
    // ── WiFi watchdog ─────────────────────────────────────
    if (millis() - lastWifiCheck >= WIFI_CHECK_INTERVAL_MS) {
        lastWifiCheck = millis();
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("[WIFI] Lost — reconnecting...");
            if (client_connected) { client.stop(); client_connected = false; }
            serverStarted = false;
            WiFi.disconnect(); delay(200);
            WiFi.begin(WIFI_SSID, WIFI_PASS);
            unsigned long t0 = millis();
            while (WiFi.status() != WL_CONNECTED && millis() - t0 < 8000) {
                delay(500); Serial.print(".");
            }
            Serial.println(WiFi.status() == WL_CONNECTED ? "\n[WIFI] Reconnected" : "\n[WIFI] Retry next cycle");
        }
        if (WiFi.status() == WL_CONNECTED && !serverStarted) {
            server.begin(); serverStarted = true;
            Serial.println("[SERVER] TCP server restarted");
        }
        Serial.printf("[WIFI] OK | IP:%s | clients:%d\n",
            WiFi.localIP().toString().c_str(), client_connected ? 1 : 0);
    }

    // ── Serial commands ───────────────────────────────────
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\r') continue;
        if (c == '\n') {
            serial_line.trim();
            if (serial_line == "TEST_PRESS") {
                bool ok = execute_press();
                if (ok) notifyWhiteArm();
            }
            else if (serial_line == "STATUS")
                Serial.printf("[STATUS] IP:%s Angle:%d WhiteArm:%s:%d\n",
                    WiFi.localIP().toString().c_str(), current_press_angle,
                    esp1WhiteHost.toString().c_str(), ESP1_WHITE_TCP_PORT);
            serial_line = "";
        } else { serial_line += c; }
    }

    // ── Accept new client ─────────────────────────────────
    WiFiClient newClient = server.accept();
    if (newClient) {
        if (client_connected && client.connected()) {
            Serial.println("[CLIENT] Dropping stale connection");
            client.stop();
        }
        client = newClient;
        client_connected = true;
        esp1Host = client.remoteIP();
        esp1_ip_known = true;
        Serial.printf("[CLIENT] Connected from %s\n", esp1Host.toString().c_str());
    }

    // ── Read from client ──────────────────────────────────
    if (client_connected && client.connected() && client.available()) {
        String line = client.readStringUntil('\n');
        handle_command(line);
    }

    // ── Press if flagged ──────────────────────────────────
    if (press_running) {
        emergency_stop_requested = false;
        bool ok = execute_press();
        send_message(ok ? "PRESS_DONE" : "PRESS_ERROR:STOPPED");
        if (ok) notifyWhiteArm();
        press_running = false;
        emergency_stop_requested = false;
    }

    // ── Client disconnect cleanup ─────────────────────────
    if (client_connected && !client.connected()) {
        client_connected = false;
        client.stop();
        Serial.println("[CLIENT] Disconnected");
        if (emergency_stop_requested) {
            emergency_stop_requested = false;
            Serial.println("[SAFETY] Emergency flag cleared");
        }
        server.begin();
        Serial.println("[SERVER] TCP server restarted");
    }

    // ── Continuous IR monitoring ──────────────────────────
    static unsigned long lastIrCheck = 0;
    if (millis() - lastIrCheck >= 100) {
        lastIrCheck = millis();
        bool blocked = isIrBlocked();
        if (blocked && !ir_stop_sent_to_esp1) {
            sendIrStopToEsp1(); ir_stop_sent_to_esp1 = true;
        } else if (!blocked && ir_stop_sent_to_esp1) {
            sendIrResumeToEsp1(); ir_stop_sent_to_esp1 = false;
        }
    }

    delay(2);
}
