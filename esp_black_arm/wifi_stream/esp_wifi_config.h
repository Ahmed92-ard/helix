#ifndef ESP_WIFI_CONFIG_H
#define ESP_WIFI_CONFIG_H

// ═══════════════════════════════════════════════════════════
//  BLACK ARM (ESP1-B) WiFi Configuration
//  Role: Arm controller for Black Arm — listens on port 8888
//        Connects to Press ESP2 on port 9999 for PRESS_SYNC
// ═══════════════════════════════════════════════════════════

#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASS "YOUR_WIFI_PASSWORD"

// This ESP1-B server port (Python Black GUI → ESP1-B)
#define ESP1_TCP_PORT  8888

// Press ESP2 server port (ESP1-B → ESP2 for press cycle)
#define ESP2_TCP_PORT  9999

// Press ESP2 fixed IP address
#define ESP2_HOST_IP   192, 168, 0, 130

// Timeout tuning for ESP1-B → ESP2 press handshake
#define ESP2_CONNECT_TIMEOUT_MS   6000
#define ESP2_RESPONSE_TIMEOUT_MS  20000

#endif
