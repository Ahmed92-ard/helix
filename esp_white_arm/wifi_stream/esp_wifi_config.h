#ifndef ESP_WIFI_CONFIG_H
#define ESP_WIFI_CONFIG_H

// ═══════════════════════════════════════════════════════════
//  WHITE ARM (ESP1-W) WiFi Configuration
//  Role: Arm controller for White Arm — listens on port 8889
//        Receives PICK_READY from Press ESP2 after press done
// ═══════════════════════════════════════════════════════════

#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASS "YOUR_WIFI_PASSWORD"

// This ESP1-W server port (Python White GUI → ESP1-W)
// Also receives PICK_READY from Press ESP2 on this port
#define ESP1_TCP_PORT  8889

// Press ESP2 server port (for optional direct commands)
#define ESP2_TCP_PORT  9999

// White Arm ESP1 fixed IP address
#define ESP1_WHITE_STATIC_IP  192, 168, 0, 129
#define ESP1_WHITE_GATEWAY    192, 168, 0, 1
#define ESP1_WHITE_SUBNET     255, 255, 255, 0

// Press ESP2 fixed IP address
#define ESP2_HOST_IP   192, 168, 0, 130

// Timeout tuning
#define ESP2_CONNECT_TIMEOUT_MS   6000
#define ESP2_RESPONSE_TIMEOUT_MS  20000

#endif
