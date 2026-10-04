#ifndef ESP_WIFI_CONFIG_H
#define ESP_WIFI_CONFIG_H

// ═══════════════════════════════════════════════════════════
//  DUAL-ARM WiFi Configuration — shared across all 3 ESPs
//  Update SSID/PASS once, flash all boards.
//
//  Device roles:
//    ESP1-B (Black Arm controller)  → port 8888
//    ESP1-W (White Arm controller)  → port 8889
//    ESP2   (Press machine)         → port 9999
// ═══════════════════════════════════════════════════════════

#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASS "YOUR_WIFI_PASSWORD"

// ── TCP ports ───────────────────────────────────────────────
// Black Arm ESP1 server (Python Black GUI → ESP1-B)
#define ESP1_BLACK_TCP_PORT  8888

// White Arm ESP1 server (Python White GUI → ESP1-W)
#define ESP1_WHITE_TCP_PORT  8889

// Press ESP2 server (ESP1-B → ESP2 for press cycle)
#define ESP2_TCP_PORT        9999

// Legacy alias — used by wifi_press.ino to refer to the arm that called it
#define ESP1_TCP_PORT        ESP1_BLACK_TCP_PORT

// ── Press ESP2 static IP ─────────────────────────────────────
// Give ESP2 a fixed address so both arms always know where the press is.
#define ESP2_STATIC_IP  192, 168, 0, 130
#define ESP2_GATEWAY    192, 168, 0, 1
#define ESP2_SUBNET     255, 255, 255, 0
#define ESP2_HOST_IP    192, 168, 0, 130

// ── White Arm ESP1 IP ────────────────────────────────────────
// Press ESP2 connects here after PRESS_DONE to send PICK_READY.
// UPDATE this to the actual DHCP/static IP of White Arm ESP1.
#define ESP1_WHITE_HOST_IP   192, 168, 0, 129

// ── Timeout tuning ──────────────────────────────────────────
#define ESP2_CONNECT_TIMEOUT_MS   6000   // connect to press
#define ESP2_RESPONSE_TIMEOUT_MS  20000  // wait for PRESS_DONE

// Timeout for Press → White Arm PICK_READY notification
#define ESP1W_CONNECT_TIMEOUT_MS  3000

#endif
