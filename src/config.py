"""
config.py — DUAL-ARM configuration (Unified Edition)

Black Arm: A → C  (ESP1-B on port 8888)
White Arm: C → B  (ESP1-W on port 8889)
Press:     C      (ESP2   on port 9999)
"""

import os

# ─────────────────────────────────────────────────────────
#  THEME / COLOURS
# ─────────────────────────────────────────────────────────
BG       = "#0d0f0e"
PANEL    = "#141716"
PANEL2   = "#0f1210"
BORDER   = "#2a2e2b"
ACCENT   = "#00ff88"
ACCENT2  = "#ff6b35"
ACCENT3  = "#33aaff"
WARN     = "#ffcc00"
DANGER   = "#ff3333"
TEXT     = "#c8d4cc"
DIM      = "#5a6b60"
ENTRY_BG = "#0a0c0b"

# Arm-specific accent colors
BLACK_ARM_COLOR = "#33aaff"   # blue tint for Black Arm
WHITE_ARM_COLOR = "#c8c8ff"   # light purple tint for White Arm

# ─────────────────────────────────────────────────────────
#  SERVO CONFIGURATION — Black Arm
# ─────────────────────────────────────────────────────────
BLACK_SERVO_NAMES = ["BASE", "SHOULDER", "ELBOW", "WRIST ROT", "WRIST PITCH", "GRIPPER"]
BLACK_SERVO_PINS  = [0, 1, 2, 3, 4, 5]
BLACK_SERVO_MIN   = [0, 0, 0, 0, 0, 10]
BLACK_SERVO_MAX   = [180, 180, 180, 180, 180, 40]
BLACK_HOME_POS    = [90, 90, 143, 90, 90, 40]
BLACK_GRIPPER_OPEN   = 40
BLACK_GRIPPER_CLOSED = 10

# ─────────────────────────────────────────────────────────
#  SERVO CONFIGURATION — White Arm
# ─────────────────────────────────────────────────────────
WHITE_SERVO_NAMES = ["BASE", "SHOULDER", "ELBOW", "WRIST ROT", "WRIST PITCH", "GRIPPER"]
WHITE_SERVO_PINS  = [0, 1, 2, 3, 4, 5]
WHITE_SERVO_MIN   = [0, 0, 0, 0, 0, 10]
WHITE_SERVO_MAX   = [180, 180, 180, 180, 180, 180]
WHITE_HOME_POS    = [90, 90, 143, 90, 90, 60]
WHITE_GRIPPER_OPEN   = 60
WHITE_GRIPPER_CLOSED = 10

# Legacy aliases used by servos.py / motion_loop.py (default to Black Arm)
SERVO_NAMES = BLACK_SERVO_NAMES
SERVO_PINS  = BLACK_SERVO_PINS
SERVO_MIN   = BLACK_SERVO_MIN
SERVO_MAX   = BLACK_SERVO_MAX
HOME_POS    = BLACK_HOME_POS
GRIPPER_OPEN   = BLACK_GRIPPER_OPEN
GRIPPER_CLOSED = BLACK_GRIPPER_CLOSED

# ─────────────────────────────────────────────────────────
#  STATION FILES
# ─────────────────────────────────────────────────────────
_BASE_DIR = os.path.dirname(__file__)

BLACK_STATIONS_FILE = os.path.join(_BASE_DIR, "stations_black.json")
WHITE_STATIONS_FILE = os.path.join(_BASE_DIR, "stations_white.json")
BLACK_ZONES_FILE    = os.path.join(_BASE_DIR, "zones_black.json")
WHITE_ZONES_FILE    = os.path.join(_BASE_DIR, "zones_white.json")

# Legacy alias
STATIONS_FILE = BLACK_STATIONS_FILE
ZONES_FILE    = BLACK_ZONES_FILE
# ─────────────────────────────────────────────────────────
#  DEFAULT STATION POSITIONS
# ─────────────────────────────────────────────────────────
BLACK_DEFAULT_STATIONS = {
    "A": {
        "above": [180, 124, 18, 65, 8, 40],
        "pick":  [180, 175, 128, 65, 68, 10],
    },
    "C": {
        "above": [90, 124, 18, 65, 8, 40],
        "pick":  [90, 175, 128, 65, 73, 10],
    },
}

WHITE_DEFAULT_STATIONS = {
    "C": {
        "above": [90, 105, 5, 81, 13, 60],
        "pick":  [90, 136, 83, 83, 70, 10],
    },
    "B": {
        "above": [180, 105, 5, 81, 13, 60],
        "pick":  [180, 136, 83, 83, 70, 10],
    },
}

DEFAULT_STATIONS = BLACK_DEFAULT_STATIONS

# ─────────────────────────────────────────────────────────
#  SEQUENCE TUNING
# ─────────────────────────────────────────────────────────
ACB_RETRACT_DELTA    = 0
ACB_RETRACT_DWELL_MS = 1000
CB_C_PICK_SETTLE_MS  = 700

# ─────────────────────────────────────────────────────────
#  CAMERA / VISION
# ─────────────────────────────────────────────────────────
CAM_SOURCE_LAPTOP = "laptop"
CAM_SOURCE_KINECT = "kinect"
CAM_INDEX         = 0
ZONE_A_RECT       = (420, 350, 640, 480)
ZONE_C_RECT       = (300, 250, 410, 380)
ZONE_B_RECT       = (150, 150, 380, 320)
MIN_DYE_RADIUS    = 12
ZONE_DWELL_FRAMES = 10
DYE_LOWER = (5, 50, 50)
DYE_UPPER = (20, 255, 200)
KINECT_TILT_MIN = -27
KINECT_TILT_MAX =  27

# ─────────────────────────────────────────────────────────
#  BLACK ARM CONNECTION (ESP1-B, port 8888)
# ─────────────────────────────────────────────────────────
BLACK_WIFI_IP   = "192.168.0.128"
BLACK_WIFI_PORT = 8888

# ─────────────────────────────────────────────────────────
#  WHITE ARM CONNECTION (ESP1-W, port 8889)
# ─────────────────────────────────────────────────────────
WHITE_WIFI_IP   = "192.168.0.129"
WHITE_WIFI_PORT = 8889

# ─────────────────────────────────────────────────────────
#  PRESS MACHINE (ESP2, port 9999)
# ─────────────────────────────────────────────────────────
ESP2_IP   = "192.168.0.130"
ESP2_PORT = 9999

# ─────────────────────────────────────────────────────────
#  SHARED CONNECTION SETTINGS
# ─────────────────────────────────────────────────────────
DEFAULT_BAUD         = "9600"
AVAILABLE_BAUDS      = ["9600", "115200"]
WIFI_SOCKET_TIMEOUT  = 90.0
WIFI_CONNECT_TIMEOUT = 5
HELLO_RETRY_MS       = 1500
HELLO_FIRST_DELAY_MS = 100

# Legacy aliases (used internally by shared modules)
DEFAULT_WIFI_IP   = BLACK_WIFI_IP
DEFAULT_WIFI_PORT = BLACK_WIFI_PORT

# ─────────────────────────────────────────────────────────
#  PRESS SYNC TIMING
# ─────────────────────────────────────────────────────────
ESP2_CONNECT_TIMEOUT = 5
ESP2_PRESS_TIMEOUT   = 10
ESP2_ENABLED         = True
ESP2_AUTO_RETRY      = True
PRESS_SYNC_TIMEOUT_S = 60.0
