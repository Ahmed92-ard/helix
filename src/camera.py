"""
camera.py — Camera & vision logic

Handles:
  • Laptop webcam via OpenCV
  • Kinect RGB+Depth via freenect
  • Brown dye detection + zone dwell logic
  • Zone rectangle persistence (zones.json)
  • Zone drag / resize via mouse events on the feed canvas

All tunable constants (zone rects, dye colour, dwell frames …) live in config.py.
"""

import json
import os
import threading
import time
from collections import deque
from datetime import datetime

from config import (
    CAM_SOURCE_LAPTOP, CAM_SOURCE_KINECT,
    ZONE_A_RECT, ZONE_B_RECT, ZONE_C_RECT,
    ZONES_FILE,
    MIN_DYE_RADIUS, ZONE_DWELL_FRAMES,
    DYE_LOWER, DYE_UPPER,
    KINECT_TILT_MIN, KINECT_TILT_MAX,
)

try:
    import cv2
    import numpy as np
    from PIL import Image, ImageTk
    CAMERA_AVAILABLE = True
except ImportError:
    CAMERA_AVAILABLE = False

try:
    import freenect
    KINECT_AVAILABLE = True
except ImportError:
    KINECT_AVAILABLE = False


# ─────────────────────────────────────────────────────────
#  CAMERA CONTROLLER
# ─────────────────────────────────────────────────────────
class CameraController:
    """All camera state and vision processing.

    The GUI mixin (CameraMixin, below) wires the tkinter side.
    This class holds only non-GUI state so the vision logic can be
    edited independently.
    """

    def __init__(self):
        self.cap             = None
        self.cam_running     = False
        self.cam_photo       = None
        self.pick_count      = 0
        self.cycle_count     = 0
        self.press_count     = 0
        self.cam_fps         = 0.0
        self.cam_prev_zone   = None
        self.cam_dwell_zone  = None
        self.cam_dwell_count = 0
        self.cam_fps_counter = 0
        self.cam_fps_timer   = time.time()
        self.cam_event_log   = deque(maxlen=30)
        self.dye_detected    = False
        self.session_start   = time.time()
        self.foam_in_press   = False  # Track if foam is currently in press zone (zone C)

        self.kinect_rgb     = None
        self.kinect_depth   = None
        self.kinect_running = False
        self.kinect_thread  = None
        self.kinect_lock    = threading.Lock()

        # Zone rectangles (mutable lists so they update in-place during drag)
        self._load_zones()

        # Zone-drag state
        self._zone_drag_mode  = None
        self._zone_drag_which = None
        self._zone_drag_edge  = None
        self._zone_drag_start = None
        self._zone_drag_orig  = None

    # ── Zone persistence ──────────────────────────────────
    def _load_zones(self):
        try:
            if os.path.exists(ZONES_FILE):
                with open(ZONES_FILE) as f:
                    data = json.load(f)
                self.zone_a_rect = list(data.get("A", list(ZONE_A_RECT)))
                self.zone_b_rect = list(data.get("B", list(ZONE_B_RECT)))
                self.zone_c_rect = list(data.get("C", list(ZONE_C_RECT)))
                return
        except Exception:
            pass
        self.zone_a_rect = list(ZONE_A_RECT)
        self.zone_b_rect = list(ZONE_B_RECT)
        self.zone_c_rect = list(ZONE_C_RECT)

    def _save_zones(self):
        try:
            with open(ZONES_FILE, "w") as f:
                json.dump({"A": self.zone_a_rect, "B": self.zone_b_rect, "C": self.zone_c_rect}, f, indent=2)
        except Exception:
            pass

    # ── Vision ────────────────────────────────────────────
    def process_frame(self, frame, src=None):
        """Detect brown dye objects, track press cycles, annotate frame, return drawn copy."""
        dye_lower = np.array(DYE_LOWER)
        dye_upper = np.array(DYE_UPPER)
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, dye_lower, dye_upper)
        mask = cv2.erode(mask,  None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=3)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        foam_xy = None
        foam_r  = 0
        if contours:
            c = max(contours, key=cv2.contourArea)
            ((cx, cy), radius) = cv2.minEnclosingCircle(c)
            if radius >= MIN_DYE_RADIUS:
                foam_xy = (int(cx), int(cy))
                foam_r  = int(radius)

        za = self.zone_a_rect
        zb = self.zone_b_rect
        zc = self.zone_c_rect

        def in_zone(cx, cy, r):
            x1, y1, x2, y2 = r
            return x1 <= cx <= x2 and y1 <= cy <= y2

        def zone_label(cx, cy):
            if in_zone(cx, cy, za): return "A"
            if in_zone(cx, cy, zc): return "C"
            if in_zone(cx, cy, zb): return "B"
            return None

        cur_zone  = zone_label(*foam_xy) if foam_xy else None
        if cur_zone == self.cam_dwell_zone:
            self.cam_dwell_count += 1
        else:
            self.cam_dwell_zone  = cur_zone
            self.cam_dwell_count = 0

        confirmed = self.cam_dwell_zone if self.cam_dwell_count >= ZONE_DWELL_FRAMES else None

        # ── PRESS CYCLE DETECTION ──────────────────────────
        # When foam is in press zone (C), track it
        foam_now_in_press = (confirmed == "C")
        if foam_now_in_press and not self.foam_in_press:
            # Foam just entered press zone
            now = datetime.now().strftime("%H:%M:%S")
            self.cam_event_log.appendleft(("appear", f"[{now}]  → Foam entering press..."))
            self.foam_in_press = True
        elif not foam_now_in_press and self.foam_in_press:
            # Foam just exited press zone - COUNT AS ONE PRESS DONE
            self.press_count += 1
            now = datetime.now().strftime("%H:%M:%S")
            self.cam_event_log.appendleft(("press", f"[{now}]  ✓ Press Complete #{self.press_count}"))
            self.foam_in_press = False

        # ── ZONE TRANSITIONS (for regular picks/cycles) ──────
        if confirmed and confirmed != self.cam_prev_zone:
            now = datetime.now().strftime("%H:%M:%S")
            if self.cam_prev_zone is None:
                self.cam_event_log.appendleft(("appear", f"[{now}]  Foam in Zone {confirmed}"))
            else:
                self.pick_count += 1
                if self.cam_prev_zone == "A" and confirmed == "B":
                    self.cam_event_log.appendleft(("a2b", f"[{now}]  ↗ A→B  Pick #{self.pick_count}"))
                elif self.cam_prev_zone == "B" and confirmed == "A":
                    self.cycle_count += 1
                    self.cam_event_log.appendleft(("b2a", f"[{now}]  ↙ B→A  Cycle #{self.cycle_count}"))
                elif self.cam_prev_zone == "A" and confirmed == "C":
                    self.cam_event_log.appendleft(("c2a", f"[{now}]  ⇒ A→C  (Pressing...)"))
                elif self.cam_prev_zone == "C" and confirmed == "A":
                    self.cam_event_log.appendleft(("c2a", f"[{now}]  ⇐ C→A"))
                elif self.cam_prev_zone == "B" and confirmed == "C":
                    self.cam_event_log.appendleft(("c2b", f"[{now}]  ⇒ B→C  (Pressing...)"))
                elif self.cam_prev_zone == "C" and confirmed == "B":
                    self.cam_event_log.appendleft(("c2b", f"[{now}]  ⇐ C→B"))
                else:
                    self.cam_event_log.appendleft(("appear", f"[{now}]  ↔ {self.cam_prev_zone}→{confirmed}"))
            self.cam_prev_zone = confirmed

        self.cam_fps_counter += 1
        if time.time() - self.cam_fps_timer >= 1.0:
            self.cam_fps         = self.cam_fps_counter / (time.time() - self.cam_fps_timer)
            self.cam_fps_counter = 0
            self.cam_fps_timer   = time.time()

        self.dye_detected = foam_xy is not None

        # ── Draw ──────────────────────────────────────────
        draw    = frame.copy()
        src_tag = "KINECT" if src == CAM_SOURCE_KINECT else "LAPTOP"
        for rect, lbl, col in [(za, "ZONE A", (0, 220, 60)), (zc, "ZONE C (PRESS)", (255, 165, 0)), (zb, "ZONE B", (0, 140, 255))]:
            x1, y1, x2, y2 = rect
            ov = draw.copy()
            cv2.rectangle(ov, (x1, y1), (x2, y2), col, -1)
            cv2.addWeighted(ov, 0.18, draw, 0.82, 0, draw)
            cv2.rectangle(draw, (x1, y1), (x2, y2), col, 2)
            cv2.putText(draw, lbl, (x1 + 6, y1 + 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, col, 2)
        if foam_xy:
            cv2.circle(draw, foam_xy, foam_r, (200, 200, 200), 3)  # Light gray for foam
            cv2.circle(draw, foam_xy, 5, (255, 255, 255), -1)
            status = "IN PRESS" if self.foam_in_press else (zone_label(*foam_xy) or "MOVING")
            cv2.putText(
                draw,
                f"FOAM [{status}]",
                (foam_xy[0] - foam_r, foam_xy[1] - foam_r - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 2,
            )
        cv2.putText(draw, f"FPS:{self.cam_fps:.1f}  [{src_tag}]",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (160, 160, 160), 1)
        return draw

    # ── Zone drag helpers ─────────────────────────────────
    def _widget_to_cam(self, event, feed_frame):
        """Convert widget-space mouse coords → 640×480 camera coords."""
        lw = feed_frame.winfo_width()
        lh = feed_frame.winfo_height()
        if lw < 10 or lh < 10:
            return None, None
        aspect = 640 / 480
        if lw / lh > aspect:
            disp_w = int(lh * aspect)
            disp_h = lh
        else:
            disp_w = lw
            disp_h = int(lw / aspect)
        ox = (lw - disp_w) // 2
        oy = (lh - disp_h) // 2
        return (
            max(0, min(639, int((event.x - ox) * 640 / max(1, disp_w)))),
            max(0, min(479, int((event.y - oy) * 480 / max(1, disp_h)))),
        )

    def _hit_test_zone(self, cx, cy):
        EDGE_TOL = 15
        for zname, rect in [("A", self.zone_a_rect), ("C", self.zone_c_rect), ("B", self.zone_b_rect)]:
            x1, y1, x2, y2 = rect
            if not ((x1 - EDGE_TOL) <= cx <= (x2 + EDGE_TOL) and
                    (y1 - EDGE_TOL) <= cy <= (y2 + EDGE_TOL)):
                continue
            edges = set()
            if abs(cx - x1) <= EDGE_TOL: edges.add("l")
            if abs(cx - x2) <= EDGE_TOL: edges.add("r")
            if abs(cy - y1) <= EDGE_TOL: edges.add("t")
            if abs(cy - y2) <= EDGE_TOL: edges.add("b")
            if edges:
                return zname, "resize", edges
            if x1 <= cx <= x2 and y1 <= cy <= y2:
                return zname, "move", None
        return None, None, None

    def zone_mouse_down(self, event, feed_frame):
        cx, cy = self._widget_to_cam(event, feed_frame)
        if cx is None:
            return
        zname, mode, edges = self._hit_test_zone(cx, cy)
        if zname is None:
            return
        self._zone_drag_which = zname
        self._zone_drag_mode  = mode
        self._zone_drag_edge  = edges
        self._zone_drag_start = (cx, cy)
        self._zone_drag_orig  = list(
            self.zone_a_rect if zname == "A" else (self.zone_c_rect if zname == "C" else self.zone_b_rect)
        )

    def zone_mouse_drag(self, event, feed_frame):
        if self._zone_drag_which is None:
            return
        cx, cy = self._widget_to_cam(event, feed_frame)
        if cx is None:
            return
        dx   = cx - self._zone_drag_start[0]
        dy   = cy - self._zone_drag_start[1]
        rect = self.zone_a_rect if self._zone_drag_which == "A" else (self.zone_c_rect if self._zone_drag_which == "C" else self.zone_b_rect)
        orig = self._zone_drag_orig
        if self._zone_drag_mode == "move":
            w  = orig[2] - orig[0]
            h  = orig[3] - orig[1]
            nx1 = max(0, min(639 - w, orig[0] + dx))
            ny1 = max(0, min(479 - h, orig[1] + dy))
            rect[0], rect[1], rect[2], rect[3] = nx1, ny1, nx1 + w, ny1 + h
        elif self._zone_drag_mode == "resize":
            e = self._zone_drag_edge
            nx1, ny1, nx2, ny2 = orig
            if "l" in e: nx1 = max(0,      min(nx2 - 20, orig[0] + dx))
            if "r" in e: nx2 = max(nx1 + 20, min(639,    orig[2] + dx))
            if "t" in e: ny1 = max(0,        min(ny2 - 20, orig[1] + dy))
            if "b" in e: ny2 = max(ny1 + 20, min(479,    orig[3] + dy))
            rect[0], rect[1], rect[2], rect[3] = nx1, ny1, nx2, ny2

    def zone_mouse_up(self, event, feed_frame, log_callback=None):
        if self._zone_drag_which is not None:
            self._save_zones()
            if log_callback:
                log_callback("info", f"Zone {self._zone_drag_which} saved")
        self._zone_drag_which = None

    # ── Kinect run-loop ───────────────────────────────────
    def kinect_runloop(self, kinect_tilt_getter,
                       on_start_cb=None, on_stop_cb=None, on_error_cb=None):
        """Blocking freenect loop — run in a daemon thread."""
        try:
            import freenect
        except ImportError as e:
            if on_error_cb:
                on_error_cb(Exception("freenect not installed: pip install freenect"))
            return

        def rgb_cb(dev, data, timestamp):
            try:
                if not self.kinect_running:
                    raise freenect.Kill
                with self.kinect_lock:
                    self.kinect_rgb = cv2.cvtColor(data, cv2.COLOR_RGB2BGR)
            except Exception as e:
                if on_error_cb:
                    on_error_cb(e)

        def depth_cb(dev, data, timestamp):
            try:
                if not self.kinect_running:
                    raise freenect.Kill
                with self.kinect_lock:
                    self.kinect_depth = data.astype(np.uint16)
            except Exception as e:
                if on_error_cb:
                    on_error_cb(e)

        def body_cb(dev, ctx):
            try:
                if not self.kinect_running:
                    raise freenect.Kill
                pending = kinect_tilt_getter()
                if not hasattr(self, "_tilt_last_sent"):
                    self._tilt_last_sent = None
                if pending != self._tilt_last_sent:
                    try:
                        freenect.set_tilt_degs(dev, pending)
                        self._tilt_last_sent = pending
                    except Exception:
                        pass
            except freenect.Kill:
                raise
            except Exception as e:
                if on_error_cb:
                    on_error_cb(e)

        try:
            if on_start_cb:
                on_start_cb()
            freenect.runloop(video=rgb_cb, depth=depth_cb, body=body_cb)
        except freenect.Kill:
            pass
        except Exception as e:
            if on_error_cb:
                on_error_cb(e)
        finally:
            self.kinect_running = False
            if on_stop_cb:
                on_stop_cb()
