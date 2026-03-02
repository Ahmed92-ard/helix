import cv2
import numpy as np
import tkinter as tk
from tkinter import font as tkfont
from PIL import Image, ImageTk   # pip install pillow
import time
from datetime import datetime
from collections import deque

# ─────────────────────────────────────────────
#  CONFIGURATION
# ─────────────────────────────────────────────
CAMERA_INDEX       = 0          # change to 1 if wrong camera

# Zone A (left) — slightly lowered
ZONE_A_RECT        = (420, 350, 640, 480)   # (x1, y1, x2, y2) — green mat bottom-left

# Zone B (center — where hand holds the ball)
ZONE_B_RECT        = (150, 150, 380, 320)  # (x1, y1, x2, y2)

# HSV range for ORANGE ball
ORANGE_HSV_LOWER   = np.array([5,  120, 120])
ORANGE_HSV_UPPER   = np.array([25, 255, 255])

MIN_BALL_RADIUS    = 12
ZONE_DWELL_FRAMES  = 10

# ─────────────────────────────────────────────
#  STATE
# ─────────────────────────────────────────────
state = {
    "pick_count":    0,
    "cycle_count":   0,
    "ball_detected": False,
    "fps":           0.0,
    "session_start": time.time(),
    "event_log":     deque(maxlen=20),
    "running":       True,
}

# ─────────────────────────────────────────────
#  ZONE HELPERS
# ─────────────────────────────────────────────
def in_zone(cx, cy, rect):
    x1, y1, x2, y2 = rect
    return x1 <= cx <= x2 and y1 <= cy <= y2

def zone_label(cx, cy):
    if in_zone(cx, cy, ZONE_A_RECT): return "A"
    if in_zone(cx, cy, ZONE_B_RECT): return "B"
    return None

# ─────────────────────────────────────────────
#  CAMERA INIT
# ─────────────────────────────────────────────
def init_camera():
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    for _ in range(15):
        cap.read()
    print(f"[camera] opened={cap.isOpened()}  "
          f"w={cap.get(cv2.CAP_PROP_FRAME_WIDTH):.0f}  "
          f"h={cap.get(cv2.CAP_PROP_FRAME_HEIGHT):.0f}")
    return cap

# ─────────────────────────────────────────────
#  THEME
# ─────────────────────────────────────────────
BG      = "#050d1a"
ACCENT  = "#00ffe7"
ACCENT2 = "#ff6b00"
DIM     = "#0d1e30"
TEXT    = "#d0eeff"

# ─────────────────────────────────────────────
#  APP
# ─────────────────────────────────────────────
class App:
    def __init__(self, root, cap):
        self.root = root
        self.cap  = cap

        self.prev_zone   = None
        self.dwell_zone  = None
        self.dwell_count = 0
        self.fps_counter = 0
        self.fps_timer   = time.time()
        self._last_log_len = 0
        self._photo = None   # keep reference to avoid GC

        root.title("ROBOTIC ARM – PICK & PLACE TRACKER")
        root.configure(bg=BG)
        root.resizable(False, False)
        root.protocol("WM_DELETE_WINDOW", self.on_close)

        self._build_ui()
        self.update_camera()
        self.update_gui()

    # ─────────────────────────────────────────
    def _build_ui(self):
        title_f  = tkfont.Font(family="Courier", size=13, weight="bold")
        mono_f   = tkfont.Font(family="Courier", size=11)
        big_f    = tkfont.Font(family="Courier", size=34, weight="bold")
        small_f  = tkfont.Font(family="Courier", size=9)

        # ── title ──
        tk.Label(self.root, text="◈  PICK & PLACE INTELLIGENCE SYSTEM  ◈",
                 bg=BG, fg=ACCENT, font=title_f).pack(pady=(14, 2))
        tk.Frame(self.root, bg=ACCENT, height=1).pack(fill="x", padx=20)

        # ── camera feed (embedded in Tkinter) ──
        cam_outer = tk.Frame(self.root, bg=ACCENT, bd=0,
                             highlightthickness=2, highlightbackground=ACCENT)
        cam_outer.pack(padx=20, pady=10)
        self.cam_label = tk.Label(cam_outer, bg="black")
        self.cam_label.pack()

        # ── big counters ──
        cnt = tk.Frame(self.root, bg=BG)
        cnt.pack(pady=8, padx=20, fill="x")
        for col, (label, attr, color) in enumerate([
            ("TOTAL PICKS",  "picks_var",  ACCENT),
            ("FULL CYCLES",  "cycles_var", ACCENT2),
        ]):
            f = tk.Frame(cnt, bg=DIM, highlightthickness=1, highlightbackground=color)
            f.grid(row=0, column=col, padx=10, sticky="nsew", ipadx=20, ipady=8)
            cnt.columnconfigure(col, weight=1)
            tk.Label(f, text=label, bg=DIM, fg=color,
                     font=tkfont.Font(family="Courier", size=10, weight="bold")).pack(pady=(6, 0))
            var = tk.StringVar(value="0")
            tk.Label(f, textvariable=var, bg=DIM, fg=color, font=big_f).pack()
            setattr(self, attr, var)

        # ── status row ──
        sf = tk.Frame(self.root, bg=BG)
        sf.pack(padx=20, fill="x", pady=4)
        for col, (label, attr, color) in enumerate([
            ("SESSION TIME", "time_var", TEXT),
            ("BALL STATUS",  "ball_var", ACCENT),
            ("FPS",          "fps_var",  "#4a7a9b"),
        ]):
            box = tk.Frame(sf, bg=DIM, highlightthickness=1, highlightbackground="#1e3a5a")
            box.grid(row=0, column=col, padx=6, sticky="nsew", ipadx=10, ipady=5)
            sf.columnconfigure(col, weight=1)
            tk.Label(box, text=label, bg=DIM, fg="#3a6a8a", font=small_f).pack()
            var = tk.StringVar(value="--")
            tk.Label(box, textvariable=var, bg=DIM, fg=color, font=mono_f).pack()
            setattr(self, attr, var)

        tk.Frame(self.root, bg=ACCENT, height=1).pack(fill="x", padx=20, pady=10)

        # ── event log ──
        tk.Label(self.root, text="▸  EVENT LOG", bg=BG, fg=ACCENT,
                 font=tkfont.Font(family="Courier", size=10, weight="bold")).pack(anchor="w", padx=24)

        log_frame = tk.Frame(self.root, bg=DIM,
                             highlightthickness=1, highlightbackground="#1e3a5a")
        log_frame.pack(padx=20, pady=4, fill="x")

        self.log_text = tk.Text(
            log_frame, bg=DIM, fg=TEXT,
            font=tkfont.Font(family="Courier", size=9),
            state="disabled", bd=0, relief="flat",
            height=7, wrap="word", cursor="arrow")
        self.log_text.pack(fill="x", padx=8, pady=6)
        self.log_text.tag_config("a2b",    foreground=ACCENT2)
        self.log_text.tag_config("b2a",    foreground=ACCENT)
        self.log_text.tag_config("appear", foreground="#4a7a9b")

        tk.Frame(self.root, bg=ACCENT, height=1).pack(fill="x", padx=20, pady=(6, 0))
        tk.Label(self.root, text="Close this window to quit",
                 bg=BG, fg="#2a4a6a", font=small_f).pack(pady=5)

    # ─────────────────────────────────────────
    def update_camera(self):
        if not state["running"]:
            return

        ret, frame = self.cap.read()
        if ret and frame is not None:
            annotated = self._process(frame)

            # Convert BGR → RGB → PIL → ImageTk and push to label
            rgb   = cv2.cvtColor(annotated, cv2.COLOR_BGR2RGB)
            img   = Image.fromarray(rgb)
            photo = ImageTk.PhotoImage(image=img)
            self.cam_label.configure(image=photo)
            self._photo = photo   # prevent garbage collection

        self.root.after(16, self.update_camera)

    # ─────────────────────────────────────────
    def _process(self, frame):
        # orange ball detection
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, ORANGE_HSV_LOWER, ORANGE_HSV_UPPER)
        mask = cv2.erode(mask,  None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=3)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        ball_xy, ball_r = None, 0
        if contours:
            c = max(contours, key=cv2.contourArea)
            ((cx, cy), radius) = cv2.minEnclosingCircle(c)
            if radius >= MIN_BALL_RADIUS:
                ball_xy = (int(cx), int(cy))
                ball_r  = int(radius)

        # zone dwell
        cur_zone = zone_label(*ball_xy) if ball_xy else None
        if cur_zone == self.dwell_zone:
            self.dwell_count += 1
        else:
            self.dwell_zone  = cur_zone
            self.dwell_count = 0

        confirmed = self.dwell_zone if self.dwell_count >= ZONE_DWELL_FRAMES else None

        if confirmed and confirmed != self.prev_zone:
            now = datetime.now().strftime("%H:%M:%S")
            if self.prev_zone is None:
                # First appearance — just note it, don't count
                state["event_log"].appendleft(
                    f"[{now}]  Ball appeared in Zone {confirmed}")
            else:
                # Any zone-to-zone move = a pick
                state["pick_count"] += 1
                if self.prev_zone == "A" and confirmed == "B":
                    state["event_log"].appendleft(
                        f"[{now}]  ↗  A → B   (Pick #{state['pick_count']})")
                elif self.prev_zone == "B" and confirmed == "A":
                    state["cycle_count"] += 1
                    state["event_log"].appendleft(
                        f"[{now}]  ↙  B → A   [Cycle #{state['cycle_count']}]")
                else:
                    state["event_log"].appendleft(
                        f"[{now}]  ↔  {self.prev_zone} → {confirmed}   (Pick #{state['pick_count']})")
            self.prev_zone = confirmed

        # fps
        self.fps_counter += 1
        if time.time() - self.fps_timer >= 1.0:
            state["fps"]     = self.fps_counter / (time.time() - self.fps_timer)
            self.fps_counter = 0
            self.fps_timer   = time.time()

        state["ball_detected"] = ball_xy is not None

        # draw
        draw = frame.copy()
        for rect, lbl, col in [
            (ZONE_A_RECT, "ZONE A", (0, 220, 60)),
            (ZONE_B_RECT, "ZONE B", (0, 140, 255)),
        ]:
            x1, y1, x2, y2 = rect
            ov = draw.copy()
            cv2.rectangle(ov, (x1, y1), (x2, y2), col, -1)
            cv2.addWeighted(ov, 0.18, draw, 0.82, 0, draw)
            cv2.rectangle(draw, (x1, y1), (x2, y2), col, 2)
            cv2.putText(draw, lbl, (x1 + 6, y1 + 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, col, 2)

        if ball_xy:
            cv2.circle(draw, ball_xy, ball_r, (0, 165, 255), 3)
            cv2.circle(draw, ball_xy, 5,      (255, 255, 255), -1)
            zl = zone_label(*ball_xy) or "TRANSIT"
            cv2.putText(draw, f"BALL [{zl}]",
                        (ball_xy[0] - ball_r, ball_xy[1] - ball_r - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 165, 255), 2)

        cv2.putText(draw,
                    f"PICKS:{state['pick_count']}  CYCLES:{state['cycle_count']}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 220, 0), 2)
        cv2.putText(draw, f"FPS:{state['fps']:.1f}",
                    (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (160, 160, 160), 1)

        return draw

    # ─────────────────────────────────────────
    def update_gui(self):
        if not state["running"]:
            return

        self.picks_var.set(str(state["pick_count"]))
        self.cycles_var.set(str(state["cycle_count"]))
        self.fps_var.set(f"{state['fps']:.1f}")
        self.ball_var.set("● DETECTED" if state["ball_detected"] else "○ SEARCHING")

        elapsed = time.time() - state["session_start"]
        h = int(elapsed // 3600)
        m = int((elapsed % 3600) // 60)
        s = int(elapsed % 60)
        self.time_var.set(f"{h:02d}:{m:02d}:{s:02d}")

        log = list(state["event_log"])
        if len(log) != self._last_log_len:
            self._last_log_len = len(log)
            self.log_text.config(state="normal")
            self.log_text.delete("1.0", "end")
            for entry in log:
                tag = ("a2b"    if "A → B" in entry else
                       "b2a"    if "B → A" in entry else
                       "appear")
                self.log_text.insert("end", entry + "\n", tag)
            self.log_text.config(state="disabled")

        self.root.after(200, self.update_gui)

    # ─────────────────────────────────────────
    def on_close(self):
        state["running"] = False
        self.cap.release()
        self.root.destroy()

# ─────────────────────────────────────────────
#  ENTRY POINT
# ─────────────────────────────────────────────
if __name__ == "__main__":
    cap  = init_camera()
    root = tk.Tk()
    app  = App(root, cap)
    root.mainloop()
    print("Session ended.")
