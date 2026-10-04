"""
gui.py — DUAL-ARM CTRL  •  Unified GUI
Controls both Black and White Arms, including Press Synchronization and Camera Feed.
"""

import os
import threading
import time
import tkinter as tk
from datetime import datetime
from tkinter import messagebox, simpledialog, ttk

try:
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

from camera import CAMERA_AVAILABLE, KINECT_AVAILABLE, CameraController
from config import (
    ACCENT, ACCENT2, ACCENT3, BG, BORDER, DANGER, DIM, ENTRY_BG, TEXT, WARN, PANEL, PANEL2,
    BLACK_ARM_COLOR, WHITE_ARM_COLOR,
    BLACK_SERVO_NAMES, BLACK_SERVO_PINS, BLACK_SERVO_MIN, BLACK_SERVO_MAX, BLACK_HOME_POS, BLACK_GRIPPER_OPEN, BLACK_GRIPPER_CLOSED,
    WHITE_SERVO_NAMES, WHITE_SERVO_PINS, WHITE_SERVO_MIN, WHITE_SERVO_MAX, WHITE_HOME_POS, WHITE_GRIPPER_OPEN, WHITE_GRIPPER_CLOSED,
    BLACK_STATIONS_FILE, WHITE_STATIONS_FILE, BLACK_ZONES_FILE, WHITE_ZONES_FILE,
    BLACK_DEFAULT_STATIONS, WHITE_DEFAULT_STATIONS,
    BLACK_WIFI_IP, BLACK_WIFI_PORT, WHITE_WIFI_IP, WHITE_WIFI_PORT,
    ESP2_IP, ESP2_PORT, DEFAULT_BAUD, AVAILABLE_BAUDS,
    WIFI_SOCKET_TIMEOUT, WIFI_CONNECT_TIMEOUT, HELLO_RETRY_MS, HELLO_FIRST_DELAY_MS,
    PRESS_SYNC_TIMEOUT_S, CB_C_PICK_SETTLE_MS, ACB_RETRACT_DWELL_MS,
    CAM_SOURCE_LAPTOP, CAM_SOURCE_KINECT, KINECT_TILT_MIN, KINECT_TILT_MAX
)
from esp_comms import create_esp_comms_handler
import motion_loop as motion_loop_module
from motion_loop import DEFAULT_AB_PHASES, DEFAULT_BLACK_PHASES, DEFAULT_WHITE_PHASES
from servos import ServoController, dark_btn, draw_arc, hsep, section_label
from stations import StationManager
from transport import SerialTransport, WifiTransport

try:
    from PIL import Image, ImageTk
    import cv2
    import numpy as np
except ImportError:
    pass


class ArmPanel(tk.Frame):
    def __init__(self, parent, gui, arm_id: str, color: str,
                 default_ip: str, default_port: int,
                 servo_names: list, servo_pins: list, servo_min: list, servo_max: list,
                 home_pos: list, gripper_open: int, gripper_closed: int,
                 stations_file: str, default_stations: dict, phases: tuple):
        super().__init__(parent, bg=BG)
        self.gui = gui
        self.arm_id = arm_id
        self.color = color
        self.default_ip = default_ip
        self.default_port = default_port

        self.servo_names = servo_names
        self.servo_pins = servo_pins
        self.servo_min = servo_min
        self.servo_max = servo_max
        self.home_pos = home_pos
        self.gripper_open = gripper_open
        self.gripper_closed = gripper_closed
        self.phases = list(phases)

        # ── State ───────────────────────────────────────
        self.transport = None
        self.connected = False
        self.handshake_done = False
        self.transport_mode = tk.StringVar(value="wifi")
        self.running = False
        self.loop_count = 0
        self.speed_delay = tk.IntVar(value=10)

        self._motion_thread = None
        self._motion_ack_cond = threading.Condition()
        self._motion_ack_counts = {"MOVE_DONE": 0, "PRESS_DONE": 0}
        self._btn_paused_confirmed = False
        self._estop_cleared = False
        self._last_sequence = None
        self._current_phase_index = 0
        self._servo_command_inflight = {}
        self._suppress_slider_callback = set()

        # ── Station Manager ─────────────────────────────
        self.sm = StationManager(
            stations_file=stations_file,
            default_stations=default_stations,
            gripper_open=gripper_open,
            gripper_closed=gripper_closed
        )

        names = self.sm.names()
        self.active_station_a = tk.StringVar(value=names[0] if names else "")
        self.active_station_b = tk.StringVar(value=names[1] if len(names) > 1 else (names[0] if names else ""))
        self.press_sync_enabled = tk.BooleanVar(value=True)

        # ── ESP comms ───────────────────────────────────
        self.esp_comms = create_esp_comms_handler(
            log_callback=lambda msg: self.log(msg),
            on_emergency_stop=self.stop_motion
        )

        self._build_layout()

    def log(self, msg: str, level: str = "info"):
        self.gui.log(f"[{self.arm_id}] {msg}", level)

    def _bind_mouse_wheel(self, widget, canvas):
        def _on_mousewheel(event):
            if event.delta:
                canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")
            else:
                if event.num == 5:
                    canvas.yview_scroll(1, "units")
                elif event.num == 4:
                    canvas.yview_scroll(-1, "units")
        try:
            class_name = widget.winfo_class()
        except Exception:
            class_name = ""
        if class_name in ("Frame", "Label", "Canvas"):
            widget.bind("<MouseWheel>", _on_mousewheel)
            widget.bind("<Button-4>", _on_mousewheel)
            widget.bind("<Button-5>", _on_mousewheel)
        for child in widget.winfo_children():
            self._bind_mouse_wheel(child, canvas)

    def _build_layout(self):
        # 3 Columns: Controls (Left), Servos (Center), Station Editor (Right)
        self.columnconfigure(0, weight=0, minsize=260)
        self.columnconfigure(1, weight=1)
        self.columnconfigure(2, weight=0, minsize=320)
        self.rowconfigure(0, weight=1)

        self._build_left_col()
        self._build_center_col()
        self._build_right_col()

    # ═══════════════════════════════════════════════════════
    #  LEFT COLUMN: Connection & Run Loop Controls
    # ═══════════════════════════════════════════════════════
    def _build_left_col(self):
        left_outer = tk.Frame(self, bg=PANEL)
        left_outer.grid(row=0, column=0, sticky="nsew", padx=(0, 1))
        left_outer.columnconfigure(0, weight=1)
        left_outer.rowconfigure(0, weight=1)

        left_canvas = tk.Canvas(left_outer, bg=PANEL, highlightthickness=0)
        left_scroll = tk.Scrollbar(left_outer, orient="vertical", command=left_canvas.yview, bg=BORDER, troughcolor=PANEL)
        left_canvas.configure(yscrollcommand=left_scroll.set)
        left_canvas.grid(row=0, column=0, sticky="nsew")
        left_scroll.grid(row=0, column=1, sticky="ns")

        left = tk.Frame(left_canvas, bg=PANEL)
        canvas_window = left_canvas.create_window((0, 0), window=left, anchor="nw")

        def _sync_scroll_left(_e=None):
            left_canvas.configure(scrollregion=left_canvas.bbox("all"))
            left_canvas.itemconfigure(canvas_window, width=left_canvas.winfo_width())
        left.bind("<Configure>", _sync_scroll_left)

        # --- Connection Box ---
        section_label(left, f"{self.arm_id} Connection")

        mode_frame = tk.Frame(left, bg=PANEL)
        mode_frame.pack(fill="x", padx=14, pady=2)
        for val, lbl, col in [("usb", "USB", ACCENT3), ("wifi", "WiFi", self.color)]:
            tk.Radiobutton(
                mode_frame, text=lbl, variable=self.transport_mode, value=val,
                bg=PANEL, fg=col, selectcolor=PANEL2, activebackground=PANEL, activeforeground=col,
                font=("Courier", 12, "bold"), relief="flat", command=self._on_transport_mode_change
            ).pack(side="left", padx=5)

        # WiFi connection UI
        self.wifi_ui = tk.Frame(left, bg=PANEL)
        self.wifi_ui.pack(fill="x", padx=14, pady=2)
        tk.Label(self.wifi_ui, text="IP:", fg=DIM, bg=PANEL, font=("Courier", 11)).pack(side="left")
        self.wifi_ip_var = tk.StringVar(value=self.default_ip)
        self.wifi_ip_entry = tk.Entry(
            self.wifi_ui, textvariable=self.wifi_ip_var, width=14,
            bg=ENTRY_BG, fg=TEXT, insertbackground=self.color, font=("Courier", 11), relief="flat"
        )
        self.wifi_ip_entry.pack(side="left", padx=2)

        tk.Label(self.wifi_ui, text=":", fg=DIM, bg=PANEL, font=("Courier", 11)).pack(side="left")
        self.wifi_port_var = tk.StringVar(value=str(self.default_port))
        self.wifi_port_entry = tk.Entry(
            self.wifi_ui, textvariable=self.wifi_port_var, width=5,
            bg=ENTRY_BG, fg=TEXT, insertbackground=self.color, font=("Courier", 11), relief="flat"
        )
        self.wifi_port_entry.pack(side="left", padx=2)

        # USB connection UI
        self.usb_ui = tk.Frame(left, bg=PANEL)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(self.usb_ui, textvariable=self.port_var, width=12, state="readonly", font=("Courier", 11))
        self.port_combo.pack(side="left", padx=2)
        self.baud_var = tk.StringVar(value=DEFAULT_BAUD)
        ttk.Combobox(self.usb_ui, textvariable=self.baud_var, width=6, values=AVAILABLE_BAUDS, state="readonly", font=("Courier", 11)).pack(side="left", padx=2)
        dark_btn(self.usb_ui, "↺", self._refresh_ports, DIM, font_size=10).pack(side="left", padx=2)

        self.connect_btn = dark_btn(left, "CONNECT", self.connect_transport, self.color, font_size=12)
        self.connect_btn.pack(fill="x", padx=14, pady=8)
        self._on_transport_mode_change()

        hsep(left)

        # --- Motion Control Box ---
        section_label(left, "Sequence Control")

        self.start_btn = tk.Button(
            left, text="▶  START LOOP", command=self.start_motion,
            bg="#003322", fg=ACCENT, activebackground="#004433", activeforeground=ACCENT,
            font=("Courier", 14, "bold"), relief="flat", highlightbackground=ACCENT, highlightthickness=1
        )
        self.start_btn.pack(fill="x", padx=14, pady=3, ipady=4)

        tk.Button(
            left, text="■  STOP", command=self.stop_motion,
            bg="#220000", fg=DANGER, activebackground="#330000", activeforeground=DANGER,
            font=("Courier", 14, "bold"), relief="flat", highlightbackground=DANGER, highlightthickness=1
        ).pack(fill="x", padx=14, pady=3, ipady=4)

        dark_btn(left, "⌂  GO HOME", self.go_home, ACCENT2).pack(fill="x", padx=14, pady=3, ipady=3)

        self.resume_btn = tk.Button(
            left, text="⟳ RESUME", command=self._resume_sequence,
            bg=PANEL2, fg=ACCENT, activebackground=BORDER, activeforeground=ACCENT,
            font=("Courier", 12, "bold"), relief="flat", highlightbackground=ACCENT, highlightthickness=1
        )
        self.resume_btn.pack(fill="x", padx=14, pady=3, ipady=3)

        # Speed scale
        hsep(left)
        section_label(left, "Speed Delay (ms/step)")
        spd_row = tk.Frame(left, bg=PANEL)
        spd_row.pack(fill="x", padx=14)
        self.speed_disp = tk.Label(spd_row, text="10 ms", fg=ACCENT2, bg=PANEL, font=("Courier", 16, "bold"))
        self.speed_disp.pack(side="right")
        tk.Scale(
            left, from_=1, to=30, orient="horizontal", variable=self.speed_delay,
            bg=PANEL, fg=TEXT, troughcolor=BORDER, activebackground=ACCENT2, highlightthickness=0, bd=0, showvalue=False,
            command=self._on_speed_change
        ).pack(fill="x", padx=14, pady=(0, 6))

        # Routing dropdowns
        hsep(left)
        section_label(left, "Routing Poses")

        row1 = tk.Frame(left, bg=PANEL)
        row1.pack(fill="x", padx=14, pady=2)
        tk.Label(row1, text="PICK:", fg=DIM, bg=PANEL, font=("Courier", 11)).pack(side="left")
        self.station_a_combo = ttk.Combobox(row1, textvariable=self.active_station_a, state="readonly", font=("Courier", 11))
        self.station_a_combo.pack(side="right", fill="x", expand=True)

        row2 = tk.Frame(left, bg=PANEL)
        row2.pack(fill="x", padx=14, pady=2)
        tk.Label(row2, text="PLACE:", fg=DIM, bg=PANEL, font=("Courier", 11)).pack(side="left")
        self.station_b_combo = ttk.Combobox(row2, textvariable=self.active_station_b, state="readonly", font=("Courier", 11))
        self.station_b_combo.pack(side="right", fill="x", expand=True)
        self._refresh_station_combos()

        # Press sync toggle
        sync_frame = tk.Frame(left, bg=PANEL)
        sync_frame.pack(fill="x", padx=14, pady=8)
        tk.Checkbutton(
            sync_frame, text="Press Sync (ESP2)", variable=self.press_sync_enabled,
            bg=PANEL, fg=ACCENT3, selectcolor=PANEL2, activebackground=PANEL, activeforeground=ACCENT3, font=("Courier", 11)
        ).pack(side="left")

        # Quick Goto Buttons — arm-specific stations
        hsep(left)
        section_label(left, "Quick Goto")
        self.goto_frame = tk.Frame(left, bg=PANEL)
        self.goto_frame.pack(fill="x", padx=14, pady=(0, 6))
        self._build_goto_buttons()
        self._bind_mouse_wheel(left_outer, left_canvas)

    def _build_goto_buttons(self):
        """Build arm-specific quick-goto buttons based on arm_id."""
        for w in self.goto_frame.winfo_children():
            w.destroy()

        if self.arm_id == "WHITE ARM":
            station_buttons = [("B", ACCENT), ("C", ACCENT3)]
        else:  # BLACK ARM
            station_buttons = [("A", ACCENT), ("C", ACCENT3)]

        for sname, color in station_buttons:
            sf = tk.Frame(self.goto_frame, bg=PANEL)
            sf.pack(fill="x", pady=3)
            tk.Label(sf, text=f"Station {sname}", fg=color, bg=PANEL,
                     font=("Courier", 11, "bold")).pack(anchor="w", pady=(0, 2))
            btn_row = tk.Frame(sf, bg=PANEL)
            btn_row.pack(fill="x")
            btn_row.columnconfigure(0, weight=1)
            btn_row.columnconfigure(1, weight=1)
            dark_btn(btn_row, f"↑ {sname} ABOVE",
                     lambda n=sname: self.goto_pose(n, "above"), color
                     ).grid(row=0, column=0, sticky="ew", padx=(0, 2), ipady=3)
            dark_btn(btn_row, f"↓ {sname} PICK",
                     lambda n=sname: self.goto_pose(n, "pick"), WARN
                     ).grid(row=0, column=1, sticky="ew", padx=(2, 0), ipady=3)

    def goto_pose(self, station_name: str, pose_type: str):
        st = self.sm.get(station_name)
        if not st:
            self.log(f"Station '{station_name}' not found", "warn")
            return
        pose = st[pose_type]
        speed = self.speed_delay.get()
        self.send_command(ServoController.move_cmd(pose, speed))
        self._apply_pose_to_ui(pose)
        self.log(f"→ {station_name} {pose_type.upper()}: {pose}", "move")

    def _on_transport_mode_change(self):
        if self.transport_mode.get() == "usb":
            self.usb_ui.pack(fill="x", padx=14, pady=2)
            self.wifi_ui.pack_forget()
            self._refresh_ports()
        else:
            self.wifi_ui.pack(fill="x", padx=14, pady=2)
            self.usb_ui.pack_forget()

    def _refresh_ports(self):
        if not SERIAL_AVAILABLE:
            return
        import serial.tools.list_ports as lp
        ports = [p.device for p in lp.comports() if self._looks_like_esp_port(p)]
        self.port_combo["values"] = ports
        if ports:
            self.port_var.set(ports[0])
            self.port_combo.config(state="readonly")
        else:
            self.port_var.set("")
            self.port_combo.config(state="disabled")

    def _looks_like_esp_port(self, port_info) -> bool:
        vid = getattr(port_info, "vid", None)
        if vid in {0x10C4, 0x1A86, 0x303A, 0x0403, 0x1D50}:
            return True
        haystack = " ".join(str(getattr(port_info, attr, "") or "") for attr in ("device", "description", "manufacturer", "product", "hwid")).lower()
        return any(k in haystack for k in ("esp32", "cp210", "ch340", "silicon labs", "usb jtag"))

    # ═══════════════════════════════════════════════════════
    #  CENTER COLUMN: Servo Angle Sliders & Live Control
    # ═══════════════════════════════════════════════════════
    def _build_center_col(self):
        center = tk.Frame(self, bg=PANEL)
        center.grid(row=0, column=1, sticky="nsew", padx=1)
        center.columnconfigure(0, weight=1)
        center.rowconfigure(1, weight=1)

        hdr = tk.Frame(center, bg=PANEL)
        hdr.grid(row=0, column=0, sticky="ew")
        tk.Frame(hdr, bg=self.color, width=3, height=14).pack(side="left", padx=(14, 6), pady=6)
        tk.Label(hdr, text="SERVO ANGLE CONTROLS (LIVE)", fg=DIM, bg=PANEL, font=("Courier", 13, "bold")).pack(side="left")

        # Scrollable container for cards
        cards_wrap = tk.Frame(center, bg=PANEL)
        cards_wrap.grid(row=1, column=0, sticky="nsew", padx=8, pady=4)
        cards_wrap.columnconfigure(0, weight=1)
        cards_wrap.rowconfigure(0, weight=1)

        cards_canvas = tk.Canvas(cards_wrap, bg=PANEL, highlightthickness=0)
        cards_scroll = tk.Scrollbar(cards_wrap, orient="vertical", command=cards_canvas.yview, bg=BORDER, troughcolor=PANEL)
        cards_canvas.configure(yscrollcommand=cards_scroll.set)
        cards_canvas.grid(row=0, column=0, sticky="nsew")
        cards_scroll.grid(row=0, column=1, sticky="ns")

        cards_frame = tk.Frame(cards_canvas, bg=PANEL)
        canvas_window = cards_canvas.create_window((0, 0), window=cards_frame, anchor="nw")

        def _sync_scroll(_e=None):
            cards_canvas.configure(scrollregion=cards_canvas.bbox("all"))
            cards_canvas.itemconfigure(canvas_window, width=cards_canvas.winfo_width())
        cards_frame.bind("<Configure>", _sync_scroll)

        self.target_vars = []
        self.target_entry_vars = []
        self.min_vars = []
        self.max_vars = []
        self.live_labels = []
        self.arc_canvas = []
        self.servo_scales = []
        self.servo_cards = []

        # Build cards for the 6 servos
        for i in range(6):
            card = self._build_servo_card(cards_frame, i)
            card.pack(fill="x", pady=4, padx=4)
            self.servo_cards.append(card)

        # Special Press Card (Only on Black Arm Panel)
        if self.arm_id == "BLACK ARM":
            press_card = self._build_press_card(cards_frame)
            press_card.pack(fill="x", pady=4, padx=4)
            self.servo_cards.append(press_card)

        # Apply all button at bottom
        tk.Button(
            center, text="APPLY ALL SERVOS ▸ MOVE",
            bg="#001a0d", fg=ACCENT, activebackground="#002a14", activeforeground=ACCENT,
            font=("Courier", 13, "bold"), relief="flat", highlightbackground=ACCENT, highlightthickness=1,
            command=self.apply_all_servos
        ).grid(row=2, column=0, sticky="ew", padx=10, pady=8, ipady=6)
        self._bind_mouse_wheel(cards_wrap, cards_canvas)

    def _build_servo_card(self, parent, idx: int):
        card = tk.Frame(parent, bg=PANEL2, highlightthickness=1, highlightbackground=BORDER)
        inner = tk.Frame(card, bg=PANEL2)
        inner.pack(fill="both", expand=True, padx=8, pady=8)

        hdr = tk.Frame(inner, bg=PANEL2)
        hdr.pack(fill="x")
        tk.Label(hdr, text=self.servo_names[idx], fg=TEXT, bg=PANEL2, font=("Courier", 13, "bold")).pack(side="left")
        tk.Label(hdr, text=f"PIN {self.servo_pins[idx]}", fg=DIM, bg=PANEL2, font=("Courier", 9)).pack(side="right")

        live = tk.Label(inner, text=f"{self.home_pos[idx]}°", fg=self.color, bg=PANEL2, font=("Courier", 20, "bold"))
        live.pack(anchor="w")
        self.live_labels.append(live)

        arc = tk.Canvas(inner, bg=PANEL2, height=6, highlightthickness=0)
        arc.pack(fill="x", pady=(1, 2))
        self.arc_canvas.append(arc)
        arc.bind("<Configure>", lambda e, i=idx: self._draw_arc(i))

        t_var = tk.IntVar(value=self.home_pos[idx])
        t_entry_var = tk.StringVar(value=str(self.home_pos[idx]))
        mn_var = tk.IntVar(value=self.servo_min[idx])
        mx_var = tk.IntVar(value=self.servo_max[idx])
        self.target_vars.append(t_var)
        self.target_entry_vars.append(t_entry_var)
        self.min_vars.append(mn_var)
        self.max_vars.append(mx_var)

        slider = tk.Scale(
            inner, from_=self.servo_min[idx], to=self.servo_max[idx], orient="horizontal", variable=t_var,
            bg=PANEL2, fg=TEXT, troughcolor=BORDER, activebackground=ACCENT3, highlightthickness=0, bd=0, showvalue=False,
            command=lambda val, i=idx: self._on_servo_slider(i, val)
        )
        slider.pack(fill="x", pady=(0, 2))
        self.servo_scales.append(slider)

        inp = tk.Frame(inner, bg=PANEL2)
        inp.pack(fill="x")
        for col, (lbl, var) in enumerate([("TARGET", t_var), ("MIN", mn_var), ("MAX", mx_var)]):
            f = tk.Frame(inp, bg=PANEL2)
            f.pack(side="left", fill="x", expand=True, padx=2)
            tk.Label(f, text=lbl, fg=DIM, bg=PANEL2, font=("Courier", 9)).pack(anchor="w")
            entry_var = t_entry_var if col == 0 else var
            e = tk.Entry(f, textvariable=entry_var, bg=ENTRY_BG, fg=TEXT, font=("Courier", 11), relief="flat", justify="center")
            e.pack(fill="x", ipady=2)
            if col == 0:
                e.bind("<Return>", lambda ev, i=idx: self._apply_from_target_entry(i))
                e.bind("<FocusOut>", lambda ev, i=idx: self._commit_target_entry(i))

        tk.Button(
            inner, text=f"APPLY ▸ {self.servo_names[idx]}",
            bg="#001a0d", fg=ACCENT, activebackground="#002a14", activeforeground=ACCENT,
            font=("Courier", 10, "bold"), relief="flat", highlightbackground=BORDER, highlightthickness=1,
            command=lambda i=idx: self.apply_single_servo(i)
        ).pack(fill="x", pady=(4, 0), ipady=2)

        self._draw_arc(idx)
        return card

    def _build_press_card(self, parent):
        card = tk.Frame(parent, bg=PANEL2, highlightthickness=1, highlightbackground=BORDER)
        inner = tk.Frame(card, bg=PANEL2)
        inner.pack(fill="both", expand=True, padx=8, pady=8)

        hdr = tk.Frame(inner, bg=PANEL2)
        hdr.pack(fill="x")
        tk.Label(hdr, text="PRESS SERVO", fg=WARN, bg=PANEL2, font=("Courier", 13, "bold")).pack(side="left")
        tk.Label(hdr, text="ESP2", fg=DIM, bg=PANEL2, font=("Courier", 9)).pack(side="right")

        self.press_live_var = tk.StringVar(value="60°")
        live = tk.Label(inner, textvariable=self.press_live_var, fg=WARN, bg=PANEL2, font=("Courier", 20, "bold"))
        live.pack(anchor="w")

        self.press_target_var = tk.IntVar(value=60)
        slider = tk.Scale(
            inner, from_=60, to=170, orient="horizontal", variable=self.press_target_var,
            bg=PANEL2, fg=TEXT, troughcolor=BORDER, activebackground=WARN, highlightthickness=0, bd=0, showvalue=False
        )
        slider.pack(fill="x", pady=(0, 2))

        tk.Button(
            inner, text="APPLY ▸ PRESS SERVO",
            bg="#221100", fg=WARN, activebackground="#332200", activeforeground=WARN,
            font=("Courier", 10, "bold"), relief="flat", highlightbackground=BORDER, highlightthickness=1,
            command=self._apply_press_servo
        ).pack(fill="x", pady=(8, 0), ipady=2)
        return card

    def _apply_press_servo(self):
        angle = self.press_target_var.get()
        self.press_live_var.set(f"{angle}°")
        self.send_command(f"PRESS_MOVE:{angle}")

    def _draw_arc(self, idx: int):
        draw_arc(self.arc_canvas[idx], self.target_vars[idx], self.min_vars[idx], self.max_vars[idx])

    # ═══════════════════════════════════════════════════════
    #  RIGHT COLUMN: Station List & Editor
    # ═══════════════════════════════════════════════════════
    def _build_right_col(self):
        right_outer = tk.Frame(self, bg=PANEL)
        right_outer.grid(row=0, column=2, sticky="nsew", padx=(1, 0))
        right_outer.columnconfigure(0, weight=1)
        right_outer.rowconfigure(0, weight=1)

        right_canvas = tk.Canvas(right_outer, bg=PANEL, highlightthickness=0)
        right_scroll = tk.Scrollbar(right_outer, orient="vertical", command=right_canvas.yview, bg=BORDER, troughcolor=PANEL)
        right_canvas.configure(yscrollcommand=right_scroll.set)
        right_canvas.grid(row=0, column=0, sticky="nsew")
        right_scroll.grid(row=0, column=1, sticky="ns")

        right = tk.Frame(right_canvas, bg=PANEL)
        canvas_window = right_canvas.create_window((0, 0), window=right, anchor="nw")

        def _sync_scroll_right(_e=None):
            right_canvas.configure(scrollregion=right_canvas.bbox("all"))
            right_canvas.itemconfigure(canvas_window, width=right_canvas.winfo_width())
        right.bind("<Configure>", _sync_scroll_right)

        section_label(right, "Station Editor")

        self.station_name_var = tk.StringVar(value="Select a station")
        nf = tk.Frame(right, bg=PANEL)
        nf.pack(fill="x", padx=14, pady=2)
        tk.Label(nf, textvariable=self.station_name_var, fg=ACCENT3, bg=PANEL, font=("Courier", 14, "bold")).pack(side="left")
        dark_btn(nf, "RENAME", self._rename_station, ACCENT3).pack(side="right")

        self.station_pose_vars = {}
        poses_frame = tk.Frame(right, bg=PANEL)
        poses_frame.pack(fill="x", padx=14, pady=4)
        for pose_type in ["above", "pick"]:
            pf = tk.Frame(poses_frame, bg=PANEL2, highlightthickness=1, highlightbackground=BORDER)
            pf.pack(side="left", fill="both", expand=True, padx=2)
            self._build_pose_editor(pf, pose_type)

        act = tk.Frame(right, bg=PANEL)
        act.pack(fill="x", padx=14, pady=4)
        dark_btn(act, "↑ CAPTURE ABOVE", lambda: self._capture_pose("above"), ACCENT).pack(fill="x", pady=2)
        dark_btn(act, "↓ CAPTURE PICK", lambda: self._capture_pose("pick"), WARN).pack(fill="x", pady=2)
        dark_btn(act, "▶ TEST ABOVE", lambda: self._test_pose("above"), ACCENT3).pack(fill="x", pady=2)
        dark_btn(act, "▶ TEST PICK", lambda: self._test_pose("pick"), WARN).pack(fill="x", pady=2)

        # Station list for selection (hidden header, compact)
        hsep(right)
        section_label(right, "Stations")
        btn_row = tk.Frame(right, bg=PANEL)
        btn_row.pack(fill="x", padx=14, pady=(0, 4))
        dark_btn(btn_row, "+ NEW", self._new_station, ACCENT).pack(side="left", fill="x", expand=True, padx=2)
        dark_btn(btn_row, "✕ DEL", self._delete_station, DANGER).pack(side="left", fill="x", expand=True, padx=2)

        lf = tk.Frame(right, bg=ENTRY_BG, highlightthickness=1, highlightbackground=BORDER)
        lf.pack(fill="both", expand=True, padx=14, pady=4)
        sb = tk.Scrollbar(lf, bg=PANEL, troughcolor=BORDER)
        sb.pack(side="right", fill="y")
        self.station_listbox = tk.Listbox(
            lf, bg=ENTRY_BG, fg=TEXT, font=("Courier", 13), selectbackground=BORDER, selectforeground=ACCENT,
            relief="flat", bd=0, yscrollcommand=sb.set, activestyle="none"
        )
        self.station_listbox.pack(fill="both", expand=True)
        sb.config(command=self.station_listbox.yview)
        self.station_listbox.bind("<<ListboxSelect>>", self._on_station_select)

        tk.Button(
            right, text="💾 SAVE STATION", command=self._save_current_station,
            bg="#001a0d", fg=ACCENT, activebackground="#002a14", activeforeground=ACCENT,
            font=("Courier", 12, "bold"), relief="flat", highlightbackground=ACCENT, highlightthickness=1
        ).pack(fill="x", padx=14, pady=(8, 14), ipady=4)

        self._refresh_station_list()
        self._bind_mouse_wheel(right_outer, right_canvas)

    def _build_pose_editor(self, parent, pose_type: str):
        color = ACCENT if pose_type == "above" else WARN
        tk.Label(parent, text=pose_type.upper(), fg=color, bg=PANEL2, font=("Courier", 11, "bold")).pack(pady=4)
        vars_ = []
        for i in range(6):
            row = tk.Frame(parent, bg=PANEL2)
            row.pack(fill="x", padx=4, pady=1)
            tk.Label(row, text=f"{i}:", fg=DIM, bg=PANEL2, font=("Courier", 10)).pack(side="left")
            v = tk.IntVar(value=0)
            e = tk.Entry(row, textvariable=v, bg=ENTRY_BG, fg=TEXT, font=("Courier", 10), relief="flat", width=4, justify="center")
            e.pack(side="right")
            vars_.append(v)
        self.station_pose_vars[pose_type] = vars_

    # ═══════════════════════════════════════════════════════
    #  LOGIC & COMMANDS
    # ═══════════════════════════════════════════════════════
    def connect_transport(self):
        if self.connected:
            self._disconnect()
            return
        if self.transport_mode.get() == "usb":
            self._connect_usb()
        else:
            self._connect_wifi()

    def _connect_usb(self):
        port = self.port_var.get()
        baud = int(self.baud_var.get())
        if not port:
            messagebox.showerror("No Port", "Select an ESP USB port.")
            return
        try:
            self.transport = SerialTransport(port, baud)
            self._finalize_connect(f"{port.split('/')[-1]} @ {baud} USB")
        except Exception as e:
            self.log(f"USB connection failed: {e}", "err")

    def _connect_wifi(self):
        ip = self.wifi_ip_var.get().strip()
        port_str = self.wifi_port_var.get().strip()
        if not ip:
            messagebox.showerror("No IP", "Enter the ESP32 IP.")
            return
        try:
            port = int(port_str)
            self.transport = WifiTransport(ip, port)
            self._finalize_connect(f"{ip}:{port} WiFi")
        except Exception as e:
            self.log(f"WiFi connection failed: {e}", "err")

    def _finalize_connect(self, label: str):
        self.connected = True
        self.handshake_done = False
        self._set_controls_enabled(False)
        self.connect_btn.config(text="DISCONNECT")
        self.log(f"✓ Connected: {label}")
        self.serial_thread = threading.Thread(target=self._read_transport, daemon=True)
        self.serial_thread.start()
        self.after(HELLO_FIRST_DELAY_MS, self._send_hello)

    def _disconnect(self):
        self.connected = False
        self.running = False
        self.handshake_done = False
        if self.transport:
            self.transport.close()
            self.transport = None
        self.connect_btn.config(text="CONNECT")
        self._set_controls_enabled(False)
        self.gui.update_arm_status(self.arm_id, "DISCONNECTED", DIM)
        self.log("Disconnected", "warn")

    def _send_hello(self):
        if not self.connected or self.handshake_done:
            return
        try:
            self.transport.send("HELLO")
        except Exception as e:
            self.log(f"HELLO failed: {e}", "err")
            return
        self.after(HELLO_RETRY_MS, self._send_hello)

    def _read_transport(self):
        while self.connected and self.transport:
            try:
                line = self.transport.readline()
                if line:
                    self.after(0, lambda l=line: self._handle_rx(l))
            except Exception as e:
                if self.connected:
                    self.after(0, lambda err=str(e): self.log(f"⚠ Transport error: {err} — attempting reconnect...", "warn"))
                    self.after(0, self._on_transport_drop)
                break

    def _on_transport_drop(self):
        """Handle a dropped connection. WiFi: auto-reconnect. USB: clean disconnect."""
        was_running = self.running
        was_wifi    = self.transport_mode.get() == "wifi"

        # Stop motion thread but keep self.connected=True so reconnect can use it
        self.running = False
        self.handshake_done = False
        with self._motion_ack_cond:
            self._motion_ack_cond.notify_all()
        if self.transport:
            try:
                self.transport.close()
            except Exception:
                pass
            self.transport = None

        self.gui.update_arm_status(self.arm_id, "RECONNECTING...", WARN)
        self.log("⚡ Connection lost — reconnecting in 3 s...", "warn")

        if was_wifi:
            self.after(3000, lambda: self._auto_reconnect(was_running))
        else:
            self.connected = False
            self.connect_btn.config(text="CONNECT")
            self._set_controls_enabled(False)
            self.gui.update_arm_status(self.arm_id, "DISCONNECTED", DIM)

    def _auto_reconnect(self, resume_motion: bool):
        if not self.connected:
            return
        ip = self.wifi_ip_var.get().strip()
        try:
            port = int(self.wifi_port_var.get().strip())
        except ValueError:
            self._disconnect()
            return
        try:
            self.transport = WifiTransport(ip, port)
            self.handshake_done = False
            self.log(f"✓ Reconnected to {ip}:{port} — re-handshaking...")
            self._pending_resume = resume_motion
            self.serial_thread = threading.Thread(target=self._read_transport, daemon=True)
            self.serial_thread.start()
            self.after(HELLO_FIRST_DELAY_MS, self._send_hello)
        except Exception as e:
            self.log(f"✕ Reconnect failed: {e} — retrying in 5 s...", "err")
            self.after(5000, lambda: self._auto_reconnect(resume_motion))

    def _handle_rx(self, line: str):
        line = line.strip()
        norm = "S" + line if line.startswith("ERVO_DONE:") else line

        # Don't spam normal moves or noisy ESP32 debug logs
        is_noisy = (
            any(line.startswith(k) for k in ("[CMD]", "[WIFI]", "[SETUP]", "[INIT]"))
            or "client connected" in line.lower()
            or "secondary client" in line.lower()
        )
        if is_noisy:
            if any(k in line for k in ("PICK_READY", "ERROR", "FAIL")):
                self.log(f"← {line}", "rx")
        elif not any(norm.startswith(k) for k in ("SERVO_DONE", "MOVE_DONE", "PRESS_DONE", "PRESS_WAIT", "SPEED_SET", "STOPPED", "HOME_DONE")):
            self.log(f"← {line}", "rx")

        if norm == "READY" and not self.handshake_done:
            self.handshake_done = True
            self._on_handshake_ready()
        elif norm.startswith("SERVO_DONE:"):
            try:
                idx = int(norm.split(":", 1)[1])
                with self._motion_ack_cond:
                    key = f"SERVO_DONE:{idx}"
                    self._motion_ack_counts[key] = self._motion_ack_counts.get(key, 0) + 1
                    self._motion_ack_cond.notify_all()
                self._servo_command_inflight[idx] = False
                self._send_pending_slider_command(idx)
            except ValueError:
                pass
        elif norm == "MOVE_DONE":
            with self._motion_ack_cond:
                self._motion_ack_counts["MOVE_DONE"] += 1
                self._motion_ack_cond.notify_all()
        elif norm == "PRESS_DONE":
            with self._motion_ack_cond:
                self._motion_ack_counts["PRESS_DONE"] += 1
                self._motion_ack_cond.notify_all()
        elif norm == "PICK_READY_FROM_PRESS":
            with self._motion_ack_cond:
                self._motion_ack_counts["PICK_READY_FROM_PRESS"] = self._motion_ack_counts.get("PICK_READY_FROM_PRESS", 0) + 1
                self._motion_ack_cond.notify_all()
        elif norm.startswith("IP:"):
            ip = line[3:]
            self.wifi_ip_var.set(ip)
            self.log(f"ESP32 IP detected: {ip}")
        elif norm in ("STOPPED", "ESTOP_TRIGGERED", "BUTTON_STOP"):
            if self.running:
                self.log(f"← {norm}: halting loop", "warn")
                self.stop_motion()
        elif norm == "BTN_PAUSED":
            self.log("⏸ Pause button pressed — paused. Click RESUME to continue.", "warn")
            self._btn_paused_confirmed = True
            if self.running:
                self.stop_motion()
            threading.Thread(target=self._send_stop_to_esp2, daemon=True).start()
        elif norm == "BTN_RESUMED":
            self.log("▶ Arm unpaused by GUI")
            self._estop_cleared = True
        elif norm == "ESTOP_ERROR:SWITCH_ENGAGED":
            self.log("✕ CANNOT RESUME! Physical Emergency Switch is still engaged.", "err")
            if self.running:
                self.stop_motion()
        elif norm.startswith("PRESS_ERROR:"):
            # Wake the motion-loop thread first so wait_for_ack unblocks and
            # exits cleanly, THEN stop.  Calling stop_motion() first would
            # set running=False before notify, which is fine, but the notify
            # ensures no deadlock if the thread is already inside wait_for_ack.
            with self._motion_ack_cond:
                self._motion_ack_cond.notify_all()
            self.log(f"← {norm} — aborting press sequence", "err")
            if self.running:
                self.stop_motion()
        elif norm == "PRESS_CHECK_OK":
            self.log("✓ ESP1 established network connection to ESP2")
        elif norm == "PRESS_CHECK_FAIL":
            if self.arm_id == "BLACK ARM":
                self.log("✕ ESP1-B could not reach ESP2! Check press machine IP/power.", "err")
            else:
                # White Arm receives PRESS_CHECK_FAIL only if PRESS_CHECK was
                # accidentally sent to it — log as a warning, not an error.
                self.log("⚠ White Arm ESP1 PRESS_CHECK_FAIL (ignored — White Arm is a PICK_READY server, not a press initiator)", "warn")
        elif norm == "ESTOP_CLEARED":
            self.log("✓ ESTOP cleared on ESP1")
            self._estop_cleared = True

    def send_command(self, cmd: str, log_cmd: bool = True):
        if not self.handshake_done:
            return
        if self.transport and self.connected:
            try:
                self.transport.send(cmd)
                if log_cmd:
                    self.log(f"→ {cmd}", "cmd")
            except Exception as e:
                self.log(f"Send error: {e}", "err")
                self._disconnect()

    def _on_handshake_ready(self):
        self._set_controls_enabled(True)
        tag = "WiFi" if self.transport_mode.get() == "wifi" else "USB"
        ip = self.wifi_ip_var.get() if tag == "WiFi" else self.port_var.get().split("/")[-1]
        self.gui.update_arm_status(self.arm_id, f"CONNECTED ({tag})", ACCENT)
        self.log(f"✓ Handshake complete [{tag}] — controls enabled")

        # Always inform ESP1 of ESP2's IP
        self.send_command(f"SET_ESP2_IP:{ESP2_IP}")

        if self.arm_id == "BLACK ARM":
            self.send_command("PRESS_CHECK")

    def _set_controls_enabled(self, enabled: bool):
        state = "normal" if enabled else "disabled"
        self.start_btn.config(state=state)
        for s in self.servo_scales:
            s.config(state=state)

    def _on_speed_change(self, val):
        v = int(float(val))
        self.speed_disp.config(text=f"{v} ms")
        self.send_command(ServoController.speed_cmd(v))

    def _on_servo_slider(self, idx: int, val):
        if idx in self._suppress_slider_callback:
            self._suppress_slider_callback.remove(idx)
            return
        v = int(float(val))
        self.live_labels[idx].config(text=f"{v}°")
        self._draw_arc(idx)
        self.target_entry_vars[idx].set(str(v))
        if not self._servo_command_inflight.get(idx, False):
            self._servo_command_inflight[idx] = True
            self.send_command(ServoController.single_cmd(idx, v, self.speed_delay.get()), log_cmd=False)

    def _send_pending_slider_command(self, idx: int):
        # Placeholder for deferred slider events if needed
        pass

    def _apply_from_target_entry(self, idx: int):
        self._commit_target_entry(idx)
        self.apply_single_servo(idx)

    def _commit_target_entry(self, idx: int):
        try:
            val = int(self.target_entry_vars[idx].get())
            mn = self.min_vars[idx].get()
            mx = self.max_vars[idx].get()
            val = max(mn, min(mx, val))
            self.target_vars[idx].set(val)
            self.live_labels[idx].config(text=f"{val}°")
            self._draw_arc(idx)
        except ValueError:
            self.target_entry_vars[idx].set(str(self.target_vars[idx].get()))

    def apply_single_servo(self, idx: int):
        self._commit_target_entry(idx)
        val = self.target_vars[idx].get()
        self.send_command(ServoController.single_cmd(idx, val, self.speed_delay.get()))

    def apply_all_servos(self):
        vals = []
        for i in range(6):
            self._commit_target_entry(i)
            vals.append(self.target_vars[i].get())
        self.send_command(ServoController.move_cmd(vals, self.speed_delay.get()))

    def go_home(self):
        self.send_command(ServoController.home_cmd())
        self._apply_pose_to_ui(self.home_pos)

    def _apply_pose_to_ui(self, pose: list):
        for i, v in enumerate(pose):
            if i < len(self.target_vars):
                self.target_vars[i].set(v)
                self.live_labels[i].config(text=f"{v}°")
                self.target_entry_vars[i].set(str(v))
                self._draw_arc(i)

    # ── Station Manager Handlers ──────────────────
    def _refresh_station_list(self):
        self.station_listbox.delete(0, "end")
        for name in self.sm.names():
            self.station_listbox.insert("end", f"  {name}")

    def _refresh_station_combos(self):
        names = self.sm.names()
        self.station_a_combo["values"] = names
        self.station_b_combo["values"] = names
        if names:
            if self.active_station_a.get() not in names: self.active_station_a.set(names[0])
            if self.active_station_b.get() not in names: self.active_station_b.set(names[-1])

    def _on_station_select(self, event):
        sel = self.station_listbox.curselection()
        if not sel:
            return
        name = self.sm.names()[sel[0]]
        self.station_name_var.set(name)
        st = self.sm.get(name)
        if st:
            for pose_type in ["above", "pick"]:
                pose = st.get(pose_type, [0]*6)
                for i, v in enumerate(self.station_pose_vars[pose_type]):
                    v.set(pose[i])

    def _new_station(self):
        name = simpledialog.askstring("New Station", "Station name:", parent=self)
        if name and name.strip():
            name = name.strip().upper()
            if name in self.sm.stations:
                return
            current = [v.get() for v in self.target_vars[:6]]
            self.sm.set_above(name, current)
            self.sm.set_pick(name, current)
            self._refresh_station_list()
            self._refresh_station_combos()
            self.log(f"Created station: {name}")

    def _delete_station(self):
        name = self.station_name_var.get()
        if name in ("Select a station", ""):
            return
        if messagebox.askyesno("Delete", f"Delete station '{name}'?", parent=self):
            self.sm.delete(name)
            self.station_name_var.set("Select a station")
            self._refresh_station_list()
            self._refresh_station_combos()

    def _rename_station(self):
        old = self.station_name_var.get()
        if old not in self.sm.stations:
            return
        new = simpledialog.askstring("Rename", f"Rename '{old}' to:", parent=self)
        if new and new.strip():
            new = new.strip().upper()
            if self.sm.rename(old, new):
                self.station_name_var.set(new)
                self._refresh_station_list()
                self._refresh_station_combos()

    def _capture_pose(self, pose_type: str):
        current = [v.get() for v in self.target_vars]
        for i, v in enumerate(self.station_pose_vars[pose_type]):
            v.set(current[i])
        self.log(f"Captured {pose_type.upper()} pose from sliders")

    def _test_pose(self, pose_type: str):
        vals = [v.get() for v in self.station_pose_vars[pose_type]]
        self.send_command(ServoController.move_cmd(vals, self.speed_delay.get()))
        self._apply_pose_to_ui(vals)

    def _save_current_station(self):
        name = self.station_name_var.get()
        if name not in self.sm.stations:
            return
        above = [v.get() for v in self.station_pose_vars["above"]]
        pick = [v.get() for v in self.station_pose_vars["pick"]]

        # Enforce correct gripper poses
        if above[5] != self.gripper_open:
            above[5] = self.gripper_open
            self.station_pose_vars["above"][5].set(self.gripper_open)
        if pick[5] != self.gripper_closed:
            pick[5] = self.gripper_closed
            self.station_pose_vars["pick"][5].set(self.gripper_closed)

        self.sm.set_above(name, above)
        self.sm.set_pick(name, pick)
        self._refresh_station_combos()
        self.log(f"Saved station '{name}' configs")

    # ═══════════════════════════════════════════════════════
    #  RUN LOOP RUNNERS
    # ═══════════════════════════════════════════════════════
    def start_motion(self, resume: bool = False):
        if not resume:
            self._current_phase_index = 0
        self._last_sequence = "ACB"
        self._estop_cleared = False

        if not self.handshake_done:
            messagebox.showwarning("Not Connected", f"Connect to {self.arm_id} first.", parent=self)
            return

        self.running = True
        self.gui.update_arm_status(self.arm_id, "RUNNING", ACCENT2)
        self.start_btn.config(bg="#004433")
        self.log("▶ Started loop sequence")

        self._motion_thread = threading.Thread(target=self._run_loop, daemon=True)
        self._motion_thread.start()

        # When the BLACK ARM starts, also start the WHITE ARM loop if it is
        # connected but idle.  The White ARM loop begins with "Wait for press
        # signal" so it just parks at B-above and waits harmlessly.
        if self.arm_id == "BLACK ARM" and not resume:
            self.after(200, self.gui.maybe_start_white_arm)

    def stop_motion(self):
        self.running = False
        with self._motion_ack_cond:
            self._motion_ack_cond.notify_all()
        self.gui.update_arm_status(self.arm_id, "IDLE", DIM)
        self.start_btn.config(bg="#003322")
        self.send_command(ServoController.stop_cmd())
        self.log("■ Stopped sequence", "warn")

    def _run_loop(self):
        try:
            motion_loop_module.run_motion_loop(
                loop_name=f"{self.arm_id} Loop",
                phases=self.phases,
                is_running=lambda: self.running,
                get_speed=self.speed_delay.get,
                get_initial_pose=lambda: [v.get() for v in self.target_vars],
                get_stations=lambda: {n: self.sm.get(n) for n in self.sm.names()},
                send_raw=self._send_motion_command,
                apply_pose=lambda pose: self.after(0, lambda p=list(pose): self._apply_pose_to_ui(p)),
                log_info=lambda msg: self.log(msg),
                log_error=lambda msg: self.log(msg, "err"),
                on_cycle_complete=self._on_cycle_complete,
                get_ack_token=self._get_motion_ack_token,
                wait_for_ack=self._wait_for_motion_ack,
                is_press_enabled=self.press_sync_enabled.get,
                start_phase_index=self._current_phase_index,
                on_phase_change=lambda idx: setattr(self, "_current_phase_index", idx)
            )
        except Exception as e:
            self.after(0, lambda err=str(e): self.log(f"Loop failed: {err}", "err"))

        self.running = False
        self.after(0, lambda: self.gui.update_arm_status(self.arm_id, "IDLE", DIM))
        self.after(0, lambda: self.start_btn.config(bg="#003322"))

    def _send_motion_command(self, cmd: str):
        if self.transport and self.connected:
            try:
                self.transport.send(cmd)
                self.after(0, lambda: self.log(f"→ {cmd}", "cmd"))
            except Exception as e:
                self.after(0, lambda err=str(e): self.log(f"Send error: {err}", "err"))

    def _on_cycle_complete(self):
        self.loop_count += 1
        self.after(0, lambda: self.gui.update_loop_lbl(self.arm_id, self.loop_count))

    def _get_motion_ack_token(self, key: str) -> int:
        with self._motion_ack_cond:
            return self._motion_ack_counts.get(key, 0)

    def _wait_for_motion_ack(self, key: str, token: int, timeout_s: float) -> bool:
        deadline = time.time() + timeout_s
        with self._motion_ack_cond:
            while self._motion_ack_counts.get(key, 0) <= token:
                if not self.running:
                    return False
                remaining = deadline - time.time()
                if remaining <= 0:
                    return False
                self._motion_ack_cond.wait(timeout=min(0.1, remaining))
            return True

    def _send_stop_to_esp2(self):
        import socket
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(1.0)
            s.connect((ESP2_IP, ESP2_PORT))
            s.sendall(b"STOP\n")
            s.close()
            self.after(0, lambda: self.log("✓ STOP command sent to ESP2"))
        except Exception as e:
            self.after(0, lambda err=str(e): self.log(f"⚠ Could not reach ESP2: {err}", "warn"))

    def _resume_sequence(self):
        if not self._last_sequence:
            return
        self.log("⟳ RESUME triggered — waiting for pause state...")
        self._btn_paused_confirmed = False
        self._estop_cleared = False
        self._resume_poll(0)

    def _resume_poll(self, elapsed_ms: int):
        if elapsed_ms >= 3000 or self._btn_paused_confirmed:
            self.log("→ Sending RESUME_BTN to ESP1")
            self.send_command("RESUME_BTN")
            self.start_motion(resume=True)
        else:
            self.after(100, lambda: self._resume_poll(elapsed_ms + 100))


class DualArmGUI:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("DUAL-ARM CTRL — Integrated Production Panel")
        self.root.configure(bg=BG)
        self.root.minsize(1280, 600)

        # ── Shared Camera Controller ─────────────────────
        self.cc = CameraController()
        self.cam_source = tk.StringVar(value=CAM_SOURCE_LAPTOP)
        self.kinect_tilt = tk.IntVar(value=0)
        self._tilt_pending = 0
        self._cap_frame = None
        self._cap_lock = threading.Lock()
        self._cap_thread = None
        self.cam_photo = None

        self._build_ui()
        self._blink_state = False
        self._blink_loop()

    def _build_ui(self):
        self.root.rowconfigure(1, weight=1)
        self.root.columnconfigure(0, weight=1)

        # Header Panel
        self._build_header()

        # Body: Left split (tabs for arms/camera), Right split (shared log)
        body = tk.Frame(self.root, bg=BG)
        body.grid(row=1, column=0, sticky="nsew")
        body.rowconfigure(0, weight=1)
        body.columnconfigure(0, weight=3) # Main Panel
        body.columnconfigure(1, weight=0, minsize=220) # Shared Log Panel

        # Left: Tab container
        tabs_frame = tk.Frame(body, bg=BG)
        tabs_frame.grid(row=0, column=0, sticky="nsew")

        self.notebook = ttk.Notebook(tabs_frame)
        self.notebook.pack(fill="both", expand=True, padx=4, pady=4)

        # Instantiating the two independent Arm Panels
        self.black_panel = ArmPanel(
            self.notebook, self, "BLACK ARM", BLACK_ARM_COLOR,
            BLACK_WIFI_IP, BLACK_WIFI_PORT,
            BLACK_SERVO_NAMES, BLACK_SERVO_PINS, BLACK_SERVO_MIN, BLACK_SERVO_MAX,
            BLACK_HOME_POS, BLACK_GRIPPER_OPEN, BLACK_GRIPPER_CLOSED,
            BLACK_STATIONS_FILE, BLACK_DEFAULT_STATIONS, DEFAULT_BLACK_PHASES
        )

        self.white_panel = ArmPanel(
            self.notebook, self, "WHITE ARM", WHITE_ARM_COLOR,
            WHITE_WIFI_IP, WHITE_WIFI_PORT,
            WHITE_SERVO_NAMES, WHITE_SERVO_PINS, WHITE_SERVO_MIN, WHITE_SERVO_MAX,
            WHITE_HOME_POS, WHITE_GRIPPER_OPEN, WHITE_GRIPPER_CLOSED,
            WHITE_STATIONS_FILE, WHITE_DEFAULT_STATIONS, DEFAULT_WHITE_PHASES
        )

        self.notebook.add(self.black_panel, text="  ⚙ BLACK ARM  ")
        self.notebook.add(self.white_panel, text="  ⚙ WHITE ARM  ")

        # Camera Tab
        self.cam_tab = tk.Frame(self.notebook, bg=BG)
        self.notebook.add(self.cam_tab, text="  📷 LIVE CAMERA  ")
        self._build_camera_tab(self.cam_tab)

        # Right: Shared Log
        log_panel = tk.Frame(body, bg=PANEL)
        log_panel.grid(row=0, column=1, sticky="nsew", padx=(1, 0))
        self._build_log_panel(log_panel)

    def _build_header(self):
        hdr = tk.Frame(self.root, bg="#111614", height=55)
        hdr.grid(row=0, column=0, sticky="ew")
        hdr.grid_propagate(False)

        tk.Label(hdr, text="DUAL-ARM CTRL", bg="#111614", fg=ACCENT, font=("Courier", 20, "bold")).pack(side="left", padx=16)

        # Status pills for both arms
        self.pills = {}
        for arm in ("BLACK ARM", "WHITE ARM"):
            f = tk.Frame(hdr, bg=ENTRY_BG, highlightthickness=1, highlightbackground=BORDER)
            f.pack(side="left", padx=6, pady=8, ipady=2, ipadx=6)
            dot = tk.Label(f, text="●", fg=DIM, bg=ENTRY_BG, font=("Courier", 12))
            dot.pack(side="left", padx=2)
            lbl = tk.Label(f, text=f"{arm}: OFFLINE", fg=DIM, bg=ENTRY_BG, font=("Courier", 11, "bold"))
            lbl.pack(side="left", padx=2)
            self.pills[arm] = (dot, lbl)

    def update_arm_status(self, arm: str, status: str, color: str):
        dot, lbl = self.pills[arm]
        dot.config(fg=color)
        lbl.config(text=f"{arm}: {status}", fg=color)

    def update_loop_lbl(self, arm: str, count: int):
        self.log(f"Cycle completed for {arm}: {count} cycles")

    def maybe_start_white_arm(self, retries: int = 5):
        """Called by Black ARM START — starts White ARM if connected and idle.
        White ARM loop immediately parks at B-above then blocks on wait_for_press.
        Retries periodically if White Arm is currently in the handshake process."""
        wp = self.white_panel
        if wp.handshake_done and not wp.running:
            self.log("[WHITE ARM] ▶ Auto-started by Black ARM START — parking at B-above, waiting for press")
            wp.start_motion()
        elif wp.connected and not wp.handshake_done and retries > 0:
            # Re-check after 500 ms in case handshake completes
            self.after(500, lambda: self.maybe_start_white_arm(retries - 1))

    def _build_log_panel(self, parent):
        parent.rowconfigure(1, weight=1)
        parent.columnconfigure(0, weight=1)

        section_label(parent, "Shared System Logs", grid_opts={"row": 0, "column": 0, "sticky": "ew", "padx": 16, "pady": (14, 6)})

        self.log_text = tk.Text(parent, bg=ENTRY_BG, fg=TEXT, font=("Courier", 9), relief="flat", state="disabled",
                                 width=28)
        self.log_text.grid(row=1, column=0, sticky="nsew", padx=10, pady=4)

        # Log coloring tags
        for tag, fg in [("info", TEXT), ("warn", WARN), ("err", DANGER), ("cmd", ACCENT3), ("rx", ACCENT2), ("ts", DIM), ("move", ACCENT3)]:
            self.log_text.tag_config(tag, foreground=fg)

        dark_btn(parent, "CLEAR LOG", self.clear_log, DANGER).grid(row=2, column=0, sticky="ew", padx=10, pady=8)

    def log(self, msg: str, level: str = "info"):
        ts = datetime.now().strftime("%H:%M:%S")
        self.log_text.config(state="normal")

        # Trim log text if it exceeds 300 lines to keep UI responsive and save space
        num_lines = int(self.log_text.index("end-1c").split(".")[0])
        if num_lines > 300:
            # Delete from start to the line to remove (keep last 300 lines)
            self.log_text.delete("1.0", f"{num_lines - 299}.0")

        self.log_text.insert("end", f"{ts} ", "ts")
        self.log_text.insert("end", f"{msg}\n", level)
        self.log_text.see("end")
        self.log_text.config(state="disabled")

    def clear_log(self):
        self.log_text.config(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.config(state="disabled")

    # ═══════════════════════════════════════════════════════
    #  CAMERA TAB BUILD & CONTROLS
    # ═══════════════════════════════════════════════════════
    def _build_camera_tab(self, parent):
        if not CAMERA_AVAILABLE:
            tk.Label(parent, text="Camera libraries not available", bg=BG, fg=DIM, font=("Courier", 16)).pack(expand=True)
            return

        parent.columnconfigure(0, weight=3)
        parent.columnconfigure(1, weight=1, minsize=260)
        parent.rowconfigure(0, weight=1)

        # Feed Frame (Left)
        feed_wrap = tk.Frame(parent, bg=BG)
        feed_wrap.grid(row=0, column=0, sticky="nsew")
        feed_wrap.rowconfigure(1, weight=1)
        feed_wrap.columnconfigure(0, weight=1)

        ctrl_bar = tk.Frame(feed_wrap, bg=PANEL)
        ctrl_bar.grid(row=0, column=0, sticky="ew")

        # Source Selection
        src_btn_row = tk.Frame(ctrl_bar, bg=PANEL)
        src_btn_row.pack(side="left", padx=10, pady=4)
        for src, label, col in [(CAM_SOURCE_LAPTOP, "LAPTOP", ACCENT3), (CAM_SOURCE_KINECT, "KINECT", ACCENT)]:
            rb = tk.Radiobutton(
                src_btn_row, text=label, variable=self.cam_source, value=src,
                bg=PANEL, fg=col, selectcolor=PANEL2, activebackground=PANEL, activeforeground=col,
                font=("Courier", 12, "bold")
            )
            rb.pack(side="left", padx=4)
            if src == CAM_SOURCE_KINECT and not KINECT_AVAILABLE:
                rb.config(state="disabled", fg=DIM)

        dark_btn(ctrl_bar, "▶ START FEED", self._start_camera, ACCENT).pack(side="left", padx=4)
        dark_btn(ctrl_bar, "■ STOP FEED", self._stop_camera, DANGER).pack(side="left", padx=4)

        # Video frame
        self.cam_feed_frame = tk.Frame(feed_wrap, bg="black", highlightthickness=2, highlightbackground=ACCENT)
        self.cam_feed_frame.grid(row=1, column=0, sticky="nsew", padx=10, pady=10)
        self.cam_label = tk.Label(self.cam_feed_frame, bg="black", text="Camera feed offline", fg=DIM, font=("Courier", 14))
        self.cam_label.place(relx=0, rely=0, relwidth=1, relheight=1)

        # Right: Camera details
        details = tk.Frame(parent, bg=PANEL)
        details.grid(row=0, column=1, sticky="nsew", padx=(1, 0))
        section_label(details, "Camera Stats")

        self.cam_picks_var = tk.StringVar(value="0")
        self.cam_press_var = tk.StringVar(value="0")
        self.cam_fps_var = tk.StringVar(value="0.0")

        for lbl, var, col in [("PICKS DETECTED", self.cam_picks_var, ACCENT),
                              ("PRESS CYCLES", self.cam_press_var, ACCENT3),
                              ("FPS", self.cam_fps_var, DIM)]:
            f = tk.Frame(details, bg=PANEL2, highlightthickness=1, highlightbackground=BORDER)
            f.pack(fill="x", padx=10, pady=4, ipady=4)
            tk.Label(f, text=lbl, fg=DIM, bg=PANEL2, font=("Courier", 11)).pack(anchor="w", padx=8)
            tk.Label(f, textvariable=var, fg=col, bg=PANEL2, font=("Courier", 16, "bold")).pack(anchor="w", padx=8)

        self._cam_stats_loop()

    def _start_camera(self):
        if self.cc.cam_running:
            return
        if self.cam_source.get() == CAM_SOURCE_KINECT:
            self._start_kinect()
        else:
            self._start_laptop()

    def _start_laptop(self):
        self.cc.cap = cv2.VideoCapture(0)
        if not self.cc.cap.isOpened():
            return
        self.cc.cam_running = True
        self._cap_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._cap_thread.start()
        self._cam_loop()

    def _start_kinect(self):
        pass # Placeholder for Kinect start logic if desired

    def _capture_loop(self):
        while self.cc.cam_running and self.cc.cap:
            ret, f = self.cc.cap.read()
            if ret and f is not None:
                with self._cap_lock:
                    self._cap_frame = f
            else:
                time.sleep(0.01)

    def _cam_loop(self):
        if not self.cc.cam_running:
            return
        frame = None
        with self._cap_lock:
            if self._cap_frame is not None:
                frame = self._cap_frame.copy()
        if frame is not None:
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(rgb)

            w = max(50, self.cam_feed_frame.winfo_width())
            h = max(50, self.cam_feed_frame.winfo_height())
            img = img.resize((w, h), Image.LANCZOS)

            photo = ImageTk.PhotoImage(image=img)
            self.cam_label.configure(image=photo, text="")
            self.cam_photo = photo
        self.root.after(33, self._cam_loop)

    def _stop_camera(self):
        self.cc.cam_running = False
        if self.cc.cap:
            self.cc.cap.release()
            self.cc.cap = None
        self.cam_label.configure(image="", text="Camera feed offline")
        self.cam_photo = None

    def _cam_stats_loop(self):
        if CAMERA_AVAILABLE and self.cc.cam_running:
            self.cam_picks_var.set(str(self.cc.pick_count))
            self.cam_press_var.set(str(self.cc.press_count))
            self.cam_fps_var.set(f"{self.cc.cam_fps:.1f}")
        self.root.after(250, self._cam_stats_loop)

    def _blink_loop(self):
        self._blink_state = not self._blink_state
        for arm_id, panel in [("BLACK ARM", self.black_panel), ("WHITE ARM", self.white_panel)]:
            if panel.connected:
                color = ACCENT if self._blink_state else "#007040"
                if panel.running:
                    color = ACCENT2 if self._blink_state else "#7a3010"
                self.update_arm_status(arm_id, "RUNNING" if panel.running else "CONNECTED", color)
        self.root.after(700, self._blink_loop)

    def on_close(self):
        self._stop_camera()
        self.black_panel._disconnect()
        self.white_panel._disconnect()
        self.root.destroy()
