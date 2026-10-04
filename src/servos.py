"""
servos.py — Servo control logic + UI widget helpers

Provides:
  • ServoController  — stateless command-builder; knows nothing about tkinter.
  • draw_arc()       — pure canvas drawing helper used by the GUI.
  • dark_btn()       — reusable styled button factory.
  • section_label()  — reusable section-header widget factory.
  • hsep()           — horizontal separator factory.

All servo-specific constants (names, pins, limits, home) live in config.py.
"""

import tkinter as tk

from config import (
    SERVO_NAMES, SERVO_MIN, SERVO_MAX, HOME_POS,
    ACCENT, ACCENT2, ACCENT3, BG, PANEL, PANEL2,
    BORDER, DIM, TEXT, ENTRY_BG, WARN, DANGER,
)


# ─────────────────────────────────────────────────────────
#  SERVO CONTROLLER  (pure logic, no GUI)
# ─────────────────────────────────────────────────────────
class ServoController:
    """Builds command strings; actual sending is done by the GUI via send_command."""

    @staticmethod
    def clamp(idx: int, value: int, mn: int, mx: int) -> int:
        return max(mn, min(mx, value))

    @staticmethod
    def single_cmd(idx: int, value: int, speed: int) -> str:
        return f"SERVO:{idx},{value},{speed}"

    @staticmethod
    def move_cmd(values: list, speed: int) -> str:
        return f"MOVE:{','.join(map(str, values))},{speed}"

    @staticmethod
    def stop_cmd() -> str:
        return "STOP"

    @staticmethod
    def home_cmd() -> str:
        return "HOME"

    @staticmethod
    def speed_cmd(speed: int) -> str:
        return f"SPEED:{speed}"


# ─────────────────────────────────────────────────────────
#  UI HELPERS
# ─────────────────────────────────────────────────────────
def draw_arc(canvas: tk.Canvas, target_var: tk.IntVar,
             min_var: tk.IntVar, max_var: tk.IntVar):
    """Render the position bar inside a servo card canvas widget."""
    try:
        canvas.update_idletasks()
        w = canvas.winfo_width()
        if w < 2:
            return
        canvas.delete("all")
        canvas.create_rectangle(0, 2, w, 6, fill=BORDER, outline="")
        val = target_var.get()
        mn  = min_var.get()
        mx  = max_var.get()
        pct = max(0.0, min(1.0, (val - mn) / max(1, mx - mn)))
        fw  = int(w * pct)
        if fw > 0:
            canvas.create_rectangle(0, 2, fw, 6, fill=ACCENT3, outline="")
        canvas.create_rectangle(max(0, fw - 1), 0, fw + 1, 8, fill=ACCENT, outline="")
    except Exception:
        pass


def dark_btn(parent, text: str, cmd, color=DIM, height=1, font_size=14):
    """Consistently styled flat dark button."""
    return tk.Button(
        parent, text=text, command=cmd,
        bg=PANEL2, fg=color,
        activebackground="#1a1f1c", activeforeground=color,
        font=("Courier", font_size, "bold"),
        relief="flat", bd=0, cursor="hand2",
        highlightthickness=1, highlightbackground=BORDER,
        height=height,
    )


def section_label(parent, text: str, grid_opts=None):
    """Accented section header with a left-side colour bar."""
    f = tk.Frame(parent, bg=PANEL)
    tk.Frame(f, bg=ACCENT, width=4, height=16).pack(side="left", padx=(0, 8), pady=2)
    tk.Label(f, text=text.upper(), fg=DIM, bg=PANEL,
             font=("Courier", 15, "bold")).pack(side="left")
    if grid_opts is not None:
        f.grid(**grid_opts)
    else:
        f.pack(fill="x", padx=16, pady=(14, 6))
    return f


def hsep(parent):
    """Thin horizontal separator line."""
    tk.Frame(parent, bg=BORDER, height=1).pack(fill="x", padx=14, pady=6)
