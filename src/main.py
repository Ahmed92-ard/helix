"""
main.py — DUAL-ARM CTRL entry point (Unified Edition)

Usage:
    python3 main.py
"""

import os
import sys

# Ensure src directory is in sys.path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def main():
    missing = []
    for pkg, install in [("serial", "pyserial"), ("PIL", "pillow")]:
        try:
            __import__(pkg)
        except ImportError:
            missing.append(install)
    if missing:
        print(f"Missing packages: {', '.join(missing)}")
        print(f"Run:  pip install {' '.join(missing)}")
        sys.exit(1)

    import tkinter as tk
    from gui import DualArmGUI

    root = tk.Tk()
    app = DualArmGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
