"""
stations.py — Station Manager

Handles persistent load/save of pick-and-place station poses (above + pick)
from stations.json.  All editable defaults live in config.py.
"""

import json
import os

from config import (
    STATIONS_FILE,
    DEFAULT_STATIONS,
    GRIPPER_OPEN,
    GRIPPER_CLOSED,
)


class StationManager:
    def __init__(self, stations_file=None, default_stations=None, gripper_open=None, gripper_closed=None):
        self.stations_file = stations_file or STATIONS_FILE
        self.default_stations = default_stations or DEFAULT_STATIONS
        self.gripper_open = gripper_open if gripper_open is not None else GRIPPER_OPEN
        self.gripper_closed = gripper_closed if gripper_closed is not None else GRIPPER_CLOSED
        self.stations: dict = {}
        self._load()

    # ── Persistence ───────────────────────────────────────
    def _load(self):
        if os.path.exists(self.stations_file):
            try:
                with open(self.stations_file) as f:
                    raw = json.load(f)
                migrated = False
                for name, st in raw.items():
                    for ptype in ("above", "pick"):
                        pose = st.get(ptype, [0] * 6)
                        if len(pose) < 6:
                            pose = pose + [0] * (6 - len(pose))
                        st[ptype] = pose[:6]
                    # Enforce gripper convention
                    if st["above"][5] != self.gripper_open:
                        st["above"][5] = self.gripper_open
                        migrated = True
                    if st["pick"][5] != self.gripper_closed:
                        st["pick"][5] = self.gripper_closed
                        migrated = True
                self.stations = raw
                if migrated:
                    self._save()
                return
            except Exception:
                pass
        # Fall back to defaults
        self.stations = {k: dict(v) for k, v in self.default_stations.items()}
        self._save()

    def _save(self):
        with open(self.stations_file, "w") as f:
            json.dump(self.stations, f, indent=2)

    # ── Read ──────────────────────────────────────────────
    def names(self) -> list:
        return list(self.stations.keys())

    def get(self, name: str) -> dict | None:
        return self.stations.get(name)

    # ── Write ─────────────────────────────────────────────
    def set_above(self, name: str, pose: list):
        if name not in self.stations:
            self.stations[name] = {"above": pose[:], "pick": pose[:]}
        else:
            self.stations[name]["above"] = pose[:]
        self._save()

    def set_pick(self, name: str, pose: list):
        if name not in self.stations:
            self.stations[name] = {"above": pose[:], "pick": pose[:]}
        else:
            self.stations[name]["pick"] = pose[:]
        self._save()

    def delete(self, name: str):
        if name in self.stations:
            del self.stations[name]
            self._save()

    def rename(self, old: str, new: str) -> bool:
        if old in self.stations and new not in self.stations:
            self.stations[new] = self.stations.pop(old)
            self._save()
            return True
        return False
