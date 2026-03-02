# Helix

## Overview
A 5-DOF servo robotic arm monitored by a computer-vision verification system.
This project uses a live camera feed to detect an object, confirm transfers between workstations, and automatically count successful picks and completed handoff cycles using a graphical dashboard.

The camera does not control the robot. It independently verifies whether the robot successfully transferred the object.

## Features
- Live OpenCV video feed inside a Tkinter GUI
- HSV color-based object detection (orange ball)
- Zone-based monitoring (Station A ↔ Station B)
- Automatic pick counting
- Full cycle detection
- Timestamped event log
- FPS monitoring
- Session runtime timer
- Ball detection status indicator

## Control Architecture
The robotic arm motion is pre-programmed on the Arduino.
The vision system is a monitoring and validation tool, not a controller.

### System behavior:
1. Arduino executes a fixed pick-and-place routine.
2. Camera observes the workspace.
3. Software detects the ball position.
4. Transfers between zones are confirmed.
5. Picks and cycles are recorded automatically.

## What the Counters Mean
**Pick:**
the object moves between zones (A → B or B → A)

**Cycle:**
a complete transfer and return: A → B → A. 
This measures real operational reliability of the robotic arm.

## Hardware Used
| Component | 
| --- | 
| 5 DOF Servo Robotic Arm | 
| Claw Gripper | 
| Arduino Uno |
| External servo power supply |
| Orange colored ball (tracked object) |

## Installation (First Time Setup)
```
git clone https://github.com/Ahmed92-ard/helix.git
cd helix/pickplace_tracker
sudo apt install python3-tk
```

Create a virtual environment:
```
python3 -m venv venv
```
activate virtual environment:
```
source venv/bin/activate
```
dependencies installation:
```
pip install -r requirements.txt
```
to run tracker:
```
python3 pickplace_tracker.py
```
### Running Later in New Terminals
do these each time you open a new terminal session:
```
cd helix/pickplace_tracker
source venv/bin/activate
python3 pickplace_tracker.py
```

## Camera Troubleshooting Tips if issues arise with camera window display: 
to change camera index in script: open `pickplace_tracker.py` and modify `CAMERA_INDEX = 0` to `CAMERA_INDEX = 1` if necessary, as some systems assign webcams as device 1 instead of 0.
