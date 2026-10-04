"""
calibrate_dye_color.py — Interactive HSV color range calibration for brown dye

INSTRUCTIONS:
1. Run this script: python calibrate_dye_color.py
2. Point your camera at the brown dye object
3. Use the trackbars to adjust HSV ranges until dye is BRIGHT WHITE in the mask
4. Print the displayed values when you get a good range
5. Copy those values to config.py as DYE_LOWER and DYE_UPPER

TIPS:
- Start with: DYE_LOWER = (5, 50, 50), DYE_UPPER = (20, 255, 200)
- Brown is in the orange-red hue range (H: 5-20)
- Adjust saturation if picking up too much background
- Adjust value (brightness) range if dye is too dark or too light
"""

import os
import sys

# Ensure parent src directory is in sys.path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2
import numpy as np
from config import CAM_INDEX

cap = cv2.VideoCapture(CAM_INDEX)

# Create trackbars for HSV range
def nothing(x):
    pass

cv2.namedWindow('Dye Color Calibration')
cv2.createTrackbar('H_min', 'Dye Color Calibration', 5, 180, nothing)
cv2.createTrackbar('S_min', 'Dye Color Calibration', 50, 255, nothing)
cv2.createTrackbar('V_min', 'Dye Color Calibration', 50, 255, nothing)
cv2.createTrackbar('H_max', 'Dye Color Calibration', 20, 180, nothing)
cv2.createTrackbar('S_max', 'Dye Color Calibration', 255, 255, nothing)
cv2.createTrackbar('V_max', 'Dye Color Calibration', 200, 255, nothing)

print("\n" + "="*60)
print("BROWN DYE COLOR CALIBRATION")
print("="*60)
print("INSTRUCTIONS:")
print("  1. Move the brown dye object in front of camera")
print("  2. Adjust trackbars until dye appears WHITE in the mask")
print("  3. Watch the values at the bottom of the image")
print("  4. Press 'q' to quit and see final values")
print("  5. Copy the DYE_LOWER and DYE_UPPER values to config.py")
print("="*60 + "\n")

while True:
    ret, frame = cap.read()
    if not ret:
        print("ERROR: Cannot read from camera")
        break

    # Get trackbar values
    h_min = cv2.getTrackbarPos('H_min', 'Dye Color Calibration')
    s_min = cv2.getTrackbarPos('S_min', 'Dye Color Calibration')
    v_min = cv2.getTrackbarPos('V_min', 'Dye Color Calibration')
    h_max = cv2.getTrackbarPos('H_max', 'Dye Color Calibration')
    s_max = cv2.getTrackbarPos('S_max', 'Dye Color Calibration')
    v_max = cv2.getTrackbarPos('V_max', 'Dye Color Calibration')

    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])

    # Convert to HSV and mask
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)

    # Apply morphology
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=3)

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Draw original frame with zones
    display = frame.copy()

    # Draw biggest contour
    if contours:
        c = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(c)
        ((cx, cy), radius) = cv2.minEnclosingCircle(c)
        if radius >= 12:
            cv2.circle(display, (int(cx), int(cy)), int(radius), (0, 255, 0), 2)
            cv2.circle(display, (int(cx), int(cy)), 3, (0, 255, 0), -1)

    # Add text info
    cv2.putText(display, f"H: {h_min}-{h_max}  S: {s_min}-{s_max}  V: {v_min}-{v_max}",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.putText(display, "Adjust trackbars until dye is WHITE in mask window  [Press 'q' to exit]",
                (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

    cv2.imshow('Dye Color Calibration', np.hstack([display, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)]))

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        print("\n" + "="*60)
        print("CALIBRATION COMPLETE!")
        print("="*60)
        print(f"\nAdd these to config.py:\n")
        print(f"DYE_LOWER = ({h_min}, {s_min}, {v_min})")
        print(f"DYE_UPPER = ({h_max}, {s_max}, {v_max})")
        print("\n" + "="*60 + "\n")
        break

cap.release()
cv2.destroyAllWindows()
