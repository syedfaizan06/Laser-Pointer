import cv2
import numpy as np
import serial
import time
import json
import os
import math
import random

# ============================================================
# SETTINGS
# ============================================================

SERIAL_PORT = "COM7" # Ensure this matches your ESP32
BAUD_RATE = 115200
CAMERA_INDEX = 0
CALIBRATION_FILE = "calibration.json"

# ============================================================
# START / RESET POSITION
# ============================================================

START_PAN = 92.0
START_TILT = 60.0

# ============================================================
# SERVO DIRECTION
# ============================================================
PAN_DIRECTION = 1
TILT_DIRECTION = -1

# ============================================================
# CLOSED LOOP SETTINGS
# ============================================================

PAN_GAIN = 0.006
TILT_GAIN = 0.006
MAX_CORRECTION = 0.8
DEADBAND_PIXELS = 8
STABLE_FRAMES_REQUIRED = 4
INITIAL_SETTLE_TIME = 0.8
CORRECTION_INTERVAL = 0.12

# ============================================================
# SERIAL CONNECTION
# ============================================================

print("\n======================================")
print("Connecting to ESP32...")
print("======================================")

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
except Exception as e:
    print("\nERROR: Could not connect to ESP32.")
    print("Check: ESP32 is connected, COM port is correct, Serial Monitor is CLOSED.")
    raise SystemExit

time.sleep(2)
print("ESP32 connected.")

# ============================================================
# CAMERA (Untouched to preserve resolution)
# ============================================================

print("\nOpening camera...")
cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_MSMF)

if not cap.isOpened():
    print("\nERROR: Camera could not be opened.")
    ser.close()
    raise SystemExit

actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f"Camera resolution: {actual_width} x {actual_height}")
print("Camera opened.")

# ============================================================
# GLOBAL VARIABLES
# ============================================================

current_pan = START_PAN
current_tilt = START_TILT
target_x = None
target_y = None
laser_x = None
laser_y = None
target_active = False
aligned = False
stable_frames = 0
last_correction_time = 0
calibration_samples = []

# ============================================================
# SEND SERVO COMMAND
# ============================================================

def move_servos(pan, tilt):
    global current_pan, current_tilt

    # Safety limits
    pan = max(0.0, min(180.0, float(pan)))
    tilt = max(0.0, min(180.0, float(tilt)))

    current_pan = pan
    current_tilt = tilt

    command = f"M,{round(pan)},{round(tilt)}\n"
    try:
        ser.write(command.encode())
    except Exception as e:
        print("Serial error:", e)
        return
    print(f"SERVO -> PAN={round(pan)} TILT={round(tilt)}")

# ============================================================
# RESET POSITION
# ============================================================

def reset_position():
    global current_pan, current_tilt
    current_pan = START_PAN
    current_tilt = START_TILT
    try:
        ser.write(b"RESET\n")
    except:
        pass
    time.sleep(0.5)
    print("RESET -> PAN=92 TILT=60")

# ============================================================
# LASER DETECTION
# ============================================================

def detect_laser(frame):
    b, g, r = cv2.split(frame)
    red_dominant = ((r > 150) & (r > g + 25) & (r > b + 20))
    red_mask = np.zeros_like(r)
    red_mask[red_dominant] = 255

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    red1 = cv2.inRange(hsv, np.array([0, 50, 100]), np.array([15, 255, 255]))
    red2 = cv2.inRange(hsv, np.array([165, 30, 100]), np.array([179, 255, 255]))
    hsv_red = cv2.bitwise_or(red1, red2)

    mask = cv2.bitwise_or(red_mask, hsv_red)
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best_candidate = None
    best_score = -1
    frame_h, frame_w = frame.shape[:2]

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < 1 or area > 5000:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        if w <= 0 or h <= 0:
            continue

        M = cv2.moments(contour)
        if M["m00"] == 0:
            continue

        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])

        x1, y1 = max(0, x), max(0, y)
        x2, y2 = min(frame_w, x + w), min(frame_h, y + h)
        roi = frame[y1:y2, x1:x2]
        if roi.size == 0:
            continue

        mean_b, mean_g, mean_r = np.mean(roi[:, :, 0]), np.mean(roi[:, :, 1]), np.mean(roi[:, :, 2])
        red_strength = mean_r - max(mean_g, mean_b)

        perimeter = cv2.arcLength(contour, True)
        if perimeter <= 0:
            continue
        circularity = (4 * math.pi * area) / (perimeter * perimeter)

        if mean_r < 120 or red_strength < 10:
            continue

        score = (red_strength * 2.0) + (circularity * 50.0) + min(area, 100)
        if score > best_score:
            best_score = score
            radius = int(max(w, h) / 2)
            best_candidate = (cx, cy, radius)

    return best_candidate

# ============================================================
# INVERSE DISTANCE WEIGHTING (KNN) PREDICTION
# ============================================================
# This replaces the polynomial model. It looks for the closest
# calibrated points to your click and estimates the angle.

def predict_angles(target_x, target_y, samples):
    if not samples:
        return START_PAN, START_TILT

    # Calculate pixel distance from click to every saved sample
    dists = []
    for s in samples:
        dx = s["x"] - target_x
        dy = s["y"] - target_y
        dist = math.hypot(dx, dy)
        dists.append((dist, s["pan"], s["tilt"]))

    # Sort by closest physical pixels
    dists.sort(key=lambda item: item[0])

    # Grab the 4 closest points
    closest = dists[:4]

    # If we clicked EXACTLY on a sample, prevent divide-by-zero
    if closest[0][0] < 1.0:
        return closest[0][1], closest[0][2]

    weight_sum = 0
    pan_sum = 0
    tilt_sum = 0

    # Calculate weighted average based on inverse distance
    for dist, p, t in closest:
        w = 1.0 / (dist ** 2) 
        weight_sum += w
        pan_sum += p * w
        tilt_sum += t * w

    return pan_sum / weight_sum, tilt_sum / weight_sum

# ============================================================
# LOAD EXISTING CALIBRATION
# ============================================================

if os.path.exists(CALIBRATION_FILE):
    try:
        with open(CALIBRATION_FILE, "r") as f:
            calibration_samples = json.load(f)
        print(f"Existing calibration loaded ({len(calibration_samples)} points).")
    except Exception as e:
        print("Calibration file could not be loaded:", e)
        calibration_samples = []
else:
    print("No calibration file found.")

# ============================================================
# AUTOMATIC RANDOM CALIBRATION (The "Feedback Loop" Approach)
# ============================================================

def automatic_calibration():
    global calibration_samples

    print("\n======================================")
    print("    STARTING RANDOM CALIBRATION MAP")
    print("======================================")
    print("Shooting laser at random valid angles...")
    
    calibration_samples = []
    TARGET_POINTS = 30
    MAX_ATTEMPTS = 150
    valid_points = 0
    attempts = 0

    while valid_points < TARGET_POINTS and attempts < MAX_ATTEMPTS:
        attempts += 1
        
        # Shoot randomly within a ~80 degree arc in front of the camera
        r_pan = random.uniform(START_PAN - 45, START_PAN + 45)
        r_tilt = random.uniform(START_TILT - 30, START_TILT + 40)

        move_servos(r_pan, r_tilt)
        
        # Wait slightly longer to ensure servo settles and camera frame updates
        time.sleep(0.7) 

        ret, frame = cap.read()
        if not ret: continue
        
        display = frame.copy()
        detection = detect_laser(frame)

        if detection is not None:
            lx, ly, radius = detection
            calibration_samples.append({
                "x": int(lx),
                "y": int(ly),
                "pan": float(r_pan),
                "tilt": float(r_tilt)
            })
            valid_points += 1
            print(f"[{valid_points}/{TARGET_POINTS}] Point mapped at X:{lx} Y:{ly}")

            cv2.circle(display, (lx, ly), max(12, radius + 7), (0, 255, 0), 2)
        else:
            print(f"Attempt {attempts} - Laser off-screen, skipping.")

        # Show UI feedback
        cv2.putText(display, f"CALIBRATING: {valid_points}/{TARGET_POINTS}", (30, 45), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
        cv2.putText(display, "ESC = CANCEL", (30, 85), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.imshow("Laser Pan Tilt", display)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            print("Calibration cancelled.")
            reset_position()
            return

    # Save to JSON
    if valid_points > 5:
        try:
            with open(CALIBRATION_FILE, "w") as f:
                json.dump(calibration_samples, f, indent=4)
            print("Calibration complete and saved!")
        except Exception as e:
            print("Could not save calibration:", e)
    else:
        print("Calibration failed: Not enough visible points found.")

    reset_position()
    time.sleep(0.8)

# ============================================================
# MOUSE CALLBACK
# ============================================================

def mouse_callback(event, x, y, flags, param):
    global target_x, target_y, target_active, aligned, stable_frames

    if event != cv2.EVENT_LBUTTONDOWN:
        return

    target_x = x
    target_y = y
    target_active = True
    aligned = False
    stable_frames = 0

    print(f"\n======================================\nTARGET = X:{x} Y:{y}")

    if len(calibration_samples) > 0:
        pan, tilt = predict_angles(x, y, calibration_samples)
        print(f"PREDICTED START -> PAN={pan:.1f} TILT={tilt:.1f}")
        move_servos(pan, tilt)
    else:
        print("No calibration samples. Using current servo position.")

    time.sleep(INITIAL_SETTLE_TIME)
    print("Closed-loop correction active.\n======================================")

# ============================================================
# WINDOW & SETUP
# ============================================================

cv2.namedWindow("Laser Pan Tilt", cv2.WINDOW_NORMAL)
cv2.setMouseCallback("Laser Pan Tilt", mouse_callback)

reset_position()

print("\n======================================")
print("       LASER PAN / TILT SYSTEM")
print("======================================")
print("G = Generate Random Calibration Map")
print("Click = Aim at target")
print("Y = Save successful alignment to map")
print("C = Cancel / reset")
print("K = Delete calibration")
print("A/D/W/S = Manual Nudge")
print("ESC = Exit\n======================================\n")

# ============================================================
# MAIN LOOP
# ============================================================

while True:
    ret, frame = cap.read()
    if not ret: continue
    
    frame_height, frame_width = frame.shape[:2]
    display = frame.copy()

    detection = detect_laser(frame)
    if detection is not None:
        laser_x, laser_y, radius = detection
        laser_x, laser_y = int(laser_x), int(laser_y)

        cv2.circle(display, (laser_x, laser_y), max(12, radius + 7), (0, 255, 255), 2)
        cv2.circle(display, (laser_x, laser_y), 3, (0, 255, 255), -1)
        cv2.putText(display, "LASER", (laser_x + 15, laser_y - 15), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    else:
        laser_x = laser_y = None

    if target_active and target_x is not None and target_y is not None:
        cv2.drawMarker(display, (int(target_x), int(target_y)), (255, 255, 0), cv2.MARKER_CROSS, 30, 2)
        cv2.circle(display, (int(target_x), int(target_y)), DEADBAND_PIXELS, (255, 255, 0), 1)

    if target_active and laser_x is not None and laser_y is not None:
        error_x = target_x - laser_x
        error_y = target_y - laser_y
        distance = math.sqrt(error_x**2 + error_y**2)

        if distance <= DEADBAND_PIXELS:
            stable_frames += 1
            cv2.putText(display, f"STABLE {stable_frames}/{STABLE_FRAMES_REQUIRED}", (30, 145), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            
            if stable_frames >= STABLE_FRAMES_REQUIRED:
                aligned = True
                cv2.putText(display, "ALIGNED - PRESS Y TO SAVE", (30, 185), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        else:
            stable_frames = 0
            aligned = False
            now = time.time()
            if now - last_correction_time >= CORRECTION_INTERVAL:
                pan_correction = PAN_DIRECTION * error_x * PAN_GAIN
                tilt_correction = TILT_DIRECTION * error_y * TILT_GAIN
                
                pan_correction = max(-MAX_CORRECTION, min(MAX_CORRECTION, pan_correction))
                tilt_correction = max(-MAX_CORRECTION, min(MAX_CORRECTION, tilt_correction))

                move_servos(current_pan + pan_correction, current_tilt + tilt_correction)
                last_correction_time = now

        cv2.putText(display, f"Error X={int(error_x)} Y={int(error_y)} D={int(distance)}", 
                    (30, 225), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    cv2.putText(display, f"PAN: {current_pan:.1f}", (30, frame_height - 80), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(display, f"TILT: {current_tilt:.1f}", (30, frame_height - 50), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    if len(calibration_samples) >= 6:
        cv2.putText(display, f"CALIBRATION: {len(calibration_samples)} PTS", 
                    (frame_width - 320, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    else:
        cv2.putText(display, "CALIBRATION: NOT SET", (frame_width - 350, 40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    cv2.imshow("Laser Pan Tilt", display)
    key = cv2.waitKey(1) & 0xFF

    if key == 27: break
    elif key == ord('g'):
        target_active = aligned = False
        automatic_calibration()
    elif key == ord('c'):
        target_active = aligned = False
        target_x = target_y = None
        reset_position()
        print("Target cancelled.")
    elif key == ord('k'):
        calibration_samples = []
        if os.path.exists(CALIBRATION_FILE):
            os.remove(CALIBRATION_FILE)
            print("Calibration deleted.")
    elif key == ord('a'): move_servos(current_pan - 1, current_tilt)
    elif key == ord('d'): move_servos(current_pan + 1, current_tilt)
    elif key == ord('w'): move_servos(current_pan, current_tilt + 1)
    elif key == ord('s'): move_servos(current_pan, current_tilt - 1)
    elif key == ord('y'):
        if target_active and aligned and laser_x is not None:
            print("\nSaving successful alignment...")
            calibration_samples.append({
                "x": int(laser_x), "y": int(laser_y), 
                "pan": float(current_pan), "tilt": float(current_tilt)
            })
            with open(CALIBRATION_FILE, "w") as f:
                json.dump(calibration_samples, f, indent=4)
            print("Calibration map updated.")
            target_active = aligned = False
            target_x = target_y = None

print("\nStopping...")
try: reset_position()
except: pass
cap.release()
ser.close()
cv2.destroyAllWindows()
print("Program stopped.")