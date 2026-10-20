# Vision-Guided Laser Pointer

A camera-based pan–tilt laser pointer built with **Python, OpenCV, and an ESP32**. The computer-vision program detects a red laser spot in a camera feed and communicates with an ESP32 over serial to control two servos. Click a point in the camera window to aim at it, then let the visual feedback loop refine the alignment.

> [!WARNING]
> Never point a laser at eyes, people, vehicles, aircraft, or reflective surfaces. Use a low-power laser module in a controlled test area. This is a hobby prototype, not a certified pointing or safety system.

## Features

- Red laser-spot detection using colour thresholds in BGR and HSV colour spaces.
- Two-axis pan–tilt control through an ESP32 and two servo motors.
- Click-to-aim interaction in the live camera preview.
- Closed-loop alignment based on pixel error and configurable servo correction gains.
- Randomized calibration to associate detected image coordinates with servo angles.
- Initial angle prediction using inverse-distance weighting of the four nearest calibration samples.
- Ability to save successful alignments as additional calibration samples.
- Manual servo nudging and reset controls.

## How it works

1. The camera captures a frame and OpenCV searches for a red laser spot.
2. The user clicks a target point in the camera preview.
3. If calibration data exists, Python predicts initial pan and tilt angles for that point.
4. Python sends a serial command to the ESP32.
5. The ESP32 moves the servos and emits a status response.
6. OpenCV measures the pixel distance between the target and detected laser spot. The controller applies small corrections until the spot remains within the configured deadband for the required number of frames.

```text
Camera ──> Python + OpenCV ──USB serial──> ESP32
   ^                                      │
   │                                      ├── Pan servo
   └──────── observe laser spot ──────────└── Tilt servo
```

## Hardware

- ESP32 development board
- Two servo motors and a pan–tilt mount
- USB webcam or other camera connected to the computer
- Low-power red laser module mounted to the pan–tilt mechanism
- Suitable servo power supply and wiring

**Power:** Do not assume the ESP32 board can safely supply the servos. Use a suitable external supply sized for the servos' current requirements, and connect grounds appropriately. Confirm component voltage and current ratings before wiring.

### Pin assignments

| Component | ESP32 connection |
|---|---|
| Pan servo signal | GPIO 18 |
| Tilt servo signal | GPIO 19 |

The firmware starts at pan **92°** and tilt **60°**. The sketch clamps angles to 0–180°, but your mechanism may have a narrower safe range. Adjust the limits to prevent binding or damage.

## Repository structure

```text
Laser-Pointer/
├── README.md
├── requirements.txt
├── .gitignore
├── opencv/
│   └── laser_pointer.py
└── esp32/
    └── laser_pointer.ino
```

The runtime calibration file, `calibration.json`, is generated locally and intentionally excluded from version control because calibration depends on each physical setup.

## Software requirements

- Python 3
- OpenCV (`opencv-python`)
- NumPy
- pySerial
- Arduino IDE or another ESP32-compatible build environment
- ESP32 Arduino core
- `ESP32Servo` Arduino library

## Setup

### 1. Wire the hardware

- Connect the pan servo signal to GPIO 18.
- Connect the tilt servo signal to GPIO 19.
- Power the servos from a suitable supply and make the required ground connections.
- Position the camera so it can see the laser spot across the intended working area.
- Keep the laser directed at a safe, non-reflective test surface.

### 2. Upload the ESP32 firmware

1. Install the ESP32 board support package and the **ESP32Servo** library.
2. Open `esp32/laser_pointer.ino` in the Arduino IDE.
3. Select the correct board and serial port.
4. Upload the sketch.
5. Close the Arduino Serial Monitor before running Python, because another program may hold the serial port open.

The firmware uses **115200 baud** and sends `READY` after startup.

### 3. Install Python dependencies

Run these commands from the repository root. A virtual environment is recommended.

```bash
python -m venv .venv
```

Activate it on Windows PowerShell:

```powershell
.venv\Scripts\Activate.ps1
```

Activate it on Linux/macOS:

```bash
source .venv/bin/activate
```

Install dependencies:

```bash
python -m pip install -r requirements.txt
```

### 4. Configure the Python script

Open `opencv/laser_pointer.py` and verify the settings near the top:

| Setting | Current value | Purpose |
|---|---|---|
| `SERIAL_PORT` | `COM7` | ESP32 serial port; change to match your computer |
| `BAUD_RATE` | `115200` | Must match the ESP32 firmware |
| `CAMERA_INDEX` | `0` | Camera index |
| `CALIBRATION_FILE` | `calibration.json` | Local calibration file |
| `START_PAN` | `92.0` | Pan reset position |
| `START_TILT` | `60.0` | Tilt reset position |

The supplied script opens the camera using `cv2.CAP_MSMF`, a Windows Media Foundation backend. On Linux or macOS, use a supported backend (for example, `cv2.CAP_V4L2` on many Linux systems) or let OpenCV choose the backend automatically.

### 5. Run the controller

From the repository root:

```bash
python opencv/laser_pointer.py
```

The camera preview opens and the controller sends the servos to the starting position. Ensure the target surface, camera, ESP32, and servos are ready before beginning calibration or aiming.

## Controls

Use the keys while the OpenCV preview window has focus.

| Key / action | Function |
|---|---|
| Left mouse click | Select a target in the camera image |
| `G` | Generate a random calibration map |
| `Y` | Save a successful alignment as a calibration sample |
| `C` | Cancel the current target and reset the servos |
| `K` | Delete the saved calibration map |
| `A` / `D` | Nudge pan by one degree |
| `W` / `S` | Nudge tilt by one degree |
| `Esc` | Exit the program |

## Calibration and alignment

Press **G** to start automatic calibration. The script attempts to record 30 points and allows up to 150 attempts. It moves the servos to randomized angles around the starting position, detects visible laser spots, and stores each spot's image coordinates alongside the corresponding pan and tilt angles. Calibration data is written to `calibration.json` when enough points have been collected.

When a target is clicked, the script estimates starting angles from the four closest calibration samples using inverse-square-distance weights. The closed-loop controller then corrects the difference between the target and the detected laser spot. Alignment is considered stable when the spot falls within the configured pixel deadband for several consecutive frames. Press **Y** to add the successful alignment to the calibration map.

Calibration quality depends on camera position, lighting, focus, laser visibility, servo backlash, mechanical alignment, and the usable scene. Recalibrate if the camera or mount moves.

## Serial protocol

Python and the ESP32 communicate over USB serial at **115200 baud**. Each command ends with a newline.

| Command | Meaning | ESP32 response |
|---|---|---|
| `M,<pan>,<tilt>` | Move servos to angle values in degrees | `MOVED PAN=<pan> TILT=<tilt>` |
| `RESET` | Return to starting position | `RESET PAN=92 TILT=60` |

At startup the ESP32 sends `READY` and `START PAN=92 TILT=60`. Requested movement angles are constrained to 0–180° in the firmware.

## Tuning

The Python script exposes settings for servo direction, correction gains, maximum correction per update, pixel deadband, number of stable frames, and correction interval.

- If the laser moves farther from the target after a correction, verify `PAN_DIRECTION` and `TILT_DIRECTION`.
- If the controller oscillates, reduce the gains or maximum correction.
- If alignment is slow, cautiously adjust the gains or correction interval.
- If the spot is not detected consistently, tune the thresholds and contour filters inside `detect_laser()` and improve scene contrast.

Change one parameter at a time and test on a safe target.

## Troubleshooting

**Python cannot connect to the ESP32**
- Confirm `SERIAL_PORT` is correct.
- Close Arduino Serial Monitor and other serial applications.
- Check that the USB cable supports data and the relevant driver is installed.

**The camera does not open**
- Check `CAMERA_INDEX` and ensure another application is not using the camera.
- Replace `cv2.CAP_MSMF` with a backend appropriate for your operating system if needed.

**The laser spot is not detected**
- Make sure the spot is in frame and clearly visible.
- Reduce red clutter and reflective surfaces in the background.
- Tune the colour thresholds and contour filters in `detect_laser()`.

**Servos jitter, stall, or reset the ESP32**
- Use a power supply appropriate for the servos.
- Check wiring, common ground, mechanical binding, and angle limits.

**Click-to-aim is inaccurate**
- Keep the camera and mount fixed during calibration and operation.
- Generate calibration points and save good alignments with `Y`.
- Recalibrate after moving the camera or changing the lens, resolution, or mount.

## Limitations

- The red-spot detector uses colour and contour heuristics rather than a trained model; other red objects and lighting can cause false detections.
- Calibration is specific to the camera view and physical setup.
- The camera backend and serial port defaults are platform-specific.
- Mechanical backlash and camera perspective affect mapping accuracy.
- The project should be tested on the actual hardware after any changes; successful Python syntax checks alone do not validate hardware behaviour.

## License

No license is included. Until you choose a license and add a `LICENSE` file, others do not automatically receive permission to reuse, modify, or redistribute the project. Add a license when you decide how you want the project to be shared.
