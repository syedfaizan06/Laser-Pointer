# Vision-Guided Laser Pointer

A **Python, OpenCV, and ESP32** project that detects a red laser spot through a camera and uses two servos to aim at a selected point.

> [!WARNING]
> Never point a laser at eyes, people, vehicles, or aircraft. Test with a low-power laser in a controlled area.

## Features
- Real-time red laser detection using OpenCV
- Click-to-aim targeting
- ESP32 pan–tilt servo control
- Automatic calibration and angle prediction
- Closed-loop alignment correction

## Hardware
- ESP32 development board
- Two servos and a pan–tilt mount
- USB camera
- Low-power red laser module
- Suitable external servo power supply

| Servo | ESP32 pin |
|---|---|
| Pan | GPIO 18 |
| Tilt | GPIO 19 |

Starting position: pan **92°**, tilt **60°**. Use a suitable external power supply for the servos.

## Setup

1. Upload `esp32/laser_pointer.ino` using Arduino IDE with the **ESP32Servo** library installed.
2. Install Python dependencies:

   ```bash
   python -m pip install -r requirements.txt
   ```

3. Check `SERIAL_PORT` (default `COM7`) and `CAMERA_INDEX` in `opencv/laser_pointer.py`.
4. Run from the repository root:

   ```bash
   python opencv/laser_pointer.py
   ```

The Python script currently uses the Windows camera backend. Change it if needed for your operating system. Close Arduino Serial Monitor before running the script.

## Controls

| Input | Action |
|---|---|
| Left click | Select target |
| `G` | Generate calibration map |
| `Y` | Save successful alignment |
| `C` | Cancel and reset |
| `K` | Delete calibration |
| `A` / `D` | Nudge pan |
| `W` / `S` | Nudge tilt |
| `Esc` | Exit |

## Calibration

Press `G` to collect samples at different servo angles. Data is saved to `calibration.json`. Click a target to estimate initial angles from calibration data; visual feedback then refines alignment. Recalibrate if the camera or mount moves.

## Serial Commands

Communication uses **115200 baud**:
- `M,<pan>,<tilt>` — Move the servos.
- `RESET` — Return to pan 92° and tilt 60°.

## Notes

Detection uses colour thresholds and contour filtering, so lighting and other red objects can affect results. Camera position and servo mechanics also affect accuracy. Test changes on the actual hardware.

## License

No license has been added yet. Add a `LICENSE` file if you want to define how others may reuse the project.
