# PID Balancing Ball

This project is a ball-balancing platform that uses PID control to keep a ball near the center of a tilting plate, or move it toward a target setpoint. The system combines a 3-RPS parallel mechanism driven by three stepper motors, an OAK camera for ball position tracking, and an Arduino Uno running PID control plus inverse kinematics.

## Reference

The mechanical concept, overall design, and inverse kinematics approach for the 3-RPS mechanism are based on this guide:

- Ball Balancer - Instructables: https://www.instructables.com/Ball-Balancer/

This repository is not a direct copy of the original project. The code has been adapted for this hardware setup, camera pipeline, Serial communication flow, and PID tuning values.

## How It Works

1. The OAK camera captures frames at `1024x836`.
2. A Python script uses OpenCV to threshold the ball color in HSV space.
3. The ball center is calculated from the moment of a valid contour.
4. The ball position is sent over Serial in this format:

   ```text
   x y\r
   ```

5. The Arduino reads the position and calculates the X/Y error.
6. The PID controller converts that error into a plate tilt vector.
7. Inverse kinematics converts the tilt vector into target angles for the three mechanism legs.
8. AccelStepper drives the three stepper motors to the calculated positions.

When the ball is not detected, the firmware moves the platform back to its default balanced position.

## Main Hardware

- Arduino Uno
- 3 stepper motors
- 3 STEP/DIR stepper drivers
- 3-RPS parallel mechanism based on the Ball Balancer design
- Luxonis OAK camera, or another DepthAI-compatible camera
- Computer running Python for image processing and Serial communication
- Suitable power supply for the stepper motors and drivers

## Arduino Pinout

Pins are configured in `src/Setup.h`:

| Function | Pin |
| --- | --- |
| Stepper 1 STEP | 2 |
| Stepper 1 DIR | 3 |
| Stepper 2 STEP | 4 |
| Stepper 2 DIR | 5 |
| Stepper 3 STEP | 6 |
| Stepper 3 DIR | 7 |
| Driver enable | 13 |

The current Serial baud rate is `112500`.

## Repository Structure

```text
.
├── platformio.ini
├── src/
│   ├── main.ino
│   └── Setup.h
├── lib/
│   ├── InverseKinematics/
│   └── Point/
└── oak-webcam/
    ├── oak-webcam.py
    └── sending.py
```

### Arduino Firmware

- `src/main.ino`: main loop, PID control, stepper control, and IK calls.
- `src/Setup.h`: baud rate, timeout, speed limits, and pin mapping.
- `lib/InverseKinematics`: calculates each leg angle for the 3-RPS mechanism.
- `lib/Point`: reads the ball position from Serial.

### Python Vision Pipeline

- `oak-webcam/oak-webcam.py`: reads frames from the OAK camera, thresholds the ball color, finds the contour center, and sends coordinates.
- `oak-webcam/sending.py`: opens the Serial port and sends data to the Arduino.

By default, `sending.py` uses `COM12`. Change this to match the port used by your Arduino.

## Firmware Setup

This is a PlatformIO project targeting Arduino Uno.

```bash
pio run
pio run --target upload
```

Main Arduino libraries:

- `AccelStepper`
- `MultiStepper`
- Internal `InverseKinematics` library
- Internal `Point` library

## Python Setup

Install the required Python packages for the camera and image processing pipeline:

```bash
pip install opencv-python depthai pyvirtualcam numpy pyserial
```

Run the camera pipeline:

```bash
python oak-webcam/oak-webcam.py
```

If the Arduino does not receive position data, check:

- The COM port in `oak-webcam/sending.py`
- Whether the baud rate matches `src/Setup.h`
- Whether the camera detects the ball with the HSV range in `oak-webcam.py`
- Whether the sent data format is `x y\r`

## Calibration

These values usually need adjustment when the hardware changes:

- `Xoffset`, `Yoffset` in `src/main.ino`: image center / platform center.
- `kp`, `ki`, `kd`: PID constants.
- `alpha`, `beta`: additional derivative-related coefficients.
- `MAX_SPEED`, `STEP_RANGE`: stepper configuration.
- `Machine machine(d, e, f, g)`: geometry dimensions of the mechanism.
- HSV threshold and contour area limits in `oak-webcam/oak-webcam.py`.

Recommended calibration order:

1. Check the rotation direction of each stepper motor.
2. Verify that the platform returns to a level home position.
3. Recalculate the image center values `Xoffset` and `Yoffset`.
4. Confirm that ball coordinates are being sent over Serial.
5. Start with small PID values and increase gradually until the system is stable.

## Safety Notes

- Do not power the stepper motors while the mechanism is mechanically blocked.
- Check the mechanical travel range before increasing speed.
- During PID tuning, start with a small tilt range to avoid throwing the ball off the plate.
- Make sure the motor supply and logic supply share a common ground if your circuit requires it.

## License / Attribution

This project uses the design idea and inverse kinematics reference from Ball Balancer on Instructables. Please keep this attribution when reusing or extending the project.
