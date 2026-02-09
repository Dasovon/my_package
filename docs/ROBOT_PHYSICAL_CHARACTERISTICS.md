# Robot Physical Characteristics and Raspberry Pi Pin Connections

This document summarizes the robot's physical characteristics and the current Raspberry Pi pin connections used for motor control and encoder feedback.

## Physical Characteristics

### Chassis/Base (URDF)
- **Base length:** 0.179 m
- **Base width:** 0.165 m
- **Base height:** 0.022 m
- **Base mass (URDF placeholder):** 5.0 kg
- **Base origin:** `base_link` centered at the chassis midpoint, elevated by wheel radius relative to `base_footprint`.

### Wheels (URDF + measured values)
- **Wheel radius:** 0.032375 m
- **Wheel diameter:** 0.06475 m
- **Wheel width:** 0.058 m
- **Wheel base (distance between wheels):** 0.165 m
- **Wheel circumference:** 0.2034 m

### Encoders (DG01D-E)
- **Encoder resolution:** 288 ticks per wheel revolution (3 pulses × 2 edges × 48:1 gear ratio)
- **Distance per tick:** 0.706 mm
- **Decoding method:** Interrupt-based quadrature via `GPIO.add_event_detect()`

## Raspberry Pi Pin Connections

### Motor Driver (L298N) Control Pins (BCM GPIO)

All pins are declared as ROS 2 parameters in `motor_controller.py` (with physical pin defaults) and overridden via `config/motor_controller.yaml` with BCM GPIO numbers. The code uses `GPIO.setmode(GPIO.BCM)`.

**Motor A (Left Wheel):**
- Enable (PWM): BCM 17 — Physical pin 11 → L298N ENA (`motor_enable_a_pin`)
- Direction 1: BCM 27 — Physical pin 13 → L298N IN1 (`motor_in1_pin`)
- Direction 2: BCM 22 — Physical pin 15 → L298N IN2 (`motor_in2_pin`)

**Motor B (Right Wheel):**
- Enable (PWM): BCM 13 — Physical pin 33 → L298N ENB (`motor_enable_b_pin`)
- Direction 1: BCM 19 — Physical pin 35 → L298N IN3 (`motor_in3_pin`)
- Direction 2: BCM 26 — Physical pin 37 → L298N IN4 (`motor_in4_pin`)

### Encoder Inputs (Interrupt-based Quadrature)

Encoders use `GPIO.add_event_detect()` on both channels for accurate quadrature decoding with a state machine lookup table.

**Motor A (Left Wheel):**
- Encoder H1: BCM 23 — Physical pin 16 (`encoder_left_a_pin`)
- Encoder H2: BCM 24 — Physical pin 18 (`encoder_left_b_pin`)

**Motor B (Right Wheel):**
- Encoder H1: BCM 25 — Physical pin 22 (`encoder_right_a_pin`)
- Encoder H2: BCM 5 — Physical pin 29 (`encoder_right_b_pin`)

### Encoder Configuration
- `encoder_ticks_per_rev`: 288 (3 pulses × 2 edges × 48:1 gear ratio)
- `encoder_left_inverted`: True (compensates for mirrored left motor mount)
- `encoder_right_inverted`: False
- `encoder_bouncetime_ms`: 1

### Encoder Power
- **+5V:** Physical pins 2 or 4
- **GND:** Physical pins 6 or 9

## Notes
- The GPIO mapping above reflects the working configuration for the L298N motor driver and DG01D-E encoders.
- Physical motors are wired opposite each other; the left motor direction is inverted in software (`set_motor_a()`) and the left encoder is inverted via `encoder_left_inverted: True`.
- All pin assignments can be reconfigured via ROS 2 parameters without code changes.
