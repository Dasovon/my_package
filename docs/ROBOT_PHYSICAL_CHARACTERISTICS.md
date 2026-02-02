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
- **Encoder resolution:** 576 ticks per wheel revolution (12 PPR × 48 gear ratio)
- **Distance per tick:** 0.353 mm

## Raspberry Pi Pin Connections

### Motor Driver (L298N) Control Pins (BCM GPIO)

**Motor A (Left Wheel):**
- Enable (PWM): GPIO 17 — Physical pin 11 → L298N ENA
- Direction 1: GPIO 27 — Physical pin 13 → L298N IN1
- Direction 2: GPIO 22 — Physical pin 15 → L298N IN2

**Motor B (Right Wheel):**
- Enable (PWM): GPIO 13 — Physical pin 33 → L298N ENB
- Direction 1: GPIO 19 — Physical pin 35 → L298N IN3
- Direction 2: GPIO 26 — Physical pin 37 → L298N IN4

### Encoder Inputs (Quadrature)

**Motor A (Left Wheel):**
- Encoder H1: GPIO 23 — Physical pin 16
- Encoder H2: GPIO 24 — Physical pin 18

**Motor B (Right Wheel):**
- Encoder H1: GPIO 25 — Physical pin 22
- Encoder H2: GPIO 5 — Physical pin 29

### Encoder Power
- **+5V:** Physical pins 2 or 4
- **GND:** Physical pins 6 or 9

## Notes
- The GPIO mapping above reflects the working test configuration for the L298N motor driver and DG01D-E encoders.
- Physical motors are wired opposite each other; software inversion is used for correct forward motion.
