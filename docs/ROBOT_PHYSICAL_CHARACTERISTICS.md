# Robot Physical Characteristics and Raspberry Pi Pin Connections

This document summarizes the robot's physical characteristics and the current Raspberry Pi pin connections used for motor control and encoder feedback.

---

## Physical Characteristics

### Chassis/Base (URDF)

| Parameter | Value | Notes |
|-----------|-------|-------|
| Base length | 0.179 m | X-axis dimension |
| Base width | 0.165 m | Y-axis dimension |
| Base height | 0.022 m | Z-axis dimension |
| Base mass | 5.0 kg | URDF placeholder value |
| Base origin | base_link | Centered at chassis midpoint, elevated by wheel radius |

### Wheels (URDF + Measured Values)

| Parameter | Value | Notes |
|-----------|-------|-------|
| Wheel radius | 0.032375 m | 32.375 mm |
| Wheel diameter | 0.06475 m | 64.75 mm |
| Wheel width | 0.058 m | 58 mm |
| Wheel base | 0.165 m | Distance between wheel centers |
| Wheel circumference | 0.2034 m | π × diameter |

### Encoders (DG01D-E)

| Parameter | Value | Notes |
|-----------|-------|-------|
| Motor type | DG01D-E | Geared DC motor with Hall effect encoders |
| Gear ratio | 1:48 | Internal gearbox |
| Encoder PPR | 12 | Pulses per revolution (before gearing) |
| Ticks per revolution | 576 | 12 PPR × 48 gear ratio |
| Distance per tick | 0.353 mm | Wheel circumference / 576 |

---

## Raspberry Pi Pin Connections

### Motor Driver (L298N) Control Pins

All pins use BCM GPIO numbering.

#### Motor A (Left Wheel)

| Function | BCM GPIO | Physical Pin | L298N Connection |
|----------|----------|--------------|------------------|
| Enable (PWM) | GPIO 17 | Pin 11 | ENA |
| Direction 1 | GPIO 27 | Pin 13 | IN1 |
| Direction 2 | GPIO 22 | Pin 15 | IN2 |

#### Motor B (Right Wheel)

| Function | BCM GPIO | Physical Pin | L298N Connection |
|----------|----------|--------------|------------------|
| Enable (PWM) | GPIO 13 | Pin 33 | ENB |
| Direction 1 | GPIO 19 | Pin 35 | IN3 |
| Direction 2 | GPIO 26 | Pin 37 | IN4 |

### Encoder Inputs (Quadrature)

#### Motor A (Left Wheel)

| Signal | BCM GPIO | Physical Pin |
|--------|----------|--------------|
| Encoder H1 (Channel A) | GPIO 23 | Pin 16 |
| Encoder H2 (Channel B) | GPIO 24 | Pin 18 |

#### Motor B (Right Wheel)

| Signal | BCM GPIO | Physical Pin |
|--------|----------|--------------|
| Encoder H1 (Channel A) | GPIO 25 | Pin 22 |
| Encoder H2 (Channel B) | GPIO 5 | Pin 29 |

### Encoder Power

| Connection | Physical Pin |
|------------|--------------|
| +5V | Pin 2 or Pin 4 |
| GND | Pin 6 or Pin 9 |

---

## Configuration Files

These values are configured in the following files:

### `config/motor_controller.yaml`

```yaml
motor_controller:
  ros__parameters:
    wheel_base: 0.165
    wheel_diameter: 0.06475
    encoder_ticks_per_rev: 576
    motor_enable_a_pin: 17
    motor_in1_pin: 27
    motor_in2_pin: 22
    motor_enable_b_pin: 13
    motor_in3_pin: 19
    motor_in4_pin: 26
    encoder_left_a_pin: 23
    encoder_left_b_pin: 24
    encoder_right_a_pin: 25
    encoder_right_b_pin: 5
```

### `urdf/robot_core.xacro`

```xml
<xacro:property name="base_width" value="0.165"/>
<xacro:property name="base_length" value="0.179"/>
<xacro:property name="base_height" value="0.022"/>
<xacro:property name="wheel_radius" value="0.032375"/>
<xacro:property name="wheel_width" value="0.058"/>
```

---

## Notes

- The GPIO mapping reflects the working configuration for the L298N motor driver and DG01D-E encoders.
- Physical motors are wired opposite each other; software inversion is used for correct forward motion.
- All parameters are fully configurable via ROS 2 parameters without code changes.

---

## Wiring Diagram Reference

```
Raspberry Pi 4                     L298N Motor Driver
─────────────                     ──────────────────
GPIO 17 (Pin 11) ──────────────── ENA (Motor A PWM)
GPIO 27 (Pin 13) ──────────────── IN1 (Motor A Dir 1)
GPIO 22 (Pin 15) ──────────────── IN2 (Motor A Dir 2)
GPIO 13 (Pin 33) ──────────────── ENB (Motor B PWM)
GPIO 19 (Pin 35) ──────────────── IN3 (Motor B Dir 1)
GPIO 26 (Pin 37) ──────────────── IN4 (Motor B Dir 2)
GND (Pin 6/9)    ──────────────── GND

Raspberry Pi 4                     DG01D-E Encoders
─────────────                     ─────────────────
GPIO 23 (Pin 16) ──────────────── Left Encoder H1
GPIO 24 (Pin 18) ──────────────── Left Encoder H2
GPIO 25 (Pin 22) ──────────────── Right Encoder H1
GPIO 5  (Pin 29) ──────────────── Right Encoder H2
+5V (Pin 2/4)    ──────────────── Encoder VCC
GND (Pin 6/9)    ──────────────── Encoder GND
```
