# Robo Car – Autonomous Mecanum Navigation

A self-navigating mecanum-wheel robot car that avoids obstacles with three ultrasonic sensors. It cruises when the path is open, steers early around obstacles it sees ahead, slides sideways away from walls, and recovers by itself when it gets too close to something or stuck.

The firmware is bare-metal C++ for the **ATmega328PB**, built in **Microchip Studio 7**. It uses no Arduino core and no libraries: timers, UART, GPIO and the motor-driver I2C bus are all driven directly through the chip's registers.

**Team:** Badsha Munna · Sadman Showmik · Jayed Amir

---

## Features

- **Mecanum drive**: independent control of all four wheels gives forward, reverse, sideways sliding and smooth arc turns.
- **Three-way ultrasonic sensing**: front, left and right HC-SR04 sensors with a 30 ms echo timeout. A missing echo is treated as open space (300 cm).
- **Priority-based navigation**: emergency escape, then stuck recovery, then obstacle avoidance, then cruise.
- **Early path planning**: the car starts steering toward the more open side as soon as an obstacle comes within 50 cm in front.
- **Smooth turns**: all four wheels keep spinning during a turn. The outer wheels get +18 PWM and the inner wheels −18 PWM, so the car arcs instead of pivoting.
- **Self-recovery**: when it gets too close or the readings stop changing for 4 s, the car reverses and turns toward the more open side.
- **Serial telemetry**: distances and decisions are printed at 9600 baud for debugging.
- **Bare-metal and small**: about 5.3 KB of flash and 172 bytes of RAM.

## Hardware

| Component | Details |
|---|---|
| Controller | Custom board with ATmega328PB @ 16 MHz (external crystal) |
| Drive | 4 × mecanum wheels with DC gear motors, motor driver board with an STC8G1K08 controller over I2C |
| Sensors | 3 × HC-SR04 ultrasonic (front, left, right) |
| Chassis | Custom chassis with LEGO-compatible holes |
| Power | 18650 Li-ion battery |

### Pin map

| Function | ATmega328PB pin | Board label |
|---|---|---|
| Front sensor | TRIG PB4 / ECHO PB5 | D12 / D13 |
| Left sensor | TRIG PD7 / ECHO PD6 | D7 / D6 |
| Right sensor | TRIG PB0 / ECHO PB3 | D8 / D11 |
| Motor driver I2C | SDA PD3 / SCL PD2 | D3 / D2 |
| Serial debug | TX PD1 (USART0) | D1 |

## Firmware architecture

| Block | Implementation |
|---|---|
| Timebase | Timer0 in CTC mode, 1 ms interrupt → `millis()` |
| Echo timing | Timer1 free-running at 2 MHz (0.5 µs resolution), 30 ms timeout → replaces `pulseIn()` |
| Serial | USART0, 9600 8N1, interrupt-driven 64-byte TX buffer; strings kept in flash (`PSTR`) |
| Motor bus | Bit-banged I2C (~100 kHz) on PD3/PD2 to the STC8G1K08; registers 1–8 hold the PWM duty (0–255) of the four motors |
| GPIO | Direct `PORTx` / `DDRx` / `PINx` access |

Everything is in a single `main.cpp`, written to compile with Microchip Studio's default avr-g++ settings (gnu++98).

## How it works

Every loop the car reads all three sensors, then acts on the first rule that matches:

1. **Emergency**: any sensor ≤ 15 cm → `escapeBack()`.
2. **Stuck**: no reading has changed by 2.5 cm or more for 4 s → `escapeBack()`.
3. **Navigate**: `navigate()` picks a movement (below).
4. Wait 5 ms and repeat.

### `navigate()`

| Situation | Action |
|---|---|
| Front > 50 cm and both sides > 10 cm | Cruise forward at 65 PWM (50 PWM if front ≤ 60 cm) |
| Front 10–50 cm | Early smooth turn toward the more open side at 50 PWM |
| Front ≤ 10 cm | Smooth turn toward the more open side at 45 PWM |
| Left ≤ 10 cm, right clear | Slide right at 45 PWM |
| Right ≤ 10 cm, left clear | Slide left at 45 PWM |
| Both sides tight | Crawl forward at 35 PWM |

"More open side" means: turn right if the right is clear and at least as open as the left (or the left is blocked); otherwise turn left if the left is clear; otherwise crawl forward.

### `escapeBack()`

Reverse for 700 ms at 43 PWM → stop for 150 ms → re-read the sensors → smooth-turn toward the more open side for 450 ms → stop for 150 ms → reset the stuck detector.

## Configuration

All tuning values are `#define`s at the top of `main.cpp`.

| Parameter | Value | Purpose |
|---|---|---|
| `DIST_EMERGENCY` | 15 cm | Immediate back-escape |
| `DIST_AVOID` | 10 cm | Close turn / side slide |
| `DIST_WARN` | 50 cm | Start early steering |
| `DIST_CLEAR` | 60 cm | Full cruise speed |
| `SPEED_CRUISE` | 65 PWM | Open path |
| `SPEED_APPROACH` | 50 PWM | Approaching an obstacle |
| `SPEED_MANEUVER` | 45 PWM | Turning and sliding |
| `SPEED_SLOW` | 35 PWM | Crawling |
| `SPEED_BACK` | 43 PWM | Reversing |
| `TURN_BOOST` | 18 PWM | Speed difference between the two sides in a turn |
| `BACK_DURATION_MS` | 700 ms | Reverse time during escape |
| `TURN_DURATION_MS` | 450 ms | Turn time during escape |
| `STUCK_TIMEOUT_MS` | 4000 ms | Time without change before "stuck" |
| `STUCK_DELTA` | 2.5 cm | Change that counts as movement |
| `MOTOR_I2C_ADDR` | 0x30 | 7-bit I2C address of the motor driver chip |

## Getting started

### Build

1. Install **Microchip Studio 7** (it includes the AVR toolchain and the ATmega328PB device pack).
2. Create a **GCC C++ Executable Project** and select **ATmega328PB** as the device.
3. Replace the generated `main.cpp` with the one from this repository.
4. Build with **F7**. The hex file is written to `Debug/` or `Release/`.

### Flash

- **With a programmer** (Atmel-ICE, MPLAB SNAP, AVRISP mkII): *Tools → Device Programming → Memories → Program*. This overwrites any bootloader.
- **Over USB with an Arduino bootloader**: add avrdude as an external tool in Microchip Studio, or run it directly:

  ```
  avrdude -p m328pb -c arduino -P COM5 -b 115200 -U flash:w:RoboCar.hex:i
  ```

### Before the first run

- **Fuses**: the chip must run from a 16 MHz external crystal with CKDIV8 disabled. A new chip runs at 1 MHz from the internal oscillator, which breaks all timing and the baud rate.
- **Motor driver address**: check that `MOTOR_I2C_ADDR` matches your driver board. If it is wrong, sensors and serial work but the wheels don't move.

### Run

1. Open a serial terminal at **9600 baud** to watch the distances and decisions.
2. Place the car on the floor and switch on the motor power.

## Tuning notes

- The distance thresholds and speeds were tuned for this chassis and battery. A different floor surface or battery charge may need different values.
- In a large open room all sensors return the "no echo" value. Because the readings don't change, the stuck detector can trigger an escape every few seconds. Raising `STUCK_TIMEOUT_MS` or skipping the stuck check when the front is clear avoids this.

## License

MIT License
Copyright (c) 2026 B M Sadman Showmik
