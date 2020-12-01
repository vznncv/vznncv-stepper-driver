# Base Mbed OS driver for DRV8825/A4988 like stepper motor controllers

This library provides stepper motor.

It's inspired by arduino [AccelStepper](https://www.airspayce.com/mikem/arduino/AccelStepper/index.html)
library, but unlike it uses timer interrupts to run steps.

MCU with 84 MHz cortex M4 core (like SMT32F401) gives the following steps per seconds limits:
- mbed-os "debug" profile: 30000 steps per second
- mbed-os "release" profile: 100000 steps per second

But it isn't recommended to reach this speeds, as in this cases IRQs consumes all CPU time,
so you main code won't work.

Note: `vznncv` prefix is added to library files and namespace to prevent possible
      name conflicts with other stepper motor driver libraries.

### Supported stepper motors

Currently library contains only implementation for stepper motor with A4988/DRV8825 controller IC or compatible chips.

### Mbed OS version support

| Mbed OS | status |
|---|---|
| 6.5 | Compiles and runs ok |
