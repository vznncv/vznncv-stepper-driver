# Base Mbed OS driver for DRV8825/A4988 like stepper motor controllers

This library provides stepper motor.

It's inspired by arduino [AccelStepper](https://www.airspayce.com/mikem/arduino/AccelStepper/index.html)
library, but unlike it uses timer interrupts to run steps.

Currently it contains only implementation for stepper motor with DRV8825/A4988 controller IC.

Note: `vznncv` prefix is added to library files and namespace to prevent possible
      name conflicts with other stepper motor driver libraries.

### Mbed OS version support

| Mbed OS | status |
|---|---|
| 6.5 | Compiles and runs ok |
