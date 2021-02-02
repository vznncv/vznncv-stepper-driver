# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/)
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

### [Unreleased]

### Changed

- Adjust ST820 direction flag for consistency with A4988.
- Move step/dir driver flags to a separate class to allow its usage in a custom step/dir driver.
- Fix typo in the`PinModeFlags::FLAG_DEFAULT_...` constants.

### Added

- Add possibility to set sequence function after `SimpleSequenceWrapper` object creation.
- Add `BaseStepperMotor::get_current_speed`, `BaseStepperMotor::get_current_speed_int` methods that allows to get current speed.
- Overload `BaseStepperMotor::set_mode_constant_acceleration` method to add `initial_speed` parameter.

## [0.1.0] - 2020-12-05

### Added

- Add "single pulse" classes
- Add base stepper motor driver
