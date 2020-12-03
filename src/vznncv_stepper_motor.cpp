#include "vznncv_stepper_motor.h"
#include <chrono>
#include <cmath>

using namespace vznncvsteppermotor;

//-----------------------------------------------------------------------------
// Common stepper driver code
//-----------------------------------------------------------------------------

BaseStepperMotor::StepTicker::StepTicker(const ticker_data_t *data, BaseStepperMotor *base_stepper_motor)
    : TimerEvent(data)
    , _base_stepper_motor(base_stepper_motor)
    , _schedule_flag(CORE_UTIL_ATOMIC_FLAG_INIT)
{
}

BaseStepperMotor::StepTicker::~StepTicker()
{
}

void BaseStepperMotor::StepTicker::trigger()
{
    CriticalSectionLock lock;
    if (!core_util_atomic_flag_test_and_set(&_schedule_flag)) {
        sleep_manager_lock_deep_sleep();
        // set initial event timestamp
        set_time_point(event, _ticker_data.now());
        // process step immediately
        handler();
    }
}

void BaseStepperMotor::StepTicker::force_cancel()
{
    CriticalSectionLock lock;
    remove();
    core_util_atomic_flag_clear(&_schedule_flag);
}

microseconds_u32 BaseStepperMotor::StepTicker::delay_before_event()
{
    CriticalSectionLock lock;

    // check if we have any event
    if (!core_util_atomic_flag_test_and_set(&_schedule_flag)) {
        core_util_atomic_flag_clear(&_schedule_flag);
        return 0us;
    }

    // check event time
    TickerDataClock::time_point current_time = _ticker_data.now();
    TickerDataClock::time_point event_time = get_time_point(event);
    if (current_time >= event_time) {
        // set minimal time to 1 us
        return 1us;
    }

    return event_time - current_time;
}

void BaseStepperMotor::StepTicker::handler()
{
    microseconds_u32 next_step = _base_stepper_motor->_execute_step();
    if (next_step != 0ms) {
        insert_absolute(get_time_point(event) + next_step);
    } else {
        sleep_manager_unlock_deep_sleep();
        core_util_atomic_flag_clear(&_schedule_flag);
    }
}

void BaseStepperMotor::StepTicker::set_time_point(ticker_event_t &event, TickerDataClock::time_point time_point)
{
    event.timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(time_point).time_since_epoch().count();
}

int BaseStepperMotor::_force_step_cancel()
{
    int err;
    _step_ticker.force_cancel();
    // reset speed for mode with constant acceleration
    if (_execute_step_impl == &BaseStepperMotor::_execute_step_with_constant_acceleration) {
        _step_params.constant_acceleration.current_speed = 0.0f;
    }
    // set neutral direction
    err = set_direction_impl(DIR_NONE);
    if (!err) {
        _last_dir = MoveDirection::DIR_NONE;
    }
    if (err && !_err) {
        _err = err;
    }
    return err;
}

microseconds_u32 BaseStepperMotor::_execute_step()
{
    if (_err) {
        return microseconds_u32(0);
    }

    step_instruction_t step_instruction = (this->*_execute_step_impl)();

    if (_err) {
        return microseconds_u32(0);
    }

    step_description_t step_description = { step_instruction.dir, step_instruction.next, 0 };

    // adjust direction
    if (step_instruction.dir != _last_dir) {
        _err = set_direction_impl(step_instruction.dir);
        if (_err) {
            return microseconds_u32(0);
        }
        _last_dir = step_instruction.dir;
    }
    // execute step
    if (step_instruction.dir != MoveDirection::DIR_NONE) {
        if (step_instruction.dir == MoveDirection::DIR_FORWARD) {
            step_description.step = _position.current + 1;
        } else {
            step_description.step = _position.current - 1;
        }
        _err = step_impl(step_description);
        if (_err) {
            return microseconds_u32(0);
        }
        _position.current = step_description.step;

        // stop motor automatically if there are no other steps
        if (step_instruction.next == 0us) {
            _err = set_direction_impl(MoveDirection::DIR_NONE);
            if (_err) {
                return microseconds_u32(0);
            }
            _last_dir = MoveDirection::DIR_NONE;
        }
    }

    return step_instruction.next;
}

BaseStepperMotor::step_instruction_t BaseStepperMotor::_execute_step_with_constant_speed()
{
    step_instruction_t step_instruction;
    StepParams::ConstantSpeed &params = _step_params.constant_speed;

    int steps_to_go = distance_to_go();

    // check if we reach a target position
    if (steps_to_go == 0) {
        step_instruction.dir = DIR_NONE;
        step_instruction.next = 0us;
        return step_instruction;
    }

    // calculate next step
    if (steps_to_go > 0) {
        step_instruction.dir = DIR_FORWARD;
        step_instruction.next = steps_to_go > 1 ? params.step_interval : 0ms;
    } else {
        step_instruction.dir = DIR_BACKWARD;
        step_instruction.next = steps_to_go < -1 ? params.step_interval : 0ms;
    }

    return step_instruction;
}

BaseStepperMotor::step_instruction_t BaseStepperMotor::_execute_step_with_constant_acceleration()
{
    step_instruction_t step_instruction;
    StepParams::ConstantAcceleration &params = _step_params.constant_acceleration;

    uint32_t dt_us;
    // note: the formula steps_to_stop = V * V / (2 * a) give the following error for our calculations:
    //       steps_to_stop_underestimation = log(steps_to_stop) / 4 + log(e)
    //       so try to com
    int steps_to_stop = (int)(params.current_speed * params.current_speed * params.one_div_double_acc) + params.STS_COMP_B;
    int steps_to_go = distance_to_go();
    float new_speed;

    // check if we reach a target position
    if (steps_to_go == 0 && abs(steps_to_stop) <= params.STS_COMP_B) {
        // target is reached
        params.current_speed = 0.0f;
        step_instruction.dir = DIR_NONE;
        step_instruction.next = 0us;
        return step_instruction;
    }

    // check minimal speed
    if (fabs(params.current_speed) < params.min_step_speed) {
        params.current_speed = steps_to_go > 0 ? params.min_step_speed : -params.min_step_speed;
    }
    // estimate time of the next step, using current speed
    dt_us = 1'000'000 / fabsf(params.current_speed);

    // calculate speed of next step
    new_speed = params.current_speed;
    if (new_speed < 0) {
        steps_to_stop = -steps_to_stop;
    }
    if (steps_to_go > 0) {
        if (steps_to_go <= steps_to_stop) {
            // need deacceleration
            new_speed -= dt_us * params.max_acceleration_div_mil;

            if (new_speed < 0) {
                new_speed = 0;
            }
        } else if (params.current_speed < params.max_speed) {
            // need acceleration
            new_speed += dt_us * params.max_acceleration_div_mil;
            if (new_speed > params.max_speed) {
                new_speed = params.max_speed;
            }
        }
    } else {
        if (steps_to_go >= steps_to_stop) {
            // need deacceleration
            new_speed += dt_us * params.max_acceleration_div_mil;

            if (new_speed > 0) {
                new_speed = 0;
            }
        } else if (-params.current_speed < params.max_speed) {
            // need acceleration
            new_speed -= dt_us * params.max_acceleration_div_mil;
            if (new_speed < -params.max_speed) {
                new_speed = -params.max_speed;
            }
        }
    }

    // calculate parameters of current step
    step_instruction.next = microseconds_u32(dt_us);
    if (params.current_speed > 0) {
        step_instruction.dir = DIR_FORWARD;
    } else {
        step_instruction.dir = DIR_BACKWARD;
    }

    // update speed for next step
    params.current_speed = new_speed;

    return step_instruction;
}

BaseStepperMotor::step_instruction_t BaseStepperMotor::_execute_step_with_custom_step()
{
    StepParams::CustomStep &params = _step_params.custom_step;
    return params.callback->call(_position);
}

BaseStepperMotor::BaseStepperMotor()
    : _step_ticker(get_us_ticker_data(), this)
    , _execute_step_impl(nullptr)
    , _position{ 0, 0 }
    , _err(0)
    , _initialized(false)
    , _last_dir(DIR_NONE)
    , _state(STATE_DISABLED)
{
    set_mode_constant_acceleration(DEFAULT_MAX_SPEED, DEFAULT_MAX_ACCELERATION);
}

int BaseStepperMotor::init(bool start)
{
    if (_initialized) {
        return 0;
    }

    // call subclass initialization
    _err = init_impl();
    if (_err) {
        return _err;
    }
    _state = STATE_DISABLED;

    // specify direction
    _last_dir = DIR_NONE;
    _err = set_direction_impl(_last_dir);
    if (_err) {
        return _err;
    }

    _initialized = true;

    if (start) {
        set_state(STATE_ENABLED);
    }

    return _err;
}

BaseStepperMotor::~BaseStepperMotor()
{
    _cleanup_step_params();
}

void BaseStepperMotor::_cleanup_step_params()
{
    if (_execute_step_impl == &BaseStepperMotor::_execute_step_with_custom_step) {
        _execute_step_impl = nullptr;
        StepParams::CustomStep &params = _step_params.custom_step;
        params.callback->~Callback();
    }
}

int BaseStepperMotor::set_mode_constant_speed(float max_speed)
{
    MBED_ASSERT(max_speed > 0);
    CriticalSectionLock lock;

    _cleanup_step_params();
    StepParams::ConstantSpeed &params = _step_params.constant_speed;

    _execute_step_impl = &BaseStepperMotor::_execute_step_with_constant_speed;
    params.max_speed = max_speed;
    params.step_interval = microseconds_u32((uint32_t)(1'000'000 / max_speed));

    return 0;
}

int BaseStepperMotor::set_mode_constant_acceleration(float max_speed, float max_acceleration)
{
    MBED_ASSERT(max_speed > 0);
    MBED_ASSERT(max_acceleration > 0);
    CriticalSectionLock lock;

    _cleanup_step_params();
    StepParams::ConstantAcceleration &params = _step_params.constant_acceleration;

    _execute_step_impl = &BaseStepperMotor::_execute_step_with_constant_acceleration;
    params.max_speed = max_speed;
    params.max_acceleration = max_acceleration;
    params.current_speed = 0.0f;
    // helper calculation constants
    params.one_div_double_acc = (1.0f / (2.0f * max_acceleration)) * params.STS_COMP_A;
    params.min_step_speed = sqrtf(max_acceleration * 2);
    params.max_acceleration_div_mil = max_acceleration / 1'000'000;

    return 0;
}

int BaseStepperMotor::set_mode_custom_step(BaseStepperMotor::CustomStepCallback callback)
{
    MBED_ASSERT(callback != nullptr);
    CriticalSectionLock lock;

    _cleanup_step_params();
    StepParams::CustomStep &params = _step_params.custom_step;

    _execute_step_impl = &BaseStepperMotor::_execute_step_with_custom_step;
    params.callback = new (params.callback_memory) CustomStepCallback(callback);

    return 0;
}

BaseStepperMotor::MoveMode BaseStepperMotor::get_mode() const
{
    if (_execute_step_impl == &BaseStepperMotor::_execute_step_with_constant_speed) {
        return MODE_CONSTANT_SPEED;
    } else if (_execute_step_impl == &BaseStepperMotor::_execute_step_with_constant_acceleration) {
        return MODE_CONSTANT_ACCELERATION;
    } else if (_execute_step_impl == &BaseStepperMotor::_execute_step_with_custom_step) {
        return MODE_CUSTOM_STEP;
    } else {
        // Invalid pointer state.
        // Implementation logic prohibit such states. So if we reach it, probably memory has been damaged.
        MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_APPLICATION, MBED_ERROR_CODE_ENOTRECOVERABLE), "Invalid BaseStepperMotor state.");
    }
}

int BaseStepperMotor::get_mode_params_constant_speed(float &max_speed)
{
    if (get_mode() != MODE_CONSTANT_SPEED) {
        return -1;
    }
    StepParams::ConstantSpeed &params = _step_params.constant_speed;

    max_speed = params.max_speed;
    return 0;
}

int BaseStepperMotor::get_mode_params_constant_acceleration(float &max_speed, float &max_acceleration)
{
    if (get_mode() != MODE_CONSTANT_ACCELERATION) {
        return -1;
    }
    StepParams::ConstantAcceleration &params = _step_params.constant_acceleration;

    max_speed = params.max_speed;
    max_acceleration = params.max_acceleration;
    return 0;
}

int BaseStepperMotor::get_mode_params_custom_step(BaseStepperMotor::CustomStepCallback &callback)
{
    if (get_mode() != MODE_CUSTOM_STEP) {
        return -1;
    }
    StepParams::CustomStep &params = _step_params.custom_step;

    callback = *params.callback;
    return 0;
}

void BaseStepperMotor::move(int rel_pos)
{
    CriticalSectionLock lock;
    // note:
    // we relies on a two's complement representation of integer number
    // and serial number arithmetic to process overflow correctly
    _position.target += (int32_t)rel_pos;
    resume_movement();
}

void BaseStepperMotor::move_to(int abs_pos)
{
    _position.target = abs_pos;
    resume_movement();
}

int BaseStepperMotor::pause_movement()
{
    return _force_step_cancel();
}

int BaseStepperMotor::resume_movement()
{
    if (!_err && _state == STATE_ENABLED) {
        _step_ticker.trigger();
    }
    return _err;
}

int BaseStepperMotor::wait_end_of_movement()
{
    do {
        if (_err || _state == STATE_DISABLED) {
            break;
        }
        microseconds_u32 delay_before_event = _step_ticker.delay_before_event();
        if (delay_before_event == 0us) {
            break;
        }

        // convert microseconds to milliseconds
        milliseconds_u32 delay_before_event_ms = milliseconds_u32((delay_before_event + 999us).count() / 1000);

        // note: currently simply wait 1ms to simplify tick logic.
        // Later this behavior may be changed.
        ThisThread::sleep_for(delay_before_event_ms);
    } while (true);

    return _err;
}

int BaseStepperMotor::get_target_position() const
{
    return _position.target;
}

void BaseStepperMotor::set_target_position(int value)
{
    CriticalSectionLock lock;
    _position.target = value;
}

int BaseStepperMotor::get_current_position() const
{
    return _position.current;
}

void BaseStepperMotor::set_current_position(int value)
{
    CriticalSectionLock lock;
    _position.current = value;
}

int BaseStepperMotor::distance_to_go() const
{
    CriticalSectionLock lock;
    // note:
    // we relies on a two's complement representation of integer number
    // and serial number arithmetic to process overflow correctly
    return _position.target - _position.current;
}

int BaseStepperMotor::set_state(BaseStepperMotor::State state)
{
    if (state == _state) {
        return 0;
    }
    State prev_state = _state;

    // set state and stop interrupts if it's needed
    {
        CriticalSectionLock lock;
        _state = state;
        if (state == STATE_DISABLED) {
            _force_step_cancel();
        }
    }
    // try perform actual operations
    int err = set_state_impl(state);

    if (err) {
        // restore original state
        CriticalSectionLock lock;
        _state = prev_state;
        _err = err;
    }

    return err;
}

BaseStepperMotor::State BaseStepperMotor::get_state() const
{
    return _state;
}

int BaseStepperMotor::get_error() const
{
    return _err;
}

void BaseStepperMotor::clear_error()
{
    CriticalSectionLock lock;
    _err = 0;
}

int BaseStepperMotor::init_impl()
{
    return 0;
}

int BaseStepperMotor::set_state_impl(BaseStepperMotor::State state)
{
    return 0;
}

//-----------------------------------------------------------------------------
// StepDirDriverStepperMotor code
//-----------------------------------------------------------------------------

constexpr nanoseconds_u32 BaseSinglePulse::DEFAULT_PULSE_WIDTH;

//
// SimpleOnePulse implementation
//

SimpleSinglePulse::SimpleSinglePulse(PinName pin, nanoseconds_u32 pulse_width, Polarity polarity)
    : _out(pin, !polarity)
    , _pulse_width_ns(pulse_width.count())
    , _polarity(polarity)
{
    MBED_ASSERT(pulse_width.count() > 0);
}

int SimpleSinglePulse::set_timings(nanoseconds_u32 pulse_width)
{
    MBED_ASSERT(pulse_width.count() > 0);
    _pulse_width_ns = pulse_width.count();
    return 0;
}

int SimpleSinglePulse::set_polarity(BaseSinglePulse::Polarity polarity)
{
    _polarity = polarity;
    _out = !_polarity;
    return 0;
}

int SimpleSinglePulse::generate()
{
    _out = _polarity;
    wait_ns(_pulse_width_ns);
    _out = !_polarity;
    return 0;
}

//
// StepDirDriverStepperMotor implementation
//

StepDirDriverStepperMotor::StepDirDriverStepperMotor(PinName step_pin, PinName dir_pin, PinName en_pin, uint32_t flags, nanoseconds_u32 pulse_lenth_us)
    : StepDirDriverStepperMotor(new SimpleSinglePulse(step_pin, pulse_lenth_us, _get_polarity_from_flags(flags)), dir_pin, en_pin, flags)
{
    _state_flags |= _STATE_FLAG_CLEANUP_PULSE_GENERATOR_MASK;
}

StepDirDriverStepperMotor::StepDirDriverStepperMotor(BaseSinglePulse *pulse_generator, PinName dir_pin, PinName en_pin, uint32_t flags)
    : _pulse_generator(pulse_generator)
    , _dir_pin(dir_pin, flags & _FLAG_DIR_MASK)
    , _en_pin(en_pin, flags & _FLAG_ENABLE_MASK)
    , _flags(flags)
    , _state_flags(0x00)
{
    MBED_ASSERT((flags & 0xFFFFFF00) == 0);
    int err;
    err = pulse_generator->set_polarity(_get_polarity_from_flags(flags));
    MBED_ASSERT(err == 0);
}

StepDirDriverStepperMotor::~StepDirDriverStepperMotor()
{
    if (_state_flags & _STATE_FLAG_CLEANUP_PULSE_GENERATOR_MASK) {
        delete _pulse_generator;
    }
}

int StepDirDriverStepperMotor::init_impl()
{
    return 0;
}

int StepDirDriverStepperMotor::step_impl(const step_description_t &step)
{
    return _pulse_generator->generate();
}

int StepDirDriverStepperMotor::set_direction_impl(BaseStepperMotor::MoveDirection dir)
{
    switch (dir) {
    case BaseStepperMotor::DIR_FORWARD:
        _dir_pin = !(_flags & _FLAG_DIR_MASK);
        break;
    case BaseStepperMotor::DIR_NONE:
        break;
    case BaseStepperMotor::DIR_BACKWARD:
        _dir_pin = _flags & _FLAG_DIR_MASK;
        break;
    default:
        MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER, MBED_ERROR_CODE_INVALID_OPERATION), "Unknown channel");
    }
    return 0;
}

int StepDirDriverStepperMotor::set_state_impl(BaseStepperMotor::State state)
{
    switch (state) {
    case BaseStepperMotor::STATE_ENABLED:
        _en_pin = !(_flags & _FLAG_ENABLE_MASK);
        break;
    case BaseStepperMotor::STATE_DISABLED:
        _en_pin = _flags & _FLAG_ENABLE_MASK;
        break;
    default:
        MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER, MBED_ERROR_CODE_INVALID_OPERATION), "Unknown state");
    }
    return 0;
}
