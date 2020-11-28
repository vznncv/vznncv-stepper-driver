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

void BaseStepperMotor::StepTicker::handler()
{
    core_util_atomic_flag_clear(&_schedule_flag);
    uint32_t next_step = _base_stepper_motor->_execute_step();
    if (next_step) {
        insert_absolute(get_time_point(event) + std::chrono::microseconds(next_step));
        core_util_atomic_flag_test_and_set(&_schedule_flag);
    } else {
        sleep_manager_unlock_deep_sleep();
    }
}

void BaseStepperMotor::StepTicker::set_time_point(ticker_event_t &event, TickerDataClock::time_point time_point)
{
    event.timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(time_point).time_since_epoch().count();
}

void BaseStepperMotor::_update_helper_consts()
{
    CriticalSectionLock lock;
    if (_max_acceleration > 0.0f) {
        _calc_const.acc.one_div_double_acc = 1.0f / (2.0f * _max_acceleration);
        _calc_const.acc.min_step_speed = sqrtf(_max_acceleration * 2);
        _calc_const.acc.max_acceleration_div_mil = _max_acceleration * _STS_COMP_A / 1'000'000;
    } else {
        _calc_const.csp.step_interval_us = 1'000'000 / _max_speed;
    }
}

uint32_t BaseStepperMotor::_execute_step()
{
    if (_err) {
        return 0;
    }
    if (_max_acceleration > 0.0f) {
        // move with acceleration
        return _execute_step_with_acceleration();
    } else {
        // move with constant speed
        return _execute_step_with_constant_speed();
    }
}

uint32_t BaseStepperMotor::_execute_step_with_acceleration()
{
    uint32_t dt_us;
    // note: the formula steps_to_stop = V * V / (2 * a) give the following error for our calculations:
    //       steps_to_stop_underestimation = log(steps_to_stop) / 4 + log(e)
    //       so try to com
    int steps_to_stop = (int)(_current_speed * _current_speed * _calc_const.acc.one_div_double_acc) + _STS_COMP_B;
    int steps_to_go = distance_to_go();
    float new_speed;

    // check if we reach a target position
    if (steps_to_go == 0 && abs(steps_to_stop) <= _STS_COMP_B) {
        // target is reached
        _current_speed = 0.0f;
        if (_last_dir != DIR_NONE) {
            _err = set_direction_impl(DIR_NONE);
            if (_err) {
                return 0;
            }
            _last_dir = DIR_NONE;
        }
        return 0;
    }

    // check minimal speed
    if (fabs(_current_speed) < _calc_const.acc.min_step_speed) {
        _current_speed = steps_to_go > 0 ? _calc_const.acc.min_step_speed : -_calc_const.acc.min_step_speed;
    }
    // estimate time of the next step, using current speed
    dt_us = 1'000'000 / fabsf(_current_speed);

    // update speed
    new_speed = _current_speed;
    if (new_speed < 0) {
        steps_to_stop = -steps_to_stop;
    }
    if (steps_to_go > 0) {
        if (steps_to_go <= steps_to_stop) {
            // need deacceleration
            new_speed -= dt_us * _calc_const.acc.max_acceleration_div_mil;

            if (new_speed < 0) {
                new_speed = 0;
            }
        } else if (_current_speed < _max_speed) {
            // need acceleration
            new_speed += dt_us * _calc_const.acc.max_acceleration_div_mil;
            if (new_speed > _max_speed) {
                new_speed = _max_speed;
            }
        }
    } else {
        if (steps_to_go >= steps_to_stop) {
            // need deacceleration
            new_speed += dt_us * _calc_const.acc.max_acceleration_div_mil;

            if (new_speed > 0) {
                new_speed = 0;
            }
        } else if (-_current_speed < _max_speed) {
            // need acceleration
            new_speed -= dt_us * _calc_const.acc.max_acceleration_div_mil;
            if (new_speed < -_max_speed) {
                new_speed = -_max_speed;
            }
        }
    }

    // calculate new step and direction
    int32_t next_pos = _current_pos;
    MoveDirection new_dir;
    if (_current_speed > 0) {
        next_pos++;
        new_dir = DIR_FORWARD;
    } else {
        next_pos--;
        new_dir = DIR_BACKWARD;
    }
    // update direction
    if (new_dir != _last_dir) {
        _err = set_direction_impl(new_dir);
        if (_err) {
            return 0;
        }
        _last_dir = new_dir;
    }
    // update position
    _err = step_impl(next_pos);
    if (_err) {
        return 0;
    }
    _current_pos = next_pos;
    _current_speed = new_speed;

    // schedule next interrupt
    return dt_us;
}

uint32_t BaseStepperMotor::_execute_step_with_constant_speed()
{
    int32_t next_pos;
    int steps_to_go = distance_to_go();

    // check if we reach a target position
    if (steps_to_go == 0) {
        if (DIR_NONE != _last_dir) {
            _err = set_direction_impl(DIR_NONE);
            if (_err) {
                return 0;
            }
            _last_dir = DIR_NONE;
        }
        return 0;
    }

    // calculate next step
    next_pos = _current_pos;
    MoveDirection new_dir;
    if (steps_to_go > 0) {
        next_pos++;
        new_dir = DIR_FORWARD;
    } else {
        next_pos--;
        new_dir = DIR_BACKWARD;
    }

    // check direction
    if (new_dir != _last_dir) {
        _err = set_direction_impl(new_dir);
        if (_err) {
            return 0;
        }
        _last_dir = new_dir;
    }

    // execute step
    _err = step_impl(next_pos);
    if (_err) {
        return 0;
    } else {
        _current_pos = next_pos;
    }

    // schedule next interrupt
    return _calc_const.csp.step_interval_us;
}

BaseStepperMotor::BaseStepperMotor(float max_speed, float max_acceleration)
    : _max_acceleration(1.0f)
    , _max_speed(1.0f)
    , _current_speed(0)
    , _current_pos(0)
    , _target_pos(0)
    , _err(0)
    , _initialized(false)
    , _last_dir(DIR_NONE)
    , _step_ticker(get_us_ticker_data(), this)
{
    set_max_speed(max_speed);
    set_max_acceleration(max_acceleration);
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
}

void BaseStepperMotor::set_max_speed(float value)
{
    MBED_ASSERT(value > 0);
    CriticalSectionLock lock;
    _max_speed = value;
    _update_helper_consts();
}

float BaseStepperMotor::get_max_speed() const
{
    return _max_speed;
}

void BaseStepperMotor::set_max_acceleration(float value)
{
    CriticalSectionLock lock;
    _max_acceleration = value;
    _update_helper_consts();
}

float BaseStepperMotor::get_max_acceleration()
{
    return _max_acceleration;
}

void BaseStepperMotor::move(int rel_pos)
{
    CriticalSectionLock lock;
    // note:
    // we relies on a two's complement representation of integer number
    // and serial number arithmetic to process overflow correctly
    _target_pos += (int32_t)rel_pos;
    resume_movement();
}

void BaseStepperMotor::move_to(int abs_pos)
{
    _target_pos = abs_pos;
    resume_movement();
}

int BaseStepperMotor::resume_movement()
{
    if (!_err && _state == STATE_ENABLED) {
        _step_ticker.trigger();
    }
    return _err;
}

int BaseStepperMotor::wait_stopping()
{
    do {
        {
            CriticalSectionLock lock;
            if (_err || _state == STATE_DISABLED) {
                break;
            }
            // check if target is reached
            if (_target_pos == _current_pos && _last_dir == DIR_NONE) {
                if (_max_acceleration > 0) {
                    // acceleration mode
                    if (_current_speed == 0.0f) {
                        break;
                    }
                } else {
                    // constant speed mode
                    break;
                }
            }
        }
        // note: currently simply wait 1ms to simplify tick logic.
        // Later this behavior may be changed.
        ThisThread::sleep_for(1ms);
    } while (true);

    return _err;
}

int BaseStepperMotor::get_target_position() const
{
    return _target_pos;
}

int BaseStepperMotor::get_current_position() const
{
    return _current_pos;
}

int BaseStepperMotor::distance_to_go() const
{
    CriticalSectionLock lock;
    // note:
    // we relies on a two's complement representation of integer number
    // and serial number arithmetic to process overflow correctly
    return _target_pos - _current_pos;
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
            _step_ticker.force_cancel();
            _current_speed = 0;
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

constexpr std::chrono::nanoseconds BaseSinglePulse::DEFAULT_WIDTH_NS;

//
// SimpleOnePulse implementation
//

SimpleSinglePulse::SimpleSinglePulse(PinName pin, Polarity polarity)
    : _out(pin, !polarity)
    , _width_ns(DEFAULT_WIDTH_NS.count())
    , _polarity(polarity)
{
}

int SimpleSinglePulse::set_timings(std::chrono::nanoseconds width_ns)
{
    MBED_ASSERT(width_ns.count() >= 0);
    _width_ns = width_ns.count();
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
    wait_ns(_width_ns);
    _out = !_polarity;
    return 0;
}

#if defined(TARGET_STM)

//
// STM32OnePulse implementation
//

uint32_t STM32SinglePulse::_pwm_get_ll_channel(pwmout_t *pwm)
{
    uint32_t ll_channel_code;
#if !defined(PWMOUT_INVERTED_NOT_SUPPORTED)
    if (pwm->inverted) {
        switch (pwm->channel) {
        case 1:
            ll_channel_code = LL_TIM_CHANNEL_CH1N;
            break;
        case 2:
            ll_channel_code = LL_TIM_CHANNEL_CH2N;
            break;
        case 3:
            ll_channel_code = LL_TIM_CHANNEL_CH3N;
            break;
#if defined(LL_TIM_CHANNEL_CH4N)
        case 4:
            ll_channel_code = LL_TIM_CHANNEL_CH4N;
            break;
#endif
        default: /* Optional */
            ll_channel_code = 0;
        }
    } else
#endif
    {
        switch (pwm->channel) {
        case 1:
            ll_channel_code = LL_TIM_CHANNEL_CH1;
            break;
        case 2:
            ll_channel_code = LL_TIM_CHANNEL_CH2;
            break;
        case 3:
            ll_channel_code = LL_TIM_CHANNEL_CH3;
            break;
        case 4:
            ll_channel_code = LL_TIM_CHANNEL_CH4;
            break;
        default: /* Optional */
            ll_channel_code = 0;
        }
    }
    MBED_ASSERT(ll_channel_code != 0);
    return ll_channel_code;
}

void STM32SinglePulse::_pwm_set_ccr(pwmout_t *pwm, uint32_t value)
{
    switch (pwm->channel) {
    case 1:
        LL_TIM_OC_SetCompareCH1(_pwm_get_instance(pwm), value);
        break;
    case 2:
        LL_TIM_OC_SetCompareCH2(_pwm_get_instance(pwm), value);
        break;
    case 3:
        LL_TIM_OC_SetCompareCH3(_pwm_get_instance(pwm), value);
        break;
    case 4:
        LL_TIM_OC_SetCompareCH4(_pwm_get_instance(pwm), value);
        break;
    default:
        // invalid channel
        MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER, MBED_ERROR_CODE_INVALID_OPERATION), "Unknown channel");
    }
}

void STM32SinglePulse::_pwm_set_polarity(pwmout_t *pwm, uint32_t ll_channel, BaseSinglePulse::Polarity polarity)
{
    // note: invert polarity to get idle state at first
    uint32_t ll_polarity = polarity == POLARITY_HIGH ? LL_TIM_OCPOLARITY_LOW : LL_TIM_OCPOLARITY_HIGH;
    LL_TIM_OC_SetPolarity(_pwm_get_instance(pwm), ll_channel, ll_polarity);
}

int STM32SinglePulse::_pwm_init(PinName pin, BaseSinglePulse::Polarity polarity)
{
    CriticalSectionLock lock;
    sleep_manager_lock_deep_sleep();

    // use PWM API to configure channel
    pwmout_init(&_pwm, pin);
    pwmout_period_us(&_pwm, 2);
    MBED_ASSERT(_pwm.prescaler == 1);
    TIM_TypeDef *tim_instance = _pwm_get_instance(&_pwm);
    // resolve LL channel
    uint32_t ll_channel = _pwm_get_ll_channel(&_pwm);
    // configure "one pulse" mode
    LL_TIM_DisableCounter(tim_instance);
    LL_TIM_SetOnePulseMode(tim_instance, LL_TIM_ONEPULSEMODE_SINGLE);
    _pwm_set_polarity(&_pwm, ll_channel, polarity);
    LL_TIM_GenerateEvent_UPDATE(tim_instance);
    // configure timings
    set_timings(DEFAULT_WIDTH_NS);

    return 0;
}

int STM32SinglePulse::_pwm_deinit()
{

    TIM_TypeDef *tim_instance = _pwm_get_instance(&_pwm);
    LL_TIM_GenerateEvent_UPDATE(tim_instance);
    LL_TIM_SetOnePulseMode(tim_instance, LL_TIM_ONEPULSEMODE_REPETITIVE);
    pwmout_free(&_pwm);
    sleep_manager_unlock_deep_sleep();
    return 0;
}

STM32SinglePulse::STM32SinglePulse(PinName pin, BaseSinglePulse::Polarity polarity)
{
    int err = _pwm_init(pin, polarity);
    MBED_ASSERT(err == 0);
}

STM32SinglePulse::~STM32SinglePulse()
{
    int err = _pwm_deinit();
    MBED_ASSERT(err == 0);
}

int STM32SinglePulse::set_timings(std::chrono::nanoseconds width_ns)
{
    MBED_ASSERT(width_ns.count() >= 0);
    CriticalSectionLock lock;
    TIM_TypeDef *tim_instance = _pwm_get_instance(&_pwm);

    // reduce timing to us to simplify timer configuration
    int width_us = width_ns.count() / 1000 + width_ns.count() % 1000 ? 1 : 0;
    MBED_ASSERT(width_us < 0xFFFF);

    // set period and pulse width
    LL_TIM_SetAutoReload(tim_instance, width_us);
    // note:
    // with 0 delay, output always have a "pulse" state,
    // so always set delay to 1
    _pwm_set_ccr(&_pwm, 0x0001);
    // update registers
    LL_TIM_GenerateEvent_UPDATE(tim_instance);
    return 0;
}

int STM32SinglePulse::set_polarity(BaseSinglePulse::Polarity polarity)
{
    // resolve LL channel
    uint32_t ll_channel = _pwm_get_ll_channel(&_pwm);
    // set polarity
    _pwm_set_polarity(&_pwm, ll_channel, polarity);
    return 0;
}

int STM32SinglePulse::generate()
{
    TIM_TypeDef *tim_instance = (TIM_TypeDef *)_pwm.pwm;
    // wait end of previous pulse
    while (LL_TIM_IsEnabledCounter(tim_instance)) {
    }
    // disable other timer by settings CCR to 0xFFFF
    LL_TIM_OC_SetCompareCH1(tim_instance, 0xFFFF);
    LL_TIM_OC_SetCompareCH2(tim_instance, 0xFFFF);
    LL_TIM_OC_SetCompareCH3(tim_instance, 0xFFFF);
    LL_TIM_OC_SetCompareCH4(tim_instance, 0xFFFF);
    _pwm_set_ccr(&_pwm, 0x0001);
    // trigger pulse
    LL_TIM_EnableCounter(tim_instance);
    return 0;
}

#endif // TARGET_STM

//
// STM32OnePulse implementation
//

constexpr std::chrono::nanoseconds StepDirDriverStepperMotor::DEFAULT_PULSE_LENTH_US;

StepDirDriverStepperMotor::StepDirDriverStepperMotor(PinName step_pin, PinName dir_pin, PinName en_pin, uint32_t flags, chrono::nanoseconds pulse_lenth_us)
    : StepDirDriverStepperMotor(new SimpleSinglePulse(step_pin, _get_polarity_from_flags(flags)), dir_pin, en_pin, flags, pulse_lenth_us)
{
    _state_flags |= _STATE_FLAG_CLEANUP_PULSE_GENERATOR_MASK;
}

StepDirDriverStepperMotor::StepDirDriverStepperMotor(BaseSinglePulse *pulse_generator, PinName dir_pin, PinName en_pin, uint32_t flags, chrono::nanoseconds pulse_lenth_us)
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
    err = pulse_generator->set_timings(pulse_lenth_us);
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

int StepDirDriverStepperMotor::step_impl(int32_t step_impl)
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
