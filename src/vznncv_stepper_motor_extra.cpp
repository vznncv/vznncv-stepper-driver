#include "vznncv_stepper_motor_extra.h"
#include <chrono>
#include <cmath>

using namespace vznncvsteppermotor;

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
    set_timings(DEFAULT_PULSE_WIDTH);

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

STM32SinglePulse::STM32SinglePulse(PinName pin, nanoseconds_u32 pulse_width, BaseSinglePulse::Polarity polarity)
{
    int err;
    err = _pwm_init(pin, polarity);
    MBED_ASSERT(err == 0);
    err = set_timings(pulse_width);
    MBED_ASSERT(err == 0);
}

STM32SinglePulse::~STM32SinglePulse()
{
    int err = _pwm_deinit();
    MBED_ASSERT(err == 0);
}

int STM32SinglePulse::set_timings(nanoseconds_u32 pulse_width)
{
    CriticalSectionLock lock;
    TIM_TypeDef *tim_instance = _pwm_get_instance(&_pwm);

    // reduce timing to us to simplify timer configuration
    int width_us = pulse_width.count() / 1000 + pulse_width.count() % 1000 ? 1 : 0;
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
// CustomSequenceWrapper implementation
//

SimpleSequenceWrapper::SimpleSequenceWrapper(Callback<sample_t()> sequence_callback, microseconds_u32 interval)
    : _sequence_callback(sequence_callback)
    , _seqeunce_interval_us(interval.count())
    , _step_count(0)
    , _step_adjustment_count(0)
    , _steps_to_go_abs(0)
{
}

BaseStepperMotor::step_instruction_t SimpleSequenceWrapper::next(const BaseStepperMotor::position_t &pos)
{
    int steps_to_go;
    sample_t sample;

    if (_step_count == 0) {
        sample = _sequence_callback();
        if (sample.flags & FLAG_STOP) {
            return { BaseStepperMotor::DIR_NONE, 0us };
        }
        steps_to_go = sample.value - pos.current;

        if (steps_to_go == 0) {
            _step_instruction.dir = BaseStepperMotor::DIR_NONE;
            _step_instruction.next = microseconds_u32(_seqeunce_interval_us);
            _step_count = 1;
        } else {
            if (steps_to_go > 0) {
                _step_instruction.dir = BaseStepperMotor::DIR_FORWARD;
            } else {
                _step_instruction.dir = BaseStepperMotor::DIR_BACKWARD;
                steps_to_go = -steps_to_go;
            }
            _step_count = steps_to_go;
            _step_instruction.next = microseconds_u32(_seqeunce_interval_us / steps_to_go);
            _step_adjustment_count = _seqeunce_interval_us / steps_to_go;
            if (_steps_to_go_abs > steps_to_go) {
                _step_adjustment_count = -_step_adjustment_count;
                _step_adjustment_count--;
            } else {
                _step_instruction.next += 1us;
                _step_adjustment_count++;
            }
        }
        _steps_to_go_abs = steps_to_go;
    }

    _step_count -= 1;
    if (_step_adjustment_count != 0) {
        if (_step_adjustment_count > 0) {
            _step_adjustment_count--;
            if (_step_adjustment_count == 0) {
                _step_instruction.next -= 1us;
            }
        } else {
            _step_adjustment_count++;
            if (_step_adjustment_count == 0) {
                _step_instruction.next += 1us;
            }
        }
    }

    return _step_instruction;
}

BaseStepperMotor::CustomStepCallback SimpleSequenceWrapper::get_custom_step_callback()
{
    return callback(this, &SimpleSequenceWrapper::next);
}
