#ifndef VZNNCV_STEPPER_MOTOR_EXTRA_H
#define VZNNCV_STEPPER_MOTOR_EXTRA_H
/**
 * This module contains experimental features.
 */

#include "vznncv_stepper_motor.h"
namespace vznncvsteppermotor {

#if defined(TARGET_STM)

/**
 * Helper SinglePulse driver that uses STM32 one pulse pwm output mode to generate a single pulse.
 *
 * limitations:
 * - if timer is used to generate single pulse, other timer functionality of this time (like PWM) cannot be used
 * - if multiple pins (with different timer channels) uses the the same timer for one pulse mode, all STM32SinglePulse
 *   instances should have the same duration.
 * - if current timer pulse isn't finished, a ::generate method doesn't return control till it can start pulse generation
 */
class STM32SinglePulse : public BaseSinglePulse {
protected:
    pwmout_t _pwm;
    static uint32_t _pwm_get_ll_channel(pwmout_t *pwm);
    static inline TIM_TypeDef *_pwm_get_instance(pwmout_t *pwm)
    {
        return (TIM_TypeDef *)pwm->pwm;
    }
    static void _pwm_set_ccr(pwmout_t *pwm, uint32_t value);
    static void _pwm_set_polarity(pwmout_t *pwm, uint32_t ll_channel, Polarity polarity);

    int _pwm_init(PinName pin, Polarity polarity);
    int _pwm_deinit();

public:
    /**
     * Constructor.
     *
     * @param pin PWM pin
     * @param pulse_width pulse width
     * @param polarity polarity
     */
    STM32SinglePulse(PinName pin, nanoseconds_u32 pulse_width = DEFAULT_PULSE_WIDTH, Polarity polarity = POLARITY_HIGH);
    ~STM32SinglePulse();

    // BaseOnePulse interface
    int set_timings(nanoseconds_u32 pulse_width) override;
    int set_polarity(Polarity polarity) override;
    int generate() override;
};

#endif // TARGET_STM

/**
 * Helper adapter of sequences with fixed step for BaseStepperMotor::CustomStepCallback callback.
 *
 * The BaseStepperMotor class provides method BaseStepperMotor::set_mode_custom_step to control interval between steps manually,
 * but it isn't convenient for simple cases, when you a sequence of target positions with fixed interval.
 * This class converts such sequences to intervals between steps for such cases.
 */
class SimpleSequenceWrapper : private NonCopyable<SimpleSequenceWrapper> {
public:
    enum ControlFlag : uint8_t {
        /**
         * Stop movement.
         *
         * If this flag is set, current value is ignored and movement is aborted.
         */
        FLAG_STOP = 0x01,
    };

    struct sample_t {
        sample_t() = default;

        sample_t(int value, uint8_t flags)
            : value(value)
            , flags(flags)
        {
        }

        sample_t(int value)
            : sample_t(value, 0)
        {
        }

        /**
         * Next target value.
         */
        int value;
        /**
         * Zero or more ORed ControlFlag flags.
         */
        uint8_t flags;
    };

private:
    Callback<sample_t()> _sequence_callback;
    uint32_t _seqeunce_interval_us;

    BaseStepperMotor::step_instruction_t _step_instruction;

    uint16_t _step_count;
    int16_t _step_adjustment_count;
    int32_t _steps_to_go_abs;

public:
    /**
     * Constructor.
     *
     * @param sequence_callback sequence callback. Each call should return next sequence value.
     * @param interval interval between sequence samples
     */
    SimpleSequenceWrapper(Callback<sample_t()> sequence_callback, microseconds_u32 interval);

    /**
     * Constructor.
     */
    SimpleSequenceWrapper();

    ~SimpleSequenceWrapper() = default;

    /**
     * Set sequence.
     *
     * This method should be invoked if default constructor is used.
     *
     * Note: this method additionally resets wrapper state.
     *
     * @param sequence_callback sequence callback. Each call should return next sequence value.
     * @param interval interval between sequence samples
     * @return 0 on success, otherwise non-zero value
     */
    int set_sequence(Callback<sample_t()> sequence_callback, microseconds_u32 interval)
    {
        _step_count = 0;
        _step_adjustment_count = 0;
        _steps_to_go_abs = 0;
        _sequence_callback = sequence_callback;
        _seqeunce_interval_us = interval.count();
        return 0;
    }

    /**
     * Get next step instruction.
     *
     * @param pos
     * @return
     */
    BaseStepperMotor::step_instruction_t next(const BaseStepperMotor::position_t &pos);

    /**
     * @c callback(wrapper_ptr, &SimpleSequenceWrapper::next) shortcut.
     *
     * @return
     */
    BaseStepperMotor::CustomStepCallback get_custom_step_callback();
};
}
#endif // VZNNCV_STEPPER_MOTOR_EXTRA_H
