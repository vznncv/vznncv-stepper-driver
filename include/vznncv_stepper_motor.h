#ifndef APP_STEPPER_MOTOR_H
#define APP_STEPPER_MOTOR_H
#include "cmsis.h"
#include "mbed.h"
#include <chrono>

#include "mbed_atomic.h"

namespace vznncvsteppermotor {

//-----------------------------------------------------------------------------
// Common stepper driver code
//-----------------------------------------------------------------------------

/**
 * Base class to control stepper motors.
 *
 * It incorporates logic of step time calculation and provides convenient API to control stepper motor.
 * Subclasses must implement logic of the step direction and single step execution.
 */
class BaseStepperMotor : private NonCopyable<BaseStepperMotor> {
public:
    // step direction
    enum MoveDirection : int8_t {
        // move forward
        DIR_FORWARD = 1,
        // neutral position (no movement)
        DIR_NONE = 0,
        // move backward
        DIR_BACKWARD = -1
    };

    // stepper motor state
    enum State : int8_t {
        // motor is disabled
        STATE_DISABLED = 0,
        // motor is enabled
        STATE_ENABLED = 1
    };

private:
    // maximal acceleration, unit - steps/s^2
    float _max_acceleration;
    // maximal speed, unit - steps/s
    float _max_speed;

    // current speed, unit - steps/s
    float _current_speed;

    int32_t _current_pos;
    int32_t _target_pos;

    // note: Due calculation errors an algorithm stat deacceleration a little bit late.
    //       It can be fixed by adding the `log(steps_to_stop) / 4 + log(e)` to steps_to_stop
    //       variable in the `_execute_step_with_acceleration` function. But logarithm logarithm
    //       calculation isn't fast, so approximate it the linear function `steps_to_stop * _STS_COMP_A + _STS_COMP_B`
    static constexpr float _STS_COMP_A = 1.0f;
    static constexpr int _STS_COMP_B = 2;

    // helper values to speed up calculations
    union {
        // constant for movement with acceleration
        struct {
            // 1.0f / (2.0f * _max_acceleration)
            float one_div_double_acc;
            // minimal speed after one step
            float min_step_speed;
            // _max_acceleration / 1'000'000
            float max_acceleration_div_mil;
        } acc;
        // constat for "constant speed mode"
        struct {
            // delay between steps
            uint32_t step_interval_us;
        } csp;
    } _calc_const;

    void _update_helper_consts();

    // step error
    int _err;

    bool _initialized;
    MoveDirection _last_dir;
    State _state;

    /**
     * Helper class to run "step" inside interrupt.
     */
    class StepTicker : public TimerEvent {
    private:
        BaseStepperMotor *_base_stepper_motor;
        core_util_atomic_flag _schedule_flag;

    public:
        StepTicker(const ticker_data_t *data, BaseStepperMotor *base_stepper_motor);
        ~StepTicker();

        void trigger();
        void force_cancel();

    protected:
        virtual void handler() override final;
        void set_time_point(ticker_event_t &event, TickerDataClock::time_point time_point);
    };

    StepTicker _step_ticker;

    /**
     * Execute step.
     *
     * @return the number of microseconds after which the next step should be scheduled or 0 if no step should be scheduled
     */
    uint32_t _execute_step();

    uint32_t _execute_step_with_acceleration();
    uint32_t _execute_step_with_constant_speed();

public:
    static constexpr float DEFAULT_MAX_SPEED = 200.0f;
    static constexpr float DEFAULT_MAX_ACCELERATION = 100.0f;

    BaseStepperMotor(float max_speed = DEFAULT_MAX_SPEED, float max_acceleration = DEFAULT_MAX_ACCELERATION);

    /**
     * Stepper motor initialization function.
     *
     * It should be invoked once before any operations.
     *
     * @param start enable stepper motor after initialization immediaty
     * @return 0 on success, otherwise non-zero value
     */
    int init(bool start = true);

    virtual ~BaseStepperMotor();

    /**
     * Set maximal stepper motor speed.
     *
     * Speed unit: steps/s
     *
     * @param value
     */
    void set_max_speed(float value);

    /**
     * Get maximal stepper motor speed, that is set with
     *
     * @return
     */
    float get_max_speed() const;

    /**
     * Set maximal acceleration.
     *
     * If negative value is set, then constant speed will be used.
     *
     * Acceleration unit: steps/s^2
     *
     * @param value
     */
    void set_max_acceleration(float value);

    /**
     * Get maximal acceleration.
     *
     * @return
     */
    float get_max_acceleration();

    /**
     * Move to specified number of steps from current target position.
     *
     * @param rel_pos
     */
    void move(int rel_pos);

    /**
     * Set new target position.
     *
     * @param abs_pos
     */
    void move_to(int abs_pos);

    /**
     * Resume resume movement if it was stopped by some error before.
     *
     * @param 0 on success, otherwise non-zero value.
     */
    int resume_movement();

    /**
     * Wait end of the movement execution;
     *
     * @return
     */
    int wait_stopping();

    /**
     * Get current position.
     *
     * @return
     */
    int get_target_position() const;

    /**
     * Get target position.
     *
     * @return
     */
    int get_current_position() const;

    /**
     * Get distant from current to target position.
     *
     * Note: to prevent overflow a serial number arithmetic (https://en.wikipedia.org/wiki/Serial_number_arithmetic) is used.
     *
     * @return
     */
    int distance_to_go() const;

    /**
     * Enable/disabled stepper motor.
     *
     * It's recommended to call ::wait_stopping before stepper motor disabling,
     * as disabling causes immediaty stops any movement.
     *
     * After enabling, stepper motor will try to resume movement.
     *
     * @param state
     * @return 0 on success, otherwise non-zero value
     */
    int set_state(State state);

    /**
     * Get current stepper motor state.
     *
     * @return stepper motor state
     */
    State get_state() const;

    /**
     * Check if stepper driver is in error state.
     *
     * @return error value or 0 if there is no any error.
     */
    int get_error() const;

    /**
     * Clear error state.
     *
     * To resume movement after error cleanup call ::resume method.
     */
    void clear_error();

protected:
    /**
     * Optional initialization hook for subclasses.
     *
     * After initialization a stepper motor should be disabled, if enable/disable functionality is supported.
     *
     * @return 0 on success, otherwise non-zero value
     */
    virtual int init_impl();

    /**
     * Make single step.
     *
     * This method must be IRQ safe.
     *
     * @param step current step. It can be used to calculate correct phase
     * @return 0 on success, otherwise non-zero value
     */
    virtual int step_impl(int32_t step) = 0;

    /**
     * Set step direction.
     *
     * This method must be IRQ safe.
     *
     * @param dir step direction
     * @return 0 on success, otherwise non-zero value
     */
    virtual int set_direction_impl(MoveDirection dir) = 0;

    /**
     * Set stepper motor state implementation
     *
     * Currently only STATE_ENABLE and STATE_DISABLE should supported.
     *
     * @param state
     * @return  0 on success, otherwise non-zero value
     */
    virtual int set_state_impl(State state);
};

//-----------------------------------------------------------------------------
// StepDirDriverStepperMotor code
//-----------------------------------------------------------------------------

/**
 * Helper class to generate single short pulse on a some pin.
 *
 * Subclasses may use hardware acceleration for this operation.
 * It's enough to provide implementation up to 0xFFFF - 1 ns pulse width.
 */
class BaseSinglePulse : private NonCopyable<BaseSinglePulse> {
public:
    /**
     * Pulse polarity.
     */
    enum Polarity : uint8_t {
        // 1 - idle state
        // 0 - pulse state
        POLARITY_LOW = 0,
        // high polarity:
        // 0 - idle state
        // 1 - pulse state
        POLARITY_HIGH = 1,
    };

protected:
    // recommended default parameters, that can be used before ::set_timings invocation
    static constexpr std::chrono::nanoseconds DEFAULT_WIDTH_NS = 10'000ns;
    static constexpr Polarity DEFAULT_POLARITY = POLARITY_HIGH;

    // maximal delay
    static constexpr int MAX_WIDTH_NS = 0xFFFF - 1;

public:
    BaseSinglePulse() = default;
    virtual ~BaseSinglePulse() = default;

    /**
     * Configure pulse parameters.
     *
     * @param width_us pulse duration
     * @return 0 on success, otherwise non-zero value
     */
    virtual int set_timings(std::chrono::nanoseconds width_ns) = 0;

    /**
     * Set pulse polarity.
     *
     * @param polarity polarity type
     * @return 0 on success, otherwise non-zero value
     */
    virtual int set_polarity(Polarity polarity) = 0;

    /**
     * Generate pulse.
     *
     * The method should be interrupt safe.
     *
     * @return 0 on success, otherwise non-zero value
     */
    virtual int generate() = 0;
};

/**
 * Base blocking implementation of the BaseOnePulse class.
 */
class SimpleSinglePulse : public BaseSinglePulse {
protected:
    DigitalOut _out;
    int _width_ns;
    Polarity _polarity;

public:
    /**
     * Constructor.
     *
     * @param pin digital out pin
     * @param polarity
     */
    SimpleSinglePulse(PinName pin, Polarity polarity = POLARITY_HIGH);

    // BaseOnePulse interface
    int set_timings(std::chrono::nanoseconds width_ns) override;
    int set_polarity(Polarity polarity) override;
    int generate() override;
};

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
     */
    STM32SinglePulse(PinName pin, Polarity polarity = POLARITY_HIGH);
    ~STM32SinglePulse();

    // BaseOnePulse interface
    int set_timings(std::chrono::nanoseconds width_ns) override;
    int set_polarity(Polarity polarity) override;
    int generate() override;
};

#endif // TARGET_STM

/**
 * Stepper motor driver for DRV8825 or A4988 like chips.
 *
 * The hardware interface include the following pins:
 * - direction
 * - step. One pulse force to make one step
 * - enabled pin (optional), that can be used to enable/disable stepper motor
 */
class StepDirDriverStepperMotor : public BaseStepperMotor {
private:
    static constexpr uint32_t _FLAG_STEP_MASK = 0x1 << 0;
    static constexpr uint32_t _FLAG_DIR_MASK = 0x1 << 1;
    static constexpr uint32_t _FLAG_ENABLE_MASK = 0x1 << 2;

    static constexpr uint32_t _EXTRA_FLAG_CLEANUP_ONE_PULSE_MASK = 0x1 << 0;

    static inline SimpleSinglePulse::Polarity _get_polarity_from_flags(uint32_t flags)
    {
        return flags & _FLAG_STEP_MASK ? BaseSinglePulse::POLARITY_HIGH : BaseSinglePulse::POLARITY_LOW;
    }

public:
    // connection pin mode flags
    enum PinModeFlags : uint32_t {
        // normal step polarity: idle - 0, pulse - 1
        FLAG_STEP_NORMAL = 0,
        // inverted step polarity: idle - 1, pulse - 0
        FLAG_STEP_INVERTED = _FLAG_STEP_MASK,
        // normal direction polarity: forward - 1, backward - 0
        FLAG_DIR_NORMAL = 0,
        // inverted direction polarity: forward - 0, backward - 1
        FLAG_DIR_INVERTED = _FLAG_DIR_MASK,
        // normal enable pin polarity: enabled - 1, disabled - 1
        FLAG_ENABLE_NORMAL = 0,
        // inveted enable pin polarity: enabled - 0, disabled - 1
        FLAG_ENABLE_INVERTED = _FLAG_ENABLE_MASK,
        // default flag combination
        FLAG_DEFAULT = FLAG_STEP_NORMAL | FLAG_DIR_NORMAL | FLAG_ENABLE_NORMAL,
        // default flag combination for DRV8825
        FLAG_DEFAULF_DRV8825 = FLAG_STEP_NORMAL | FLAG_DIR_NORMAL | FLAG_ENABLE_INVERTED,
        // default flag combination for A4988
        FLAG_DEFAULF_A4988 = FLAG_STEP_NORMAL | FLAG_DIR_NORMAL | FLAG_ENABLE_INVERTED,
    };

    static constexpr std::chrono::nanoseconds DEFAULT_PULSE_LENTH_US = 2000ns;

private:
    enum StateFlags : uint8_t {
        _STATE_FLAG_CLEANUP_PULSE_GENERATOR_MASK = 0x1 << 0
    };

    BaseSinglePulse *_pulse_generator;
    DigitalOut _dir_pin;
    DigitalOut _en_pin;

    uint8_t _flags;
    uint8_t _state_flags;

public:
    /**
     * Constructor.
     *
     * @param step_pin step pulse pin
     * @param dir_pin direction
     * @param en_pin enable pin
     * @param flags pin flags. See ::PinModeFlags
     * @param pulse_lenth_us minimal pulse length
     */
    StepDirDriverStepperMotor(PinName step_pin, PinName dir_pin, PinName en_pin = NC, uint32_t flags = FLAG_DEFAULT, std::chrono::nanoseconds pulse_lenth_us = DEFAULT_PULSE_LENTH_US);

    /**
     * @param step_pin step pulse pin
     * @param dir_pin direction
     * @param en_pin enable pin
     * @param flags pin flags. See ::PinModeFlags
     * @param pulse_lenth_us minimal pulse length
     */
    StepDirDriverStepperMotor(BaseSinglePulse *pulse_generator, PinName dir_pin, PinName en_pin = NC, uint32_t flags = FLAG_DEFAULT, std::chrono::nanoseconds pulse_lenth_us = DEFAULT_PULSE_LENTH_US);
    virtual ~StepDirDriverStepperMotor() override;

protected:
    // BaseStepperMotor interface
    virtual int init_impl() override;
    virtual int step_impl(int32_t step_impl) override;
    virtual int set_direction_impl(MoveDirection dir) override;
    virtual int set_state_impl(State state) override;
};
}

#endif // APP_STEPPER_MOTOR_H
