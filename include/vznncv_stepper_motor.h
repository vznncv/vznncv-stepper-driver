#ifndef APP_STEPPER_MOTOR_H
#define APP_STEPPER_MOTOR_H
#include "cmsis.h"
#include "mbed.h"
#include <chrono>

#include "mbed_atomic.h"
#include "mbed_chrono.h"

namespace vznncvsteppermotor {

//-----------------------------------------------------------------------------
// Common stepper driver code
//-----------------------------------------------------------------------------

// helper chrono constants
using microseconds_u32 = mbed::chrono::microseconds_u32;
using milliseconds_u32 = mbed::chrono::milliseconds_u32;
using nanoseconds_u32 = std::chrono::duration<std::uint32_t, std::nano>;

/**
 * Base class to control stepper motors.
 *
 * It incorporates logic of step time calculation and provides convenient API to control stepper motor.
 * Subclasses must implement logic of the step direction and single step execution.
 */
class BaseStepperMotor : private NonCopyable<BaseStepperMotor> {
public:
    /**
     * Move direction.
     */
    enum MoveDirection : int8_t {
        /** Move forward */
        DIR_FORWARD = 1,
        /** neutral position (no movement) */
        DIR_NONE = 0,
        /** move backward */
        DIR_BACKWARD = -1
    };

    /**
     * Stepper motor state.
     */
    enum State : int8_t {
        /** motor is disabled */
        STATE_DISABLED = 0,
        /** motor is enabled */
        STATE_ENABLED = 1
    };

    /**
     * Current step instruction.
     */
    struct step_instruction_t {
        /**
         * Movement of current step
         */
        MoveDirection dir;
        /**
         * Delay before next step.
         *
         * Zero value indicates end of movement.
         */
        microseconds_u32 next;
    };

    /**
     * Description of current step for ::step_impl method.
     */
    struct step_description_t {
        /**
         * Movement direction of current step
         */
        MoveDirection dir;
        /**
         * Delay before next step.
         *
         * Zero value indicates end of movement.
         * It can be used to add extra hardware-accelerated sub steps to increase speed by ::step_impl.
         */
        microseconds_u32 next;

        /**
         * Current step number.
         *
         * It can be used to calculate correct phase for driver by ::step_impl.
         */
        int32_t step;
    };

    /**
     * Stepper motor position.
     *
     * Note: to handle overflow a serial number arithmetic (https://en.wikipedia.org/wiki/Serial_number_arithmetic) is used.
     */
    struct position_t {
        /** Current stepper motor position */
        int32_t current;
        /** Target stepper motor position */
        int32_t target;
    };

    enum MoveMode : int8_t {
        MODE_CONSTANT_SPEED = 0,
        MODE_CONSTANT_ACCELERATION = 1,
        MODE_CUSTOM_STEP = 2
    };

    typedef Callback<step_instruction_t(const position_t &pos)> CustomStepCallback;

private:
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

        /**
         * Get approximate delay before next event.
         *
         * @return return 0us if there is no event or positive delay interval
         */
        microseconds_u32 delay_before_event();

    protected:
        virtual void handler() override final;
        void set_time_point(ticker_event_t &event, TickerDataClock::time_point time_point);
    };

    int _force_step_cancel();

    StepTicker _step_ticker;

    /**
     * Execute step.
     *
     * @return the number of microseconds after which the next step should be scheduled or 0 if no step should be scheduled
     */
    microseconds_u32 _execute_step();

    step_instruction_t _execute_step_with_constant_speed();
    step_instruction_t _execute_step_with_constant_acceleration();
    step_instruction_t _execute_step_with_custom_step();

    // current step algorithm implementation
    step_instruction_t (BaseStepperMotor::*_execute_step_impl)();

    // step algorithm specific parameters
    union StepParams {
        struct ConstantSpeed {
            // maximal speed, unit - steps/s
            float max_speed;
            // delay between steps
            microseconds_u32 step_interval;

        } constant_speed;
        struct ConstantAcceleration {
            // maximal acceleration, unit - steps/s^2
            float max_acceleration;
            // maximal speed, unit - steps/s
            float max_speed;

            // 1.0f / (2.0f * max_acceleration)
            float one_div_double_acc;
            // minimal speed after one step
            float min_step_speed;
            // max_acceleration / 1'000'000
            float max_acceleration_div_mil;

            // current speed
            float current_speed;

            // note: Due calculation errors an algorithm stat deacceleration a little bit late.
            //       It can be fixed by adding the `log(steps_to_stop) / 4 + log(e)` to steps_to_stop
            //       variable in the `_execute_step_with_acceleration` function. But logarithm logarithm
            //       calculation isn't fast, so approximate it the linear function `steps_to_stop * _STS_COMP_A + _STS_COMP_B`
            static constexpr float STS_COMP_A = 1.01f;
            static constexpr int STS_COMP_B = 2;

        } constant_acceleration;
        struct CustomStep {
            // callback
            alignas(CustomStepCallback) uint8_t callback_memory[sizeof(CustomStepCallback)];
            // callback memory
            CustomStepCallback *callback;
        } custom_step;
    } _step_params;

    // current and target position
    position_t _position;

    // step error
    int _err;

    bool _initialized;
    MoveDirection _last_dir;
    State _state;

public:
    static constexpr float DEFAULT_MAX_SPEED = 200.0f;
    static constexpr float DEFAULT_MAX_ACCELERATION = 100.0f;

    /**
     * Constructor.
     *
     * By default it set mode with constant acceleration and ::DEFAULT_MAX_SPEED, ::DEFAULT_MAX_ACCELERATION parameters.
     *
     * @brief BaseStepperMotor
     */
    BaseStepperMotor();

    /**
     * Stepper motor initialization function.
     *
     * It should be invoked once before any operations.
     *
     * @param start enable stepper motor after initialization immediaty
     * @return 0 on success, otherwise non-zero value
     */
    int init(bool start = true);

    /**
     * Destructor.
     */
    virtual ~BaseStepperMotor();

private:
    void _cleanup_step_params();

public:
    /**
     * Set movement to mode with constant speed.
     *
     * @param max_speed maximal stepper motor speed. Unit - steps/s.
     * @return 0 on success, otherwise non-zero value
     */
    int set_mode_constant_speed(float max_speed);

    /**
     * Set movement to mode with constant acceleration.
     *
     * @param max_speed maximal stepper motor speed. Unit - steps/s
     * @param max_acceleration maximal stepper motor acceleration. Unit - steps/s^2
     * @return 0 on success, otherwise non-zero value
     */
    int set_mode_constant_acceleration(float max_speed, float max_acceleration);

    /**
     * Set movement to mode with custom step.
     *
     * Notes:
     * 1. custom step mode configuration doesn't startup move itself. You need to call ::resume_movement to start it.
     * 2. after end of movement, to resume it you should invoke ::resume_movement again
     * 3. callback is executed in the IRQ context and must be IRQ safe
     *
     * @param callback step function. It will be invoked once per step and should return information about current step and delay before next step.
     * @return 0 on success, otherwise non-zero value
     */
    int set_mode_custom_step(CustomStepCallback callback);

    /**
     * Get current mode.
     *
     * @return
     */
    MoveMode get_mode() const;

    /**
     * Get current parameters of constant speed mode.
     *
     * @param max_speed
     * @return 0 on success, otherwise non-zero value
     */
    int get_mode_params_constant_speed(float &max_speed);

    /**
     * Get current parameters of constant acceleration mode.
     *
     * @param max_speed
     * @param max_acceleration
     * @return 0 on success, otherwise non-zero value
     */
    int get_mode_params_constant_acceleration(float &max_speed, float &max_acceleration);

    /**
     * Get current parameters of custom step mode.
     *
     * @param callback
     * @return 0 on success, otherwise non-zero value
     */
    int get_mode_params_custom_step(CustomStepCallback &callback);

    /**
     * Move to specified number of steps from current target position.
     *
     * This action automatically resume movement if it was finished.
     *
     * @param rel_pos
     */
    void move(int rel_pos);

    /**
     * Set new target position.
     *
     * This action automatically resume movement if it was finished.
     *
     * @param abs_pos
     */
    void move_to(int abs_pos);

    /**
     * Stop any movement if exists.
     *
     * @return 0 on success, otherwise non-zero value.
     */
    int pause_movement();

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
    int wait_stop();

    /**
     * Get current position.
     *
     * @return
     */
    int get_target_position() const;

    /**
     * Set target position.
     *
     * This method works like ::move_to, but doesn't resume movement automatically.
     *
     * @param value
     */
    void set_target_position(int value);

    /**
     * Get target position.
     *
     * @return
     */
    int get_current_position() const;

    /**
     * Change current position forcely.
     *
     * This method can be used to reset driver state or change "zero" stepper motor position after calibration.
     *
     * @param value
     */
    void set_current_position(int value);

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
     * @param step current step description. Implementation can use step information if it's needed or ignore it.
     * @return 0 on success, otherwise non-zero value
     */
    virtual int step_impl(const step_description_t &step) = 0;

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

    // recommended default parameters, that can be used before ::set_timings invocation
    static constexpr nanoseconds_u32 DEFAULT_PULSE_WIDTH = 2'000ns;
    static constexpr Polarity DEFAULT_POLARITY = POLARITY_HIGH;

public:
    BaseSinglePulse() = default;
    virtual ~BaseSinglePulse() = default;

    /**
     * Configure pulse parameters.
     *
     * @param pulse_width pulse duration
     * @return 0 on success, otherwise non-zero value
     */
    virtual int set_timings(nanoseconds_u32 pulse_width) = 0;

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
    uint32_t _pulse_width_ns;
    Polarity _polarity;

public:
    /**
     * Constructor.
     *
     * @param pin digital out pin
     * @param pulse_width pulse width
     * @param polarity
     */
    SimpleSinglePulse(PinName pin, nanoseconds_u32 pulse_width = DEFAULT_PULSE_WIDTH, Polarity polarity = POLARITY_HIGH);

    // BaseOnePulse interface
    int set_timings(nanoseconds_u32 pulse_width) override;
    int set_polarity(Polarity polarity) override;
    int generate() override;
};

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
        // default flag combination for STSPIN820
        FLAG_DEFAULF_ST820 = FLAG_STEP_NORMAL | FLAG_DIR_NORMAL | FLAG_ENABLE_NORMAL,
    };

    static constexpr nanoseconds_u32 DEFAULT_PULSE_LENTH_US = BaseSinglePulse::DEFAULT_PULSE_WIDTH;

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
    StepDirDriverStepperMotor(PinName step_pin, PinName dir_pin, PinName en_pin = NC, uint32_t flags = FLAG_DEFAULT, nanoseconds_u32 pulse_lenth_us = DEFAULT_PULSE_LENTH_US);

    /**
     * @param step_pin step pulse pin
     * @param dir_pin direction
     * @param en_pin enable pin
     * @param flags pin flags. See ::PinModeFlags
     * @param pulse_lenth_us minimal pulse length
     */
    StepDirDriverStepperMotor(BaseSinglePulse *pulse_generator, PinName dir_pin, PinName en_pin = NC, uint32_t flags = FLAG_DEFAULT);
    virtual ~StepDirDriverStepperMotor() override;

protected:
    // BaseStepperMotor interface
    virtual int init_impl() override;
    virtual int step_impl(const step_description_t &step) override;
    virtual int set_direction_impl(MoveDirection dir) override;
    virtual int set_state_impl(State state) override;
};
}

#endif // APP_STEPPER_MOTOR_H
